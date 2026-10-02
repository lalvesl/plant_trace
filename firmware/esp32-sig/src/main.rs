//! ESP32-WROOM-32 signal generator.
//!
//! Drives the plant's two inputs — `u_T` on DAC1/GPIO25 and `p_s` on
//! DAC2/GPIO26 — from waveforms staged by the host. A 1 kHz tick evaluates
//! [`plant_trace_proto::waveform`], the same code the host uses to replay the
//! excitation during identification.
//!
//! Division of labour with the host: the device owns *timing*, because that is
//! a property of this hardware tick and not of the USB link; the host owns
//! *calibration*, because the volts↔code map is a property of the divider on
//! the bench and not of this firmware. Levels on the wire are therefore always
//! volts at the plant input.
//!
//! There is no logging. The command link is UART0, which is the only serial
//! port the devkit exposes over USB, so anything printed would land in the
//! middle of a frame. Panics are the exception: a panic message is garbage to
//! the host's decoder, which resynchronises at the next delimiter — and by
//! then the firmware is dead anyway. The ROM bootloader's own banner at reset
//! is absorbed the same way.
#![no_std]
#![no_main]

mod config;

use esp_backtrace as _;
use esp_hal::{
    analog::dac::Dac,
    main,
    time::{Duration, Instant},
    uart::{Config as UartConfig, Uart},
};

use plant_trace_proto as proto;
use proto::{
    frame,
    gen::{GenError, GenInfo, GenStatus, GenToHost, HostToGen, N_OUTPUTS},
    scale::DacScale,
    waveform::{Generator, Waveform},
};

esp_bootloader_esp_idf::esp_app_desc!();

/// One output channel.
struct Output {
    /// Bench-measured volts↔code map and safe window.
    scale: DacScale,
    /// Waveform waiting for `Start`.
    staged: Option<Waveform>,
    /// Waveform being evaluated.
    active: Option<Generator>,
    /// Level held when no waveform is running, volts.
    held: f32,
    /// Level actually applied after clamping and quantisation, volts.
    applied: f32,
    /// Code written to the DAC.
    code: u8,
}

impl Output {
    const fn new() -> Self {
        Self {
            scale: config::DEFAULT_SCALE,
            staged: None,
            active: None,
            held: 0.0,
            applied: 0.0,
            code: 0,
        }
    }

    /// Level this output should hold at `t` seconds into the run.
    fn evaluate(&mut self, t_s: f32) -> u8 {
        let want = match &mut self.active {
            Some(gen) => gen.sample(t_s),
            None => self.held,
        };
        self.code = self.scale.to_code(want);
        self.applied = self.scale.to_volts(self.code);
        self.code
    }
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut uart = Uart::new(
        peripherals.UART0,
        UartConfig::default().with_baudrate(config::BAUDRATE),
    )
    .expect("UART0")
    .with_rx(peripherals.GPIO3)
    .with_tx(peripherals.GPIO1);

    let mut dac_u_t = Dac::new(peripherals.DAC1, peripherals.GPIO25);
    let mut dac_p_s = Dac::new(peripherals.DAC2, peripherals.GPIO26);

    let mut outputs = [Output::new(), Output::new()];
    let mut started_at: Option<Instant> = None;

    // Park both outputs before anything else: the plant sees a defined input
    // from the first millisecond after reset.
    for out in outputs.iter_mut() {
        out.held = out.scale.min_v;
    }
    dac_u_t.write(outputs[0].evaluate(0.0));
    dac_p_s.write(outputs[1].evaluate(0.0));

    let tick = Duration::from_micros(1_000_000 / config::TICK_HZ);
    let mut next_tick = Instant::now() + tick;

    let mut decoder = frame::Decoder::<{ frame::MAX_FRAME }>::new();
    let mut scratch = [0u8; frame::MAX_FRAME];
    let mut out_buf = [0u8; frame::MAX_FRAME];
    let mut rx = [0u8; 64];

    loop {
        // ── commands ────────────────────────────────────────────────────────
        if let Ok(n) = uart.read_buffered(&mut rx) {
            for byte in &rx[..n] {
                if !decoder.push(*byte) {
                    continue;
                }
                let Some(raw) = decoder.frame() else { continue };
                let Ok(cmd) = frame::decode::<HostToGen>(raw) else {
                    // A corrupted frame is dropped silently: the host will
                    // notice the missing reply, and answering "what?" to noise
                    // only adds noise.
                    continue;
                };
                let reply = handle(cmd, &mut outputs, &mut started_at);
                if let Ok(len) = frame::encode(&reply, &mut scratch, &mut out_buf) {
                    let _ = uart.write(&out_buf[..len]);
                    let _ = uart.flush();
                }
            }
        }

        // ── waveform tick ───────────────────────────────────────────────────
        let now = Instant::now();
        if now < next_tick {
            continue;
        }
        next_tick += tick;
        if now >= next_tick {
            // Fell behind — skip the backlog rather than burst-writing it, so
            // a stall shows up as a missing sample and not as a time shift.
            next_tick = now + tick;
        }

        let t_s = started_at
            .map(|t0| t0.elapsed().as_micros() as f32 / 1e6)
            .unwrap_or(0.0);

        dac_u_t.write(outputs[0].evaluate(t_s));
        dac_p_s.write(outputs[1].evaluate(t_s));

        // A finished waveform holds its last value: the plant stays where the
        // excitation left it instead of jumping back to the operating point.
        for out in outputs.iter_mut() {
            if let Some(gen) = &out.active {
                if gen.is_done(t_s) {
                    out.held = out.applied;
                    out.active = None;
                }
            }
        }
        if outputs.iter().all(|o| o.active.is_none()) {
            started_at = None;
        }
    }
}

/// Apply one command and produce its reply.
fn handle(
    cmd: HostToGen,
    outputs: &mut [Output; N_OUTPUTS],
    started_at: &mut Option<Instant>,
) -> GenToHost<'static> {
    match cmd {
        HostToGen::Ping => GenToHost::Pong,

        HostToGen::Info => GenToHost::Info(GenInfo {
            protocol: proto::PROTOCOL_VERSION,
            firmware: config::FIRMWARE_VERSION,
            outputs: N_OUTPUTS as u8,
            bits: config::DAC_BITS,
            tick_hz: config::TICK_HZ as u32,
        }),

        HostToGen::Status => GenToHost::Status(GenStatus {
            t_ms: started_at
                .map(|t0| t0.elapsed().as_millis() as u32)
                .unwrap_or(0),
            volts: [outputs[0].applied, outputs[1].applied],
            codes: [outputs[0].code, outputs[1].code],
            running: started_at.is_some(),
        }),

        HostToGen::SetScale { ch, scale } => match outputs.get_mut(ch as usize) {
            Some(out) => {
                out.scale = scale;
                GenToHost::Ok
            }
            None => GenToHost::Error(GenError::BadChannel),
        },

        HostToGen::SetLevel { ch, volts } => match outputs.get_mut(ch as usize) {
            Some(out) => {
                out.held = out.scale.clamp(volts);
                out.active = None;
                out.staged = None;
                GenToHost::Ok
            }
            None => GenToHost::Error(GenError::BadChannel),
        },

        HostToGen::Program { ch, wave } => match outputs.get_mut(ch as usize) {
            Some(out) => {
                // Refuse an excitation that would leave the safe window, before
                // any of it reaches the plant — clamping it silently would
                // distort the very waveform the identification assumes.
                let (lo, hi) = wave.span();
                if lo < out.scale.min_v - 1e-6 || hi > out.scale.max_v + 1e-6 {
                    GenToHost::Error(GenError::OutOfRange)
                } else {
                    out.staged = Some(wave);
                    GenToHost::Ok
                }
            }
            None => GenToHost::Error(GenError::BadChannel),
        },

        HostToGen::Start => {
            for out in outputs.iter_mut() {
                if let Some(wave) = out.staged.take() {
                    out.active = Some(Generator::new(wave));
                }
            }
            // One timestamp for both channels: a two-channel excitation starts
            // on the same tick, with no skew between them.
            *started_at = Some(Instant::now());
            GenToHost::Ok
        }

        HostToGen::Stop => {
            for out in outputs.iter_mut() {
                out.held = out.applied;
                out.active = None;
            }
            *started_at = None;
            GenToHost::Ok
        }

        HostToGen::Park => {
            for out in outputs.iter_mut() {
                out.held = out.scale.min_v;
                out.active = None;
                out.staged = None;
            }
            *started_at = None;
            GenToHost::Ok
        }
    }
}
