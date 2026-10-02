//! nRF52840 Supermini rig firmware: two analog outputs and three analog inputs.
//!
//! **Acquisition.** Three single-ended SAADC channels are sampled by a hardware
//! timer through PPI into double-buffered EasyDMA, so the sample interval is
//! exact and the CPU only ever sees whole buffers. Each buffer is copied into a
//! block and handed to the USB task; the host reconstructs time from the block
//! sequence number and the sample rate, which is why nothing here timestamps
//! anything.
//!
//! **Generation.** The same board drives the plant's two inputs with a 62.5 kHz
//! PWM pair, smoothed into DC by the RC network in `ngspice_filter.cir`. Two of
//! the three SAADC channels read those filter outputs back, so what the plant
//! is fed and what the plant answers land on the same scan — no clock has to be
//! aligned, and the identification never has to trust a commanded value.
//!
//! **Link.** Native USB CDC-ACM on the nRF52840's own peripheral: this board has
//! neither a debug probe nor a USB-serial bridge, so one cable carries power,
//! flashing (through the UF2 bootloader) and the whole protocol.
//!
//! Wiring and the reasoning behind the front-end settings: `docs/HARDWARE.md`.
#![no_std]
#![no_main]

mod config;

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::{
    bind_interrupts,
    config::{HfclkSource, LfclkSource},
    gpio::Level,
    peripherals,
    pwm::{CounterMode, DutyCycle, SimpleConfig, SimplePwm},
    saadc::{self, CallbackResult, ChannelConfig, Saadc},
    timer::Frequency,
    usb::{self, vbus_detect::HardwareVbusDetect},
    Peri,
};
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, Mutex},
    channel::Channel,
};
use embassy_time::{Duration, Instant, Ticker};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
    driver::EndpointError,
    Builder, UsbDevice,
};
use static_cell::StaticCell;

use plant_trace_proto as proto;
use proto::{
    daq::{DaqError, DaqInfo, DaqToHost, HostToDaq, SampleBlock},
    frame,
    gen::{GenError, GenInfo, GenStatus, GenToHost, HostToGen, N_OUTPUTS},
    scale::DacScale,
    waveform::{Generator, Waveform},
};

use defmt_rtt as _;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

/// The USB driver, spelled once so the task signatures stay readable.
type UsbDriver = usb::Driver<'static, HardwareVbusDetect>;

/// Channels per frame, in the order `proto::Channel` defines.
const N_CH: usize = proto::N_CHANNELS;
/// Payload bytes in a full block.
const BLOCK_BYTES: usize = N_CH * config::BLOCK_SAMPLES * 2;

/// One acquisition buffer on its way to the host.
struct Block {
    seq: u32,
    n: u16,
    dropped: u32,
    data: [u8; BLOCK_BYTES],
}

/// Answers to host commands. Kept separate from the sample path so a slow
/// reply can never stall acquisition, and a flood of samples can never starve
/// a reply.
enum Reply {
    Pong,
    Info,
    Started(u32),
    Stopped { blocks: u32, dropped: u32 },
    Calibrated,
    Failed(DaqError),
    Gen(GenReply),
}

/// Answer to a generator command. `Info` carries nothing because the firmware
/// description is entirely static.
enum GenReply {
    Pong,
    Info,
    Ok,
    Status(GenStatus),
    Failed(GenError),
}

static BLOCKS: Channel<CriticalSectionRawMutex, Block, 3> = Channel::new();
static REPLIES: Channel<CriticalSectionRawMutex, Reply, 4> = Channel::new();
static COMMANDS: Channel<CriticalSectionRawMutex, HostToDaq, 4> = Channel::new();

/// The output half of the rig.
///
/// A blocking mutex rather than a channel: the receive task mutates it and the
/// waveform task reads it, both in a few microseconds, and a command has to
/// take effect on the very next tick. Nothing that can block is ever done while
/// holding it — writing the PWM duty happens after the lock is released.
static GEN: Mutex<CriticalSectionRawMutex, RefCell<Gen>> = Mutex::new(RefCell::new(Gen::new()));

/// Set by the receive task so a `Stop` can interrupt the sampling loop, which
/// owns the executor's main task while it runs.
static STOP: AtomicBool = AtomicBool::new(false);

/// One analog output.
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
    /// Duty written to the PWM, 0…255 against a 256-count top.
    code: u8,
}

impl Output {
    /// An output with the nominal map of the hardware behind it — for `u_T`
    /// that is the offset ladder, whose window starts at 2.25 V, not 0.
    const fn new(scale: DacScale) -> Self {
        Self {
            scale,
            staged: None,
            active: None,
            // Parked, so the plant sees a defined input from the first
            // millisecond after reset.
            held: scale.min_v,
            applied: scale.min_v,
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

/// Both outputs and the clock their waveforms share.
struct Gen {
    outputs: [Output; N_OUTPUTS],
    /// When the running waveforms were started, or `None` while idle.
    started_at: Option<Instant>,
}

impl Gen {
    const fn new() -> Self {
        Self {
            outputs: [
                Output::new(DacScale::NOMINAL_OUTPUTS[0]),
                Output::new(DacScale::NOMINAL_OUTPUTS[1]),
            ],
            started_at: None,
        }
    }

    /// Re-evaluate both outputs and return the duties to write.
    fn tick(&mut self, now: Instant) -> [u8; N_OUTPUTS] {
        let t_s = self
            .started_at
            .map(|t0| (now - t0).as_micros() as f32 / 1e6)
            .unwrap_or(0.0);

        let mut codes = [0u8; N_OUTPUTS];
        for (i, out) in self.outputs.iter_mut().enumerate() {
            codes[i] = out.evaluate(t_s);
        }

        // A finished waveform holds its last value: the plant stays where the
        // excitation left it instead of jumping back to the operating point.
        for out in self.outputs.iter_mut() {
            if let Some(gen) = &out.active {
                if gen.is_done(t_s) {
                    out.held = out.applied;
                    out.active = None;
                }
            }
        }
        if self.outputs.iter().all(|o| o.active.is_none()) {
            self.started_at = None;
        }
        codes
    }

    fn status(&self) -> GenStatus {
        GenStatus {
            t_ms: self
                .started_at
                .map(|t0| t0.elapsed().as_millis() as u32)
                .unwrap_or(0),
            volts: [self.outputs[0].applied, self.outputs[1].applied],
            codes: [self.outputs[0].code, self.outputs[1].code],
            running: self.started_at.is_some(),
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // The USB peripheral will not run off the internal oscillator — it needs
    // the 32 MHz crystal, and embassy's driver does not start it for us.
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.hfclk_source = HfclkSource::ExternalXtal;
    // The low-frequency clock times everything the generator does: the RTC1
    // time driver behind `Instant` and `Ticker` runs on it, so a sine's
    // frequency, a staircase's dwell and a PRBS bit are all in LFCLK seconds.
    // Left on the internal RC oscillator it is off by a few tenths of a
    // percent — the bench measured it as fitted amplitudes that fall with
    // frequency (half the sine gone at 100 Hz over a 2 s window) because the
    // host fits at the nominal frequency. Synthesising it from the crystal
    // above makes generator time crystal time, the same clock the sampling
    // timer runs on.
    nrf_config.lfclk_source = LfclkSource::Synthesized;
    let p = embassy_nrf::init(nrf_config);

    // ── host link ───────────────────────────────────────────────────────────
    let driver = usb::Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));

    let mut usb_config = embassy_usb::Config::new(config::USB_VID, config::USB_PID);
    usb_config.manufacturer = Some(config::USB_MANUFACTURER);
    usb_config.product = Some(config::USB_PRODUCT);
    usb_config.serial_number = Some(config::USB_SERIAL);
    usb_config.max_power = config::USB_MAX_POWER_MA;
    usb_config.max_packet_size_0 = config::USB_PACKET_BYTES as u8;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 0]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; 64]),
    );
    let class = CdcAcmClass::new(
        &mut builder,
        CDC_STATE.init(State::new()),
        config::USB_PACKET_BYTES as u16,
    );
    spawner.spawn(unwrap_task(run_usb(builder.build())));

    let (tx, rx) = class.split();
    spawner.spawn(unwrap_task(receive(rx)));
    spawner.spawn(unwrap_task(transmit(tx)));

    // ── analog outputs ──────────────────────────────────────────────────────
    let mut pwm_config = SimpleConfig::default();
    pwm_config.counter_mode = CounterMode::Up;
    pwm_config.max_duty = config::PWM_TOP;
    pwm_config.prescaler = config::PWM_PRESCALER;
    pwm_config.ch0_drive = config::PWM_DRIVE;
    pwm_config.ch1_drive = config::PWM_DRIVE;
    // Idle low is parked: if the PWM is ever disabled the plant sees 0 V, not
    // whatever the last duty happened to be.
    pwm_config.ch0_idle_level = Level::Low;
    pwm_config.ch1_idle_level = Level::Low;

    let mut pwm = SimplePwm::new_2ch(
        p.PWM0,
        p.P1_11, // D14 — u_T
        p.P1_13, // D15 — p_s
        &pwm_config,
    );
    // Both outputs at 0 V before the first tick, not whatever the peripheral
    // comes up with: the plant is connected from power-on.
    pwm.set_all_duties(duties([0; N_OUTPUTS]));
    spawner.spawn(unwrap_task(waveform(pwm)));

    // ── analog front end ────────────────────────────────────────────────────
    let mut saadc_config = saadc::Config::default();
    saadc_config.resolution = config::RESOLUTION;
    saadc_config.oversample = config::OVERSAMPLE;

    // The only analog-capable pins this board brings out, and exactly as many
    // as the rig needs.
    let channels = [
        channel_config(ChannelConfig::single_ended(p.P0_02)), // A0 / AIN0 — u_T
        channel_config(ChannelConfig::single_ended(p.P0_29)), // A1 / AIN5 — p_s
        channel_config(ChannelConfig::single_ended(p.P0_31)), // A2 / AIN7 — P_e
    ];
    let mut saadc: Saadc<'_, N_CH> = Saadc::new(p.SAADC, Irqs, saadc_config, channels);

    // The SAADC's offset drifts with temperature and supply; calibrating once
    // at boot is what makes a 0 V input read 0 counts instead of a handful.
    saadc.calibrate().await;
    info!(
        "nrf-daq up: {} ch, {} mV full scale, {}x oversample; {} outputs, {} Hz PWM",
        N_CH,
        config::FULL_SCALE_MV,
        config::OVERSAMPLE_FACTOR,
        N_OUTPUTS,
        config::PWM_HZ
    );

    let mut timer = p.TIMER1;
    let mut ppi_a = p.PPI_CH0;
    let mut ppi_b = p.PPI_CH1;

    loop {
        match COMMANDS.receive().await {
            HostToDaq::Ping => REPLIES.send(Reply::Pong).await,
            HostToDaq::Info => REPLIES.send(Reply::Info).await,
            HostToDaq::Stop => REPLIES.send(Reply::Failed(DaqError::NotRunning)).await,
            // Handled by the receive task, which is the only path that works
            // while `acquire` owns this loop.
            HostToDaq::Gen(_) => {}
            HostToDaq::Calibrate => {
                saadc.calibrate().await;
                REPLIES.send(Reply::Calibrated).await;
            }
            HostToDaq::Start {
                fs_hz,
                block_samples,
            } => {
                let Some(ticks) = timer_ticks(fs_hz) else {
                    REPLIES.send(Reply::Failed(DaqError::BadConfig)).await;
                    continue;
                };
                // The DMA buffers are sized at compile time, so the only
                // block size on offer is the one they hold. Accepting a
                // smaller one would mean discarding the rest of every buffer.
                if block_samples != 0 && block_samples as usize != config::BLOCK_SAMPLES {
                    REPLIES.send(Reply::Failed(DaqError::BadConfig)).await;
                    continue;
                }
                let effective_fs = 1_000_000 / ticks;
                REPLIES.send(Reply::Started(effective_fs)).await;

                let (blocks, dropped) = acquire(
                    &mut saadc,
                    timer.reborrow(),
                    ppi_a.reborrow(),
                    ppi_b.reborrow(),
                    ticks,
                )
                .await;

                info!("stopped after {} blocks, {} dropped", blocks, dropped);
                REPLIES.send(Reply::Stopped { blocks, dropped }).await;
            }
        }
    }
}

/// Apply the bench's front-end settings to a channel.
fn channel_config(mut cfg: ChannelConfig<'_>) -> ChannelConfig<'_> {
    cfg.reference = config::REFERENCE;
    cfg.gain = config::GAIN;
    cfg.time = config::ACQUISITION_TIME;
    cfg
}

/// Timer ticks per sample at 1 MHz, or `None` if the rate is out of range.
///
/// `fs_hz == 0` asks for the firmware's own default, so a host that does not
/// care does not have to hard-code a number that already lives here.
fn timer_ticks(fs_hz: u32) -> Option<u32> {
    let fs_hz = if fs_hz == 0 {
        config::DEFAULT_FS_HZ
    } else {
        fs_hz
    };
    if !(config::MIN_FS_HZ..=config::MAX_FS_HZ).contains(&fs_hz) {
        return None;
    }
    Some(1_000_000 / fs_hz)
}

/// Stream until the host sends `Stop`, returning `(blocks, dropped)`.
async fn acquire(
    saadc: &mut Saadc<'_, N_CH>,
    timer: Peri<'_, peripherals::TIMER1>,
    ppi_a: Peri<'_, peripherals::PPI_CH0>,
    ppi_b: Peri<'_, peripherals::PPI_CH1>,
    ticks: u32,
) -> (u32, u32) {
    STOP.store(false, Ordering::Relaxed);

    let mut bufs = [[[0i16; N_CH]; config::BLOCK_SAMPLES]; 2];
    let mut seq = 0u32;
    let mut dropped = 0u32;

    saadc
        .run_task_sampler(
            timer,
            ppi_a,
            ppi_b,
            Frequency::F1MHz,
            ticks,
            &mut bufs,
            |frames| {
                // Runs once per filled buffer, while the SAADC fills the other
                // one — so it must stay short: one memcpy and a non-blocking
                // send, never an await and never a UART write.
                let n = frames.len().min(config::BLOCK_SAMPLES);
                let mut block = Block {
                    seq,
                    n: n as u16,
                    dropped,
                    data: [0; BLOCK_BYTES],
                };
                for (i, frame) in frames[..n].iter().enumerate() {
                    for (c, sample) in frame.iter().enumerate() {
                        let off = (i * N_CH + c) * 2;
                        block.data[off..off + 2].copy_from_slice(&sample.to_le_bytes());
                    }
                }
                seq = seq.wrapping_add(1);
                if BLOCKS.try_send(block).is_err() {
                    // The link is behind. Dropping a whole block keeps every
                    // block that does arrive internally consistent, and the
                    // host sees the gap in `seq`.
                    dropped = dropped.wrapping_add(1);
                }

                if STOP.load(Ordering::Relaxed) {
                    CallbackResult::Stop
                } else {
                    CallbackResult::Continue
                }
            },
        )
        .await;

    (seq, dropped)
}

/// Re-evaluate both waveforms and push the new duties to the PWM.
///
/// A task of its own, because acquisition owns the main task for the whole
/// length of a recording and an excitation has to start *during* one.
#[embassy_executor::task]
async fn waveform(mut pwm: SimplePwm<'static>) {
    let mut ticker = Ticker::every(Duration::from_hz(config::TICK_HZ));
    let mut last = [u8::MAX; N_OUTPUTS];

    loop {
        ticker.next().await;
        let now = Instant::now();
        let codes = GEN.lock(|g| g.borrow_mut().tick(now));
        if codes == last {
            continue;
        }
        last = codes;
        // One DMA transfer for both channels: a two-channel excitation has no
        // skew between its channels, not even a PWM period's worth.
        pwm.set_all_duties(duties(codes));
    }
}

/// The compare values for a pair of output codes.
///
/// `DutyCycle::inverted` is the one that means what a code means here. In
/// embassy-nrf, `normal(v)` holds the pin *high* while the counter is at or
/// above `v`, so code 0 would be full scale and 255 almost nothing;
/// `inverted(v)` holds it high while the counter is below `v`, which makes the
/// mean `v / 256` of VDD. The bench caught this: with `normal`, commanding one
/// output up drove its filter down.
fn duties(codes: [u8; N_OUTPUTS]) -> [DutyCycle; 4] {
    [
        DutyCycle::inverted(codes[0] as u16),
        DutyCycle::inverted(codes[1] as u16),
        DutyCycle::inverted(0),
        DutyCycle::inverted(0),
    ]
}

/// Apply one generator command and produce its reply.
fn handle_gen(cmd: HostToGen) -> GenReply {
    GEN.lock(|cell| {
        let mut g = cell.borrow_mut();
        match cmd {
            HostToGen::Ping => GenReply::Pong,
            HostToGen::Info => GenReply::Info,
            HostToGen::Status => GenReply::Status(g.status()),

            HostToGen::SetScale { ch, scale } => match g.outputs.get_mut(ch as usize) {
                Some(out) => {
                    out.scale = scale;
                    GenReply::Ok
                }
                None => GenReply::Failed(GenError::BadChannel),
            },

            HostToGen::SetLevel { ch, volts } => match g.outputs.get_mut(ch as usize) {
                Some(out) => {
                    out.held = out.scale.clamp(volts);
                    out.active = None;
                    out.staged = None;
                    GenReply::Ok
                }
                None => GenReply::Failed(GenError::BadChannel),
            },

            HostToGen::Program { ch, wave } => match g.outputs.get_mut(ch as usize) {
                Some(out) => {
                    // Refuse an excitation that would leave the safe window,
                    // before any of it reaches the plant — clamping it silently
                    // would distort the very waveform the identification
                    // assumes was applied.
                    let (lo, hi) = wave.span();
                    if lo < out.scale.min_v - 1e-6 || hi > out.scale.max_v + 1e-6 {
                        GenReply::Failed(GenError::OutOfRange)
                    } else {
                        out.staged = Some(wave);
                        GenReply::Ok
                    }
                }
                None => GenReply::Failed(GenError::BadChannel),
            },

            HostToGen::Start => {
                for out in g.outputs.iter_mut() {
                    if let Some(wave) = out.staged.take() {
                        out.active = Some(Generator::new(wave));
                    }
                }
                // One timestamp for both channels: a two-channel excitation
                // starts on the same tick, with no skew between them.
                g.started_at = Some(Instant::now());
                GenReply::Ok
            }

            HostToGen::Stop => {
                for out in g.outputs.iter_mut() {
                    out.held = out.applied;
                    out.active = None;
                }
                g.started_at = None;
                GenReply::Ok
            }

            HostToGen::Park => {
                for out in g.outputs.iter_mut() {
                    out.held = out.scale.min_v;
                    out.active = None;
                    out.staged = None;
                }
                g.started_at = None;
                GenReply::Ok
            }
        }
    })
}

/// Drive the USB device. Nothing else touches it; the CDC endpoints are used
/// from the two tasks below.
#[embassy_executor::task]
async fn run_usb(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

/// Decode host commands off the CDC endpoint.
#[embassy_executor::task]
async fn receive(mut rx: Receiver<'static, UsbDriver>) {
    let mut decoder = frame::Decoder::<{ frame::MAX_FRAME }>::new();
    let mut buf = [0u8; config::USB_PACKET_BYTES];
    loop {
        rx.wait_connection().await;
        info!("host connected");
        loop {
            let Ok(n) = rx.read_packet(&mut buf).await else {
                // Only ever `Disabled`: the cable came out, or the host
                // suspended us. Wait for it to come back rather than spinning.
                break;
            };
            for byte in &buf[..n] {
                if !decoder.push(*byte) {
                    continue;
                }
                let Some(raw) = decoder.frame() else {
                    continue;
                };
                match frame::decode::<HostToDaq>(raw) {
                    // Generator commands are answered here rather than queued:
                    // the main task is inside `acquire` for the whole length of
                    // a recording, and that is exactly when an excitation
                    // starts.
                    Ok(HostToDaq::Gen(cmd)) => REPLIES.send(Reply::Gen(handle_gen(cmd))).await,
                    Ok(cmd) => {
                        // `Stop` has to reach the sampling loop, which is not
                        // reading the command queue while it runs.
                        if matches!(cmd, HostToDaq::Stop) {
                            STOP.store(true, Ordering::Relaxed);
                        }
                        if COMMANDS.try_send(cmd).is_err() {
                            warn!("command queue full, dropping");
                        }
                    }
                    Err(e) => warn!("bad frame: {:?}", defmt::Debug2Format(&e)),
                }
            }
        }
        warn!("host disconnected");
    }
}

/// Write one frame as a sequence of bulk packets.
///
/// A frame that happens to be a whole number of packets long is terminated with
/// a zero-length one, so a host that waits for a short packet to end the
/// transfer is not left holding the last block until the next one arrives.
async fn write_frame(
    tx: &mut Sender<'static, UsbDriver>,
    data: &[u8],
) -> Result<(), EndpointError> {
    for chunk in data.chunks(config::USB_PACKET_BYTES) {
        tx.write_packet(chunk).await?;
    }
    if data.len().is_multiple_of(config::USB_PACKET_BYTES) {
        tx.write_packet(&[]).await?;
    }
    Ok(())
}

/// Serialise blocks and replies onto the CDC endpoint.
///
/// `write_frame` blocks while the host is not draining the endpoint, which is
/// the backpressure the rig wants: the block channel fills, the acquisition
/// callback's `try_send` starts failing, and the host sees the gap in `seq`
/// and in `dropped` instead of receiving a stream that quietly lost its
/// alignment.
#[embassy_executor::task]
async fn transmit(mut tx: Sender<'static, UsbDriver>) {
    let mut scratch = [0u8; frame::MAX_FRAME];
    let mut out = [0u8; frame::MAX_FRAME];

    tx.wait_connection().await;
    loop {
        let msg_len = match select(BLOCKS.receive(), REPLIES.receive()).await {
            Either::First(block) => {
                let used = block.n as usize * N_CH * 2;
                let msg = DaqToHost::Block(SampleBlock {
                    seq: block.seq,
                    n: block.n,
                    channels: N_CH as u8,
                    dropped: block.dropped,
                    data: &block.data[..used],
                });
                frame::encode(&msg, &mut scratch, &mut out)
            }
            Either::Second(reply) => {
                let msg = match reply {
                    Reply::Pong => DaqToHost::Pong,
                    Reply::Info => DaqToHost::Info(DaqInfo {
                        protocol: proto::PROTOCOL_VERSION,
                        firmware: config::FIRMWARE_VERSION,
                        channels: N_CH as u8,
                        bits: 12,
                        full_scale_mv: config::FULL_SCALE_MV,
                        oversample: config::OVERSAMPLE_FACTOR,
                        block_samples: config::BLOCK_SAMPLES as u16,
                        max_fs_hz: config::MAX_FS_HZ,
                    }),
                    Reply::Started(fs_hz) => DaqToHost::Started { fs_hz },
                    Reply::Stopped { blocks, dropped } => DaqToHost::Stopped { blocks, dropped },
                    Reply::Calibrated => DaqToHost::Calibrated,
                    Reply::Failed(e) => DaqToHost::Error(e),
                    Reply::Gen(r) => DaqToHost::Gen(match r {
                        GenReply::Pong => GenToHost::Pong,
                        GenReply::Info => GenToHost::Info(GenInfo {
                            protocol: proto::PROTOCOL_VERSION,
                            firmware: config::FIRMWARE_VERSION,
                            outputs: N_OUTPUTS as u8,
                            bits: 8,
                            tick_hz: config::TICK_HZ as u32,
                        }),
                        GenReply::Ok => GenToHost::Ok,
                        GenReply::Status(s) => GenToHost::Status(s),
                        GenReply::Failed(e) => GenToHost::Error(e),
                    }),
                };
                frame::encode(&msg, &mut scratch, &mut out)
            }
        };

        match msg_len {
            Ok(len) => {
                if write_frame(&mut tx, &out[..len]).await.is_err() {
                    // The host went away mid-frame. Whatever is queued is stale
                    // by the time it comes back, but the decoder resynchronises
                    // on the next delimiter either way.
                    warn!("usb tx failed; waiting for the host");
                    tx.wait_connection().await;
                }
            }
            Err(e) => warn!("encode failed: {:?}", defmt::Debug2Format(&e)),
        }
    }
}

/// Turn a task token into a spawnable one, panicking if the task pool is
/// exhausted — which can only happen if a task is spawned twice, i.e. a bug
/// rather than a runtime condition.
fn unwrap_task<S>(
    token: Result<embassy_executor::SpawnToken<S>, embassy_executor::SpawnError>,
) -> embassy_executor::SpawnToken<S> {
    match token {
        Ok(t) => t,
        Err(_) => defmt::panic!("task pool exhausted"),
    }
}
