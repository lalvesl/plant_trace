//! A simulated rig, backed by [`plant_model`].
//!
//! It exists so the acquisition path, the experiment runner and the
//! identification code can be developed and tested without the bench. It speaks
//! the real protocol over TCP — one connection carrying both halves, exactly
//! like the nRF's USB link — so the only thing the CLI does differently against
//! the simulator is the link spec it opens.
//!
//! What is simulated faithfully: the 8-bit quantisation of the PWM duty, the
//! two real poles of the output filter, the ADC's counts and saturation, the
//! sample rate, the block structure, the plant's dynamics, and the SAADC's
//! channel-to-channel scan skew ([`Options::scan_spacing_s`]). What is not:
//! serial timing, dropped blocks, electrical noise pickup, and the carrier
//! ripple — which SPICE puts at about 2 mV peak-to-peak at the plant, and which
//! the SAADC's burst averaging takes back down to roughly one count.

use std::{
    io::Write,
    net::{SocketAddr, TcpListener, TcpStream},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use anyhow::{Context, Result};
use plant_model::{Plant, PlantParams};
use plant_trace_proto::{
    daq::{DaqError, DaqInfo, DaqToHost, HostToDaq, SampleBlock},
    frame,
    gen::{GenError, GenInfo, GenStatus, GenToHost, HostToGen, N_OUTPUTS},
    scale::{AdcScale, DacScale},
    waveform::Generator,
    N_CHANNELS, SCAN_CHANNEL_SPACING_S,
};

/// Samples per channel in a simulated block — the same as the firmware's.
const BLOCK_SAMPLES: usize = 64;
/// Firmware string the simulator reports, so a CSV recorded against it is
/// never mistaken for one from the bench.
const FIRMWARE: &str = "sim 0.1.0";
/// Rate at which the simulated firmware says it re-evaluates its waveforms —
/// the nominal figure the firmware reports.
const TICK_HZ: u32 = 1000;
/// Period the waveform tick actually runs at, seconds.
///
/// The firmware asks for `Ticker::every(Duration::from_hz(1000))`, but its
/// clock is the RTC1 time driver at 32 768 Hz and `from_hz` rounds to whole
/// ticks of it: 33 of them, 1.00708 ms, 992.97 Hz. The sample clock is TIMER1
/// off the crystal, so the two are **not synchronous** — and that matters to
/// a simulation more than the 0.7 %: with a tick that divides the sample rate
/// exactly, the tick's images alias coherently onto the excitation frequency
/// and bias every fitted phase by a fixed, offset-dependent amount the bench
/// never shows. At the real period they land beside it and the fit rejects
/// them, as on the bench.
const TICK_PERIOD_S: f64 = 33.0 / 32_768.0;
/// Rates the simulated firmware accepts, matching `firmware/nrf-daq`.
const MIN_FS_HZ: u32 = 10;
/// See [`MIN_FS_HZ`].
const MAX_FS_HZ: u32 = 2000;

/// How long a reply or a block may wait for the host to make room for it
/// before the session is given up. Generous, because a host that stalls for a
/// moment is normal; finite, because a simulator thread stuck forever in a
/// write can never be shut down.
const WRITE_TIMEOUT: Duration = Duration::from_secs(10);
/// Most samples per channel the simulator produces per second of wall clock,
/// whatever [`Options::speed`] asks for.
///
/// Not a performance limit — a fidelity one. The real rig cannot get ahead of
/// the host: it runs in real time and drops blocks rather than queue them. A
/// simulator running faster than the host consumes piles blocks up in the
/// socket, and then a command lands in the stream seconds (of sample time)
/// after the host thinks it did — a step recorded after its recording ended.
/// A debug-build host keeps up with this rate comfortably.
const MAX_SAMPLES_PER_WALL_S: f32 = 20_000.0;

/// What is connected to the `P_e` input of the simulated rig.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SimPlant {
    /// The steam-turbine model of [`plant_model`] — the plant the assignment
    /// is about.
    #[default]
    Model,
    /// A plain wire from the filter output of one analog output to the `P_e`
    /// input, which is what the bench looks like with the plant unplugged and
    /// a jumper in its place.
    ///
    /// The true transfer is then exactly 1 — 0 dB and 0° at every frequency —
    /// so anything a measurement makes of it other than that is the
    /// measurement: in practice, the scan skew between the channels. That is
    /// what makes it the end-to-end check of [`crate::bode`].
    Wire {
        /// Output feeding the `P_e` input: 0 = `u_T`, 1 = `p_s`.
        from: u8,
    },
}

/// How to run the simulator.
#[derive(Debug, Clone)]
pub struct Options {
    /// Address the rig listens on. Ignored by [`spawn`], which always binds an
    /// ephemeral port on the loopback interface.
    pub addr: String,
    /// Sample rate offered to the host.
    pub fs_hz: u32,
    /// Wall-clock speed-up: 1.0 is real time, 20.0 runs a 60 s experiment in
    /// three seconds. Sample timestamps are unaffected. Capped so that no more
    /// than 20 000 samples per channel are produced per wall-clock second (10×
    /// at 2 kHz, 20× at 1 kHz), which keeps the simulator from running ahead
    /// of the host.
    pub speed: f32,
    /// Plant parameters.
    pub params: PlantParams,
    /// ADC scaling used to turn volts into counts.
    pub adc: AdcScale,
    /// Output scaling applied to the generator outputs, in wire order — the
    /// maps the firmware boots with unless a test says otherwise.
    pub dac: [DacScale; N_OUTPUTS],
    /// Simulated seconds of settling applied before the first block, so a
    /// recording starts from steady state like the real plant would.
    pub settle_s: f32,
    /// What the `P_e` input is connected to.
    pub plant: SimPlant,
    /// Time between the conversions of consecutive channels inside one scan,
    /// seconds; channel `k` of a row is the signal at `t + k × spacing`, as on
    /// the real SAADC. Defaults to
    /// [`plant_trace_proto::SCAN_CHANNEL_SPACING_S`]; 0 makes the three
    /// channels of a row simultaneous, which no real rig is.
    ///
    /// How it is modelled: the command is constant between two samples in
    /// this simulator (ticks land on sample instants), so the output filters
    /// are advanced to the later instant *exactly*, with their own
    /// discretisation — the sense channels and a [`SimPlant::Wire`] carry no
    /// approximation at all. The plant model is interpolated linearly between
    /// its two neighbouring steps instead; its fastest dynamics are a few
    /// hertz, where the error of doing so at a 1 kHz step is far below one
    /// ADC count. Offsets longer than one sample period (only possible above
    /// ~5 kHz, which the rig does not offer) are clamped to it.
    pub scan_spacing_s: f64,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            addr: "127.0.0.1:7801".into(),
            fs_hz: 1000,
            speed: 1.0,
            params: PlantParams::default(),
            adc: AdcScale::NOMINAL,
            dac: DacScale::NOMINAL_OUTPUTS,
            settle_s: 120.0,
            plant: SimPlant::Model,
            scan_spacing_s: SCAN_CHANNEL_SPACING_S,
        }
    }
}

/// The RC network between a PWM pin and the plant input.
///
/// `ngspice_filter.cir` is a two-section ladder built entirely from 1 kΩ and
/// 100 nF, and its two poles are real: 796 Hz and 4775 Hz. Nothing loads it —
/// the plant input and the 10 kΩ into the sense pin are both high impedance.
/// Modelling it is not decoration: every commanded edge reaches the plant
/// through it, 90 % of the way in 0.5 ms and to within one count in 1.5 ms.
///
/// Poles of the reconstruction filter between a PWM output and the plant, in
/// hertz, for the unloaded ladder (`R1 = 1 k + 1 k`, `R2 = R3 = 1 k`,
/// `C1 = C2 = 100 n`), as ngspice reports them. Together they put the corner
/// at 773 Hz.
///
/// The bench checks this with `plant-trace bode experiments/bode-wire.toml`:
/// the excitation amplitude it fits against frequency *is* this response. It
/// is how an 8.2 kΩ-for-820 Ω mix-up was found — poles fitted at exactly a
/// tenth of these, rms error 0.03 mV.
///
/// Public because `check sine` compares a bench measurement against them:
/// agreement says the board was built the way the simulation says it was.
pub const OUTPUT_FILTER_POLES_HZ: [f64; 2] = [795.8, 4774.6];

/// It is unity-gain here because the volts↔code map has already converted a
/// duty into volts at the plant input.
struct OutputFilter {
    y: [f32; 2],
}

impl OutputFilter {
    /// Time constants of [`OUTPUT_FILTER_POLES_HZ`].
    const TAU_S: [f32; 2] = [
        1.0 / (core::f32::consts::TAU * OUTPUT_FILTER_POLES_HZ[0] as f32),
        1.0 / (core::f32::consts::TAU * OUTPUT_FILTER_POLES_HZ[1] as f32),
    ];

    fn new(v: f32) -> Self {
        Self { y: [v; 2] }
    }

    /// Voltage at the plant input right now.
    fn output(&self) -> f32 {
        self.y[1]
    }

    /// Advance by `dt` with the input held constant over it — which is exactly
    /// what a PWM duty does between two ticks.
    ///
    /// Exact for the *cascade*, not just for each section: the second section
    /// is driven by the first one's exponential approach, not by its end
    /// value held over the whole step. Discretising the sections one at a
    /// time (each exact on its own, which is what this used to do) makes the
    /// pair faster than it is, by an amount that depends on `dt` — invisible
    /// on a level, but a spurious fraction of a degree between two channels
    /// read at different instants, which is exactly what the scan skew is
    /// about. With a held input `x`, `a_i = e^(-dt/τ_i)`:
    ///
    /// ```text
    /// y1' = x + (y1 − x)·a1
    /// y2' = x + (y2 − x)·a2 + (y1 − x)·τ1/(τ1 − τ2)·(a1 − a2)
    /// ```
    ///
    /// No Euler step anywhere: at 1 kHz, `dt/τ` for the fast section is 37,
    /// and Euler would diverge instead of simply collapsing onto the input.
    fn step(&mut self, dt: f32, x: f32) -> f32 {
        let [t1, t2] = Self::TAU_S;
        let [y1, y2] = self.y;
        let a1 = (-dt / t1).exp();
        let a2 = (-dt / t2).exp();
        self.y = [
            x + (y1 - x) * a1,
            x + (y2 - x) * a2 + (y1 - x) * t1 / (t1 - t2) * (a1 - a2),
        ];
        self.y[1]
    }
}

/// Both outputs from the waveform tick to the plant input, advanced event by
/// event through continuous time.
///
/// Ticks fall at their own instants ([`TICK_PERIOD_S`]), not on the sample
/// grid, and a channel is read at the instant the SAADC converts it — so the
/// staircase, the filter and the scan skew are all where they would be on the
/// bench, and every one of those instants is exact: between two events the
/// input is constant and [`OutputFilter::step`] is exact for a constant input.
struct Chain {
    filters: [OutputFilter; N_OUTPUTS],
    /// Level each output was last told to take by a tick, volts.
    commanded: [f32; N_OUTPUTS],
    /// Simulation time the filters are at.
    t: f64,
    /// Time of tick 0; ticks are counted, not accumulated, so they do not drift.
    tick_t0: f64,
    /// Next tick to apply.
    tick_k: u64,
}

impl Chain {
    /// A chain at rest at `levels`, with its first tick due at `t`.
    fn new(t: f64, levels: [f32; N_OUTPUTS]) -> Self {
        Self {
            filters: levels.map(OutputFilter::new),
            commanded: levels,
            t,
            tick_t0: t,
            tick_k: 0,
        }
    }

    /// Run forward to `to`, applying every tick due on the way.
    fn advance(&mut self, gen: &mut Gen, to: f64) {
        loop {
            let tick = self.tick_t0 + self.tick_k as f64 * TICK_PERIOD_S;
            if tick > to {
                break;
            }
            self.run_to(tick);
            self.commanded = gen.evaluate(tick);
            self.tick_k += 1;
        }
        self.run_to(to);
    }

    /// Run forward to `to` with the commands held.
    fn run_to(&mut self, to: f64) {
        if to > self.t {
            let h = (to - self.t) as f32;
            for (f, x) in self.filters.iter_mut().zip(self.commanded) {
                f.step(h, x);
            }
            self.t = to;
        }
    }

    /// Voltage at one plant input now.
    fn output(&self, ch: usize) -> f32 {
        self.filters[ch].output()
    }
}

/// The output half of the simulated rig.
struct Gen {
    scales: [DacScale; N_OUTPUTS],
    staged: [Option<Generator>; N_OUTPUTS],
    active: [Option<Generator>; N_OUTPUTS],
    /// Level held when no waveform is running, volts.
    held: [f32; N_OUTPUTS],
    /// Quantised level actually commanded, volts — before the filter.
    applied: [f32; N_OUTPUTS],
    codes: [u8; N_OUTPUTS],
    /// Simulation time at which the running waveforms started.
    started_at: Option<f64>,
    /// The acquisition loop's clock. It only advances while streaming — the
    /// simulated plant does not exist between recordings. Double precision
    /// because a run lasts many minutes, and at a thousand seconds an `f32`
    /// clock is only good to 60 µs — a visible phase error on a fast sine.
    sim_t: f64,
}

impl Gen {
    fn new(scales: [DacScale; N_OUTPUTS]) -> Self {
        Self {
            scales,
            staged: [const { None }; N_OUTPUTS],
            active: [const { None }; N_OUTPUTS],
            held: scales.map(|s| s.min_v),
            applied: scales.map(|s| s.min_v),
            codes: [0; N_OUTPUTS],
            started_at: None,
            sim_t: 0.0,
        }
    }

    /// Level commanded on both outputs at simulation time `t`, quantised
    /// through the 8-bit duty.
    ///
    /// A finite waveform that has run its course holds its last value and
    /// stops being a waveform, exactly as the firmware's tick does — a sine
    /// of `cycles = 3` ends on its centre line and stays there, rather than
    /// running on until the next command.
    fn evaluate(&mut self, t: f64) -> [f32; N_OUTPUTS] {
        let t_run = self.started_at.map(|t0| (t - t0) as f32);
        for ch in 0..N_OUTPUTS {
            let want = match (&mut self.active[ch], t_run) {
                (Some(gen), Some(t_run)) => gen.sample(t_run),
                _ => self.held[ch],
            };
            let code = self.scales[ch].to_code(want);
            self.codes[ch] = code;
            self.applied[ch] = self.scales[ch].to_volts(code);
        }
        if let Some(t_run) = t_run {
            for ch in 0..N_OUTPUTS {
                if self.active[ch].as_ref().is_some_and(|g| g.is_done(t_run)) {
                    self.held[ch] = self.applied[ch];
                    self.active[ch] = None;
                }
            }
            if self.active.iter().all(|a| a.is_none()) {
                self.started_at = None;
            }
        }
        self.applied
    }

    fn status(&self) -> GenStatus {
        GenStatus {
            t_ms: self
                .started_at
                .map(|t0| ((self.sim_t - t0) * 1000.0) as u32)
                .unwrap_or(0),
            volts: self.applied,
            codes: self.codes,
            running: self.started_at.is_some(),
        }
    }

    /// Apply one generator command and produce its reply.
    fn handle(&mut self, cmd: HostToGen) -> GenToHost<'static> {
        match cmd {
            HostToGen::Ping => GenToHost::Pong,
            HostToGen::Info => GenToHost::Info(GenInfo {
                protocol: plant_trace_proto::PROTOCOL_VERSION,
                firmware: FIRMWARE,
                outputs: N_OUTPUTS as u8,
                bits: 8,
                tick_hz: TICK_HZ,
            }),
            HostToGen::Status => GenToHost::Status(self.status()),

            HostToGen::SetScale { ch, scale } => match self.scales.get_mut(ch as usize) {
                Some(slot) => {
                    *slot = scale;
                    GenToHost::Ok
                }
                None => GenToHost::Error(GenError::BadChannel),
            },

            HostToGen::SetLevel { ch, volts } => {
                if ch as usize >= N_OUTPUTS {
                    GenToHost::Error(GenError::BadChannel)
                } else {
                    self.held[ch as usize] = self.scales[ch as usize].clamp(volts);
                    self.active[ch as usize] = None;
                    self.staged[ch as usize] = None;
                    if self.active.iter().all(|a| a.is_none()) {
                        self.started_at = None;
                    }
                    let t = self.sim_t;
                    self.evaluate(t);
                    GenToHost::Ok
                }
            }

            HostToGen::Program { ch, wave } => {
                if ch as usize >= N_OUTPUTS {
                    GenToHost::Error(GenError::BadChannel)
                } else {
                    let (lo, hi) = wave.span();
                    let scale = self.scales[ch as usize];
                    if lo < scale.min_v - 1e-6 || hi > scale.max_v + 1e-6 {
                        GenToHost::Error(GenError::OutOfRange)
                    } else {
                        self.staged[ch as usize] = Some(Generator::new(wave));
                        GenToHost::Ok
                    }
                }
            }

            HostToGen::Start => {
                let t = self.sim_t;
                for ch in 0..N_OUTPUTS {
                    if let Some(g) = self.staged[ch].take() {
                        self.active[ch] = Some(g);
                    }
                }
                self.started_at = Some(t);
                self.evaluate(t);
                GenToHost::Ok
            }

            HostToGen::Stop => {
                for ch in 0..N_OUTPUTS {
                    self.held[ch] = self.applied[ch];
                    self.active[ch] = None;
                }
                self.started_at = None;
                GenToHost::Ok
            }

            HostToGen::Park => {
                for ch in 0..N_OUTPUTS {
                    self.held[ch] = self.scales[ch].min_v;
                    self.active[ch] = None;
                    self.staged[ch] = None;
                }
                self.started_at = None;
                let t = self.sim_t;
                self.evaluate(t);
                GenToHost::Ok
            }
        }
    }
}

/// Serve the rig until the process is killed.
pub fn run(opts: Options) -> Result<()> {
    let listener = TcpListener::bind(&opts.addr)
        .with_context(|| format!("binding the rig port {}", opts.addr))?;

    println!("simulator listening on tcp://{}", opts.addr);
    println!(
        "  {} Hz, {:.1}× real time, {:.0} s of settling before each run, plant: {}",
        opts.fs_hz,
        opts.speed,
        opts.settle_s,
        match opts.plant {
            SimPlant::Model => "model".to_string(),
            SimPlant::Wire { from } => format!("wire from output {from}"),
        }
    );

    let never = AtomicBool::new(false);
    for stream in listener.incoming() {
        match stream {
            Ok(s) => {
                if let Err(e) = serve(s, &opts, &never) {
                    eprintln!("session ended: {e:#}");
                }
            }
            Err(e) => eprintln!("accept failed: {e}"),
        }
    }
    Ok(())
}

/// A simulated rig running on a background thread of this process.
///
/// What the GUI's "simulated rig" is: no second process, no fixed port to
/// collide with. Open it like any other rig, with [`Self::spec`].
///
/// Sessions are served one at a time, as the real USB link would: close one
/// [`crate::daq::Daq`] before opening the next, or the second waits.
///
/// Dropping the handle shuts the simulator down (see [`Self::shutdown`]).
pub struct SimHandle {
    spec: String,
    addr: SocketAddr,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl SimHandle {
    /// Link spec to hand to [`crate::daq::Daq::open`]: `tcp://127.0.0.1:<port>`.
    pub fn spec(&self) -> &str {
        &self.spec
    }

    /// Address the simulator is listening on.
    pub fn addr(&self) -> SocketAddr {
        self.addr
    }

    /// Stop serving and wait for the thread to finish.
    ///
    /// A session in progress is ended from the simulator's side — its
    /// [`crate::daq::Daq`] sees the link close. Returns within a few
    /// milliseconds: the serving loop polls for the request between blocks
    /// and while waiting for a slow reader.
    pub fn shutdown(mut self) {
        self.stop_and_join();
    }

    fn stop_and_join(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for SimHandle {
    fn drop(&mut self) {
        self.stop_and_join();
    }
}

/// Start a simulated rig on an ephemeral loopback port, served from a
/// background thread, and return once it is accepting connections.
///
/// `opts.addr` is ignored: the OS picks a free port, so any number of these can
/// run side by side (parallel tests, a GUI next to a CLI simulator).
pub fn spawn(opts: Options) -> Result<SimHandle> {
    let listener = TcpListener::bind("127.0.0.1:0").context("binding a loopback port")?;
    let addr = listener.local_addr()?;
    // Non-blocking so the accept loop can notice a shutdown request.
    listener.set_nonblocking(true)?;
    let stop = Arc::new(AtomicBool::new(false));

    let flag = stop.clone();
    let thread = thread::Builder::new()
        .name("plant-trace-sim".into())
        .spawn(move || {
            while !flag.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((s, _)) => {
                        if let Err(e) = serve(s, &opts, &flag) {
                            if !flag.load(Ordering::Relaxed) {
                                eprintln!("simulated rig: session ended: {e:#}");
                            }
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        thread::sleep(Duration::from_millis(5));
                    }
                    Err(e) => {
                        eprintln!("simulated rig: accept failed: {e}");
                        thread::sleep(Duration::from_millis(50));
                    }
                }
            }
        })
        .context("starting the simulator thread")?;

    Ok(SimHandle {
        spec: format!("tcp://{addr}"),
        addr,
        stop,
        thread: Some(thread),
    })
}

fn serve(stream: TcpStream, opts: &Options, stop: &AtomicBool) -> Result<()> {
    stream.set_nodelay(true)?;
    // Non-blocking: a read that finds nothing returns at once, so a fast
    // simulation is not throttled by a read timeout on every block, and a
    // command is seen within one block of arriving. Writes retry instead (see
    // `Conn::send`).
    stream.set_nonblocking(true)?;
    let mut conn = Conn::new(stream, stop);

    // The rate the host asked for, not the one the options carry: a simulated
    // rig that reports one rate and produces another puts every timestamp
    // downstream out by that ratio.
    let mut fs_hz = opts.fs_hz;
    let mut dt = 1.0 / fs_hz as f32;
    let mut plant = Plant::new(opts.params);
    let mut gen = Gen::new(opts.dac);
    // The outputs are re-evaluated on their own tick, not once per sample:
    // that staircase is what the filter actually sees, and above ~20 Hz it is
    // the rig's real bandwidth limit rather than the filter. Samples are
    // counted as integers from the start of the stream, so the sample clock
    // never drifts the way an accumulated float would.
    let mut chain = Chain::new(0.0, opts.dac.map(|s| s.min_v));
    let mut stream_t0 = 0.0f64;
    let mut sample_n = 0u64;
    // `P_e` at the current sample instant, for the interpolation that places
    // the plant's channel later in the scan.
    let mut p_e_now = 0.0f32;
    let mut streaming = false;
    let mut seq = 0u32;
    let mut blocks = 0u32;
    let mut next_block_at = Instant::now();

    loop {
        if stop.load(Ordering::Relaxed) {
            return Ok(());
        }
        // Commands first, all of them: they are rare, and a `Stop` should not
        // wait for the block that is being assembled.
        while let Some(cmd) = conn.next::<HostToDaq>()? {
            match cmd {
                HostToDaq::Ping => conn.send(&DaqToHost::Pong)?,
                HostToDaq::Info => conn.send(&DaqToHost::Info(DaqInfo {
                    protocol: plant_trace_proto::PROTOCOL_VERSION,
                    firmware: FIRMWARE,
                    channels: N_CHANNELS as u8,
                    bits: opts.adc.bits,
                    full_scale_mv: (opts.adc.full_scale_v * 1000.0) as u16,
                    oversample: 8,
                    block_samples: BLOCK_SAMPLES as u16,
                    max_fs_hz: MAX_FS_HZ,
                }))?,
                HostToDaq::Calibrate => conn.send(&DaqToHost::Calibrated)?,
                HostToDaq::Gen(cmd) => {
                    let reply = gen.handle(cmd);
                    conn.send(&DaqToHost::Gen(reply))?;
                }
                HostToDaq::Start { fs_hz: want, .. } => {
                    if streaming {
                        conn.send(&DaqToHost::Error(DaqError::Busy))?;
                    } else if want != 0 && !(MIN_FS_HZ..=MAX_FS_HZ).contains(&want) {
                        conn.send(&DaqToHost::Error(DaqError::BadConfig))?;
                    } else {
                        fs_hz = if want == 0 { opts.fs_hz } else { want };
                        dt = 1.0 / fs_hz as f32;
                        // Settle the plant — and the output filters — at
                        // whatever is being held, so the recording starts from
                        // equilibrium rather than from a boot transient.
                        let t = gen.sim_t;
                        let v = gen.evaluate(t);
                        chain = Chain::new(t, v);
                        stream_t0 = t;
                        sample_n = 0;
                        p_e_now = match opts.plant {
                            SimPlant::Model => {
                                box_p_e(plant.settle(dt, model_u_t(v[0]), v[1], opts.settle_s))
                            }
                            SimPlant::Wire { from } => v[wire_source(from)],
                        };
                        streaming = true;
                        seq = 0;
                        blocks = 0;
                        next_block_at = Instant::now();
                        conn.send(&DaqToHost::Started { fs_hz })?;
                    }
                }
                HostToDaq::Stop => {
                    streaming = false;
                    conn.send(&DaqToHost::Stopped { blocks, dropped: 0 })?;
                }
            }
        }
        if conn.closed {
            return Ok(());
        }
        if !streaming {
            thread::sleep(Duration::from_millis(1));
            continue;
        }

        // One block per pass, paced to `speed`. Short sleeps rather than one
        // long one, so commands keep being answered while a slow block (6.4 s
        // of them at 10 Hz) is being waited for.
        let now = Instant::now();
        if now < next_block_at {
            thread::sleep((next_block_at - now).min(Duration::from_millis(1)));
            continue;
        }
        let speed = opts
            .speed
            .clamp(0.01, MAX_SAMPLES_PER_WALL_S / fs_hz as f32);
        let block_wall = Duration::from_secs_f32(BLOCK_SAMPLES as f32 / fs_hz as f32 / speed);
        next_block_at += block_wall;

        // How much later than channel 0 each channel is converted — never more
        // than a sample period, which the rig's own rates cannot reach anyway.
        let offset = |ch: usize| (ch as f64 * opts.scan_spacing_s).clamp(0.0, dt as f64);
        let (off_p, off_e) = (offset(1), offset(2));

        let mut counts = [0i16; BLOCK_SAMPLES * N_CHANNELS];
        for i in 0..BLOCK_SAMPLES {
            let t = stream_t0 + sample_n as f64 / fs_hz as f64;
            let t_next = stream_t0 + (sample_n + 1) as f64 / fs_hz as f64;

            // What reaches the plant is the filter's output, and it is also
            // what the two feedback channels measure — the command itself is
            // never recorded anywhere. Channel `k` is converted `k` scan slots
            // after channel 0, so it holds the signal a little later than the
            // row's nominal instant; the chain is simply read on its way past.
            chain.advance(&mut gen, t);
            let u_now = chain.output(0);
            chain.advance(&mut gen, t + off_p);
            let p_late = chain.output(1);
            chain.advance(&mut gen, t + off_e);
            let wire = match opts.plant {
                SimPlant::Wire { from } => Some(chain.output(wire_source(from))),
                SimPlant::Model => None,
            };
            chain.advance(&mut gen, t_next);

            let p_e_late = match wire {
                Some(v) => v,
                None => {
                    let p_e_next =
                        box_p_e(plant.step(dt, model_u_t(chain.output(0)), chain.output(1)));
                    let late = p_e_now + (p_e_next - p_e_now) * (off_e as f32 / dt);
                    p_e_now = p_e_next;
                    late
                }
            };

            counts[i * N_CHANNELS] = to_counts(u_now, &opts.adc);
            counts[i * N_CHANNELS + 1] = to_counts(p_late, &opts.adc);
            counts[i * N_CHANNELS + 2] = to_counts(p_e_late, &opts.adc);
            sample_n += 1;
            gen.sim_t = t_next;
        }

        let bytes: Vec<u8> = counts.iter().flat_map(|c| c.to_le_bytes()).collect();
        conn.send(&DaqToHost::Block(SampleBlock {
            seq,
            n: BLOCK_SAMPLES as u16,
            channels: N_CHANNELS as u8,
            dropped: 0,
            data: &bytes,
        }))?;
        seq = seq.wrapping_add(1);
        blocks = blocks.wrapping_add(1);
    }
}

/// The box takes `u_T` and gives `P_e` in a 2.25-2.75 V window around
/// 2.5 V, while [`plant_model`] works in 0-1 on both — the range the
/// assignment's figures are in. The simulator maps one onto the other
/// linearly, window bottom to 0 and window top to 1; how the real box scales
/// them is what the static curve measures. `p_s` is 0-1 V on both sides.
const BOX_WINDOW_V: (f32, f32) = (DacScale::U_T_NOMINAL.min_v, DacScale::U_T_NOMINAL.max_v);

/// Volts at the box's `u_T` terminal → the model's valve command.
fn model_u_t(volts: f32) -> f32 {
    (volts - BOX_WINDOW_V.0) / (BOX_WINDOW_V.1 - BOX_WINDOW_V.0)
}

/// The model's `P_e` → volts at the box's `P_e` terminal.
fn box_p_e(model: f32) -> f32 {
    BOX_WINDOW_V.0 + model * (BOX_WINDOW_V.1 - BOX_WINDOW_V.0)
}

/// Output index a [`SimPlant::Wire`] reads, with an out-of-range index taken
/// as the last output rather than a panic.
fn wire_source(from: u8) -> usize {
    (from as usize).min(N_OUTPUTS - 1)
}

/// Volts → counts, with the ADC's saturation. Anything above full scale reads
/// as full scale, exactly as the SAADC would.
fn to_counts(volts: f32, adc: &AdcScale) -> i16 {
    let full = (1i32 << adc.bits) - 1;
    let raw = (volts / adc.volts_per_count()).round() as i32;
    raw.clamp(0, full) as i16
}

/// A framed TCP connection, non-blocking enough to interleave commands with
/// the simulation loop.
struct Conn<'a> {
    stream: TcpStream,
    /// Shutdown request, checked while a write waits for room.
    stop: &'a AtomicBool,
    decoder: frame::Decoder<{ frame::MAX_FRAME }>,
    scratch: [u8; frame::MAX_FRAME],
    out: [u8; frame::MAX_FRAME],
    buf: [u8; 512],
    filled: usize,
    cursor: usize,
    closed: bool,
}

impl<'a> Conn<'a> {
    fn new(stream: TcpStream, stop: &'a AtomicBool) -> Self {
        Self {
            stream,
            stop,
            decoder: frame::Decoder::new(),
            scratch: [0; frame::MAX_FRAME],
            out: [0; frame::MAX_FRAME],
            buf: [0; 512],
            filled: 0,
            cursor: 0,
            closed: false,
        }
    }

    fn send<T: serde::Serialize>(&mut self, msg: &T) -> Result<()> {
        let n = frame::encode(msg, &mut self.scratch, &mut self.out)
            .map_err(|e| anyhow::anyhow!("encoding a reply: {e}"))?;
        // The socket is non-blocking, so a full send buffer is a retry rather
        // than a wait inside the kernel — which is what lets a shutdown request
        // get through to a session whose host has stopped reading.
        let deadline = Instant::now() + WRITE_TIMEOUT;
        let mut pending = &self.out[..n];
        while !pending.is_empty() {
            match self.stream.write(pending) {
                Ok(0) => anyhow::bail!("the host closed the link"),
                Ok(k) => pending = &pending[k..],
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    if self.stop.load(Ordering::Relaxed) {
                        anyhow::bail!("shutting down");
                    }
                    if Instant::now() >= deadline {
                        anyhow::bail!("the host stopped reading for {WRITE_TIMEOUT:?}");
                    }
                    thread::sleep(Duration::from_micros(200));
                }
                Err(e) => return Err(e.into()),
            }
        }
        Ok(())
    }

    /// One pending message, if a whole frame has arrived.
    ///
    /// Bytes left over from a read are kept: two commands can land in one
    /// packet, and dropping the second would lose a `Start` that arrived right
    /// behind a `Program`.
    fn next<T: serde::de::DeserializeOwned>(&mut self) -> Result<Option<T>> {
        use std::io::Read;
        'search: loop {
            while self.cursor < self.filled {
                let byte = self.buf[self.cursor];
                self.cursor += 1;
                if self.decoder.push(byte) {
                    break 'search;
                }
            }
            match self.stream.read(&mut self.buf) {
                Ok(0) => {
                    self.closed = true;
                    return Ok(None);
                }
                Ok(n) => {
                    self.filled = n;
                    self.cursor = 0;
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                    ) =>
                {
                    return Ok(None)
                }
                Err(e) => return Err(e.into()),
            }
        }
        match self.decoder.frame() {
            Some(raw) => Ok(frame::decode::<T>(raw).ok()),
            None => Ok(None),
        }
    }
}
