//! plant-trace: acquisition, experiment orchestration and identification for
//! the steam-turbine characterisation rig.
//!
//! See `docs/PLAN.md` for the design and `docs/EXPERIMENTS.md` for the bench
//! procedure.

use std::{
    path::PathBuf,
    time::{Duration, Instant},
};

use anyhow::Result;
use clap::{Args, Parser, Subcommand};
use plant_trace_proto::{
    gen::N_OUTPUTS,
    scale::{AdcScale, DacScale},
    Channel,
};

use plant_trace::{
    analysis::report,
    bode::{self, BodePlan, ConsoleBodeObserver},
    check,
    csvout::{RunMeta, RunWriter},
    daq::{Daq, Event},
    experiment::Experiment,
    link::DEFAULT_BAUD,
    runner::{self, RunConfig},
    sim,
};

#[derive(Parser)]
#[command(name = "plant-trace", version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Print what the acquisition firmware reports about itself.
    Info(DeviceArgs),
    /// Record the three channels to a CSV file.
    Record(RecordArgs),
    /// Run a simulated rig, so everything downstream can be exercised without
    /// the bench.
    Simulate(SimulateArgs),
    /// Drive the signal generator by hand.
    Gen(GenArgs),
    /// Measure the analog outputs with the rig's own ADC.
    Check(CheckArgs),
    /// Run an experiment description against the rig.
    Run(RunArgs),
    /// Open the bench GUI: live view, output checks, scenario editor and
    /// automatic Bode sweeps.
    #[cfg(feature = "gui")]
    Gui(GuiArgs),
    /// Turn a finished run into tables and figures.
    Analyze(AnalyzeArgs),
    /// Measure a frequency response automatically: one sine per frequency on
    /// one stream, fitted as it goes.
    Bode(BodeArgs),
}

#[derive(Args)]
struct BodeArgs {
    /// Bode plan (TOML). See `experiments/bode-*.toml`, or write the default
    /// one with `--init`.
    #[arg(required_unless_present = "init")]
    plan: Option<PathBuf>,
    /// Write the default plan to this path and exit.
    #[arg(long, value_name = "PATH", conflicts_with = "plan")]
    init: Option<PathBuf>,
    /// Rig link: a serial device or `tcp://host:port`.
    #[arg(long, default_value = "/dev/ttyACM0")]
    daq: String,
    /// Baud rate of a serial link.
    #[arg(long, default_value_t = DEFAULT_BAUD)]
    baud: u32,
    /// Where to put `bode.csv`, `bode.json` and `stream.csv`; defaults to a
    /// timestamped directory under `data/`.
    #[arg(long)]
    out: Option<PathBuf>,
}

#[derive(Args)]
struct AnalyzeArgs {
    /// Directory written by `plant-trace run` (the one holding `run.json`).
    run_dir: PathBuf,
    /// Write the CSV tables but skip the gnuplot figures.
    #[arg(long)]
    no_plots: bool,
}

#[derive(Args)]
struct RunArgs {
    /// Experiment file (TOML). See `experiments/` for the ones the assignment
    /// asks for.
    experiment: PathBuf,
    /// Rig link: a serial device or `tcp://host:port`.
    #[arg(long, default_value = "/dev/ttyACM0")]
    daq: String,
    /// Baud rate of a serial link.
    #[arg(long, default_value_t = DEFAULT_BAUD)]
    daq_baud: u32,
    /// Where to put the CSVs and the manifest; defaults to a timestamped
    /// directory under `data/`.
    #[arg(long)]
    out_dir: Option<PathBuf>,
}

#[derive(Args)]
struct CheckArgs {
    #[command(flatten)]
    device: DeviceArgs,
    #[command(subcommand)]
    action: CheckAction,
}

#[derive(Subcommand)]
enum CheckAction {
    /// Step the outputs through a range of levels and read each plateau back.
    ///
    /// The fitted line is the bench calibration: paste its two numbers into
    /// the `[outputs.*]` tables of the experiment files.
    Dc {
        /// Outputs to drive.
        #[arg(long, value_enum, default_value_t = ChannelSel::Both)]
        ch: ChannelSel,
        /// First level, volts at the plant input, for every driven output.
        /// Default: the bottom of each output's window (2.25 V on u_t, 0 V on
        /// p_s).
        #[arg(long)]
        from: Option<f32>,
        /// Last level, volts. Default: the top of each output's window (2.75 V
        /// on u_t, 1 V on p_s). Either end is clamped to the window anyway.
        #[arg(long)]
        to: Option<f32>,
        /// Number of levels, including both ends.
        #[arg(long, default_value_t = 11)]
        points: usize,
        /// Discarded after each level change, milliseconds.
        #[arg(long, default_value_t = 250)]
        settle_ms: u64,
        /// Averaged at each level, milliseconds.
        #[arg(long, default_value_t = 500)]
        average_ms: u64,
        /// Sample rate; 0 asks the firmware for its default.
        #[arg(long, default_value_t = 0)]
        fs: u32,
    },
    /// Apply a sinusoid and fit it back off the sense channels.
    Sine {
        /// Outputs to drive.
        #[arg(long, value_enum, default_value_t = ChannelSel::Both)]
        ch: ChannelSel,
        /// Level the sinusoid rides on, volts, for every driven output.
        /// Default: the middle of each output's window (2.5 V on u_t, 0.5 V on
        /// p_s).
        #[arg(long)]
        center: Option<f32>,
        /// Peak amplitude, volts. Default: 40 % of each output's window
        /// (0.2 V on u_t, 0.4 V on p_s).
        #[arg(long)]
        amplitude: Option<f32>,
        /// Frequency, hertz.
        #[arg(long, default_value_t = 10.0)]
        freq: f32,
        /// Length of the window the fit runs on, seconds.
        #[arg(long, default_value_t = 2.0)]
        seconds: f32,
        /// Discarded before that window, milliseconds.
        #[arg(long, default_value_t = 250)]
        settle_ms: u64,
        /// Sample rate. The default is deliberately twice the firmware's, so
        /// the images the 1 kHz output tick leaves do not fold onto the
        /// fundamental.
        #[arg(long, default_value_t = 2000)]
        fs: u32,
    },
}

/// Which outputs a check should drive.
#[derive(Clone, Copy, Debug, clap::ValueEnum)]
enum ChannelSel {
    /// Valve command only.
    #[value(name = "u_t")]
    ValveCmd,
    /// Steam pressure only.
    #[value(name = "p_s")]
    SteamPressure,
    /// Both, driven with the same signal.
    Both,
}

impl ChannelSel {
    fn indices(self) -> Vec<u8> {
        match self {
            ChannelSel::ValveCmd => vec![0],
            ChannelSel::SteamPressure => vec![1],
            ChannelSel::Both => vec![0, 1],
        }
    }
}

#[derive(Args)]
struct GenArgs {
    #[command(flatten)]
    device: DeviceArgs,
    #[command(subcommand)]
    action: GenAction,
}

#[derive(Subcommand)]
enum GenAction {
    /// Print what the generator firmware reports about itself.
    Info,
    /// Print the current output levels.
    Status,
    /// Drive one output to a fixed level.
    Level {
        /// Output: `u_t` or `p_s`.
        #[arg(long)]
        ch: OutputName,
        /// Level in volts at the plant input.
        #[arg(long)]
        volts: f32,
    },
    /// Drive both outputs to their configured minimum.
    Park,
}

/// Names the two outputs the way the assignment does.
#[derive(Clone, Copy, Debug, clap::ValueEnum)]
enum OutputName {
    /// Valve command.
    #[value(name = "u_t")]
    ValveCmd,
    /// Steam pressure.
    #[value(name = "p_s")]
    SteamPressure,
}

impl OutputName {
    fn index(self) -> u8 {
        match self {
            OutputName::ValveCmd => 0,
            OutputName::SteamPressure => 1,
        }
    }
}

#[derive(Args, Clone)]
struct DeviceArgs {
    /// Rig link: a serial device (`/dev/ttyACM0`) or `tcp://host:port`.
    #[arg(long, default_value = "/dev/ttyACM0")]
    daq: String,
    /// Baud rate for a serial link.
    #[arg(long, default_value_t = DEFAULT_BAUD)]
    baud: u32,
}

#[derive(Args)]
struct RecordArgs {
    #[command(flatten)]
    device: DeviceArgs,
    /// Where to write the CSV.
    #[arg(long, short)]
    out: PathBuf,
    /// Recording length in seconds.
    #[arg(long, short, default_value_t = 10.0)]
    duration: f64,
    /// Sample rate per channel; 0 asks the firmware for its default.
    #[arg(long, default_value_t = 0)]
    fs: u32,
    /// Free-form note stored in the file header.
    #[arg(long)]
    note: Option<String>,
    /// Run the SAADC offset calibration before recording.
    #[arg(long)]
    calibrate: bool,
}

#[derive(Args)]
struct SimulateArgs {
    /// Address the simulated rig listens on.
    #[arg(long, default_value = "127.0.0.1:7801")]
    addr: String,
    /// Sample rate offered to the host.
    #[arg(long, default_value_t = 1000)]
    fs: u32,
    /// Wall-clock speed-up; sample timestamps are unaffected.
    #[arg(long, default_value_t = 1.0)]
    speed: f32,
    /// Simulated seconds of settling before each recording starts.
    #[arg(long, default_value_t = 120.0)]
    settle: f32,
    /// What the `P_e` input is connected to: the plant model, or a wire from
    /// the `u_T` or `p_s` filter output (the bench with the plant unplugged).
    #[arg(long, value_enum, default_value_t = SimPlantArg::Model)]
    plant: SimPlantArg,
}

/// `--plant` for the simulator.
#[derive(Clone, Copy, Debug, clap::ValueEnum)]
enum SimPlantArg {
    /// The steam-turbine model.
    Model,
    /// `P_e` wired to the `u_T` filter output.
    WireU,
    /// `P_e` wired to the `p_s` filter output.
    WireP,
}

fn main() -> Result<()> {
    match Cli::parse().command {
        Command::Info(args) => info(args),
        Command::Record(args) => record(args),
        Command::Simulate(args) => simulate(args),
        Command::Gen(args) => generator(args),
        Command::Check(args) => run_check(args),
        Command::Run(args) => run_experiment(args),
        #[cfg(feature = "gui")]
        Command::Gui(args) => plant_trace::gui::run(plant_trace::gui::Options {
            daq: args.daq,
            light: args.light,
        }),
        Command::Analyze(args) => analyze(args),
        Command::Bode(args) => run_bode(args),
    }
}

fn run_bode(args: BodeArgs) -> Result<()> {
    if let Some(path) = args.init {
        let plan = BodePlan::default();
        plan.save(&path)?;
        println!(
            "default plan written to {} — {} point(s), about {:.0} min",
            path.display(),
            plan.frequencies_hz.len(),
            plan.estimated_duration().as_secs_f64() / 60.0
        );
        return Ok(());
    }
    let Some(path) = args.plan else {
        anyhow::bail!("no plan given");
    };
    let plan = BodePlan::load(&path)?;
    let out_dir = args.out.unwrap_or_else(|| bode::default_out_dir(&plan));

    println!("bode        {}", plan.name);
    println!("rig         {}", args.daq);
    println!(
        "excite      {} = {:.3} ± {:.3} V, {} held at {:.3} V",
        plan.excite.name(),
        plan.center_v,
        plan.amplitude_v,
        plan.excite.other().name(),
        plan.hold_v
    );
    println!(
        "response    {} against the {} sense channel; skew {:+.0} µs {}",
        plan.response.name(),
        plan.excite.name(),
        plan.skew_s() * 1e6,
        if plan.correct_skew {
            "corrected"
        } else {
            "not corrected"
        }
    );
    println!(
        "duration    about {:.1} min",
        plan.estimated_duration().as_secs_f64() / 60.0
    );
    println!("output      {}", out_dir.display());

    let mut device = Daq::open(&args.daq, args.baud)?;
    let result = bode::run_bode(
        &mut device,
        &plan,
        Some(&out_dir),
        &mut ConsoleBodeObserver::new(),
    )?;

    println!();
    if let Some(d) = result.delay {
        println!(
            "pure-delay fit through the raw phase: {:+.1} µs (rms {:.2}° over {} points)",
            d.delay_s * 1e6,
            d.rms_residual_deg,
            d.points
        );
        if let Some(sp) = result.implied_scan_spacing_s {
            println!(
                "  — if the true phase is zero (a wire), the scan spacing is {:.1} µs per \
                 channel (nominal {:.0} µs)",
                sp * 1e6,
                plant_trace_proto::SCAN_CHANNEL_SPACING_S * 1e6
            );
        }
    }
    println!("results     {}", out_dir.display());
    Ok(())
}

fn analyze(args: AnalyzeArgs) -> Result<()> {
    println!("analysing {}", args.run_dir.display());
    let summary = report::analyze_run(&args.run_dir, !args.no_plots)?;
    println!(
        "\n{} step(s), {} static curve(s), {} Bode point(s); {} file(s) written to {}",
        summary.steps,
        summary.curves,
        summary.bode_points,
        summary.artefacts.len(),
        args.run_dir.join("analysis").display()
    );
    Ok(())
}

fn run_experiment(args: RunArgs) -> Result<()> {
    let experiment = Experiment::load(&args.experiment)?;
    let dir = runner::run(
        &experiment,
        &RunConfig {
            daq: args.daq,
            daq_baud: args.daq_baud,
            out_dir: args.out_dir,
        },
    )?;
    println!("results     {}", dir.display());
    Ok(())
}

#[cfg(feature = "gui")]
#[derive(Args)]
struct GuiArgs {
    /// Rig link to open at start-up (`/dev/ttyACM0`, `tcp://host:port`).
    #[arg(long)]
    daq: Option<String>,
    /// Start in the light theme.
    #[arg(long)]
    light: bool,
}

fn run_check(args: CheckArgs) -> Result<()> {
    let mut device = Daq::open(&args.device.daq, args.device.baud)?;
    match args.action {
        CheckAction::Dc {
            ch,
            from,
            to,
            points,
            settle_ms,
            average_ms,
            fs,
        } => {
            let mut opts = check::DcOptions {
                channels: ch.indices(),
                points,
                settle: Duration::from_millis(settle_ms),
                average: Duration::from_millis(average_ms),
                fs_hz: fs,
                ..check::DcOptions::default()
            };
            if let Some(v) = from {
                opts.from_v = [v; N_OUTPUTS];
            }
            if let Some(v) = to {
                opts.to_v = [v; N_OUTPUTS];
            }
            println!("DC sweep: {points} level(s)");
            for c in &opts.channels {
                let i = *c as usize;
                println!(
                    "  {}: {:.3} V to {:.3} V",
                    check::name(*c),
                    opts.from_v[i],
                    opts.to_v[i]
                );
            }
            print_dc(&check::dc(&mut device, &opts)?);
        }
        CheckAction::Sine {
            ch,
            center,
            amplitude,
            freq,
            seconds,
            settle_ms,
            fs,
        } => {
            let mut opts = check::SineOptions {
                channels: ch.indices(),
                freq_hz: freq,
                duration: Duration::from_secs_f32(seconds.max(0.1)),
                settle: Duration::from_millis(settle_ms),
                fs_hz: fs,
                ..check::SineOptions::default()
            };
            if let Some(v) = center {
                opts.center_v = [v; N_OUTPUTS];
            }
            if let Some(v) = amplitude {
                opts.amplitude_v = [v; N_OUTPUTS];
            }
            println!("sine: {freq} Hz for {seconds:.1} s");
            for c in &opts.channels {
                let i = *c as usize;
                println!(
                    "  {}: {:.3} V peak on {:.3} V",
                    check::name(*c),
                    opts.amplitude_v[i],
                    opts.center_v[i]
                );
            }
            print_sine(&check::sine(&mut device, &opts)?);
        }
    }
    Ok(())
}

fn print_dc(report: &check::DcReport) {
    println!(
        "sampled at {} Hz; {} samples averaged per level\n",
        report.fs_hz,
        report.points.first().map(|p| p.measured[0].n).unwrap_or(0),
    );
    println!("   u_t asked  code  u_t read   pp mV    p_s asked  code  p_s read   pp mV");
    for p in &report.points {
        println!(
            "     {:6.3}  {:>4}  {:8.4}  {:6.2}       {:6.3}  {:>4}  {:8.4}  {:6.2}",
            p.commanded_v[0],
            p.codes[0],
            p.measured[0].mean_v,
            p.measured[0].span_v() * 1000.0,
            p.commanded_v[1],
            p.codes[1],
            p.measured[1].mean_v,
            p.measured[1].span_v() * 1000.0,
        );
    }

    println!();
    for (i, cal) in report.calibration.iter().enumerate() {
        let Some(cal) = cal else { continue };
        println!(
            "{:>4}  {:.6} V/code   offset {:+.4} V   worst deviation {:.2} mV over {} points",
            check::name(i as u8),
            cal.volts_per_code,
            cal.offset_v,
            cal.max_deviation_v * 1000.0,
            cal.points,
        );
    }

    let measured: Vec<(usize, &check::Calibration)> = report
        .calibration
        .iter()
        .enumerate()
        .filter_map(|(i, c)| c.as_ref().map(|c| (i, c)))
        .collect();
    if !measured.is_empty() {
        println!("\nfor the [outputs.*] tables of an experiment file:");
        for (i, cal) in measured {
            println!("  [outputs.{}]", check::name(i as u8));
            println!("  volts_per_code = {:.6}", cal.volts_per_code);
            println!("  offset_v = {:.6}", cal.offset_v);
        }
    }
}

fn print_sine(report: &check::SineReport) {
    println!(
        "sampled at {} Hz against a {} Hz output tick\n",
        report.fs_hz, report.tick_hz
    );
    println!("   ch   amplitude V    gain dB   centre err mV   residual");
    for r in &report.results {
        println!(
            "  {:>3}     {:8.4}     {:+7.3}       {:+7.2}      {:7.4}",
            check::name(r.channel),
            r.fit.amplitude,
            r.gain_db,
            r.center_error_v * 1000.0,
            r.fit.residual_ratio,
        );
    }

    println!(
        "\nthe design predicts {:+.3} dB and {:+.2}° here \
         (filter poles {:.0}/{:.0} Hz, plus the tick's hold)",
        report.expected.gain_db,
        report.expected.phase_deg,
        plant_trace::sim::OUTPUT_FILTER_POLES_HZ[0],
        plant_trace::sim::OUTPUT_FILTER_POLES_HZ[1],
    );
    println!(
        "absolute phase is not measurable from here: the host has no view of the tick \
         the waveform started on."
    );
    if let (Some(deg), Some(us)) = (report.skew_deg, report.skew_us()) {
        println!(
            "channel skew {deg:+.3}° ({us:+.1} µs) — both duties go out in one DMA transfer; \
             the SAADC reading p_s one scan slot after u_T accounts for {:+.3}°, \
             leaving {:+.3}° for the rig.",
            report.scan_skew_deg(),
            deg - report.scan_skew_deg(),
        );
    }
    let worst = report
        .results
        .iter()
        .map(|r| r.fit.residual_ratio)
        .fold(0.0, f64::max);
    if worst > 0.1 {
        if report.freq_hz * 50.0 > report.tick_hz as f64 {
            println!(
                "\nresidual {worst:.3} — the staircase the {} Hz tick leaves. It grows with \
                 frequency over tick and is the rig's bandwidth limit, not a fault.",
                report.tick_hz
            );
        } else {
            println!(
                "\nresidual {worst:.3} — more than a tenth of the signal is not at {:.1} Hz, \
                 and at this frequency the tick does not explain it. Check that the swing is \
                 inside the output's window and that nothing is clipping.",
                report.freq_hz
            );
        }
    }
}

fn generator(args: GenArgs) -> Result<()> {
    let mut device = Daq::open(&args.device.daq, args.device.baud)?;
    match args.action {
        GenAction::Info => {
            let info = device.gen_info()?;
            println!("firmware       {}", info.firmware);
            println!("protocol       {}", info.protocol);
            println!("outputs        {}", info.outputs);
            println!("resolution     {} bit", info.bits);
            println!("tick           {} Hz", info.tick_hz);
        }
        GenAction::Status => {
            let s = device.gen_status()?;
            println!(
                "u_t {:.4} V (code {})   p_s {:.4} V (code {})   {}",
                s.volts[0],
                s.codes[0],
                s.volts[1],
                s.codes[1],
                if s.running {
                    format!("running, t = {:.3} s", s.t_ms as f32 / 1000.0)
                } else {
                    "idle".to_string()
                }
            );
        }
        GenAction::Level { ch, volts } => {
            device.set_level(ch.index(), volts)?;
            // The firmware applies a level on its next 1 ms tick; a status
            // asked for straight away reports the code before it.
            std::thread::sleep(Duration::from_millis(5));
            let s = device.gen_status()?;
            println!(
                "{:?} set to {:.4} V (code {}) — the RC filter takes ~8 ms to get there",
                ch,
                s.volts[ch.index() as usize],
                s.codes[ch.index() as usize]
            );
        }
        GenAction::Park => {
            device.park()?;
            println!("both outputs parked");
        }
    }
    Ok(())
}

fn info(args: DeviceArgs) -> Result<()> {
    let mut device = Daq::open(&args.daq, args.baud)?;
    let info = device.info()?;
    let scale = info.adc_scale();
    println!("firmware       {}", info.firmware);
    println!("protocol       {}", info.protocol);
    println!("channels       {}", info.channels);
    println!("resolution     {} bit", info.bits);
    println!(
        "full scale     {:.3} V  ({:.1} µV/count, {:.2} V above the top of the u_t window)",
        scale.plant_full_scale_v(),
        scale.volts_per_count() * 1e6,
        scale.plant_full_scale_v() - DacScale::U_T_NOMINAL.max_v,
    );
    println!("oversample     {}×", info.oversample);
    println!("block          {} samples/channel", info.block_samples);
    println!("max rate       {} Hz", info.max_fs_hz);
    Ok(())
}

fn record(args: RecordArgs) -> Result<()> {
    let mut device = Daq::open(&args.device.daq, args.device.baud)?;
    let info = device.info()?;
    anyhow::ensure!(
        info.protocol == plant_trace_proto::PROTOCOL_VERSION,
        "firmware speaks protocol v{}, this build speaks v{}",
        info.protocol,
        plant_trace_proto::PROTOCOL_VERSION
    );

    if args.calibrate {
        println!("calibrating the SAADC offset…");
        device.calibrate()?;
    }

    let fs_hz = device.start(args.fs)?;
    let meta = RunMeta {
        source: args.device.daq.clone(),
        fs_hz,
        adc: info.adc_scale(),
        oversample: info.oversample,
        firmware: info.firmware.clone(),
        note: args.note.clone(),
        origin_sample: 0,
    };
    let mut writer = RunWriter::create(&args.out, &meta)?;
    println!(
        "recording {:.1} s at {} Hz to {}",
        args.duration,
        fs_hz,
        args.out.display()
    );

    let wanted_rows = (args.duration * fs_hz as f64) as u64;
    let scale = info.adc_scale();
    let started = Instant::now();
    let mut last_report = started;
    let mut rows = 0u64;
    let mut firmware_dropped = 0u32;

    while rows < wanted_rows {
        match device.next_event(Duration::from_secs(2))? {
            Some(Event::Block {
                seq,
                n,
                channels,
                dropped,
                counts,
            }) => {
                writer.push_block(seq, n, channels, &counts)?;
                rows += n as u64;
                firmware_dropped = dropped;

                // A live line matters on the bench: it is how you notice that
                // a channel is railed or unplugged before spending ten minutes
                // recording it.
                if last_report.elapsed() >= Duration::from_millis(500) {
                    last_report = Instant::now();
                    let last = &counts[counts.len() - channels as usize..];
                    let elapsed = started.elapsed().as_secs_f64();
                    print!(
                        "\r  {:6.1} s  {:5.0} rows/s  u_t {:+.4} V  p_s {:+.4} V  p_e {:+.4} V  dropped {}",
                        rows as f64 / fs_hz as f64,
                        rows as f64 / elapsed.max(1e-3),
                        scale.to_volts(last[0]),
                        scale.to_volts(last[1]),
                        scale.to_volts(last[2]),
                        dropped,
                    );
                    let _ = std::io::Write::flush(&mut std::io::stdout());
                }
            }
            Some(other) => eprintln!("unexpected message while recording: {other:?}"),
            None => anyhow::bail!("the rig went quiet for 2 s — check the link"),
        }
    }

    let (blocks, dropped) = device.stop()?;
    let stats = writer.finish()?;

    println!();
    println!(
        "{} rows in {} blocks; {} lost in transit, {} dropped by the firmware",
        stats.rows,
        stats.blocks,
        stats.gaps,
        dropped.max(firmware_dropped)
    );
    let _ = blocks;
    for (i, ch) in Channel::ALL.iter().enumerate() {
        let s = stats.channels[i];
        println!(
            "  {:>3}  min {:+.4} V  mean {:+.4} V  max {:+.4} V",
            ch.name(),
            s.min_v,
            s.mean_v,
            s.max_v
        );
    }
    Ok(())
}

fn simulate(args: SimulateArgs) -> Result<()> {
    sim::run(sim::Options {
        addr: args.addr,
        fs_hz: args.fs,
        speed: args.speed,
        settle_s: args.settle,
        plant: match args.plant {
            SimPlantArg::Model => sim::SimPlant::Model,
            SimPlantArg::WireU => sim::SimPlant::Wire { from: 0 },
            SimPlantArg::WireP => sim::SimPlant::Wire { from: 1 },
        },
        adc: AdcScale::NOMINAL,
        ..Default::default()
    })
}
