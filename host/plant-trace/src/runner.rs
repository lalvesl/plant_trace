//! Running an experiment against the rig.
//!
//! The DAQ streams for the whole experiment and the host segments the stream,
//! rather than starting and stopping acquisition per step. Two reasons: the
//! sample index stays a single monotonic clock across the run, so the manifest
//! can say exactly where each step begins; and a start/stop per step would put
//! an unrecorded gap exactly where the interesting transient is.
//!
//! The work is done by [`run_on`], on a link the caller already has open, and
//! everything it has to say goes through a [`RunObserver`]. The CLI's
//! [`run`] is that plus a console observer; the GUI is the same plus a plot.

use std::{
    collections::VecDeque,
    io::Write as _,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

use anyhow::{Context, Result};
use plant_trace_proto::{
    scale::{AdcScale, DacScale},
    Channel, N_CHANNELS,
};
use serde::{Deserialize, Serialize};

use crate::{
    csvout::{utc_now, RunMeta, RunWriter},
    daq::{self, Daq, Event},
    experiment::{Experiment, Step, WaveSpec},
};

/// Where and how to run an experiment.
pub struct RunConfig {
    /// Rig link spec.
    pub daq: String,
    /// Baud rate for a serial link.
    pub daq_baud: u32,
    /// Directory for the CSVs and the manifest; created if missing.
    pub out_dir: Option<PathBuf>,
}

/// What happened, written next to the data as `run.json`.
///
/// Deserialisable as well as serialisable because the analysis reads it back:
/// it is how `analyze` knows which excitation produced which CSV, and at what
/// frequency, without parsing file names.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Manifest {
    /// Experiment name.
    pub experiment: String,
    /// Experiment description.
    pub description: String,
    /// When the run started.
    pub started_utc: String,
    /// Effective sample rate.
    pub fs_hz: u32,
    /// Rig firmware version — one firmware now drives and measures.
    pub daq_firmware: String,
    /// Carrier the outputs were generated with, hertz.
    pub output_tick_hz: u32,
    /// ADC full scale, volts.
    pub adc_full_scale_v: f32,
    /// ADC resolution.
    pub adc_bits: u8,
    /// Output calibration in force.
    pub outputs: Vec<OutputRecord>,
    /// The steps, in order — only those that were reached when the run was
    /// cancelled.
    pub steps: Vec<StepRecord>,
    /// The run was stopped by [`RunObserver::cancelled`] before its last
    /// step finished. Absent from the file (and `false`) for a complete run.
    #[serde(default, skip_serializing_if = "is_false")]
    pub cancelled: bool,
}

/// Calibration of one output, as it was installed for this run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OutputRecord {
    /// `u_t` or `p_s`.
    pub channel: String,
    /// Volts per DAC code.
    pub volts_per_code: f32,
    /// Volts at code 0.
    pub offset_v: f32,
    /// Lowest level allowed.
    pub min_v: f32,
    /// Highest level allowed.
    pub max_v: f32,
}

/// What one step of the experiment did.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StepRecord {
    /// Step label.
    pub name: String,
    /// `settle` or `record`.
    pub kind: String,
    /// Index of the first sample of this step in the continuous stream.
    pub first_sample: u64,
    /// Samples the step covers.
    pub samples: u64,
    /// CSV written for this step, if any.
    pub csv: Option<String>,
    /// Whether a settle step reached steady state before its timeout.
    pub settled: Option<bool>,
    /// Level the step settled at, volts.
    pub settled_p_e_v: Option<f32>,
    /// Excitation applied to the valve command, as written in the experiment.
    pub u_t: Option<WaveSpec>,
    /// Excitation applied to the steam pressure.
    pub p_s: Option<WaveSpec>,
    /// The step was cut short by a cancel: its CSV holds only what was
    /// recorded up to then, and the analysis skips it. Absent from the file
    /// (and `false`) for a step that ran to its end.
    #[serde(default, skip_serializing_if = "is_false")]
    pub interrupted: bool,
}

fn is_false(b: &bool) -> bool {
    !*b
}

/// What the rig said about itself when a run started, handed to
/// [`RunObserver::started`].
#[derive(Debug, Clone)]
pub struct RunStart {
    /// Experiment name.
    pub experiment: String,
    /// Firmware version string.
    pub firmware: String,
    /// Number of analog outputs.
    pub outputs: u8,
    /// Output resolution, bits.
    pub output_bits: u8,
    /// Waveform tick the firmware reports, hertz.
    pub tick_hz: u32,
    /// Effective sample rate of the stream, hertz.
    pub fs_hz: u32,
    /// Where the CSVs and `run.json` are being written.
    pub out_dir: PathBuf,
}

/// Everything [`run_on`] reports while it runs.
///
/// Every method has an empty default, so an observer implements only what it
/// shows. The calls come from the thread running [`run_on`], in stream order;
/// they should return quickly — a slow observer holds the link, and the
/// rig's blocks pile up behind it.
pub trait RunObserver {
    /// The stream has started; nothing has been driven yet.
    fn started(&mut self, _start: &RunStart) {}

    /// Step `index` of the experiment is about to be applied.
    fn step_started(&mut self, _index: usize, _step: &Step) {}

    /// Samples as they arrive, in volts at the plant, one row per sample in
    /// wire order `[u_t, p_s, p_e]`. `first_sample` is the index of the
    /// first row in the continuous stream (the clock of
    /// [`StepRecord::first_sample`]), so time since the start of the run is
    /// `(first_sample + i) / fs_hz`. Every sample of the stream is reported
    /// exactly once, whichever step it falls in; a block lost on the link
    /// shows up as a jump in `first_sample`.
    fn samples(&mut self, _first_sample: u64, _fs_hz: u32, _rows: &[[f32; 3]]) {}

    /// Transient progress text — a settle's drift, a recording's elapsed
    /// time. Each line replaces the previous one; roughly two a second.
    fn status(&mut self, _line: &str) {}

    /// A line worth keeping: how a step ended, where its data went.
    fn log(&mut self, _line: &str) {}

    /// Step `index` is over; `record` is what goes into the manifest.
    fn step_finished(&mut self, _index: usize, _record: &StepRecord) {}

    /// Polled once per block (every 32–64 ms at the usual rates) and before
    /// each step. Returning `true` ends the run cleanly: the step in progress
    /// is cut short (its CSV is finished with what it has, and its record is
    /// marked [`StepRecord::interrupted`]), the generator is frozen and the
    /// stream stopped, the manifest is written with
    /// [`Manifest::cancelled`] set, and [`run_on`] returns `Ok` with it — a
    /// cancel is not an error.
    fn cancelled(&self) -> bool {
        false
    }
}

/// The observer that wants to know nothing.
impl RunObserver for () {}

/// Directory a run lands in when the caller does not name one:
/// `data/<name>-<UTC stamp>`.
pub fn default_out_dir(name: &str) -> PathBuf {
    PathBuf::from("data").join(format!(
        "{}-{}",
        name,
        utc_now().replace(['-', ':'], "").replace('Z', "")
    ))
}

/// Run `experiment` and return the directory the results landed in.
///
/// Opens the link, runs [`run_on`] with progress on the terminal, closes it.
pub fn run(experiment: &Experiment, config: &RunConfig) -> Result<PathBuf> {
    let out_dir = config
        .out_dir
        .clone()
        .unwrap_or_else(|| default_out_dir(&experiment.name));
    std::fs::create_dir_all(&out_dir).with_context(|| format!("creating {}", out_dir.display()))?;

    let mut daq = Daq::open(&config.daq, config.daq_baud)?;
    let mut console = Console {
        daq: config.daq.clone(),
    };
    run_on(&mut daq, experiment, &out_dir, &mut console)?;

    println!("\nmanifest    {}", out_dir.join("run.json").display());
    Ok(out_dir)
}

/// Run `experiment` on an already open link, writing the step CSVs and
/// `run.json` into `out_dir` (created if missing), and return the manifest.
///
/// Installs the experiment's output calibration, starts the stream at the
/// experiment's rate, applies the steps in order, and always — on success,
/// on error and on cancel — freezes the generator and stops the stream
/// before returning. The outputs are left where the last step left them, not
/// parked: a plant taken off its operating point by surprise is a transient
/// nobody asked for. The link stays open and usable.
///
/// A cancel (see [`RunObserver::cancelled`]) returns `Ok` with
/// [`Manifest::cancelled`] set; errors are link or file failures.
pub fn run_on(
    daq: &mut Daq,
    experiment: &Experiment,
    out_dir: &Path,
    obs: &mut dyn RunObserver,
) -> Result<Manifest> {
    let started = utc_now();
    std::fs::create_dir_all(out_dir).with_context(|| format!("creating {}", out_dir.display()))?;

    // ── connect ─────────────────────────────────────────────────────────────
    let daq_info = daq.info()?;
    anyhow::ensure!(
        daq_info.protocol == plant_trace_proto::PROTOCOL_VERSION,
        "the rig speaks protocol v{}, this build speaks v{}",
        daq_info.protocol,
        plant_trace_proto::PROTOCOL_VERSION
    );
    let gen_info = daq.gen_info()?;

    // ── calibration ─────────────────────────────────────────────────────────
    let scales: [(&str, DacScale); 2] = [
        ("u_t", experiment.scale("u_t")),
        ("p_s", experiment.scale("p_s")),
    ];
    for (i, (name, scale)) in scales.iter().enumerate() {
        daq.set_scale(i as u8, *scale)
            .with_context(|| format!("installing the {name} calibration"))?;
    }

    let fs_hz = daq.start(experiment.fs_hz)?;
    let adc = daq_info.adc_scale();
    obs.started(&RunStart {
        experiment: experiment.name.clone(),
        firmware: daq_info.firmware.clone(),
        outputs: gen_info.outputs,
        output_bits: gen_info.bits,
        tick_hz: gen_info.tick_hz,
        fs_hz,
        out_dir: out_dir.to_path_buf(),
    });

    let mut stream = Stream::new(fs_hz, adc);
    let mut steps = Vec::new();
    let mut cancelled = false;

    let result = (|| -> Result<()> {
        for (index, step) in experiment.steps.iter().enumerate() {
            if obs.cancelled() {
                cancelled = true;
                break;
            }
            obs.step_started(index, step);
            let record = match step {
                Step::Settle {
                    name,
                    u_t,
                    p_s,
                    timeout_s,
                    tol_v,
                    window_s,
                } => {
                    daq.set_level(0, *u_t)?;
                    daq.set_level(1, *p_s)?;
                    settle(daq, &mut stream, obs, name, *timeout_s, *tol_v, *window_s)?
                }
                Step::Record {
                    name,
                    duration_s,
                    u_t,
                    p_s,
                } => {
                    for (ch, spec) in [(0u8, u_t), (1u8, p_s)] {
                        if let Some(spec) = spec {
                            daq.program(ch, spec.to_wire())?;
                        }
                    }
                    capture(
                        daq,
                        &mut stream,
                        obs,
                        experiment,
                        &daq_info,
                        out_dir,
                        name,
                        *duration_s,
                        *u_t,
                        *p_s,
                    )?
                }
            };
            obs.step_finished(index, &record);
            let interrupted = record.interrupted;
            steps.push(record);
            if interrupted {
                cancelled = true;
                break;
            }
        }
        Ok(())
    })();

    // Always stop the stream and freeze the outputs, even if a step failed: a
    // rig left driving a waveform into a plant is not how a run should end.
    let _ = daq.gen_stop();
    let _ = daq.stop();
    result?;

    let manifest = Manifest {
        experiment: experiment.name.clone(),
        description: experiment.description.clone(),
        started_utc: started,
        fs_hz,
        daq_firmware: daq_info.firmware.clone(),
        output_tick_hz: gen_info.tick_hz,
        adc_full_scale_v: adc.full_scale_v,
        adc_bits: adc.bits,
        outputs: scales
            .iter()
            .map(|(name, s)| OutputRecord {
                channel: (*name).to_string(),
                volts_per_code: s.volts_per_code,
                offset_v: s.offset_v,
                min_v: s.min_v,
                max_v: s.max_v,
            })
            .collect(),
        steps,
        cancelled,
    };
    let path = out_dir.join("run.json");
    std::fs::write(&path, serde_json::to_string_pretty(&manifest)?)
        .with_context(|| format!("writing {}", path.display()))?;
    Ok(manifest)
}

/// The terminal's view of a run: exactly what `plant-trace run` has always
/// printed.
struct Console {
    /// Link spec, for the header.
    daq: String,
}

impl RunObserver for Console {
    fn started(&mut self, s: &RunStart) {
        println!("experiment  {}", s.experiment);
        println!("rig         {} ({})", self.daq, s.firmware);
        println!(
            "outputs     {} × {} bit at {} Hz",
            s.outputs, s.output_bits, s.tick_hz
        );
        println!("output      {}", s.out_dir.display());
    }

    fn step_started(&mut self, _index: usize, step: &Step) {
        match step {
            Step::Settle {
                name,
                u_t,
                p_s,
                timeout_s,
                ..
            } => println!(
                "\n▸ {name}: settling at u_T = {u_t:.3} V, p_s = {p_s:.3} V (≤ {timeout_s:.0} s)"
            ),
            Step::Record {
                name, duration_s, ..
            } => println!("\n▸ {name}: recording {duration_s:.0} s"),
        }
    }

    fn status(&mut self, line: &str) {
        print!("\r{line}");
        let _ = std::io::stdout().flush();
    }

    fn log(&mut self, line: &str) {
        println!("\r{line}");
    }
}

/// The continuous sample stream, and where it has got to.
///
/// Shared with [`crate::bode`], which segments one stream the same way.
pub(crate) struct Stream {
    pub(crate) fs_hz: u32,
    /// Counts→volts for the rows handed to observers.
    pub(crate) adc: AdcScale,
    /// Samples consumed since `Start` — the experiment's clock.
    pub(crate) position: u64,
    /// Blocks that never arrived.
    pub(crate) gaps: u64,
}

impl Stream {
    pub(crate) fn new(fs_hz: u32, adc: AdcScale) -> Self {
        Self {
            fs_hz,
            adc,
            position: 0,
            gaps: 0,
        }
    }
}

/// One block, already converted to the values this module works in.
pub(crate) struct Block {
    pub(crate) n: u16,
    pub(crate) channels: u8,
    pub(crate) counts: Vec<i16>,
    pub(crate) seq: u32,
    /// Stream index of the block's first sample.
    pub(crate) first_sample: u64,
    /// The block in volts, one `[u_t, p_s, p_e]` row per sample.
    pub(crate) rows: Vec<[f32; 3]>,
}

/// Pull the next block, keeping the stream's position in step with the block
/// sequence so a dropped block advances time instead of compressing it.
pub(crate) fn next_block(daq: &mut Daq, stream: &mut Stream) -> Result<Block> {
    loop {
        match daq.next_event(Duration::from_secs(3))? {
            Some(Event::Block {
                seq,
                n,
                channels,
                counts,
                ..
            }) => {
                let first = seq as u64 * n as u64;
                if first > stream.position {
                    stream.gaps += (first - stream.position) / n.max(1) as u64;
                }
                stream.position = first + n as u64;
                let rows = to_rows(&stream.adc, channels, &counts);
                return Ok(Block {
                    n,
                    channels,
                    counts,
                    seq,
                    first_sample: first,
                    rows,
                });
            }
            Some(other) => eprintln!("unexpected message mid-run: {other:?}"),
            None => anyhow::bail!("the rig went quiet for 3 s"),
        }
    }
}

/// Interleaved counts → rows of volts in wire order. A frame with fewer than
/// three channels leaves the missing ones at zero rather than failing: the
/// CSV writer is where a wrong channel count is refused.
fn to_rows(adc: &AdcScale, channels: u8, counts: &[i16]) -> Vec<[f32; 3]> {
    let ch = channels as usize;
    if ch == 0 {
        return Vec::new();
    }
    counts
        .chunks_exact(ch)
        .map(|frame| {
            let mut row = [0.0f32; N_CHANNELS];
            for (v, c) in row.iter_mut().zip(frame) {
                *v = adc.to_volts(*c);
            }
            row
        })
        .collect()
}

/// Wait until `P_e` stops moving, or the timeout expires.
#[allow(clippy::too_many_arguments)]
fn settle(
    daq: &mut Daq,
    stream: &mut Stream,
    obs: &mut dyn RunObserver,
    name: &str,
    timeout_s: f32,
    tol_v: f32,
    window_s: f32,
) -> Result<StepRecord> {
    let first_sample = stream.position;
    let mut detector = Steady::new((window_s * stream.fs_hz as f32) as usize, tol_v);
    let deadline = Instant::now() + Duration::from_secs_f32(timeout_s);
    let mut last_print = Instant::now();
    let mut settled = false;
    let mut interrupted = false;
    let mut last_v = 0.0;

    while Instant::now() < deadline {
        let block = next_block(daq, stream)?;
        obs.samples(block.first_sample, stream.fs_hz, &block.rows);
        for row in &block.rows {
            let v = row[Channel::ElectricalPower as usize];
            last_v = v;
            if detector.push(v) {
                settled = true;
            }
        }
        if settled {
            break;
        }
        if obs.cancelled() {
            interrupted = true;
            break;
        }
        if last_print.elapsed() >= Duration::from_millis(500) {
            last_print = Instant::now();
            obs.status(&format!(
                "  P_e {:+.4} V  drift {:+.4} V  ripple {:.4} V      ",
                last_v,
                detector.drift(),
                detector.ripple()
            ));
        }
    }
    obs.log(&format!(
        "  {} at P_e = {:+.4} V after {:.1} s{}",
        if settled {
            "settled"
        } else if interrupted {
            "CANCELLED"
        } else {
            "GAVE UP"
        },
        last_v,
        (stream.position - first_sample) as f32 / stream.fs_hz as f32,
        if settled || interrupted {
            String::new()
        } else {
            format!(
                " — still drifting {:+.4} V over {:.0} s",
                detector.drift(),
                window_s
            )
        }
    ));

    Ok(StepRecord {
        name: name.to_string(),
        kind: "settle".to_string(),
        first_sample,
        samples: stream.position - first_sample,
        csv: None,
        settled: Some(settled),
        settled_p_e_v: Some(last_v),
        u_t: None,
        p_s: None,
        interrupted,
    })
}

/// Start the staged excitation and record it.
#[allow(clippy::too_many_arguments)]
fn capture(
    daq: &mut Daq,
    stream: &mut Stream,
    obs: &mut dyn RunObserver,
    experiment: &Experiment,
    info: &daq::Info,
    out_dir: &Path,
    name: &str,
    duration_s: f32,
    u_t: Option<WaveSpec>,
    p_s: Option<WaveSpec>,
) -> Result<StepRecord> {
    let file = out_dir.join(format!("{name}.csv"));
    let first_sample = stream.position;
    let mut writer = RunWriter::create(
        &file,
        &RunMeta {
            source: format!("{} / {}", experiment.name, name),
            fs_hz: stream.fs_hz,
            adc: info.adc_scale(),
            oversample: info.oversample,
            firmware: info.firmware.clone(),
            note: Some(format!("{} — step '{}'", experiment.name, name)),
            origin_sample: first_sample,
        },
    )?;

    // Started after the writer exists, so the first samples of the excitation
    // are in the file. Blocks that arrive while the acknowledgement is in
    // flight are queued by the session, not dropped — the leading edge of the
    // excitation is the last thing a recording can afford to lose.
    daq.gen_start()?;

    let wanted = (duration_s * stream.fs_hz as f32) as u64;
    let mut last_print = Instant::now();
    let mut interrupted = false;
    while stream.position - first_sample < wanted {
        let block = next_block(daq, stream)?;
        writer.push_block(block.seq, block.n, block.channels, &block.counts)?;
        obs.samples(block.first_sample, stream.fs_hz, &block.rows);
        if obs.cancelled() {
            interrupted = true;
            break;
        }
        if last_print.elapsed() >= Duration::from_millis(500) {
            last_print = Instant::now();
            let last = block.rows.last().copied().unwrap_or_default();
            obs.status(&format!(
                "  {:6.1}/{:.0} s  u_t {:+.4} V  p_s {:+.4} V  p_e {:+.4} V   ",
                (stream.position - first_sample) as f32 / stream.fs_hz as f32,
                duration_s,
                last[0],
                last[1],
                last[2],
            ));
        }
    }
    daq.gen_stop()?;
    let stats = writer.finish()?;
    obs.log(&format!(
        "  {} rows to {}{}{}                    ",
        stats.rows,
        file.display(),
        if stats.gaps > 0 {
            format!(" ({} blocks lost)", stats.gaps)
        } else {
            String::new()
        },
        if interrupted { " — CANCELLED" } else { "" }
    ));

    Ok(StepRecord {
        name: name.to_string(),
        kind: "record".to_string(),
        first_sample,
        samples: stream.position - first_sample,
        csv: Some(format!("{name}.csv")),
        settled: None,
        settled_p_e_v: None,
        u_t,
        p_s,
        interrupted,
    })
}

/// Steady-state detector over a sliding window.
///
/// Two conditions, because they catch different things: the peak-to-peak
/// *ripple* catches an oscillation that is not decaying, and the difference
/// between the two halves' means catches a slow *drift* that a ripple test
/// would happily call settled.
struct Steady {
    window: VecDeque<f32>,
    capacity: usize,
    tol_v: f32,
}

impl Steady {
    fn new(capacity: usize, tol_v: f32) -> Self {
        Self {
            window: VecDeque::with_capacity(capacity.max(1)),
            capacity: capacity.max(1),
            tol_v,
        }
    }

    fn push(&mut self, v: f32) -> bool {
        if self.window.len() == self.capacity {
            self.window.pop_front();
        }
        self.window.push_back(v);
        self.window.len() == self.capacity
            && self.ripple() < self.tol_v
            && self.drift().abs() < self.tol_v / 2.0
    }

    fn ripple(&self) -> f32 {
        let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
        for v in &self.window {
            lo = lo.min(*v);
            hi = hi.max(*v);
        }
        if self.window.is_empty() {
            0.0
        } else {
            hi - lo
        }
    }

    fn drift(&self) -> f32 {
        if self.window.len() < 2 {
            return 0.0;
        }
        let half = self.window.len() / 2;
        let mean =
            |it: &mut dyn Iterator<Item = &f32>, n: usize| -> f32 { it.sum::<f32>() / n as f32 };
        let first = mean(&mut self.window.iter().take(half), half);
        let second = mean(&mut self.window.iter().skip(self.window.len() - half), half);
        second - first
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steady_waits_for_both_ripple_and_drift() {
        let mut d = Steady::new(100, 0.002);

        // A clean ramp has no ripple to speak of but is clearly not settled.
        for i in 0..200 {
            let settled = d.push(0.5 + i as f32 * 1e-4);
            assert!(!settled, "a ramp was called settled at sample {i}");
        }
        // Flat and quiet: settled once the window has refilled.
        let mut settled = false;
        for i in 0..200 {
            settled = d.push(0.5 + if i % 2 == 0 { 1e-5 } else { -1e-5 });
        }
        assert!(settled, "a flat trace was never called settled");

        // An oscillation inside the window is not settled, however centred.
        let mut d = Steady::new(100, 0.002);
        let mut any = false;
        for i in 0..400 {
            any |= d.push(0.5 + 0.01 * (i as f32 * 0.3).sin());
        }
        assert!(!any, "a 10 mV oscillation was called settled");
    }
}
