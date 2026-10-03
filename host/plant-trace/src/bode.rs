//! Automatic stepped-sine Bode: one continuous stream, one sine per
//! frequency, one least-squares fit per channel.
//!
//! What `experiments/freq-*.toml` does by hand — a record step per frequency,
//! analysed afterwards — this does in one call, with the point on screen the
//! moment its window closes. The measurement is the same one
//! [`crate::analysis::freq`] makes: [`sine_fit`] at a frequency known exactly,
//! on the *measured* input (the sense channel of the excited output, never
//! the commanded value) and on the response, and the ratio of the two.
//!
//! # How each point is timed
//!
//! Every frequency runs as a finite sine of a whole number of cycles
//! (`Waveform::Sine { cycles: N }`), so the firmware ends it on its centre
//! line and holds there: the next frequency starts from the operating point
//! instead of from wherever a cut-off sine happened to be, and the plant is
//! not kicked between points.
//!
//! The host does not know on which sample the firmware started the sine —
//! only that it was after every sample the host had already consumed (`p`)
//! and, allowing for blocks in flight, before `p + margin`
//! ([`BodePlan::margin_s`]). The fit window is placed so that it is inside
//! the sine and past the settle span for *any* start in that interval: it
//! ends at `p + run − margin` and is `measure` long; the sine is made long
//! enough (`run ≥ settle + measure + 2·margin`, rounded up to whole cycles)
//! for that to hold. After the window the host waits until `p + run +
//! margin`, when the sine is certainly over, before starting the next one.
//!
//! # Phase, skew and sign
//!
//! Phases are in degrees, **negative = the response lags the input**, the
//! convention of [`crate::analysis::freq::BodePoint`].
//!
//! The SAADC converts the channels one after another
//! ([`plant_trace_proto::SCAN_CHANNEL_SPACING_S`]), so the response and the
//! input of one row are not simultaneous. A channel converted `Δt` *later*
//! but filed on the same row reads as a **lead** of `360·f·Δt` degrees — for
//! `P_e` against `u_T`, `Δt = 2 × 89.5 µs` and +6.4° at 100 Hz. That lead is
//! [`BodePoint::skew_deg`] (positive when the response channel is converted
//! after the input channel), and the corrected phase is the raw one minus it:
//!
//! ```text
//! phase_deg = raw_phase_deg − skew_deg,   skew_deg = +360·f·(t_response − t_input)
//! ```
//!
//! On a wire loopback (`P_e` wired to the `u_T` filter output) the true phase
//! is 0, so the raw phase *is* the skew and the corrected one should sit at
//! 0° ± noise at every frequency.

use std::{
    collections::BTreeMap,
    io::Write as _,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

use anyhow::{bail, ensure, Context, Result};
use plant_trace_proto::{scale::DacScale, waveform::Waveform, Channel, SCAN_CHANNEL_SPACING_S};
use serde::{Deserialize, Serialize};

use crate::{
    analysis::freq::sine_fit,
    csvout::{utc_now, RunMeta, RunWriter},
    daq::Daq,
    experiment::OutputScale,
    runner::{next_block, Stream},
};

/// Samples per block the margin is sized for — the firmware's and the
/// simulator's block length.
const NOMINAL_BLOCK_SAMPLES: f64 = 64.0;

// ── what to excite, what to measure ─────────────────────────────────────────

/// The output a Bode plan drives with its sine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum ExcitedOutput {
    /// Valve command, output 0.
    #[default]
    #[serde(rename = "u_t")]
    ValveCmd,
    /// Steam pressure, output 1.
    #[serde(rename = "p_s")]
    SteamPressure,
}

impl ExcitedOutput {
    /// Output index on the wire (0 = `u_T`, 1 = `p_s`) — also the index of its
    /// sense channel in a row.
    pub fn index(self) -> u8 {
        match self {
            ExcitedOutput::ValveCmd => 0,
            ExcitedOutput::SteamPressure => 1,
        }
    }

    /// `u_t` or `p_s`, the name used in files.
    pub fn name(self) -> &'static str {
        match self {
            ExcitedOutput::ValveCmd => "u_t",
            ExcitedOutput::SteamPressure => "p_s",
        }
    }

    /// The other output — the one held at [`BodePlan::hold_v`].
    pub fn other(self) -> Self {
        match self {
            ExcitedOutput::ValveCmd => ExcitedOutput::SteamPressure,
            ExcitedOutput::SteamPressure => ExcitedOutput::ValveCmd,
        }
    }
}

/// The acquired channel a Bode plan reads as the response.
///
/// Any of the three: `P_e` for the plant, one of the sense channels to
/// characterise a loopback or the output chain itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum ResponseChannel {
    /// `u_T` sense channel, 0.
    #[serde(rename = "u_t")]
    ValveCmd,
    /// `p_s` sense channel, 1.
    #[serde(rename = "p_s")]
    SteamPressure,
    /// `P_e`, the plant output, 2.
    #[default]
    #[serde(rename = "p_e")]
    ElectricalPower,
}

impl ResponseChannel {
    /// The proto channel this is.
    pub fn channel(self) -> Channel {
        match self {
            ResponseChannel::ValveCmd => Channel::ValveCmd,
            ResponseChannel::SteamPressure => Channel::SteamPressure,
            ResponseChannel::ElectricalPower => Channel::ElectricalPower,
        }
    }

    /// Index in a row, wire order.
    pub fn index(self) -> usize {
        self.channel() as usize
    }

    /// `u_t`, `p_s` or `p_e`.
    pub fn name(self) -> &'static str {
        self.channel().name()
    }
}

// ── frequency grids ─────────────────────────────────────────────────────────

/// `n` frequencies from `f0` to `f1` inclusive, evenly spaced on a log axis.
///
/// Works in either direction (`f1 < f0` gives a descending grid). Empty when
/// `n == 0` or either end is not a positive finite number; `[f0]` when
/// `n == 1`.
pub fn log_space(f0: f64, f1: f64, n: usize) -> Vec<f64> {
    if n == 0 || !(f0.is_finite() && f1.is_finite() && f0 > 0.0 && f1 > 0.0) {
        return Vec::new();
    }
    if n == 1 {
        return vec![f0];
    }
    let (l0, l1) = (f0.ln(), f1.ln());
    (0..n)
        .map(|i| {
            if i == n - 1 {
                // Exactly the end asked for, not whatever exp(ln) rounds to.
                f1
            } else {
                (l0 + (l1 - l0) * i as f64 / (n - 1) as f64).exp()
            }
        })
        .collect()
}

/// A log grid from `f0` to `f1` inclusive with at least `points_per_decade`
/// points in every decade: `ceil(decades × points_per_decade) + 1` points.
pub fn per_decade(f0: f64, f1: f64, points_per_decade: f64) -> Vec<f64> {
    let usable = |x: f64| x.is_finite() && x > 0.0;
    if !(usable(f0) && usable(f1) && usable(points_per_decade)) {
        return Vec::new();
    }
    let decades = (f1 / f0).log10().abs();
    // The epsilon keeps an exact whole number of decades from gaining a point.
    let n = (decades * points_per_decade - 1e-9).ceil().max(0.0) as usize + 1;
    log_space(f0, f1, n)
}

// ── the plan ────────────────────────────────────────────────────────────────

/// Everything a stepped-sine Bode run needs, as it is written in a TOML file.
///
/// Every field has a default ([`BodePlan::default`], the valve-to-power
/// response at the nominal operating point), so a file only has to say what
/// differs. Unknown keys are refused — a typo in a plan should not silently
/// become a default.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BodePlan {
    /// Short name; used for the output directory.
    pub name: String,
    /// What the run is for — copied into `bode.json`.
    pub description: String,
    /// Output driven with the sine.
    pub excite: ExcitedOutput,
    /// Channel read as the response. The input is always the sense channel
    /// of [`Self::excite`].
    pub response: ResponseChannel,
    /// Operating point the sine rides on, volts at the plant input.
    pub center_v: f32,
    /// Peak amplitude of the sine, volts. Small enough to stay in the local
    /// linearisation (the assignment asks for ±5 % of span), large enough to
    /// sit well above the 0.3 mV ADC count.
    pub amplitude_v: f32,
    /// Level the *other* output is held at throughout, volts.
    pub hold_v: f32,
    /// Frequencies, hertz, measured in this order. Phase is unwrapped along
    /// this order, so keep it monotonic (see [`log_space`], [`per_decade`]).
    pub frequencies_hz: Vec<f64>,
    /// Sample rate to ask the rig for, hertz. At least four samples per
    /// period of the highest frequency, as for `check sine`.
    pub fs_hz: u32,
    /// Time spent at the operating point before the first sine, seconds, so
    /// the sweep starts from steady state.
    pub initial_settle_s: f64,
    /// Settle span discarded before each window, in periods of the frequency…
    pub settle_cycles: f64,
    /// …but never less than this, seconds — the plant's own transients (the
    /// reheater's 7 s) do not shrink with the period…
    pub settle_min_s: f64,
    /// …and never more than this, seconds, so the slowest points do not spend
    /// minutes settling to a precision the fit's drift term makes moot.
    pub settle_max_s: f64,
    /// Periods to fit at each frequency (rounded up to whole periods)…
    pub measure_cycles: f64,
    /// …but at least this long, seconds, so fast points average over more
    /// periods and more noise…
    pub measure_min_s: f64,
    /// …and at most this long, seconds, except that a window is never less
    /// than one whole period. This cap is what keeps a sweep down to 0.02 Hz
    /// at minutes rather than an hour.
    pub measure_max_s: f64,
    /// Time between the conversions of consecutive channels in one scan,
    /// seconds — the skew the correction removes. Defaults to the nominal
    /// [`SCAN_CHANNEL_SPACING_S`]; a wire loopback measures the real one
    /// ([`BodeResult::implied_scan_spacing_s`]).
    pub scan_spacing_s: f64,
    /// Subtract the scan skew from the phase ([`BodePoint::phase_deg`]). Off,
    /// `phase_deg` equals `raw_phase_deg`.
    pub correct_skew: bool,
    /// Highest frequency the plan may contain, hertz. 15 Hz by default — the
    /// limit the plant is tested to ([`crate::experiment::PLANT_MAX_FREQ_HZ`]).
    /// Only a plan with no plant in the loop (the wire loopback) raises it.
    pub max_freq_hz: f64,
    /// Bench calibration per output, keyed `u_t` and `p_s`, exactly as in an
    /// experiment file. An output left out gets the nominal map.
    pub outputs: BTreeMap<String, OutputScale>,
}

impl Default for BodePlan {
    fn default() -> Self {
        let scale = |s: DacScale| OutputScale {
            volts_per_code: s.volts_per_code,
            offset_v: s.offset_v,
            min_v: s.min_v,
            max_v: s.max_v,
        };
        Self {
            name: "bode-u".into(),
            description: "Stepped-sine frequency response of P_e to u_T at the nominal \
                          operating point"
                .into(),
            excite: ExcitedOutput::ValveCmd,
            response: ResponseChannel::ElectricalPower,
            // The middle of the u_T window, ±5 % of its 0.5 V span.
            center_v: 2.50,
            amplitude_v: 0.025,
            hold_v: 0.80,
            // 0.02 Hz is the reheater's corner region, 5 Hz well past the
            // 1.4 Hz rotor mode: 13 points, 5 per decade.
            frequencies_hz: per_decade(0.02, 5.0, 5.0),
            fs_hz: 1000,
            initial_settle_s: 60.0,
            settle_cycles: 2.0,
            settle_min_s: 5.0,
            settle_max_s: 30.0,
            measure_cycles: 4.0,
            measure_min_s: 10.0,
            measure_max_s: 100.0,
            scan_spacing_s: SCAN_CHANNEL_SPACING_S,
            correct_skew: true,
            max_freq_hz: crate::experiment::PLANT_MAX_FREQ_HZ as f64,
            outputs: [
                ("u_t".to_string(), scale(DacScale::U_T_NOMINAL)),
                ("p_s".to_string(), scale(DacScale::NOMINAL)),
            ]
            .into_iter()
            .collect(),
        }
    }
}

/// When one frequency of a plan happens, as [`BodePlan::schedule`] lays it out.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointSchedule {
    /// Position in [`BodePlan::frequencies_hz`].
    pub index: usize,
    /// Frequency, hertz.
    pub freq_hz: f64,
    /// Whole periods the sine runs for — what is programmed.
    pub cycles: u32,
    /// How long the sine runs, seconds (`cycles / freq_hz`).
    pub run_s: f64,
    /// Settle span the plan asks for, seconds. The span actually discarded is
    /// at least this long, whatever the start latency.
    pub settle_s: f64,
    /// Whole periods in the fit window.
    pub measure_cycles: u32,
    /// Length of the fit window, seconds.
    pub measure_s: f64,
    /// Time this point takes on the stream, seconds: the run plus the wait
    /// for it to be certainly over.
    pub total_s: f64,
}

impl BodePlan {
    /// Read and validate a plan file.
    pub fn load(path: &Path) -> Result<Self> {
        let text =
            std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        let plan = Self::from_toml(&text).with_context(|| format!("in {}", path.display()))?;
        Ok(plan)
    }

    /// Parse and validate a plan from TOML text.
    pub fn from_toml(text: &str) -> Result<Self> {
        let plan: BodePlan = toml::from_str(text).context("parsing the Bode plan")?;
        plan.validate()?;
        Ok(plan)
    }

    /// The plan as TOML text.
    pub fn to_toml(&self) -> Result<String> {
        toml::to_string_pretty(self).context("serialising the Bode plan")
    }

    /// Write the plan to `path` as TOML.
    pub fn save(&self, path: &Path) -> Result<()> {
        if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
        }
        std::fs::write(path, self.to_toml()?).with_context(|| format!("writing {}", path.display()))
    }

    /// Scaling for one output (`u_t` or `p_s`), or the nominal one if the plan
    /// does not say.
    pub fn scale(&self, output: &str) -> DacScale {
        self.outputs
            .get(output)
            .copied()
            .map(DacScale::from)
            .unwrap_or(crate::experiment::nominal_scale(output))
    }

    /// Refuse a plan that cannot be run, or would leave a safe window, before
    /// anything reaches the plant — with a message that says which field.
    pub fn validate(&self) -> Result<()> {
        if self.frequencies_hz.is_empty() {
            bail!("the plan has no frequencies");
        }
        for f in &self.frequencies_hz {
            if !(f.is_finite() && *f > 0.0) {
                bail!("frequency {f} Hz is not a positive number");
            }
        }
        let f_max = self.f_max();
        if f_max > self.max_freq_hz {
            bail!(
                "{f_max} Hz is above the {} Hz this plan allows (max_freq_hz); the plant \
                 is tested up to {} Hz",
                self.max_freq_hz,
                crate::experiment::PLANT_MAX_FREQ_HZ
            );
        }
        if self.fs_hz == 0 {
            bail!("fs_hz must be given explicitly (the rig's default is not known in advance)");
        }
        if (self.fs_hz as f64) < 4.0 * f_max {
            bail!(
                "{} Hz sampling is too slow to fit a {f_max} Hz sinusoid — ask for at least \
                 {:.0} Hz",
                self.fs_hz,
                (4.0 * f_max).ceil()
            );
        }
        if !(self.amplitude_v.is_finite() && self.amplitude_v > 0.0) {
            bail!("amplitude_v must be positive, got {}", self.amplitude_v);
        }
        if !self.center_v.is_finite() || !self.hold_v.is_finite() {
            bail!("center_v and hold_v must be numbers");
        }

        for name in crate::experiment::OUTPUT_KEYS {
            crate::experiment::check_scale(name, self.scale(name))?;
        }
        let (driven, held) = (self.excite, self.excite.other());
        let scale = self.scale(driven.name());
        let (lo, hi) = (
            self.center_v - self.amplitude_v,
            self.center_v + self.amplitude_v,
        );
        if lo < scale.min_v - 1e-6 || hi > scale.max_v + 1e-6 {
            bail!(
                "the sine on {} swings {lo:.3}…{hi:.3} V, outside its window {:.3}…{:.3} V",
                driven.name(),
                scale.min_v,
                scale.max_v
            );
        }
        let other = self.scale(held.name());
        if self.hold_v < other.min_v - 1e-6 || self.hold_v > other.max_v + 1e-6 {
            bail!(
                "hold_v = {:.3} V on {} is outside its window {:.3}…{:.3} V",
                self.hold_v,
                held.name(),
                other.min_v,
                other.max_v
            );
        }

        for (name, v) in [
            ("initial_settle_s", self.initial_settle_s),
            ("settle_cycles", self.settle_cycles),
            ("settle_min_s", self.settle_min_s),
            ("settle_max_s", self.settle_max_s),
            ("measure_min_s", self.measure_min_s),
        ] {
            if !(v.is_finite() && v >= 0.0) {
                bail!("{name} must be zero or more, got {v}");
            }
        }
        if !(self.measure_cycles.is_finite() && self.measure_cycles >= 1.0) {
            bail!(
                "measure_cycles must be at least 1, got {}",
                self.measure_cycles
            );
        }
        if !(self.measure_max_s.is_finite() && self.measure_max_s > 0.0) {
            bail!("measure_max_s must be positive, got {}", self.measure_max_s);
        }
        if !(self.scan_spacing_s.is_finite() && (0.0..1e-3).contains(&self.scan_spacing_s)) {
            bail!(
                "scan_spacing_s = {} s is not a plausible scan spacing (0 … 1 ms)",
                self.scan_spacing_s
            );
        }
        if let Some(p) = self
            .schedule()
            .iter()
            .find(|p| p.cycles == u32::MAX || !p.run_s.is_finite())
        {
            bail!("{} Hz needs more periods than a sine can run", p.freq_hz);
        }
        Ok(())
    }

    /// Highest frequency of the plan, hertz (0 for an empty plan).
    pub fn f_max(&self) -> f64 {
        self.frequencies_hz.iter().copied().fold(0.0, f64::max)
    }

    /// How far a sine's first sample may be behind the host's position when
    /// it is started, seconds: six blocks in flight plus 100 ms of link and
    /// command latency. See the module docs for how it is used.
    pub fn margin_s(&self) -> f64 {
        6.0 * NOMINAL_BLOCK_SAMPLES / self.fs_hz.max(1) as f64 + 0.1
    }

    /// Timing of every point, in plan order.
    pub fn schedule(&self) -> Vec<PointSchedule> {
        let margin = self.margin_s();
        self.frequencies_hz
            .iter()
            .enumerate()
            .map(|(index, &f)| {
                let settle_s = (self.settle_cycles / f)
                    .min(self.settle_max_s)
                    .max(self.settle_min_s);
                let wanted = self.measure_cycles.max(self.measure_min_s * f).ceil();
                let cap = (self.measure_max_s * f).floor();
                let measure_cycles = wanted.min(cap).max(1.0);
                let measure_s = measure_cycles / f;
                let cycles = ((settle_s + measure_s + 2.0 * margin) * f)
                    .ceil()
                    .min(u32::MAX as f64);
                let run_s = cycles / f;
                PointSchedule {
                    index,
                    freq_hz: f,
                    cycles: cycles as u32,
                    run_s,
                    settle_s,
                    measure_cycles: measure_cycles as u32,
                    measure_s,
                    total_s: run_s + margin,
                }
            })
            .collect()
    }

    /// How long the whole sweep takes on the stream: the initial settle plus
    /// every point's [`PointSchedule::total_s`]. Command round trips add a few
    /// milliseconds per point on top.
    pub fn estimated_duration(&self) -> Duration {
        let s = self.initial_settle_s + self.schedule().iter().map(|p| p.total_s).sum::<f64>();
        Duration::from_secs_f64(s.max(0.0))
    }

    /// When, after the input channel, the response channel is converted in a
    /// scan, seconds — negative if before. This is the skew the correction
    /// removes.
    pub fn skew_s(&self) -> f64 {
        (self.response.index() as f64 - self.excite.index() as f64) * self.scan_spacing_s
    }
}

// ── results ─────────────────────────────────────────────────────────────────

/// One measured frequency.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodePoint {
    /// Position in [`BodePlan::frequencies_hz`].
    pub index: usize,
    /// Frequency, hertz.
    pub freq_hz: f64,
    /// |G| = response amplitude / input amplitude, V/V.
    pub gain: f64,
    /// 20·log₁₀|G|, dB.
    pub gain_db: f64,
    /// Phase to plot, degrees, negative = response lags input:
    /// `raw_phase_deg − skew_deg` when the plan corrects the skew,
    /// `raw_phase_deg` when it does not. Unwrapped along the sweep.
    pub phase_deg: f64,
    /// Phase as measured, without the scan-skew correction, degrees;
    /// unwrapped along the sweep (the first point is in −180…180).
    pub raw_phase_deg: f64,
    /// Apparent lead the scan order puts on the response at this frequency,
    /// degrees: `+360·f·(t_response − t_input)` — positive when the response
    /// channel is converted after the input channel (`P_e` vs `u_T`: +6.4°
    /// at 100 Hz). Computed whether or not the correction is applied.
    pub skew_deg: f64,
    /// Amplitude fitted on the input (sense) channel, volts.
    pub input_amplitude_v: f64,
    /// Amplitude fitted on the response channel, volts.
    pub response_amplitude_v: f64,
    /// Level the input sine rode on, volts.
    pub input_offset_v: f64,
    /// Level the response rode on, volts.
    pub response_offset_v: f64,
    /// Residual ratio of the input fit — see
    /// [`crate::analysis::freq::SineFit::residual_ratio`].
    pub input_residual: f64,
    /// Residual ratio of the response fit. Near 0 is a clean point; above
    /// ~0.3 the point is mostly noise, drift or distortion.
    pub response_residual: f64,
    /// Periods in the fit window.
    pub cycles: f64,
    /// Stream index of the window's first sample.
    pub first_sample: u64,
    /// Samples fitted (fewer than the window if blocks were lost).
    pub samples: usize,
}

/// Least-squares pure delay through a set of phases.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DelayEstimate {
    /// Delay, seconds, in the model `phase = −360·f·delay`: positive = the
    /// response lags. On a wire loopback read with the uncorrected phase it
    /// comes out **negative** — the later-converted channel reads early —
    /// at minus the scan skew.
    pub delay_s: f64,
    /// RMS of the phases around the fitted line, degrees. Large means the
    /// phase is not a straight line in frequency, i.e. not a pure delay.
    pub rms_residual_deg: f64,
    /// Points used.
    pub points: usize,
}

/// Fit `phase_deg = −360·f·τ` (a line through the origin) to the given
/// points and return `τ`.
///
/// Only meaningful when the true system has no phase of its own — a wire, or
/// a channel against itself — so that all the phase there is is delay.
/// `None` with no usable point (non-finite values are skipped).
pub fn estimate_delay(freq_hz: &[f64], phase_deg: &[f64]) -> Option<DelayEstimate> {
    let pts: Vec<(f64, f64)> = freq_hz
        .iter()
        .zip(phase_deg)
        .filter(|(f, p)| f.is_finite() && p.is_finite() && **f > 0.0)
        .map(|(f, p)| (*f, *p))
        .collect();
    let sff: f64 = pts.iter().map(|(f, _)| f * f).sum();
    if pts.is_empty() || sff <= 0.0 {
        return None;
    }
    let sfp: f64 = pts.iter().map(|(f, p)| f * p).sum();
    let slope = sfp / sff; // degrees per hertz
    let delay_s = -slope / 360.0;
    let rms = (pts
        .iter()
        .map(|(f, p)| (p - slope * f).powi(2))
        .sum::<f64>()
        / pts.len() as f64)
        .sqrt();
    Some(DelayEstimate {
        delay_s,
        rms_residual_deg: rms,
        points: pts.len(),
    })
}

/// A finished (or cancelled) sweep: `bode.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodeResult {
    /// The plan as it was run.
    pub plan: BodePlan,
    /// When the run started.
    pub started_utc: String,
    /// Effective sample rate.
    pub fs_hz: u32,
    /// Waveform tick the firmware reports, hertz.
    pub tick_hz: u32,
    /// Rig firmware version.
    pub firmware: String,
    /// [`BodePlan::skew_s`] as used for [`BodePoint::skew_deg`], seconds.
    pub skew_s: f64,
    /// The measured points, in plan order — all of them, or those finished
    /// before a cancel.
    pub points: Vec<BodePoint>,
    /// Pure delay fitted through the **uncorrected** phases
    /// ([`BodePoint::raw_phase_deg`]).
    ///
    /// Only meaningful when the true system has no phase of its own — the
    /// wire loopback, or a sense channel against another — where it measures
    /// the scan skew itself: about −179 µs for `P_e` against `u_T` (negative:
    /// the response reads early). On the plant it is a number without a
    /// meaning.
    pub delay: Option<DelayEstimate>,
    /// Scan spacing implied by [`Self::delay`] — `−delay / (k_response −
    /// k_input)`, seconds per channel slot — when the two channels differ.
    /// On a loopback this is the measured replacement for
    /// [`BodePlan::scan_spacing_s`] (89.5 µs as measured on this board).
    pub implied_scan_spacing_s: Option<f64>,
    /// The sweep was stopped by [`BodeObserver::cancelled`].
    pub cancelled: bool,
}

impl BodeResult {
    /// Write `bode.csv` and `bode.json` into `dir` (created if missing).
    pub fn save(&self, dir: &Path) -> Result<()> {
        std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
        let json = dir.join("bode.json");
        std::fs::write(&json, serde_json::to_string_pretty(self)?)
            .with_context(|| format!("writing {}", json.display()))?;
        let csv = dir.join("bode.csv");
        std::fs::write(&csv, self.to_csv()).with_context(|| format!("writing {}", csv.display()))
    }

    /// Read a `bode.json` back — the file, or the directory holding it.
    pub fn load(path: &Path) -> Result<Self> {
        let file = if path.is_dir() {
            path.join("bode.json")
        } else {
            path.to_path_buf()
        };
        let text = std::fs::read_to_string(&file)
            .with_context(|| format!("reading {}", file.display()))?;
        serde_json::from_str(&text).with_context(|| format!("parsing {}", file.display()))
    }

    /// The points as the text of `bode.csv`: a commented header, then one row
    /// per point. Units are in the column names (`_hz`, `_db`, `_deg`, `_v`;
    /// `gain` is V/V, residuals are ratios).
    pub fn to_csv(&self) -> String {
        let mut s = String::new();
        let p = &self.plan;
        s.push_str("# plant-trace stepped-sine Bode\n");
        s.push_str(&format!("# plan = {}\n", p.name));
        s.push_str(&format!("# started_utc = {}\n", self.started_utc));
        s.push_str(&format!("# firmware = {}\n", self.firmware));
        s.push_str(&format!(
            "# input = {} (sense), response = {}\n",
            p.excite.name(),
            p.response.name()
        ));
        s.push_str(&format!(
            "# center_v = {}, amplitude_v = {}, hold_v = {}\n",
            p.center_v, p.amplitude_v, p.hold_v
        ));
        s.push_str(&format!("# fs_hz = {}\n", self.fs_hz));
        s.push_str(&format!(
            "# skew_s = {:e} (phase_deg = raw_phase_deg - skew_deg: {})\n",
            self.skew_s,
            if p.correct_skew {
                "applied"
            } else {
                "not applied"
            }
        ));
        s.push_str("# phase sign: negative = response lags input\n");
        if let Some(d) = &self.delay {
            s.push_str(&format!(
                "# delay_s = {:e} (fitted on raw_phase_deg, rms {:.3} deg over {} points)\n",
                d.delay_s, d.rms_residual_deg, d.points
            ));
        }
        if self.cancelled {
            s.push_str("# cancelled = true\n");
        }
        s.push_str(
            "freq_hz,gain,gain_db,phase_deg,raw_phase_deg,skew_deg,input_amplitude_v,\
             response_amplitude_v,input_offset_v,response_offset_v,input_residual,\
             response_residual,cycles,first_sample,samples\n",
        );
        for q in &self.points {
            s.push_str(&format!(
                "{},{:.6},{:.4},{:.4},{:.4},{:.4},{:.6},{:.6},{:.6},{:.6},{:.5},{:.5},{},{},{}\n",
                q.freq_hz,
                q.gain,
                q.gain_db,
                q.phase_deg,
                q.raw_phase_deg,
                q.skew_deg,
                q.input_amplitude_v,
                q.response_amplitude_v,
                q.input_offset_v,
                q.response_offset_v,
                q.input_residual,
                q.response_residual,
                q.cycles,
                q.first_sample,
                q.samples
            ));
        }
        s
    }
}

// ── running it ──────────────────────────────────────────────────────────────

/// Everything [`run_bode`] reports while it runs. Every method has an empty
/// default; calls come from the thread running the sweep, in stream order,
/// and should return quickly.
pub trait BodeObserver {
    /// The stream has started at `fs_hz`; `schedule` is what will happen.
    fn started(&mut self, _fs_hz: u32, _schedule: &[PointSchedule]) {}

    /// Frequency `index` is about to be programmed.
    fn point_started(&mut self, _index: usize, _freq_hz: f64) {}

    /// Samples as they arrive — same contract as
    /// [`crate::runner::RunObserver::samples`]: volts, rows `[u_t, p_s, p_e]`,
    /// `first_sample` the stream index of the first row, every sample once.
    fn samples(&mut self, _first_sample: u64, _fs_hz: u32, _rows: &[[f32; 3]]) {}

    /// Transient progress text, about two lines a second.
    fn status(&mut self, _line: &str) {}

    /// A point has been measured.
    fn point_finished(&mut self, _point: &BodePoint) {}

    /// Polled once per block. Returning `true` ends the sweep cleanly: the
    /// point in progress is dropped, the rig is stopped and parked, what was
    /// measured is written, and [`run_bode`] returns `Ok` with
    /// [`BodeResult::cancelled`] set — a cancel is not an error.
    fn cancelled(&self) -> bool {
        false
    }
}

/// The observer that wants to know nothing.
impl BodeObserver for () {}

/// Where a sweep's files go when the caller does not say:
/// `data/<plan name>-<UTC stamp>`.
pub fn default_out_dir(plan: &BodePlan) -> PathBuf {
    crate::runner::default_out_dir(&plan.name)
}

/// Run a stepped-sine sweep on an open link.
///
/// Validates the plan, installs its output calibration, parks the excited
/// output at the operating point and the other at [`BodePlan::hold_v`],
/// streams for the initial settle, then measures every frequency in turn on
/// one continuous stream (see the module docs for the timing). Whatever
/// happens — success, error or cancel — the generator is stopped, both
/// outputs are parked and the stream is stopped before this returns; the link
/// stays open.
///
/// With `out_dir`, the raw stream goes to `stream.csv` there as it arrives
/// (the format of every other recording, so the existing tooling reads it)
/// and `bode.csv` / `bode.json` are written at the end — also after a cancel,
/// with the points that were finished. Without it nothing touches the disk.
pub fn run_bode(
    daq: &mut Daq,
    plan: &BodePlan,
    out_dir: Option<&Path>,
    obs: &mut dyn BodeObserver,
) -> Result<BodeResult> {
    plan.validate()?;
    let started_utc = utc_now();

    let info = daq.info()?;
    ensure!(
        info.protocol == plant_trace_proto::PROTOCOL_VERSION,
        "the rig speaks protocol v{}, this build speaks v{}",
        info.protocol,
        plant_trace_proto::PROTOCOL_VERSION
    );
    let gen_info = daq.gen_info()?;
    for name in ["u_t", "p_s"] {
        let ch = if name == "u_t" { 0 } else { 1 };
        daq.set_scale(ch, plan.scale(name))
            .with_context(|| format!("installing the {name} calibration"))?;
    }
    daq.set_level(plan.excite.index(), plan.center_v)?;
    daq.set_level(plan.excite.other().index(), plan.hold_v)?;

    let stream_csv = match out_dir {
        Some(dir) => {
            std::fs::create_dir_all(dir).with_context(|| format!("creating {}", dir.display()))?;
            Some(dir.join("stream.csv"))
        }
        None => None,
    };

    let fs_hz = daq.start(plan.fs_hz)?;
    let mut points: Vec<BodePoint> = Vec::new();
    let mut cancelled = false;

    let outcome = (|| -> Result<()> {
        ensure!(
            fs_hz as f64 >= 4.0 * plan.f_max(),
            "the rig runs at {fs_hz} Hz, too slow for {} Hz",
            plan.f_max()
        );
        let mut writer = match &stream_csv {
            Some(path) => Some(RunWriter::create(
                path,
                &RunMeta {
                    source: format!("bode / {}", plan.name),
                    fs_hz,
                    adc: info.adc_scale(),
                    oversample: info.oversample,
                    firmware: info.firmware.clone(),
                    note: Some(format!(
                        "stepped-sine Bode '{}': {} excited, {} measured",
                        plan.name,
                        plan.excite.name(),
                        plan.response.name()
                    )),
                    origin_sample: 0,
                },
            )?),
            None => None,
        };
        let schedule = plan.schedule();
        obs.started(fs_hz, &schedule);

        let mut stream = Stream::new(fs_hz, info.adc_scale());
        let mut sweep = Sweep {
            daq,
            stream: &mut stream,
            writer: writer.as_mut(),
            obs,
            last_status: Instant::now(),
        };

        // ── operating point ────────────────────────────────────────────────
        let until = secs_to_samples(plan.initial_settle_s, fs_hz);
        while sweep.stream.position < until {
            if !sweep.pull(|_, _| {})? {
                cancelled = true;
                break;
            }
            if sweep.due() {
                let pos = sweep.stream.position;
                sweep.obs.status(&format!(
                    "operating point: {:5.1}/{:.0} s",
                    pos as f64 / fs_hz as f64,
                    plan.initial_settle_s
                ));
            }
        }

        // ── the sweep ──────────────────────────────────────────────────────
        let margin = plan.margin_s();
        let input_ch = plan.excite.index() as usize;
        let response_ch = plan.response.index();
        let skew_s = plan.skew_s();
        let mut prev_phase: Option<f64> = None;

        for p in &schedule {
            if cancelled || sweep.obs.cancelled() {
                cancelled = true;
                break;
            }
            sweep.obs.point_started(p.index, p.freq_hz);
            // The frequency the firmware will actually run, so the fit is at
            // exactly that one.
            let f_wire = p.freq_hz as f32;
            let f = f_wire as f64;
            sweep.daq.program(
                plan.excite.index(),
                Waveform::Sine {
                    center: plan.center_v,
                    amplitude: plan.amplitude_v,
                    freq_hz: f_wire,
                    cycles: p.cycles,
                },
            )?;
            let base = sweep.stream.position;
            sweep.daq.gen_start()?;

            let run_s = p.cycles as f64 / f;
            let w_start = base + secs_to_samples(run_s - margin - p.measure_s, fs_hz);
            let w_end = base + secs_to_samples(run_s - margin, fs_hz);
            let done = base + secs_to_samples(run_s + margin, fs_hz);
            let mut t = Vec::with_capacity((w_end - w_start) as usize);
            let mut u = Vec::with_capacity(t.capacity());
            let mut y = Vec::with_capacity(t.capacity());

            while sweep.stream.position < done {
                let ok = sweep.pull(|index, row| {
                    if (w_start..w_end).contains(&index) {
                        t.push((index - w_start) as f64 / fs_hz as f64);
                        u.push(row[input_ch] as f64);
                        y.push(row[response_ch] as f64);
                    }
                })?;
                if !ok {
                    cancelled = true;
                    break;
                }
                if sweep.due() {
                    let pos = sweep.stream.position;
                    let phase = if pos < w_start {
                        "settling "
                    } else if pos < w_end {
                        "measuring"
                    } else {
                        "finishing"
                    };
                    sweep.obs.status(&format!(
                        "{:>8.4} Hz  [{}/{}]  {phase}  {:6.1}/{:.1} s",
                        p.freq_hz,
                        p.index + 1,
                        schedule.len(),
                        pos.saturating_sub(base) as f64 / fs_hz as f64,
                        p.total_s
                    ));
                }
            }
            if cancelled {
                break;
            }

            let point = measure(
                p,
                f,
                &t,
                &u,
                &y,
                w_start,
                skew_s,
                plan.correct_skew,
                &mut prev_phase,
            )?;
            sweep.obs.point_finished(&point);
            points.push(point);
        }
        if let Some(w) = writer {
            w.finish()?;
        }
        Ok(())
    })();

    // The rig keeps streaming and keeps driving whatever it was last told to,
    // so every way out goes through here.
    let _ = daq.gen_stop();
    let _ = daq.park();
    let _ = daq.stop();
    outcome?;

    let freqs: Vec<f64> = points.iter().map(|p| p.freq_hz).collect();
    let raw: Vec<f64> = points.iter().map(|p| p.raw_phase_deg).collect();
    let delay = estimate_delay(&freqs, &raw);
    let slots = plan.response.index() as f64 - plan.excite.index() as f64;
    let implied_scan_spacing_s = delay.filter(|_| slots != 0.0).map(|d| -d.delay_s / slots);

    let result = BodeResult {
        plan: plan.clone(),
        started_utc,
        fs_hz,
        tick_hz: gen_info.tick_hz,
        firmware: info.firmware.clone(),
        skew_s: plan.skew_s(),
        points,
        delay,
        implied_scan_spacing_s,
        cancelled,
    };
    if let Some(dir) = out_dir {
        result.save(dir)?;
    }
    Ok(result)
}

/// The moving parts of a sweep, so the loops above can share one "next
/// block" that also records, reports and checks for a cancel.
struct Sweep<'a> {
    daq: &'a mut Daq,
    stream: &'a mut Stream,
    writer: Option<&'a mut RunWriter>,
    obs: &'a mut dyn BodeObserver,
    last_status: Instant,
}

impl Sweep<'_> {
    /// Take one block: write it, show it, hand each row with its stream index
    /// to `each`. `false` if the observer asked to cancel.
    fn pull(&mut self, mut each: impl FnMut(u64, &[f32; 3])) -> Result<bool> {
        let block = next_block(self.daq, self.stream)?;
        if let Some(w) = self.writer.as_deref_mut() {
            w.push_block(block.seq, block.n, block.channels, &block.counts)?;
        }
        self.obs
            .samples(block.first_sample, self.stream.fs_hz, &block.rows);
        for (i, row) in block.rows.iter().enumerate() {
            each(block.first_sample + i as u64, row);
        }
        Ok(!self.obs.cancelled())
    }

    /// Whether a progress line is due (twice a second).
    fn due(&mut self) -> bool {
        if self.last_status.elapsed() >= Duration::from_millis(500) {
            self.last_status = Instant::now();
            true
        } else {
            false
        }
    }
}

fn secs_to_samples(s: f64, fs_hz: u32) -> u64 {
    (s.max(0.0) * fs_hz as f64).round() as u64
}

/// Fit one window and turn it into a point.
#[allow(clippy::too_many_arguments)]
fn measure(
    p: &PointSchedule,
    f: f64,
    t: &[f64],
    u: &[f64],
    y: &[f64],
    first_sample: u64,
    skew_s: f64,
    correct: bool,
    prev_phase: &mut Option<f64>,
) -> Result<BodePoint> {
    ensure!(
        t.len() >= 8,
        "{} Hz: only {} samples in the fit window — the stream lost too much",
        p.freq_hz,
        t.len()
    );
    let fu = sine_fit(t, u, f);
    let fy = sine_fit(t, y, f);
    ensure!(
        fu.amplitude > 1e-6,
        "{} Hz: the input amplitude is {:.2e} V — the excited output is not moving \
         (is its sense channel wired?)",
        p.freq_hz,
        fu.amplitude
    );
    let gain = fy.amplitude / fu.amplitude;
    // Each fit reports its own lag; the transfer function's phase is how much
    // *more* the output lags than the input.
    let wrapped = wrap_deg(-(fy.phase_rad - fu.phase_rad).to_degrees());
    let raw = match *prev_phase {
        Some(prev) => wrapped + 360.0 * ((prev - wrapped) / 360.0).round(),
        None => wrapped,
    };
    *prev_phase = Some(raw);
    let skew_deg = 360.0 * f * skew_s;
    Ok(BodePoint {
        index: p.index,
        freq_hz: p.freq_hz,
        gain,
        gain_db: 20.0 * gain.log10(),
        phase_deg: if correct { raw - skew_deg } else { raw },
        raw_phase_deg: raw,
        skew_deg,
        input_amplitude_v: fu.amplitude,
        response_amplitude_v: fy.amplitude,
        input_offset_v: fu.offset,
        response_offset_v: fy.offset,
        input_residual: fu.residual_ratio,
        response_residual: fy.residual_ratio,
        cycles: p.measure_cycles as f64,
        first_sample,
        samples: t.len(),
    })
}

fn wrap_deg(deg: f64) -> f64 {
    let w = (deg + 180.0).rem_euclid(360.0) - 180.0;
    if w <= -180.0 {
        w + 360.0
    } else {
        w
    }
}

/// Console progress for the CLI: a header, then one table row per point.
pub struct ConsoleBodeObserver {
    rows: usize,
}

impl ConsoleBodeObserver {
    /// A fresh table.
    pub fn new() -> Self {
        Self { rows: 0 }
    }
}

impl Default for ConsoleBodeObserver {
    fn default() -> Self {
        Self::new()
    }
}

impl BodeObserver for ConsoleBodeObserver {
    fn started(&mut self, fs_hz: u32, schedule: &[PointSchedule]) {
        println!("{} point(s) at {fs_hz} Hz sampling\n", schedule.len());
    }

    fn status(&mut self, line: &str) {
        print!("\r  {line}      ");
        let _ = std::io::stdout().flush();
    }

    fn point_finished(&mut self, p: &BodePoint) {
        if self.rows == 0 {
            println!(
                "\r   freq Hz     gain dB    phase °    raw °   skew °    in V      out V    resid"
            );
        }
        self.rows += 1;
        println!(
            "\r  {:>9.4}   {:+8.3}   {:+8.2}   {:+7.2}  {:+6.2}   {:7.4}   {:8.5}   {:5.3}          ",
            p.freq_hz,
            p.gain_db,
            p.phase_deg,
            p.raw_phase_deg,
            p.skew_deg,
            p.input_amplitude_v,
            p.response_amplitude_v,
            p.response_residual.max(p.input_residual),
        );
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    #[test]
    fn log_space_hits_both_ends_and_is_geometric() {
        let f = log_space(0.1, 100.0, 4);
        assert_eq!(f.len(), 4);
        for (got, want) in f.iter().zip([0.1, 1.0, 10.0, 100.0]) {
            assert!((got / want - 1.0).abs() < 1e-12, "{f:?}");
        }
        assert_eq!(log_space(5.0, 5.0, 1), vec![5.0]);
        assert!(log_space(0.0, 1.0, 3).is_empty());
        assert!(log_space(1.0, 2.0, 0).is_empty());
        // Descending works too.
        let d = log_space(10.0, 1.0, 3);
        assert!((d[1] - 10f64.sqrt()).abs() < 1e-12 && d[2] == 1.0, "{d:?}");

        // Five per decade over exactly two decades: ten intervals, eleven
        // points — not twelve from rounding.
        assert_eq!(per_decade(0.1, 10.0, 5.0).len(), 11);
        let g = per_decade(0.02, 5.0, 5.0);
        assert_eq!(g.len(), 13);
        assert_eq!(*g.last().unwrap(), 5.0);
    }

    #[test]
    fn the_default_plan_is_valid_round_trips_and_takes_minutes() {
        let plan = BodePlan::default();
        plan.validate().unwrap();
        let text = plan.to_toml().unwrap();
        let back = BodePlan::from_toml(&text).unwrap();
        assert_eq!(back, plan);

        let d = plan.estimated_duration().as_secs_f64();
        assert!(
            (5.0 * 60.0..30.0 * 60.0).contains(&d),
            "a default sweep takes {:.1} min",
            d / 60.0
        );
        for p in plan.schedule() {
            assert!(p.measure_cycles >= 1);
            // The window fits inside the run, past the settle span, for any
            // start latency up to the margin.
            let m = plan.margin_s();
            assert!(
                p.run_s - 2.0 * m - p.measure_s >= p.settle_s - 1e-9,
                "{p:?}"
            );
            assert!((p.measure_s * p.freq_hz - p.measure_cycles as f64).abs() < 1e-9);
        }

        // A partial file takes everything else from the defaults.
        let partial =
            BodePlan::from_toml("excite = \"p_s\"\ncenter_v = 0.8\nhold_v = 2.5\n").unwrap();
        assert_eq!(partial.excite, ExcitedOutput::SteamPressure);
        assert_eq!(partial.frequencies_hz, plan.frequencies_hz);
    }

    #[test]
    fn validation_says_what_is_wrong() {
        let err = |f: &dyn Fn(&mut BodePlan)| {
            let mut p = BodePlan::default();
            f(&mut p);
            p.validate().unwrap_err().to_string()
        };
        assert!(err(&|p| p.frequencies_hz.clear()).contains("no frequencies"));
        assert!(err(&|p| p.frequencies_hz.push(-1.0)).contains("positive"));
        assert!(err(&|p| p.frequencies_hz.push(20.0)).contains("tested up to 15 Hz"));
        assert!(err(&|p| {
            p.max_freq_hz = 1000.0;
            p.frequencies_hz.push(400.0)
        })
        .contains("too slow"));
        assert!(err(&|p| p.amplitude_v = 0.6).contains("outside its window"));
        assert!(err(&|p| p.hold_v = 1.2).contains("hold_v"));
        assert!(err(&|p| p.measure_cycles = 0.5).contains("measure_cycles"));
        assert!(err(&|p| p.scan_spacing_s = 0.01).contains("scan_spacing_s"));
        assert!(BodePlan::from_toml("frequncies_hz = [1.0]").is_err());
    }

    #[test]
    fn a_pure_delay_is_recovered_and_the_skew_has_the_right_sign() {
        let f = [1.0, 10.0, 30.0, 100.0];
        // The response channel is converted two scan slots later: it reads as a lead.
        let phase: Vec<f64> = f.iter().map(|f| 360.0 * f * 192e-6).collect();
        let d = estimate_delay(&f, &phase).unwrap();
        assert!((d.delay_s + 192e-6).abs() < 1e-12, "{d:?}");
        assert!(d.rms_residual_deg < 1e-9);
        assert!(estimate_delay(&[], &[]).is_none());

        let plan = BodePlan::default();
        assert!((plan.skew_s() - 2.0 * SCAN_CHANNEL_SPACING_S).abs() < 1e-15);
        let mut prev = None;
        let sched = PointSchedule {
            index: 0,
            freq_hz: 100.0,
            cycles: 10,
            run_s: 0.1,
            settle_s: 0.0,
            measure_cycles: 10,
            measure_s: 0.1,
            total_s: 0.1,
        };
        // A wire: the response is the input two scan slots later.
        let fs = 2000.0;
        let t: Vec<f64> = (0..2000).map(|i| i as f64 / fs).collect();
        let u: Vec<f64> = t.iter().map(|t| (2.0 * PI * 100.0 * t).sin()).collect();
        let y: Vec<f64> = t
            .iter()
            .map(|t| (2.0 * PI * 100.0 * (t + plan.skew_s())).sin())
            .collect();
        let p = measure(&sched, 100.0, &t, &u, &y, 0, plan.skew_s(), true, &mut prev).unwrap();
        let lead = 360.0 * 100.0 * plan.skew_s();
        assert!((p.raw_phase_deg - lead).abs() < 1e-6, "{p:?}");
        assert!(p.phase_deg.abs() < 1e-6, "{p:?}");
        assert!(p.gain_db.abs() < 1e-9, "{p:?}");
    }

    #[test]
    fn phase_unwraps_along_the_sweep() {
        assert_eq!(wrap_deg(190.0), -170.0);
        assert_eq!(wrap_deg(-180.0), 180.0);
        let mut prev = Some(-170.0);
        let sched = PointSchedule {
            index: 1,
            freq_hz: 1.0,
            cycles: 4,
            run_s: 4.0,
            settle_s: 0.0,
            measure_cycles: 4,
            measure_s: 4.0,
            total_s: 4.0,
        };
        let fs = 100.0;
        let t: Vec<f64> = (0..400).map(|i| i as f64 / fs).collect();
        let u: Vec<f64> = t.iter().map(|t| (2.0 * PI * t).cos()).collect();
        // 200° of lag: wraps to +160°, unwraps to −200° next to −170°.
        let lag = 200f64.to_radians();
        let y: Vec<f64> = t.iter().map(|t| (2.0 * PI * t - lag).cos()).collect();
        let p = measure(&sched, 1.0, &t, &u, &y, 0, 0.0, true, &mut prev).unwrap();
        assert!((p.raw_phase_deg + 200.0).abs() < 1e-6, "{p:?}");
    }
}
