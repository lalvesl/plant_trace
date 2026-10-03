//! Experiment descriptions.
//!
//! An experiment is a list of steps executed in order against the rig: settle
//! at an operating point, then record while an excitation runs. Keeping it in
//! a file rather than in flags means the run that produced a CSV can be
//! reproduced exactly, and is what the report cites.
//!
//! The waveform types here mirror [`plant_trace_proto::waveform::Waveform`]
//! with a `kind = "..."` tag, because the wire format's externally-tagged enum
//! reads badly in TOML and a description a human edits should read well.

use std::collections::BTreeMap;

use anyhow::{bail, Context, Result};
use plant_trace_proto::{
    scale::DacScale,
    waveform::{Generator, Waveform},
};
use serde::{Deserialize, Serialize};

/// A complete experiment.
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Experiment {
    /// Short name; used for the output directory.
    pub name: String,
    /// What the experiment is for — copied into the run manifest.
    #[serde(default)]
    pub description: String,
    /// Sample rate to ask the DAQ for; 0 takes the firmware's default.
    #[serde(default)]
    pub fs_hz: u32,
    /// Bench calibration per output, keyed `u_t` and `p_s`.
    #[serde(default)]
    pub outputs: BTreeMap<String, OutputScale>,
    /// The steps, in order.
    #[serde(rename = "step")]
    pub steps: Vec<Step>,
}

/// Bench-measured scaling and safety window for one output.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub struct OutputScale {
    /// Volts at the plant input per DAC code.
    pub volts_per_code: f32,
    /// Volts at the plant input for code 0.
    #[serde(default)]
    pub offset_v: f32,
    /// Lowest voltage the generator may command.
    pub min_v: f32,
    /// Highest voltage the generator may command.
    pub max_v: f32,
}

impl From<OutputScale> for DacScale {
    fn from(o: OutputScale) -> Self {
        DacScale {
            volts_per_code: o.volts_per_code,
            offset_v: o.offset_v,
            min_v: o.min_v,
            max_v: o.max_v,
        }
    }
}

/// One step of an experiment.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum Step {
    /// Drive both outputs to a level and wait for the plant to stop moving.
    ///
    /// Every measurement in the assignment starts from steady state, and
    /// "steady" is a property of the output, not of a stopwatch — so this
    /// watches `P_e` instead of sleeping.
    Settle {
        /// Label for the manifest.
        name: String,
        /// Valve command to hold, volts.
        u_t: f32,
        /// Steam pressure to hold, volts.
        p_s: f32,
        /// Give up after this long, seconds.
        #[serde(default = "default_settle_timeout")]
        timeout_s: f32,
        /// Peak-to-peak band that counts as settled, volts.
        #[serde(default = "default_tolerance")]
        tol_v: f32,
        /// Length of the window the band is measured over, seconds.
        #[serde(default = "default_window")]
        window_s: f32,
    },
    /// Record while the staged excitation runs.
    Record {
        /// Label; also the CSV file name.
        name: String,
        /// How long to record, seconds.
        duration_s: f32,
        /// Excitation for the valve command, if any.
        #[serde(default)]
        u_t: Option<WaveSpec>,
        /// Excitation for the steam pressure, if any.
        #[serde(default)]
        p_s: Option<WaveSpec>,
    },
}

impl Step {
    /// Label used in the manifest and on screen.
    pub fn name(&self) -> &str {
        match self {
            Step::Settle { name, .. } | Step::Record { name, .. } => name,
        }
    }
}

fn default_settle_timeout() -> f32 {
    180.0
}
fn default_tolerance() -> f32 {
    0.002
}
fn default_window() -> f32 {
    5.0
}

/// A waveform, in the shape a human writes it.
#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum WaveSpec {
    /// Constant level.
    Hold {
        /// Level, volts.
        level: f32,
    },
    /// Linear ramp, then hold.
    Ramp {
        /// Starting level, volts.
        from: f32,
        /// Final level, volts.
        to: f32,
        /// Duration, seconds.
        duration_s: f32,
    },
    /// A single step, expressed the way the assignment does: a base level, a
    /// dwell, then a jump of `step` volts.
    Step {
        /// Level before the jump, volts.
        base: f32,
        /// Size of the jump, volts (negative steps down).
        step: f32,
        /// Time spent at `base` before the jump, seconds.
        hold_s: f32,
    },
    /// Staircase for a static gain curve.
    Staircase {
        /// First plateau, volts.
        start: f32,
        /// Increment between plateaus, volts.
        step: f32,
        /// Number of plateaus.
        steps: u16,
        /// Time on each plateau, seconds.
        dwell_s: f32,
    },
    /// One Bode point.
    Sine {
        /// Operating point, volts.
        center: f32,
        /// Peak amplitude, volts.
        amplitude: f32,
        /// Frequency, hertz.
        freq_hz: f32,
        /// Periods to run; 0 runs until the step ends.
        #[serde(default)]
        cycles: u32,
    },
    /// Exponential sweep.
    Chirp {
        /// Operating point, volts.
        center: f32,
        /// Peak amplitude, volts.
        amplitude: f32,
        /// Starting frequency, hertz.
        f0_hz: f32,
        /// Final frequency, hertz.
        f1_hz: f32,
        /// Sweep length, seconds.
        duration_s: f32,
    },
    /// Pseudo-random binary sequence.
    Prbs {
        /// Operating point, volts.
        center: f32,
        /// Half the peak-to-peak swing, volts.
        amplitude: f32,
        /// Time per bit, seconds.
        bit_s: f32,
        /// LFSR order, 5..=15.
        order: u8,
        /// Run length, seconds.
        duration_s: f32,
    },
}

impl WaveSpec {
    /// Convert to the wire form.
    ///
    /// `Step` has no wire counterpart: it is a two-plateau staircase, which is
    /// exactly what a step test is, and expressing it that way keeps the
    /// device's waveform set small.
    pub fn to_wire(self) -> Waveform {
        match self {
            WaveSpec::Hold { level } => Waveform::Hold { level },
            WaveSpec::Ramp {
                from,
                to,
                duration_s,
            } => Waveform::Ramp {
                from,
                to,
                duration_s,
            },
            WaveSpec::Step { base, step, hold_s } => Waveform::Staircase {
                start: base,
                step,
                steps: 2,
                dwell_s: hold_s,
            },
            WaveSpec::Staircase {
                start,
                step,
                steps,
                dwell_s,
            } => Waveform::Staircase {
                start,
                step,
                steps,
                dwell_s,
            },
            WaveSpec::Sine {
                center,
                amplitude,
                freq_hz,
                cycles,
            } => Waveform::Sine {
                center,
                amplitude,
                freq_hz,
                cycles,
            },
            WaveSpec::Chirp {
                center,
                amplitude,
                f0_hz,
                f1_hz,
                duration_s,
            } => Waveform::Chirp {
                center,
                amplitude,
                f0_hz,
                f1_hz,
                duration_s,
            },
            WaveSpec::Prbs {
                center,
                amplitude,
                bit_s,
                order,
                duration_s,
            } => Waveform::Prbs {
                center,
                amplitude,
                bit_s,
                order,
                duration_s,
            },
        }
    }
}

impl Experiment {
    /// Read and validate an experiment file.
    pub fn load(path: &std::path::Path) -> Result<Self> {
        let text =
            std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        let experiment: Experiment =
            toml::from_str(&text).with_context(|| format!("parsing {}", path.display()))?;
        experiment.validate()?;
        Ok(experiment)
    }

    /// Scaling for one output, or the nominal one if the file does not say.
    pub fn scale(&self, output: &str) -> DacScale {
        self.outputs
            .get(output)
            .copied()
            .map(DacScale::from)
            .unwrap_or(nominal_scale(output))
    }

    /// Refuse an experiment that would leave a safe window, before any of it
    /// reaches the plant. The firmware checks this too; catching it here means
    /// the operator finds out at load time rather than ten minutes into a run.
    ///
    /// Public so an editor can check a scenario as it is typed, not only when
    /// it is loaded from a file.
    pub fn validate(&self) -> Result<()> {
        if self.steps.is_empty() {
            bail!("the experiment has no steps");
        }
        for name in OUTPUT_KEYS {
            check_scale(name, self.scale(name))?;
        }
        for step in &self.steps {
            match step {
                Step::Settle { name, u_t, p_s, .. } => {
                    check_level("u_t", *u_t, self.scale("u_t"), name)?;
                    check_level("p_s", *p_s, self.scale("p_s"), name)?;
                }
                Step::Record {
                    name,
                    duration_s,
                    u_t,
                    p_s,
                } => {
                    if *duration_s <= 0.0 {
                        bail!("step '{name}' records for {duration_s} s");
                    }
                    for (which, spec) in [("u_t", u_t), ("p_s", p_s)] {
                        let Some(spec) = spec else { continue };
                        check_frequency(name, which, spec)?;
                        let (lo, hi) = spec.to_wire().span();
                        check_level(which, lo, self.scale(which), name)?;
                        check_level(which, hi, self.scale(which), name)?;
                    }
                }
            }
        }
        Ok(())
    }
}

/// Keys of the `[outputs.*]` tables, in wire order.
pub const OUTPUT_KEYS: [&str; 2] = ["u_t", "p_s"];

/// The nominal map of an output by its key: the offset ladder for `u_t`, the
/// bare ladder for anything else. See [`DacScale::U_T_NOMINAL`].
pub fn nominal_scale(output: &str) -> DacScale {
    match output {
        "u_t" => DacScale::U_T_NOMINAL,
        _ => DacScale::NOMINAL,
    }
}

/// Refuse a calibration table that could put the plant outside its range.
///
/// Two ways a file can do that. Its window can be wider than the plant's
/// (`u_t` 2.25-2.75 V, `p_s` 0-1 V), and the window in the file is what the
/// firmware clamps to. Or its line can be one the rig cannot follow: a `u_t`
/// table without its 2.15 V `offset_v` puts the whole window above code 255,
/// and every level would silently clamp to the top.
pub fn check_scale(which: &str, scale: DacScale) -> Result<()> {
    let plant = nominal_scale(which);
    if !(scale.volts_per_code.is_finite() && scale.volts_per_code > 0.0) {
        bail!("[outputs.{which}] volts_per_code must be positive");
    }
    if scale.min_v > scale.max_v {
        bail!("[outputs.{which}] min_v is above max_v");
    }
    if scale.min_v < plant.min_v - 1e-6 || scale.max_v > plant.max_v + 1e-6 {
        bail!(
            "[outputs.{which}] opens the window to {:.3}…{:.3} V; the plant takes {which} in              {:.3}…{:.3} V",
            scale.min_v,
            scale.max_v,
            plant.min_v,
            plant.max_v
        );
    }
    let (lo, hi) = (scale.to_volts(0), scale.to_volts(u8::MAX));
    if lo > scale.min_v + 1e-6 || hi < scale.max_v - 1e-6 {
        bail!(
            "[outputs.{which}] reaches only {lo:.3}…{hi:.3} V with an 8-bit duty, not the              whole window {:.3}…{:.3} V — is offset_v missing?",
            scale.min_v,
            scale.max_v
        );
    }
    Ok(())
}

/// Highest sine (or chirp) frequency an experiment may put on the plant, hertz.
///
/// A limit of the plant, not of the rig: the rig itself is characterised to
/// 100 Hz with a wire in the plant's place (`experiments/bode-wire.toml`),
/// but the box is only to be tested up to 15 Hz. Its dynamics of interest sit
/// well below that (the rotor mode is ~1.4 Hz).
pub const PLANT_MAX_FREQ_HZ: f32 = 15.0;

fn check_frequency(step: &str, which: &str, spec: &WaveSpec) -> Result<()> {
    let f = match spec {
        WaveSpec::Sine { freq_hz, .. } => *freq_hz,
        WaveSpec::Chirp { f0_hz, f1_hz, .. } => f0_hz.max(*f1_hz),
        _ => return Ok(()),
    };
    if f > PLANT_MAX_FREQ_HZ {
        bail!(
            "step '{step}' drives {which} at {f} Hz; the plant is tested up to \
             {PLANT_MAX_FREQ_HZ} Hz"
        );
    }
    Ok(())
}

fn check_level(which: &str, volts: f32, scale: DacScale, step: &str) -> Result<()> {
    if volts < scale.min_v - 1e-6 || volts > scale.max_v + 1e-6 {
        bail!(
            "step '{step}' asks {which} for {volts:.3} V, outside the configured window \
             {:.3}…{:.3} V",
            scale.min_v,
            scale.max_v
        );
    }
    Ok(())
}

// ── preview ─────────────────────────────────────────────────────────────────

/// What an experiment will command, against time — for plotting a scenario
/// before it is run.
///
/// Levels are the *commanded* ones, before the 8-bit quantisation and the
/// output filter; the waveforms are evaluated by the same
/// [`plant_trace_proto::waveform::Generator`] the firmware and the simulator
/// run, so a PRBS here is bit for bit the one the plant will get.
#[derive(Debug, Clone, Default)]
pub struct Preview {
    /// Time since the start of the first step, seconds.
    pub t_s: Vec<f64>,
    /// Commanded valve level at each instant, volts.
    pub u_t: Vec<f32>,
    /// Commanded steam pressure at each instant, volts.
    pub p_s: Vec<f32>,
    /// Where each step sits on that time axis, in order.
    pub spans: Vec<PreviewSpan>,
}

/// One step's place on a [`Preview`].
#[derive(Debug, Clone)]
pub struct PreviewSpan {
    /// Index of the step in [`Experiment::steps`].
    pub index: usize,
    /// Step label.
    pub name: String,
    /// Start, seconds since the start of the preview.
    pub start_s: f64,
    /// End, seconds.
    pub end_s: f64,
    /// The length is a guess, not a schedule: a settle step lasts until the
    /// plant is steady, which nobody knows in advance. It is drawn at its
    /// shortest possible length, [`Step::nominal_duration_s`].
    pub nominal: bool,
}

impl Step {
    /// How long the step is drawn in a [`Preview`], seconds.
    ///
    /// A record step lasts exactly `duration_s`. A settle step lasts until
    /// the plant is steady, anywhere up to `timeout_s`; the shortest it can
    /// possibly take is one full `window_s` (the detector has to fill its
    /// window before it can call anything settled), and that is the length
    /// used.
    pub fn nominal_duration_s(&self) -> f64 {
        match self {
            Step::Settle { window_s, .. } => *window_s as f64,
            Step::Record { duration_s, .. } => *duration_s as f64,
        }
    }

    /// This step alone on a [`Preview`] of its own (time from 0, one span),
    /// starting with the outputs at `held` — `[u_t, p_s]`, volts: what the
    /// previous step left them at. Also returns the levels this step leaves
    /// them at, to chain the next one. See [`Experiment::preview`] for the
    /// semantics.
    pub fn preview(&self, held: [f32; 2], rate_hz: f64) -> (Preview, [f32; 2]) {
        let mut out = Preview::default();
        let end = preview_step(self, held, rate_hz, 0.0, &mut out);
        out.spans.push(PreviewSpan {
            index: 0,
            name: self.name().to_string(),
            start_s: 0.0,
            end_s: self.nominal_duration_s().max(0.0),
            nominal: matches!(self, Step::Settle { .. }),
        });
        (out, end)
    }
}

impl Experiment {
    /// Commanded `u_T` and `p_s` against time, sampled at `rate_hz` (100 Hz
    /// is plenty for a plot; every step boundary gets a point of its own
    /// regardless, so edges stay sharp).
    ///
    /// Follows the runner's semantics: a settle step holds its two levels; a
    /// record step starts its waveforms at its own `t = 0`, a finite waveform
    /// that ends before the step does holds its last value (as the firmware
    /// does), and an output the step does not mention keeps whatever level
    /// the previous step left it at. Before the first step the outputs are
    /// assumed parked at the bottom of their windows.
    pub fn preview(&self, rate_hz: f64) -> Preview {
        let mut out = Preview::default();
        let mut held = [self.scale("u_t").min_v, self.scale("p_s").min_v];
        let mut t0 = 0.0f64;
        for (index, step) in self.steps.iter().enumerate() {
            let d = step.nominal_duration_s().max(0.0);
            held = preview_step(step, held, rate_hz, t0, &mut out);
            out.spans.push(PreviewSpan {
                index,
                name: step.name().to_string(),
                start_s: t0,
                end_s: t0 + d,
                nominal: matches!(step, Step::Settle { .. }),
            });
            t0 += d;
        }
        out
    }
}

/// Append one step to `out`, starting at `t0` with the outputs at `held`, and
/// return the levels it leaves the outputs at.
fn preview_step(step: &Step, held: [f32; 2], rate_hz: f64, t0: f64, out: &mut Preview) -> [f32; 2] {
    let d = step.nominal_duration_s().max(0.0);
    let rate = if rate_hz.is_finite() && rate_hz > 0.0 {
        rate_hz
    } else {
        100.0
    };
    // Points at the rate, plus the step's last instant.
    let n = (d * rate).ceil() as usize;
    let times = (0..n)
        .map(|k| k as f64 / rate)
        .filter(|t| *t < d)
        .chain(std::iter::once(d));

    match step {
        Step::Settle { u_t, p_s, .. } => {
            for t in times {
                out.t_s.push(t0 + t);
                out.u_t.push(*u_t);
                out.p_s.push(*p_s);
            }
            [*u_t, *p_s]
        }
        Step::Record { u_t, p_s, .. } => {
            let mut gens =
                [u_t, p_s].map(|spec| spec.map(|s| (Generator::new(s.to_wire()), None::<f32>)));
            let mut last = held;
            for t in times {
                let mut level = held;
                for (ch, slot) in gens.iter_mut().enumerate() {
                    let Some((gen, frozen)) = slot else { continue };
                    level[ch] = match *frozen {
                        Some(v) => v,
                        None => match gen.waveform().duration_s() {
                            // Frozen where it ended, as the firmware's tick
                            // does, rather than running on.
                            Some(end) if t as f32 >= end => {
                                let v = gen.sample(end);
                                *frozen = Some(v);
                                v
                            }
                            _ => gen.sample(t as f32),
                        },
                    };
                }
                out.t_s.push(t0 + t);
                out.u_t.push(level[0]);
                out.p_s.push(level[1]);
                last = level;
            }
            last
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn a_sine_above_the_plant_limit_is_refused() {
        let mut exp: Experiment = toml::from_str(
            "name = \"t\"\n[[step]]\nkind = \"record\"\nname = \"s\"\nduration_s = 5.0\n\
             [step.u_t]\nkind = \"sine\"\ncenter = 2.5\namplitude = 0.025\nfreq_hz = 15.0\n",
        )
        .unwrap();
        exp.validate().unwrap();
        if let Step::Record {
            u_t: Some(WaveSpec::Sine { freq_hz, .. }),
            ..
        } = &mut exp.steps[0]
        {
            *freq_hz = 16.0;
        }
        let e = exp.validate().unwrap_err().to_string();
        assert!(e.contains("tested up to 15 Hz"), "{e}");
    }

    use super::*;

    const SAMPLE: &str = r#"
name = "demo"
fs_hz = 1000

[outputs.u_t]
volts_per_code = 0.00277
offset_v = 2.150
min_v = 2.25
max_v = 2.75

[[step]]
kind = "settle"
name = "operating-point"
u_t = 2.5
p_s = 0.8

[[step]]
kind = "record"
name = "step-up"
duration_s = 30
[step.u_t]
kind = "step"
base = 2.5
step = 0.025
hold_s = 5
"#;

    #[test]
    fn parses_a_two_step_experiment() {
        let e: Experiment = toml::from_str(SAMPLE).unwrap();
        e.validate().unwrap();
        assert_eq!(e.steps.len(), 2);
        assert_eq!(e.steps[1].name(), "step-up");
        // A step test is a two-plateau staircase on the wire.
        match &e.steps[1] {
            Step::Record { u_t: Some(w), .. } => {
                assert!(matches!(w.to_wire(), Waveform::Staircase { steps: 2, .. }));
            }
            other => panic!("unexpected step: {other:?}"),
        }
    }

    #[test]
    fn refuses_an_excitation_that_leaves_the_safe_window() {
        let text = SAMPLE.replace("step = 0.025", "step = 0.30");
        let e: Experiment = toml::from_str(&text).unwrap();
        let err = e.validate().unwrap_err().to_string();
        assert!(err.contains("outside the configured window"), "{err}");
    }

    #[test]
    fn the_preview_follows_the_steps_and_the_waveforms() {
        let e: Experiment = toml::from_str(SAMPLE).unwrap();
        let p = e.preview(100.0);
        assert_eq!(p.spans.len(), 2);
        // The settle step is drawn at its window, flagged as a guess.
        assert!(p.spans[0].nominal);
        assert!((p.spans[0].end_s - 5.0).abs() < 1e-9);
        assert!((p.spans[1].end_s - 35.0).abs() < 1e-9);
        assert_eq!(p.t_s.len(), p.u_t.len());
        assert_eq!(p.t_s.len(), p.p_s.len());
        assert!(
            p.t_s.windows(2).all(|w| w[1] >= w[0]),
            "time goes backwards"
        );

        let at = |t: f64| {
            let i = p.t_s.iter().position(|x| *x >= t - 1e-9).unwrap();
            (p.u_t[i], p.p_s[i])
        };
        // Settling at the operating point...
        assert_eq!(at(1.0), (2.5, 0.8));
        // ...then the step: 5 s at the base, then +0.025 V, and p_s untouched.
        assert_eq!(at(5.0 + 4.9), (2.5, 0.8));
        let (u, p_s) = at(5.0 + 5.5);
        assert!((u - 2.525).abs() < 1e-6, "{u}");
        assert_eq!(p_s, 0.8);
        // The staircase is 10 s long; after it the level holds, not resets.
        let (u, _) = at(5.0 + 29.0);
        assert!((u - 2.525).abs() < 1e-6, "{u}");
    }

    #[test]
    fn a_calibration_that_could_leave_the_plant_range_is_refused() {
        // Wider than the box takes.
        let text = SAMPLE.replace("max_v = 2.75", "max_v = 3.5");
        let e: Experiment = toml::from_str(&text).unwrap();
        let err = e.validate().unwrap_err().to_string();
        assert!(err.contains("the plant takes u_t"), "{err}");
        // The u_T stage without its offset cannot reach the window at all.
        let text = SAMPLE.replace("offset_v = 2.150\n", "");
        let e: Experiment = toml::from_str(&text).unwrap();
        let err = e.validate().unwrap_err().to_string();
        assert!(err.contains("offset_v missing"), "{err}");
        // A file that says nothing about u_t gets the stage's nominal map.
        let text = SAMPLE.replace("[outputs.u_t]", "[outputs.unused]");
        let e: Experiment = toml::from_str(&text).unwrap();
        assert_eq!(e.scale("u_t"), DacScale::U_T_NOMINAL);
        e.validate().unwrap();
    }
}
