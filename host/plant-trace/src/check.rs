//! Bench self-tests for the two analog outputs.
//!
//! Both tests drive a PWM and read the *same* voltage back through the 10 kΩ
//! sense resistor on the next pin, in one session over one link: the rig measures
//! itself. That is the only way to see what the filter actually did, and it is
//! why `u_T` and `p_s` are wired to ADC channels at all.
//!
//! Neither test looks at `P_e`. They characterise the output chain — duty →
//! ladder → plant input — and say nothing about the plant, which is deliberate:
//! a static curve that is wrong because the DAC is wrong looks exactly like a
//! plant that is wrong.
//!
//! - [`dc`] steps through levels and averages each plateau. Its fit is the
//!   two-point calibration the experiment files ask for, done at ten points.
//! - [`sine`] runs a sinusoid and fits it back. Its gain and phase are the
//!   filter's, measured, against what `ngspice_filter.cir` predicts.

use std::time::Duration;

use anyhow::{bail, Context, Result};
use plant_trace_proto::{
    gen::N_OUTPUTS,
    scale::{AdcScale, DacScale},
    waveform::Waveform,
};

use crate::{
    analysis::freq::{sine_fit, SineFit},
    daq::{Daq, Event},
    sim::OUTPUT_FILTER_POLES_HZ,
};

/// How long to wait for a block before declaring the link dead.
const BLOCK_TIMEOUT: Duration = Duration::from_secs(3);

/// Names of the two outputs and of the sense channel each one is measured on.
pub const OUTPUT_NAMES: [&str; N_OUTPUTS] = ["u_t", "p_s"];

// ── DC sweep ────────────────────────────────────────────────────────────────

/// What [`dc`] should sweep.
#[derive(Debug, Clone)]
pub struct DcOptions {
    /// Outputs to drive, by index. The ones left out are not touched, so their
    /// sense column shows whether anything crosses over.
    pub channels: Vec<u8>,
    /// First level per output, volts at the plant input.
    pub from_v: [f32; N_OUTPUTS],
    /// Last level per output, volts. The outputs step together, each through
    /// its own range: `u_T` lives in 2.25-2.75 V and `p_s` in 0-1 V.
    pub to_v: [f32; N_OUTPUTS],
    /// Number of levels, including both ends.
    pub points: usize,
    /// Discarded after each level change, so the reading is of a settled
    /// plateau rather than of the edge.
    pub settle: Duration,
    /// Averaged at each level.
    pub average: Duration,
    /// Sample rate to ask the rig for; 0 takes its default.
    pub fs_hz: u32,
}

impl DcOptions {
    /// The same range on every output.
    pub fn with_range(mut self, from_v: f32, to_v: f32) -> Self {
        self.from_v = [from_v; N_OUTPUTS];
        self.to_v = [to_v; N_OUTPUTS];
        self
    }
}

impl Default for DcOptions {
    fn default() -> Self {
        Self {
            channels: vec![0, 1],
            from_v: DacScale::NOMINAL_OUTPUTS.map(|s| s.min_v),
            to_v: DacScale::NOMINAL_OUTPUTS.map(|s| s.max_v),
            points: 11,
            settle: Duration::from_millis(250),
            average: Duration::from_millis(500),
            fs_hz: 0,
        }
    }
}

/// What one sense channel read over one plateau.
#[derive(Debug, Clone, Copy)]
pub struct Plateau {
    /// Mean, volts at the plant input.
    pub mean_v: f64,
    /// Lowest sample, volts.
    pub min_v: f64,
    /// Highest sample, volts.
    pub max_v: f64,
    /// Standard deviation, volts. On a flat plateau this is the noise floor
    /// plus whatever carrier ripple the burst averaging left behind.
    pub std_v: f64,
    /// Samples averaged.
    pub n: usize,
}

impl Plateau {
    /// Peak-to-peak spread, volts.
    pub fn span_v(&self) -> f64 {
        self.max_v - self.min_v
    }
}

/// One level of the sweep.
#[derive(Debug, Clone)]
pub struct DcPoint {
    /// Level asked for, per output, volts.
    pub commanded_v: [f32; N_OUTPUTS],
    /// Duty the firmware chose, per output.
    pub codes: [u8; N_OUTPUTS],
    /// Level the firmware says that duty is, per output — the commanded value
    /// after clamping and quantisation.
    pub applied_v: [f32; N_OUTPUTS],
    /// What each sense channel read, per output.
    pub measured: [Plateau; N_OUTPUTS],
}

/// Straight line through one output's points.
#[derive(Debug, Clone, Copy)]
pub struct Calibration {
    /// Slope, volts at the plant per code — `volts_per_code` for the
    /// experiment files.
    pub volts_per_code: f64,
    /// Intercept, volts at code 0 — `offset_v`.
    pub offset_v: f64,
    /// Largest distance from the line, volts. This is the integral
    /// non-linearity of the whole chain; anything much above a millivolt means
    /// something is loading the ladder.
    pub max_deviation_v: f64,
    /// Points the line was fitted through.
    pub points: usize,
}

/// Result of a DC sweep.
#[derive(Debug, Clone)]
pub struct DcReport {
    /// Effective sample rate the rig streamed at.
    pub fs_hz: u32,
    /// Outputs that were driven.
    pub channels: Vec<u8>,
    /// One entry per level, in the order they were applied.
    pub points: Vec<DcPoint>,
    /// Fitted calibration, for the driven outputs only.
    pub calibration: [Option<Calibration>; N_OUTPUTS],
}

/// Step an output through a range of levels and read each plateau back.
///
/// Levels outside the safe window the firmware holds are clamped by the
/// firmware, not refused; `applied_v` is what it settled on, and fitting
/// against `codes` rather than against `commanded_v` is what keeps a clamped
/// endpoint from bending the line.
pub fn dc(daq: &mut Daq, opts: &DcOptions) -> Result<DcReport> {
    let channels = checked_channels(&opts.channels)?;
    if opts.points < 2 {
        bail!("a sweep needs at least two points, got {}", opts.points);
    }

    let info = daq.info()?;
    let adc = info.adc_scale();
    let fs_hz = daq.start(opts.fs_hz)?;
    // Never zero: the status read below needs at least one tick behind it.
    let settle_rows = rows(opts.settle, fs_hz).max(4);
    let average_rows = rows(opts.average, fs_hz).max(1);

    let mut points = Vec::with_capacity(opts.points);
    let outcome = (|| -> Result<()> {
        for i in 0..opts.points {
            let f = i as f32 / (opts.points - 1) as f32;
            let mut level = [0.0; N_OUTPUTS];
            for ch in &channels {
                let i = *ch as usize;
                level[i] = opts.from_v[i] + (opts.to_v[i] - opts.from_v[i]) * f;
                daq.set_level(*ch, level[i])
                    .with_context(|| format!("setting {} to {:.4} V", name(*ch), level[i]))?;
            }
            collect(daq, &adc, settle_rows)?;
            // Asked for only after the settle: the firmware applies a level on
            // its next 1 ms tick, so a status taken straight after `set_level`
            // reports the codes of the level before.
            let status = daq.gen_status()?;
            let window = collect(daq, &adc, average_rows)?;

            points.push(DcPoint {
                commanded_v: level,
                codes: status.codes,
                applied_v: status.volts,
                measured: [plateau(&window[0]), plateau(&window[1])],
            });
        }
        Ok(())
    })();

    // The rig keeps streaming and keeps driving whatever it was last told to
    // drive, so a failure halfway through has to be cleaned up after.
    let _ = daq.park();
    let _ = daq.stop();
    outcome?;

    let mut calibration = [None; N_OUTPUTS];
    for ch in &channels {
        let i = *ch as usize;
        calibration[i] = fit_line(&points, i);
    }

    Ok(DcReport {
        fs_hz,
        channels,
        points,
        calibration,
    })
}

// ── sine ────────────────────────────────────────────────────────────────────

/// What [`sine`] should apply.
#[derive(Debug, Clone)]
pub struct SineOptions {
    /// Outputs to drive, by index.
    pub channels: Vec<u8>,
    /// Level the sinusoid rides on, per output, volts.
    pub center_v: [f32; N_OUTPUTS],
    /// Peak amplitude, per output, volts.
    pub amplitude_v: [f32; N_OUTPUTS],
    /// Frequency, hertz.
    pub freq_hz: f32,
    /// Length of the window the fit runs on.
    pub duration: Duration,
    /// Discarded after the waveform starts, before the window.
    pub settle: Duration,
    /// Sample rate to ask the rig for; 0 takes its default.
    ///
    /// Worth raising above a kilohertz for anything fast. The outputs are
    /// updated at [`crate::daq::GenInfo::tick_hz`], so a sinusoid carries
    /// reconstruction images at `tick ± freq`; sampling at 1 kHz folds the one
    /// at `1000 − freq` straight onto the fundamental and the measured
    /// amplitude beats instead of settling.
    pub fs_hz: u32,
}

impl Default for SineOptions {
    fn default() -> Self {
        Self {
            channels: vec![0, 1],
            // The middle of each output's window, swinging over 80 % of it.
            center_v: DacScale::NOMINAL_OUTPUTS.map(|s| s.mid_v()),
            amplitude_v: DacScale::NOMINAL_OUTPUTS.map(|s| 0.4 * s.span_v()),
            freq_hz: 10.0,
            duration: Duration::from_secs(2),
            settle: Duration::from_millis(250),
            fs_hz: 2000,
        }
    }
}

/// What the design says the output chain does at one frequency.
#[derive(Debug, Clone, Copy)]
pub struct Expected {
    /// Gain, decibels.
    pub gain_db: f64,
    /// Phase, degrees; negative is lag.
    pub phase_deg: f64,
}

/// Gain and phase of the reconstruction filter plus the sample-and-hold the
/// waveform tick amounts to, at `freq_hz`.
///
/// Two real poles from [`OUTPUT_FILTER_POLES_HZ`] and the hold's `sinc`. The
/// hold is the part that surprises people: it costs almost no amplitude but
/// half a tick of delay, which at 100 Hz off a 1 kHz tick is already 18°.
pub fn expected_response(freq_hz: f64, tick_hz: f64) -> Expected {
    let mut gain = 1.0;
    let mut phase = 0.0;
    for pole in OUTPUT_FILTER_POLES_HZ {
        let r = freq_hz / pole;
        gain /= (1.0 + r * r).sqrt();
        phase -= r.atan().to_degrees();
    }
    if tick_hz > 0.0 {
        let x = std::f64::consts::PI * freq_hz / tick_hz;
        if x.abs() > 1e-9 {
            gain *= (x.sin() / x).abs();
        }
        phase -= 180.0 * freq_hz / tick_hz;
    }
    Expected {
        gain_db: 20.0 * gain.log10(),
        phase_deg: phase,
    }
}

/// What one sense channel made of the sinusoid.
#[derive(Debug, Clone, Copy)]
pub struct SineResult {
    /// Output index.
    pub channel: u8,
    /// The fit itself.
    pub fit: SineFit,
    /// Measured amplitude over commanded amplitude, decibels.
    pub gain_db: f64,
    /// Measured centre minus commanded centre, volts.
    pub center_error_v: f64,
}

/// Result of a sine test.
#[derive(Debug, Clone)]
pub struct SineReport {
    /// Effective sample rate the rig streamed at.
    pub fs_hz: u32,
    /// Rate the outputs were re-evaluated at.
    pub tick_hz: u32,
    /// Frequency applied.
    pub freq_hz: f64,
    /// Amplitude asked for, per output, volts.
    pub commanded_amplitude_v: [f64; N_OUTPUTS],
    /// One per driven output.
    pub results: Vec<SineResult>,
    /// What the design predicts, for the same frequency and tick.
    pub expected: Expected,
    /// Lag of the second driven output behind the first, as read off the two
    /// sense channels, degrees. Only present when two outputs were driven.
    ///
    /// The outputs themselves cannot skew — both duties go out in one DMA
    /// transfer — but the *reading* does: the SAADC converts `p_s` one scan
    /// slot after `u_T`, so `p_s` appears to lead by
    /// [`Self::scan_skew_deg`]. What is left after subtracting that is the
    /// rig.
    pub skew_deg: Option<f64>,
}

impl SineReport {
    /// What the scan order alone puts between the two sense channels at this
    /// frequency, degrees, in the sign convention of [`Self::skew_deg`]:
    /// negative, because the later-converted `p_s` reads as a lead. See
    /// [`plant_trace_proto::SCAN_CHANNEL_SPACING_S`].
    pub fn scan_skew_deg(&self) -> f64 {
        -360.0 * self.freq_hz * plant_trace_proto::SCAN_CHANNEL_SPACING_S
    }

    /// [`Self::skew_deg`] as a time, microseconds.
    pub fn skew_us(&self) -> Option<f64> {
        self.skew_deg
            .map(|deg| deg / 360.0 / self.freq_hz * 1e6)
            .filter(|us| us.is_finite())
    }
}

/// Apply a sinusoid and fit it back off the sense channels.
pub fn sine(daq: &mut Daq, opts: &SineOptions) -> Result<SineReport> {
    let channels = checked_channels(&opts.channels)?;
    if opts.freq_hz <= 0.0 {
        bail!("frequency must be positive, got {}", opts.freq_hz);
    }

    let info = daq.info()?;
    let adc = info.adc_scale();
    let tick_hz = daq.gen_info()?.tick_hz;
    let fs_hz = daq.start(opts.fs_hz)?;
    if (fs_hz as f64) < 4.0 * opts.freq_hz as f64 {
        bail!(
            "{} Hz sampling is too slow to fit a {} Hz sinusoid — ask for at least {:.0} Hz",
            fs_hz,
            opts.freq_hz,
            4.0 * opts.freq_hz as f64
        );
    }

    let wave = |ch: u8| Waveform::Sine {
        center: opts.center_v[ch as usize],
        amplitude: opts.amplitude_v[ch as usize],
        freq_hz: opts.freq_hz,
        cycles: 0,
    };

    let window = (|| -> Result<Vec<Vec<f64>>> {
        // Start from the centre so the first cycle is the waveform's and not
        // the filter's answer to a step onto the operating point.
        for ch in &channels {
            daq.set_level(*ch, opts.center_v[*ch as usize])?;
        }
        collect(daq, &adc, rows(opts.settle, fs_hz))?;

        for ch in &channels {
            daq.program(*ch, wave(*ch))
                .with_context(|| format!("programming {}", name(*ch)))?;
        }
        daq.gen_start()?;
        collect(daq, &adc, rows(opts.settle, fs_hz))?;

        let wanted = rows(opts.duration, fs_hz).max((fs_hz as f64 / opts.freq_hz as f64) as usize);
        collect(daq, &adc, wanted)
    })();

    let _ = daq.gen_stop();
    let _ = daq.park();
    let _ = daq.stop();
    let window = window?;

    let dt = 1.0 / fs_hz as f64;
    let t: Vec<f64> = (0..window[0].len()).map(|i| i as f64 * dt).collect();
    let freq_hz = opts.freq_hz as f64;
    let commanded = opts.amplitude_v.map(|a| a as f64);

    let mut results = Vec::with_capacity(channels.len());
    for ch in &channels {
        let i = *ch as usize;
        let fit = sine_fit(&t, &window[i], freq_hz);
        results.push(SineResult {
            channel: *ch,
            fit,
            gain_db: 20.0 * (fit.amplitude / commanded[i]).log10(),
            center_error_v: fit.offset - opts.center_v[i] as f64,
        });
    }

    let skew_deg = (results.len() == 2)
        .then(|| (results[1].fit.phase_rad - results[0].fit.phase_rad).to_degrees())
        .map(wrap_deg);

    Ok(SineReport {
        fs_hz,
        tick_hz,
        freq_hz,
        commanded_amplitude_v: commanded,
        results,
        expected: expected_response(freq_hz, tick_hz as f64),
        skew_deg,
    })
}

// ── shared ──────────────────────────────────────────────────────────────────

/// Name of an output, for messages.
pub fn name(ch: u8) -> &'static str {
    OUTPUT_NAMES.get(ch as usize).copied().unwrap_or("?")
}

/// Reject a bad channel list before the rig is told to do anything.
fn checked_channels(channels: &[u8]) -> Result<Vec<u8>> {
    if channels.is_empty() {
        bail!("no outputs selected");
    }
    let mut seen = channels.to_vec();
    seen.sort_unstable();
    seen.dedup();
    if let Some(bad) = seen.iter().find(|c| **c as usize >= N_OUTPUTS) {
        bail!("output {bad} does not exist — this rig has {N_OUTPUTS}");
    }
    Ok(seen)
}

/// Samples in a duration at `fs_hz`.
fn rows(d: Duration, fs_hz: u32) -> usize {
    (d.as_secs_f64() * fs_hz as f64).round() as usize
}

/// Read `rows` samples per channel off the stream, in volts at the plant.
///
/// Blocks arrive whole, so the count is met or exceeded rather than hit
/// exactly; a sweep only cares that it averaged over at least as long as it
/// asked for.
fn collect(daq: &mut Daq, adc: &AdcScale, rows: usize) -> Result<Vec<Vec<f64>>> {
    let mut out: Vec<Vec<f64>> = vec![Vec::new(); N_OUTPUTS + 1];
    if rows == 0 {
        return Ok(out);
    }
    let mut got = 0;
    while got < rows {
        match daq.next_event(BLOCK_TIMEOUT)? {
            Some(Event::Block {
                n,
                channels,
                counts,
                ..
            }) => {
                let ch = channels as usize;
                if ch == 0 {
                    continue;
                }
                if out.len() < ch {
                    out.resize(ch, Vec::new());
                }
                for frame in counts.chunks_exact(ch) {
                    for (c, sample) in frame.iter().enumerate() {
                        out[c].push(adc.to_volts(*sample) as f64);
                    }
                }
                got += n as usize;
            }
            Some(other) => eprintln!("unexpected message during the check: {other:?}"),
            None => bail!("the rig went quiet for {BLOCK_TIMEOUT:?}"),
        }
    }
    Ok(out)
}

/// Reduce one channel's window to the numbers the report prints.
fn plateau(v: &[f64]) -> Plateau {
    if v.is_empty() {
        return Plateau {
            mean_v: f64::NAN,
            min_v: f64::NAN,
            max_v: f64::NAN,
            std_v: f64::NAN,
            n: 0,
        };
    }
    let n = v.len();
    let mean = v.iter().sum::<f64>() / n as f64;
    let var = v.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;
    Plateau {
        mean_v: mean,
        min_v: v.iter().cloned().fold(f64::INFINITY, f64::min),
        max_v: v.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
        std_v: var.sqrt(),
        n,
    }
}

/// Least-squares line of measured volts against output code.
///
/// Against the *code*, not against the commanded voltage: the code is what the
/// hardware was actually told to do, so a level the firmware clamped or
/// rounded sits on the line instead of pulling it.
fn fit_line(points: &[DcPoint], ch: usize) -> Option<Calibration> {
    let xy: Vec<(f64, f64)> = points
        .iter()
        .filter(|p| p.measured[ch].n > 0)
        .map(|p| (p.codes[ch] as f64, p.measured[ch].mean_v))
        .collect();
    if xy.len() < 2 {
        return None;
    }
    let n = xy.len() as f64;
    let mx = xy.iter().map(|(x, _)| x).sum::<f64>() / n;
    let my = xy.iter().map(|(_, y)| y).sum::<f64>() / n;
    let sxx: f64 = xy.iter().map(|(x, _)| (x - mx).powi(2)).sum();
    if sxx <= 0.0 {
        return None;
    }
    let sxy: f64 = xy.iter().map(|(x, y)| (x - mx) * (y - my)).sum();
    let slope = sxy / sxx;
    let intercept = my - slope * mx;
    let max_deviation_v = xy
        .iter()
        .map(|(x, y)| (y - (slope * x + intercept)).abs())
        .fold(0.0, f64::max);
    Some(Calibration {
        volts_per_code: slope,
        offset_v: intercept,
        max_deviation_v,
        points: xy.len(),
    })
}

fn wrap_deg(mut deg: f64) -> f64 {
    while deg > 180.0 {
        deg -= 360.0;
    }
    while deg < -180.0 {
        deg += 360.0;
    }
    deg
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_expected_response_is_flat_where_the_plant_lives_and_lags_where_it_does_not() {
        // A 1.4 Hz rotor mode is nowhere near either pole or the tick.
        let slow = expected_response(1.4, 1000.0);
        assert!(slow.gain_db.abs() < 0.01, "{slow:?}");
        assert!(slow.phase_deg.abs() < 0.5, "{slow:?}");

        // At 100 Hz the filter is still almost transparent in gain and the
        // hold costs most of the phase: half a tick is 18 degrees, the two
        // poles (796 and 4775 Hz) another 8.4.
        let fast = expected_response(100.0, 1000.0);
        assert!((-0.3..0.0).contains(&fast.gain_db), "{fast:?}");
        assert!(
            (fast.phase_deg + 26.4).abs() < 1.0,
            "expected ~-26.4 deg, got {fast:?}"
        );
    }

    #[test]
    fn a_fit_recovers_the_amplitude_and_the_skew_that_were_put_in() {
        let fs = 2000.0;
        let freq = 10.0;
        let n = 4000;
        let t: Vec<f64> = (0..n).map(|i| i as f64 / fs).collect();
        let lag = 0.3_f64;
        let a: Vec<f64> = t
            .iter()
            .map(|ti| 0.5 + 0.4 * (std::f64::consts::TAU * freq * ti).cos())
            .collect();
        let b: Vec<f64> = t
            .iter()
            .map(|ti| 0.5 + 0.4 * (std::f64::consts::TAU * freq * ti - lag).cos())
            .collect();

        let fa = sine_fit(&t, &a, freq);
        let fb = sine_fit(&t, &b, freq);
        assert!((fa.amplitude - 0.4).abs() < 1e-6, "{fa:?}");
        assert!((fa.offset - 0.5).abs() < 1e-6, "{fa:?}");
        let skew = wrap_deg((fb.phase_rad - fa.phase_rad).to_degrees());
        assert!(
            (skew - lag.to_degrees()).abs() < 0.01,
            "expected {} deg of skew, got {skew}",
            lag.to_degrees()
        );
    }

    #[test]
    fn the_calibration_fits_the_line_the_points_lie_on() {
        let point = |code: u8, v: f64| DcPoint {
            commanded_v: [v as f32; 2],
            codes: [code, code],
            applied_v: [v as f32; 2],
            measured: [Plateau {
                mean_v: v,
                min_v: v,
                max_v: v,
                std_v: 0.0,
                n: 10,
            }; 2],
        };
        let (slope, offset) = (0.004022, 0.002);
        let points: Vec<DcPoint> = (0..6)
            .map(|i| {
                let code = i * 50;
                point(code as u8, offset + slope * code as f64)
            })
            .collect();

        let cal = fit_line(&points, 0).expect("six points make a line");
        assert!((cal.volts_per_code - slope).abs() < 1e-9, "{cal:?}");
        assert!((cal.offset_v - offset).abs() < 1e-9, "{cal:?}");
        assert!(cal.max_deviation_v < 1e-9, "{cal:?}");
    }

    #[test]
    fn a_channel_that_does_not_exist_is_refused_before_anything_is_driven() {
        assert!(checked_channels(&[0, 1]).is_ok());
        assert!(checked_channels(&[]).is_err());
        assert!(checked_channels(&[2]).is_err());
        // Repeats are a typo, not an error: driving twice is driving once.
        assert_eq!(checked_channels(&[1, 1, 0]).unwrap(), vec![0, 1]);
    }
}
