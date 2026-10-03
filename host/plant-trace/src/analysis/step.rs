//! Step-response metrics and the first-order-plus-dead-time fit.
//!
//! These are the numbers §1.3 asks for by name: apparent delay, rise time,
//! settling time, overshoot, and the period and damping of the
//! electromechanical oscillation. The FOPDT fit on top of them is what a
//! controller gets designed against.

use anyhow::{bail, Result};
use serde::Serialize;

use super::{mean, std_dev, Recording};

/// Everything measured from one step.
#[derive(Debug, Clone, Serialize)]
pub struct StepMetrics {
    /// Channel that was stepped.
    pub input: String,
    /// When the input changed, seconds into the recording.
    pub t_step_s: f64,
    /// Input level before and after, volts.
    pub u_before_v: f64,
    /// See `u_before_v`.
    pub u_after_v: f64,
    /// Output before the step and after it settled, volts.
    pub y_before_v: f64,
    /// See `y_before_v`.
    pub y_final_v: f64,
    /// Static gain of the step, V/V.
    pub gain: f64,
    /// Time from the step until the output leaves the noise band, seconds.
    pub apparent_delay_s: f64,
    /// 10 %–90 % rise time, seconds.
    pub rise_time_s: f64,
    /// Time to enter and stay within ±2 % of the final value, seconds.
    pub settling_time_s: f64,
    /// Overshoot beyond the final value, per cent of the step.
    pub overshoot_pct: f64,
    /// Period of the oscillation riding on the response, seconds.
    pub oscillation_period_s: Option<f64>,
    /// Damping ratio from the logarithmic decrement.
    pub damping_ratio: Option<f64>,
    /// Natural frequency implied by the period and the damping, rad/s.
    pub omega_n_rad_s: Option<f64>,
    /// Best first-order-plus-dead-time model of the same step.
    pub fopdt: Fopdt,
    /// Output noise before the step, volts — the floor every threshold above
    /// is measured against.
    pub noise_sd_v: f64,
}

/// A first-order-plus-dead-time model, `K·e^{-Ls}/(1+τs)`.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Fopdt {
    /// Static gain, V/V.
    pub gain: f64,
    /// Time constant, seconds.
    pub tau_s: f64,
    /// Dead time, seconds.
    pub delay_s: f64,
    /// RMS of the fit residual, volts.
    pub rms_error_v: f64,
}

/// Measure the step in `rec`, where `input` is the channel that moved.
pub fn metrics(rec: &Recording, input: &str) -> Result<StepMetrics> {
    let u = rec.channel(input)?.to_vec();
    let y = &rec.p_e;
    let n = rec.len();
    let dt = 1.0 / rec.fs_hz;

    let i_step = step_index(&u).ok_or_else(|| {
        anyhow::anyhow!("no step found on {input}: the channel never changes level")
    })?;
    // Stay a few samples clear of the edge itself when averaging either side.
    let margin = (rec.fs_hz * 0.05) as usize + 1;
    if i_step < 10 * margin || n - i_step < 20 * margin {
        bail!(
            "the step at {:.2} s is too close to the edge of a {:.1} s recording",
            rec.t[i_step],
            rec.duration_s()
        );
    }

    let pre = &y[i_step / 4..i_step.saturating_sub(margin)];
    let u_before = mean(&u[i_step / 4..i_step.saturating_sub(margin)]);
    let u_after = mean(&u[i_step + margin..]);
    let y_before = mean(pre);
    let noise_sd = std_dev(pre);
    let tail = n - (n - i_step) / 10;
    let y_final = mean(&y[tail..]);

    let du = u_after - u_before;
    let dy = y_final - y_before;
    if du.abs() < 1e-6 {
        bail!("the step on {input} is {du:.6} V — too small to divide by");
    }

    let t0 = rec.t[i_step];
    let after = &y[i_step..];
    let sign = dy.signum();

    // Apparent delay: the output has to leave the noise before anything can be
    // said to have happened. Five per cent of the step or three sigma of the
    // pre-step noise, whichever is larger.
    let threshold = (0.05 * dy.abs()).max(3.0 * noise_sd);
    let delay_s = after
        .iter()
        .position(|v| (v - y_before) * sign > threshold)
        .map(|i| i as f64 * dt)
        .unwrap_or(f64::NAN);

    let crossing = |frac: f64| -> Option<f64> {
        let target = y_before + dy * frac;
        after
            .iter()
            .position(|v| (v - target) * sign >= 0.0)
            .map(|i| i as f64 * dt)
    };
    let rise_time_s = match (crossing(0.1), crossing(0.9)) {
        (Some(a), Some(b)) => b - a,
        _ => f64::NAN,
    };

    // Settling: the *last* time it leaves the ±2 % band, not the first time it
    // enters — an oscillation crosses the band several times on the way down.
    let band = 0.02 * dy.abs();
    let settling_time_s = after
        .iter()
        .rposition(|v| (v - y_final).abs() > band)
        .map(|i| (i + 1) as f64 * dt)
        .unwrap_or(0.0);

    let extreme = after
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, |acc, v| acc.max((v - y_final) * sign));
    let overshoot_pct = (extreme / dy.abs() * 100.0).max(0.0);

    let (oscillation_period_s, damping_ratio, omega_n_rad_s) = oscillation(after, dt, noise_sd);

    let fopdt = fit_fopdt(after, y_before, dy, du, dt);

    Ok(StepMetrics {
        input: input.to_string(),
        t_step_s: t0,
        u_before_v: u_before,
        u_after_v: u_after,
        y_before_v: y_before,
        y_final_v: y_final,
        gain: dy / du,
        apparent_delay_s: delay_s,
        rise_time_s,
        settling_time_s,
        overshoot_pct,
        oscillation_period_s,
        damping_ratio,
        omega_n_rad_s,
        fopdt,
        noise_sd_v: noise_sd,
    })
}

/// Index of the largest single change in a channel.
fn step_index(u: &[f64]) -> Option<usize> {
    let mut best = (0usize, 0.0f64);
    for i in 1..u.len() {
        let d = (u[i] - u[i - 1]).abs();
        if d > best.1 {
            best = (i, d);
        }
    }
    (best.1 > 1e-4).then_some(best.0)
}

/// Period, damping ratio and natural frequency of the ringing on a step.
///
/// Finding the peaks directly does not work on this plant: the reheater takes
/// tens of seconds while the rotor mode rings at over a hertz, so the
/// oscillation rides on a slow rise and never crosses the final value. So the
/// frequency comes from the spectrum first, and the amplitudes are measured
/// after removing a moving average one period wide — which is exactly the
/// filter that leaves an oscillation and deletes the trend under it.
fn oscillation(after: &[f64], dt: f64, noise_sd: f64) -> (Option<f64>, Option<f64>, Option<f64>) {
    let Some(period) = dominant_period(after, dt) else {
        return (None, None, None);
    };

    // Detrend: subtract a centred moving average exactly one period long.
    let w = (period / dt).round() as usize;
    if w < 4 || after.len() < 3 * w {
        return (Some(period), None, None);
    }
    let mut detrended = vec![0.0; after.len() - w];
    let mut acc: f64 = after[..w].iter().sum();
    for i in 0..detrended.len() {
        detrended[i] = after[i + w / 2] - acc / w as f64;
        acc += after[i + w] - after[i];
    }

    // Successive maxima, at least most of a period apart.
    let min_gap = (0.6 * period / dt) as usize;
    let mut peaks: Vec<f64> = Vec::new();
    let mut last = 0usize;
    for i in 1..detrended.len() - 1 {
        if detrended[i] > detrended[i - 1]
            && detrended[i] >= detrended[i + 1]
            && detrended[i] > 5.0 * noise_sd
            && (peaks.is_empty() || i - last >= min_gap)
        {
            peaks.push(detrended[i]);
            last = i;
        }
    }
    if peaks.len() < 2 || peaks[1] >= peaks[0] {
        return (Some(period), None, None);
    }

    // Logarithmic decrement by least squares over every peak, not just the
    // first pair: on a lightly damped mode riding on a slow rise, two
    // consecutive peaks differ by little more than the noise, and the
    // two-point estimate then swings between implausible values. The slope of
    // ln(A_k) against k is the same quantity, measured with all the evidence.
    let usable: Vec<(f64, f64)> = peaks
        .iter()
        .enumerate()
        .take_while(|(_, a)| **a > 0.0)
        .map(|(k, a)| (k as f64, a.ln()))
        .collect();
    let decrement = if usable.len() >= 3 {
        let n = usable.len() as f64;
        let mean_k = usable.iter().map(|(k, _)| k).sum::<f64>() / n;
        let mean_l = usable.iter().map(|(_, l)| l).sum::<f64>() / n;
        let num: f64 = usable
            .iter()
            .map(|(k, l)| (k - mean_k) * (l - mean_l))
            .sum();
        let den: f64 = usable.iter().map(|(k, _)| (k - mean_k).powi(2)).sum();
        if den <= 0.0 {
            return (Some(period), None, None);
        }
        let slope = num / den;
        // How much of the peak-to-peak decay the straight line actually
        // explains. A sequence of noise peaks has no trend, fits badly, and
        // would otherwise come back as a confidently tiny damping ratio.
        let ss_res: f64 = usable
            .iter()
            .map(|(k, l)| (l - (mean_l + slope * (k - mean_k))).powi(2))
            .sum();
        let ss_tot: f64 = usable.iter().map(|(_, l)| (l - mean_l).powi(2)).sum();
        let r2 = if ss_tot > 0.0 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };
        if r2 < 0.5 {
            return (Some(period), None, None);
        }
        -slope
    } else {
        (peaks[0] / peaks[1]).ln()
    };
    // Below about ζ = 0.002 the decay over the few visible peaks is smaller
    // than the noise that found them, so the number would be an artefact of
    // the measurement rather than a property of the plant.
    if decrement <= 0.01 {
        return (Some(period), None, None);
    }
    let zeta = decrement / (4.0 * std::f64::consts::PI.powi(2) + decrement.powi(2)).sqrt();
    let omega_n = std::f64::consts::TAU / (period * (1.0 - zeta * zeta).sqrt());
    (Some(period), Some(zeta), Some(omega_n))
}

/// Lowest and highest frequency searched for the electromechanical mode.
///
/// The band starts at 0.2 Hz on purpose: below that is the turbine's own rise
/// — the reheater alone is 0.023 Hz — and its leakage would otherwise win every
/// time. A rotor swinging against a stiff grid lands between roughly 0.2 and
/// 3 Hz for any machine this exercise could involve; the upper bound is loose
/// because nothing else lives up there.
const OSC_BAND_HZ: (f64, f64) = (0.2, 20.0);

/// Dominant oscillation period inside [`OSC_BAND_HZ`], or `None` if nothing
/// there stands out from the rest of the spectrum.
fn dominant_period(after: &[f64], dt: f64) -> Option<f64> {
    let fs = 1.0 / dt;
    let n = after.len().min(16384).next_power_of_two() / 2;
    if n < 256 || after.len() < n {
        return None;
    }

    // Differentiate before transforming. A step response's own rise is a 1/f
    // skirt that buries a resonance an octave above it; differencing flattens
    // that skirt and weights the spectrum by f, which is exactly the wrong
    // thing for noise and exactly the right thing here — so a short moving
    // average goes first to keep the differencing from amplifying the ADC's
    // own hiss.
    let smooth_w = ((0.005 / dt) as usize).max(1);
    let smoothed: Vec<f64> = (0..n)
        .map(|i| {
            let lo = i.saturating_sub(smooth_w / 2);
            let hi = (i + smooth_w / 2 + 1).min(n);
            after[lo..hi].iter().sum::<f64>() / (hi - lo) as f64
        })
        .collect();

    let mut re: Vec<f64> = (0..n)
        .map(|i| {
            let d = if i + 1 < n {
                smoothed[i + 1] - smoothed[i]
            } else {
                0.0
            };
            let w = 0.5 - 0.5 * (std::f64::consts::TAU * i as f64 / n as f64).cos();
            d * w
        })
        .collect();
    let mut im = vec![0.0; n];
    super::freq::fft_for_analysis(&mut re, &mut im);

    let df = fs / n as f64;
    let lo = (OSC_BAND_HZ.0 / df).ceil() as usize;
    let hi = ((OSC_BAND_HZ.1 / df) as usize).min(n / 2 - 2);
    if hi <= lo + 2 {
        return None;
    }

    let mag: Vec<f64> = (lo..hi)
        .map(|k| (re[k] * re[k] + im[k] * im[k]).sqrt())
        .collect();
    let (best, peak) = mag
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))?;
    let mut sorted = mag.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = sorted[sorted.len() / 2];
    if *peak < 5.0 * median.max(1e-30) {
        return None; // no line in the band: this step simply does not ring
    }
    if best == 0 {
        // The maximum sits on the band's own edge, which is what leakage from
        // the slow rise below it looks like — not a resonance.
        return None;
    }

    // Parabolic interpolation across the peak, so the period is not quantised
    // to the bin spacing.
    let k = best + lo;
    let (a, b, c) = (
        mag[best.saturating_sub(1)],
        *peak,
        mag[(best + 1).min(mag.len() - 1)],
    );
    let denom = a - 2.0 * b + c;
    let offset = if denom.abs() > 1e-30 {
        0.5 * (a - c) / denom
    } else {
        0.0
    };
    let freq = (k as f64 + offset) * df;
    (freq > 0.0).then(|| 1.0 / freq)
}

/// Fit `K·(1 − e^{−(t−L)/τ})` to the step response.
///
/// Seeded with the classical two-point construction (the 28.3 % and 63.2 %
/// crossings), then refined on a local grid: the two-point estimate is exact
/// for a pure FOPDT and merely close for a real plant with extra poles, and
/// the refinement is what absorbs that difference.
fn fit_fopdt(after: &[f64], y_before: f64, dy: f64, du: f64, dt: f64) -> Fopdt {
    let sign = dy.signum();
    let crossing = |frac: f64| -> Option<f64> {
        let target = y_before + dy * frac;
        after
            .iter()
            .position(|v| (v - target) * sign >= 0.0)
            .map(|i| i as f64 * dt)
    };
    let (t283, t632) = (crossing(0.283), crossing(0.632));
    let (mut tau, mut delay) = match (t283, t632) {
        (Some(a), Some(b)) if b > a => (1.5 * (b - a), b - 1.5 * (b - a)),
        _ => (after.len() as f64 * dt / 4.0, 0.0),
    };
    tau = tau.max(dt);
    delay = delay.max(0.0);

    let error = |tau: f64, delay: f64| -> f64 {
        let mut acc = 0.0;
        for (i, v) in after.iter().enumerate() {
            let t = i as f64 * dt;
            let model = if t <= delay {
                y_before
            } else {
                y_before + dy * (1.0 - (-(t - delay) / tau).exp())
            };
            acc += (v - model).powi(2);
        }
        (acc / after.len() as f64).sqrt()
    };

    // Coordinate descent on a shrinking grid: two parameters, a smooth
    // surface, and no dependency.
    let mut best = (tau, delay, error(tau, delay));
    let mut span = 0.5;
    for _ in 0..8 {
        let mut improved = false;
        for scale in [1.0 - span, 1.0 + span] {
            let t = (best.0 * scale).max(dt);
            let e = error(t, best.1);
            if e < best.2 {
                best = (t, best.1, e);
                improved = true;
            }
        }
        for shift in [-span * best.0, span * best.0] {
            let d = (best.1 + shift).max(0.0);
            let e = error(best.0, d);
            if e < best.2 {
                best = (best.0, d, e);
                improved = true;
            }
        }
        if !improved {
            span *= 0.5;
        }
    }

    Fopdt {
        gain: dy / du,
        tau_s: best.0,
        delay_s: best.1,
        rms_error_v: best.2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// A recording of a synthetic response to a unit step at `t = 5 s`.
    fn synth(fs: f64, secs: f64, du: f64, f: impl Fn(f64) -> f64) -> Recording {
        let n = (fs * secs) as usize;
        let t_step = 5.0;
        let mut u = Vec::with_capacity(n);
        let mut y = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 / fs;
            u.push(if t < t_step { 0.5 } else { 0.5 + du });
            y.push(if t < t_step { 0.0 } else { f(t - t_step) });
        }
        Recording::from_parts(fs, u, vec![0.8; n], y)
    }

    #[test]
    fn recovers_a_first_order_plant_with_dead_time() {
        let (k, tau, delay, du) = (2.0, 3.0, 0.8, 0.05);
        let rec = synth(200.0, 40.0, du, |t| {
            if t <= delay {
                0.0
            } else {
                k * du * (1.0 - (-(t - delay) / tau).exp())
            }
        });
        let m = metrics(&rec, "u_t").unwrap();

        assert!((m.gain - k).abs() < 0.02, "gain {}", m.gain);
        assert!(
            (m.fopdt.tau_s - tau).abs() / tau < 0.05,
            "tau {}",
            m.fopdt.tau_s
        );
        assert!(
            (m.fopdt.delay_s - delay).abs() < 0.15,
            "delay {}",
            m.fopdt.delay_s
        );
        assert!(
            m.fopdt.rms_error_v < 1e-3,
            "residual {}",
            m.fopdt.rms_error_v
        );
        // A first-order response does not overshoot and rises in 2.2τ.
        assert!(m.overshoot_pct < 1.0, "overshoot {}", m.overshoot_pct);
        assert!(
            (m.rise_time_s - 2.197 * tau).abs() / tau < 0.05,
            "rise {}",
            m.rise_time_s
        );
    }

    #[test]
    fn recovers_the_damping_of_an_oscillatory_step() {
        let (zeta, omega_n, du) = (0.15f64, 2.0 * PI * 1.4, 0.05f64);
        let k = 1.0;
        let omega_d = omega_n * (1.0 - zeta * zeta).sqrt();
        let rec = synth(500.0, 30.0, du, |t| {
            k * du
                * (1.0
                    - (-zeta * omega_n * t).exp() * (omega_d * t).cos()
                    - zeta / (1.0 - zeta * zeta).sqrt()
                        * (-zeta * omega_n * t).exp()
                        * (omega_d * t).sin())
        });
        let m = metrics(&rec, "u_t").unwrap();

        let expected_overshoot = 100.0 * (-PI * zeta / (1.0 - zeta * zeta).sqrt()).exp();
        assert!(
            (m.overshoot_pct - expected_overshoot).abs() < 2.0,
            "overshoot {:.1} %, expected {:.1} %",
            m.overshoot_pct,
            expected_overshoot
        );

        let period = m.oscillation_period_s.expect("no oscillation found");
        assert!(
            (period - 2.0 * PI / omega_d).abs() / period < 0.03,
            "period {period} s, expected {}",
            2.0 * PI / omega_d
        );
        let z = m.damping_ratio.expect("no damping estimate");
        assert!((z - zeta).abs() < 0.03, "damping {z}, expected {zeta}");
        let w = m.omega_n_rad_s.expect("no natural frequency");
        assert!(
            (w - omega_n).abs() / omega_n < 0.05,
            "omega_n {w}, expected {omega_n}"
        );
    }

    #[test]
    fn measures_the_reference_plant_against_its_own_parameters() {
        use plant_model::{Plant, PlantParams};
        let params = PlantParams {
            noise_v: 0.0,
            ..Default::default()
        };
        let fs = 500.0;
        let dt = 1.0 / fs as f32;
        let mut plant = Plant::new(params);
        plant.settle(dt, 0.50, params.p_s_nominal_v, 200.0);

        let (mut u, mut p, mut y) = (vec![], vec![], vec![]);
        for i in 0..(fs as usize * 60) {
            let level = if i < fs as usize * 5 { 0.50 } else { 0.55 };
            y.push(plant.step(dt, level, params.p_s_nominal_v) as f64);
            u.push(level as f64);
            p.push(params.p_s_nominal_v as f64);
        }
        let rec = Recording::from_parts(fs, u, p, y);
        let m = metrics(&rec, "u_t").unwrap();

        // Static gain: the valve's slope at the midpoint of the step.
        let analytic_gain = params.flow_gain(0.525) as f64;
        assert!(
            (m.gain - analytic_gain).abs() / analytic_gain < 0.08,
            "gain {:.3}, valve slope {:.3}",
            m.gain,
            analytic_gain
        );
        // Apparent delay is *not* the transport delay, and the difference is
        // one of the things §1.3 asks about: before the output can move by 5 %
        // of the step it has to get through the transport delay, then the
        // actuator lag, then the steam chest. So it is bounded below by the
        // transport delay alone and above by their sum.
        let transport = params.valve_delay_s as f64;
        let upper = transport + params.valve_tau_s as f64 + params.t_ch_s as f64;
        assert!(
            m.apparent_delay_s > transport,
            "apparent delay {:.3} s is below the transport delay {transport:.3} s",
            m.apparent_delay_s
        );
        assert!(
            m.apparent_delay_s < upper,
            "apparent delay {:.3} s exceeds transport + lag + chest = {upper:.3} s",
            m.apparent_delay_s
        );
        // The FOPDT fit is an approximation of three lags by one, and it
        // shows: the HP stage contributes power immediately, so the best
        // single-lag model takes little or no dead time and pays for it in
        // residual. That residual *is* the "limitation of the local linear
        // model" the report is asked to discuss, so the test pins its size
        // rather than pretending the fit is exact.
        let step_size = (m.y_final_v - m.y_before_v).abs();
        assert!(
            m.fopdt.tau_s > params.t_ch_s as f64 && m.fopdt.tau_s < 4.0 * params.t_rh_s as f64,
            "fitted tau {:.3} s is nowhere near the turbine's {:.1}…{:.1} s lags",
            m.fopdt.tau_s,
            params.t_ch_s,
            params.t_rh_s
        );
        assert!(
            m.fopdt.rms_error_v < 0.2 * step_size,
            "FOPDT residual {:.4} V against a {step_size:.4} V step — that is not a fit",
            m.fopdt.rms_error_v
        );
        // And the rotor mode shows up with roughly the predicted period.
        let (omega_n, zeta) = params.rotor_mode(m.y_final_v as f32);
        let (omega_n, zeta) = (omega_n as f64, zeta as f64);
        let expected = 2.0 * PI / (omega_n * (1.0 - zeta * zeta).sqrt());
        let period = m.oscillation_period_s.expect("no ringing");
        assert!(
            (period - expected).abs() / expected < 0.15,
            "period {period:.3} s, predicted {expected:.3} s"
        );
    }
}
