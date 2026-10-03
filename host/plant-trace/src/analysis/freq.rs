//! Frequency response: one Bode point per sine, and a broadband estimate from
//! the PRBS.
//!
//! The stepped-sine path is a least-squares fit at a frequency that is known
//! exactly, which is why it tolerates noise and drift so well: everything that
//! is not at that frequency ends up in the residual, and the residual is
//! reported so a bad point is visible rather than plausible.

use std::f64::consts::{PI, TAU};

use anyhow::{bail, Result};
use serde::Serialize;

use super::{mean, solve, Recording};

/// One point of a Bode plot.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct BodePoint {
    /// Excitation frequency, hertz.
    pub freq_hz: f64,
    /// |G|, V/V.
    pub magnitude: f64,
    /// 20·log₁₀|G|.
    pub magnitude_db: f64,
    /// ∠G in degrees; negative is lag.
    pub phase_deg: f64,
    /// Amplitude found on the input channel, volts.
    pub input_amplitude_v: f64,
    /// Amplitude found on the output channel, volts.
    pub output_amplitude_v: f64,
    /// RMS of what the fit could not explain, over the RMS of the output's
    /// own variation. Near 0 is a clean point; above ~0.3 the point is mostly
    /// noise, drift or distortion.
    pub residual_ratio: f64,
}

/// Fit one sinusoid at a known frequency to both channels and divide.
///
/// `skip_fraction` discards the beginning of the record, where the plant is
/// still showing the transient of having been switched on rather than its
/// steady-state response.
pub fn sine_point(
    rec: &Recording,
    input: &str,
    freq_hz: f64,
    skip_fraction: f64,
) -> Result<BodePoint> {
    if freq_hz <= 0.0 {
        bail!("frequency must be positive, got {freq_hz}");
    }
    let skip = ((rec.len() as f64) * skip_fraction.clamp(0.0, 0.9)) as usize;
    let n = rec.len() - skip;
    if (n as f64) < 2.0 * rec.fs_hz / freq_hz {
        bail!(
            "only {:.1} periods of {freq_hz} Hz left after skipping the transient",
            n as f64 * freq_hz / rec.fs_hz
        );
    }

    let t = &rec.t[skip..];
    let u = &rec.channel(input)?[skip..];
    let y = &rec.p_e[skip..];

    let f_u = sine_fit(t, u, freq_hz);
    let f_y = sine_fit(t, y, freq_hz);

    if f_u.amplitude < 1e-6 {
        bail!(
            "the input amplitude at {freq_hz} Hz is {:.2e} V — nothing to divide by",
            f_u.amplitude
        );
    }

    // Each fit reports its own lag; the transfer function's phase is how much
    // *more* the output lags than the input.
    let phase = -(f_y.phase_rad - f_u.phase_rad);
    let phase_deg = wrap_deg(phase.to_degrees());
    let magnitude = f_y.amplitude / f_u.amplitude;

    Ok(BodePoint {
        freq_hz,
        magnitude,
        magnitude_db: 20.0 * magnitude.log10(),
        phase_deg,
        input_amplitude_v: f_u.amplitude,
        output_amplitude_v: f_y.amplitude,
        residual_ratio: f_y.residual_ratio,
    })
}

/// One sinusoid fitted to one channel at a frequency that is known exactly.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct SineFit {
    /// Peak amplitude at the fitted frequency, volts.
    pub amplitude: f64,
    /// Lag of the fitted sinusoid behind `cos(ωt)`, radians, with `t = 0`
    /// wherever the caller's time base puts it. Only differences between
    /// channels mean anything: the host's time base has no fixed relationship
    /// to the tick the device generated the waveform on.
    pub phase_rad: f64,
    /// Constant term — the level the sinusoid rides on, volts.
    pub offset: f64,
    /// Linear term, volts per second: slow drift the fit refuses to read as
    /// amplitude.
    pub drift: f64,
    /// RMS of what the fit could not explain, over the RMS of the signal's own
    /// variation. Noise, distortion and everything else that is not at the
    /// excitation frequency ends up here.
    pub residual_ratio: f64,
}

/// Fit `a·cos(ωt) + b·sin(ωt) + c + d·t` to one channel.
///
/// The constant absorbs the operating point and the ramp absorbs slow drift,
/// so neither leaks into the amplitude the way it would with a plain
/// correlation.
pub fn sine_fit(t: &[f64], x: &[f64], freq_hz: f64) -> SineFit {
    let ([a, b, c, d], residual_ratio) = fit_parts(t, x, freq_hz);
    SineFit {
        amplitude: (a * a + b * b).sqrt(),
        phase_rad: b.atan2(a),
        offset: c,
        drift: d,
        residual_ratio,
    }
}

/// The four coefficients behind [`sine_fit`], and the residual ratio.
fn fit_parts(t: &[f64], x: &[f64], freq_hz: f64) -> ([f64; 4], f64) {
    let w = TAU * freq_hz;
    let basis = |ti: f64| [(w * ti).cos(), (w * ti).sin(), 1.0, ti];

    let mut ata = vec![vec![0.0; 4]; 4];
    let mut atb = vec![0.0; 4];
    for (ti, xi) in t.iter().zip(x) {
        let b = basis(*ti);
        for r in 0..4 {
            for c in 0..4 {
                ata[r][c] += b[r] * b[c];
            }
            atb[r] += b[r] * xi;
        }
    }
    let Some(coef) = solve(ata, atb) else {
        return ([0.0; 4], f64::NAN);
    };

    let mut sse = 0.0;
    for (ti, xi) in t.iter().zip(x) {
        let b = basis(*ti);
        let model: f64 = (0..4).map(|k| coef[k] * b[k]).sum();
        sse += (xi - model).powi(2);
    }
    let m = mean(x);
    let sst: f64 = x.iter().map(|v| (v - m).powi(2)).sum();
    let residual_ratio = if sst > 0.0 { (sse / sst).sqrt() } else { 0.0 };

    ([coef[0], coef[1], coef[2], coef[3]], residual_ratio)
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

/// One frequency of a broadband (PRBS) estimate.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct EtfePoint {
    /// Frequency, hertz.
    pub freq_hz: f64,
    /// |G|, V/V.
    pub magnitude: f64,
    /// 20·log₁₀|G|.
    pub magnitude_db: f64,
    /// ∠G in degrees.
    pub phase_deg: f64,
    /// Magnitude-squared coherence, 0…1. Below ~0.8 the point is not
    /// trustworthy — usually because the excitation had little energy there.
    pub coherence: f64,
}

/// Welch-averaged empirical transfer function estimate.
///
/// `segment` is the FFT length in samples (rounded down to a power of two);
/// segments overlap by half and are Hann-windowed. Averaging is what makes the
/// estimate usable: a single periodogram of a PRBS is unbiased but so noisy
/// that a Bode plot drawn from it says nothing.
pub fn etfe(
    rec: &Recording,
    input: &str,
    segment: usize,
    max_freq_hz: f64,
) -> Result<Vec<EtfePoint>> {
    let u_all = rec.channel(input)?;
    let y_all = &rec.p_e;
    let n_fft = segment.next_power_of_two() / 2;
    if n_fft < 64 || rec.len() < n_fft * 2 {
        bail!(
            "need at least {} samples for a {n_fft}-point estimate, have {}",
            n_fft * 2,
            rec.len()
        );
    }

    let u_mean = mean(u_all);
    let y_mean = mean(y_all);
    let window: Vec<f64> = (0..n_fft)
        .map(|i| 0.5 - 0.5 * (TAU * i as f64 / n_fft as f64).cos())
        .collect();

    let mut suu = vec![0.0; n_fft / 2];
    let mut syy = vec![0.0; n_fft / 2];
    let mut syu = vec![(0.0, 0.0); n_fft / 2];
    let mut segments = 0usize;

    let mut start = 0;
    while start + n_fft <= rec.len() {
        let mut ur: Vec<f64> = (0..n_fft)
            .map(|i| (u_all[start + i] - u_mean) * window[i])
            .collect();
        let mut ui = vec![0.0; n_fft];
        let mut yr: Vec<f64> = (0..n_fft)
            .map(|i| (y_all[start + i] - y_mean) * window[i])
            .collect();
        let mut yi = vec![0.0; n_fft];
        fft(&mut ur, &mut ui);
        fft(&mut yr, &mut yi);

        for k in 0..n_fft / 2 {
            suu[k] += ur[k] * ur[k] + ui[k] * ui[k];
            syy[k] += yr[k] * yr[k] + yi[k] * yi[k];
            // Y · conj(U)
            syu[k].0 += yr[k] * ur[k] + yi[k] * ui[k];
            syu[k].1 += yi[k] * ur[k] - yr[k] * ui[k];
        }
        segments += 1;
        start += n_fft / 2;
    }
    if segments == 0 {
        bail!("no complete segments");
    }

    let df = rec.fs_hz / n_fft as f64;
    let mut points = Vec::new();
    for k in 1..n_fft / 2 {
        let f = k as f64 * df;
        if f > max_freq_hz {
            break;
        }
        if suu[k] <= 0.0 {
            continue;
        }
        let (re, im) = (syu[k].0 / suu[k], syu[k].1 / suu[k]);
        let mag = (re * re + im * im).sqrt();
        let coherence = (syu[k].0.powi(2) + syu[k].1.powi(2)) / (suu[k] * syy[k]).max(1e-30);
        points.push(EtfePoint {
            freq_hz: f,
            magnitude: mag,
            magnitude_db: 20.0 * mag.log10(),
            phase_deg: wrap_deg(im.atan2(re).to_degrees()),
            coherence: coherence.min(1.0),
        });
    }
    Ok(points)
}

/// In-place radix-2 FFT, for the sibling modules.
pub(super) fn fft_for_analysis(re: &mut [f64], im: &mut [f64]) {
    fft(re, im)
}

/// In-place radix-2 Cooley-Tukey FFT. `re.len()` must be a power of two.
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    debug_assert!(n.is_power_of_two());

    // Bit-reversal permutation.
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    let mut len = 2;
    while len <= n {
        let ang = -2.0 * PI / len as f64;
        let (wr, wi) = (ang.cos(), ang.sin());
        let mut i = 0;
        while i < n {
            let (mut cr, mut ci) = (1.0f64, 0.0f64);
            for k in 0..len / 2 {
                let (ur, ui) = (re[i + k], im[i + k]);
                let (vr, vi) = (
                    re[i + k + len / 2] * cr - im[i + k + len / 2] * ci,
                    re[i + k + len / 2] * ci + im[i + k + len / 2] * cr,
                );
                re[i + k] = ur + vr;
                im[i + k] = ui + vi;
                re[i + k + len / 2] = ur - vr;
                im[i + k + len / 2] = ui - vi;
                let next = (cr * wr - ci * wi, cr * wi + ci * wr);
                cr = next.0;
                ci = next.1;
            }
            i += len;
        }
        len <<= 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fft_matches_a_hand_computed_dft() {
        // A pure tone at bin 2 of an 8-point transform.
        let n = 8;
        let mut re: Vec<f64> = (0..n)
            .map(|i| (TAU * 2.0 * i as f64 / n as f64).cos())
            .collect();
        let mut im = vec![0.0; n];
        fft(&mut re, &mut im);
        for k in 0..n {
            let expect = if k == 2 || k == n - 2 {
                n as f64 / 2.0
            } else {
                0.0
            };
            assert!(
                (re[k] - expect).abs() < 1e-9 && im[k].abs() < 1e-9,
                "bin {k}: {} + {}i",
                re[k],
                im[k]
            );
        }
    }

    #[test]
    fn sine_fit_recovers_a_known_gain_and_phase() {
        let (fs, freq, gain, lag_deg) = (1000.0f64, 1.3f64, 0.42f64, -57.0f64);
        let n = (fs * 20.0) as usize;
        let lag = lag_deg.to_radians();
        let u: Vec<f64> = (0..n)
            .map(|i| 0.5 + 0.05 * (TAU * freq * i as f64 / fs).sin())
            .collect();
        let y: Vec<f64> = (0..n)
            .map(|i| {
                // Operating point, drift and noise-free phase-shifted response.
                0.3 + 1e-4 * (i as f64 / fs)
                    + gain * 0.05 * (TAU * freq * i as f64 / fs + lag).sin()
            })
            .collect();
        let rec = Recording::from_parts(fs, u, vec![0.8; n], y);

        let p = sine_point(&rec, "u_t", freq, 0.1).unwrap();
        assert!((p.magnitude - gain).abs() < 1e-3, "|G| = {}", p.magnitude);
        assert!(
            (p.phase_deg - lag_deg).abs() < 0.5,
            "phase {} deg, expected {lag_deg}",
            p.phase_deg
        );
        assert!(p.residual_ratio < 1e-6, "residual {}", p.residual_ratio);
    }

    #[test]
    fn etfe_finds_the_gain_of_a_first_order_filter() {
        // Drive a known first-order filter with a pseudo-random signal and
        // check the estimate at a frequency well inside the excited band.
        let (fs, tau) = (200.0, 0.5);
        let n = 200_000;
        let mut lfsr = 0xACE1u16;
        let mut u = Vec::with_capacity(n);
        let mut y = Vec::with_capacity(n);
        let mut state = 0.0;
        let dt = 1.0 / fs;
        for i in 0..n {
            if i % 20 == 0 {
                let bit = (lfsr ^ (lfsr >> 1)) & 1;
                lfsr = (lfsr >> 1) | (bit << 15);
            }
            let drive = if lfsr & 1 == 1 { 0.05 } else { -0.05 };
            state += dt / tau * (drive - state);
            u.push(0.5 + drive);
            y.push(0.3 + state);
        }
        let rec = Recording::from_parts(fs, u, vec![0.8; n], y);
        let points = etfe(&rec, "u_t", 4096, 10.0).unwrap();

        // |G(f)| = 1/sqrt(1 + (2πfτ)²) where coherence says the estimate means
        // something.
        let mut checked = 0;
        for p in points.iter().filter(|p| p.coherence > 0.9) {
            let expect = 1.0 / (1.0 + (TAU * p.freq_hz * tau).powi(2)).sqrt();
            if expect < 0.05 {
                continue; // deep in the roll-off the numerator is all noise
            }
            assert!(
                (p.magnitude - expect).abs() / expect < 0.25,
                "{:.3} Hz: |G| = {:.3}, expected {:.3}",
                p.freq_hz,
                p.magnitude,
                expect
            );
            checked += 1;
        }
        assert!(checked > 10, "only {checked} usable points");
    }
}
