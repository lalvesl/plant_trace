//! The excitation waveforms, evaluated identically on the ESP32 and the host.
//!
//! The generator runs on the device so that the timing comes from a hardware
//! tick rather than from USB latency, but the analysis needs the *same*
//! sequence — particularly for the PRBS, where the host has to correlate
//! against the exact bit pattern that was applied. One implementation, used by
//! both, is the only way those two can be guaranteed equal.
//!
//! Every level is in volts at the plant input; clamping to the safe window is
//! the caller's job (see [`crate::scale::DacScale`]).

use serde::{Deserialize, Serialize};

use core::f32::consts::TAU;

/// An excitation applied to one channel.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Waveform {
    /// Constant level. Runs until replaced.
    Hold {
        /// Output level, volts.
        level: f32,
    },
    /// Linear ramp, then hold at `to`.
    Ramp {
        /// Starting level, volts.
        from: f32,
        /// Final level, volts.
        to: f32,
        /// Time to get there, seconds.
        duration_s: f32,
    },
    /// Staircase for the static gain curves: `steps` plateaus of `dwell_s`.
    Staircase {
        /// First plateau, volts.
        start: f32,
        /// Increment between plateaus, volts (negative walks down).
        step: f32,
        /// Number of plateaus, including the first.
        steps: u16,
        /// Time spent on each plateau, seconds.
        dwell_s: f32,
    },
    /// Single-frequency excitation for one Bode point.
    Sine {
        /// Operating point the sine rides on, volts.
        center: f32,
        /// Peak amplitude, volts.
        amplitude: f32,
        /// Frequency, hertz.
        freq_hz: f32,
        /// Number of periods to run; 0 runs until stopped.
        cycles: u32,
    },
    /// Exponential (constant-per-octave) sweep.
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
    /// Maximum-length pseudo-random binary sequence, for a broadband estimate
    /// in a fraction of the time a stepped sine takes.
    Prbs {
        /// Operating point, volts.
        center: f32,
        /// Half the peak-to-peak swing, volts.
        amplitude: f32,
        /// Time per bit, seconds — sets the excited bandwidth.
        bit_s: f32,
        /// LFSR order, 5..=15. Sequence length is `2^order - 1` bits.
        order: u8,
        /// Total run length, seconds.
        duration_s: f32,
    },
}

impl Waveform {
    /// How long the waveform runs, or `None` if it runs until stopped.
    pub fn duration_s(&self) -> Option<f32> {
        match *self {
            Waveform::Hold { .. } => None,
            Waveform::Ramp { duration_s, .. } => Some(duration_s),
            Waveform::Staircase { steps, dwell_s, .. } => Some(steps as f32 * dwell_s),
            Waveform::Sine {
                freq_hz, cycles, ..
            } => {
                if cycles == 0 || freq_hz <= 0.0 {
                    None
                } else {
                    Some(cycles as f32 / freq_hz)
                }
            }
            Waveform::Chirp { duration_s, .. } => Some(duration_s),
            Waveform::Prbs { duration_s, .. } => Some(duration_s),
        }
    }

    /// Lowest and highest level the waveform can command, so the caller can
    /// refuse an excitation that would leave the plant's safe window before
    /// any of it is applied.
    pub fn span(&self) -> (f32, f32) {
        let (lo, hi) = match *self {
            Waveform::Hold { level } => (level, level),
            Waveform::Ramp { from, to, .. } => (from.min(to), from.max(to)),
            Waveform::Staircase {
                start, step, steps, ..
            } => {
                let last = start + step * (steps.saturating_sub(1)) as f32;
                (start.min(last), start.max(last))
            }
            Waveform::Sine {
                center, amplitude, ..
            }
            | Waveform::Chirp {
                center, amplitude, ..
            }
            | Waveform::Prbs {
                center, amplitude, ..
            } => (center - amplitude.abs(), center + amplitude.abs()),
        };
        (lo, hi)
    }
}

/// Maximum-length LFSR taps (Fibonacci form, XOR of the listed bit positions).
/// Maximum-length LFSR tap masks for orders 5..=15.
///
/// The register shifts right and the feedback bit is inserted at the top, so a
/// polynomial `x^n + x^m + 1` taps bits 0 and `n - m` — bit 0 first, which is
/// the mirror of the textbook \"taps at 5, 3\" notation.
const PRBS_TAPS: [u16; 11] = [
    0x0005, // order 5:  x^5 + x^3 + 1
    0x0003, // order 6:  x^6 + x^5 + 1
    0x0003, // order 7:  x^7 + x^6 + 1
    0x001d, // order 8:  x^8 + x^6 + x^5 + x^4 + 1
    0x0011, // order 9:  x^9 + x^5 + 1
    0x0009, // order 10: x^10 + x^7 + 1
    0x0005, // order 11: x^11 + x^9 + 1
    0x0107, // order 12: x^12 + x^11 + x^10 + x^4 + 1
    0x0027, // order 13: x^13 + x^12 + x^11 + x^8 + 1
    0x1007, // order 14: x^14 + x^13 + x^12 + x^2 + 1
    0x0003, // order 15: x^15 + x^14 + 1
];

/// Stateful evaluator for one channel's waveform.
///
/// `sample` must be called with a non-decreasing `t_s`, which is how both the
/// firmware tick and the host's replay use it; the PRBS state machine is the
/// only part that cares.
#[derive(Debug, Clone)]
pub struct Generator {
    wave: Waveform,
    lfsr: u16,
    next_bit: u64,
    level: f32,
}

impl Generator {
    /// Start a generator at `t = 0`.
    pub fn new(wave: Waveform) -> Self {
        let level = match wave {
            Waveform::Hold { level } => level,
            Waveform::Ramp { from, .. } => from,
            Waveform::Staircase { start, .. } => start,
            Waveform::Sine { center, .. }
            | Waveform::Chirp { center, .. }
            | Waveform::Prbs { center, .. } => center,
        };
        Self {
            wave,
            lfsr: prbs_seed(&wave),
            next_bit: 0,
            level,
        }
    }

    /// The waveform being evaluated.
    pub fn waveform(&self) -> Waveform {
        self.wave
    }

    /// Whether `t_s` is past the end of a finite waveform.
    pub fn is_done(&self, t_s: f32) -> bool {
        self.wave.duration_s().is_some_and(|d| t_s >= d)
    }

    /// Output level, in volts, at `t_s` seconds after the waveform started.
    pub fn sample(&mut self, t_s: f32) -> f32 {
        let t = if t_s < 0.0 { 0.0 } else { t_s };
        self.level = match self.wave {
            Waveform::Hold { level } => level,

            Waveform::Ramp {
                from,
                to,
                duration_s,
            } => {
                if duration_s <= 0.0 || t >= duration_s {
                    to
                } else {
                    from + (to - from) * (t / duration_s)
                }
            }

            Waveform::Staircase {
                start,
                step,
                steps,
                dwell_s,
            } => {
                let idx = if dwell_s <= 0.0 {
                    0
                } else {
                    libm::floorf(t / dwell_s) as i64
                };
                let idx = idx.clamp(0, steps.saturating_sub(1) as i64);
                start + step * idx as f32
            }

            Waveform::Sine {
                center,
                amplitude,
                freq_hz,
                ..
            } => center + amplitude * libm::sinf(TAU * freq_hz * t),

            Waveform::Chirp {
                center,
                amplitude,
                f0_hz,
                f1_hz,
                duration_s,
            } => center + amplitude * libm::sinf(chirp_phase(f0_hz, f1_hz, duration_s, t)),

            Waveform::Prbs {
                center,
                amplitude,
                bit_s,
                order,
                ..
            } => {
                let want = if bit_s <= 0.0 {
                    0
                } else {
                    libm::floorf(t / bit_s) as i64
                };
                let want = want.max(0) as u64;
                // Advance the LFSR to the requested bit. Monotonic `t` keeps
                // this O(1) per call; a rewind restarts the sequence rather
                // than silently returning a stale bit.
                if want < self.next_bit {
                    self.lfsr = prbs_seed(&self.wave);
                    self.next_bit = 0;
                }
                let n = order.clamp(PRBS_MIN_ORDER, PRBS_MAX_ORDER);
                let taps = prbs_taps(n);
                let mask = (1u16 << n) - 1;
                while self.next_bit <= want {
                    let feedback = (self.lfsr & taps).count_ones() & 1;
                    self.lfsr = ((self.lfsr >> 1) | ((feedback as u16) << (n - 1))) & mask;
                    self.next_bit += 1;
                }
                if self.lfsr & 1 == 1 {
                    center + amplitude
                } else {
                    center - amplitude
                }
            }
        };
        self.level
    }

    /// Last level produced, without advancing time.
    pub fn level(&self) -> f32 {
        self.level
    }
}

/// Narrowest and widest LFSR this supports: 31-bit and 32767-bit sequences.
const PRBS_MIN_ORDER: u8 = 5;
const PRBS_MAX_ORDER: u8 = 15;

fn prbs_taps(order: u8) -> u16 {
    PRBS_TAPS[(order.clamp(PRBS_MIN_ORDER, PRBS_MAX_ORDER) - PRBS_MIN_ORDER) as usize]
}

/// Fixed, register-width seed, so the host replays exactly the sequence the
/// device applied. Any non-zero value gives the same maximal-length sequence
/// at a different phase; this one is arbitrary.
fn prbs_seed(wave: &Waveform) -> u16 {
    let order = match wave {
        Waveform::Prbs { order, .. } => order.clamp(&PRBS_MIN_ORDER, &PRBS_MAX_ORDER),
        _ => &PRBS_MAX_ORDER,
    };
    let seed = 0x2c1du16 & ((1u16 << order) - 1);
    if seed == 0 {
        1
    } else {
        seed
    }
}

/// Phase of an exponential sweep from `f0` to `f1` over `duration`.
///
/// The exponential (rather than linear) law spends equal time per octave,
/// which is what a Bode plot's log frequency axis wants.
fn chirp_phase(f0_hz: f32, f1_hz: f32, duration_s: f32, t: f32) -> f32 {
    if duration_s <= 0.0 || f0_hz <= 0.0 || (f1_hz - f0_hz).abs() < 1e-9 {
        return TAU * f0_hz * t;
    }
    let k = libm::logf(f1_hz / f0_hz);
    TAU * f0_hz * duration_s / k * (libm::expf(k * t / duration_s) - 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(wave: Waveform, fs: f32, n: usize) -> Vec<f32> {
        let mut g = Generator::new(wave);
        (0..n).map(|i| g.sample(i as f32 / fs)).collect()
    }

    #[test]
    fn sine_has_the_right_amplitude_and_period() {
        let v = run(
            Waveform::Sine {
                center: 0.5,
                amplitude: 0.05,
                freq_hz: 2.0,
                cycles: 4,
            },
            1000.0,
            2000,
        );
        let max = v.iter().cloned().fold(f32::MIN, f32::max);
        let min = v.iter().cloned().fold(f32::MAX, f32::min);
        assert!((max - 0.55).abs() < 1e-3, "max {max}");
        assert!((min - 0.45).abs() < 1e-3, "min {min}");
        // A 2 Hz sine crosses its centre every half period. The window is
        // [0, 2 s): the crossings at 0.25 … 1.75 are inside it, the ones at
        // t = 0 and t = 2.0 are its endpoints.
        let crossings = v
            .windows(2)
            .filter(|w| (w[0] - 0.5).signum() != (w[1] - 0.5).signum())
            .count();
        assert_eq!(crossings, 7);
    }

    #[test]
    fn staircase_holds_each_level_for_its_dwell() {
        let v = run(
            Waveform::Staircase {
                start: 0.2,
                step: 0.1,
                steps: 4,
                dwell_s: 1.0,
            },
            100.0,
            500,
        );
        assert!((v[0] - 0.2).abs() < 1e-6);
        assert!((v[150] - 0.3).abs() < 1e-6);
        assert!((v[250] - 0.4).abs() < 1e-6);
        assert!((v[350] - 0.5).abs() < 1e-6);
        // Past the end it stays on the last plateau instead of running away.
        assert!((v[499] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn prbs_is_binary_and_maximal_length() {
        let bit_s = 0.01;
        let order = 7u8;
        let mut g = Generator::new(Waveform::Prbs {
            center: 0.5,
            amplitude: 0.05,
            bit_s,
            order,
            duration_s: 10.0,
        });
        // Sample once per bit, for two full sequence lengths.
        let n = (1 << order) - 1;
        let bits: Vec<bool> = (0..2 * n)
            .map(|i| g.sample(i as f32 * bit_s + bit_s / 2.0) > 0.5)
            .collect();
        assert!(bits.iter().any(|b| *b) && bits.iter().any(|b| !*b));
        // A maximal-length sequence repeats after 2^order - 1 bits.
        assert_eq!(bits[..n], bits[n..]);
        // ...and is balanced to within one bit.
        let ones = bits[..n].iter().filter(|b| **b).count();
        assert_eq!(ones, n.div_ceil(2));
    }

    #[test]
    fn chirp_sweeps_between_its_endpoints() {
        // Instantaneous frequency from the phase derivative at both ends.
        let (f0, f1, dur) = (0.1f32, 10.0f32, 20.0f32);
        let df = |t: f32| {
            let h = 1e-3;
            (chirp_phase(f0, f1, dur, t + h) - chirp_phase(f0, f1, dur, t)) / h / TAU
        };
        assert!((df(0.0) - f0).abs() < 0.02, "start {}", df(0.0));
        assert!((df(dur) - f1).abs() < 0.2, "end {}", df(dur));
    }

    #[test]
    fn span_bounds_every_waveform() {
        let (lo, hi) = Waveform::Staircase {
            start: 0.2,
            step: 0.1,
            steps: 4,
            dwell_s: 1.0,
        }
        .span();
        assert!((lo - 0.2).abs() < 1e-6 && (hi - 0.5).abs() < 1e-6);

        let (lo, hi) = Waveform::Sine {
            center: 0.5,
            amplitude: 0.05,
            freq_hz: 1.0,
            cycles: 0,
        }
        .span();
        assert!((lo - 0.45).abs() < 1e-6 && (hi - 0.55).abs() < 1e-6);
    }
}
