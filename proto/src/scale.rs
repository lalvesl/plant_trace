//! Counts ↔ volts, on both ends of the rig.
//!
//! Kept in the shared crate so the firmware, the CSV writer and the analysis
//! code cannot drift apart on what a count means.

use serde::{Deserialize, Serialize};

use crate::gen::N_OUTPUTS;

/// SAADC front-end description: internal 0.6 V reference at gain 1/5 gives
/// 3.0 V full scale **at the pin**, which is the setting this rig uses.
///
/// 3.0 V because `u_T` and `P_e` live in the plant's 2.25-2.75 V window (see
/// [`DacScale::U_T_NOMINAL`]); the next gain up, 1/4, would stop at 2.4 V,
/// below the window. The other two channels share the setting and pay for
/// it in resolution: 732 µV per count instead of 293.
///
/// Nordic's transfer function for single-ended input is
/// `RESULT = V * (GAIN / REFERENCE) * 2^RESOLUTION`, so full scale is
/// `REFERENCE / GAIN` and one count is `full_scale / 2^bits` — note the
/// divisor is `2^bits`, not `2^bits - 1`.
///
/// Every channel reaches its pin through a single 10 kΩ series resistor and
/// nothing else. The pin draws no DC, so the resistor drops nothing and the
/// pin sees the plant's voltage unchanged. [`Self::gain_correction`] is
/// therefore 1 by design and carries only what the bench finds the real
/// reference to be. [`Self::to_volts`] returns **volts at the plant**, which
/// are also volts at the pin.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AdcScale {
    /// Input voltage corresponding to a full-scale reading.
    pub full_scale_v: f32,
    /// Resolution in bits.
    pub bits: u8,
    /// Measured offset in counts, subtracted before scaling (0 after a
    /// successful `CALIBRATEOFFSET`).
    pub offset_counts: f32,
    /// Plant volts per pin volt. 1 by design, since nothing between the plant
    /// and the pin divides; a bench calibration folds the real reference's
    /// error in here.
    pub gain_correction: f32,
}

impl AdcScale {
    /// The rig's nominal front end: 3.0 V at the pin, which is 3.0 V at the
    /// plant, 12 bit, uncalibrated.
    pub const NOMINAL: Self = Self {
        full_scale_v: 3.0,
        bits: 12,
        offset_counts: 0.0,
        gain_correction: 1.0,
    };

    /// Volts per count.
    pub fn volts_per_count(&self) -> f32 {
        self.full_scale_v * self.gain_correction / (1u32 << self.bits) as f32
    }

    /// Convert a raw count to volts at the plant.
    pub fn to_volts(&self, counts: i16) -> f32 {
        (counts as f32 - self.offset_counts) * self.volts_per_count()
    }

    /// Highest plant voltage this front end can read before it saturates.
    ///
    /// The number to check a signal against: a step response that rings above
    /// this clips, and a clipped peak looks exactly like a well-damped one.
    pub fn plant_full_scale_v(&self) -> f32 {
        self.full_scale_v * self.gain_correction
    }
}

/// Output channel scaling: what the plant sees for a given 8-bit output code.
///
/// On this rig the code is a PWM duty against a 256-count top, and the analog
/// value is the mean of that square wave after the RC network in
/// `ngspice_filter.cir`. Averaging is linear in the duty and the filter's
/// divider is linear in the amplitude, so `volts = offset_v + code × volts_per_code` holds
/// exactly — including the GPIO's on-resistance, which only moves the two
/// endpoints. That is why a two-point measurement with a DMM is enough, and why
/// these numbers are data rather than constants.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DacScale {
    /// Volts at the plant input per output code.
    pub volts_per_code: f32,
    /// Volts at the plant input for code 0.
    pub offset_v: f32,
    /// Lowest voltage the generator may command.
    pub min_v: f32,
    /// Highest voltage the generator may command.
    pub max_v: f32,
}

/// Unloaded output of one PWM ladder at full duty, volts: `VDD` through
/// 1 k + 1 k over 1 k.
pub const LADDER_FULL_SCALE_V: f32 = 1.1;

/// The `u_T` ladder: the `p_s` ladder with its shunt to ground replaced by a
/// divider off the 3.3 V rail, so the filtered PWM rides on ~2.15 V instead
/// of 0 V. Passive, from the same rail as the PWM.
///
/// ```text
///                          node A                 node B
///   P1.11 ──[1k]──[1k]───────┬──────────[1k]─────────┬──► u_T (plant), and 10k to P0.02
///                            ├──[R_UP 330+330]── 3.3 V
///                            ├──[R_DOWN 1k2+1k2+820]── GND
///                          [100n]                  [100n]
///                           GND                     GND
/// ```
///
/// Node A is the weighted mean of its three branches (Millman): the PWM pin,
/// swinging 0-3.3 V through `R_PWM`, and the divider, which is a
/// [`REF_V`](u_t_stage::REF_V) source behind [`REF_OHMS`](u_t_stage::REF_OHMS).
/// With `k = REF_OHMS / (REF_OHMS + R_PWM)`:
///
/// ```text
/// u_T = (1 − k)·REF_V + k·3.3 V·duty      (2.150 V + 0.709 V·duty)
/// ```
///
/// The divider sits at 2.74 V, not 2.5 V, because the PWM's own mean pulls
/// node A down: at 50 % duty it lands on 2.50 V. The second section carries
/// no DC, so node B is node A.
///
/// An earlier version put a CA3130 summer after the plain ladder; on the 5 V
/// of USB — the bottom of that part's rated supply — its output was erratic,
/// and this network does the same job with nothing to power.
pub mod u_t_stage {
    /// The rail the divider and the PWM both hang off, volts.
    pub const RAIL_V: f32 = 3.3;
    /// Node A to the rail, ohms: 330 + 330.
    pub const R_UP_OHMS: f32 = 330.0 + 330.0;
    /// Node A to ground, ohms: 1k2 + 1k2 + 820.
    pub const R_DOWN_OHMS: f32 = 1200.0 + 1200.0 + 820.0;
    /// PWM pin to node A, ohms: the ladder's 1 k + 1 k.
    pub const R_PWM_OHMS: f32 = 2000.0;
    /// The divider's open-circuit voltage — node A with the PWM pin floating.
    pub const REF_V: f32 = RAIL_V * R_DOWN_OHMS / (R_UP_OHMS + R_DOWN_OHMS);
    /// The divider's source resistance, ohms.
    pub const REF_OHMS: f32 = R_UP_OHMS * R_DOWN_OHMS / (R_UP_OHMS + R_DOWN_OHMS);
    /// Share of node A that follows the PWM.
    pub const PWM_WEIGHT: f32 = REF_OHMS / (REF_OHMS + R_PWM_OHMS);
}

impl DacScale {
    /// An output straight off its PWM ladder, before any bench calibration —
    /// what `p_s` is.
    ///
    /// Full duty puts 1.100 V at the plant: `VDD` through the 1:3 divider
    /// (1 k + 1 k over 1 k), unloaded, because the sense path is a 10 kΩ series
    /// resistor into a high-impedance pin. One code is therefore 4.297 mV and
    /// the highest code (255) reaches 1.0957 V, so the plant's whole 0-1 V
    /// window is inside range.
    ///
    /// **Measure VDD under load before trusting this.** Running from a LiPo
    /// rather than from USB drags the rail — and this whole line with it — as
    /// the cell drains.
    pub const NOMINAL: Self = Self {
        volts_per_code: LADDER_FULL_SCALE_V / 256.0,
        offset_v: 0.0,
        min_v: 0.0,
        max_v: 1.0,
    };

    /// Volts at the plant for code 0 on `u_T`: the divider's share of node A.
    /// See [`u_t_stage`].
    pub const U_T_OFFSET_V: f32 = (1.0 - u_t_stage::PWM_WEIGHT) * u_t_stage::REF_V;

    /// `u_T` as it reaches the plant, through the offset ladder of
    /// [`u_t_stage`]: 2.150 V at code 0, 2.77 mV per code, 2.857 V at code 255.
    ///
    /// The plant takes `u_T` in **2.25-2.75 V**, centred on 2.5 V, and that is
    /// the window: nothing outside it may be commanded. The window is ~180
    /// codes wide, and the ladder cannot leave 2.15-2.86 V whatever the duty —
    /// under the ADC's 3.0 V full scale and the pin's 3.3 V rail.
    ///
    /// Both numbers are nominal, from the resistor values; with 5 % parts
    /// code 0 can sit anywhere in 2.07-2.22 V and code 255 in 2.82-2.89 V,
    /// which still covers the window. `plant-trace check dc` reads node B
    /// back and fits the real line.
    pub const U_T_NOMINAL: Self = Self {
        volts_per_code: u_t_stage::PWM_WEIGHT * u_t_stage::RAIL_V / 256.0,
        offset_v: Self::U_T_OFFSET_V,
        min_v: 2.25,
        max_v: 2.75,
    };

    /// Nominal map of each output, in wire order (`u_T`, `p_s`). The firmware
    /// boots with these, and the host falls back on them where a file does
    /// not say.
    pub const NOMINAL_OUTPUTS: [Self; N_OUTPUTS] = [Self::U_T_NOMINAL, Self::NOMINAL];

    /// Nominal map of output `ch`; an unknown index gets the ladder's.
    pub const fn nominal(ch: usize) -> Self {
        if ch < N_OUTPUTS {
            Self::NOMINAL_OUTPUTS[ch]
        } else {
            Self::NOMINAL
        }
    }

    /// Middle of the safe window, volts — the operating point a check or a
    /// sine defaults to.
    pub fn mid_v(&self) -> f32 {
        (self.min_v + self.max_v) / 2.0
    }

    /// Width of the safe window, volts.
    pub fn span_v(&self) -> f32 {
        self.max_v - self.min_v
    }

    /// Clamp a requested voltage to the safe window.
    pub fn clamp(&self, volts: f32) -> f32 {
        if volts < self.min_v {
            self.min_v
        } else if volts > self.max_v {
            self.max_v
        } else {
            volts
        }
    }

    /// Nearest output code for a requested voltage, after clamping — never a
    /// code whose voltage is outside the window.
    pub fn to_code(&self, volts: f32) -> u8 {
        let v = self.clamp(volts);
        let code = (v - self.offset_v) / self.volts_per_code;
        // `as` saturates at the integer bounds in Rust, and NaN maps to 0.
        let mut code = ((code + 0.5) as i32).clamp(0, u8::MAX as i32);
        // Round inwards at the edges: the window is a promise to the plant,
        // and half a code past it is still past it. On the `u_T` stage half a
        // code is 4.3 mV.
        if self.to_volts(code as u8) > self.max_v + 1e-6 && code > 0 {
            code -= 1;
        } else if self.to_volts(code as u8) < self.min_v - 1e-6 && code < u8::MAX as i32 {
            code += 1;
        }
        code as u8
    }

    /// Voltage the plant actually sees for a code — the quantised value the
    /// generator reports back, not the one that was asked for. It is the *mean*
    /// of the PWM waveform; the carrier ripple riding on it is about 2 mV
    /// peak-to-peak at worst (SPICE, `sim_results/report.md`), which the
    /// SAADC's burst averaging takes back down to roughly a count.
    pub fn to_volts(&self, code: u8) -> f32 {
        self.offset_v + code as f32 * self.volts_per_code
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adc_full_scale_matches_nordic_transfer_function() {
        let s = AdcScale::NOMINAL;
        // 2.500 V at the plant is 2.500 V at the pin, and 2.5 * 4096 / 3.0
        // = 3413 counts.
        assert!(
            (s.to_volts(3413) - 2.5).abs() < 8e-4,
            "{}",
            s.to_volts(3413)
        );
        assert_eq!(s.to_volts(0), 0.0);
    }

    #[test]
    fn the_front_end_has_headroom_for_an_overshoot() {
        // Worth pinning down rather than rediscovering from a clipped step
        // response: a clipped peak is indistinguishable from a well-damped one,
        // so the ringing the assignment asks us to measure has to fit. 3.0 V
        // against a u_T window that ends at 2.75 V leaves half the window
        // again.
        let s = AdcScale::NOMINAL;
        assert!(
            s.plant_full_scale_v()
                >= DacScale::U_T_NOMINAL.max_v + DacScale::U_T_NOMINAL.span_v() / 2.0,
            "readable up to {} V only",
            s.plant_full_scale_v()
        );
    }

    #[test]
    fn dac_round_trips_within_half_a_code() {
        let s = DacScale::NOMINAL;
        for mv in 0..=1000 {
            let want = mv as f32 / 1000.0;
            let got = s.to_volts(s.to_code(want));
            // Half a code, except at the top edge, where rounding goes inwards.
            let slack = if want > s.max_v - s.volts_per_code {
                1.0
            } else {
                0.5
            };
            assert!(
                (got - want).abs() <= s.volts_per_code * slack + 1e-6,
                "{want} -> {got}"
            );
            assert!(got <= s.max_v + 1e-6, "{want} -> {got}");
        }
    }

    #[test]
    fn dac_clamps_instead_of_wrapping() {
        let s = DacScale::NOMINAL;
        assert_eq!(s.to_code(-5.0), 0);
        assert_eq!(s.to_code(99.0), s.to_code(s.max_v));
    }

    #[test]
    fn pwm_full_duty_lands_where_the_divider_says() {
        // 256 counts of duty against the unloaded full scale of 1.100 V.
        let s = DacScale::NOMINAL;
        assert!((s.to_volts(255) - 1.1 * 255.0 / 256.0).abs() < 1e-6);
        // The configured window has to be reachable with an 8-bit code, or
        // every request near the top would be silently clamped.
        assert!(s.max_v <= s.to_volts(255));
    }

    #[test]
    fn the_u_t_stage_centres_its_window_inside_the_duty_range() {
        let s = DacScale::U_T_NOMINAL;
        // 2.150 V at code 0 and 2.77 mV a code, as the divider and the ladder
        // say.
        assert!((s.to_volts(0) - 2.1499).abs() < 1e-3, "{}", s.to_volts(0));
        assert!(
            (s.volts_per_code - 0.0027713).abs() < 1e-6,
            "{}",
            s.volts_per_code
        );
        // The whole window is reachable, ~180 codes of it, and 2.5 V is code 126.
        assert!(s.to_volts(0) < s.min_v && s.to_volts(255) > s.max_v);
        assert!(s.span_v() / s.volts_per_code > 170.0);
        assert_eq!(s.to_code(2.5), 126);
        // Even code 255 stays under the ADC's 3.0 V full scale.
        assert!(s.to_volts(255) < AdcScale::NOMINAL.full_scale_v);
        // Nothing outside the window can be commanded, although the ladder
        // itself reaches past both ends of it.
        assert_eq!(s.to_code(4.0), s.to_code(2.75));
        assert_eq!(s.to_code(0.0), s.to_code(2.25));
        assert!(s.to_volts(s.to_code(2.75)) <= 2.75);
        assert!(s.to_volts(s.to_code(2.25)) >= 2.25);
        assert_eq!(DacScale::nominal(0), s);
        assert_eq!(DacScale::nominal(1), DacScale::NOMINAL);
    }
}
