//! The bench calibration: what sits between the plant and the nRF that the
//! firmware cannot know about.
//!
//! Today that is the divider on an input whose signal would otherwise pass the
//! SAADC's 3.0 V full scale — `P_e` reaches ~3.5 V. Measure its two resistors
//! with a DMM and write them down once:
//!
//! ```toml
//! [inputs.p_e]
//! divider_top_ohms = 9900     # plant to the tap
//! divider_bottom_ohms = 9800  # tap to ground
//! ```
//!
//! or give the gain directly (`gain = 2.0102`, plant volts per pin volt). The
//! file is `calibration.toml` in the working directory — the repository root,
//! where `experiments/` and `data/` already have to be — and
//! `PLANT_TRACE_CALIBRATION` names another. Without a file every input is
//! direct, as the rig was built. Not searched for upwards on purpose: the
//! tests run from `host/plant-trace` against a simulated rig that has no
//! divider, and must not pick up the bench's.
//!
//! The SAADC itself has an error too: its 0.6 V reference is only trimmed to a
//! percent or so. The firmware zeroes the offset at power-up
//! (`CALIBRATEOFFSET`); the gain is found against a DMM on the same node — set
//! an output, read both — and written down too, the same for every channel
//! since they share the reference and the gain setting:
//!
//! ```toml
//! [adc]
//! gain_correction = 0.99163   # DMM volts / ADC volts
//! offset_mv = 0.0             # what a grounded pin reads, at the pin
//! ```
//!
//! [`crate::daq::Daq::open`] loads it, and every counts→volts conversion —
//! recordings, experiments, Bode, checks, the GUI — goes through the
//! [`AdcScale`](plant_trace_proto::scale::AdcScale) it ends up in, so a
//! channel is never scaled twice or forgotten once.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use anyhow::{bail, ensure, Context, Result};
use plant_trace_proto::{scale::AdcScale, Channel, N_CHANNELS};
use serde::{Deserialize, Serialize};

/// File name searched for.
pub const FILE: &str = "calibration.toml";

/// Environment variable that points at a calibration file elsewhere.
pub const ENV: &str = "PLANT_TRACE_CALIBRATION";

/// Contents of `calibration.toml`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Calibration {
    /// Per input channel, keyed by its name (`u_t`, `p_s`, `p_e`).
    #[serde(default)]
    pub inputs: BTreeMap<String, InputCalibration>,
    /// The SAADC's own error, the same on every channel.
    #[serde(default)]
    pub adc: AdcCalibration,
}

/// The SAADC against a DMM: `true = (read − offset) × gain_correction`, at the
/// pin, before any divider.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AdcCalibration {
    /// DMM volts per ADC volt.
    #[serde(default = "unity")]
    pub gain_correction: f64,
    /// What the ADC reads with its pin at 0 V, millivolts.
    #[serde(default)]
    pub offset_mv: f64,
}

fn unity() -> f64 {
    1.0
}

impl Default for AdcCalibration {
    fn default() -> Self {
        Self {
            gain_correction: 1.0,
            offset_mv: 0.0,
        }
    }
}

/// Everything the bench calibration changes in the counts→volts conversion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrontEnd {
    /// Plant volts per pin volt, per channel in [`Channel`] order.
    pub input_gain: [f32; N_CHANNELS],
    /// DMM volts per ADC volt.
    pub gain_correction: f32,
    /// What a grounded pin reads, volts.
    pub offset_v: f32,
}

impl FrontEnd {
    /// No calibration: every input direct, the ADC taken at its word.
    pub const DIRECT: Self = Self {
        input_gain: [1.0; N_CHANNELS],
        gain_correction: 1.0,
        offset_v: 0.0,
    };

    /// `adc` — the firmware's nominal conversion — with this calibration
    /// folded in.
    pub fn apply(&self, adc: AdcScale) -> AdcScale {
        let nominal_volts_per_count = adc.full_scale_v / (1u32 << adc.bits) as f32;
        AdcScale {
            gain_correction: self.gain_correction,
            offset_counts: self.offset_v / nominal_volts_per_count,
            input_gain: self.input_gain,
            ..adc
        }
    }
}

/// What sits in front of one input: a divider, or a gain given directly.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InputCalibration {
    /// Plant to the tap, ohms.
    pub divider_top_ohms: Option<f64>,
    /// Tap to ground, ohms.
    pub divider_bottom_ohms: Option<f64>,
    /// Plant volts per pin volt, instead of the two resistors.
    pub gain: Option<f64>,
}

impl InputCalibration {
    /// Plant volts per pin volt.
    pub fn gain(&self) -> Result<f64> {
        match (self.divider_top_ohms, self.divider_bottom_ohms, self.gain) {
            (Some(top), Some(bottom), None) => {
                ensure!(
                    top >= 0.0 && bottom > 0.0 && top.is_finite() && bottom.is_finite(),
                    "divider_top_ohms = {top}, divider_bottom_ohms = {bottom}: \
                     both must be positive"
                );
                Ok((top + bottom) / bottom)
            }
            (None, None, Some(gain)) => {
                ensure!(
                    gain.is_finite() && gain > 0.0,
                    "gain = {gain} must be positive"
                );
                Ok(gain)
            }
            _ => bail!(
                "give either divider_top_ohms and divider_bottom_ohms, or gain — not both, \
                 not half a divider"
            ),
        }
    }
}

impl Calibration {
    /// Parse and check a calibration file.
    pub fn load(path: &Path) -> Result<Self> {
        let text =
            std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        let cal: Self =
            toml::from_str(&text).with_context(|| format!("parsing {}", path.display()))?;
        cal.front_end()
            .with_context(|| format!("checking {}", path.display()))?;
        Ok(cal)
    }

    /// The file to use: [`ENV`] if set, else [`FILE`] in the working
    /// directory. `None`: there is none, every input is direct.
    pub fn find() -> Option<PathBuf> {
        if let Some(path) = std::env::var_os(ENV) {
            return Some(PathBuf::from(path));
        }
        let path = PathBuf::from(FILE);
        path.is_file().then_some(path)
    }

    /// Load the file [`Self::find`] picks, or the empty calibration.
    pub fn load_default() -> Result<(Self, Option<PathBuf>)> {
        match Self::find() {
            Some(path) => Ok((Self::load(&path)?, Some(path))),
            None => Ok((Self::default(), None)),
        }
    }

    /// The conversion this calibration describes, checked: a gain or an
    /// offset far outside what the part can be off by is a typo, and refused.
    pub fn front_end(&self) -> Result<FrontEnd> {
        let AdcCalibration {
            gain_correction,
            offset_mv,
        } = self.adc;
        ensure!(
            (0.9..=1.1).contains(&gain_correction),
            "[adc] gain_correction = {gain_correction}: the SAADC is off by a few percent at \
             most — DMM volts over ADC volts, near 1"
        );
        ensure!(
            offset_mv.abs() <= 50.0,
            "[adc] offset_mv = {offset_mv}: a few millivolts at most"
        );
        Ok(FrontEnd {
            input_gain: self.input_gain()?,
            gain_correction: gain_correction as f32,
            offset_v: (offset_mv / 1000.0) as f32,
        })
    }

    /// Plant volts per pin volt for every channel, in [`Channel`] order.
    pub fn input_gain(&self) -> Result<[f32; N_CHANNELS]> {
        let mut gains = [1.0; N_CHANNELS];
        for (name, input) in &self.inputs {
            let Some(ch) = Channel::ALL.iter().position(|c| c.name() == name) else {
                bail!(
                    "[inputs.{name}]: no such channel (the inputs are {})",
                    Channel::ALL.map(Channel::name).join(", ")
                );
            };
            gains[ch] = input.gain().with_context(|| format!("[inputs.{name}]"))? as f32;
        }
        Ok(gains)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_measured_divider_becomes_the_gain_of_its_channel_only() {
        let cal: Calibration =
            toml::from_str("[inputs.p_e]\ndivider_top_ohms = 9900\ndivider_bottom_ohms = 9800\n")
                .unwrap();
        let g = cal.input_gain().unwrap();
        assert_eq!(&g[..2], &[1.0, 1.0]);
        assert!((g[2] - 19700.0 / 9800.0).abs() < 1e-6, "{}", g[2]);
    }

    #[test]
    fn a_gain_can_be_given_directly() {
        let cal: Calibration = toml::from_str("[inputs.p_s]\ngain = 1.5\n").unwrap();
        assert_eq!(cal.input_gain().unwrap(), [1.0, 1.5, 1.0]);
    }

    #[test]
    fn no_file_means_every_input_is_direct() {
        assert_eq!(
            Calibration::default().input_gain().unwrap(),
            [1.0; N_CHANNELS]
        );
    }

    #[test]
    fn the_adc_correction_reaches_every_channel_of_the_conversion() {
        let cal: Calibration =
            toml::from_str("[adc]\ngain_correction = 0.99163\n\n[inputs.p_e]\ngain = 2\n").unwrap();
        let adc = cal.front_end().unwrap().apply(AdcScale::NOMINAL);
        // 2.270 V read at the u_T pin is 3100 counts; the DMM said 2.251 V.
        let counts = (2.270 / AdcScale::NOMINAL.volts_per_count()).round() as i16;
        assert!(
            (adc.to_volts(0, counts) - 2.251).abs() < 1e-3,
            "{}",
            adc.to_volts(0, counts)
        );
        assert!((adc.to_volts(2, counts) - 4.502).abs() < 2e-3);

        let cal: Calibration = toml::from_str("[adc]\noffset_mv = 3\n").unwrap();
        let adc = cal.front_end().unwrap().apply(AdcScale::NOMINAL);
        let counts = (0.003 / AdcScale::NOMINAL.volts_per_count()).round() as i16;
        assert!(
            adc.to_volts(1, counts).abs() < 4e-4,
            "{}",
            adc.to_volts(1, counts)
        );
    }

    #[test]
    fn no_file_is_the_direct_front_end() {
        assert_eq!(
            Calibration::default().front_end().unwrap(),
            FrontEnd::DIRECT
        );
        assert_eq!(FrontEnd::DIRECT.apply(AdcScale::NOMINAL), AdcScale::NOMINAL);
    }

    #[test]
    fn a_mistake_is_refused_rather_than_read_as_unity() {
        for bad in [
            "[adc]\ngain_correction = 1.9916\n",
            "[adc]\noffset_mv = 300\n",
        ] {
            let cal: Calibration = toml::from_str(bad).unwrap();
            assert!(cal.front_end().is_err(), "{bad}");
        }
        for bad in [
            "[inputs.pe]\ngain = 2\n",
            "[inputs.p_e]\ndivider_top_ohms = 9900\n",
            "[inputs.p_e]\ngain = 2\ndivider_top_ohms = 1\ndivider_bottom_ohms = 1\n",
            "[inputs.p_e]\ndivider_top_ohms = 9900\ndivider_bottom_ohms = 0\n",
            "[inputs.p_e]\ngain = -2\n",
        ] {
            let cal: Calibration = toml::from_str(bad).unwrap();
            assert!(cal.input_gain().is_err(), "{bad}");
        }
        assert!(toml::from_str::<Calibration>("[inputs.p_e]\ngian = 2\n").is_err());
    }
}
