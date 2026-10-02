//! Bench-specific constants for the generator.
//!
//! **This firmware is kept as an alternative, not as part of the rig.** The two
//! plant inputs are driven by the nRF's PWM pairs now; see `docs/HARDWARE.md`.
//! It still builds and still speaks `proto::gen`, so it is one `SetScale` away
//! from standing in if a DK output ever has to be freed up.

use plant_trace_proto::scale::DacScale;

/// Reported to the host and stored in every run manifest.
pub const FIRMWARE_VERSION: &str = concat!("esp32-sig ", env!("CARGO_PKG_VERSION"));

/// Rate at which waveforms are re-evaluated and written to the DACs.
///
/// 1 kHz is 70× the fastest dynamics of interest (the ~1.4 Hz rotor mode) and
/// leaves the CPU almost entirely idle, so the tick never competes with the
/// command link for time.
pub const TICK_HZ: u64 = 1000;

/// Baud rate of the CP2102 bridge. The commands are tens of bytes, so this is
/// about compatibility rather than throughput.
pub const BAUDRATE: u32 = 115_200;

/// Bits of the ESP32's DAC.
pub const DAC_BITS: u8 = 8;

/// Default volts↔code map, until the host installs the measured one.
///
/// A real 8-bit DAC swinging 0-3.3 V into a 3.3:1 divider, which is a different
/// chain from the nRF's PWM — so this is stated here rather than borrowed from
/// [`DacScale::NOMINAL`], which describes that one.
pub const DEFAULT_SCALE: DacScale = DacScale {
    volts_per_code: 1.0 / 255.0,
    offset_v: 0.0,
    min_v: 0.0,
    max_v: 1.0,
};
