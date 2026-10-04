//! Wire format and shared numerics for the plant-trace rig.
//!
//! One crate, three consumers: the host CLI, the nRF52840 firmware that *is*
//! the rig, and the archived ESP32 signal generator. It therefore stays
//! `no_std` by default and pulls in `std` only for the host's convenience
//! impls.
//!
//! The link is framed as `COBS(postcard(msg) ++ CRC16) ++ 0x00`, which gives
//! self-synchronising framing (any `0x00` is a frame boundary and nothing else
//! can contain one) plus corruption detection that postcard alone does not
//! provide — a flipped bit inside a sample payload would otherwise decode into
//! a perfectly plausible wrong number.
#![cfg_attr(not(feature = "std"), no_std)]
#![deny(missing_docs)]

pub mod daq;
pub mod frame;
pub mod gen;
pub mod scale;
pub mod waveform;

/// Bumped whenever a message changes shape. The host refuses to talk to a
/// firmware reporting a different value rather than mis-decoding it.
///
/// v2 folded the generator commands into [`daq::HostToDaq`], when the outputs
/// moved from the ESP32's DACs to the nRF's PWM pairs.
pub const PROTOCOL_VERSION: u16 = 2;

/// Number of analog channels the DAQ acquires, in wire order:
/// `0 = u_T`, `1 = p_s`, `2 = P_e`.
pub const N_CHANNELS: usize = 3;

/// Wire order of the acquired channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    /// Valve command as measured at the plant input.
    ValveCmd = 0,
    /// Steam pressure (the disturbance) as measured at the plant input.
    SteamPressure = 1,
    /// Electrical power delivered to the grid — the output of interest.
    ElectricalPower = 2,
}

impl Channel {
    /// Short column name used in CSV headers and in experiment files.
    pub const fn name(self) -> &'static str {
        match self {
            Channel::ValveCmd => "u_t",
            Channel::SteamPressure => "p_s",
            Channel::ElectricalPower => "p_e",
        }
    }

    /// All channels in wire order.
    pub const ALL: [Channel; N_CHANNELS] = [
        Channel::ValveCmd,
        Channel::SteamPressure,
        Channel::ElectricalPower,
    ];

    /// When, within one scan, this channel is actually converted — see
    /// [`scan_offset_s`].
    pub const fn scan_offset_s(self) -> f64 {
        scan_offset_s(self as usize)
    }
}

/// Time between the conversions of two consecutive channels inside one SAADC
/// scan, seconds — **measured on the bench**, 89.5 µs.
///
/// A "sample" on the wire is one row of three values that the host treats as
/// simultaneous. They are not: the SAADC converts the channels **in
/// sequence**, and in burst mode each channel takes its whole oversampling
/// burst before the next one starts. With the firmware's settings
/// (`firmware/nrf-daq/src/config.rs`: `ACQUISITION_TIME = 10 µs`,
/// `OVERSAMPLE = 8×`) and the SAADC's ~2 µs conversion time that is
/// `8 × (10 µs + 2 µs) = 96 µs` per channel. Each reported value is the mean
/// of its burst, so its effective sampling instant is the burst's centre, and
/// the centres are one burst apart: `p_s` is taken one spacing and `P_e` two
/// after `u_T` in the same row.
///
/// Nothing on the wire says so and the protocol does not change for it; the
/// host compensates where it matters. It matters for phase: a skew `Δt` between
/// two channels reads as a phase of `360·f·Δt` degrees — 6.4° between `u_T`
/// and `P_e` at 100 Hz, 0.1° at the 1.4 Hz rotor mode. Because the later
/// channel is *sampled late but filed early*, the skew shows up as an
/// apparent **lead** of the later channel.
///
/// The 96 µs above is the datasheet estimate; the conversion time is only
/// bounded ("≤ 2 µs") in Nordic's table. The value used here is what a
/// loopback measured on this board (the `P_e` input wired to the `u_T` filter
/// output, `experiments/bode-wire.toml`): a pure-delay fit through the raw
/// phase of 1-100 Hz gave −178.7 µs and −179.0 µs in two runs, rms 0.01°,
/// i.e. 89.4-89.5 µs per channel — a conversion of about 1.2 µs, not 2. See
/// `docs/EXPERIMENTS.md`, *Automatic Bode*.
pub const SCAN_CHANNEL_SPACING_S: f64 = 89.5e-6;

/// Nominal delay of channel `channel` (wire order) behind channel 0 within one
/// scan, seconds: `channel × SCAN_CHANNEL_SPACING_S`.
pub const fn scan_offset_s(channel: usize) -> f64 {
    channel as f64 * SCAN_CHANNEL_SPACING_S
}
