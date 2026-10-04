//! Messages that drive the two analog outputs.
//!
//! The generator owns timing (a hardware tick evaluates the waveform) and the
//! host owns calibration (the volts↔code map measured on the bench), so the
//! device never has to know what a divider is and the host never has to know
//! what an output code is.
//!
//! Two firmwares speak these: the nRF nests them inside [`crate::daq`] so one
//! link carries the whole rig, and the archived ESP32 generator speaks them
//! bare on a link of its own. The messages are identical either way, which is
//! the point of defining them here instead of twice.

use serde::{Deserialize, Serialize};

use crate::{scale::DacScale, waveform::Waveform};

/// Output channels of the generator, in the order the plant expects them.
pub const N_OUTPUTS: usize = 2;

/// Commands the host sends to the generator.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum HostToGen {
    /// Liveness check.
    Ping,
    /// Ask for firmware description.
    Info,
    /// Install the bench-measured volts↔code map and safety limits.
    SetScale {
        /// Output index: 0 = `u_T`, 1 = `p_s`.
        ch: u8,
        /// Calibration for that output.
        scale: DacScale,
    },
    /// Drive one output to a level immediately, cancelling its waveform.
    SetLevel {
        /// Output index.
        ch: u8,
        /// Level in volts at the plant input.
        volts: f32,
    },
    /// Stage a waveform without starting it.
    Program {
        /// Output index.
        ch: u8,
        /// The excitation to apply once started.
        wave: Waveform,
    },
    /// Start every staged waveform on the same tick, so a two-channel
    /// excitation has no skew between its channels.
    Start,
    /// Freeze the outputs where they are.
    Stop,
    /// Drive both outputs to their configured minimum and stop.
    Park,
    /// Ask for the current output levels.
    Status,
}

/// Replies the generator sends to the host.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum GenToHost<'a> {
    /// Answer to [`HostToGen::Ping`].
    Pong,
    /// Answer to [`HostToGen::Info`].
    Info(#[serde(borrow)] GenInfo<'a>),
    /// The command was carried out.
    Ok,
    /// Current state of both outputs.
    Status(GenStatus),
    /// The command was refused.
    Error(GenError),
}

/// Static description of the generator.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct GenInfo<'a> {
    /// Value of [`crate::PROTOCOL_VERSION`] the firmware was built against.
    pub protocol: u16,
    /// Firmware version string.
    #[serde(borrow)]
    pub firmware: &'a str,
    /// Number of analog outputs.
    pub outputs: u8,
    /// Output resolution in bits (8: a PWM duty against a 256-count top on the
    /// nRF, a DAC code on the ESP32).
    pub bits: u8,
    /// Rate at which the waveform is re-evaluated, in hertz.
    pub tick_hz: u32,
}

/// What both outputs are doing right now.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GenStatus {
    /// Milliseconds since the running waveform started.
    pub t_ms: u32,
    /// Level actually applied, per output, after clamping and quantisation.
    pub volts: [f32; N_OUTPUTS],
    /// Code written to each output.
    pub codes: [u8; N_OUTPUTS],
    /// Whether a waveform is running.
    pub running: bool,
}

/// Why a command was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GenError {
    /// No such output index.
    BadChannel,
    /// The waveform would leave the configured safe window.
    OutOfRange,
    /// The command is not valid while a waveform is running.
    Busy,
}
