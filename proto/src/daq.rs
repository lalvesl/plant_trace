//! Messages exchanged with the nRF52840 firmware.
//!
//! The nRF owns the whole rig — it drives both plant inputs and samples all
//! three analog channels — so one link carries both halves. The generator half
//! is nested rather than copied: [`crate::gen`] defines those messages once,
//! and the archived ESP32 firmware speaks them bare on a link of its own.

use serde::{Deserialize, Serialize};

use crate::gen;

/// Commands the host sends to the rig.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum HostToDaq {
    /// Liveness check.
    Ping,
    /// Ask for firmware and front-end description.
    Info,
    /// Begin streaming.
    ///
    /// Both fields accept 0, meaning "whatever the firmware defaults to" —
    /// query [`DaqInfo`] to learn what that is. The block size is fixed at
    /// compile time by the DMA buffers, so any other value is refused rather
    /// than silently rounded.
    Start {
        /// Sample rate per channel in hertz, or 0 for the firmware default.
        fs_hz: u32,
        /// Samples per channel per block, or 0 for the firmware default.
        block_samples: u16,
    },
    /// Stop streaming and report the block counters.
    Stop,
    /// Run the SAADC offset calibration. Only valid while idle.
    Calibrate,
    /// A command for the output half of the rig.
    ///
    /// These are answered while acquisition is running — an excitation has to
    /// start in the middle of a recording, not between two of them.
    Gen(gen::HostToGen),
}

/// Replies and data the rig sends to the host.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DaqToHost<'a> {
    /// Answer to [`HostToDaq::Ping`].
    Pong,
    /// Answer to [`HostToDaq::Info`].
    Info(#[serde(borrow)] DaqInfo<'a>),
    /// Acquisition started at the given effective rate.
    Started {
        /// Effective sample rate after rounding to the timer's resolution.
        fs_hz: u32,
    },
    /// Acquisition stopped; totals for the run that just ended.
    Stopped {
        /// Blocks handed to the link.
        blocks: u32,
        /// Blocks dropped because the link could not keep up.
        dropped: u32,
    },
    /// One block of interleaved samples.
    Block(#[serde(borrow)] SampleBlock<'a>),
    /// Offset calibration finished.
    Calibrated,
    /// The last command could not be carried out.
    Error(DaqError),
    /// Answer to a [`HostToDaq::Gen`].
    Gen(#[serde(borrow)] gen::GenToHost<'a>),
}

/// Static description of the acquisition front end.
///
/// The host writes these fields into the CSV metadata header so a recording
/// can be interpreted years later without knowing which firmware made it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DaqInfo<'a> {
    /// Value of [`crate::PROTOCOL_VERSION`] the firmware was built against.
    pub protocol: u16,
    /// Firmware version string.
    #[serde(borrow)]
    pub firmware: &'a str,
    /// Number of channels in each frame.
    pub channels: u8,
    /// ADC resolution in bits.
    pub bits: u8,
    /// Input voltage that corresponds to full scale, in millivolts.
    pub full_scale_mv: u16,
    /// Oversampling factor applied in hardware (1 = bypassed).
    pub oversample: u16,
    /// Samples per channel in every block this firmware emits.
    pub block_samples: u16,
    /// Highest sample rate the firmware will accept.
    pub max_fs_hz: u32,
}

/// One block of interleaved samples.
///
/// `data` is `n * channels` little-endian `i16` values in channel order, which
/// is the SAADC's own EasyDMA layout — no reshuffling happens on the device.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SampleBlock<'a> {
    /// Block counter since `Start`; gaps mean dropped blocks.
    pub seq: u32,
    /// Samples per channel in this block.
    pub n: u16,
    /// Channels per frame.
    pub channels: u8,
    /// Running count of blocks dropped since `Start`.
    pub dropped: u32,
    /// Interleaved little-endian `i16` samples.
    #[serde(with = "serde_bytes", borrow)]
    pub data: &'a [u8],
}

impl SampleBlock<'_> {
    /// Iterate the raw counts in wire order (frame by frame, channel by
    /// channel).
    pub fn samples(&self) -> impl Iterator<Item = i16> + '_ {
        self.data
            .chunks_exact(2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
    }

    /// Iterate one frame at a time, each frame holding `channels` counts.
    pub fn frames(&self) -> impl Iterator<Item = &[u8]> + '_ {
        self.data.chunks_exact(2 * self.channels as usize)
    }

    /// Count of one channel's sample in `data`, or `None` if out of range.
    pub fn at(&self, frame: usize, channel: usize) -> Option<i16> {
        let off = (frame * self.channels as usize + channel) * 2;
        let b = self.data.get(off..off + 2)?;
        Some(i16::from_le_bytes([b[0], b[1]]))
    }
}

/// Why a command was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DaqError {
    /// The command is not valid while streaming.
    Busy,
    /// The command is only valid while streaming.
    NotRunning,
    /// The requested rate or block size is outside what the firmware supports.
    BadConfig,
}
