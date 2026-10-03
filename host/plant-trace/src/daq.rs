//! Session with the rig.
//!
//! One link carries both halves of the nRF firmware: the sample stream and the
//! commands that drive the plant's two inputs. That is why a generator command
//! issued *during* a recording cannot simply throw away what it reads while
//! waiting for its reply — the blocks that arrive in the meantime are queued
//! and handed back by the next [`Daq::next_event`], in order.
//!
//! Wire messages borrow from the decoder's buffer, which is right for the
//! firmware and wrong for a host that wants to keep a block around; everything
//! here is converted to owned values the moment it arrives.

use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

use anyhow::{anyhow, bail, Result};
use plant_trace_proto::{
    daq::{DaqError, DaqToHost, HostToDaq},
    frame,
    gen::{GenError, GenStatus, GenToHost, HostToGen},
    scale::{AdcScale, DacScale},
    waveform::Waveform,
};

use crate::link::Framed;

/// How long to wait for a reply to a command before giving up.
pub const REPLY_TIMEOUT: Duration = Duration::from_millis(1500);

/// Silence that ends [`Daq::open`]'s flush: longer than the gap between two
/// blocks at the slowest rate a leftover stream would plausibly run at.
const FLUSH_QUIET: Duration = Duration::from_millis(150);

/// Owned form of `DaqInfo`.
#[derive(Debug, Clone)]
pub struct Info {
    /// Protocol version the firmware was built against.
    pub protocol: u16,
    /// Firmware version string.
    pub firmware: String,
    /// Channels per frame.
    pub channels: u8,
    /// ADC resolution.
    pub bits: u8,
    /// Input voltage at full scale, millivolts.
    pub full_scale_mv: u16,
    /// Hardware oversampling factor.
    pub oversample: u16,
    /// Samples per channel in every block.
    pub block_samples: u16,
    /// Highest rate the firmware accepts.
    pub max_fs_hz: u32,
}

impl Info {
    /// The counts→volts conversion this firmware implies, before any bench
    /// calibration is applied.
    pub fn adc_scale(&self) -> AdcScale {
        AdcScale {
            full_scale_v: self.full_scale_mv as f32 / 1000.0,
            bits: self.bits,
            ..AdcScale::NOMINAL
        }
    }
}

/// Owned form of `GenInfo` — the output half of the same firmware.
#[derive(Debug, Clone)]
pub struct GenInfo {
    /// Protocol version the firmware was built against.
    pub protocol: u16,
    /// Firmware version string.
    pub firmware: String,
    /// Number of analog outputs.
    pub outputs: u8,
    /// Output resolution in bits.
    pub bits: u8,
    /// Waveform evaluation rate.
    pub tick_hz: u32,
}

/// Owned form of a reply from the output half.
#[derive(Debug, Clone)]
pub enum GenReply {
    /// Liveness reply.
    Pong,
    /// Description of the outputs.
    Info(GenInfo),
    /// The command was carried out.
    Ok,
    /// Current state of both outputs.
    Status(GenStatus),
    /// The command was refused.
    Failed(GenError),
}

/// Owned form of a message from the rig.
#[derive(Debug, Clone)]
pub enum Event {
    /// Liveness reply.
    Pong,
    /// Front-end description.
    Info(Info),
    /// Streaming began at this effective rate.
    Started {
        /// Effective sample rate.
        fs_hz: u32,
    },
    /// Streaming ended.
    Stopped {
        /// Blocks produced.
        blocks: u32,
        /// Blocks dropped by the firmware.
        dropped: u32,
    },
    /// A block of interleaved counts.
    Block {
        /// Block index since `Start`.
        seq: u32,
        /// Samples per channel.
        n: u16,
        /// Channels per frame.
        channels: u8,
        /// Firmware's running drop counter.
        dropped: u32,
        /// `n × channels` interleaved counts.
        counts: Vec<i16>,
    },
    /// Offset calibration finished.
    Calibrated,
    /// The firmware refused a command.
    Failed(DaqError),
    /// A reply from the output half.
    Gen(GenReply),
}

/// An open session with the rig.
pub struct Daq {
    framed: Framed,
    /// Sample blocks that arrived while a command was waiting for its reply.
    /// Dropping them would punch a hole in the recording exactly where an
    /// excitation starts, which is the worst possible place for one.
    pending: VecDeque<Event>,
}

impl Daq {
    /// Connect to `spec` (`tcp://host:port` or a serial device).
    ///
    /// The rig outlives any one session: a host that went away mid-stream —
    /// killed, crashed, unplugged from the far end — leaves the firmware
    /// streaming and replies still in flight. Opening therefore stops the
    /// stream and throws away whatever arrives for a moment, so the first
    /// request of this session gets the first reply of this session.
    pub fn open(spec: &str, baud: u32) -> Result<Self> {
        let mut daq = Self {
            // A short read timeout keeps `next_event` responsive; the deadline
            // logic on top of it is what actually bounds the wait.
            framed: Framed::open(spec, baud, Duration::from_millis(50))?,
            pending: VecDeque::new(),
        };
        daq.flush()?;
        Ok(daq)
    }

    /// Stop any stream a previous session left running and discard everything
    /// in flight, until the link has been quiet for [`FLUSH_QUIET`].
    fn flush(&mut self) -> Result<()> {
        self.send(HostToDaq::Stop)?;
        let give_up = Instant::now() + REPLY_TIMEOUT;
        while Instant::now() < give_up {
            if self.recv(Instant::now() + FLUSH_QUIET)?.is_none() {
                break;
            }
        }
        self.pending.clear();
        Ok(())
    }

    /// Send a command without waiting for anything.
    pub fn send(&mut self, cmd: HostToDaq) -> Result<()> {
        self.framed.send(&cmd)
    }

    /// Next event, queued ones first, or `None` if nothing arrived in time.
    pub fn next_event(&mut self, timeout: Duration) -> Result<Option<Event>> {
        if let Some(event) = self.pending.pop_front() {
            return Ok(Some(event));
        }
        self.recv(Instant::now() + timeout)
    }

    /// Next event straight off the link, ignoring the queue.
    fn recv(&mut self, deadline: Instant) -> Result<Option<Event>> {
        loop {
            let Some(raw) = self.framed.next_frame(deadline)? else {
                return Ok(None);
            };
            match frame::decode::<DaqToHost>(raw) {
                Ok(msg) => return Ok(Some(own(msg))),
                Err(_) => {
                    self.framed.note_bad_frame();
                    if Instant::now() >= deadline {
                        return Ok(None);
                    }
                }
            }
        }
    }

    /// Send a command and wait for its reply, queueing the sample blocks that
    /// arrive in the meantime — a reply is easily stuck behind blocks that were
    /// already in flight when the command went out.
    ///
    /// Only an event `is_reply` accepts counts as the reply. Anything else
    /// that is not a block is a stale answer to an earlier request — one that
    /// timed out, or one from a session that died — and is dropped. Without
    /// that, one late reply shifts every later one by a place: `info()` gets
    /// the answer to the `gen_info()` before it, and so on for the rest of the
    /// session.
    pub fn request(
        &mut self,
        cmd: HostToDaq,
        timeout: Duration,
        is_reply: impl Fn(&Event) -> bool,
    ) -> Result<Event> {
        self.send(cmd)?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            match self.recv(deadline)? {
                Some(block @ Event::Block { .. }) => self.pending.push_back(block),
                Some(event) if is_reply(&event) => return Ok(event),
                Some(stale) => eprintln!("rig: dropping a stale reply: {stale:?}"),
                None => break,
            }
        }
        bail!("the rig did not answer within {timeout:?}")
    }

    /// Ask for the front-end description.
    pub fn info(&mut self) -> Result<Info> {
        match self.request(HostToDaq::Info, REPLY_TIMEOUT, |e| {
            matches!(e, Event::Info(_))
        })? {
            Event::Info(info) => Ok(info),
            other => Err(anyhow!("expected device info, got {other:?}")),
        }
    }

    /// Start streaming; returns the effective sample rate.
    pub fn start(&mut self, fs_hz: u32) -> Result<u32> {
        let cmd = HostToDaq::Start {
            fs_hz,
            block_samples: 0,
        };
        match self.request(cmd, REPLY_TIMEOUT, |e| {
            matches!(e, Event::Started { .. } | Event::Failed(_))
        })? {
            Event::Started { fs_hz } => Ok(fs_hz),
            Event::Failed(e) => Err(anyhow!("the rig refused to start: {e:?}")),
            other => Err(anyhow!("expected Started, got {other:?}")),
        }
    }

    /// Stop streaming, consuming the blocks still in flight.
    pub fn stop(&mut self) -> Result<(u32, u32)> {
        match self.request(HostToDaq::Stop, REPLY_TIMEOUT, |e| {
            matches!(e, Event::Stopped { .. } | Event::Failed(_))
        })? {
            Event::Stopped { blocks, dropped } => Ok((blocks, dropped)),
            other => Err(anyhow!("expected Stopped, got {other:?}")),
        }
    }

    /// Run the SAADC offset calibration.
    pub fn calibrate(&mut self) -> Result<()> {
        match self.request(HostToDaq::Calibrate, Duration::from_secs(5), |e| {
            matches!(e, Event::Calibrated | Event::Failed(_))
        })? {
            Event::Calibrated => Ok(()),
            other => Err(anyhow!("expected Calibrated, got {other:?}")),
        }
    }

    // ── the output half ─────────────────────────────────────────────────────

    /// Send a generator command and wait for a reply of the kind it expects.
    fn gen_request(&mut self, cmd: HostToGen) -> Result<GenReply> {
        let is_reply = move |e: &Event| match (e, cmd) {
            (Event::Gen(GenReply::Info(_)), HostToGen::Info) => true,
            (Event::Gen(GenReply::Status(_)), HostToGen::Status) => true,
            (Event::Gen(GenReply::Pong), HostToGen::Ping) => true,
            (Event::Gen(GenReply::Ok | GenReply::Failed(_)), c) => {
                !matches!(c, HostToGen::Info | HostToGen::Status | HostToGen::Ping)
            }
            _ => false,
        };
        match self.request(HostToDaq::Gen(cmd), REPLY_TIMEOUT, is_reply)? {
            Event::Gen(reply) => Ok(reply),
            other => Err(anyhow!("expected a generator reply, got {other:?}")),
        }
    }

    /// Accept only an acknowledgement, turning a refusal into an error.
    fn gen_ack(&mut self, cmd: HostToGen) -> Result<()> {
        match self.gen_request(cmd)? {
            GenReply::Ok => Ok(()),
            GenReply::Failed(e) => Err(refusal(e)),
            other => Err(anyhow!("expected an acknowledgement, got {other:?}")),
        }
    }

    /// Ask what the output half is.
    pub fn gen_info(&mut self) -> Result<GenInfo> {
        match self.gen_request(HostToGen::Info)? {
            GenReply::Info(i) => Ok(i),
            other => Err(anyhow!("expected output info, got {other:?}")),
        }
    }

    /// Current output levels.
    pub fn gen_status(&mut self) -> Result<GenStatus> {
        match self.gen_request(HostToGen::Status)? {
            GenReply::Status(s) => Ok(s),
            other => Err(anyhow!("expected status, got {other:?}")),
        }
    }

    /// Install the bench-measured volts↔code map for one output.
    pub fn set_scale(&mut self, ch: u8, scale: DacScale) -> Result<()> {
        self.gen_ack(HostToGen::SetScale { ch, scale })
    }

    /// Drive one output to a level now.
    pub fn set_level(&mut self, ch: u8, volts: f32) -> Result<()> {
        self.gen_ack(HostToGen::SetLevel { ch, volts })
    }

    /// Stage a waveform without starting it.
    pub fn program(&mut self, ch: u8, wave: Waveform) -> Result<()> {
        self.gen_ack(HostToGen::Program { ch, wave })
    }

    /// Start every staged waveform on the same tick.
    pub fn gen_start(&mut self) -> Result<()> {
        self.gen_ack(HostToGen::Start)
    }

    /// Freeze the outputs where they are.
    pub fn gen_stop(&mut self) -> Result<()> {
        self.gen_ack(HostToGen::Stop)
    }

    /// Drive both outputs to their configured minimum.
    pub fn park(&mut self) -> Result<()> {
        self.gen_ack(HostToGen::Park)
    }
}

fn refusal(e: GenError) -> anyhow::Error {
    match e {
        GenError::BadChannel => anyhow!("no such output"),
        GenError::OutOfRange => {
            anyhow!("the waveform would leave the safe window configured for that output")
        }
        GenError::Busy => anyhow!("the generator is running a waveform; stop it first"),
    }
}

fn own(msg: DaqToHost<'_>) -> Event {
    match msg {
        DaqToHost::Pong => Event::Pong,
        DaqToHost::Info(i) => Event::Info(Info {
            protocol: i.protocol,
            firmware: i.firmware.to_owned(),
            channels: i.channels,
            bits: i.bits,
            full_scale_mv: i.full_scale_mv,
            oversample: i.oversample,
            block_samples: i.block_samples,
            max_fs_hz: i.max_fs_hz,
        }),
        DaqToHost::Started { fs_hz } => Event::Started { fs_hz },
        DaqToHost::Stopped { blocks, dropped } => Event::Stopped { blocks, dropped },
        DaqToHost::Block(b) => Event::Block {
            seq: b.seq,
            n: b.n,
            channels: b.channels,
            dropped: b.dropped,
            counts: b.samples().collect(),
        },
        DaqToHost::Calibrated => Event::Calibrated,
        DaqToHost::Error(e) => Event::Failed(e),
        DaqToHost::Gen(g) => Event::Gen(match g {
            GenToHost::Pong => GenReply::Pong,
            GenToHost::Ok => GenReply::Ok,
            GenToHost::Status(s) => GenReply::Status(s),
            GenToHost::Error(e) => GenReply::Failed(e),
            GenToHost::Info(i) => GenReply::Info(GenInfo {
                protocol: i.protocol,
                firmware: i.firmware.to_owned(),
                outputs: i.outputs,
                bits: i.bits,
                tick_hz: i.tick_hz,
            }),
        }),
    }
}
