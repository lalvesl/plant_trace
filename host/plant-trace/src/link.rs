//! The byte pipe to a device, and the framing on top of it.
//!
//! A link is either a serial port or a TCP connection. TCP exists so that
//! `plant-trace simulate` can stand in for the hardware without a virtual
//! serial port: every other part of the CLI is then exercised for real, which
//! is the only way the acquisition path gets tested away from the bench.

use std::{
    io::{self, Read, Write},
    net::TcpStream,
    time::{Duration, Instant},
};

use anyhow::{Context, Result};
use plant_trace_proto::frame;
use serde::Serialize;

/// Default line rate.
///
/// The rig is a native USB CDC device, so this number never reaches a UART and
/// nothing is clocked at it — the port just has to be opened with *some* rate.
/// It stays configurable because a board behind a real USB-serial bridge would
/// care, and because a wrong baud rate is the first thing to suspect when a
/// serial link produces nothing but undecodable frames.
pub const DEFAULT_BAUD: u32 = 460_800;

/// A device endpoint.
pub enum Link {
    /// A real serial port.
    Serial(Box<dyn serialport::SerialPort>),
    /// A TCP connection, used by the simulator.
    Tcp(TcpStream),
}

impl Link {
    /// Open `spec`, which is either `tcp://host:port` or a serial device path
    /// (optionally written `serial:///dev/ttyACM0`).
    pub fn open(spec: &str, baud: u32, timeout: Duration) -> Result<Self> {
        if let Some(addr) = spec.strip_prefix("tcp://") {
            let stream =
                TcpStream::connect(addr).with_context(|| format!("connecting to {addr}"))?;
            stream.set_read_timeout(Some(timeout))?;
            stream.set_write_timeout(Some(timeout))?;
            // Frames are small and latency matters more than packing: without
            // this, Nagle holds a command back waiting for more bytes.
            stream.set_nodelay(true)?;
            Ok(Link::Tcp(stream))
        } else {
            let path = spec.strip_prefix("serial://").unwrap_or(spec);
            let port = serialport::new(path, baud)
                .timeout(timeout)
                // No flow control: there is no UART on the other side. USB
                // already has the only backpressure that matters — an endpoint
                // that is not ready NAKs, and the transfer waits — so asking a
                // CDC port for RTS/CTS would at best toggle a modem line
                // nothing reads, and on some drivers fails the open outright.
                .flow_control(serialport::FlowControl::None)
                .open()
                .with_context(|| format!("opening {path} at {baud} baud"))?;
            Ok(Link::Serial(port))
        }
    }
}

impl Read for Link {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            Link::Serial(p) => p.read(buf),
            Link::Tcp(s) => s.read(buf),
        }
    }
}

impl Write for Link {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self {
            Link::Serial(p) => p.write(buf),
            Link::Tcp(s) => s.write(buf),
        }
    }
    fn flush(&mut self) -> io::Result<()> {
        match self {
            Link::Serial(p) => p.flush(),
            Link::Tcp(s) => s.flush(),
        }
    }
}

/// A link that speaks frames instead of bytes.
pub struct Framed {
    link: Link,
    decoder: frame::Decoder<{ frame::MAX_FRAME }>,
    /// Bytes read from the link but not yet fed to the decoder — a read can
    /// easily contain the tail of one frame and the head of the next.
    pending: Vec<u8>,
    filled: usize,
    cursor: usize,
    scratch: [u8; frame::MAX_FRAME],
    out: [u8; frame::MAX_FRAME],
    /// Frames that failed to decode; a non-zero count on a healthy link means
    /// the baud rate or the wiring is wrong.
    pub bad_frames: u64,
}

impl Framed {
    /// Wrap an open link.
    pub fn new(link: Link) -> Self {
        Self {
            link,
            decoder: frame::Decoder::new(),
            pending: vec![0; 4096],
            filled: 0,
            cursor: 0,
            scratch: [0; frame::MAX_FRAME],
            out: [0; frame::MAX_FRAME],
            bad_frames: 0,
        }
    }

    /// Open a link and wrap it.
    pub fn open(spec: &str, baud: u32, timeout: Duration) -> Result<Self> {
        Ok(Self::new(Link::open(spec, baud, timeout)?))
    }

    /// Encode and send one message.
    pub fn send<T: Serialize>(&mut self, msg: &T) -> Result<()> {
        let n = frame::encode(msg, &mut self.scratch, &mut self.out)
            .map_err(|e| anyhow::anyhow!("encoding a command: {e}"))?;
        self.link.write_all(&self.out[..n])?;
        self.link.flush()?;
        Ok(())
    }

    /// Next complete frame, or `None` if `deadline` passed first.
    ///
    /// The returned bytes are decoded in place, so they stay valid only until
    /// the next call — callers convert to owned values immediately.
    pub fn next_frame(&mut self, deadline: Instant) -> Result<Option<&mut [u8]>> {
        'search: loop {
            // Drain whatever is already buffered before touching the link.
            while self.cursor < self.filled {
                let byte = self.pending[self.cursor];
                self.cursor += 1;
                if self.decoder.push(byte) {
                    break 'search;
                }
            }

            if Instant::now() >= deadline {
                return Ok(None);
            }
            match self.link.read(&mut self.pending) {
                Ok(0) => return Ok(None), // the peer closed the connection
                Ok(n) => {
                    self.filled = n;
                    self.cursor = 0;
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                    ) =>
                {
                    if Instant::now() >= deadline {
                        return Ok(None);
                    }
                }
                Err(e) => return Err(e).context("reading from the link"),
            }
        }
        // Taken after the loop on purpose: a borrow of `self.decoder` created
        // inside it could not outlive the iteration that made it.
        Ok(self.decoder.frame())
    }

    /// Frames received that did not decode.
    pub fn note_bad_frame(&mut self) {
        self.bad_frames += 1;
    }
}
