//! `COBS(postcard(msg) ++ CRC16) ++ 0x00` framing.
//!
//! Both directions use the same envelope, so the encoder and the decoder are
//! shared by all three nodes and tested once, on the host.

use serde::{Deserialize, Serialize};

/// Largest encoded frame accepted or produced, in bytes.
///
/// A full sample block is `N_CHANNELS * BLOCK_SAMPLES * 2` bytes of payload
/// plus a small header; 1024 leaves room for the largest block the DAQ emits
/// and for COBS' worst-case overhead of one byte per 254.
pub const MAX_FRAME: usize = 1024;

/// CRC-16/IBM-SDLC (a.k.a. X.25): the same polynomial HDLC uses, chosen for
/// its good short-frame Hamming distance rather than for speed.
const CRC: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_IBM_SDLC);

/// Why a frame could not be encoded or decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The destination buffer was too small for the encoded frame.
    BufferTooSmall,
    /// The frame was not valid COBS.
    Cobs,
    /// The frame was shorter than its CRC.
    Truncated,
    /// The CRC did not match: the frame was corrupted in transit.
    Crc,
    /// The payload was not a valid message of the expected type.
    Postcard,
}

impl From<postcard::Error> for Error {
    fn from(e: postcard::Error) -> Self {
        match e {
            postcard::Error::SerializeBufferFull => Error::BufferTooSmall,
            _ => Error::Postcard,
        }
    }
}

#[cfg(feature = "std")]
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let s = match self {
            Error::BufferTooSmall => "buffer too small",
            Error::Cobs => "invalid COBS framing",
            Error::Truncated => "frame shorter than its CRC",
            Error::Crc => "CRC mismatch",
            Error::Postcard => "payload is not a valid message",
        };
        f.write_str(s)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

/// Encode `msg` into `out`, returning the number of bytes written including
/// the trailing delimiter.
///
/// `scratch` holds the postcard body plus its CRC and must be at least as
/// large as the serialised message plus two bytes.
pub fn encode<T: Serialize>(msg: &T, scratch: &mut [u8], out: &mut [u8]) -> Result<usize, Error> {
    let body_len = postcard::to_slice(msg, scratch)?.len();
    if scratch.len() < body_len + 2 {
        return Err(Error::BufferTooSmall);
    }
    let crc = CRC.checksum(&scratch[..body_len]).to_le_bytes();
    scratch[body_len..body_len + 2].copy_from_slice(&crc);

    let framed = body_len + 2;
    // COBS overhead is one byte per 254 payload bytes, plus the leading
    // overhead byte and the trailing delimiter.
    if out.len() < framed + framed / 254 + 2 {
        return Err(Error::BufferTooSmall);
    }
    let n = cobs::encode(&scratch[..framed], out);
    out[n] = 0;
    Ok(n + 1)
}

/// Verify and strip the envelope of one COBS-decoded-in-place frame body.
///
/// `frame` is the raw bytes between two delimiters.
fn open(frame: &mut [u8]) -> Result<&[u8], Error> {
    let n = cobs::decode_in_place(frame).map_err(|_| Error::Cobs)?;
    if n < 3 {
        return Err(Error::Truncated);
    }
    let (body, crc) = frame[..n].split_at(n - 2);
    if CRC.checksum(body) != u16::from_le_bytes([crc[0], crc[1]]) {
        return Err(Error::Crc);
    }
    Ok(&frame[..n - 2])
}

/// Decode one complete frame (without its delimiter) into a message.
pub fn decode<'a, T: Deserialize<'a>>(frame: &'a mut [u8]) -> Result<T, Error> {
    let body = open(frame)?;
    postcard::from_bytes(body).map_err(Error::from)
}

/// Byte-at-a-time frame reassembler for a stream that may start mid-frame.
///
/// Feed it everything that arrives; it yields one frame body per delimiter and
/// silently resynchronises after corruption, because a lost byte can only ever
/// damage the frame it belongs to.
pub struct Decoder<const N: usize = MAX_FRAME> {
    buf: [u8; N],
    len: usize,
    /// Length of the frame `push` just completed, waiting to be taken.
    complete: Option<usize>,
    /// Frames dropped because they did not fit in `buf` — a stuck counter here
    /// means the two ends disagree about `MAX_FRAME`.
    pub overflows: u32,
}

impl<const N: usize> Default for Decoder<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Decoder<N> {
    /// Create an empty decoder.
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
            complete: None,
            overflows: 0,
        }
    }

    /// Push one received byte; `true` means [`Decoder::frame`] now has one.
    ///
    /// Splitting "a frame finished" from "hand me the frame" is what lets a
    /// caller find the frame inside a read loop and return it afterwards —
    /// a borrow created inside the loop could not outlive it.
    ///
    /// Take the frame before pushing further bytes: the next frame reuses the
    /// same buffer.
    pub fn push(&mut self, byte: u8) -> bool {
        if byte == 0 {
            let len = core::mem::take(&mut self.len);
            // A delimiter with nothing before it is the idle line, not a frame.
            if len > 0 {
                self.complete = Some(len);
                return true;
            }
            return false;
        }
        if self.len == N {
            // Oversized frame: drop it and wait for the next delimiter.
            self.len = 0;
            self.overflows = self.overflows.wrapping_add(1);
            return false;
        }
        self.buf[self.len] = byte;
        self.len += 1;
        false
    }

    /// The frame completed by the last [`Decoder::push`], if it has not been
    /// taken yet. Decode it in place with [`decode`].
    pub fn frame(&mut self) -> Option<&mut [u8]> {
        let len = self.complete.take()?;
        Some(&mut self.buf[..len])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::daq::{DaqToHost, HostToDaq, SampleBlock};

    #[test]
    fn round_trips_a_command() {
        let mut scratch = [0u8; 64];
        let mut out = [0u8; 64];
        let msg = HostToDaq::Start {
            fs_hz: 1000,
            block_samples: 64,
        };
        let n = encode(&msg, &mut scratch, &mut out).unwrap();
        assert_eq!(out[n - 1], 0, "frame must end with the delimiter");
        assert!(!out[..n - 1].contains(&0), "COBS must remove inner zeros");

        let mut dec = Decoder::<64>::new();
        let mut got = None;
        for b in &out[..n] {
            if dec.push(*b) {
                got = Some(decode::<HostToDaq>(dec.frame().unwrap()).unwrap());
            }
        }
        assert_eq!(got, Some(msg));
    }

    #[test]
    fn round_trips_a_sample_block() {
        let data: Vec<u8> = (0..384u16).map(|i| (i & 0xff) as u8).collect();
        let msg = DaqToHost::Block(SampleBlock {
            seq: 7,
            n: 64,
            channels: 3,
            dropped: 0,
            data: &data,
        });
        let mut scratch = [0u8; MAX_FRAME];
        let mut out = [0u8; MAX_FRAME];
        let n = encode(&msg, &mut scratch, &mut out).unwrap();

        let mut dec = Decoder::<MAX_FRAME>::new();
        // The decoded message borrows the decoder's buffer, so anything worth
        // keeping has to be copied out before the next byte is pushed.
        let mut got: Option<(u32, Vec<u8>, usize)> = None;
        for b in &out[..n] {
            if dec.push(*b) {
                match decode::<DaqToHost>(dec.frame().unwrap()).unwrap() {
                    DaqToHost::Block(blk) => {
                        got = Some((blk.seq, blk.data.to_vec(), blk.samples().count()))
                    }
                    other => panic!("expected a block, got {other:?}"),
                }
            }
        }
        let (seq, payload, count) = got.expect("no frame decoded");
        assert_eq!(seq, 7);
        assert_eq!(payload, data);
        assert_eq!(count, 192);
    }

    #[test]
    fn rejects_a_corrupted_frame() {
        let mut scratch = [0u8; 64];
        let mut out = [0u8; 64];
        let msg = HostToDaq::Start {
            fs_hz: 1000,
            block_samples: 64,
        };
        let n = encode(&msg, &mut scratch, &mut out).unwrap();
        // Flip a bit in a literal payload byte. `out[0]` is the COBS overhead
        // byte, so corrupting that would fail framing instead of the CRC —
        // which is a different defence and already covered by the decoder.
        out[2] ^= 0x20;

        let mut dec = Decoder::<64>::new();
        let mut verdict = None;
        for b in &out[..n] {
            if dec.push(*b) {
                verdict = Some(decode::<HostToDaq>(dec.frame().unwrap()));
            }
        }
        assert_eq!(verdict, Some(Err(Error::Crc)));
    }

    #[test]
    fn resynchronises_after_garbage() {
        let mut scratch = [0u8; 64];
        let mut out = [0u8; 64];
        let n = encode(&HostToDaq::Ping, &mut scratch, &mut out).unwrap();

        let mut dec = Decoder::<64>::new();
        // A truncated frame, then a delimiter, then a good frame.
        for b in [0x42, 0x17, 0x99, 0x00] {
            let _ = dec.push(b);
        }
        let mut got = None;
        for b in &out[..n] {
            if dec.push(*b) {
                got = Some(decode::<HostToDaq>(dec.frame().unwrap()).unwrap());
            }
        }
        assert_eq!(got, Some(HostToDaq::Ping));
    }
}
