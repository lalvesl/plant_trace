//! CSV output.
//!
//! One file per recording, with a commented metadata header. The comments are
//! there so a file found a year later can still be interpreted — which
//! firmware, which front-end scaling, which rate — and because gnuplot, numpy
//! and every spreadsheet skip `#` lines by default.

use std::{
    fs::File,
    io::{BufWriter, Write},
    path::Path,
    time::{SystemTime, UNIX_EPOCH},
};

use anyhow::{Context, Result};
use plant_trace_proto::{scale::AdcScale, Channel, N_CHANNELS};

/// What a recording was.
#[derive(Debug, Clone)]
pub struct RunMeta {
    /// Where the samples came from (link spec).
    pub source: String,
    /// Effective sample rate.
    pub fs_hz: u32,
    /// Counts→volts conversion in force.
    pub adc: AdcScale,
    /// Hardware oversampling factor.
    pub oversample: u16,
    /// Firmware version string.
    pub firmware: String,
    /// Free-form label for the experiment step this file belongs to.
    pub note: Option<String>,
    /// Sample index that `t_s = 0` refers to.
    ///
    /// An experiment records several segments out of one continuous stream;
    /// each file starts its clock at its own first sample, so a step response
    /// plots against time-since-the-step rather than time-since-the-run.
    pub origin_sample: u64,
}

/// Per-channel summary of a recording.
#[derive(Debug, Clone, Copy, Default)]
pub struct ChannelStats {
    /// Smallest value seen, volts.
    pub min_v: f32,
    /// Largest value seen, volts.
    pub max_v: f32,
    /// Mean, volts.
    pub mean_v: f32,
}

/// Summary of a finished recording.
#[derive(Debug, Clone)]
pub struct RunStats {
    /// Rows written.
    pub rows: u64,
    /// Blocks accepted.
    pub blocks: u64,
    /// Blocks missing from the sequence — samples that never reached the host.
    pub gaps: u64,
    /// Per-channel summary, in wire order.
    pub channels: [ChannelStats; N_CHANNELS],
}

/// Streaming CSV writer.
pub struct RunWriter {
    out: BufWriter<File>,
    adc: AdcScale,
    fs_hz: u32,
    origin_sample: u64,
    rows: u64,
    blocks: u64,
    gaps: u64,
    next_seq: Option<u32>,
    acc: [Acc; N_CHANNELS],
}

#[derive(Clone, Copy)]
struct Acc {
    min: f32,
    max: f32,
    sum: f64,
}

impl Default for Acc {
    fn default() -> Self {
        Self {
            min: f32::INFINITY,
            max: f32::NEG_INFINITY,
            sum: 0.0,
        }
    }
}

impl RunWriter {
    /// Create the file and write the metadata header.
    pub fn create(path: &Path, meta: &RunMeta) -> Result<Self> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).ok();
        }
        let file = File::create(path).with_context(|| format!("creating {}", path.display()))?;
        let mut out = BufWriter::new(file);

        writeln!(out, "# plant-trace recording")?;
        writeln!(out, "# started_utc = {}", utc_now())?;
        writeln!(out, "# source = {}", meta.source)?;
        writeln!(out, "# firmware = {}", meta.firmware)?;
        writeln!(out, "# fs_hz = {}", meta.fs_hz)?;
        writeln!(out, "# origin_sample = {}", meta.origin_sample)?;
        writeln!(out, "# oversample = {}", meta.oversample)?;
        writeln!(out, "# adc_full_scale_v = {}", meta.adc.full_scale_v)?;
        writeln!(out, "# adc_bits = {}", meta.adc.bits)?;
        writeln!(out, "# adc_offset_counts = {}", meta.adc.offset_counts)?;
        writeln!(out, "# adc_gain_correction = {}", meta.adc.gain_correction)?;
        writeln!(out, "# volts_per_count = {:e}", meta.adc.volts_per_count())?;
        if let Some(note) = &meta.note {
            writeln!(out, "# note = {note}")?;
        }
        writeln!(
            out,
            "t_s,{}_counts,{}_counts,{}_counts,{}_v,{}_v,{}_v",
            Channel::ValveCmd.name(),
            Channel::SteamPressure.name(),
            Channel::ElectricalPower.name(),
            Channel::ValveCmd.name(),
            Channel::SteamPressure.name(),
            Channel::ElectricalPower.name(),
        )?;

        Ok(Self {
            out,
            adc: meta.adc,
            fs_hz: meta.fs_hz,
            origin_sample: meta.origin_sample,
            rows: 0,
            blocks: 0,
            gaps: 0,
            next_seq: None,
            acc: [Acc::default(); N_CHANNELS],
        })
    }

    /// Append one block.
    ///
    /// Time comes from the block sequence number, so a dropped block leaves a
    /// hole in `t_s` rather than shifting everything after it — which is what
    /// keeps a recording with a gap still usable for identification.
    pub fn push_block(&mut self, seq: u32, n: u16, channels: u8, counts: &[i16]) -> Result<()> {
        let channels = channels as usize;
        anyhow::ensure!(
            channels == N_CHANNELS,
            "expected {N_CHANNELS} channels, the device sent {channels}"
        );
        anyhow::ensure!(
            counts.len() >= n as usize * channels,
            "block {seq} is short: {} counts for {n}×{channels}",
            counts.len()
        );

        if let Some(expected) = self.next_seq {
            if seq > expected {
                self.gaps += (seq - expected) as u64;
            }
        }
        self.next_seq = Some(seq.wrapping_add(1));
        self.blocks += 1;

        let base = seq as u64 * n as u64;
        for i in 0..n as usize {
            let index = (base + i as u64) as i64 - self.origin_sample as i64;
            let t = index as f64 / self.fs_hz as f64;
            write!(self.out, "{t:.6}")?;
            for c in 0..channels {
                write!(self.out, ",{}", counts[i * channels + c])?;
            }
            for c in 0..channels {
                let v = self.adc.to_volts(counts[i * channels + c]);
                let a = &mut self.acc[c];
                a.min = a.min.min(v);
                a.max = a.max.max(v);
                a.sum += v as f64;
                write!(self.out, ",{v:.6}")?;
            }
            writeln!(self.out)?;
            self.rows += 1;
        }
        Ok(())
    }

    /// Flush and summarise.
    pub fn finish(mut self) -> Result<RunStats> {
        self.out.flush()?;
        let mut channels = [ChannelStats::default(); N_CHANNELS];
        for (i, acc) in self.acc.iter().enumerate() {
            channels[i] = ChannelStats {
                min_v: if self.rows == 0 { 0.0 } else { acc.min },
                max_v: if self.rows == 0 { 0.0 } else { acc.max },
                mean_v: if self.rows == 0 {
                    0.0
                } else {
                    (acc.sum / self.rows as f64) as f32
                },
            };
        }
        Ok(RunStats {
            rows: self.rows,
            blocks: self.blocks,
            gaps: self.gaps,
            channels,
        })
    }
}

/// Current UTC time as `YYYY-MM-DDTHH:MM:SSZ`.
///
/// Public because the run manifest and the output directory name use the same
/// stamp as the CSV headers.
///
/// Hand-rolled rather than pulling in a date library for one line of output:
/// the civil-from-days conversion below is Howard Hinnant's, valid for any
/// date after 0000-03-01.
pub fn utc_now() -> String {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    let (y, m, d) = civil_from_days(days);
    let (hh, mm, ss) = (rem / 3600, (rem % 3600) / 60, rem % 60);
    format!("{y:04}-{m:02}-{d:02}T{hh:02}:{mm:02}:{ss:02}Z")
}

fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn civil_from_days_matches_known_dates() {
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(19_000), (2022, 1, 8));
        // 2024 was a leap year: day 60 of it is February 29th.
        assert_eq!(civil_from_days(19_782), (2024, 2, 29));
    }
}
