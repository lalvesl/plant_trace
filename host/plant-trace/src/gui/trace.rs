//! Sample buffers the plots read from.
//!
//! The worker hands over rows of `[u_t, p_s, p_e]` in volts together with the
//! index of the first row in the continuous stream. A [`Trace`] turns that into
//! one `(t, v)` series per channel, keeping time on the stream's own clock so a
//! dropped block shows up as a gap instead of compressing the plot.
//!
//! Each channel lives in an `Arc<Vec<[f64; 2]>>`. The chart takes a clone of
//! the `Arc` for the frame and drops it when the frame ends, so by the next
//! push the buffer is uniquely owned again and `Arc::make_mut` appends in place
//! rather than copying a million points.

use std::sync::Arc;

/// Channel names in block order.
pub const CHANNELS: [&str; 3] = ["u_T", "p_s", "P_e"];

/// A growing (or rolling) record of the three channels.
#[derive(Clone)]
pub struct Trace {
    /// `(t, v)` per channel, t in seconds since the stream started.
    pub series: [Arc<Vec<[f64; 2]>>; 3],
    /// Sample rate of the stream the rows came from.
    pub fs_hz: u32,
    /// Keep only the last this-many seconds; `None` keeps everything.
    pub window_s: Option<f64>,
    /// Index of the next sample expected, to detect gaps.
    next: Option<u64>,
    /// Stream index that maps to t = 0.
    origin: Option<u64>,
}

impl Trace {
    /// An empty record that keeps everything.
    pub fn unbounded() -> Self {
        Self {
            series: Default::default(),
            fs_hz: 0,
            window_s: None,
            next: None,
            origin: None,
        }
    }

    /// An empty record that keeps the last `window_s` seconds.
    pub fn rolling(window_s: f64) -> Self {
        Self {
            window_s: Some(window_s),
            ..Self::unbounded()
        }
    }

    /// Forget every sample, keeping the configuration.
    pub fn clear(&mut self) {
        *self = Self {
            window_s: self.window_s,
            ..Self::unbounded()
        };
    }

    /// Whether nothing has been pushed yet.
    pub fn is_empty(&self) -> bool {
        self.series[0].is_empty()
    }

    /// Time of the newest sample, seconds.
    pub fn last_t(&self) -> f64 {
        self.series[0].last().map_or(0.0, |p| p[0])
    }

    /// Append rows that start at stream index `first`.
    pub fn push(&mut self, first: u64, fs_hz: u32, rows: &[[f32; 3]]) {
        if rows.is_empty() || fs_hz == 0 {
            return;
        }
        if self.fs_hz != fs_hz {
            // A new stream at a different rate is a new time base.
            self.clear();
            self.fs_hz = fs_hz;
        }
        let origin = *self.origin.get_or_insert(first);
        if first < origin {
            // The stream restarted: begin again rather than go backwards.
            self.clear();
            self.fs_hz = fs_hz;
            self.origin = Some(first);
            return self.push(first, fs_hz, rows);
        }
        let dt = 1.0 / fs_hz as f64;
        let gap = self.next.is_some_and(|n| first > n);
        for (ch, series) in self.series.iter_mut().enumerate() {
            let s = Arc::make_mut(series);
            if gap {
                // NaN breaks the line, so a dropped block reads as a hole.
                let t = (first - origin) as f64 * dt;
                s.push([t, f64::NAN]);
            }
            s.extend(
                rows.iter()
                    .enumerate()
                    .map(|(i, r)| [(first - origin + i as u64) as f64 * dt, r[ch] as f64]),
            );
        }
        self.next = Some(first + rows.len() as u64);
        self.trim();
    }

    fn trim(&mut self) {
        let Some(window) = self.window_s else { return };
        let newest = self.last_t();
        for series in &mut self.series {
            let cut = series.partition_point(|p| p[0] < newest - window);
            // Trim in chunks so a rolling view does not shift the vector on
            // every block.
            if cut > 4096 {
                Arc::make_mut(series).drain(..cut);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rows_land_on_the_stream_clock_and_a_gap_is_a_hole() {
        let mut t = Trace::unbounded();
        t.push(0, 1000, &[[1.0, 2.0, 3.0], [1.1, 2.1, 3.1]]);
        t.push(5, 1000, &[[1.5, 2.5, 3.5]]);
        let s = &t.series[2];
        assert_eq!(s.len(), 4);
        assert!((s[1][0] - 0.001).abs() < 1e-12);
        assert!(
            s[2][1].is_nan(),
            "the dropped samples should break the line"
        );
        assert!((s[3][0] - 0.005).abs() < 1e-12);
        assert_eq!(s[3][1], 3.5);
    }

    #[test]
    fn a_rolling_trace_forgets_old_samples() {
        let mut t = Trace::rolling(1.0);
        let rows = vec![[0.0f32; 3]; 1000];
        for k in 0..20 {
            t.push(k * 1000, 1000, &rows);
        }
        let s = &t.series[0];
        assert!(
            s[0][0] >= 20.0 - 1.0 - 5.0,
            "kept too much: first t = {}",
            s[0][0]
        );
        assert!((t.last_t() - 19.999).abs() < 1e-9);
    }
}
