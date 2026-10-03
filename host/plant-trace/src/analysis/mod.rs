//! Turning recordings into the numbers the report is made of.
//!
//! Everything here works on a [`Recording`] — three channels on a common time
//! base — and produces values with units, not opinions. Each estimator is
//! tested against a signal whose answer is known analytically, because an
//! identification routine that is only ever checked against its own output is
//! checked against nothing.

pub mod freq;
pub mod plot;
pub mod report;
pub mod statics;
pub mod step;

use std::{collections::BTreeMap, path::Path};

use anyhow::{bail, Context, Result};

/// One recorded segment: the three channels plus what the header said.
#[derive(Debug, Clone)]
pub struct Recording {
    /// Sample rate, hertz.
    pub fs_hz: f64,
    /// Time of each sample, seconds, relative to the segment's own origin.
    pub t: Vec<f64>,
    /// Valve command as measured at the plant input, volts.
    pub u_t: Vec<f64>,
    /// Steam pressure as measured at the plant input, volts.
    pub p_s: Vec<f64>,
    /// Electrical power, volts.
    pub p_e: Vec<f64>,
    /// The `# key = value` header lines.
    pub meta: BTreeMap<String, String>,
}

impl Recording {
    /// Read a CSV written by `plant-trace record` or `plant-trace run`.
    pub fn load(path: &Path) -> Result<Self> {
        let text =
            std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        let mut meta = BTreeMap::new();
        let (mut t, mut u_t, mut p_s, mut p_e) = (vec![], vec![], vec![], vec![]);

        for line in text.lines() {
            if let Some(rest) = line.strip_prefix('#') {
                if let Some((k, v)) = rest.split_once('=') {
                    meta.insert(k.trim().to_string(), v.trim().to_string());
                }
                continue;
            }
            if line.starts_with("t_s") || line.trim().is_empty() {
                continue;
            }
            let f: Vec<f64> = line
                .split(',')
                .map(|v| v.trim().parse::<f64>())
                .collect::<Result<_, _>>()
                .with_context(|| format!("parsing a row of {}", path.display()))?;
            if f.len() < 7 {
                bail!("{}: expected 7 columns, found {}", path.display(), f.len());
            }
            t.push(f[0]);
            u_t.push(f[4]);
            p_s.push(f[5]);
            p_e.push(f[6]);
        }

        if t.len() < 2 {
            bail!("{} has no samples", path.display());
        }
        let fs_hz = meta
            .get("fs_hz")
            .and_then(|v| v.parse::<f64>().ok())
            .unwrap_or_else(|| 1.0 / (t[1] - t[0]));

        Ok(Self {
            fs_hz,
            t,
            u_t,
            p_s,
            p_e,
            meta,
        })
    }

    /// Build a recording from raw vectors — used by the tests and by the
    /// simulator-fed checks.
    pub fn from_parts(fs_hz: f64, u_t: Vec<f64>, p_s: Vec<f64>, p_e: Vec<f64>) -> Self {
        let t = (0..u_t.len()).map(|i| i as f64 / fs_hz).collect();
        Self {
            fs_hz,
            t,
            u_t,
            p_s,
            p_e,
            meta: BTreeMap::new(),
        }
    }

    /// Number of samples.
    pub fn len(&self) -> usize {
        self.t.len()
    }

    /// Whether the recording is empty.
    pub fn is_empty(&self) -> bool {
        self.t.is_empty()
    }

    /// Duration, seconds.
    pub fn duration_s(&self) -> f64 {
        self.t.last().copied().unwrap_or(0.0) - self.t.first().copied().unwrap_or(0.0)
    }

    /// The channel named `u_t`, `p_s` or `p_e`.
    pub fn channel(&self, name: &str) -> Result<&[f64]> {
        match name {
            "u_t" => Ok(&self.u_t),
            "p_s" => Ok(&self.p_s),
            "p_e" => Ok(&self.p_e),
            other => bail!("no channel named '{other}' (expected u_t, p_s or p_e)"),
        }
    }
}

/// Mean of a slice; 0 for an empty one.
pub fn mean(xs: &[f64]) -> f64 {
    if xs.is_empty() {
        0.0
    } else {
        xs.iter().sum::<f64>() / xs.len() as f64
    }
}

/// Sample standard deviation; 0 for fewer than two values.
pub fn std_dev(xs: &[f64]) -> f64 {
    if xs.len() < 2 {
        return 0.0;
    }
    let m = mean(xs);
    (xs.iter().map(|x| (x - m).powi(2)).sum::<f64>() / (xs.len() - 1) as f64).sqrt()
}

/// Solve `A x = b` for a small dense system by Gaussian elimination with
/// partial pivoting.
///
/// Every fit here is a handful of unknowns, so this is both fast enough and
/// one fewer dependency than a linear-algebra crate.
pub fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| {
            a[i][col]
                .abs()
                .partial_cmp(&a[j][col].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        if a[pivot][col].abs() < 1e-12 {
            return None; // singular: the fit is not determined by this data
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in (col + 1)..n {
            let factor = a[row][col] / a[col][col];
            let (upper, lower) = a.split_at_mut(row);
            for (target, source) in lower[0][col..n].iter_mut().zip(&upper[col][col..n]) {
                *target -= factor * source;
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let mut acc = b[row];
        for k in (row + 1)..n {
            acc -= a[row][k] * x[k];
        }
        x[row] = acc / a[row][row];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_a_small_system() {
        // 2x + y = 5 ; x - 3y = -8  →  x = 1, y = 3
        let x = solve(vec![vec![2.0, 1.0], vec![1.0, -3.0]], vec![5.0, -8.0]).unwrap();
        assert!(
            (x[0] - 1.0).abs() < 1e-9 && (x[1] - 3.0).abs() < 1e-9,
            "{x:?}"
        );
    }

    #[test]
    fn reports_a_singular_system_instead_of_guessing() {
        assert!(solve(vec![vec![1.0, 2.0], vec![2.0, 4.0]], vec![3.0, 6.0]).is_none());
    }
}
