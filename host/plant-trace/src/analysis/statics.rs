//! Static gain curves: the plateaus of a staircase, and the incremental gain
//! between them.
//!
//! The incremental gain is the point of the exercise — a valve whose gain is
//! four times larger at the top of its travel than at the bottom is the
//! non-linearity §1.3 asks to be shown, and a single "the gain is 0.9" would
//! hide it.

use anyhow::{bail, Result};
use serde::Serialize;

use super::{mean, std_dev, Recording};

/// One plateau of a staircase.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Plateau {
    /// Time the plateau starts, seconds.
    pub t_start_s: f64,
    /// Time it ends, seconds.
    pub t_end_s: f64,
    /// Mean input over the settled part, volts.
    pub input_v: f64,
    /// Mean output over the settled part, volts.
    pub output_v: f64,
    /// Spread of the output over the settled part, volts — how flat the
    /// plateau really was.
    pub output_sd_v: f64,
    /// Samples averaged.
    pub samples: usize,
}

/// A static curve plus the gains between its points.
#[derive(Debug, Clone, Serialize)]
pub struct StaticCurve {
    /// Which channel was stepped (`u_t` or `p_s`).
    pub input: String,
    /// The plateaus, in the order they were recorded.
    pub points: Vec<Plateau>,
    /// Incremental gain at the midpoint of each consecutive pair, V/V.
    pub gains: Vec<GainPoint>,
}

/// Incremental gain between two plateaus.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct GainPoint {
    /// Input level the gain belongs to (midpoint of the pair), volts.
    pub input_v: f64,
    /// ΔP_e/Δinput between the two plateaus, V/V.
    pub gain: f64,
}

/// Find the plateaus of a staircase and average the settled part of each.
///
/// `settle_fraction` is how much of each plateau to discard before averaging;
/// 0.6 keeps the last 40 %, which is past the plant's transient for a dwell
/// chosen to be several time constants long.
pub fn curve(rec: &Recording, input: &str, settle_fraction: f64) -> Result<StaticCurve> {
    let u = rec.channel(input)?;
    let edges = plateau_edges(u, rec.fs_hz);
    if edges.len() < 2 {
        bail!(
            "found {} plateau(s) on {input}: this does not look like a staircase",
            edges.len()
        );
    }

    let mut points = Vec::new();
    for w in edges.windows(2) {
        let (start, end) = (w[0], w[1]);
        let skip = start + ((end - start) as f64 * settle_fraction) as usize;
        if end.saturating_sub(skip) < 2 {
            continue;
        }
        points.push(Plateau {
            t_start_s: rec.t[start],
            t_end_s: rec.t[end - 1],
            input_v: mean(&u[skip..end]),
            output_v: mean(&rec.p_e[skip..end]),
            output_sd_v: std_dev(&rec.p_e[skip..end]),
            samples: end - skip,
        });
    }

    let gains = points
        .windows(2)
        .filter_map(|p| {
            let du = p[1].input_v - p[0].input_v;
            (du.abs() > 1e-6).then(|| GainPoint {
                input_v: (p[0].input_v + p[1].input_v) / 2.0,
                gain: (p[1].output_v - p[0].output_v) / du,
            })
        })
        .collect();

    Ok(StaticCurve {
        input: input.to_string(),
        points,
        gains,
    })
}

/// Sample indices where the input changes level, plus the end of the record.
///
/// The threshold is deliberately generous: the command channel is a quantised
/// DAC output read back through the ADC, so it is flat to within a count or
/// two and any real step is at least several counts.
fn plateau_edges(u: &[f64], fs_hz: f64) -> Vec<usize> {
    // Ignore changes closer together than this: a plateau shorter than a
    // second is a glitch, not a step.
    let min_len = (fs_hz as usize).max(1);
    let threshold = 3e-3;

    let mut edges = vec![0usize];
    let mut level = u[0];
    for (i, v) in u.iter().enumerate() {
        if (v - level).abs() > threshold && i - edges[edges.len() - 1] >= min_len {
            edges.push(i);
            level = *v;
        } else if (v - level).abs() <= threshold {
            // Track slow drift within a plateau so the comparison stays
            // anchored to where the signal actually is.
            level = 0.99 * level + 0.01 * v;
        }
    }
    edges.push(u.len());
    edges
}

#[cfg(test)]
mod tests {
    use super::*;
    use plant_model::{Plant, PlantParams};

    /// A staircase through the reference plant, as the rig would record it.
    fn staircase(params: PlantParams, dwell_s: f64, levels: &[f64]) -> Recording {
        let fs = 200.0;
        let dt = 1.0 / fs as f32;
        let mut plant = Plant::new(params);
        plant.settle(dt, levels[0] as f32, params.p_s_nominal_v, 200.0);

        let (mut u, mut p, mut y) = (vec![], vec![], vec![]);
        for level in levels {
            for _ in 0..(dwell_s * fs) as usize {
                let out = plant.step(dt, *level as f32, params.p_s_nominal_v);
                u.push(*level);
                p.push(params.p_s_nominal_v as f64);
                y.push(out as f64);
            }
        }
        Recording::from_parts(fs, u, p, y)
    }

    #[test]
    fn recovers_the_plateaus_of_a_staircase() {
        let params = PlantParams {
            noise_v: 0.0,
            ..Default::default()
        };
        let levels = [0.30, 0.40, 0.50, 0.60, 0.70];
        let rec = staircase(params, 60.0, &levels);
        let curve = curve(&rec, "u_t", 0.6).unwrap();

        assert_eq!(curve.points.len(), levels.len());
        for (point, level) in curve.points.iter().zip(levels) {
            assert!(
                (point.input_v - level).abs() < 1e-6,
                "plateau at {} V, expected {level}",
                point.input_v
            );
            let expected = params.flow(level as f32) as f64;
            assert!(
                (point.output_v - expected).abs() < 3e-3,
                "plateau output {} V, the model says {expected}",
                point.output_v
            );
        }
    }

    #[test]
    fn the_incremental_gain_tracks_the_valve_curve() {
        let params = PlantParams {
            noise_v: 0.0,
            ..Default::default()
        };
        let rec = staircase(params, 60.0, &[0.30, 0.40, 0.50, 0.60, 0.70]);
        let curve = curve(&rec, "u_t", 0.6).unwrap();

        assert_eq!(curve.gains.len(), 4);
        for g in &curve.gains {
            let analytic = params.flow_gain(g.input_v as f32) as f64;
            assert!(
                (g.gain - analytic).abs() / analytic < 0.05,
                "gain {:.3} at {:.2} V, analytic {:.3}",
                g.gain,
                g.input_v,
                analytic
            );
        }
        // And the whole point: it is not constant.
        let first = curve.gains.first().unwrap().gain;
        let last = curve.gains.last().unwrap().gain;
        assert!(last > 1.5 * first, "gain barely moved: {first} -> {last}");
    }
}
