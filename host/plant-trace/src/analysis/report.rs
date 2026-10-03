//! Turning a finished run into the tables and figures a report is written
//! from.
//!
//! The run manifest says which excitation produced which CSV, so the analysis
//! dispatches on that rather than on file names: a staircase becomes a static
//! curve, a step becomes step metrics, every sine becomes one Bode point of a
//! shared plot, and a PRBS or a chirp becomes a broadband estimate.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

use crate::{
    analysis::{freq, plot, statics, step, Recording},
    experiment::WaveSpec,
    runner::Manifest,
};

/// What `analyze run` produced.
#[derive(Debug, Default)]
pub struct Summary {
    /// Files written.
    pub artefacts: Vec<PathBuf>,
    /// Step segments measured.
    pub steps: usize,
    /// Bode points measured.
    pub bode_points: usize,
    /// Static curves measured.
    pub curves: usize,
}

/// Analyse every recorded segment of a run directory.
pub fn analyze_run(dir: &Path, plots: bool) -> Result<Summary> {
    let manifest_path = dir.join("run.json");
    let manifest: Manifest = serde_json::from_str(
        &std::fs::read_to_string(&manifest_path)
            .with_context(|| format!("reading {}", manifest_path.display()))?,
    )
    .with_context(|| format!("parsing {}", manifest_path.display()))?;

    let out_dir = dir.join("analysis");
    std::fs::create_dir_all(&out_dir)?;
    let mut summary = Summary::default();
    let mut bode: Vec<(String, freq::BodePoint)> = Vec::new();
    let mut step_rows: Vec<(String, step::StepMetrics)> = Vec::new();

    for record in &manifest.steps {
        let Some(csv) = &record.csv else { continue };
        // A step a cancel cut short is a fragment of the excitation it was
        // meant to be; fitting it would report a number for a test that was
        // never run.
        if record.interrupted {
            eprintln!("{csv}: the run was cancelled during this step — skipped");
            continue;
        }
        let path = dir.join(csv);
        if !path.exists() {
            eprintln!("{} is listed in the manifest but missing", path.display());
            continue;
        }
        let rec = Recording::load(&path)?;

        // The channel that was excited is the one with a waveform that is not
        // a plain hold.
        let Some((input, spec)) = excited(record.u_t, record.p_s) else {
            continue;
        };

        match spec {
            WaveSpec::Staircase { .. } => {
                let curve = statics::curve(&rec, input, 0.6)?;
                let stem = format!("static-{}", record.name);
                write_csv(
                    &out_dir.join(format!("{stem}.csv")),
                    "input_v,output_v,output_sd_v,samples",
                    curve.points.iter().map(|p| {
                        format!(
                            "{:.6},{:.6},{:.6},{}",
                            p.input_v, p.output_v, p.output_sd_v, p.samples
                        )
                    }),
                    &mut summary,
                )?;
                write_csv(
                    &out_dir.join(format!("{stem}-gain.csv")),
                    "input_v,gain_v_per_v",
                    curve
                        .gains
                        .iter()
                        .map(|g| format!("{:.6},{:.6}", g.input_v, g.gain)),
                    &mut summary,
                )?;
                println!(
                    "  {stem}: {} plateaus, incremental gain {:.3} → {:.3} V/V",
                    curve.points.len(),
                    curve.gains.first().map(|g| g.gain).unwrap_or(f64::NAN),
                    curve.gains.last().map(|g| g.gain).unwrap_or(f64::NAN),
                );
                if plots {
                    summary
                        .artefacts
                        .push(plot::static_curve(&out_dir, &stem, &curve)?);
                }
                summary.curves += 1;
            }

            WaveSpec::Step { .. } | WaveSpec::Ramp { .. } => {
                let metrics = step::metrics(&rec, input)?;
                println!(
                    "  {}: K = {:.3} V/V, delay {:.2} s, rise {:.2} s, settling {:.1} s, \
                     overshoot {:.1} %{}",
                    record.name,
                    metrics.gain,
                    metrics.apparent_delay_s,
                    metrics.rise_time_s,
                    metrics.settling_time_s,
                    metrics.overshoot_pct,
                    match (metrics.oscillation_period_s, metrics.damping_ratio) {
                        (Some(t), Some(z)) => format!(", ringing {:.2} s at ζ = {:.3}", t, z),
                        _ => String::new(),
                    }
                );
                if plots {
                    summary.artefacts.push(plot::step_response(
                        &out_dir,
                        &format!("step-{}", record.name),
                        &path,
                        &metrics,
                    )?);
                }
                step_rows.push((record.name.clone(), metrics));
                summary.steps += 1;
            }

            WaveSpec::Sine { freq_hz, .. } => {
                let point = freq::sine_point(&rec, input, freq_hz as f64, 1.0 / 3.0)?;
                println!(
                    "  {}: {:.3} Hz  |G| = {:.4} ({:+.2} dB)  ∠G = {:+.1}°  residual {:.3}",
                    record.name,
                    point.freq_hz,
                    point.magnitude,
                    point.magnitude_db,
                    point.phase_deg,
                    point.residual_ratio
                );
                bode.push((input.to_string(), point));
                summary.bode_points += 1;
            }

            WaveSpec::Prbs { .. } | WaveSpec::Chirp { .. } => {
                let points = freq::etfe(&rec, input, 8192, 5.0)?;
                let stem = format!("etfe-{}", record.name);
                write_csv(
                    &out_dir.join(format!("{stem}.csv")),
                    "freq_hz,magnitude,magnitude_db,phase_deg,coherence",
                    points.iter().map(|p| {
                        format!(
                            "{:.6},{:.6},{:.4},{:.3},{:.4}",
                            p.freq_hz, p.magnitude, p.magnitude_db, p.phase_deg, p.coherence
                        )
                    }),
                    &mut summary,
                )?;
                let usable = points.iter().filter(|p| p.coherence > 0.8).count();
                println!(
                    "  {stem}: {} points, {usable} with coherence above 0.8",
                    points.len()
                );
            }

            WaveSpec::Hold { .. } => {}
        }
    }

    if !step_rows.is_empty() {
        write_csv(
            &out_dir.join("steps.csv"),
            "name,input,gain_v_per_v,apparent_delay_s,rise_time_s,settling_time_s,\
             overshoot_pct,oscillation_period_s,damping_ratio,omega_n_rad_s,\
             fopdt_gain,fopdt_tau_s,fopdt_delay_s,fopdt_rms_v,u_before_v,u_after_v,\
             y_before_v,y_final_v,noise_sd_v",
            step_rows.iter().map(|(name, m)| {
                format!(
                    "{name},{},{:.6},{:.4},{:.4},{:.4},{:.2},{},{},{},{:.6},{:.4},{:.4},{:.6},\
                     {:.6},{:.6},{:.6},{:.6},{:.6}",
                    m.input,
                    m.gain,
                    m.apparent_delay_s,
                    m.rise_time_s,
                    m.settling_time_s,
                    m.overshoot_pct,
                    opt(m.oscillation_period_s),
                    opt(m.damping_ratio),
                    opt(m.omega_n_rad_s),
                    m.fopdt.gain,
                    m.fopdt.tau_s,
                    m.fopdt.delay_s,
                    m.fopdt.rms_error_v,
                    m.u_before_v,
                    m.u_after_v,
                    m.y_before_v,
                    m.y_final_v,
                    m.noise_sd_v,
                )
            }),
            &mut summary,
        )?;
    }

    if !bode.is_empty() {
        let input = bode[0].0.clone();
        let mut points: Vec<freq::BodePoint> = bode.into_iter().map(|(_, p)| p).collect();
        points.sort_by(|a, b| a.freq_hz.partial_cmp(&b.freq_hz).unwrap());
        // Each point is fitted independently, so its phase comes back folded
        // into ±180°. A plot wants the continuous branch: past the resonance
        // the true phase is below −180°, and a curve that jumps to +106° there
        // reads as a measurement error rather than as the lag it is.
        let unwrapped = unwrap_phase(&points);
        write_csv(
            &out_dir.join(format!("bode-{input}.csv")),
            "freq_hz,magnitude,magnitude_db,phase_deg,phase_unwrapped_deg,\
             input_amplitude_v,output_amplitude_v,residual_ratio",
            points.iter().zip(&unwrapped).map(|(p, phase)| {
                format!(
                    "{:.6},{:.6},{:.4},{:.3},{:.3},{:.6},{:.6},{:.4}",
                    p.freq_hz,
                    p.magnitude,
                    p.magnitude_db,
                    p.phase_deg,
                    phase,
                    p.input_amplitude_v,
                    p.output_amplitude_v,
                    p.residual_ratio
                )
            }),
            &mut summary,
        )?;
        if plots {
            let title = if input == "u_t" {
                "ΔP_e/Δu_T"
            } else {
                "ΔP_e/Δp_s"
            };
            summary.artefacts.push(plot::bode(
                &out_dir,
                &format!("bode-{input}"),
                &points,
                title,
            )?);
        }
    }

    Ok(summary)
}

/// The channel carrying a real excitation, if exactly one does.
fn excited(u_t: Option<WaveSpec>, p_s: Option<WaveSpec>) -> Option<(&'static str, WaveSpec)> {
    let interesting = |w: Option<WaveSpec>| -> Option<WaveSpec> {
        match w {
            Some(WaveSpec::Hold { .. }) | None => None,
            Some(other) => Some(other),
        }
    };
    match (interesting(u_t), interesting(p_s)) {
        (Some(w), None) => Some(("u_t", w)),
        (None, Some(w)) => Some(("p_s", w)),
        // Both moving at once would make ΔP_e/Δu_T and ΔP_e/Δp_s
        // indistinguishable from one record, so it is not analysed.
        _ => None,
    }
}

/// Continuous phase branch through a set of Bode points.
///
/// Each step moves by whole turns until it is within half a turn of the
/// previous point, which is the right reconstruction as long as consecutive
/// frequencies are close enough that the true phase moves less than 180°
/// between them — true for the logarithmic grid the experiments use.
fn unwrap_phase(points: &[freq::BodePoint]) -> Vec<f64> {
    let mut out = Vec::with_capacity(points.len());
    let mut offset = 0.0;
    let mut previous = points.first().map(|p| p.phase_deg).unwrap_or(0.0);
    for p in points {
        let mut phase = p.phase_deg + offset;
        while phase - previous > 180.0 {
            offset -= 360.0;
            phase -= 360.0;
        }
        while previous - phase > 180.0 {
            offset += 360.0;
            phase += 360.0;
        }
        previous = phase;
        out.push(phase);
    }
    out
}

fn opt(v: Option<f64>) -> String {
    v.map(|x| format!("{x:.6}")).unwrap_or_default()
}

fn write_csv(
    path: &Path,
    header: &str,
    rows: impl Iterator<Item = String>,
    summary: &mut Summary,
) -> Result<()> {
    let mut text = String::from(header);
    text.push('\n');
    for row in rows {
        text.push_str(&row);
        text.push('\n');
    }
    std::fs::write(path, text).with_context(|| format!("writing {}", path.display()))?;
    summary.artefacts.push(path.to_path_buf());
    Ok(())
}
