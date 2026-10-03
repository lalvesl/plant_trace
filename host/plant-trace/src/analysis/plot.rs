//! gnuplot scripts for the report's figures.
//!
//! The assignment makes formatting a multiplier on the grade, and the rules it
//! lists — at least 7 pt type, axes labelled with units, curves distinguished
//! by dash pattern as well as colour, comparable scales between comparable
//! plots — are the same every time. So they live here once, in a style
//! preamble every figure shares, instead of being reapplied by hand per plot.

use std::{
    io::Write,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result};

use super::{freq::BodePoint, statics::StaticCurve, step::StepMetrics};

/// Shared preamble: PDF output, readable type, a colour-blind-safe cycle whose
/// members also differ in dash pattern.
fn preamble(out: &Path, width_cm: f64, height_cm: f64) -> String {
    format!(
        r##"set terminal pdfcairo enhanced size {width_cm}cm,{height_cm}cm font "sans,9"
set output "{}"
set encoding utf8
set border linewidth 0.8
set tics nomirror font ",8"
set key font ",8" box opaque height 0.4
set grid linewidth 0.4 dashtype 3 linecolor rgb "#b0b0b0"
# Colour and dash pattern both carry the distinction, so the figure survives
# being printed in greyscale.
set style line 1 linecolor rgb "#0072b2" linewidth 2.0 dashtype 1 pointtype 7 pointsize 0.4
set style line 2 linecolor rgb "#d55e00" linewidth 2.0 dashtype 2 pointtype 5 pointsize 0.4
set style line 3 linecolor rgb "#009e73" linewidth 2.0 dashtype 4 pointtype 9 pointsize 0.4
set style line 4 linecolor rgb "#cc79a7" linewidth 2.0 dashtype 5 pointtype 11 pointsize 0.4
set style line 9 linecolor rgb "#666666" linewidth 1.0 dashtype 3
"##,
        out.display()
    )
}

/// Write a gnuplot script and run it if gnuplot is on the path.
///
/// A missing gnuplot is not an error: the script is the durable artefact, and
/// it can be run later or edited into the exact figure a report wants.
fn emit(script_path: &Path, script: &str) -> Result<PathBuf> {
    if let Some(dir) = script_path.parent() {
        std::fs::create_dir_all(dir).ok();
    }
    let mut file = std::fs::File::create(script_path)
        .with_context(|| format!("creating {}", script_path.display()))?;
    file.write_all(script.as_bytes())?;
    drop(file);

    match Command::new("gnuplot").arg(script_path).output() {
        Ok(out) if out.status.success() => {}
        Ok(out) => eprintln!(
            "gnuplot failed on {}: {}",
            script_path.display(),
            String::from_utf8_lossy(&out.stderr).trim()
        ),
        Err(_) => eprintln!(
            "gnuplot not found; {} is written and can be run later",
            script_path.display()
        ),
    }
    Ok(script_path.to_path_buf())
}

/// Static curve and the incremental gain beneath it.
pub fn static_curve(dir: &Path, stem: &str, curve: &StaticCurve) -> Result<PathBuf> {
    let data = dir.join(format!("{stem}.dat"));
    let mut text = String::from("# input_v output_v output_sd_v\n");
    for p in &curve.points {
        text.push_str(&format!(
            "{:.6} {:.6} {:.6}\n",
            p.input_v, p.output_v, p.output_sd_v
        ));
    }
    std::fs::write(&data, text)?;

    let gains = dir.join(format!("{stem}-gain.dat"));
    let mut text = String::from("# input_v gain\n");
    for g in &curve.gains {
        text.push_str(&format!("{:.6} {:.6}\n", g.input_v, g.gain));
    }
    std::fs::write(&gains, text)?;

    let label = if curve.input == "u_t" {
        "u_T [V]"
    } else {
        "p_s [V]"
    };
    let script = format!(
        r##"{}
set multiplot layout 2,1 margins 0.14,0.97,0.12,0.97 spacing 0,0.09
set ylabel "P_e [V]"
set xlabel ""
plot "{}" using 1:2:3 with yerrorbars linestyle 1 title "measured", \
     "{}" using 1:2 with lines linestyle 1 notitle
set ylabel "ΔP_e/Δ{} [V/V]"
set xlabel "{}"
plot "{}" using 1:2 with linespoints linestyle 2 title "incremental gain"
unset multiplot
"##,
        preamble(&dir.join(format!("{stem}.pdf")), 14.0, 12.0),
        data.display(),
        data.display(),
        if curve.input == "u_t" { "u_T" } else { "p_s" },
        label,
        gains.display(),
    );
    emit(&dir.join(format!("{stem}.gp")), &script)
}

/// Step response with the command that caused it.
pub fn step_response(dir: &Path, stem: &str, csv: &Path, metrics: &StepMetrics) -> Result<PathBuf> {
    let input_col = if metrics.input == "u_t" { 5 } else { 6 };
    let script = format!(
        r##"{}
set datafile separator ","
set multiplot layout 2,1 margins 0.14,0.97,0.12,0.97 spacing 0,0.06
set ylabel "{} [V]"
set xlabel ""
set key off
plot "{}" using 1:{} with lines linestyle 2
set ylabel "P_e [V]"
set xlabel "t [s]"
set key on
set arrow 1 from {:.4},graph 0 to {:.4},graph 1 nohead linestyle 9
set label 1 "step" at {:.4},graph 0.94 left offset 0.5,0 font ",8"
plot "{}" using 1:7 with lines linestyle 1 title "P_e", \
     {:.6} with lines linestyle 9 title "final {:.4} V"
unset multiplot
"##,
        preamble(&dir.join(format!("{stem}.pdf")), 14.0, 11.0),
        if metrics.input == "u_t" { "u_T" } else { "p_s" },
        csv.display(),
        input_col,
        metrics.t_step_s,
        metrics.t_step_s,
        metrics.t_step_s,
        csv.display(),
        metrics.y_final_v,
        metrics.y_final_v,
    );
    emit(&dir.join(format!("{stem}.gp")), &script)
}

/// Bode magnitude and phase from a set of measured points.
pub fn bode(dir: &Path, stem: &str, points: &[BodePoint], title: &str) -> Result<PathBuf> {
    let data = dir.join(format!("{stem}.dat"));
    let mut text = String::from("# freq_hz magnitude_db phase_deg magnitude residual\n");
    let mut offset = 0.0;
    let mut previous = points.first().map(|p| p.phase_deg).unwrap_or(0.0);
    for p in points {
        // Same continuous branch the CSV carries — see `report::unwrap_phase`.
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
        text.push_str(&format!(
            "{:.6} {:.4} {:.3} {:.6} {:.4}\n",
            p.freq_hz, p.magnitude_db, phase, p.magnitude, p.residual_ratio
        ));
    }
    std::fs::write(&data, text)?;

    let script = format!(
        r##"{}
set logscale x
set format x "10^{{%T}}"
set multiplot layout 2,1 margins 0.14,0.97,0.12,0.94 spacing 0,0.04
set title "{title}" font ",9"
set ylabel "|G| [dB]"
set xlabel ""
plot "{}" using 1:2 with linespoints linestyle 1 title "measured"
unset title
set ylabel "∠G [°]"
set xlabel "f [Hz]"
set ytics 45
plot "{}" using 1:3 with linespoints linestyle 2 notitle
unset multiplot
"##,
        preamble(&dir.join(format!("{stem}.pdf")), 14.0, 12.0),
        data.display(),
        data.display(),
    );
    emit(&dir.join(format!("{stem}.gp")), &script)
}
