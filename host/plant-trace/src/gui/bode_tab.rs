//! Automatic Bode: describe the frequencies, run the stepped-sine sweep, watch
//! the diagram grow point by point.
//!
//! The tab edits a [`BodePlan`] — the same TOML `plant-trace bode` reads — so a
//! plan saved here runs from the command line and a hand-written one opens
//! here.

use std::{path::PathBuf, sync::Arc};

use egui::Ui;
use egui_sc::egui_components::*;

use super::{
    app::Shared,
    jobs::{JobEvent, JobOutcome, JobSpec},
    plot::{self, AxisSpec, Line, Plot},
    trace::{Trace, CHANNELS},
    widgets::{choose, field_label, num_f32, num_f64, num_u32, section, two},
    worker::Cmd,
};
use crate::{
    bode::{self, BodePlan, BodePoint, BodeResult, ExcitedOutput, ResponseChannel},
    experiment::nominal_scale,
};

const EXCITE: [&str; 2] = ["u_T (valve)", "p_s (steam pressure)"];
const RESPONSE: [&str; 3] = ["u_T sense", "p_s sense", "P_e (plant output)"];
const SPACING: [&str; 2] = ["logarithmic", "linear"];
/// Height of each of the two Bode plots, points.
const BODE_PLOT_HEIGHT: f32 = 460.0;

fn excite_index(e: ExcitedOutput) -> usize {
    match e {
        ExcitedOutput::ValveCmd => 0,
        ExcitedOutput::SteamPressure => 1,
    }
}

fn response_index(r: ResponseChannel) -> usize {
    match r {
        ResponseChannel::ValveCmd => 0,
        ResponseChannel::SteamPressure => 1,
        ResponseChannel::ElectricalPower => 2,
    }
}

pub(crate) struct BodeTab {
    plan: BodePlan,
    /// Frequency generator: from, to, count, spacing.
    gen_from: f64,
    gen_to: f64,
    gen_count: u32,
    gen_spacing: usize,
    /// A frequency to add by hand.
    extra: f64,
    plan_path: PathBuf,
    plans: Vec<PathBuf>,
    /// Finished sweeps under `data/` (directories holding a `bode.json`),
    /// newest first, and the one picked in the list.
    sweeps: Vec<PathBuf>,
    sweep_sel: Option<usize>,
    /// The plan of a sweep reopened from disk: the diagram follows it (its
    /// response channel, skew correction and frequency range) instead of the
    /// plan being edited, until the next run.
    shown: Option<BodePlan>,

    running: bool,
    current: Option<(usize, f64)>,
    points: Vec<BodePoint>,
    trace: Trace,
    delay_us: Option<f64>,
    spacing_us: Option<f64>,
    last_dir: Option<PathBuf>,
    last_error: Option<String>,
}

impl BodeTab {
    pub(crate) fn new() -> Self {
        let plan_path = PathBuf::from("experiments/bode-u.toml");
        let plan = BodePlan::load(&plan_path).unwrap_or_default();
        let lo = plan
            .frequencies_hz
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let hi = plan.frequencies_hz.iter().cloned().fold(0.0, f64::max);
        let mut tab = Self {
            gen_from: if lo.is_finite() { lo } else { 0.05 },
            gen_to: if hi > 0.0 { hi } else { 5.0 },
            gen_count: plan.frequencies_hz.len().max(2) as u32,
            gen_spacing: 0,
            extra: 1.0,
            plan_path,
            plans: Vec::new(),
            sweeps: Vec::new(),
            sweep_sel: None,
            shown: None,
            plan,
            running: false,
            current: None,
            points: Vec::new(),
            trace: Trace::rolling(30.0),
            delay_us: None,
            spacing_us: None,
            last_dir: None,
            last_error: None,
        };
        tab.rescan();
        tab
    }

    fn rescan(&mut self) {
        self.plans = std::fs::read_dir("experiments")
            .map(|rd| {
                rd.filter_map(|e| e.ok().map(|e| e.path()))
                    .filter(|p| {
                        p.extension().is_some_and(|x| x == "toml")
                            && p.file_stem()
                                .is_some_and(|s| s.to_string_lossy().starts_with("bode"))
                    })
                    .collect()
            })
            .unwrap_or_default();
        self.plans.sort();
        self.rescan_sweeps();
    }

    fn rescan_sweeps(&mut self) {
        let mut sweeps: Vec<(std::time::SystemTime, PathBuf)> = std::fs::read_dir("data")
            .map(|rd| {
                rd.filter_map(|e| e.ok().map(|e| e.path()))
                    .filter_map(|d| {
                        let json = d.join("bode.json");
                        let t = json.metadata().and_then(|m| m.modified()).ok()?;
                        Some((t, d))
                    })
                    .collect()
            })
            .unwrap_or_default();
        sweeps.sort_by_key(|(t, _)| std::cmp::Reverse(*t));
        let picked = self.sweep_sel.and_then(|i| self.sweeps.get(i).cloned());
        self.sweeps = sweeps.into_iter().map(|(_, d)| d).collect();
        self.sweep_sel = picked.and_then(|p| self.sweeps.iter().position(|d| *d == p));
    }

    /// Show a finished sweep from disk: its points, its fitted delay, its
    /// directory.
    fn open_sweep(&mut self, ctx: &egui::Context, dir: PathBuf) {
        match BodeResult::load(&dir) {
            Ok(r) => {
                self.points = r.points;
                self.points.sort_by(|a, b| a.freq_hz.total_cmp(&b.freq_hz));
                self.delay_us = r.delay.map(|d| d.delay_s * 1e6);
                self.spacing_us = r.implied_scan_spacing_s.map(|s| s * 1e6);
                self.shown = Some(r.plan);
                self.trace.clear();
                self.last_error = None;
                Toaster::push_with_desc(
                    ctx,
                    "Sweep opened",
                    format!(
                        "{} points{} — {}",
                        self.points.len(),
                        if r.cancelled { " (cancelled)" } else { "" },
                        dir.display()
                    ),
                    ToastVariant::Success,
                );
                self.last_dir = Some(dir);
            }
            Err(e) => Toaster::push_with_desc(
                ctx,
                "Could not open the sweep",
                format!("{e:#}"),
                ToastVariant::Destructive,
            ),
        }
    }

    pub(crate) fn job_event(&mut self, e: &JobEvent) {
        if !self.running {
            return;
        }
        match e {
            JobEvent::BodeFrequency { index, freq_hz } => {
                self.current = Some((*index, *freq_hz));
            }
            JobEvent::BodePoint(p) => {
                self.points.push(p.clone());
                self.points.sort_by(|a, b| a.freq_hz.total_cmp(&b.freq_hz));
            }
            JobEvent::Samples { first, fs_hz, rows } => self.trace.push(*first, *fs_hz, rows),
            JobEvent::StepStarted { .. } => {}
        }
    }

    pub(crate) fn job_done(&mut self, r: &Result<JobOutcome, String>) {
        if !self.running {
            return;
        }
        self.running = false;
        self.current = None;
        match r {
            Ok(o) => {
                self.last_dir = Some(o.dir.clone());
                self.rescan_sweeps();
                self.sweep_sel = self.sweeps.iter().position(|d| *d == o.dir);
                self.delay_us = o.delay_s.map(|d| d * 1e6);
                self.spacing_us = o.implied_spacing_s.map(|d| d * 1e6);
                self.last_error = None;
            }
            Err(e) => self.last_error = Some(e.clone()),
        }
    }

    pub(crate) fn show(&mut self, ui: &mut Ui, shared: &mut Shared) {
        self.file_bar(ui);
        Spacing::Sm.show(ui);
        two(
            ui,
            self,
            |ui, s| s.plan_form(ui),
            |ui, s| s.frequency_form(ui),
        );
        Spacing::Sm.show(ui);
        self.run_bar(ui, shared);
        Spacing::Sm.show(ui);
        self.diagram(ui);
    }

    fn file_bar(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            field_label(ui, "Plan file");
            let names: Vec<String> = self
                .plans
                .iter()
                .map(|p| {
                    p.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned()
                })
                .collect();
            let refs: Vec<&str> = names.iter().map(String::as_str).collect();
            let mut sel = self.plans.iter().position(|p| *p == self.plan_path);
            if Select::new(&mut sel, &refs).width(220.0).show(ui) {
                if let Some(i) = sel {
                    let path = self.plans[i].clone();
                    match BodePlan::load(&path) {
                        Ok(plan) => {
                            self.plan = plan;
                            self.plan_path = path;
                        }
                        Err(e) => Toaster::push_with_desc(
                            ui.ctx(),
                            "Could not load",
                            format!("{e:#}"),
                            ToastVariant::Destructive,
                        ),
                    }
                }
            }
            Input::new(&mut self.plan.name).width(180.0).show(ui);
            if Button::new("Save").size(ButtonSize::Sm).show(ui).clicked() {
                self.save(ui.ctx(), self.plan_path.clone());
            }
            if Button::new("Save as name")
                .variant(ButtonVariant::Outline)
                .size(ButtonSize::Sm)
                .show(ui)
                .clicked()
            {
                let stem = if self.plan.name.starts_with("bode") {
                    self.plan.name.clone()
                } else {
                    format!("bode-{}", self.plan.name)
                };
                self.save(
                    ui.ctx(),
                    PathBuf::from("experiments").join(format!("{stem}.toml")),
                );
            }
            small_text(ui, &self.plan_path.display().to_string());
        });
        ui.horizontal(|ui| {
            field_label(ui, "Saved sweep");
            let names: Vec<String> = self
                .sweeps
                .iter()
                .map(|d| {
                    d.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned()
                })
                .collect();
            let refs: Vec<&str> = names.iter().map(String::as_str).collect();
            Select::new(&mut self.sweep_sel, &refs)
                .width(300.0)
                .show(ui);
            let picked = self.sweep_sel.and_then(|i| self.sweeps.get(i).cloned());
            if Button::new("Open")
                .size(ButtonSize::Sm)
                .enabled(picked.is_some() && !self.running)
                .show(ui)
                .clicked()
            {
                if let Some(dir) = picked {
                    self.open_sweep(ui.ctx(), dir);
                }
            }
            if Button::new("Refresh")
                .variant(ButtonVariant::Outline)
                .size(ButtonSize::Sm)
                .show(ui)
                .clicked()
            {
                self.rescan_sweeps();
            }
            if let Some(plan) = &self.shown {
                if Button::new("Use its plan")
                    .variant(ButtonVariant::Outline)
                    .size(ButtonSize::Sm)
                    .enabled(!self.running)
                    .show(ui)
                    .clicked()
                {
                    self.plan = plan.clone();
                }
            }
            muted_text(
                ui,
                "every sweep is saved to data/<plan>-<time>/ (bode.csv, bode.json)",
            );
        });
    }

    fn save(&mut self, ctx: &egui::Context, path: PathBuf) {
        match self.plan.validate().and_then(|()| self.plan.save(&path)) {
            Ok(()) => {
                self.plan_path = path;
                self.rescan();
                Toaster::push(ctx, "Plan saved", ToastVariant::Success);
            }
            Err(e) => Toaster::push_with_desc(
                ctx,
                "Not saved",
                format!("{e:#}"),
                ToastVariant::Destructive,
            ),
        }
    }

    fn plan_form(&mut self, ui: &mut Ui) {
        section(
            ui,
            "Excitation",
            Some("A sine per frequency around the operating point; both channels are fitted at exactly that frequency."),
            |ui| {
                let p = &mut self.plan;
                let mut ex = excite_index(p.excite);
                if choose(ui, "Excite", &mut ex, &EXCITE) {
                    let now = if ex == 0 {
                        ExcitedOutput::ValveCmd
                    } else {
                        ExcitedOutput::SteamPressure
                    };
                    if now != p.excite {
                        // The two outputs live in different windows: the old
                        // operating point becomes the level held, and the
                        // other way round, and the amplitude is 5 % of the
                        // new output's span.
                        p.excite = now;
                        std::mem::swap(&mut p.center_v, &mut p.hold_v);
                        p.amplitude_v = 0.05 * nominal_scale(now.name()).span_v();
                    }
                }
                let mut re = response_index(p.response);
                if choose(ui, "Response", &mut re, &RESPONSE) {
                    p.response = match re {
                        0 => ResponseChannel::ValveCmd,
                        1 => ResponseChannel::SteamPressure,
                        _ => ResponseChannel::ElectricalPower,
                    };
                }
                // Each field inside the window of the output it drives: u_T
                // 2.25-2.75 V, p_s 0-1 V.
                let driven = nominal_scale(p.excite.name());
                let held = nominal_scale(p.excite.other().name());
                num_f32(ui, "Operating point", &mut p.center_v, driven.min_v..=driven.max_v, 0.005, " V");
                num_f32(ui, "Amplitude (peak)", &mut p.amplitude_v, 0.001..=driven.span_v() / 2.0, 0.001, " V");
                num_f32(ui, "Other output held at", &mut p.hold_v, held.min_v..=held.max_v, 0.005, " V");
                num_f64(ui, "Initial settle", &mut p.initial_settle_s, 0.0..=600.0, 1.0, " s");
                num_f64(ui, "Settle per frequency", &mut p.settle_cycles, 0.0..=50.0, 0.1, " cycles");
                num_f64(ui, "  at least", &mut p.settle_min_s, 0.0..=600.0, 0.5, " s");
                num_f64(ui, "  at most", &mut p.settle_max_s, 0.0..=3600.0, 1.0, " s");
                num_f64(ui, "Measure per frequency", &mut p.measure_cycles, 1.0..=200.0, 0.5, " cycles");
                num_f64(ui, "  at least", &mut p.measure_min_s, 0.0..=600.0, 0.5, " s");
                num_f64(ui, "  at most", &mut p.measure_max_s, 0.0..=3600.0, 1.0, " s");
                num_u32(ui, "Sample rate", &mut p.fs_hz, 10..=2000, " Hz");
                ui.horizontal(|ui| {
                    field_label(ui, "Scan-skew correction");
                    Switch::new(&mut p.correct_skew).show(ui);
                    let mut us = p.scan_spacing_s * 1e6;
                    if NumberInput::new("bode_scan_spacing", &mut us)
                        .range(0.0..=1000.0)
                        .step(1.0)
                        .unit("µs per channel")
                        .width(100.0)
                        .show(ui)
                    {
                        p.scan_spacing_s = us * 1e-6;
                    }
                });
                muted_text(
                    ui,
                    &format!(
                        "The response channel is converted {:+.0} µs after the excitation's in each scan.",
                        p.skew_s() * 1e6
                    ),
                );
            },
        );
    }

    fn frequency_form(&mut self, ui: &mut Ui) {
        section(
            ui,
            "Frequencies",
            Some("Generate a sweep, then add or remove single points."),
            |ui| {
                num_f64(ui, "From", &mut self.gen_from, 0.001..=500.0, 0.01, " Hz");
                num_f64(ui, "To", &mut self.gen_to, 0.001..=500.0, 0.01, " Hz");
                num_u32(ui, "Points", &mut self.gen_count, 1..=200, "");
                choose(ui, "Spacing", &mut self.gen_spacing, &SPACING);
                ui.horizontal(|ui| {
                    if Button::new("Generate")
                        .variant(ButtonVariant::Secondary)
                        .size(ButtonSize::Sm)
                        .show(ui)
                        .clicked()
                    {
                        self.plan.frequencies_hz = generate(
                            self.gen_from,
                            self.gen_to,
                            self.gen_count as usize,
                            self.gen_spacing == 0,
                        );
                    }
                    NumberInput::new("bode_extra_freq", &mut self.extra)
                        .range(0.001..=500.0)
                        .step(0.1)
                        .unit("Hz")
                        .width(100.0)
                        .show(ui);
                    if Button::new("Add")
                        .variant(ButtonVariant::Outline)
                        .size(ButtonSize::Sm)
                        .show(ui)
                        .clicked()
                    {
                        let f = &mut self.plan.frequencies_hz;
                        f.push(self.extra);
                        f.sort_by(f64::total_cmp);
                        f.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
                    }
                });
                let mut remove = None;
                ui.horizontal_wrapped(|ui| {
                    for (i, f) in self.plan.frequencies_hz.iter().enumerate() {
                        if Button::new(&format!("{} ×", fmt_hz(*f)))
                            .variant(ButtonVariant::Ghost)
                            .size(ButtonSize::Sm)
                            .show(ui)
                            .on_hover_text("remove")
                            .clicked()
                        {
                            remove = Some(i);
                        }
                    }
                });
                if let Some(i) = remove {
                    self.plan.frequencies_hz.remove(i);
                }
                muted_text(
                    ui,
                    &format!(
                        "{} frequencies · about {}",
                        self.plan.frequencies_hz.len(),
                        fmt_duration(self.plan.estimated_duration().as_secs_f64()),
                    ),
                );
                if let Err(e) = self.plan.validate() {
                    Alert::new("This plan would be refused")
                        .description(&format!("{e:#}"))
                        .variant(AlertVariant::Destructive)
                        .show(ui);
                }
            },
        );
    }

    fn run_bar(&mut self, ui: &mut Ui, shared: &mut Shared) {
        ui.horizontal(|ui| {
            let can = shared.idle() && !self.running && self.plan.validate().is_ok();
            if Button::new("Run Bode sweep")
                .enabled(can)
                .show(ui)
                .clicked()
            {
                self.points.clear();
                self.shown = None;
                self.trace.clear();
                self.delay_us = None;
                self.spacing_us = None;
                self.last_error = None;
                self.running = true;
                shared.send(Cmd::Job(JobSpec::Bode {
                    plan: self.plan.clone(),
                    out_dir: bode::default_out_dir(&self.plan),
                }));
            }
            if !shared.link.is_open() {
                muted_text(ui, "connect to a rig first");
            }
            if let Some((i, f)) = self.current {
                let n = self.plan.frequencies_hz.len().max(1);
                Badge::new(&format!("{}/{}", i + 1, n)).show(ui);
                small_text(ui, &format!("measuring {}", fmt_hz(f)));
                Progress::new(i as f32 / n as f32).show(ui);
            }
            if let Some(d) = self.delay_us {
                code_text(ui, &format!("pure-delay fit {d:+.1} µs"));
            }
            if let Some(s) = self.spacing_us {
                small_text(
                    ui,
                    &format!("→ {s:.1} µs per channel, if the plant is a wire"),
                );
            }
            if let Some(dir) = &self.last_dir {
                small_text(ui, &format!("saved in {}", dir.display()));
            }
        });
        if let Some(e) = &self.last_error {
            Alert::new("The sweep stopped")
                .description(e)
                .variant(AlertVariant::Destructive)
                .show(ui);
        }
    }

    fn diagram(&mut self, ui: &mut Ui) {
        let plan = self.shown.as_ref().unwrap_or(&self.plan);
        let mag: Vec<[f64; 2]> = self.points.iter().map(|p| [p.freq_hz, p.gain_db]).collect();
        let raw: Vec<[f64; 2]> = self
            .points
            .iter()
            .map(|p| [p.freq_hz, p.raw_phase_deg])
            .collect();
        let cor: Vec<[f64; 2]> = self
            .points
            .iter()
            .map(|p| [p.freq_hz, p.phase_deg])
            .collect();
        let lo = plan
            .frequencies_hz
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let hi = plan.frequencies_hz.iter().cloned().fold(0.0, f64::max);
        let x = || {
            let a = AxisSpec::log("f [Hz]");
            if lo.is_finite() && hi > lo {
                a.range(lo / 1.3, hi * 1.3)
            } else {
                a
            }
        };
        let resp = response_index(plan.response);
        let mut phase_lines =
            vec![Line::new("phase", Arc::new(cor), plot::channel_color(resp)).markers()];
        if plan.correct_skew {
            phase_lines.push(
                Line::new("uncorrected", Arc::new(raw), plot::reference_color())
                    .dashed()
                    .markers(),
            );
        }
        section(
            ui,
            "Bode diagram",
            Some("Gain and phase of the response channel against the measured excitation; negative phase is a lag."),
            |ui| {
                // Stacked on a shared frequency axis, each the full width.
                plot::show(
                    ui,
                    Plot {
                        id: "bode_mag",
                        x: x(),
                        y: AxisSpec::linear("|G| [dB]"),
                        lines: vec![Line::new("gain", Arc::new(mag), plot::channel_color(resp)).markers()],
                        height: BODE_PLOT_HEIGHT,
                    },
                );
                plot::show(
                    ui,
                    Plot {
                        id: "bode_phase",
                        x: x(),
                        y: AxisSpec::linear("phase [deg]"),
                        lines: phase_lines,
                        height: BODE_PLOT_HEIGHT,
                    },
                );
                if self.running || !self.trace.is_empty() {
                    let lines = (0..3)
                        .map(|ch| {
                            Line::new(CHANNELS[ch], self.trace.series[ch].clone(), plot::channel_color(ch))
                                .sorted()
                        })
                        .collect();
                    let end = self.trace.last_t();
                    plot::show(
                        ui,
                        Plot {
                            id: "bode_live",
                            x: AxisSpec::linear("t [s]").range((end - 30.0).max(0.0), end.max(1.0)),
                            y: AxisSpec::linear("V"),
                            lines,
                            height: 200.0,
                        },
                    );
                }
                if !self.points.is_empty() {
                    point_table(ui, &self.points);
                }
            },
        );
    }
}

fn point_table(ui: &mut Ui, points: &[BodePoint]) {
    let columns = [
        TableColumn {
            header: "f [Hz]",
            width: Some(90.0),
        },
        TableColumn {
            header: "gain [dB]",
            width: Some(90.0),
        },
        TableColumn {
            header: "phase [°]",
            width: Some(90.0),
        },
        TableColumn {
            header: "raw [°]",
            width: Some(80.0),
        },
        TableColumn {
            header: "in [V]",
            width: Some(80.0),
        },
        TableColumn {
            header: "out [V]",
            width: Some(80.0),
        },
        TableColumn {
            header: "residual in/out",
            width: Some(120.0),
        },
        TableColumn {
            header: "cycles",
            width: Some(70.0),
        },
    ];
    Table::new(&columns)
        .striped(true)
        .show(ui, points.len(), |i, row| {
            let p = &points[i];
            for text in [
                fmt_hz(p.freq_hz),
                format!("{:+.3}", p.gain_db),
                format!("{:+.2}", p.phase_deg),
                format!("{:+.2}", p.raw_phase_deg),
                format!("{:.4}", p.input_amplitude_v),
                format!("{:.4}", p.response_amplitude_v),
                format!("{:.3} / {:.3}", p.input_residual, p.response_residual),
                format!("{:.1}", p.cycles),
            ] {
                row.cell(|ui| {
                    ui.label(text);
                });
            }
        });
}

/// `n` frequencies from `f0` to `f1`, log or linear, rounded to four
/// significant figures so the table reads well.
pub(crate) fn generate(f0: f64, f1: f64, n: usize, log: bool) -> Vec<f64> {
    let (a, b) = if f0 <= f1 { (f0, f1) } else { (f1, f0) };
    if n <= 1 || a <= 0.0 {
        return vec![a.max(1e-3)];
    }
    let raw = if log {
        bode::log_space(a, b, n)
    } else {
        (0..n)
            .map(|i| a + (b - a) * i as f64 / (n - 1) as f64)
            .collect()
    };
    raw.into_iter()
        .map(|f| {
            let mag = 10f64.powf(f.log10().floor() - 3.0);
            (f / mag).round() * mag
        })
        .collect()
}

fn fmt_hz(f: f64) -> String {
    if f >= 1.0 {
        format!("{f:.3} Hz")
    } else {
        format!("{:.1} mHz", f * 1e3)
    }
}

fn fmt_duration(s: f64) -> String {
    if s < 120.0 {
        format!("{s:.0} s")
    } else if s < 7200.0 {
        format!("{:.1} min", s / 60.0)
    } else {
        format!("{:.1} h", s / 3600.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_log_sweep_hits_both_ends_and_is_evenly_spaced_in_log() {
        let f = generate(0.01, 10.0, 4, true);
        assert_eq!(f, vec![0.01, 0.1, 1.0, 10.0]);
        let g = generate(5.0, 1.0, 5, false);
        assert_eq!(g, vec![1.0, 2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn every_checked_in_plan_opens_in_the_tab() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../experiments");
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            let stem = path.file_stem().unwrap().to_string_lossy().into_owned();
            if stem.starts_with("bode") {
                BodePlan::load(&path).unwrap().validate().unwrap();
            }
        }
    }
}
