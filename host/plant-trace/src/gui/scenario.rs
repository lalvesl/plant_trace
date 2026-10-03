//! The scenario editor: experiment files as forms, a preview of what the
//! outputs will be commanded to do, and a run with the data drawn live.
//!
//! A scenario *is* an `experiments/*.toml` file — the same one `plant-trace
//! run` takes — so anything built here runs from the command line too, and a
//! file written by hand opens here. The editor never holds a format of its own.

use std::{
    path::{Path, PathBuf},
    sync::Arc,
};

use egui::Ui;
use egui_sc::egui_components::*;
use plant_trace_proto::scale::DacScale;

use super::{
    app::Shared,
    jobs::{JobEvent, JobOutcome, JobSpec},
    plot::{self, AxisSpec, Line, Plot},
    trace::{Trace, CHANNELS},
    widgets::{choose, field_label, num_f32, num_u32, section, two},
    worker::Cmd,
};
use crate::experiment::{nominal_scale, Experiment, OutputScale, Step, WaveSpec};

const OUTPUTS: [&str; 2] = ["u_t", "p_s"];

const WAVE_KINDS: [&str; 8] = [
    "keep previous",
    "hold",
    "ramp",
    "step",
    "staircase",
    "sine",
    "chirp",
    "prbs",
];

fn wave_kind(w: &Option<WaveSpec>) -> usize {
    match w {
        None => 0,
        Some(WaveSpec::Hold { .. }) => 1,
        Some(WaveSpec::Ramp { .. }) => 2,
        Some(WaveSpec::Step { .. }) => 3,
        Some(WaveSpec::Staircase { .. }) => 4,
        Some(WaveSpec::Sine { .. }) => 5,
        Some(WaveSpec::Chirp { .. }) => 6,
        Some(WaveSpec::Prbs { .. }) => 7,
    }
}

/// The level a waveform starts from, carried over when its kind changes; the
/// middle of the output's window when there is none yet.
fn wave_center(w: &Option<WaveSpec>, window: DacScale) -> f32 {
    match w {
        None => window.mid_v(),
        Some(WaveSpec::Hold { level }) => *level,
        Some(WaveSpec::Ramp { from, .. }) => *from,
        Some(WaveSpec::Step { base, .. }) => *base,
        Some(WaveSpec::Staircase { start, .. }) => *start,
        Some(WaveSpec::Sine { center, .. })
        | Some(WaveSpec::Chirp { center, .. })
        | Some(WaveSpec::Prbs { center, .. }) => *center,
    }
}

/// A new waveform of `kind` at level `c`, sized to the output's window: steps
/// and amplitudes are 5 % of its span (25 mV on `u_T`, 50 mV on `p_s`).
fn default_wave(kind: usize, c: f32, duration_s: f32, window: DacScale) -> Option<WaveSpec> {
    let small = 0.05 * window.span_v();
    Some(match kind {
        0 => return None,
        1 => WaveSpec::Hold { level: c },
        2 => WaveSpec::Ramp {
            from: c,
            to: (c + 2.0 * small).min(window.max_v),
            duration_s: duration_s * 0.5,
        },
        3 => WaveSpec::Step {
            base: c,
            step: small,
            hold_s: duration_s * 0.25,
        },
        4 => WaveSpec::Staircase {
            start: c,
            step: small,
            steps: 5,
            dwell_s: duration_s / 5.0,
        },
        5 => WaveSpec::Sine {
            center: c,
            amplitude: small,
            freq_hz: 1.0,
            cycles: 0,
        },
        6 => WaveSpec::Chirp {
            center: c,
            amplitude: small,
            f0_hz: 0.05,
            f1_hz: 5.0,
            duration_s,
        },
        _ => WaveSpec::Prbs {
            center: c,
            amplitude: small,
            bit_s: 0.5,
            order: 9,
            duration_s,
        },
    })
}

/// The editor's new-scenario template, for tests elsewhere in the GUI.
#[cfg(test)]
pub(crate) fn template_for_tests() -> Experiment {
    template("test")
}

/// The nominal map of an output as an `[outputs.*]` table.
fn nominal_table(output: &str) -> OutputScale {
    let s = nominal_scale(output);
    OutputScale {
        volts_per_code: s.volts_per_code,
        offset_v: s.offset_v,
        min_v: s.min_v,
        max_v: s.max_v,
    }
}

/// A new scenario: settle mid-window, then record a small step on u_T.
fn template(name: &str) -> Experiment {
    let u = nominal_scale("u_t");
    Experiment {
        name: name.to_string(),
        description: String::new(),
        fs_hz: 1000,
        outputs: OUTPUTS
            .iter()
            .map(|o| (o.to_string(), nominal_table(o)))
            .collect(),
        steps: vec![
            Step::Settle {
                name: "settle".into(),
                u_t: u.mid_v(),
                p_s: 0.8,
                timeout_s: 180.0,
                tol_v: 0.002,
                window_s: 5.0,
            },
            Step::Record {
                name: "step-up".into(),
                duration_s: 40.0,
                u_t: Some(WaveSpec::Step {
                    base: u.mid_v(),
                    step: 0.05 * u.span_v(),
                    hold_s: 10.0,
                }),
                p_s: None,
            },
        ],
    }
}

/// The commanded level of both outputs over the whole scenario, at 50 Hz, as
/// `(t, v)` lines. The evaluation is `Experiment::preview`, which runs the
/// generator the firmware runs, so what is drawn is what the plant is sent.
pub(crate) fn preview(exp: &Experiment) -> [Vec<[f64; 2]>; 2] {
    let p = exp.preview(50.0);
    [&p.u_t, &p.p_s].map(|v| {
        p.t_s
            .iter()
            .zip(v.iter())
            .map(|(t, v)| [*t, *v as f64])
            .collect()
    })
}

/// Run progress, as reported by the job.
#[derive(Default)]
struct RunView {
    trace: Option<Trace>,
    /// Index and name of the step in progress.
    step: Option<(usize, String)>,
    running: bool,
    last_dir: Option<PathBuf>,
    last_error: Option<String>,
}

pub(crate) struct ScenarioTab {
    dir: PathBuf,
    files: Vec<PathBuf>,
    selected: Option<usize>,
    path: Option<PathBuf>,
    exp: Experiment,
    open: Vec<bool>,
    dirty: bool,
    source: String,
    source_open: bool,
    source_error: Option<String>,
    preview: [Arc<Vec<[f64; 2]>>; 2],
    preview_stale: bool,
    run: RunView,
}

impl ScenarioTab {
    pub(crate) fn new() -> Self {
        let dir = PathBuf::from("experiments");
        let mut tab = Self {
            files: Vec::new(),
            selected: None,
            path: None,
            exp: template("new-scenario"),
            open: vec![true; 2],
            dirty: false,
            source: String::new(),
            source_open: false,
            source_error: None,
            preview: Default::default(),
            preview_stale: true,
            run: RunView::default(),
            dir,
        };
        tab.rescan();
        tab
    }

    fn rescan(&mut self) {
        self.files = std::fs::read_dir(&self.dir)
            .map(|rd| {
                rd.filter_map(|e| e.ok().map(|e| e.path()))
                    .filter(|p| p.extension().is_some_and(|x| x == "toml"))
                    .filter(|p| {
                        // Bode plans live beside the scenarios but are not
                        // experiments.
                        !p.file_stem()
                            .is_some_and(|s| s.to_string_lossy().starts_with("bode"))
                    })
                    .collect()
            })
            .unwrap_or_default();
        self.files.sort();
    }

    fn load(&mut self, path: &Path) -> anyhow::Result<()> {
        let text = std::fs::read_to_string(path)?;
        let exp: Experiment = toml::from_str(&text)?;
        self.open = vec![false; exp.steps.len()];
        self.exp = exp;
        self.path = Some(path.to_path_buf());
        self.dirty = false;
        self.source_error = None;
        self.preview_stale = true;
        Ok(())
    }

    fn save(&mut self, path: &Path) -> anyhow::Result<()> {
        self.exp.validate()?;
        let text = toml::to_string_pretty(&self.exp)?;
        std::fs::write(path, text)?;
        self.path = Some(path.to_path_buf());
        self.dirty = false;
        self.rescan();
        Ok(())
    }

    pub(crate) fn job_event(&mut self, e: &JobEvent) {
        if !self.run.running {
            return;
        }
        match e {
            JobEvent::StepStarted { index, name } => self.run.step = Some((*index, name.clone())),
            JobEvent::Samples { first, fs_hz, rows } => {
                self.run
                    .trace
                    .get_or_insert_with(Trace::unbounded)
                    .push(*first, *fs_hz, rows);
            }
            _ => {}
        }
    }

    pub(crate) fn job_done(&mut self, r: &Result<JobOutcome, String>) {
        if !self.run.running {
            return;
        }
        self.run.running = false;
        self.run.step = None;
        match r {
            Ok(o) => {
                self.run.last_dir = Some(o.dir.clone());
                self.run.last_error = None;
            }
            Err(e) => self.run.last_error = Some(e.clone()),
        }
    }

    pub(crate) fn show(&mut self, ui: &mut Ui, shared: &mut Shared) {
        if self.preview_stale {
            let [a, b] = preview(&self.exp);
            self.preview = [Arc::new(a), Arc::new(b)];
            self.preview_stale = false;
        }
        self.file_bar(ui, shared);
        Spacing::Sm.show(ui);

        let validation = self.exp.validate().err().map(|e| format!("{e:#}"));
        if let Some(msg) = &validation {
            Alert::new("This scenario would be refused")
                .description(msg)
                .variant(AlertVariant::Destructive)
                .show(ui);
            Spacing::Sm.show(ui);
        }

        let valid = validation.is_none();
        two(
            ui,
            &mut (&mut *self, &mut *shared),
            |ui, (s, sh)| {
                let changed = s.header_form(ui) | s.outputs_form(ui, sh.calibration);
                let steps_changed = s.steps_form(ui);
                if changed || steps_changed {
                    s.dirty = true;
                    s.preview_stale = true;
                }
            },
            |ui, (s, sh)| s.preview_and_run(ui, sh, valid),
        );

        Spacing::Md.show(ui);
        self.source_view(ui);
    }

    fn file_bar(&mut self, ui: &mut Ui, shared: &mut Shared) {
        let _ = shared;
        ui.horizontal(|ui| {
            let names: Vec<String> = self
                .files
                .iter()
                .map(|p| {
                    p.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned()
                })
                .collect();
            let refs: Vec<&str> = names.iter().map(String::as_str).collect();
            field_label(ui, "Scenario file");
            let mut sel = self.selected;
            if Select::new(&mut sel, &refs)
                .placeholder("open…")
                .width(220.0)
                .show(ui)
            {
                if let Some(i) = sel {
                    let path = self.files[i].clone();
                    match self.load(&path) {
                        Ok(()) => self.selected = Some(i),
                        Err(e) => Toaster::push_with_desc(
                            ui.ctx(),
                            "Could not open",
                            format!("{e:#}"),
                            ToastVariant::Destructive,
                        ),
                    }
                }
            }
            if Button::new("New")
                .variant(ButtonVariant::Outline)
                .size(ButtonSize::Sm)
                .show(ui)
                .clicked()
            {
                self.exp = template("new-scenario");
                self.open = vec![true; self.exp.steps.len()];
                self.path = None;
                self.selected = None;
                self.dirty = true;
                self.preview_stale = true;
            }
            let save_target = self
                .path
                .clone()
                .unwrap_or_else(|| self.dir.join(format!("{}.toml", self.exp.name)));
            if Button::new("Save")
                .size(ButtonSize::Sm)
                .enabled(self.dirty)
                .show(ui)
                .clicked()
            {
                self.save_to(ui.ctx(), &save_target);
            }
            if Button::new("Save as name")
                .variant(ButtonVariant::Outline)
                .size(ButtonSize::Sm)
                .show(ui)
                .clicked()
            {
                let target = self.dir.join(format!("{}.toml", self.exp.name));
                self.save_to(ui.ctx(), &target);
            }
            if self.dirty {
                Badge::new("unsaved")
                    .variant(BadgeVariant::Outline)
                    .show(ui);
            }
            small_text(ui, &save_target.display().to_string());
        });
    }

    fn save_to(&mut self, ctx: &egui::Context, path: &Path) {
        match self.save(path) {
            Ok(()) => {
                self.selected = self.files.iter().position(|p| p == path);
                Toaster::push_with_desc(
                    ctx,
                    "Saved",
                    path.display().to_string(),
                    ToastVariant::Success,
                );
            }
            Err(e) => Toaster::push_with_desc(
                ctx,
                "Not saved",
                format!("{e:#}"),
                ToastVariant::Destructive,
            ),
        }
    }

    fn header_form(&mut self, ui: &mut Ui) -> bool {
        let mut changed = false;
        section(ui, "Scenario", None, |ui| {
            ui.horizontal(|ui| {
                field_label(ui, "Name");
                changed |= Input::new(&mut self.exp.name)
                    .width(240.0)
                    .show(ui)
                    .changed();
            });
            ui.horizontal(|ui| {
                field_label(ui, "Description");
                changed |= Input::new(&mut self.exp.description)
                    .width(360.0)
                    .show(ui)
                    .changed();
            });
            changed |= num_u32(ui, "Sample rate", &mut self.exp.fs_hz, 0..=2000, " Hz");
            let total: f32 = self
                .exp
                .steps
                .iter()
                .map(|s| match s {
                    Step::Settle { timeout_s, .. } => *timeout_s,
                    Step::Record { duration_s, .. } => *duration_s,
                })
                .sum();
            muted_text(
                ui,
                &format!(
                    "{} steps · at most {:.0} min (settle steps end as soon as P_e is steady)",
                    self.exp.steps.len(),
                    total / 60.0
                ),
            );
        });
        changed
    }

    fn outputs_form(&mut self, ui: &mut Ui, measured: [Option<(f32, f32)>; 2]) -> bool {
        let mut changed = false;
        Spacing::Sm.show(ui);
        let mut open = ui
            .ctx()
            .data(|d| d.get_temp::<bool>(egui::Id::new("scn_outputs_open")))
            .unwrap_or(false);
        Accordion::new("scn_outputs", "Output calibration and safe window", &mut open).show(ui, |ui| {
            if measured.iter().any(Option::is_some) {
                if Button::new("Use the last DC sweep")
                    .variant(ButtonVariant::Secondary)
                    .size(ButtonSize::Sm)
                    .show(ui)
                    .clicked()
                {
                    for (name, m) in OUTPUTS.iter().zip(measured) {
                        if let (Some((vpc, off)), Some(entry)) = (m, self.exp.outputs.get_mut(*name)) {
                            entry.volts_per_code = vpc;
                            entry.offset_v = off;
                            changed = true;
                        }
                    }
                }
                muted_text(ui, "Replaces volts-per-code and offset with the line the Output check tab fitted; the safe window stays.");
            }
            for name in OUTPUTS {
                let entry = self
                    .exp
                    .outputs
                    .entry(name.to_string())
                    .or_insert_with(|| nominal_table(name));
                // The window can be narrowed, never widened past what the
                // plant takes on this terminal.
                let plant = nominal_scale(name);
                heading4(ui, name);
                changed |= num_f32(ui, "volts per code", &mut entry.volts_per_code, 0.001..=0.02, 0.000001, " V");
                changed |= num_f32(ui, "offset", &mut entry.offset_v, -0.1..=3.0, 0.0001, " V");
                changed |= num_f32(ui, "min", &mut entry.min_v, plant.min_v..=plant.max_v, 0.01, " V");
                changed |= num_f32(ui, "max", &mut entry.max_v, plant.min_v..=plant.max_v, 0.01, " V");
            }
        });
        ui.ctx()
            .data_mut(|d| d.insert_temp(egui::Id::new("scn_outputs_open"), open));
        changed
    }

    fn steps_form(&mut self, ui: &mut Ui) -> bool {
        let mut changed = false;
        Spacing::Sm.show(ui);
        heading4(ui, "Steps");
        self.open.resize(self.exp.steps.len(), true);
        let mut action: Option<(usize, StepAction)> = None;
        let n = self.exp.steps.len();
        for i in 0..n {
            let step = &mut self.exp.steps[i];
            let title = match step {
                Step::Settle { name, u_t, p_s, .. } => format!(
                    "{}. settle '{name}' — u_T {u_t:.3} V, p_s {p_s:.3} V",
                    i + 1
                ),
                Step::Record {
                    name,
                    duration_s,
                    u_t,
                    p_s,
                } => format!(
                    "{}. record '{name}' — {duration_s:.0} s, u_T {}, p_s {}",
                    i + 1,
                    WAVE_KINDS[wave_kind(u_t)],
                    WAVE_KINDS[wave_kind(p_s)]
                ),
            };
            let mut open = self.open[i];
            Accordion::new(("scn_step", i), &title, &mut open).show(ui, |ui| {
                changed |= step_form(ui, i, step);
                ui.horizontal(|ui| {
                    for (label, act, enabled) in [
                        ("Up", StepAction::Up, i > 0),
                        ("Down", StepAction::Down, i + 1 < n),
                        ("Duplicate", StepAction::Duplicate, true),
                        ("Remove", StepAction::Remove, n > 1),
                    ] {
                        let variant = if matches!(act, StepAction::Remove) {
                            ButtonVariant::Destructive
                        } else {
                            ButtonVariant::Outline
                        };
                        if Button::new(label)
                            .variant(variant)
                            .size(ButtonSize::Sm)
                            .enabled(enabled)
                            .show(ui)
                            .clicked()
                        {
                            action = Some((i, act));
                        }
                    }
                });
            });
            self.open[i] = open;
        }
        ui.horizontal(|ui| {
            if Button::new("+ Settle")
                .variant(ButtonVariant::Secondary)
                .size(ButtonSize::Sm)
                .show(ui)
                .clicked()
            {
                action = Some((n, StepAction::AddSettle));
            }
            if Button::new("+ Record")
                .variant(ButtonVariant::Secondary)
                .size(ButtonSize::Sm)
                .show(ui)
                .clicked()
            {
                action = Some((n, StepAction::AddRecord));
            }
        });
        if let Some((i, act)) = action {
            changed = true;
            let steps = &mut self.exp.steps;
            match act {
                StepAction::Up => {
                    steps.swap(i, i - 1);
                    self.open.swap(i, i - 1);
                }
                StepAction::Down => {
                    steps.swap(i, i + 1);
                    self.open.swap(i, i + 1);
                }
                StepAction::Duplicate => {
                    let mut copy = steps[i].clone();
                    rename(&mut copy, "-copy");
                    steps.insert(i + 1, copy);
                    self.open.insert(i + 1, true);
                }
                StepAction::Remove => {
                    steps.remove(i);
                    self.open.remove(i);
                }
                StepAction::AddSettle => {
                    steps.push(Step::Settle {
                        name: format!("settle-{}", i + 1),
                        u_t: 0.5,
                        p_s: 0.5,
                        timeout_s: 180.0,
                        tol_v: 0.002,
                        window_s: 5.0,
                    });
                    self.open.push(true);
                }
                StepAction::AddRecord => {
                    steps.push(Step::Record {
                        name: format!("record-{}", i + 1),
                        duration_s: 30.0,
                        u_t: None,
                        p_s: None,
                    });
                    self.open.push(true);
                }
            }
        }
        changed
    }

    fn preview_and_run(&mut self, ui: &mut Ui, shared: &mut Shared, valid: bool) {
        section(ui, "Commanded outputs", Some("What the generator will be told to do. Settle steps are drawn at their shortest, one steady window; on the rig they last until P_e is steady."), |ui| {
            let lines = (0..2)
                .map(|ch| Line::new(CHANNELS[ch], self.preview[ch].clone(), plot::channel_color(ch)).sorted())
                .collect();
            plot::show(
                ui,
                Plot {
                    id: "scenario_preview",
                    x: AxisSpec::linear("t [s]"),
                    y: AxisSpec::linear("V"),
                    lines,
                    height: 220.0,
                },
            );
        });
        Spacing::Sm.show(ui);
        section(ui, "Run", None, |ui| {
            ui.horizontal(|ui| {
                let can = shared.idle() && valid && !self.run.running;
                if Button::new("Run scenario").enabled(can).show(ui).clicked() {
                    let stamp = crate::csvout::utc_now()
                        .replace(['-', ':'], "")
                        .replace('Z', "");
                    let out_dir = PathBuf::from("data").join(format!("{}-{stamp}", self.exp.name));
                    self.run = RunView {
                        trace: Some(Trace::unbounded()),
                        running: true,
                        ..Default::default()
                    };
                    shared.send(Cmd::Job(JobSpec::Experiment {
                        experiment: self.exp.clone(),
                        out_dir,
                    }));
                }
                if !shared.link.is_open() {
                    muted_text(ui, "connect to a rig first");
                }
                if let Some((i, name)) = &self.run.step {
                    Badge::new(&format!("step {}/{}", i + 1, self.exp.steps.len())).show(ui);
                    small_text(ui, name);
                }
            });
            if let Some(dir) = &self.run.last_dir {
                ui.horizontal(|ui| {
                    muted_text(ui, "results in");
                    code_text(ui, &dir.display().to_string());
                });
            }
            if let Some(e) = &self.run.last_error {
                Alert::new("The run stopped")
                    .description(e)
                    .variant(AlertVariant::Destructive)
                    .show(ui);
            }
            if let Some(trace) = &self.run.trace {
                let lines = (0..3)
                    .map(|ch| {
                        Line::new(
                            CHANNELS[ch],
                            trace.series[ch].clone(),
                            plot::channel_color(ch),
                        )
                        .sorted()
                    })
                    .collect();
                plot::show(
                    ui,
                    Plot {
                        id: "scenario_run",
                        x: AxisSpec::linear("t [s]"),
                        y: AxisSpec::linear("V"),
                        lines,
                        height: 320.0,
                    },
                );
            }
        });
    }

    fn source_view(&mut self, ui: &mut Ui) {
        let was_open = self.source_open;
        Accordion::new("scn_source", "TOML source", &mut self.source_open).show(ui, |ui| {
            muted_text(ui, "The file as it will be saved. Edit it and press Apply to take the text back into the form.");
            Textarea::new(&mut self.source).rows(18).max_rows(40).show(ui);
            ui.horizontal(|ui| {
                if Button::new("Apply").size(ButtonSize::Sm).show(ui).clicked() {
                    match toml::from_str::<Experiment>(&self.source) {
                        Ok(exp) => {
                            self.open = vec![false; exp.steps.len()];
                            self.exp = exp;
                            self.dirty = true;
                            self.preview_stale = true;
                            self.source_error = None;
                        }
                        Err(e) => self.source_error = Some(e.to_string()),
                    }
                }
                if Button::new("Reload from form")
                    .variant(ButtonVariant::Outline)
                    .size(ButtonSize::Sm)
                    .show(ui)
                    .clicked()
                {
                    self.source = toml::to_string_pretty(&self.exp).unwrap_or_default();
                }
            });
            if let Some(e) = &self.source_error {
                Alert::new("Not valid").description(e).variant(AlertVariant::Destructive).show(ui);
            }
        });
        if self.source_open && !was_open {
            self.source = toml::to_string_pretty(&self.exp).unwrap_or_default();
        }
    }
}

#[derive(Clone, Copy)]
enum StepAction {
    Up,
    Down,
    Duplicate,
    Remove,
    AddSettle,
    AddRecord,
}

fn rename(step: &mut Step, suffix: &str) {
    match step {
        Step::Settle { name, .. } | Step::Record { name, .. } => name.push_str(suffix),
    }
}

/// The form for one step. Returns true when anything changed.
fn step_form(ui: &mut Ui, i: usize, step: &mut Step) -> bool {
    let mut changed = false;
    let mut kind = matches!(step, Step::Record { .. }) as usize;
    if choose(ui, "Kind", &mut kind, &["settle", "record"]) {
        changed = true;
        let name = step.name().to_string();
        *step = if kind == 0 {
            Step::Settle {
                name,
                u_t: nominal_scale("u_t").mid_v(),
                p_s: 0.8,
                timeout_s: 180.0,
                tol_v: 0.002,
                window_s: 5.0,
            }
        } else {
            Step::Record {
                name,
                duration_s: 30.0,
                u_t: None,
                p_s: None,
            }
        };
    }
    match step {
        Step::Settle {
            name,
            u_t,
            p_s,
            timeout_s,
            tol_v,
            window_s,
        } => {
            ui.horizontal(|ui| {
                field_label(ui, "Name");
                changed |= Input::new(name).width(200.0).show(ui).changed();
            });
            let (u, p) = (nominal_scale("u_t"), nominal_scale("p_s"));
            changed |= num_f32(ui, "u_T", u_t, u.min_v..=u.max_v, 0.005, " V");
            changed |= num_f32(ui, "p_s", p_s, p.min_v..=p.max_v, 0.005, " V");
            changed |= num_f32(ui, "Give up after", timeout_s, 1.0..=3600.0, 1.0, " s");
            changed |= num_f32(ui, "Steady band", tol_v, 0.0001..=0.1, 0.0001, " V pp");
            changed |= num_f32(ui, "over a window of", window_s, 0.1..=120.0, 0.1, " s");
        }
        Step::Record {
            name,
            duration_s,
            u_t,
            p_s,
        } => {
            ui.horizontal(|ui| {
                field_label(ui, "Name");
                changed |= Input::new(name).width(200.0).show(ui).changed();
            });
            changed |= num_f32(ui, "Duration", duration_s, 0.1..=7200.0, 1.0, " s");
            let d = *duration_s;
            for (label, wave, key) in [
                ("u_T excitation", u_t, "u_t"),
                ("p_s excitation", p_s, "p_s"),
            ] {
                Spacing::Xs.show(ui);
                changed |= wave_form(ui, (i, label), label, wave, d, nominal_scale(key));
            }
        }
    }
    changed
}

/// The form for one waveform, with a kind selector that keeps the operating
/// point when the kind changes.
fn wave_form(
    ui: &mut Ui,
    id: (usize, &str),
    label: &str,
    wave: &mut Option<WaveSpec>,
    duration_s: f32,
    window: DacScale,
) -> bool {
    let _ = id;
    let mut changed = false;
    let mut kind = wave_kind(wave);
    if choose(ui, label, &mut kind, &WAVE_KINDS) {
        *wave = default_wave(kind, wave_center(wave, window), duration_s, window);
        changed = true;
    }
    // Levels inside the output's window, jumps and amplitudes that fit in it.
    let in_window = || window.min_v..=window.max_v;
    let jump = || -window.span_v()..=window.span_v();
    let amp = || 0.0..=window.span_v() / 2.0;
    let Some(w) = wave.as_mut() else {
        return changed;
    };
    ui.indent(label, |ui| match w {
        WaveSpec::Hold { level } => {
            changed |= num_f32(ui, "level", level, in_window(), 0.005, " V");
        }
        WaveSpec::Ramp {
            from,
            to,
            duration_s,
        } => {
            changed |= num_f32(ui, "from", from, in_window(), 0.005, " V");
            changed |= num_f32(ui, "to", to, in_window(), 0.005, " V");
            changed |= num_f32(ui, "over", duration_s, 0.01..=7200.0, 0.1, " s");
        }
        WaveSpec::Step { base, step, hold_s } => {
            changed |= num_f32(ui, "base", base, in_window(), 0.005, " V");
            changed |= num_f32(ui, "jump", step, jump(), 0.005, " V");
            changed |= num_f32(ui, "jump after", hold_s, 0.0..=7200.0, 0.1, " s");
        }
        WaveSpec::Staircase {
            start,
            step,
            steps,
            dwell_s,
        } => {
            changed |= num_f32(ui, "first plateau", start, in_window(), 0.005, " V");
            changed |= num_f32(ui, "increment", step, jump(), 0.005, " V");
            let mut n = *steps as u32;
            if num_u32(ui, "plateaus", &mut n, 1..=1000, "") {
                *steps = n as u16;
                changed = true;
            }
            changed |= num_f32(ui, "dwell", dwell_s, 0.01..=7200.0, 0.1, " s");
        }
        WaveSpec::Sine {
            center,
            amplitude,
            freq_hz,
            cycles,
        } => {
            changed |= num_f32(ui, "centre", center, in_window(), 0.005, " V");
            changed |= num_f32(ui, "amplitude", amplitude, amp(), 0.001, " V");
            changed |= num_f32(ui, "frequency", freq_hz, 0.001..=250.0, 0.01, " Hz");
            changed |= num_u32(
                ui,
                "cycles (0 = until the step ends)",
                cycles,
                0..=100_000,
                "",
            );
        }
        WaveSpec::Chirp {
            center,
            amplitude,
            f0_hz,
            f1_hz,
            duration_s,
        } => {
            changed |= num_f32(ui, "centre", center, in_window(), 0.005, " V");
            changed |= num_f32(ui, "amplitude", amplitude, amp(), 0.001, " V");
            changed |= num_f32(ui, "from", f0_hz, 0.001..=250.0, 0.01, " Hz");
            changed |= num_f32(ui, "to", f1_hz, 0.001..=250.0, 0.01, " Hz");
            changed |= num_f32(ui, "sweep time", duration_s, 0.1..=7200.0, 0.1, " s");
        }
        WaveSpec::Prbs {
            center,
            amplitude,
            bit_s,
            order,
            duration_s,
        } => {
            changed |= num_f32(ui, "centre", center, in_window(), 0.005, " V");
            changed |= num_f32(ui, "amplitude", amplitude, amp(), 0.001, " V");
            changed |= num_f32(ui, "bit time", bit_s, 0.001..=60.0, 0.01, " s");
            let mut o = *order as u32;
            if num_u32(ui, "LFSR order", &mut o, 5..=15, "") {
                *order = o as u8;
                changed = true;
            }
            changed |= num_f32(ui, "run for", duration_s, 0.1..=7200.0, 0.1, " s");
        }
    });
    changed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_template_is_a_valid_scenario_and_round_trips_through_toml() {
        let exp = template("t");
        exp.validate().unwrap();
        let text = toml::to_string_pretty(&exp).unwrap();
        let back: Experiment = toml::from_str(&text).unwrap();
        back.validate().unwrap();
        assert_eq!(back.steps.len(), exp.steps.len());
    }

    #[test]
    fn every_checked_in_scenario_opens_in_the_editor() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../experiments");
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            let stem = path.file_stem().unwrap().to_string_lossy().into_owned();
            if path.extension().is_some_and(|x| x == "toml") && !stem.starts_with("bode") {
                let text = std::fs::read_to_string(&path).unwrap();
                let exp: Experiment = toml::from_str(&text).unwrap();
                // Saving must not lose anything the runner reads.
                let again: Experiment =
                    toml::from_str(&toml::to_string_pretty(&exp).unwrap()).unwrap();
                assert_eq!(
                    format!("{exp:?}"),
                    format!("{again:?}"),
                    "{}",
                    path.display()
                );
            }
        }
    }

    #[test]
    fn the_preview_follows_the_step_and_keeps_the_other_output() {
        let exp = template("t");
        let [u, p] = preview(&exp);
        let last_u = u.last().unwrap()[1];
        assert!((last_u - 2.525).abs() < 1e-5, "u_T ends at {last_u}");
        // p_s is not programmed in the record step: it keeps the settle level.
        let settle_end = 5.0; // the template's settle window
        assert!(p
            .iter()
            .filter(|pt| pt[0] >= settle_end)
            .all(|pt| (pt[1] - 0.8).abs() < 1e-6));
    }

    #[test]
    fn changing_the_kind_keeps_the_operating_point() {
        let window = nominal_scale("u_t");
        let w = Some(WaveSpec::Hold { level: 2.42 });
        for k in 1..WAVE_KINDS.len() {
            let n = default_wave(k, wave_center(&w, window), 10.0, window);
            assert!((wave_center(&n, window) - 2.42).abs() < 1e-6, "kind {k}");
        }
    }
}
