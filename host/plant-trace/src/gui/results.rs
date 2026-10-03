//! Results: open a finished run, look at every recorded segment, and run the
//! same analysis `plant-trace analyze` does, with its tables drawn.

use std::{
    path::{Path, PathBuf},
    sync::{mpsc, Arc},
};

use egui::Ui;
use egui_sc::egui_components::*;

use super::{
    plot::{self, AxisSpec, Line, Plot},
    trace::CHANNELS,
    widgets::{field_label, section},
};
use crate::{analysis::Recording, runner::Manifest};

/// A CSV the analysis wrote: its header and numeric columns.
struct CsvTable {
    name: String,
    header: Vec<String>,
    rows: Vec<Vec<f64>>,
}

impl CsvTable {
    fn load(path: &Path) -> Option<Self> {
        let text = std::fs::read_to_string(path).ok()?;
        let mut lines = text
            .lines()
            .filter(|l| !l.starts_with('#') && !l.trim().is_empty());
        let header: Vec<String> = lines
            .next()?
            .split(',')
            .map(|s| s.trim().to_string())
            .collect();
        let rows = lines
            .map(|l| {
                l.split(',')
                    .map(|v| v.trim().parse::<f64>().unwrap_or(f64::NAN))
                    .collect()
            })
            .collect();
        Some(Self {
            name: path.file_name()?.to_string_lossy().into_owned(),
            header,
            rows,
        })
    }

    fn column(&self, i: usize) -> Vec<f64> {
        self.rows
            .iter()
            .map(|r| r.get(i).copied().unwrap_or(f64::NAN))
            .collect()
    }
}

/// A recorded segment, ready to draw.
struct Segment {
    name: String,
    series: [Arc<Vec<[f64; 2]>>; 3],
}

enum Loaded {
    Run {
        manifest: Manifest,
        segments: Vec<Segment>,
    },
    Analysis {
        tables: Vec<CsvTable>,
        error: Option<String>,
    },
}

pub(crate) struct ResultsTab {
    runs: Vec<PathBuf>,
    selected: Option<usize>,
    manifest: Option<Manifest>,
    segments: Vec<Segment>,
    segment: usize,
    tables: Vec<CsvTable>,
    analysis_error: Option<String>,
    pending: Option<mpsc::Receiver<Loaded>>,
}

impl ResultsTab {
    pub(crate) fn new() -> Self {
        let mut tab = Self {
            runs: Vec::new(),
            selected: None,
            manifest: None,
            segments: Vec::new(),
            segment: 0,
            tables: Vec::new(),
            analysis_error: None,
            pending: None,
        };
        tab.rescan();
        tab
    }

    fn rescan(&mut self) {
        self.runs = std::fs::read_dir("data")
            .map(|rd| {
                rd.filter_map(|e| e.ok().map(|e| e.path()))
                    .filter(|p| p.join("run.json").exists())
                    .collect()
            })
            .unwrap_or_default();
        self.runs.sort();
        self.runs.reverse();
    }

    /// Point the tab at a run that just finished.
    pub(crate) fn open(&mut self, dir: &Path) {
        self.rescan();
        self.selected = self.runs.iter().position(|p| p == dir);
        self.load(dir.to_path_buf());
    }

    fn load(&mut self, dir: PathBuf) {
        let (tx, rx) = mpsc::channel();
        self.pending = Some(rx);
        self.tables.clear();
        self.analysis_error = None;
        std::thread::spawn(move || {
            let loaded = (|| -> anyhow::Result<Loaded> {
                let manifest: Manifest =
                    serde_json::from_str(&std::fs::read_to_string(dir.join("run.json"))?)?;
                let mut segments = Vec::new();
                for step in &manifest.steps {
                    let Some(csv) = &step.csv else { continue };
                    let rec = Recording::load(&dir.join(csv))?;
                    let series = [&rec.u_t, &rec.p_s, &rec.p_e].map(|v| {
                        Arc::new(rec.t.iter().zip(v.iter()).map(|(t, v)| [*t, *v]).collect())
                    });
                    segments.push(Segment {
                        name: step.name.clone(),
                        series,
                    });
                }
                Ok(Loaded::Run { manifest, segments })
            })();
            let _ = tx.send(loaded.unwrap_or_else(|e| Loaded::Analysis {
                tables: Vec::new(),
                error: Some(format!("{e:#}")),
            }));
        });
    }

    fn analyze(&mut self, dir: PathBuf) {
        let (tx, rx) = mpsc::channel();
        self.pending = Some(rx);
        std::thread::spawn(move || {
            let result = crate::analysis::report::analyze_run(&dir, false);
            let loaded = match result {
                Ok(summary) => Loaded::Analysis {
                    tables: summary
                        .artefacts
                        .iter()
                        .filter(|p| p.extension().is_some_and(|x| x == "csv"))
                        .filter_map(|p| CsvTable::load(p))
                        .collect(),
                    error: None,
                },
                Err(e) => Loaded::Analysis {
                    tables: Vec::new(),
                    error: Some(format!("{e:#}")),
                },
            };
            let _ = tx.send(loaded);
        });
    }

    fn poll(&mut self) {
        let Some(rx) = &self.pending else { return };
        if let Ok(loaded) = rx.try_recv() {
            self.pending = None;
            match loaded {
                Loaded::Run { manifest, segments } => {
                    self.manifest = Some(manifest);
                    self.segments = segments;
                    self.segment = 0;
                }
                Loaded::Analysis { tables, error } => {
                    self.tables = tables;
                    self.analysis_error = error;
                }
            }
        }
    }

    pub(crate) fn show(&mut self, ui: &mut Ui) {
        self.poll();
        if self.pending.is_some() {
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(100));
        }
        ui.horizontal(|ui| {
            field_label(ui, "Run");
            let names: Vec<String> = self
                .runs
                .iter()
                .map(|p| {
                    p.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned()
                })
                .collect();
            let refs: Vec<&str> = names.iter().map(String::as_str).collect();
            let mut sel = self.selected;
            if Select::new(&mut sel, &refs)
                .placeholder("pick a run…")
                .width(320.0)
                .show(ui)
            {
                if let Some(i) = sel {
                    self.selected = Some(i);
                    self.load(self.runs[i].clone());
                }
            }
            if Button::new("Refresh")
                .variant(ButtonVariant::Ghost)
                .size(ButtonSize::Sm)
                .show(ui)
                .clicked()
            {
                self.rescan();
            }
            let dir = self.selected.and_then(|i| self.runs.get(i).cloned());
            if Button::new("Analyze")
                .size(ButtonSize::Sm)
                .enabled(dir.is_some() && self.pending.is_none())
                .show(ui)
                .clicked()
            {
                if let Some(dir) = dir {
                    self.analyze(dir);
                }
            }
            if self.pending.is_some() {
                Spinner::new().size(Size::Sm).show(ui);
            }
        });
        Spacing::Sm.show(ui);

        if let Some(m) = &self.manifest {
            section(ui, &m.experiment, Some(&m.description), |ui| {
                muted_text(
                    ui,
                    &format!(
                        "{} · {} Hz · {} · {} steps",
                        m.started_utc,
                        m.fs_hz,
                        m.daq_firmware,
                        m.steps.len()
                    ),
                );
                if !self.segments.is_empty() {
                    let names: Vec<&str> = self.segments.iter().map(|s| s.name.as_str()).collect();
                    let mut sel = Some(self.segment);
                    ui.horizontal(|ui| {
                        field_label(ui, "Segment");
                        Select::new(&mut sel, &names).width(260.0).show(ui);
                    });
                    self.segment = sel.unwrap_or(0).min(self.segments.len() - 1);
                    let seg = &self.segments[self.segment];
                    let lines = (0..3)
                        .map(|ch| {
                            Line::new(
                                CHANNELS[ch],
                                seg.series[ch].clone(),
                                plot::channel_color(ch),
                            )
                            .sorted()
                        })
                        .collect();
                    plot::show(
                        ui,
                        Plot {
                            id: "results_segment",
                            x: AxisSpec::linear("t [s]"),
                            y: AxisSpec::linear("V"),
                            lines,
                            height: 320.0,
                        },
                    );
                }
            });
        }

        if let Some(e) = &self.analysis_error {
            Alert::new("Analysis failed")
                .description(e)
                .variant(AlertVariant::Destructive)
                .show(ui);
        }
        for table in &self.tables {
            Spacing::Sm.show(ui);
            show_table(ui, table);
        }
    }
}

/// An analysis table: the numbers, and the first column against the second
/// drawn, which for every table the analysis writes is the curve that matters
/// (input against output, frequency against gain).
fn show_table(ui: &mut Ui, table: &CsvTable) {
    section(ui, &table.name, None, |ui| {
        if table.header.len() >= 2 && !table.rows.is_empty() {
            let x = table.column(0);
            let log_x = table.header[0].contains("freq") || table.header[0].contains("hz");
            let lines = (1..table.header.len().min(3))
                .map(|c| {
                    let pts = x
                        .iter()
                        .zip(table.column(c))
                        .map(|(x, y)| [*x, y])
                        .collect();
                    Line::new(
                        table.header[c].clone(),
                        Arc::new(pts),
                        plot::channel_color(c + 1),
                    )
                    .markers()
                })
                .collect();
            plot::show(
                ui,
                Plot {
                    id: &format!("table_{}", table.name),
                    x: if log_x {
                        AxisSpec::log(table.header[0].clone())
                    } else {
                        AxisSpec::linear(table.header[0].clone())
                    },
                    y: AxisSpec::linear(""),
                    lines,
                    height: 220.0,
                },
            );
        }
        let columns: Vec<TableColumn> = table
            .header
            .iter()
            .map(|h| TableColumn {
                header: h.as_str(),
                width: Some(110.0),
            })
            .collect();
        Table::new(&columns)
            .striped(true)
            .show(ui, table.rows.len().min(200), |i, row| {
                for v in &table.rows[i] {
                    row.cell(|ui| {
                        ui.label(format!("{v:.5}"));
                    });
                }
            });
    });
}
