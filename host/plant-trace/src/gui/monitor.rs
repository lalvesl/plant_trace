//! Live view: the three channels as they arrive, and a hand on each output.

use egui::Ui;
use egui_sc::egui_components::*;
use plant_trace_proto::scale::DacScale;

use super::{
    app::{Link, Shared},
    plot::{self, AxisSpec, Line, Plot},
    trace::{Trace, CHANNELS},
    widgets::{choose, num_f32, section, two},
    worker::Cmd,
};

const RATES: [u32; 5] = [100, 250, 500, 1000, 2000];
const RATE_LABELS: [&str; 5] = ["100 Hz", "250 Hz", "500 Hz", "1 kHz", "2 kHz"];
const WINDOWS: [f64; 5] = [1.0, 5.0, 10.0, 30.0, 120.0];
const WINDOW_LABELS: [&str; 5] = ["1 s", "5 s", "10 s", "30 s", "2 min"];

pub(crate) struct MonitorTab {
    trace: Trace,
    rate: usize,
    window: usize,
    paused: bool,
    levels: [f32; 2],
    /// Levels last sent, so dragging a slider sends each value once.
    sent: [f32; 2],
}

impl Default for MonitorTab {
    fn default() -> Self {
        Self {
            trace: Trace::rolling(WINDOWS[4]),
            rate: 3,
            window: 1,
            paused: false,
            levels: parked(),
            sent: [f32::NAN; 2],
        }
    }
}

impl MonitorTab {
    pub(crate) fn fs_hz(&self) -> u32 {
        RATES[self.rate]
    }

    pub(crate) fn push(&mut self, first: u64, fs_hz: u32, rows: &[[f32; 3]]) {
        if !self.paused {
            self.trace.push(first, fs_hz, rows);
        }
    }

    /// The live chart over the chosen window. Other tabs draw it too — the
    /// Output check tab's generator — under their own `id`.
    pub(crate) fn chart(&self, ui: &mut Ui, id: &'static str, height: f32) {
        let window = WINDOWS[self.window];
        let end = self.trace.last_t().max(window);
        let lines = (0..3)
            .map(|ch| {
                Line::new(
                    CHANNELS[ch],
                    self.trace.series[ch].clone(),
                    plot::channel_color(ch),
                )
                .sorted()
            })
            .collect();
        plot::show(
            ui,
            Plot {
                id,
                x: AxisSpec::linear("t [s]").range(end - window, end),
                y: AxisSpec::linear("V"),
                lines,
                height,
            },
        );
    }

    pub(crate) fn show(&mut self, ui: &mut Ui, shared: &mut Shared) {
        section(
            ui,
            "Signals",
            Some("u_T and p_s are read back at the nodes the plant is fed from; P_e is the plant's answer."),
            |ui| self.chart(ui, "monitor", 420.0),
        );
        Spacing::Md.show(ui);
        two(
            ui,
            &mut (&mut *self, &mut *shared),
            |ui, (s, sh)| s.controls(ui, sh),
            |ui, (s, _)| s.readout(ui),
        );
    }

    fn controls(&mut self, ui: &mut Ui, shared: &mut Shared) {
        section(ui, "Acquisition", None, |ui| {
            let live = shared.streaming.is_some();
            ui.horizontal(|ui| {
                let label = if live { "Stop" } else { "Start" };
                if Button::new(label)
                    .variant(if live {
                        ButtonVariant::Outline
                    } else {
                        ButtonVariant::Default
                    })
                    .enabled(shared.idle())
                    .show(ui)
                    .clicked()
                {
                    shared.send(Cmd::Monitor {
                        on: !live,
                        fs_hz: self.fs_hz(),
                    });
                }
                Switch::new(&mut self.paused).label("Freeze").show(ui);
                if Button::new("Clear")
                    .variant(ButtonVariant::Ghost)
                    .show(ui)
                    .clicked()
                {
                    self.trace.clear();
                }
            });
            if choose(ui, "Sample rate", &mut self.rate, &RATE_LABELS) && live {
                shared.send(Cmd::Monitor {
                    on: true,
                    fs_hz: self.fs_hz(),
                });
            }
            choose(ui, "Window", &mut self.window, &WINDOW_LABELS);
        });
        Spacing::Sm.show(ui);
        section(
            ui,
            "Outputs",
            Some("Held levels, volts at the plant input, inside each output's window."),
            |ui| {
                for (ch, name) in ["u_T", "p_s"].iter().enumerate() {
                    let w = DacScale::nominal(ch);
                    num_f32(
                        ui,
                        name,
                        &mut self.levels[ch],
                        w.min_v..=w.max_v,
                        0.002,
                        " V",
                    );
                    if shared.idle() && (self.levels[ch] - self.sent[ch]).abs() > 1e-6 {
                        shared.send(Cmd::SetLevel {
                            ch: ch as u8,
                            volts: self.levels[ch],
                        });
                        self.sent[ch] = self.levels[ch];
                    }
                }
                if Button::new("Park both")
                    .variant(ButtonVariant::Secondary)
                    .enabled(shared.idle())
                    .show(ui)
                    .clicked()
                {
                    shared.send(Cmd::Park);
                    self.levels = parked();
                    self.sent = parked();
                }
            },
        );
        Spacing::Sm.show(ui);
        section(
            ui,
            "Inputs",
            Some("Divider in front of each sense pin, from calibration.toml: volts at the plant per volt at the pin, already applied to every reading and every file."),
            |ui| {
                let Link::Open {
                    info, calibration, ..
                } = &shared.link
                else {
                    muted_text(ui, "connect to see the bench calibration");
                    return;
                };
                let adc = info.adc_scale();
                for (ch, name) in CHANNELS.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.colored_label(plot::channel_color(ch), *name);
                        code_text(
                            ui,
                            &format!(
                                "×{:.4}   full scale {:.3} V",
                                adc.input_gain(ch),
                                adc.plant_full_scale_v(ch)
                            ),
                        );
                    });
                }
                match calibration {
                    Some(path) => small_text(ui, &path.display().to_string()),
                    None => muted_text(
                        ui,
                        "no calibration.toml found: every input is read as direct",
                    ),
                }
            },
        );
    }

    fn readout(&self, ui: &mut Ui) {
        section(
            ui,
            "Now",
            Some("Mean of the last 100 ms, and its peak-to-peak."),
            |ui| {
                let fs = self.trace.fs_hz.max(1) as usize;
                let n = (fs / 10).max(1);
                for (ch, s) in self.trace.series.iter().enumerate() {
                    let tail = &s[s.len().saturating_sub(n)..];
                    let vals: Vec<f64> = tail
                        .iter()
                        .map(|p| p[1])
                        .filter(|v| v.is_finite())
                        .collect();
                    ui.horizontal(|ui| {
                        ui.colored_label(plot::channel_color(ch), CHANNELS[ch]);
                        if vals.is_empty() {
                            muted_text(ui, "—");
                        } else {
                            let mean = vals.iter().sum::<f64>() / vals.len() as f64;
                            let lo = vals.iter().cloned().fold(f64::INFINITY, f64::min);
                            let hi = vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                            code_text(ui, &format!("{mean:+.4} V"));
                            muted_text(ui, &format!("{:.2} mV pp", (hi - lo) * 1e3));
                        }
                    });
                }
            },
        );
    }
}

/// Where Park leaves the outputs: the bottom of each window.
fn parked() -> [f32; 2] {
    [DacScale::nominal(0).min_v, DacScale::nominal(1).min_v]
}
