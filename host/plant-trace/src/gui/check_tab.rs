//! A free-running sine generator on the outputs, and `check dc` and
//! `check sine` with their results drawn.

use std::{sync::Arc, time::Duration};

use egui::{Context, Ui};
use egui_sc::egui_components::*;

use super::{
    app::Shared,
    monitor::MonitorTab,
    plot::{self, AxisSpec, Line, Plot},
    widgets::{choose, num_f32, num_u32, section, two},
    worker::Cmd,
};
use plant_trace_proto::{scale::DacScale, waveform::Waveform};

use crate::{
    check::{DcOptions, DcReport, SineOptions, SineReport, OUTPUT_NAMES},
    experiment::PLANT_MAX_FREQ_HZ,
};

const CHANNEL_CHOICES: [&str; 3] = ["Both", "u_T only", "p_s only"];

/// Output indices a choice drives.
fn driven(choice: usize) -> Vec<usize> {
    channels(choice).into_iter().map(usize::from).collect()
}

fn channels(choice: usize) -> Vec<u8> {
    match choice {
        1 => vec![0],
        2 => vec![1],
        _ => vec![0, 1],
    }
}

pub(crate) struct CheckTab {
    dc_ch: usize,
    dc_from: [f32; 2],
    dc_to: [f32; 2],
    dc_points: u32,
    dc_settle_ms: u32,
    dc_average_ms: u32,
    dc: Option<DcReport>,

    sine_ch: usize,
    sine_center: [f32; 2],
    sine_amplitude: [f32; 2],
    sine_freq: f32,
    sine_seconds: f32,
    sine: Option<SineReport>,

    /// Free-running generator, per output: on, centre, peak amplitude,
    /// frequency.
    gen_on: [bool; 2],
    gen_center: [f32; 2],
    gen_amplitude: [f32; 2],
    gen_freq: [f32; 2],
}

impl Default for CheckTab {
    fn default() -> Self {
        let d = DcOptions::default();
        let s = SineOptions::default();
        Self {
            dc_ch: 0,
            dc_from: d.from_v,
            dc_to: d.to_v,
            dc_points: d.points as u32,
            dc_settle_ms: d.settle.as_millis() as u32,
            dc_average_ms: d.average.as_millis() as u32,
            dc: None,
            sine_ch: 0,
            sine_center: s.center_v,
            sine_amplitude: s.amplitude_v,
            sine_freq: s.freq_hz,
            sine_seconds: s.duration.as_secs_f32(),
            sine: None,
            gen_on: [true, false],
            // Mid-window, ±5 % of its span, at 1 Hz: the size of an
            // identification excitation, slow enough to watch.
            gen_center: DacScale::NOMINAL_OUTPUTS.map(|s| s.mid_v()),
            gen_amplitude: DacScale::NOMINAL_OUTPUTS.map(|s| 0.05 * s.span_v()),
            gen_freq: [1.0; 2],
        }
    }
}

impl CheckTab {
    pub(crate) fn dc_done(&mut self, ctx: &Context, r: Result<DcReport, String>) {
        match r {
            Ok(rep) => self.dc = Some(rep),
            Err(e) => {
                Toaster::push_with_desc(ctx, "DC sweep failed", &e, ToastVariant::Destructive)
            }
        }
    }

    pub(crate) fn sine_done(&mut self, ctx: &Context, r: Result<SineReport, String>) {
        match r {
            Ok(rep) => self.sine = Some(rep),
            Err(e) => {
                Toaster::push_with_desc(ctx, "Sine check failed", &e, ToastVariant::Destructive)
            }
        }
    }

    pub(crate) fn show(&mut self, ui: &mut Ui, shared: &mut Shared, monitor: &MonitorTab) {
        self.generator(ui, shared, monitor);
        Spacing::Md.show(ui);
        two(
            ui,
            &mut (&mut *self, &mut *shared),
            |ui, (s, sh)| s.dc_form(ui, sh),
            |ui, (s, sh)| s.sine_form(ui, sh),
        );
        Spacing::Md.show(ui);
        if let Some(rep) = &self.dc {
            dc_result(ui, rep);
            Spacing::Md.show(ui);
        }
        if let Some(rep) = &self.sine {
            sine_result(ui, rep);
        }
    }

    /// A sine that runs until stopped, on either output or both, started on
    /// the same tick — for looking at the plant by eye, not for measuring:
    /// nothing is recorded, and the live view shows it.
    fn generator(&mut self, ui: &mut Ui, shared: &mut Shared, monitor: &MonitorTab) {
        section(
            ui,
            "Signal generator",
            Some("A sine on each output that is switched on, until Stop. Nothing is recorded; the live view below is the Monitor's."),
            |ui| {
                let mut problems = Vec::new();
                two(
                    ui,
                    &mut (&mut *self, &mut problems),
                    |ui, (s, p)| s.gen_channel(ui, 0, p),
                    |ui, (s, p)| s.gen_channel(ui, 1, p),
                );
                if !self.gen_on.iter().any(|on| *on) {
                    problems.push("switch on at least one output".to_string());
                }
                for p in &problems {
                    ui.colored_label(ShadcnTheme::get(ui.ctx()).destructive, p);
                }
                ui.horizontal(|ui| {
                    let running = shared.generating;
                    let label = if running { "Apply" } else { "Start" };
                    if Button::new(label)
                        .enabled(shared.idle() && problems.is_empty())
                        .show(ui)
                        .clicked()
                    {
                        shared.send(Cmd::Generate(std::array::from_fn(|ch| {
                            self.gen_on[ch].then(|| Waveform::Sine {
                                center: self.gen_center[ch],
                                amplitude: self.gen_amplitude[ch],
                                freq_hz: self.gen_freq[ch],
                                cycles: 0,
                            })
                        })));
                    }
                    if Button::new("Stop")
                        .variant(ButtonVariant::Secondary)
                        .enabled(shared.idle() && running)
                        .show(ui)
                        .clicked()
                    {
                        // Back to each sine's centre line, not wherever the
                        // tick froze it.
                        shared.send(Cmd::GenStop(std::array::from_fn(|ch| {
                            self.gen_on[ch].then_some(self.gen_center[ch])
                        })));
                    }
                    if running {
                        Badge::new("running").show(ui);
                    }
                    if shared.streaming.is_none()
                        && Button::new("Start live view")
                            .variant(ButtonVariant::Outline)
                            .enabled(shared.idle())
                            .show(ui)
                            .clicked()
                    {
                        shared.send(Cmd::Monitor {
                            on: true,
                            fs_hz: monitor.fs_hz(),
                        });
                    }
                });
                if shared.streaming.is_some() {
                    Spacing::Sm.show(ui);
                    monitor.chart(ui, "generator", 280.0);
                }
            },
        );
    }

    /// One output's generator fields; what is wrong with them goes to
    /// `problems`.
    fn gen_channel(&mut self, ui: &mut Ui, ch: usize, problems: &mut Vec<String>) {
        let w = DacScale::nominal(ch);
        let name = OUTPUT_NAMES[ch];
        Switch::new(&mut self.gen_on[ch]).label(name).show(ui);
        if !self.gen_on[ch] {
            muted_text(ui, "held where it is");
            return;
        }
        num_f32(
            ui,
            &format!("{name} offset"),
            &mut self.gen_center[ch],
            w.min_v..=w.max_v,
            0.005,
            " V",
        );
        num_f32(
            ui,
            &format!("{name} amplitude"),
            &mut self.gen_amplitude[ch],
            0.0..=w.span_v() / 2.0,
            0.001,
            " V",
        );
        num_f32(
            ui,
            &format!("{name} frequency"),
            &mut self.gen_freq[ch],
            0.001..=PLANT_MAX_FREQ_HZ,
            0.1,
            " Hz",
        );
        let (lo, hi) = (
            self.gen_center[ch] - self.gen_amplitude[ch],
            self.gen_center[ch] + self.gen_amplitude[ch],
        );
        if lo < w.min_v - 1e-6 || hi > w.max_v + 1e-6 {
            problems.push(format!(
                "{name} swings {lo:.3}…{hi:.3} V, outside its window {:.2}…{:.2} V",
                w.min_v, w.max_v
            ));
        }
    }

    fn dc_form(&mut self, ui: &mut Ui, shared: &mut Shared) {
        section(
            ui,
            "DC sweep",
            Some("Steps the outputs and reads every plateau back; the fitted line is the calibration."),
            |ui| {
                choose(ui, "Outputs", &mut self.dc_ch, &CHANNEL_CHOICES);
                // Each output steps through its own window; the ends are
                // clamped to it by the rig anyway.
                for ch in driven(self.dc_ch) {
                    let w = DacScale::nominal(ch);
                    let name = OUTPUT_NAMES[ch];
                    num_f32(ui, &format!("{name} from"), &mut self.dc_from[ch], w.min_v..=w.max_v, 0.01, " V");
                    num_f32(ui, &format!("{name} to"), &mut self.dc_to[ch], w.min_v..=w.max_v, 0.01, " V");
                }
                num_u32(ui, "Levels", &mut self.dc_points, 2..=64, "");
                num_u32(ui, "Settle", &mut self.dc_settle_ms, 10..=10_000, " ms");
                num_u32(ui, "Average", &mut self.dc_average_ms, 10..=10_000, " ms");
                if Button::new("Run DC sweep")
                    .enabled(shared.idle())
                    .show(ui)
                    .clicked()
                {
                    shared.send(Cmd::CheckDc(DcOptions {
                        channels: channels(self.dc_ch),
                        from_v: self.dc_from,
                        to_v: self.dc_to,
                        points: self.dc_points as usize,
                        settle: Duration::from_millis(self.dc_settle_ms as u64),
                        average: Duration::from_millis(self.dc_average_ms as u64),
                        fs_hz: 0,
                    }));
                }
            },
        );
    }

    fn sine_form(&mut self, ui: &mut Ui, shared: &mut Shared) {
        section(
            ui,
            "Sine",
            Some("A sinusoid on the outputs, fitted back at its own frequency."),
            |ui| {
                choose(ui, "Outputs", &mut self.sine_ch, &CHANNEL_CHOICES);
                for ch in driven(self.sine_ch) {
                    let w = DacScale::nominal(ch);
                    let name = OUTPUT_NAMES[ch];
                    num_f32(
                        ui,
                        &format!("{name} centre"),
                        &mut self.sine_center[ch],
                        w.min_v..=w.max_v,
                        0.01,
                        " V",
                    );
                    num_f32(
                        ui,
                        &format!("{name} amplitude"),
                        &mut self.sine_amplitude[ch],
                        0.0..=w.span_v() / 2.0,
                        0.005,
                        " V",
                    );
                }
                num_f32(
                    ui,
                    "Frequency",
                    &mut self.sine_freq,
                    0.01..=250.0,
                    0.1,
                    " Hz",
                );
                num_f32(ui, "Window", &mut self.sine_seconds, 0.2..=120.0, 0.1, " s");
                if Button::new("Run sine check")
                    .enabled(shared.idle())
                    .show(ui)
                    .clicked()
                {
                    shared.send(Cmd::CheckSine(SineOptions {
                        channels: channels(self.sine_ch),
                        center_v: self.sine_center,
                        amplitude_v: self.sine_amplitude,
                        freq_hz: self.sine_freq,
                        duration: Duration::from_secs_f32(self.sine_seconds),
                        ..SineOptions::default()
                    }));
                }
            },
        );
    }
}

fn dc_result(ui: &mut Ui, rep: &DcReport) {
    section(ui, "DC sweep — result", None, |ui| {
        // Measured volts against the code actually written, per output, and
        // the fitted calibration line over it.
        let mut lines = Vec::new();
        for &ch in &rep.channels {
            let i = ch as usize;
            let pts: Vec<[f64; 2]> = rep
                .points
                .iter()
                .map(|p| [p.codes[i] as f64, p.measured[i].mean_v])
                .collect();
            lines.push(
                Line::new(
                    OUTPUT_NAMES[i],
                    Arc::new(pts.clone()),
                    plot::channel_color(i),
                )
                .markers(),
            );
            if let Some(cal) = &rep.calibration[i] {
                let fit: Vec<[f64; 2]> = [0.0, 255.0]
                    .iter()
                    .map(|&c| [c, cal.offset_v + c * cal.volts_per_code])
                    .collect();
                lines.push(
                    Line::new(
                        format!("{} fit", OUTPUT_NAMES[i]),
                        Arc::new(fit),
                        plot::reference_color(),
                    )
                    .dashed(),
                );
            }
        }
        two(
            ui,
            &mut lines,
            |ui, lines| {
                plot::show(
                    ui,
                    Plot {
                        id: "dc_line",
                        x: AxisSpec::linear("code"),
                        y: AxisSpec::linear("read [V]"),
                        lines: std::mem::take(lines),
                        height: 280.0,
                    },
                );
            },
            |ui, _| {
                let mut resid = Vec::new();
                for &ch in &rep.channels {
                    let i = ch as usize;
                    if let Some(cal) = &rep.calibration[i] {
                        let pts: Vec<[f64; 2]> = rep
                            .points
                            .iter()
                            .map(|p| {
                                let c = p.codes[i] as f64;
                                [
                                    c,
                                    (p.measured[i].mean_v
                                        - (cal.offset_v + c * cal.volts_per_code))
                                        * 1e3,
                                ]
                            })
                            .collect();
                        resid.push(
                            Line::new(OUTPUT_NAMES[i], Arc::new(pts), plot::channel_color(i))
                                .markers(),
                        );
                    }
                }
                plot::show(
                    ui,
                    Plot {
                        id: "dc_resid",
                        x: AxisSpec::linear("code"),
                        y: AxisSpec::linear("deviation from the line [mV]"),
                        lines: resid,
                        height: 280.0,
                    },
                );
            },
        );

        let columns = [
            TableColumn {
                header: "asked u_T, p_s [V]",
                width: Some(140.0),
            },
            TableColumn {
                header: "codes",
                width: Some(90.0),
            },
            TableColumn {
                header: "u_T read [V]",
                width: Some(110.0),
            },
            TableColumn {
                header: "u_T pp [mV]",
                width: Some(100.0),
            },
            TableColumn {
                header: "p_s read [V]",
                width: Some(110.0),
            },
            TableColumn {
                header: "p_s pp [mV]",
                width: Some(100.0),
            },
        ];
        Table::new(&columns)
            .striped(true)
            .show(ui, rep.points.len(), |i, row| {
                let p = &rep.points[i];
                row.cell(|ui| {
                    ui.label(format!("{:.3}, {:.3}", p.commanded_v[0], p.commanded_v[1]));
                });
                row.cell(|ui| {
                    ui.label(format!("{},{}", p.codes[0], p.codes[1]));
                });
                for ch in 0..2 {
                    row.cell(|ui| {
                        ui.label(format!("{:.4}", p.measured[ch].mean_v));
                    });
                    row.cell(|ui| {
                        ui.label(format!("{:.2}", p.measured[ch].span_v() * 1e3));
                    });
                }
            });

        for &ch in &rep.channels {
            let i = ch as usize;
            if let Some(cal) = &rep.calibration[i] {
                ui.horizontal(|ui| {
                    ui.colored_label(plot::channel_color(i), OUTPUT_NAMES[i]);
                    code_text(
                        ui,
                        &format!(
                            "volts_per_code = {:.6}   offset_v = {:+.6}   worst deviation {:.2} mV",
                            cal.volts_per_code,
                            cal.offset_v,
                            cal.max_deviation_v * 1e3
                        ),
                    );
                });
            }
        }
    });
}

fn sine_result(ui: &mut Ui, rep: &SineReport) {
    section(ui, "Sine — result", None, |ui| {
        let columns = [
            TableColumn {
                header: "output",
                width: Some(80.0),
            },
            TableColumn {
                header: "amplitude [V]",
                width: Some(120.0),
            },
            TableColumn {
                header: "gain [dB]",
                width: Some(90.0),
            },
            TableColumn {
                header: "predicted [dB]",
                width: Some(110.0),
            },
            TableColumn {
                header: "centre error [mV]",
                width: Some(130.0),
            },
            TableColumn {
                header: "residual",
                width: Some(90.0),
            },
        ];
        Table::new(&columns)
            .striped(true)
            .show(ui, rep.results.len(), |i, row| {
                let r = &rep.results[i];
                row.cell(|ui| {
                    ui.colored_label(
                        plot::channel_color(r.channel as usize),
                        OUTPUT_NAMES[r.channel as usize],
                    );
                });
                row.cell(|ui| {
                    ui.label(format!("{:.4}", r.fit.amplitude));
                });
                row.cell(|ui| {
                    ui.label(format!("{:+.3}", r.gain_db));
                });
                row.cell(|ui| {
                    ui.label(format!("{:+.3}", rep.expected.gain_db));
                });
                row.cell(|ui| {
                    ui.label(format!("{:+.2}", r.center_error_v * 1e3));
                });
                row.cell(|ui| {
                    ui.label(format!("{:.3}", r.fit.residual_ratio));
                });
            });
        ui.horizontal(|ui| {
            muted_text(
                ui,
                &format!(
                    "{:.2} Hz at {} Hz sampling; design predicts {:+.3} dB, {:.1}°",
                    rep.freq_hz, rep.fs_hz, rep.expected.gain_db, rep.expected.phase_deg
                ),
            );
            if let (Some(deg), Some(us)) = (rep.skew_deg, rep.skew_us()) {
                code_text(ui, &format!("skew {deg:+.2}° ({us:+.0} µs)"));
            }
        });
    });
}
