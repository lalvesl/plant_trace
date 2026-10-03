//! The window: connection bar, tabs, and the event loop that folds the rig
//! thread's messages into state.

use egui::{Context, Ui};
use egui_sc::egui_components::*;

use super::{
    bode_tab::BodeTab,
    check_tab::CheckTab,
    jobs::JobEvent,
    monitor::MonitorTab,
    results::ResultsTab,
    scenario::ScenarioTab,
    worker::{Cmd, Evt, Worker},
    Options,
};
use crate::daq::{GenInfo, Info};

/// Connection state as the UI sees it.
pub(crate) enum Link {
    Closed,
    Opening(String),
    Open {
        spec: String,
        info: Info,
        outputs: GenInfo,
    },
}

impl Link {
    pub(crate) fn is_open(&self) -> bool {
        matches!(self, Link::Open { .. })
    }
}

/// Shared state every tab may read: the link, whether a job holds it, and the
/// way to talk to the rig thread.
pub(crate) struct Shared {
    pub(crate) worker: Worker,
    pub(crate) link: Link,
    /// Label of the job holding the link, if any.
    pub(crate) busy: Option<String>,
    /// Effective rate of the live stream, when it runs.
    pub(crate) streaming: Option<u32>,
    /// The free-running generator of the Output check tab is running. Any
    /// job ends by parking the outputs, which stops it too.
    pub(crate) generating: bool,
    /// Latest progress line.
    pub(crate) status: String,
    /// `(volts_per_code, offset_v)` per output from the last DC sweep, for
    /// the scenario editor to adopt.
    pub(crate) calibration: [Option<(f32, f32)>; 2],
    /// Volts at the plant per volt at the rig's input, per channel: the ratio
    /// of whatever divider sits in front of a sense pin. Applied to every row
    /// the GUI draws. 1 everywhere on the bench today: every signal reaches
    /// its pin through the 10 kΩ series resistor alone.
    pub(crate) input_gain: [f32; 3],
}

impl Shared {
    pub(crate) fn send(&mut self, cmd: Cmd) {
        self.worker.send(cmd);
    }

    /// Whether a new job may start now.
    pub(crate) fn idle(&self) -> bool {
        self.link.is_open() && self.busy.is_none()
    }
}

const SIM_PLANTS: [&str; 3] = ["plant model", "wire from u_T", "wire from p_s"];

const TABS: [&str; 5] = ["Monitor", "Output check", "Scenarios", "Bode", "Results"];

/// The application.
pub struct App {
    shared: Shared,
    dark: bool,
    tab: usize,
    spec: String,
    ports: Vec<String>,
    monitor: MonitorTab,
    check: CheckTab,
    scenario: ScenarioTab,
    bode: BodeTab,
    results: ResultsTab,
    capture: Option<Capture>,
    /// The in-process simulator, while the link is to it.
    sim: Option<crate::sim::SimHandle>,
    /// Which plant the simulator runs: 0 = model, 1 = wire from u_T,
    /// 2 = wire from p_s.
    sim_plant: usize,
}

/// Self-screenshot for development and docs: with `PLANT_TRACE_CAPTURE=out.ppm`
/// set, the window saves an image of itself after a few seconds and closes.
/// It captures this window only, never the desktop.
struct Capture {
    path: std::path::PathBuf,
    frames: u32,
    requested: bool,
}

impl App {
    /// Build the app and register what egui_shadcn needs before it paints.
    pub fn new(ctx: &Context, opts: Options) -> Self {
        register_font(ctx);
        let dark = !opts.light;
        let theme = ShadcnTheme::build(dark, None);
        ShadcnTheme::set(ctx, theme.clone());
        theme.apply(ctx);

        let wake = {
            let ctx = ctx.clone();
            move || ctx.request_repaint()
        };
        let mut app = Self {
            shared: Shared {
                worker: Worker::spawn(wake),
                link: Link::Closed,
                busy: None,
                streaming: None,
                generating: false,
                status: String::new(),
                calibration: [None; 2],
                input_gain: [1.0; 3],
            },
            dark,
            tab: 0,
            spec: opts.daq.clone().unwrap_or_else(|| "/dev/ttyACM0".into()),
            ports: list_ports(),
            monitor: MonitorTab::default(),
            check: CheckTab::default(),
            scenario: ScenarioTab::new(),
            bode: BodeTab::new(),
            results: ResultsTab::new(),
            sim: None,
            sim_plant: 0,
            capture: std::env::var_os("PLANT_TRACE_CAPTURE").map(|p| Capture {
                path: p.into(),
                frames: 0,
                requested: false,
            }),
        };
        if let Some(tab) = std::env::var("PLANT_TRACE_TAB")
            .ok()
            .and_then(|t| t.parse().ok())
        {
            app.tab = tab;
        }
        if let Some(spec) = opts.daq {
            app.connect(spec);
        }
        app
    }

    fn connect(&mut self, spec: String) {
        self.shared.link = Link::Opening(spec.clone());
        self.shared.send(Cmd::Connect(spec));
    }

    /// Fold every pending message from the rig thread into state.
    fn absorb(&mut self, ctx: &Context) {
        for evt in self.shared.worker.drain() {
            match evt {
                Evt::Connected {
                    spec,
                    info,
                    outputs,
                } => {
                    Toaster::push_with_desc(
                        ctx,
                        "Connected",
                        format!("{} — {}", spec, info.firmware),
                        ToastVariant::Success,
                    );
                    self.shared.link = Link::Open {
                        spec,
                        info,
                        outputs,
                    };
                    // Live view on by default: the first thing anyone wants
                    // is to see the three signals move.
                    self.shared.send(Cmd::Monitor {
                        on: true,
                        fs_hz: self.monitor.fs_hz(),
                    });
                }
                Evt::Disconnected(reason) => {
                    self.sim = None;
                    self.shared.generating = false;
                    self.shared.link = Link::Closed;
                    self.shared.busy = None;
                    if let Some(r) = reason {
                        Toaster::push_with_desc(ctx, "Link lost", &r, ToastVariant::Destructive);
                    }
                }
                Evt::Streaming(fs) => self.shared.streaming = fs,
                Evt::Generating(on) => self.shared.generating = on,
                Evt::Samples { first, fs_hz, rows } => {
                    let rows = scaled(rows, self.shared.input_gain);
                    self.monitor.push(first, fs_hz, &rows)
                }
                Evt::Busy(label) => {
                    // Every job parks the outputs when it ends.
                    self.shared.generating = false;
                    self.shared.status = format!("{label}…");
                    self.shared.busy = Some(label);
                }
                Evt::Status(line) => self.shared.status = line,
                Evt::DcDone(r) => {
                    self.shared.busy = None;
                    if let Ok(rep) = &r {
                        for (slot, cal) in self.shared.calibration.iter_mut().zip(&rep.calibration)
                        {
                            if let Some(c) = cal {
                                *slot = Some((c.volts_per_code as f32, c.offset_v as f32));
                            }
                        }
                    }
                    self.check.dc_done(ctx, r);
                }
                Evt::SineDone(r) => {
                    self.shared.busy = None;
                    self.check.sine_done(ctx, r);
                }
                Evt::Job(e) => {
                    let e = match e {
                        JobEvent::Samples { first, fs_hz, rows } => JobEvent::Samples {
                            first,
                            fs_hz,
                            rows: scaled(rows, self.shared.input_gain),
                        },
                        other => other,
                    };
                    self.scenario.job_event(&e);
                    self.bode.job_event(&e);
                }
                Evt::JobDone(r) => {
                    self.shared.busy = None;
                    self.shared.status.clear();
                    match &r {
                        Ok(o) => Toaster::push_with_desc(
                            ctx,
                            "Finished",
                            format!("{} — {}", o.summary, o.dir.display()),
                            ToastVariant::Success,
                        ),
                        Err(e) => {
                            Toaster::push_with_desc(ctx, "Stopped", e, ToastVariant::Destructive)
                        }
                    }
                    self.scenario.job_done(&r);
                    self.bode.job_done(&r);
                    if let Ok(o) = &r {
                        if o.dir.join("run.json").exists() {
                            self.results.open(&o.dir);
                        }
                    }
                }
                Evt::Error(e) => {
                    if matches!(self.shared.link, Link::Opening(_)) {
                        self.shared.link = Link::Closed;
                    }
                    Toaster::push_with_desc(ctx, "Error", &e, ToastVariant::Destructive);
                }
            }
        }
    }

    /// Render the whole window into `ui`. Split out of [`eframe::App::ui`] so
    /// tests can drive the app headlessly.
    pub fn show(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();
        let theme = ShadcnTheme::build(self.dark, None);
        ShadcnTheme::set(&ctx, theme.clone());
        theme.apply(&ctx);
        self.absorb(&ctx);

        let theme = ShadcnTheme::get(&ctx);
        egui::Panel::top("connection")
            .frame(
                egui::Frame::new()
                    .fill(theme.background)
                    .inner_margin(egui::Margin::symmetric(16, 10))
                    .stroke(egui::Stroke::new(1.0, theme.border)),
            )
            .show(ui, |ui| self.connection_bar(ui));

        egui::Panel::bottom("status")
            .frame(
                egui::Frame::new()
                    .fill(theme.background)
                    .inner_margin(egui::Margin::symmetric(16, 6))
                    .stroke(egui::Stroke::new(1.0, theme.border)),
            )
            .show(ui, |ui| self.status_bar(ui));

        egui::Frame::new()
            .fill(theme.background)
            .inner_margin(egui::Margin::symmetric(16, 8))
            .show(ui, |ui| {
                let mut tab = self.tab;
                Tabs::new("main_tabs", &TABS, &mut tab).show(ui, |ui, index| {
                    egui::ScrollArea::vertical()
                        .id_salt(("tab_scroll", index))
                        .auto_shrink([false; 2])
                        .show(ui, |ui| match index {
                            0 => self.monitor.show(ui, &mut self.shared),
                            1 => self.check.show(ui, &mut self.shared, &self.monitor),
                            2 => self.scenario.show(ui, &mut self.shared),
                            3 => self.bode.show(ui, &mut self.shared),
                            _ => self.results.show(ui),
                        });
                });
                self.tab = tab;
            });

        Toaster::show(&ctx);
        self.capture_step(&ctx);
        if self.shared.streaming.is_some() || self.shared.busy.is_some() {
            // Keep the plots moving even between events.
            ctx.request_repaint_after(std::time::Duration::from_millis(50));
        }
    }

    /// Start the simulator in this process and connect to it: the whole GUI
    /// works without a board, and with the wire plant it reproduces a bench
    /// whose `P_e` input is wired to an output.
    fn start_sim(&mut self, ctx: &Context) {
        use crate::sim::{self, SimPlant};
        let plant = match self.sim_plant {
            1 => SimPlant::Wire { from: 0 },
            2 => SimPlant::Wire { from: 1 },
            _ => SimPlant::Model,
        };
        match sim::spawn(sim::Options {
            plant,
            settle_s: 30.0,
            ..sim::Options::default()
        }) {
            Ok(handle) => {
                let spec = handle.spec().to_string();
                self.sim = Some(handle);
                self.connect(spec);
            }
            Err(e) => Toaster::push_with_desc(
                ctx,
                "Simulator failed",
                format!("{e:#}"),
                ToastVariant::Destructive,
            ),
        }
    }

    /// Which tab is showing (0 = monitor, 1 = output check, 2 = scenarios,
    /// 3 = Bode, 4 = results). For tests and for starting on a given tab.
    pub fn set_tab(&mut self, tab: usize) {
        self.tab = tab.min(TABS.len() - 1);
    }

    /// Whether a rig link is open.
    pub fn is_connected(&self) -> bool {
        self.shared.link.is_open()
    }

    /// Effective rate of the live view, when it streams.
    pub fn streaming(&self) -> Option<u32> {
        self.shared.streaming
    }

    fn capture_step(&mut self, ctx: &Context) {
        let Some(cap) = self.capture.as_mut() else {
            return;
        };
        cap.frames += 1;
        ctx.request_repaint_after(std::time::Duration::from_millis(30));
        if !cap.requested && cap.frames > 150 {
            cap.requested = true;
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(Default::default()));
        }
        let shot = ctx.input(|i| {
            i.events.iter().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        if let Some(img) = shot {
            let [w, h] = img.size;
            let mut ppm = format!("P6\n{w} {h}\n255\n").into_bytes();
            for px in &img.pixels {
                ppm.extend_from_slice(&[px.r(), px.g(), px.b()]);
            }
            let _ = std::fs::write(&cap.path, ppm);
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }

    fn connection_bar(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            heading4(ui, "plant-trace");
            Separator::vertical().show(ui);
            match &self.shared.link {
                Link::Open { spec, info, outputs } => {
                    Badge::new("connected").show(ui);
                    small_text(
                        ui,
                        &format!(
                            "{spec} · {} · {:.2} V full scale · {} Hz max · {} outputs × {} bit at {} Hz",
                            info.firmware,
                            info.full_scale_mv as f32 / 1000.0,
                            info.max_fs_hz,
                            outputs.outputs,
                            outputs.bits,
                            outputs.tick_hz
                        ),
                    );
                    if Button::new("Disconnect")
                        .variant(ButtonVariant::Outline)
                        .size(ButtonSize::Sm)
                        .enabled(self.shared.busy.is_none())
                        .show(ui)
                        .clicked()
                    {
                        self.shared.send(Cmd::Disconnect);
                    }
                }
                Link::Opening(spec) => {
                    Spinner::new().show(ui);
                    small_text(ui, &format!("opening {spec}…"));
                }
                Link::Closed => {
                    Badge::new("offline").variant(BadgeVariant::Secondary).show(ui);
                    Input::new(&mut self.spec)
                        .placeholder("/dev/ttyACM0 or tcp://127.0.0.1:7801")
                        .width(260.0)
                        .show(ui);
                    if !self.ports.is_empty() {
                        let items: Vec<DropdownItem> = self
                            .ports
                            .iter()
                            .map(|p| DropdownItem::Item {
                                label: p.as_str(),
                                disabled: false,
                            })
                            .collect();
                        if let Some(i) = DropdownMenu::new("ports", "Ports", &items).show(ui) {
                            self.spec = self.ports[i].clone();
                        }
                    }
                    if Button::new("Refresh")
                        .variant(ButtonVariant::Ghost)
                        .size(ButtonSize::Sm)
                        .show(ui)
                        .clicked()
                    {
                        self.ports = list_ports();
                    }
                    if Button::new("Connect").size(ButtonSize::Sm).show(ui).clicked() {
                        let spec = self.spec.trim().to_string();
                        self.connect(spec);
                    }
                    Separator::vertical().show(ui);
                    let mut plant = Some(self.sim_plant);
                    Select::new(&mut plant, &SIM_PLANTS).width(150.0).show(ui);
                    self.sim_plant = plant.unwrap_or(0);
                    if Button::new("Simulated rig")
                        .variant(ButtonVariant::Secondary)
                        .size(ButtonSize::Sm)
                        .show(ui)
                        .clicked()
                    {
                        self.start_sim(ui.ctx());
                    }
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                Switch::new(&mut self.dark).label("Dark").show(ui);
            });
        });
    }

    fn status_bar(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            match self.shared.streaming {
                Some(fs) => Badge::new(&format!("live {fs} Hz")).show(ui),
                None => Badge::new("idle").variant(BadgeVariant::Outline).show(ui),
            }
            if let Some(label) = &self.shared.busy {
                Spinner::new().size(Size::Sm).show(ui);
                small_text(ui, label);
                if Button::new("Cancel")
                    .variant(ButtonVariant::Destructive)
                    .size(ButtonSize::Sm)
                    .show(ui)
                    .clicked()
                {
                    self.shared.worker.cancel();
                }
            }
            muted_text(ui, &self.shared.status);
        });
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        self.show(ui);
    }
}

/// Rows in rig-input volts to rows in plant volts.
fn scaled(mut rows: Vec<[f32; 3]>, gain: [f32; 3]) -> Vec<[f32; 3]> {
    for r in &mut rows {
        for (v, g) in r.iter_mut().zip(gain) {
            *v *= g;
        }
    }
    rows
}

/// Serial ports that look like the rig, newest first.
fn list_ports() -> Vec<String> {
    let mut ports: Vec<String> = serialport::available_ports()
        .map(|v| v.into_iter().map(|p| p.port_name).collect())
        .unwrap_or_default();
    ports.retain(|p| p.contains("ACM") || p.contains("USB") || p.contains("usbmodem"));
    ports.sort();
    ports
}
