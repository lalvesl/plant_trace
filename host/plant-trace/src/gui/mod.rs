//! The bench GUI: `plant-trace gui`.
//!
//! One window over the same library the commands use — the rig link, the
//! output checks, the experiment runner and the Bode sweep — with the data
//! drawn as it arrives. Nothing here measures or computes anything the command
//! line cannot; the GUI adds seeing it happen, and editing scenarios without a
//! text editor.
//!
//! Built on egui_shadcn (`egui_sc`): components from `egui_components`, plots
//! from `egui_charts`.

mod app;
mod bode_tab;
mod check_tab;
mod jobs;
mod monitor;
mod plot;
mod results;
mod scenario;
mod trace;
mod widgets;
mod worker;

pub use app::App;

/// How the window starts.
#[derive(Debug, Clone, Default)]
pub struct Options {
    /// Link to open at start-up, if any (`/dev/ttyACM0`, `tcp://…`).
    pub daq: Option<String>,
    /// Start in the light theme.
    pub light: bool,
}

/// Open the window and run until it is closed.
pub fn run(opts: Options) -> anyhow::Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("plant-trace")
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([900.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native(
        "plant-trace",
        native_options,
        Box::new(move |cc| Ok(Box::new(App::new(&cc.egui_ctx, opts)))),
    )
    .map_err(|e| anyhow::anyhow!("the window failed: {e}"))
}
