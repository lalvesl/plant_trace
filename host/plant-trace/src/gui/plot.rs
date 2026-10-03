//! Every chart the GUI draws goes through here, so the rest of the GUI names
//! what it wants to see — lines, axes, a follow window — and not how a chart
//! library spells it.

use std::sync::Arc;

use egui::{Color32, Ui};
use egui_sc::{
    egui_charts::{Axis, Chart, ChartTheme, ChartWidget, Harmony, Series, SymbolKind, XyData},
    egui_components::ShadcnTheme,
};

/// One line on a chart.
#[derive(Clone)]
pub struct Line {
    /// Legend label.
    pub name: String,
    /// `(x, y)` points; NaN y breaks the line.
    pub data: Arc<Vec<[f64; 2]>>,
    /// Colour, fixed per signal so a channel looks the same everywhere.
    pub color: Color32,
    /// Dashed instead of solid (fits, predictions).
    pub dashed: bool,
    /// Draw markers on the points (sparse data: Bode points, DC levels).
    pub markers: bool,
    /// x never decreases, so the chart can skip checking (stream buffers,
    /// which can hold a million points).
    pub sorted: bool,
}

impl Line {
    /// A solid line without markers.
    pub fn new(name: impl Into<String>, data: Arc<Vec<[f64; 2]>>, color: Color32) -> Self {
        Self {
            name: name.into(),
            data,
            color,
            dashed: false,
            markers: false,
            sorted: false,
        }
    }

    /// x is known to be non-decreasing.
    pub fn sorted(mut self) -> Self {
        self.sorted = true;
        self
    }

    /// Dashed.
    pub fn dashed(mut self) -> Self {
        self.dashed = true;
        self
    }

    /// With point markers.
    pub fn markers(mut self) -> Self {
        self.markers = true;
        self
    }
}

/// One axis.
#[derive(Clone, Default)]
pub struct AxisSpec {
    /// Title with unit.
    pub title: String,
    /// Log10 scale.
    pub log: bool,
    /// Fixed range; `None` fits the data.
    pub range: Option<(f64, f64)>,
}

impl AxisSpec {
    /// Linear axis with a title.
    pub fn linear(title: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            ..Default::default()
        }
    }

    /// Log axis with a title.
    pub fn log(title: impl Into<String>) -> Self {
        Self {
            title: title.into(),
            log: true,
            range: None,
        }
    }

    /// Fixed range.
    pub fn range(mut self, lo: f64, hi: f64) -> Self {
        self.range = Some((lo, hi));
        self
    }
}

/// A chart request.
pub struct Plot<'a> {
    /// Stable id: zoom state is kept per id.
    pub id: &'a str,
    /// Horizontal axis.
    pub x: AxisSpec,
    /// Vertical axis.
    pub y: AxisSpec,
    /// Lines, drawn in order.
    pub lines: Vec<Line>,
    /// Height in points.
    pub height: f32,
}

/// Colour of each rig channel, the same on every chart.
pub fn channel_color(ch: usize) -> Color32 {
    match ch {
        0 => Color32::from_rgb(0x3b, 0x82, 0xf6),
        1 => Color32::from_rgb(0xf5, 0x9e, 0x0b),
        _ => Color32::from_rgb(0x10, 0xb9, 0x81),
    }
}

/// Colour for fits and predictions.
pub fn reference_color() -> Color32 {
    Color32::from_rgb(0xa8, 0x55, 0xf7)
}

/// Draw the chart: an `egui_charts` XY chart, zoomable (wheel; Shift for x
/// only, Ctrl for y only), pannable (drag), reset by double-click. A zoom
/// overrides the axis range asked for here until it is reset, so a follow
/// window stops following while someone is looking closely.
pub fn show(ui: &mut Ui, plot: Plot<'_>) {
    let axis = |spec: &AxisSpec| {
        let mut a = if spec.log { Axis::log() } else { Axis::value() };
        if !spec.title.is_empty() {
            a = a.name(spec.title.clone());
        }
        if let Some((lo, hi)) = spec.range {
            a = a.min(lo).max(hi);
        }
        a
    };
    let mut chart = Chart::new().x_axis(axis(&plot.x)).y_axis(axis(&plot.y));
    for line in plot.lines {
        let mut data = XyData::from_arc_vec(line.data);
        if line.sorted {
            data = data.assume_sorted();
        }
        let mut series = Series::xy_line(line.name)
            .data(data)
            .color(line.color)
            .width(if line.dashed { 1.2 } else { 1.5 });
        if line.dashed {
            series = series.dashed();
        }
        if line.markers {
            series = series.markers(SymbolKind::Circle).marker_size(5.0);
        }
        chart = chart.series(series);
    }

    // The chart sits inside a card already: no second background or border.
    let ctx = ui.ctx().clone();
    let mut theme = ChartTheme::follow_egui(
        &ctx,
        ShadcnTheme::get(&ctx).primary,
        Harmony::Square,
        chart.series.len().max(3),
    );
    theme.background = Color32::TRANSPARENT;
    ChartWidget::new(&chart)
        .id(egui::Id::new(("plant_trace_plot", plot.id)))
        .theme(theme)
        .interactive(true)
        .min_size(egui::vec2(ui.available_width(), plot.height))
        .show(ui);
}
