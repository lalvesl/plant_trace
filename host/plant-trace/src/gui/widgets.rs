//! Small form helpers shared by the tabs.
//!
//! A lab form is mostly labelled numbers with units, so that row is the one
//! thing every tab needs; it is built from egui_shadcn components only.

use egui::Ui;
use egui_sc::egui_components::*;

/// A labelled numeric field with its unit: egui_shadcn's `NumberInput`, so
/// the value is always an editable box — typed, committed on Enter or when
/// focus leaves, stepped with the arrow keys. Returns true when a new value
/// was committed.
fn num<T: egui::emath::Numeric>(
    ui: &mut Ui,
    label: &str,
    value: &mut T,
    range: std::ops::RangeInclusive<T>,
    step: f64,
    unit: &str,
) -> bool {
    ui.horizontal(|ui| {
        // Keyed by position, not by `ui.id()`: in egui that is the *stable*
        // id of the row, identical for two rows built the same way in two
        // cards (both forms of the Output check tab have an "Outputs" row).
        // The next auto id is unique per widget position.
        let id = ui.next_auto_id().with(label);
        field_label(ui, label);
        let mut field = NumberInput::new(id, value)
            .range(range)
            .step(step)
            .width(130.0);
        let unit = unit.trim();
        if !unit.is_empty() {
            field = field.unit(unit);
        }
        field.show(ui)
    })
    .inner
}

/// A labelled `f32`.
pub fn num_f32(
    ui: &mut Ui,
    label: &str,
    value: &mut f32,
    range: std::ops::RangeInclusive<f32>,
    step: f64,
    unit: &str,
) -> bool {
    num(ui, label, value, range, step, unit)
}

/// A labelled integer.
pub fn num_u32(
    ui: &mut Ui,
    label: &str,
    value: &mut u32,
    range: std::ops::RangeInclusive<u32>,
    unit: &str,
) -> bool {
    num(ui, label, value, range, 1.0, unit)
}

/// A labelled `f64`, for frequencies and durations.
pub fn num_f64(
    ui: &mut Ui,
    label: &str,
    value: &mut f64,
    range: std::ops::RangeInclusive<f64>,
    step: f64,
    unit: &str,
) -> bool {
    num(ui, label, value, range, step, unit)
}

/// The fixed-width label every form row starts with, so the fields line up.
pub fn field_label(ui: &mut Ui, text: &str) {
    ui.allocate_ui_with_layout(
        egui::vec2(150.0, 20.0),
        egui::Layout::left_to_right(egui::Align::Center),
        |ui| {
            ui.set_min_width(150.0);
            ui.add(egui::Label::new(text).truncate());
        },
    );
}

/// Card with a title and an optional description.
pub fn section<R>(
    ui: &mut Ui,
    title: &str,
    description: Option<&str>,
    body: impl FnOnce(&mut Ui) -> R,
) -> R {
    let mut out = None;
    Card::new().show(ui, |ui| {
        // Fill the column: stacked cards of different widths read as a mess.
        ui.set_min_width(ui.available_width());
        card_header(ui, title, description);
        out = Some(body(ui));
    });
    out.expect("card body runs")
}

/// Pick one of a fixed list by index, returning true on change.
pub fn choose(ui: &mut Ui, label: &str, current: &mut usize, options: &[&str]) -> bool {
    ui.horizontal(|ui| {
        field_label(ui, label);
        let mut sel = Some(*current);
        // No explicit id: the Select keys its open state by its own widget
        // id, which is unique per position. `ui.id()` is not (see `num`).
        let changed = Select::new(&mut sel, options).width(180.0).show(ui);
        if let Some(s) = sel {
            *current = s;
        }
        changed
    })
    .inner
}

/// Two panes side by side when there is room, stacked when there is not.
/// `state` is handed to both closures so they can each borrow it mutably.
pub fn two<S>(
    ui: &mut Ui,
    state: &mut S,
    left: impl FnOnce(&mut Ui, &mut S),
    right: impl FnOnce(&mut Ui, &mut S),
) {
    if ui.available_width() >= 1100.0 {
        ui.columns(2, |cols| {
            let (a, b) = cols.split_at_mut(1);
            left(&mut a[0], state);
            right(&mut b[0], state);
        });
    } else {
        left(ui, state);
        Spacing::Sm.show(ui);
        right(ui, state);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::{Event, Modifiers, PointerButton, Pos2, RawInput, Rect, Vec2};

    fn input(click: Option<Pos2>) -> RawInput {
        let mut i = RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(800.0, 800.0))),
            ..Default::default()
        };
        if let Some(pos) = click {
            i.events = vec![
                Event::PointerMoved(pos),
                Event::PointerButton {
                    pos,
                    button: PointerButton::Primary,
                    pressed: true,
                    modifiers: Modifiers::default(),
                },
                Event::PointerButton {
                    pos,
                    button: PointerButton::Primary,
                    pressed: false,
                    modifiers: Modifiers::default(),
                },
            ];
        }
        i
    }

    /// The Output check tab's shape: two cards, each with an "Outputs" row.
    fn two_forms(
        ctx: &egui::Context,
        click: Option<Pos2>,
        a: &mut usize,
        b: &mut usize,
    ) -> [Rect; 2] {
        let mut rects = [Rect::NOTHING; 2];
        let mut out = ctx.run_ui(input(click), |ui| {
            for (k, v) in [&mut *a, &mut *b].into_iter().enumerate() {
                section(ui, "form", None, |ui| {
                    rects[k] = ui
                        .scope(|ui| choose(ui, "Outputs", v, &["Both", "u_T", "p_s"]))
                        .response
                        .rect;
                    // Room for an open list below the row.
                    ui.allocate_space(Vec2::new(10.0, 150.0));
                });
            }
        });
        out.textures_delta.clear();
        rects
    }

    #[test]
    fn two_rows_with_the_same_label_in_two_cards_open_separately() {
        let ctx = egui::Context::default();
        ctx.set_fonts(font_definitions());
        let (mut a, mut b) = (0usize, 0usize);
        let [ra, rb] = two_forms(&ctx, None, &mut a, &mut b);
        // The trigger sits right of the 150-point label.
        let trigger = |r: Rect| Pos2::new(r.left() + 150.0 + 60.0, r.center().y);
        two_forms(&ctx, Some(trigger(ra)), &mut a, &mut b);
        two_forms(&ctx, None, &mut a, &mut b);
        // Second option ("u_T") of B's list, if B had opened as well.
        let second_option_b = Pos2::new(trigger(rb).x, rb.bottom() + 4.0 + 4.0 + 32.0 + 16.0);
        two_forms(&ctx, Some(second_option_b), &mut a, &mut b);
        two_forms(&ctx, None, &mut a, &mut b);
        assert_eq!(b, 0, "B's list opened together with A's");

        // A's list was open: its second option is where expected.
        two_forms(&ctx, Some(trigger(ra)), &mut a, &mut b);
        two_forms(&ctx, None, &mut a, &mut b);
        let second_option_a = Pos2::new(trigger(ra).x, ra.bottom() + 4.0 + 4.0 + 32.0 + 16.0);
        two_forms(&ctx, Some(second_option_a), &mut a, &mut b);
        assert_eq!(a, 1, "A's second option should have been picked");
    }
}
