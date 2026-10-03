//! The GUI, driven headlessly against the simulator: no window, no GPU — a
//! real `egui::Context` stepped frame by frame, as egui_shadcn tests its own
//! components.
#![cfg(feature = "gui")]

use std::time::{Duration, Instant};

use egui::{Pos2, RawInput, Rect, Vec2};
use plant_trace::{
    gui::{App, Options},
    sim,
};

fn frame(ctx: &egui::Context, app: &mut App) {
    let input = RawInput {
        screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400.0, 900.0))),
        ..Default::default()
    };
    let mut out = ctx.run_ui(input, |ui| app.show(ui));
    out.textures_delta.clear();
}

#[test]
fn the_gui_connects_streams_and_draws_every_tab() {
    let addr = "127.0.0.1:7841";
    std::thread::spawn(move || {
        let _ = sim::run(sim::Options {
            addr: addr.into(),
            speed: 5.0,
            settle_s: 1.0,
            ..sim::Options::default()
        });
    });
    std::thread::sleep(Duration::from_millis(200));

    let ctx = egui::Context::default();
    let mut app = App::new(
        &ctx,
        Options {
            daq: Some(format!("tcp://{addr}")),
            light: false,
        },
    );
    let deadline = Instant::now() + Duration::from_secs(10);
    while app.streaming().is_none() && Instant::now() < deadline {
        frame(&ctx, &mut app);
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(app.is_connected(), "never connected to the simulator");
    assert!(app.streaming().is_some(), "the live view never started");

    for tab in 0..5 {
        app.set_tab(tab);
        for _ in 0..5 {
            frame(&ctx, &mut app);
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}
