use crate::controls::{Action, Controls};
use bounce::DrawMode;
use wgame_egui::{Canvas, egui};

pub fn layout(ui: &mut egui::Ui, canvas: &Canvas, controls: &mut Controls) -> egui::Response {
    egui::Panel::top("toolbar").show(ui, |ui| {
        ui.horizontal_wrapped(|ui| {
            ui.strong("BOUNCE");
            ui.label(format!("{} bodies", controls.bodies));
            for (action, label) in [
                (Action::Pause, if controls.paused { "Resume [Space]" } else { "Pause [Space]" }),
                (Action::Add, "Add [+]"), (Action::Remove, "Remove [-]"),
                (Action::Slow, if controls.slow { "Normal speed [S]" } else { "Slow motion [S]" }),
                (Action::Mode, if controls.mode == DrawMode::Normal { "Debug [\\]" } else { "Normal [\\]" }),
                (Action::Reset, "Reset [R]"),
            ] {
                if ui.button(label).clicked() { controls.actions.push(action); }
            }
        });
        ui.small("Drag a body with the mouse or touch. Click the playground for keyboard shortcuts. Esc: quit.");
    });
    egui::CentralPanel::default()
        .frame(egui::Frame::NONE)
        .show(ui, |ui| canvas.show(ui))
        .inner
}
