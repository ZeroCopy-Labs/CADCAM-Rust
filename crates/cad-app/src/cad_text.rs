//! Basic font rendering/editing; native TEXT remains authoritative for DXF.
use bevy::prelude::*;
use bevy_egui::egui;
use cad_core::model::{Geometry, Text};

#[derive(Default)]
pub struct Controls {
    source: Option<(u32, u64)>,
    content: String,
    height: f64,
}
impl Controls {
    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        id: u32,
        revision: u64,
        text: &Text,
    ) -> Option<Geometry> {
        if self.source != Some((id, revision)) {
            self.source = Some((id, revision));
            self.content = text.content.clone();
            self.height = text.height;
        }
        ui.strong("单行文字属性");
        ui.text_edit_singleline(&mut self.content);
        ui.horizontal(|ui| {
            ui.label("字高 mm");
            ui.add(egui::DragValue::new(&mut self.height).speed(0.1));
        });
        if ui.button("应用文字与字高").clicked() {
            let mut text = text.clone();
            text.content = self.content.clone();
            text.height = self.height;
            return Some(Geometry::Text(text));
        }
        None
    }
}
pub fn paint(
    context: &egui::Context,
    geometry: &Geometry,
    frame: crate::cad_view::ViewFrame,
    camera: &Camera,
    transform: &GlobalTransform,
    color: egui::Color32,
    clip: egui::Rect,
) {
    let Geometry::Text(t) = geometry else { return };
    let p = t.position;
    let u = t.plane.x_axis * t.height;
    let v = t.plane.y_axis() * t.height;
    let Some(anchor) = frame.project(camera, transform, [p.x, p.y, p.z]) else {
        return;
    };
    let Some(end) = frame.project(camera, transform, [p.x + u.x, p.y + u.y, p.z + u.z]) else {
        return;
    };
    let Some(top) = frame.project(camera, transform, [p.x + v.x, p.y + v.y, p.z + v.z]) else {
        return;
    };
    let height = top.distance(anchor);
    if height < 1.0 || end.distance(anchor) < 0.1 {
        return;
    }
    let painter = context
        .layer_painter(egui::LayerId::new(
            egui::Order::Background,
            egui::Id::new("cad_text"),
        ))
        .with_clip_rect(clip);
    let galley = painter.layout_no_wrap(
        t.content.clone(),
        egui::FontId::proportional(height.min(2048.0)),
        color,
    );
    let d = end - anchor;
    let mut shape = egui::epaint::TextShape::new(egui::pos2(top.x, top.y), galley, color);
    shape.angle = d.y.atan2(d.x);
    painter.add(shape);
}
