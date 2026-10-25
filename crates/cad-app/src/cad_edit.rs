//! CAD selection editing controls. Every button creates one batch action.

use std::collections::BTreeSet;

use bevy_egui::egui;
use cad_core::model::{Color, DrawingDocument, EntityId, LayerId};
use cad_core::DrawingEdit;

#[derive(Clone, Copy)]
pub enum EditAction {
    Apply(DrawingEdit),
    Undo,
    Redo,
}

pub struct EditControls {
    selected: BTreeSet<u32>,
    color: Color,
    rgb: [u8; 3],
    aci: u8,
    layer: Option<LayerId>,
    offset: [f64; 3],
}

impl Default for EditControls {
    fn default() -> Self {
        Self {
            selected: BTreeSet::new(),
            color: Color::ByLayer,
            rgb: [255; 3],
            aci: 7,
            layer: None,
            offset: [0.0; 3],
        }
    }
}

pub fn editing_ui(
    ui: &mut egui::Ui,
    document: &DrawingDocument,
    selected: &BTreeSet<u32>,
    controls: &mut EditControls,
) -> Option<EditAction> {
    let mut action = None;
    if controls.selected != *selected {
        controls.selected = selected.clone();
        if let Some(entity) = document
            .entities
            .iter()
            .find(|entity| selected.contains(&entity.id.0))
        {
            controls.color = entity.color;
            controls.layer = Some(entity.layer_id);
            match entity.color {
                Color::Rgb { r, g, b } => controls.rgb = [r, g, b],
                Color::Aci(index) => controls.aci = index,
                _ => {}
            }
        }
    }
    ui.strong(format!("批量编辑 · {} 个图元", selected.len()));
    ui.add_enabled_ui(!selected.is_empty(), |ui| {
        let first_color = document
            .entities
            .iter()
            .find(|entity| selected.contains(&entity.id.0))
            .map(|entity| entity.color);
        if document
            .entities
            .iter()
            .any(|entity| selected.contains(&entity.id.0) && Some(entity.color) != first_color)
        {
            ui.weak("所选图元包含不同颜色");
        }
        egui::ComboBox::from_id_salt("cad_edit_color")
            .selected_text(match controls.color {
                Color::ByLayer | Color::None => "随层".to_string(),
                Color::ByBlock => "随块".to_string(),
                Color::Aci(index) => format!("ACI {index}"),
                Color::Rgb { .. } => "自定义 RGB".to_string(),
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut controls.color, Color::ByLayer, "随层");
                ui.selectable_value(&mut controls.color, Color::ByBlock, "随块");
                ui.selectable_value(&mut controls.color, Color::Aci(controls.aci), "ACI 索引色");
                let [r, g, b] = controls.rgb;
                ui.selectable_value(&mut controls.color, Color::Rgb { r, g, b }, "自定义 RGB");
            });
        match controls.color {
            Color::Aci(_) => {
                ui.horizontal(|ui| {
                    ui.label("ACI");
                    ui.add(egui::DragValue::new(&mut controls.aci).range(1..=255));
                });
                controls.color = Color::Aci(controls.aci);
            }
            Color::Rgb { .. } => {
                ui.horizontal(|ui| {
                    ui.label("颜色");
                    ui.color_edit_button_srgb(&mut controls.rgb);
                });
                let [r, g, b] = controls.rgb;
                controls.color = Color::Rgb { r, g, b };
            }
            _ => {}
        }
        if ui.button("应用颜色到所选图元").clicked() {
            action = Some(EditAction::Apply(DrawingEdit::SetColor(controls.color)));
        }
        ui.separator();
        egui::ComboBox::from_id_salt("cad_edit_layer")
            .selected_text(
                controls
                    .layer
                    .and_then(|id| document.layer(id))
                    .map(|layer| layer.name.as_str())
                    .unwrap_or("选择目标图层"),
            )
            .show_ui(ui, |ui| {
                for layer in &document.layers {
                    ui.selectable_value(&mut controls.layer, Some(layer.id), &layer.name);
                }
            });
        if ui
            .add_enabled(
                controls.layer.is_some(),
                egui::Button::new("应用图层到所选图元"),
            )
            .clicked()
        {
            action = Some(EditAction::Apply(DrawingEdit::SetLayer(
                controls.layer.unwrap(),
            )));
        }
        ui.separator();
        egui::Grid::new("cad_move_delta")
            .num_columns(2)
            .show(ui, |ui| {
                for (label, value) in ["ΔX (mm)", "ΔY (mm)", "ΔZ (mm)"]
                    .into_iter()
                    .zip(&mut controls.offset)
                {
                    ui.label(label);
                    ui.add(egui::DragValue::new(value).speed(0.1));
                    ui.end_row();
                }
            });
        if ui.button("按三维偏移移动所选图元").clicked() {
            action = Some(EditAction::Apply(DrawingEdit::Translate(controls.offset)));
        }
        ui.separator();
        if ui.button("删除所选图元  Delete").clicked() {
            action = Some(EditAction::Apply(DrawingEdit::Delete));
        }
    });
    ui.separator();
    action
}

pub fn entity_ids(selected: &BTreeSet<u32>) -> BTreeSet<EntityId> {
    selected.iter().copied().map(EntityId).collect()
}
