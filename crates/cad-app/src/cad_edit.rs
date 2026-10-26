//! CAD selection editing controls. Every button creates one batch action.

use std::collections::{BTreeMap, BTreeSet};

use bevy_egui::egui;
use cad_core::model::{Color, DrawingDocument, EntityId, LayerId};
use cad_core::properties::{self, GripKind};
use cad_core::{DrawingEdit, LayerEdit};

#[derive(Clone)]
pub enum EditAction {
    Annotation(Vec<cad_core::Geometry>),
    Paste {
        items: Vec<cad_core::geometry_edit::ClipboardItem>,
        offset: [f64; 3],
    },
    Apply(DrawingEdit),
    Create(Vec<cad_core::geometry_edit::NewEntity>),
    Transform {
        transforms: Vec<cad_core::geometry_edit::AffineTransform>,
        copies: bool,
    },
    Geometry(BTreeMap<EntityId, cad_core::Geometry>),
    Curves(cad_core::CurveEdit),
    Layer(LayerEdit),
    BeginGrip {
        id: EntityId,
        kind: GripKind,
        target: [f64; 3],
    },
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
    radius: f64,
    grip: usize,
    point: [f64; 3],
    color_changed: bool,
    layer_changed: bool,
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
            radius: 1.0,
            grip: 0,
            point: [0.0; 3],
            color_changed: false,
            layer_changed: false,
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
            controls.color_changed = false;
            controls.layer_changed = false;
            controls.radius = properties::radius(&entity.geometry).unwrap_or(1.0);
            controls.grip = 0;
            if let Some(grip) = properties::grips(&entity.geometry).first() {
                controls.point = [grip.point.x, grip.point.y, grip.point.z];
            }
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
        let mixed_color = document
            .entities
            .iter()
            .any(|entity| selected.contains(&entity.id.0) && Some(entity.color) != first_color);
        let first_layer = document
            .entities
            .iter()
            .find(|e| selected.contains(&e.id.0))
            .map(|e| e.layer_id);
        let mixed_layer = document
            .entities
            .iter()
            .any(|e| selected.contains(&e.id.0) && Some(e.layer_id) != first_layer);
        if mixed_color {
            ui.weak("所选图元包含不同颜色");
        }
        egui::ComboBox::from_id_salt("cad_edit_color")
            .selected_text(if mixed_color && !controls.color_changed {
                "*多个颜色值*".to_string()
            } else {
                match controls.color {
                    Color::ByLayer | Color::None => "随层".to_string(),
                    Color::ByBlock => "随块".to_string(),
                    Color::Aci(index) => format!("ACI {index}"),
                    Color::Rgb { .. } => "自定义 RGB".to_string(),
                }
            })
            .show_ui(ui, |ui| {
                let [r, g, b] = controls.rgb;
                for (value, label) in [
                    (Color::ByLayer, "随层"),
                    (Color::ByBlock, "随块"),
                    (Color::Aci(controls.aci), "ACI 索引色"),
                    (Color::Rgb { r, g, b }, "自定义 RGB"),
                ] {
                    if ui
                        .selectable_value(&mut controls.color, value, label)
                        .clicked()
                    {
                        controls.color_changed = true;
                    }
                }
            });
        match controls.color {
            Color::Aci(_) => {
                ui.horizontal(|ui| {
                    ui.label("ACI");
                    if ui
                        .add(egui::DragValue::new(&mut controls.aci).range(1..=255))
                        .changed()
                    {
                        controls.color_changed = true;
                    }
                });
                controls.color = Color::Aci(controls.aci);
            }
            Color::Rgb { .. } => {
                ui.horizontal(|ui| {
                    ui.label("颜色");
                    if ui.color_edit_button_srgb(&mut controls.rgb).changed() {
                        controls.color_changed = true;
                    }
                });
                let [r, g, b] = controls.rgb;
                controls.color = Color::Rgb { r, g, b };
            }
            _ => {}
        }
        if ui
            .add_enabled(
                !mixed_color || controls.color_changed,
                egui::Button::new("应用颜色到所选图元"),
            )
            .clicked()
        {
            action = Some(EditAction::Apply(DrawingEdit::SetColor(controls.color)));
        }
        ui.separator();
        egui::ComboBox::from_id_salt("cad_edit_layer")
            .selected_text(if mixed_layer && !controls.layer_changed {
                "*多个图层值*"
            } else {
                controls
                    .layer
                    .and_then(|id| document.layer(id))
                    .map(|layer| layer.name.as_str())
                    .unwrap_or("选择目标图层")
            })
            .show_ui(ui, |ui| {
                for layer in &document.layers {
                    if ui
                        .add_enabled(
                            layer.visible && !layer.locked,
                            egui::Button::selectable(controls.layer == Some(layer.id), &layer.name),
                        )
                        .clicked()
                    {
                        controls.layer = Some(layer.id);
                        controls.layer_changed = true;
                    }
                }
            });
        if ui
            .add_enabled(
                controls.layer.is_some() && (!mixed_layer || controls.layer_changed),
                egui::Button::new("应用图层到所选图元"),
            )
            .clicked()
        {
            action = Some(EditAction::Apply(DrawingEdit::SetLayer(
                controls.layer.unwrap(),
            )));
        }
        ui.separator();
        let entities: Vec<_> = document
            .entities
            .iter()
            .filter(|e| selected.contains(&e.id.0))
            .collect();
        if !entities.is_empty()
            && entities
                .iter()
                .all(|e| properties::radius(&e.geometry).is_some())
        {
            let first = properties::radius(&entities[0].geometry).unwrap();
            let mixed = entities
                .iter()
                .any(|e| properties::radius(&e.geometry) != Some(first));
            ui.label(if mixed {
                "当前半径：*多个值*".into()
            } else {
                format!("当前半径：{first:.6} mm")
            });
            ui.horizontal(|ui| {
                ui.label("目标半径 mm");
                ui.add(egui::DragValue::new(&mut controls.radius).speed(0.1));
            });
            if ui
                .add_enabled(
                    controls.radius.is_finite() && controls.radius > 1e-9,
                    egui::Button::new("应用半径到所选圆 / 圆弧"),
                )
                .clicked()
            {
                let updates = entities
                    .iter()
                    .map(|e| {
                        (
                            e.id,
                            properties::with_radius(&e.geometry, controls.radius)
                                .expect("radius applicability and value checked above"),
                        )
                    })
                    .collect();
                action = Some(EditAction::Geometry(updates));
            }
        }
        if entities.len() == 1 {
            let e = entities[0];
            let grips = properties::grips(&e.geometry);
            if !grips.is_empty() {
                controls.grip = controls.grip.min(grips.len() - 1);
                let before = controls.grip;
                egui::ComboBox::from_id_salt("geometry_grip")
                    .selected_text(&grips[controls.grip].label)
                    .show_ui(ui, |ui| {
                        for (i, g) in grips.iter().enumerate() {
                            ui.selectable_value(&mut controls.grip, i, &g.label);
                        }
                    });
                if before != controls.grip {
                    let p = grips[controls.grip].point;
                    controls.point = [p.x, p.y, p.z];
                }
                egui::Grid::new("grip_point").num_columns(2).show(ui, |ui| {
                    for (label, value) in ["X mm", "Y mm", "Z mm"]
                        .into_iter()
                        .zip(&mut controls.point)
                    {
                        ui.label(label);
                        ui.add(egui::DragValue::new(value).speed(0.1));
                        ui.end_row();
                    }
                });
                if ui.button("预览夹点坐标修改").clicked() {
                    action = Some(EditAction::BeginGrip {
                        id: e.id,
                        kind: grips[controls.grip].kind,
                        target: controls.point,
                    });
                }
                ui.small("也可点击画布夹点，移动或输入后再次点击 / Enter 提交；Esc 取消。");
            } else {
                ui.weak("此样条暂无可编辑控制点（周期 / 仅拟合点）。");
            }
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
