//! Object snaps and planar drafting aids shared by command point acquisition.
use super::cad_view::ViewFrame;
use bevy::prelude::*;
use bevy_egui::egui;
use cad_core::{
    model::{Point3, Vector3, WorkPlane},
    snap::{self, SnapKind},
    DrawingDocument,
};
use std::collections::BTreeSet;

#[derive(Clone)]
struct Candidate {
    point: Point3,
    kind: SnapKind,
    distance: f32,
}
pub struct DraftingAids {
    pub enabled: bool,
    pub magnet: bool,
    pub grid: bool,
    pub grid_snap: bool,
    pub ortho: bool,
    pub polar: bool,
    spacing: f64,
    angle: f64,
    project: bool,
    modes: BTreeSet<SnapKind>,
    candidates: Vec<Candidate>,
    index: usize,
    cursor: Option<Vec2>,
    marker: Option<Candidate>,
    clipped: bool,
}
impl Default for DraftingAids {
    fn default() -> Self {
        Self {
            enabled: true,
            magnet: true,
            grid: false,
            grid_snap: false,
            ortho: false,
            polar: false,
            spacing: 10.0,
            angle: 15.0,
            project: false,
            modes: [
                SnapKind::Endpoint,
                SnapKind::Midpoint,
                SnapKind::Center,
                SnapKind::Quadrant,
                SnapKind::Intersection,
            ]
            .into_iter()
            .collect(),
            candidates: Vec::new(),
            index: 0,
            cursor: None,
            marker: None,
            clipped: false,
        }
    }
}
fn array(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}
impl DraftingAids {
    pub fn snap_point(&self, point: Point3) -> Option<Point3> {
        self.marker
            .as_ref()
            .filter(|marker| self.enabled && snap::vector(marker.point, point).length() < 1e-6)
            .map(|_| point)
    }
    pub fn has_candidates(&self) -> bool {
        !self.candidates.is_empty()
    }
    pub fn clear(&mut self) {
        self.marker = None;
        self.clipped = false;
        self.candidates.clear();
        self.cursor = None;
        self.index = 0;
    }
    pub fn cycle(&mut self, reverse: bool) -> Option<Point3> {
        if self.candidates.is_empty() {
            return None;
        }
        let n = self.candidates.len();
        self.index = (self.index + if reverse { n - 1 } else { 1 }) % n;
        self.marker = Some(self.candidates[self.index].clone());
        Some(self.candidates[self.index].point)
    }
    pub fn resolve(
        &mut self,
        document: &DrawingDocument,
        frame: ViewFrame,
        camera: &Camera,
        transform: &GlobalTransform,
        cursor: Vec2,
        ray: (Point3, Vector3),
        plane: WorkPlane,
        reference: Option<Point3>,
    ) -> Result<Point3, String> {
        let (origin, direction) = ray;
        let mut candidates = Vec::new();
        let mut push = |point: Point3, kind: SnapKind| {
            let point = if self.project {
                let mut local = plane.to_local(point);
                local[2] = 0.0;
                plane.to_world(local)
            } else {
                point
            };
            if let Some(screen) = frame.project(camera, transform, array(point)) {
                let distance = screen.distance(cursor);
                if distance <= super::PICK_TOLERANCE {
                    candidates.push(Candidate {
                        point,
                        kind,
                        distance,
                    });
                }
            }
        };
        if self.enabled {
            let mut near_entities = Vec::new();
            let radius = frame.pixel_size * super::PICK_TOLERANCE as f64;
            for entity in &document.entities {
                if !entity.visible || document.layer(entity.layer_id).is_some_and(|l| !l.visible) {
                    continue;
                }
                let mut modes = self.modes.clone();
                let near = (self.modes.contains(&SnapKind::Nearest)
                    || self.modes.contains(&SnapKind::Intersection))
                    && snap::near_ray(&entity.geometry, origin, direction, radius);
                if near {
                    near_entities.push(entity);
                }
                if !near {
                    modes.remove(&SnapKind::Nearest);
                }
                for p in snap::snap_points(&entity.geometry, origin, direction, reference, &modes) {
                    push(p.point, p.kind);
                }
            }
            if self.modes.contains(&SnapKind::Intersection) {
                self.clipped = near_entities.len() > 24;
                let entities: Vec<_> = near_entities.into_iter().take(24).collect();
                for entity in &entities {
                    if matches!(entity.geometry, cad_core::Geometry::Polyline(_)) {
                        for p in snap::intersections(
                            &entity.geometry,
                            &entity.geometry,
                            origin,
                            direction,
                            radius,
                        ) {
                            push(p, SnapKind::Intersection);
                        }
                    }
                }
                for i in 0..entities.len() {
                    for j in i + 1..entities.len() {
                        for p in snap::intersections(
                            &entities[i].geometry,
                            &entities[j].geometry,
                            origin,
                            direction,
                            frame.pixel_size * super::PICK_TOLERANCE as f64,
                        ) {
                            push(p, SnapKind::Intersection);
                        }
                    }
                }
            }
        }
        candidates.sort_by(|a, b| {
            ((a.distance / 2.0).floor() as i32)
                .cmp(&((b.distance / 2.0).floor() as i32))
                .then(a.kind.cmp(&b.kind))
                .then(a.distance.total_cmp(&b.distance))
        });
        let mut unique: Vec<Candidate> = Vec::new();
        for candidate in candidates {
            if !unique
                .iter()
                .any(|p| snap::vector(p.point, candidate.point).length() < 1e-6)
            {
                unique.push(candidate);
            }
        }
        if self.cursor.is_none_or(|p| p.distance(cursor) > 3.0)
            || unique.len() != self.candidates.len()
        {
            self.index = 0;
        }
        self.cursor = Some(cursor);
        self.candidates = unique;
        if !self.candidates.is_empty() {
            self.index %= self.candidates.len();
            let candidate = self.candidates[self.index].clone();
            let point = candidate.point;
            self.marker = Some(candidate);
            return Ok(point);
        }
        self.marker = None;
        let point = plane.intersect_ray(origin, direction)?;
        if let Some(reference) = reference {
            let local = plane.to_local(point);
            let base = plane.to_local(reference);
            let u = local[0] - base[0];
            let v = local[1] - base[1];
            if self.ortho || self.polar {
                let step = if self.ortho {
                    std::f64::consts::FRAC_PI_2
                } else {
                    self.angle.to_radians()
                };
                let angle = (v.atan2(u) / step).round() * step;
                let length = u * angle.cos() + v * angle.sin();
                return Ok(plane.to_world([
                    base[0] + length * angle.cos(),
                    base[1] + length * angle.sin(),
                    base[2],
                ]));
            }
        }
        if self.grid_snap {
            let mut local = plane.to_local(point);
            local[0] = (local[0] / self.spacing).round() * self.spacing;
            local[1] = (local[1] / self.spacing).round() * self.spacing;
            return Ok(plane.to_world(local));
        }
        Ok(point)
    }
    pub fn snap_settings(&mut self, ui: &mut egui::Ui) {
        ui.set_max_width(320.0);
        let mut changed = ui.checkbox(&mut self.enabled, "对象捕捉 F3").changed();
        changed |= ui
            .checkbox(&mut self.magnet, "自动吸附（磁吸）")
            .on_hover_text("取点时将十字光标吸附到捕捉点；关闭只取消光标吸附，仍保留精确捕捉。")
            .changed();
        changed |= ui
            .checkbox(&mut self.project, "将捕捉点投影到工作平面")
            .changed();
        ui.separator();
        for kind in [
            SnapKind::Endpoint,
            SnapKind::Midpoint,
            SnapKind::Center,
            SnapKind::Quadrant,
            SnapKind::Intersection,
            SnapKind::Perpendicular,
            SnapKind::Tangent,
            SnapKind::Nearest,
        ] {
            let mut enabled = self.modes.contains(&kind);
            if ui.checkbox(&mut enabled, kind.label()).changed() {
                if enabled {
                    self.modes.insert(kind);
                } else {
                    self.modes.remove(&kind);
                }
                changed = true;
            }
        }
        ui.add(
            egui::Label::new(
                egui::RichText::new(
                    "Tab / Shift+Tab 切换候选；交点、垂足和切点按支持的曲线类型提供。",
                )
                .small(),
            )
            .wrap(),
        );
        if self.clipped {
            ui.small("交点仅搜索当前 24 个邻近图元，可放大视图缩小范围");
        }
        if changed {
            self.clear();
        }
    }
    pub fn grid_settings(&mut self, ui: &mut egui::Ui) {
        ui.set_max_width(320.0);
        let mut changed = ui.checkbox(&mut self.grid, "显示栅格 F7").changed();
        changed |= ui.checkbox(&mut self.grid_snap, "栅格捕捉 F9").changed();
        ui.horizontal(|ui| {
            ui.label("栅格间距 mm");
            changed |= ui
                .add(
                    egui::DragValue::new(&mut self.spacing)
                        .range(0.000001..=1e9)
                        .speed(0.1),
                )
                .changed();
        });
        if !self.spacing.is_finite() || self.spacing <= 0.0 {
            self.spacing = 10.0;
            changed = true;
        }
        ui.add(
            egui::Label::new(
                egui::RichText::new("每 5 格显示粗主线；显示随视角与缩放按 5 倍层级稀疏，捕捉间距保持设置值。")
                    .small(),
            )
            .wrap(),
        );
        if changed {
            self.clear();
        }
    }
    pub fn direction_settings(&mut self, ui: &mut egui::Ui) {
        let mut changed = false;
        if ui.checkbox(&mut self.ortho, "正交 F8").changed() {
            if self.ortho {
                self.polar = false;
            }
            changed = true;
        }
        if ui.checkbox(&mut self.polar, "极轴 F10").changed() {
            if self.polar {
                self.ortho = false;
            }
            changed = true;
        }
        ui.horizontal(|ui| {
            ui.label("极轴增量 °");
            changed |= ui
                .add(
                    egui::DragValue::new(&mut self.angle)
                        .range(0.1..=90.0)
                        .speed(1.0),
                )
                .changed();
        });
        ui.horizontal(|ui| {
            for angle in [15.0, 30.0, 45.0, 90.0] {
                if ui
                    .selectable_label((self.angle - angle).abs() < 1e-9, format!("{angle}°"))
                    .clicked()
                {
                    self.angle = angle;
                    changed = true;
                }
            }
        });
        if !self.angle.is_finite() || self.angle <= 0.0 {
            self.angle = 15.0;
            changed = true;
        }
        ui.small("正交与极轴互斥；方向沿当前工作平面。");
        if changed {
            self.clear();
        }
    }
    pub fn settings_ui(&mut self, ui: &mut egui::Ui) {
        ui.set_max_width(320.0);
        ui.menu_button("对象捕捉与投影", |ui| self.snap_settings(ui));
        ui.menu_button("栅格显示与捕捉", |ui| self.grid_settings(ui));
        ui.menu_button("正交与极轴", |ui| self.direction_settings(ui));
        ui.separator();
        ui.add(
            egui::Label::new(
                egui::RichText::new(
                    "几何捕捉优先，其次正交/极轴，最后栅格；显式坐标不受辅助约束。",
                )
                .small(),
            )
            .wrap(),
        );
    }
    pub fn snap_button(&mut self, ui: &mut egui::Ui, compact: bool) -> bool {
        if ui
            .toggle_value(
                &mut self.enabled,
                if compact { "捕捉" } else { "对象捕捉 F3" },
            )
            .on_hover_text("对象捕捉 F3；右侧下拉选择端点、中点、圆心等类型")
            .changed()
        {
            self.clear();
        }
        ui.push_id("snap_status_settings", |ui| {
            ui.menu_button("▾", |ui| self.snap_settings(ui))
        })
        .inner
        .inner
        .is_some()
    }
    pub fn status_buttons(&mut self, ui: &mut egui::Ui, compact: bool) -> bool {
        let mut open = self.snap_button(ui, compact);
        let before = (self.grid, self.grid_snap, self.ortho, self.polar);
        ui.toggle_value(&mut self.grid, if compact { "栅格" } else { "栅格 F7" })
            .on_hover_text("显示栅格 F7");
        open |= ui
            .push_id("grid_status_settings", |ui| {
                ui.menu_button("▾", |ui| self.grid_settings(ui))
            })
            .inner
            .inner
            .is_some();
        ui.toggle_value(
            &mut self.grid_snap,
            if compact { "栅捕" } else { "栅格捕捉 F9" },
        )
        .on_hover_text("栅格捕捉 F9，与显示栅格独立开关");
        if ui
            .toggle_value(&mut self.ortho, if compact { "正交" } else { "正交 F8" })
            .on_hover_text("正交 F8")
            .changed()
            && self.ortho
        {
            self.polar = false;
        }
        if ui
            .toggle_value(&mut self.polar, if compact { "极轴" } else { "极轴 F10" })
            .on_hover_text("极轴 F10，与正交互斥")
            .changed()
            && self.polar
        {
            self.ortho = false;
        }
        open |= ui
            .push_id("direction_status_settings", |ui| {
                ui.menu_button("▾", |ui| self.direction_settings(ui))
            })
            .inner
            .inner
            .is_some();
        if before != (self.grid, self.grid_snap, self.ortho, self.polar) {
            self.clear();
        }
        open
    }
    pub fn draw(
        &self,
        gizmos: &mut Gizmos<super::MinorGridGizmos>,
        major_grid: &mut Gizmos<super::MajorGridGizmos>,
        frame: ViewFrame,
        plane: WorkPlane,
        camera: &Camera,
        transform: &GlobalTransform,
    ) {
        if !self.grid {
            return;
        }
        let Some(viewport) = camera.logical_viewport_rect() else {
            return;
        };
        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        for cursor in [
            viewport.min,
            Vec2::new(viewport.max.x, viewport.min.y),
            viewport.max,
            Vec2::new(viewport.min.x, viewport.max.y),
        ] {
            let Some((origin, direction)) = frame.ray(camera, transform, cursor) else {
                return;
            };
            let normal = plane.normal();
            let denominator = normal.dot(direction);
            if denominator.abs() < 1e-4 {
                return;
            }
            // Extend the plane beyond the camera near plane; rendering clips it.
            let distance = Vector3::new(
                plane.origin.x - origin.x,
                plane.origin.y - origin.y,
                plane.origin.z - origin.z,
            )
            .dot(normal)
                / denominator;
            let point = Point3::new(
                origin.x + direction.x * distance,
                origin.y + direction.y * distance,
                origin.z + direction.z * distance,
            );
            let local = plane.to_local(point);
            for axis in 0..2 {
                if !local[axis].is_finite() {
                    return;
                }
                min[axis] = min[axis].min(local[axis]);
                max[axis] = max[axis].max(local[axis]);
            }
        }
        let span = [max[0] - min[0], max[1] - min[1]];
        let density = (span[0] / viewport.width().max(1.0) as f64)
            .max(span[1] / viewport.height().max(1.0) as f64);
        let target = (density * 16.0).max(span[0] / 512.0).max(span[1] / 512.0);
        // Powers of five keep major lines aligned through density changes.
        let level = (target / self.spacing).log(5.0).ceil().max(0.0);
        let step = self.spacing * 5.0_f64.powf(level);
        if !step.is_finite() || step <= 0.0 {
            return;
        }
        for axis in 0..2 {
            let first = (min[axis] / step).floor() - 1.0;
            let count = (span[axis] / step).ceil() as usize + 4;
            for i in 0..count {
                let index = first + i as f64;
                let mut a = [min[0] - step, min[1] - step, 0.0];
                let mut b = [max[0] + step, max[1] + step, 0.0];
                a[axis] = index * step;
                b[axis] = a[axis];
                let a = frame.point(array(plane.to_world(a)));
                let b = frame.point(array(plane.to_world(b)));
                if a.is_finite() && b.is_finite() {
                    if index.rem_euclid(5.0) < 0.5 {
                        let color = if index.abs() < 0.5 {
                            Color::srgb(0.28, 0.37, 0.44)
                        } else {
                            Color::srgb(0.21, 0.29, 0.35)
                        };
                        major_grid.line(a, b, color);
                    } else {
                        gizmos.line(a, b, Color::srgb(0.12, 0.18, 0.23));
                    }
                }
            }
        }
    }
    pub fn marker_ui(
        &self,
        context: &egui::Context,
        frame: ViewFrame,
        camera: &Camera,
        transform: &GlobalTransform,
        canvas: egui::Rect,
    ) {
        let Some(marker) = &self.marker else {
            return;
        };
        let Some(p) = frame.project(camera, transform, array(marker.point)) else {
            return;
        };
        let p = egui::pos2(p.x, p.y);
        let painter = context
            .layer_painter(egui::LayerId::new(
                egui::Order::Foreground,
                egui::Id::new("snap_marker"),
            ))
            .with_clip_rect(canvas);
        painter.rect_stroke(
            egui::Rect::from_center_size(p, egui::vec2(12.0, 12.0)),
            0.0,
            egui::Stroke::new(2.0, egui::Color32::LIGHT_GREEN),
            egui::StrokeKind::Outside,
        );
        painter.text(
            p + egui::vec2(12.0, -14.0),
            egui::Align2::LEFT_BOTTOM,
            format!(
                "{}{} {}/{}",
                if self.project { "投影·" } else { "" },
                marker.kind.label(),
                self.index + 1,
                self.candidates.len()
            ),
            egui::FontId::proportional(14.0),
            egui::Color32::LIGHT_GREEN,
        );
    }
}
