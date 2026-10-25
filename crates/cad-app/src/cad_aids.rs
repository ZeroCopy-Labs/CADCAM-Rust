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
    pub fn controls(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("捕捉与绘图辅助").show(ui, |ui| {
            ui.checkbox(&mut self.enabled, "几何捕捉 F3");
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.grid, "网格 F7");
                ui.checkbox(&mut self.grid_snap, "网格捕捉 F9");
            });
            ui.horizontal(|ui| {
                if ui.checkbox(&mut self.ortho, "正交 F8").changed() && self.ortho {
                    self.polar = false;
                }
                if ui.checkbox(&mut self.polar, "极轴 F10").changed() && self.polar {
                    self.ortho = false;
                }
            });
            ui.horizontal(|ui| {
                ui.label("网格 mm");
                ui.add(
                    egui::DragValue::new(&mut self.spacing)
                        .range(0.000001..=1e9)
                        .speed(0.1),
                );
            });
            ui.horizontal(|ui| {
                ui.label("极轴角度 °");
                ui.add(
                    egui::DragValue::new(&mut self.angle)
                        .range(0.1..=90.0)
                        .speed(1.0),
                );
            });
            ui.checkbox(&mut self.project, "将捕捉点投影到工作平面");
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
                }
            }
            ui.small("几何捕捉优先，其次正交/极轴，最后网格；数值输入不受辅助约束。");
            ui.small(
                "Tab 切换捕捉候选，Shift+Tab 反向。交点、垂足和切点按当前支持的曲线类型提供。",
            );
        });
        ui.small(format!(
            "捕捉 {} · 方向 {} · 网格捕捉 {}",
            if self.enabled { "开" } else { "关" },
            if self.ortho {
                "正交"
            } else if self.polar {
                "极轴"
            } else {
                "自由"
            },
            if self.grid_snap { "开" } else { "关" }
        ));
        if self.clipped {
            ui.small("交点仅搜索当前 24 个邻近图元，可放大视图缩小范围");
        }
        if !self.spacing.is_finite() || self.spacing <= 0.0 {
            self.spacing = 10.0;
        }
        if !self.angle.is_finite() || self.angle <= 0.0 {
            self.angle = 15.0;
        }
    }
    pub fn draw(&self, gizmos: &mut Gizmos, frame: ViewFrame, plane: WorkPlane) {
        if self.grid {
            let center = plane.to_local(Point3::new(
                frame.origin[0],
                frame.origin[1],
                frame.origin[2],
            ));
            let step = self.spacing * ((frame.pixel_size * 18.0 / self.spacing).ceil().max(1.0));
            let radius = (frame.pixel_size * 900.0 / step).ceil().clamp(1.0, 80.0) as i32;
            let u = (center[0] / step).round() * step;
            let v = (center[1] / step).round() * step;
            for i in -radius..=radius {
                for (a, b) in [
                    (
                        [u + i as f64 * step, v - radius as f64 * step, 0.0],
                        [u + i as f64 * step, v + radius as f64 * step, 0.0],
                    ),
                    (
                        [u - radius as f64 * step, v + i as f64 * step, 0.0],
                        [u + radius as f64 * step, v + i as f64 * step, 0.0],
                    ),
                ] {
                    let a = frame.point(array(plane.to_world(a)));
                    let b = frame.point(array(plane.to_world(b)));
                    if a.is_finite() && b.is_finite() {
                        gizmos.line(a, b, Color::srgb(0.13, 0.2, 0.25));
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
    ) {
        let Some(marker) = &self.marker else {
            return;
        };
        let Some(p) = frame.project(camera, transform, array(marker.point)) else {
            return;
        };
        let p = egui::pos2(p.x, p.y);
        let painter = context.layer_painter(egui::LayerId::new(
            egui::Order::Foreground,
            egui::Id::new("snap_marker"),
        ));
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
