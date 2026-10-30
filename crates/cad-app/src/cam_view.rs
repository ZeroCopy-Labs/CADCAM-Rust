//! Read-only machine-coordinate trajectory inspection, separate from CAD workpiece coordinates.

use crate::{
    cam::UnitId,
    cam_analysis::CamAnalysis,
    cam_sync::Inputs,
    cam_trajectory::{ArcPlane, MachineMoveKind, MachinePose, MachineTrajectory},
};
use bevy_egui::egui;
use bevy::prelude::*;
use std::time::Instant;

struct Segment {
    a: [f64; 3],
    b: [f64; 3],
    movement: usize,
    cutting: bool,
    fly: bool,
    angles: [f64; 2],
    order: Option<u32>,
}

pub struct TrajectoryView {
    pub open: bool,
    projection: usize,
    index: usize,
    only_unit: bool,
    show_cutting: bool,
    show_rapid: bool,
    zoom: f64,
    pan: egui::Vec2,
    version: Option<Inputs>,
    segments: Vec<Segment>,
    overlay: bool,
    played_only: bool,
    playing: bool,
    rate: f64,
    tick: Instant,
    fraction: f64,
}

impl Default for TrajectoryView {
    fn default() -> Self {
        Self {
            open: false,
            projection: 3,
            index: 0,
            only_unit: false,
            show_cutting: true,
            show_rapid: true,
            zoom: 1.0,
            pan: egui::Vec2::ZERO,
            version: None,
            segments: Vec::new(),
            overlay: true,
            played_only: false,
            playing: false,
            rate: 120.0,
            tick: Instant::now(),
            fraction: 0.0,
        }
    }
}

impl TrajectoryView {
    fn prepare(&mut self, trajectory: Option<&MachineTrajectory>, version: Option<Inputs>) {
        let Some(t) =
            trajectory.filter(|t| version.is_some() && t.error.is_none() && !t.moves.is_empty())
        else {
            self.version = None;
            self.segments.clear();
            self.playing = false;
            return;
        };
        if self.version != version {
            self.segments = plot_segments(t);
            self.version = version;
            self.index = self.index.min(t.moves.len() - 1);
            self.playing = false;
            self.fraction = 0.0;
            self.zoom = 1.0;
            self.pan = egui::Vec2::ZERO;
        }
    }

    /// Returns a request to rebuild the current trajectory.
    pub fn controls(
        &mut self,
        ui: &mut egui::Ui,
        trajectory: Option<&MachineTrajectory>,
        version: Option<Inputs>,
    ) -> bool {
        self.prepare(trajectory, version);
        let mut rebuild = false;
        ui.horizontal_wrapped(|ui| {
            ui.checkbox(&mut self.overlay, "显示刀路");
            ui.checkbox(&mut self.show_cutting, "切削");
            ui.checkbox(&mut self.show_rapid, "空移/关光连接");
            ui.checkbox(&mut self.only_unit, "仅当前单元");
            ui.checkbox(&mut self.played_only, "仅已播放");
            rebuild = ui.button("重算刀路").clicked();
        });
        let Some(t) = trajectory.filter(|_| self.version.is_some()) else {
            if let Some(error) = trajectory.and_then(|t| t.error.as_ref()) {
                ui.colored_label(egui::Color32::LIGHT_RED, error);
            } else {
                ui.small("刀路待重算；失效轨迹已隐藏。");
            }
            return rebuild;
        };
        let elapsed = self.tick.elapsed().as_secs_f64().min(0.25);
        self.tick = Instant::now();
        if self.playing {
            self.fraction += elapsed * self.rate;
            let count = self.fraction.floor() as usize;
            self.fraction -= count as f64;
            self.index = self.index.saturating_add(count).min(t.moves.len() - 1);
            if self.index == t.moves.len() - 1 {
                self.playing = false;
            }
            ui.ctx().request_repaint();
        }
        ui.horizontal_wrapped(|ui| {
            if ui
                .button(if self.playing { "暂停" } else { "播放" })
                .clicked()
            {
                if self.index == t.moves.len() - 1 {
                    self.index = 0;
                }
                self.playing = !self.playing;
                self.tick = Instant::now();
            }
            if ui.small_button("起点").clicked() {
                self.index = 0;
                self.fraction = 0.0;
            }
            if ui.small_button("上一段").clicked() {
                self.index = self.index.saturating_sub(1);
            }
            ui.add(egui::Slider::new(&mut self.index, 0..=t.moves.len() - 1).text("运动段"));
            if ui.small_button("下一段").clicked() {
                self.index = (self.index + 1).min(t.moves.len() - 1);
            }
            ui.label("演示段/秒");
            ui.add(
                egui::DragValue::new(&mut self.rate)
                    .range(1.0..=10000.0)
                    .speed(10.0),
            );
        });
        let m = t.moves[self.index];
        let order = t
            .unit_ranges
            .iter()
            .find(|r| self.index >= r.start && self.index < r.end)
            .map(|r| r.order);
        ui.small(format!("{} · 编号 {} · X {:.3} Y {:.3} Z {:.3} A {:.3}° · 飞扫 {} 孔 / {} 加工段 · 主画布为工件相对刀路；播放不代表实际加工时间",
        movement_label(m.kind), order.map_or("—".into(), |n| n.to_string()), m.pose.x,m.pose.y,m.pose.z,m.pose.a,t.fly_unit_count,t.fly_pass_count));
        rebuild
    }

    pub fn draw_overlay(
        &mut self,
        gizmos: &mut Gizmos<super::SelectedGizmos>,
        frame: super::cad_view::ViewFrame,
        trajectory: Option<&MachineTrajectory>,
        version: Option<Inputs>,
        center: Option<[f64; 2]>,
    ) {
        self.prepare(trajectory, version);
        let Some(t) = trajectory.filter(|_| self.overlay && self.version.is_some()) else {
            return;
        };
        let order = t
            .unit_ranges
            .iter()
            .find(|r| self.index >= r.start && self.index < r.end)
            .map(|r| r.order);
        for s in &self.segments {
            if (s.cutting && !self.show_cutting)
                || (!s.cutting && !self.show_rapid)
                || self.only_unit && s.order != order
                || self.played_only && s.movement > self.index
            {
                continue;
            }
            let color = if s.movement == self.index {
                Color::srgb(1.0, 0.9, 0.1)
            } else if s.cutting {
                Color::srgb(1.0, 0.48, 0.12)
            } else if s.fly {
                Color::srgb(0.75, 0.38, 1.0)
            } else {
                Color::srgb(0.15, 0.65, 1.0)
            };
            let a = frame.point(workpiece(s.a, s.angles[0], center));
            let b = frame.point(workpiece(s.b, s.angles[1], center));
            if a.is_finite() && b.is_finite() {
                gizmos.line(a, b, color);
            }
        }
        let pose = t.moves[self.index].pose;
        let p = frame.point(workpiece(xyz(pose), pose.a, center));
        if p.is_finite() {
            for direction in [Vec3::X, Vec3::Y, Vec3::Z] {
                let d = direction * (frame.pixel_size as f32 * 5.0);
                gizmos.line(p - d, p + d, Color::srgb(1.0, 0.15, 0.15));
            }
        }
    }
    pub fn focus_order(&mut self, order: u32, trajectory: Option<&MachineTrajectory>) {
        if let Some(range) =
            trajectory.and_then(|t| t.unit_ranges.iter().find(|r| r.order == order))
        {
            self.index = range.start;
            self.fraction = 0.0;
        }
    }

    pub fn ui(
        &mut self,
        context: &egui::Context,
        trajectory: Option<&MachineTrajectory>,
        version: Option<Inputs>,
        analysis: &CamAnalysis,
        allow_locate: bool,
    ) -> Option<UnitId> {
        if !self.open {
            return None;
        }
        self.prepare(trajectory, version);
        let mut open = self.open;
        let mut locate = None;
        egui::Window::new("机床轨迹查看").open(&mut open).default_size(egui::vec2(920.0, 680.0))
            .resizable(true).show(context, |ui| {
            ui.label("机床 XYZ 投影视图，A 值单独显示；不将旋转后的刀头坐标叠加到工件 CAD 坐标。");
            ui.small("蓝：空移 · 紫：飞扫关光连接 · 橙：切削 · 黄：当前段 · 红：当前点。滚轮缩放，拖动平移。");
            let Some(trajectory) = trajectory.filter(|_| version.is_some()) else {
                self.version = None;
                self.segments.clear();
                ui.label("当前轨迹尚未计算或存在待处理项，请先在 CAM / G代码配置中重算并处理提示。");
                return;
            };
            if let Some(error) = &trajectory.error { ui.colored_label(egui::Color32::LIGHT_RED, error); return; }
            if trajectory.moves.is_empty() { ui.label("没有运动段"); return; }
            ui.horizontal(|ui| {
                for (i, name) in ["XY", "XZ", "YZ", "等轴测"].into_iter().enumerate() {
                    if ui.selectable_value(&mut self.projection, i, name).changed() {
                        self.zoom = 1.0; self.pan = egui::Vec2::ZERO;
                    }
                }
                ui.checkbox(&mut self.show_cutting, "切削");
                ui.checkbox(&mut self.show_rapid, "空移");
                ui.checkbox(&mut self.only_unit, "仅当前单元（含进场）");
                if ui.button("适应轨迹").clicked() { self.zoom = 1.0; self.pan = egui::Vec2::ZERO; }
            });
            ui.horizontal(|ui| {
                if ui.button("上一段").clicked() { self.index = self.index.saturating_sub(1); }
                ui.add(egui::Slider::new(&mut self.index, 0..=trajectory.moves.len()-1).text("运动点索引"));
                if ui.button("下一段").clicked() { self.index = (self.index+1).min(trajectory.moves.len()-1); }
            });
            let movement = trajectory.moves[self.index];
            let pose = movement.pose;
            let range = trajectory.unit_ranges.iter().find(|r| self.index >= r.start && self.index < r.end);
            let unit = range.and_then(|r| analysis.plan.groups.iter().flat_map(|g| &g.units).find(|u| u.order == r.order));
            ui.label(format!("点 {} / {} · {} · X {:.4}  Y {:.4}  Z {:.4}  A {:.4}°",
                self.index, trajectory.moves.len()-1, movement_label(movement.kind),
                pose.x, pose.y, pose.z, pose.a));
            ui.horizontal(|ui| {
                ui.label(unit.map(|u| format!("加工编号 {} · {} · 单元 #{:016X}", u.order, u.zone.label(), u.unit_id.0))
                    .unwrap_or_else(|| "首尾进退场 / 未关联单元".into()));
                if ui.add_enabled(unit.is_some() && allow_locate, egui::Button::new("定位对应 CAD 图元")).clicked() {
                    locate = unit.map(|u| u.unit_id);
                }
            });
            ui.small("首点仅表示首个已知姿态，不绘制未知机床位置到首点的移动；圆弧仅为显示离散，A 轴与自定义头尾指令未作机床仿真。");
            let visible = |s: &&Segment| (if s.cutting { self.show_cutting } else { self.show_rapid })
                && (!self.only_unit || range.is_some_and(|r| s.order == Some(r.order)))
                && (!self.played_only || s.movement <= self.index);
            let point = project([pose.x, pose.y, pose.z], self.projection);
            let mut min = point;
            let mut max = point;
            for s in self.segments.iter().filter(visible) {
                for p in [s.a, s.b] {
                    let p = project(p, self.projection);
                    for i in 0..2 { min[i] = min[i].min(p[i]); max[i] = max[i].max(p[i]); }
                }
            }
            let size = ui.available_size().max(egui::vec2(240.0, 180.0));
            let (response, painter) = ui.allocate_painter(size, egui::Sense::drag());
            let rect = response.rect.shrink(14.0);
            painter.rect_filled(response.rect, 4.0, egui::Color32::from_rgb(14, 20, 29));
            if !min[0].is_finite() { return; }
            if response.dragged() { self.pan += ui.input(|i| i.pointer.delta()); }
            if response.hovered() {
                self.zoom = (self.zoom * ui.input(|i| (i.smooth_scroll_delta.y as f64 * 0.002).exp())).clamp(0.01, 1000.0);
            }
            let center = [(min[0]+max[0])*0.5, (min[1]+max[1])*0.5];
            let scale = ((rect.width() as f64 / (max[0]-min[0]).max(1.0))
                .min(rect.height() as f64 / (max[1]-min[1]).max(1.0))) * self.zoom;
            let screen = |p| {
                let p = project(p, self.projection);
                rect.center() + self.pan + egui::vec2(((p[0]-center[0])*scale) as f32, (-(p[1]-center[1])*scale) as f32)
            };
            let painter = painter.with_clip_rect(response.rect);
            for s in self.segments.iter().filter(visible) {
                let selected = s.movement == self.index;
                let color = if selected { egui::Color32::YELLOW } else if s.cutting {
                    egui::Color32::from_rgb(255, 155, 40)
                } else if s.fly { egui::Color32::from_rgb(190, 95, 255) } else { egui::Color32::from_rgb(50, 160, 245) };
                painter.line_segment([screen(s.a), screen(s.b)], egui::Stroke::new(if selected {3.0} else {1.2}, color));
            }
            painter.circle_filled(screen([pose.x, pose.y, pose.z]), 4.0, egui::Color32::LIGHT_RED);
        });
        self.open = open;
        locate
    }
}

fn project(p: [f64; 3], plane: usize) -> [f64; 2] {
    match plane {
        0 => [p[0], p[1]],
        1 => [p[0], p[2]],
        2 => [p[1], p[2]],
        _ => [
            (p[0] - p[1]) * std::f64::consts::FRAC_1_SQRT_2,
            p[2] - (p[0] + p[1]) * 0.5,
        ],
    }
}

fn xyz(p: MachinePose) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn movement_label(kind: MachineMoveKind) -> &'static str {
    match kind {
        MachineMoveKind::Rapid => "安全空移",
        MachineMoveKind::FlyTransfer => "飞扫关光连接",
        _ => "切削",
    }
}

fn workpiece(p: [f64; 3], angle: f64, center: Option<[f64; 2]>) -> [f64; 3] {
    let Some([cy, cz]) = center else {
        return p;
    };
    let (sin, cos) = angle.to_radians().sin_cos();
    [
        p[0],
        cy + (p[1] - cy) * cos + (p[2] - cz) * sin,
        cz - (p[1] - cy) * sin + (p[2] - cz) * cos,
    ]
}

fn plot_segments(trajectory: &MachineTrajectory) -> Vec<Segment> {
    let mut result = Vec::new();
    for (i, pair) in trajectory.moves.windows(2).enumerate() {
        let start = xyz(pair[0].pose);
        let end = xyz(pair[1].pose);
        let kind = pair[1].kind;
        let arc = if let MachineMoveKind::Arc {
            plane,
            center,
            clockwise,
        } = kind
        {
            let (u, v, w) = match plane {
                ArcPlane::XY => (0, 1, 2),
                ArcPlane::XZ => (0, 2, 1),
                ArcPlane::YZ => (1, 2, 0),
            };
            let radius = (start[u] - center[u]).hypot(start[v] - center[v]);
            let a = (start[v] - center[v]).atan2(start[u] - center[u]);
            let b = (end[v] - center[v]).atan2(end[u] - center[u]);
            let mut delta = if clockwise {
                -(a - b).rem_euclid(std::f64::consts::TAU)
            } else {
                (b - a).rem_euclid(std::f64::consts::TAU)
            };
            if delta.abs() < 1e-10 {
                delta = if clockwise {
                    -std::f64::consts::TAU
                } else {
                    std::f64::consts::TAU
                };
            }
            Some((u, v, w, center, radius, a, delta))
        } else {
            None
        };
        let angle_steps = ((pair[1].pose.a - pair[0].pose.a).abs() / 2.0).ceil();
        let arc_steps = arc.map_or(1.0, |a| (a.6.abs() / 2.0_f64.to_radians()).ceil());
        let steps = angle_steps.max(arc_steps).clamp(1.0, 4096.0) as usize;
        let order = trajectory
            .unit_ranges
            .get(trajectory.unit_ranges.partition_point(|r| r.end <= i + 1))
            .map(|r| r.order);
        let mut previous = start;
        let mut previous_angle = pair[0].pose.a;
        for j in 1..=steps {
            let t = j as f64 / steps as f64;
            let mut p = std::array::from_fn(|k| start[k] + (end[k] - start[k]) * t);
            if let Some((u, v, w, center, radius, a, delta)) = arc {
                p[u] = center[u] + radius * (a + delta * t).cos();
                p[v] = center[v] + radius * (a + delta * t).sin();
                p[w] = start[w] + (end[w] - start[w]) * t;
            }
            if j == steps {
                p = end;
            }
            let angle = pair[0].pose.a + (pair[1].pose.a - pair[0].pose.a) * t;
            result.push(Segment {
                a: previous,
                b: p,
                movement: i + 1,
                cutting: kind.is_cutting(),
                fly: kind == MachineMoveKind::FlyTransfer,
                angles: [previous_angle, angle],
                order,
            });
            previous = p;
            previous_angle = angle;
        }
    }
    result
}
