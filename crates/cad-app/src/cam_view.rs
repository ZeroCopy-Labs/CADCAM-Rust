//! Read-only machine-coordinate trajectory inspection, separate from CAD workpiece coordinates.

use crate::{
    cam::UnitId,
    cam_analysis::CamAnalysis,
    cam_sync::Inputs,
    cam_trajectory::{ArcPlane, MachineMoveKind, MachinePose, MachineTrajectory},
};
use bevy_egui::egui;

struct Segment {
    a: [f64; 3],
    b: [f64; 3],
    movement: usize,
    cutting: bool,
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
        }
    }
}

impl TrajectoryView {
    pub fn focus_order(&mut self, order: u32, trajectory: Option<&MachineTrajectory>) {
        if let Some(range) =
            trajectory.and_then(|t| t.unit_ranges.iter().find(|r| r.order == order))
        {
            self.index = range.start;
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
        let mut open = self.open;
        let mut locate = None;
        egui::Window::new("机床轨迹查看").open(&mut open).default_size(egui::vec2(920.0, 680.0))
            .resizable(true).show(context, |ui| {
            ui.label("机床 XYZ 投影视图，A 值单独显示；不将旋转后的刀头坐标叠加到工件 CAD 坐标。");
            ui.small("蓝：空移 · 橙：切削 · 黄：当前运动段 · 红：当前段终点。滚轮缩放，拖动平移。");
            let Some(trajectory) = trajectory.filter(|_| version.is_some()) else {
                self.version = None;
                self.segments.clear();
                ui.label("当前轨迹尚未计算或存在待处理项，请先在 CAM / G代码配置中重算并处理提示。");
                return;
            };
            if let Some(error) = &trajectory.error { ui.colored_label(egui::Color32::LIGHT_RED, error); return; }
            if trajectory.moves.is_empty() { ui.label("没有运动段"); return; }
            if self.version != version {
                self.segments = plot_segments(trajectory);
                self.version = version;
                self.index = self.index.min(trajectory.moves.len()-1);
                self.zoom = 1.0;
                self.pan = egui::Vec2::ZERO;
            }
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
                self.index, trajectory.moves.len()-1, if movement.kind.is_cutting() {"切削"} else {"空移"},
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
                && (!self.only_unit || range.is_some_and(|r| s.movement >= r.start && s.movement < r.end));
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
            if response.dragged() { self.pan += response.drag_delta(); }
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
                } else { egui::Color32::from_rgb(50, 160, 245) };
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

fn plot_segments(trajectory: &MachineTrajectory) -> Vec<Segment> {
    let mut result = Vec::new();
    for (i, pair) in trajectory.moves.windows(2).enumerate() {
        let start = xyz(pair[0].pose);
        let end = xyz(pair[1].pose);
        let kind = pair[1].kind;
        let mut previous = start;
        if let MachineMoveKind::Arc {
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
            let delta = if clockwise {
                -((a - b).rem_euclid(std::f64::consts::TAU))
            } else {
                (b - a).rem_euclid(std::f64::consts::TAU)
            };
            let delta = if delta.abs() < 1e-10 {
                if clockwise {
                    -std::f64::consts::TAU
                } else {
                    std::f64::consts::TAU
                }
            } else {
                delta
            };
            let steps = (delta.abs() / 2.0_f64.to_radians()).ceil() as usize;
            for j in 1..steps {
                let t = j as f64 / steps as f64;
                let mut p = start;
                p[u] = center[u] + radius * (a + delta * t).cos();
                p[v] = center[v] + radius * (a + delta * t).sin();
                p[w] = start[w] + (end[w] - start[w]) * t;
                result.push(Segment {
                    a: previous,
                    b: p,
                    movement: i + 1,
                    cutting: true,
                });
                previous = p;
            }
        }
        result.push(Segment {
            a: previous,
            b: end,
            movement: i + 1,
            cutting: kind.is_cutting(),
        });
    }
    result
}
