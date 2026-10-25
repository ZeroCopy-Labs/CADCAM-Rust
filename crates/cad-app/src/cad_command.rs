//! Command steps and transient previews; only confirmation submits a CAD edit.

use std::collections::BTreeSet;

use bevy::prelude::*;
use bevy_egui::egui;
use cad_core::model::{Point3, WorkPlane};

enum Step {
    Select,
    Base {
        ids: BTreeSet<u32>,
    },
    Target {
        ids: BTreeSet<u32>,
        base: [f64; 3],
    },
    Confirm {
        ids: BTreeSet<u32>,
        base: [f64; 3],
        target: [f64; 3],
    },
    PlaneOrigin,
    PlaneAxis {
        origin: [f64; 3],
    },
    PlaneThird {
        origin: [f64; 3],
        x: [f64; 3],
    },
}

pub struct MoveCommit {
    pub ids: BTreeSet<u32>,
    pub offset: [f64; 3],
}

#[derive(Default)]
pub struct CommandState {
    pub aids: super::cad_aids::DraftingAids,
    pub plane: WorkPlane,
    step: Option<Step>,
    hover: Option<[f64; 3]>,
    pub error: String,
    ray_error: String,
    origin_input: [f64; 3],
}

impl CommandState {
    pub fn error_message(&self) -> &str {
        if self.ray_error.is_empty() {
            &self.error
        } else {
            &self.ray_error
        }
    }
    pub fn is_active(&self) -> bool {
        self.step.is_some()
    }
    pub fn selecting(&self) -> bool {
        matches!(self.step, Some(Step::Select))
    }
    pub fn takes_point(&self) -> bool {
        self.is_active() && !self.selecting() && !matches!(self.step, Some(Step::Confirm { .. }))
    }

    pub fn prompt(&self) -> &'static str {
        match &self.step {
            None => "就绪：MOVE / M 移动；UCS XY / YZ / ZX / 3P 设置工作平面",
            Some(Step::Select) => "移动 1/4：选择图元，Enter 确认选择；Esc 取消",
            Some(Step::Base { .. }) => "移动 2/4：指定基点（画布点选或输入坐标）；Esc 取消",
            Some(Step::Target { .. }) => "移动 3/4：指定目标点；仅预览，不改图纸；Esc 取消",
            Some(Step::Confirm { .. }) => {
                "移动 4/4：Enter / 确认移动提交；可重新输入目标坐标；Esc 取消"
            }
            Some(Step::PlaneOrigin) => "三点平面 1/3：指定原点；Esc 取消",
            Some(Step::PlaneAxis { .. }) => "三点平面 2/3：指定 +U 方向上的点；Esc 取消",
            Some(Step::PlaneThird { .. }) => "三点平面 3/3：指定 +V 一侧的点；三点不能共线",
        }
    }

    pub fn begin_move(&mut self, ids: &BTreeSet<u32>) {
        self.ray_error.clear();
        self.step = Some(if ids.is_empty() {
            Step::Select
        } else {
            Step::Base { ids: ids.clone() }
        });
        self.hover = None;
        self.error.clear();
    }

    pub fn cancel(&mut self) {
        self.aids.clear();
        self.ray_error.clear();
        self.step = None;
        self.hover = None;
        self.error.clear();
    }

    pub fn set_preset(&mut self, name: &str) -> Result<(), String> {
        if self.is_active() {
            return Err("请先完成或取消当前命令".into());
        }
        self.error.clear();
        self.ray_error.clear();
        self.hover = None;
        self.plane = match name {
            "XY" => WorkPlane::xy(self.plane.origin),
            "YZ" => WorkPlane::yz(self.plane.origin),
            "ZX" => WorkPlane::zx(self.plane.origin),
            "3P" => {
                self.step = Some(Step::PlaneOrigin);
                return Ok(());
            }
            _ => return Err("工作平面选项：UCS XY / YZ / ZX / 3P".into()),
        };
        self.error.clear();
        Ok(())
    }

    fn reference(&self) -> [f64; 3] {
        match &self.step {
            Some(Step::Target { base, .. } | Step::Confirm { base, .. }) => *base,
            Some(Step::PlaneAxis { origin } | Step::PlaneThird { origin, .. }) => *origin,
            _ => array(self.plane.origin),
        }
    }

    fn snap_reference(&self) -> Option<Point3> {
        match self.step {
            Some(
                Step::Target { .. }
                | Step::Confirm { .. }
                | Step::PlaneAxis { .. }
                | Step::PlaneThird { .. },
            ) => Some(as_point(self.reference())),
            _ => None,
        }
    }
    pub fn cycle_snap(&mut self, reverse: bool) {
        if self.takes_point() {
            if let Some(point) = self.aids.cycle(reverse) {
                self.hover = Some(array(point));
            }
        }
    }
    pub fn input(
        &mut self,
        text: &str,
        selected: &BTreeSet<u32>,
    ) -> Result<Option<MoveCommit>, String> {
        if matches!(self.step, Some(Step::Select)) {
            if !text.trim().is_empty() {
                return Err("请先在画布选择图元，再按 Enter 确认".into());
            }
            if selected.is_empty() {
                return Err("尚未选择图元".into());
            }
            self.step = Some(Step::Base {
                ids: selected.clone(),
            });
            self.error.clear();
            return Ok(None);
        }
        if text.trim().is_empty() {
            if let Some(Step::Confirm { ids, base, target }) = &self.step {
                return Ok(Some(MoveCommit {
                    ids: ids.clone(),
                    offset: std::array::from_fn(|i| target[i] - base[i]),
                }));
            }
            let point = self.hover.ok_or("请点选有效位置，或输入精确坐标")?;
            self.accept_point(point)?;
        } else {
            let point = parse_point(text, self.plane, self.reference())?;
            self.accept_point(point)?;
        }
        Ok(None)
    }

    fn accept_point(&mut self, point: [f64; 3]) -> Result<(), String> {
        self.step = match self.step.as_ref().ok_or("没有正在取点的命令")? {
            Step::Base { ids } => Some(Step::Target {
                ids: ids.clone(),
                base: point,
            }),
            Step::Target { ids, base } | Step::Confirm { ids, base, .. } => {
                if !point.iter().zip(base).all(|(p, b)| (p - b).is_finite()) {
                    return Err("移动量超出有效数值范围".into());
                }
                Some(Step::Confirm {
                    ids: ids.clone(),
                    base: *base,
                    target: point,
                })
            }
            Step::PlaneOrigin => Some(Step::PlaneAxis { origin: point }),
            Step::PlaneAxis { origin } => {
                let length = point
                    .iter()
                    .zip(origin)
                    .map(|(p, o)| (p - o).powi(2))
                    .sum::<f64>()
                    .sqrt();
                if !length.is_finite() || length < 1e-9 {
                    return Err("+U 方向点不能与原点重合".into());
                }
                Some(Step::PlaneThird {
                    origin: *origin,
                    x: point,
                })
            }
            Step::PlaneThird { origin, x } => {
                self.plane =
                    WorkPlane::from_points(as_point(*origin), as_point(*x), as_point(point))?;
                self.origin_input = array(self.plane.origin);
                None
            }
            Step::Select => return Err("请先确认选择".into()),
        };
        self.hover = None;
        self.aids.clear();
        self.error.clear();
        self.ray_error.clear();
        Ok(())
    }

    pub fn viewport_point(
        &mut self,
        camera: &Camera,
        transform: &GlobalTransform,
        cursor: Vec2,
        frame: super::cad_view::ViewFrame,
        document: &cad_core::DrawingDocument,
        click: bool,
    ) -> bool {
        if !self.takes_point() {
            return false;
        }
        let reference = self.snap_reference();
        let point = frame
            .ray(camera, transform, cursor)
            .ok_or_else(|| "当前相机无法取点".to_string())
            .and_then(|ray| {
                self.aids.resolve(
                    document, frame, camera, transform, cursor, ray, self.plane, reference,
                )
            })
            .map(array)
            .and_then(finite);
        match point {
            Ok(point) => {
                self.hover = Some(point);
                self.ray_error.clear();
                if click {
                    match self.accept_point(point) {
                        Ok(()) => return true,
                        Err(error) => self.error = error,
                    }
                }
            }
            Err(error) => {
                self.hover = None;
                self.ray_error = error;
            }
        }
        false
    }

    pub fn clear_hover(&mut self) {
        self.aids.clear();
        self.hover = None;
    }

    pub fn preview(&self) -> Option<MoveCommit> {
        let (ids, base, target) = match &self.step {
            Some(Step::Target { ids, base }) => (ids, base, self.hover?),
            Some(Step::Confirm { ids, base, target }) => (ids, base, *target),
            _ => return None,
        };
        Some(MoveCommit {
            ids: ids.clone(),
            offset: std::array::from_fn(|i| target[i] - base[i]),
        })
    }

    /// Return a command-bar submission, keeping all buttons on the same execution path.
    pub fn controls_ui(&mut self, ui: &mut egui::Ui) -> Option<String> {
        let mut command = None;
        ui.strong("精确移动与工作平面");
        self.aids.controls(ui);
        ui.label(self.prompt());
        if self.is_active() {
            if !self.error_message().is_empty() {
                ui.colored_label(egui::Color32::LIGHT_RED, self.error_message());
            }
            if let Some(point) = self.hover {
                ui.label(format!(
                    "WCS：{:.4}, {:.4}, {:.4} mm",
                    point[0], point[1], point[2]
                ));
                let local = self.plane.to_local(as_point(point));
                ui.small(format!(
                    "平面 U/V/N：{:.4}, {:.4}, {:.4} mm",
                    local[0], local[1], local[2]
                ));
            }
            if let Some(preview) = self.preview() {
                let [x, y, z] = preview.offset;
                ui.label(format!("预览 Δ：{x:.4}, {y:.4}, {z:.4} mm"));
            }
            ui.horizontal(|ui| {
                if matches!(self.step, Some(Step::Select)) && ui.button("确认选择").clicked() {
                    command = Some(String::new());
                }
                if matches!(self.step, Some(Step::Confirm { .. }))
                    && ui.button("确认移动").clicked()
                {
                    command = Some(String::new());
                }
                if ui.button("取消  Esc").clicked() {
                    command = Some("CANCEL".into());
                }
            });
            ui.small("x,y,z：WCS；u,v：平面；@：相对；@长度<角度：平面极坐标");
        } else {
            if ui.button("基点 → 目标点移动  Ctrl+M").clicked() {
                command = Some("MOVE".into());
            }
            ui.horizontal(|ui| {
                for name in ["XY", "YZ", "ZX", "3P"] {
                    if ui
                        .button(if name == "3P" { "三点平面" } else { name })
                        .clicked()
                    {
                        command = Some(format!("UCS {name}"));
                    }
                }
            });
            ui.label("工作平面原点 WCS（mm）");
            egui::Grid::new("work_plane_origin")
                .num_columns(2)
                .show(ui, |ui| {
                    for (name, value) in ["X", "Y", "Z"].into_iter().zip(&mut self.origin_input) {
                        ui.label(name);
                        ui.add(egui::DragValue::new(value).speed(0.1));
                        ui.end_row();
                    }
                });
            if ui.button("设置工作平面原点").clicked() {
                if self.origin_input.iter().all(|value| value.is_finite()) {
                    self.plane.origin = as_point(self.origin_input);
                    self.error.clear();
                } else {
                    self.error = "工作平面原点必须为有限坐标".into();
                }
            }
            if !self.error.is_empty() {
                ui.colored_label(egui::Color32::LIGHT_RED, &self.error);
            }
        }
        let origin = self.plane.origin;
        ui.small(format!(
            "当前原点：{:.3}, {:.3}, {:.3} mm",
            origin.x, origin.y, origin.z
        ));
        let u = self.plane.x_axis;
        let v = self.plane.y_axis;
        ui.small("平面轴：U 红 · V 蓝 · N 绿；边框为当前取点平面");
        ui.small(format!(
            "U：{:.3}, {:.3}, {:.3}；V：{:.3}, {:.3}, {:.3}",
            u.x, u.y, u.z, v.x, v.y, v.z
        ));
        ui.separator();
        command
    }

    pub fn draw_plane(&self, gizmos: &mut Gizmos, extent: f64, frame: super::cad_view::ViewFrame) {
        self.aids.draw(gizmos, frame, self.plane);
        let length = (extent * 0.2).clamp(5.0, 1000.0);
        let origin = frame.point(array(self.plane.origin));
        if !origin.is_finite() {
            return;
        }
        for (local, color) in [
            ([length, 0.0, 0.0], Color::srgb(1.0, 0.4, 0.4)),
            ([0.0, length, 0.0], Color::srgb(0.3, 0.8, 1.0)),
            ([0.0, 0.0, length * 0.5], Color::srgb(0.4, 1.0, 0.5)),
        ] {
            gizmos.line(
                origin,
                frame.point(array(self.plane.to_world(local))),
                color,
            );
        }
        let corners = [
            [-length, -length, 0.0],
            [length, -length, 0.0],
            [length, length, 0.0],
            [-length, length, 0.0],
            [-length, -length, 0.0],
        ];
        for pair in corners.windows(2) {
            gizmos.line(
                frame.point(array(self.plane.to_world(pair[0]))),
                frame.point(array(self.plane.to_world(pair[1]))),
                Color::srgb(0.2, 0.35, 0.45),
            );
        }
        if let Some(point) = self.hover {
            let p = frame.point(point);
            if p.is_finite() {
                for direction in [Vec3::X, Vec3::Y, Vec3::Z] {
                    gizmos.line(
                        p - direction * (length * 0.025) as f32,
                        p + direction * (length * 0.025) as f32,
                        Color::srgb(0.2, 1.0, 1.0),
                    );
                }
            }
        }
        let anchored = match &self.step {
            Some(Step::Target { base, .. }) => self.hover.map(|target| (*base, target)),
            Some(Step::Confirm { base, target, .. }) => Some((*base, *target)),
            Some(Step::PlaneAxis { origin }) => self.hover.map(|target| (*origin, target)),
            Some(Step::PlaneThird { origin, x }) => {
                let start = frame.point(*origin);
                let end = frame.point(*x);
                if start.is_finite() && end.is_finite() {
                    gizmos.line(start, end, Color::srgb(1.0, 0.6, 0.2));
                }
                self.hover.map(|target| (*origin, target))
            }
            _ => None,
        };
        if let Some((base, target)) = anchored {
            let base = frame.point(base);
            let target = frame.point(target);
            if base.is_finite() && target.is_finite() {
                gizmos.line(base, target, Color::srgb(1.0, 0.6, 0.2));
                for point in [base, target] {
                    for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
                        gizmos.line(
                            point - axis * (length * 0.025) as f32,
                            point + axis * (length * 0.025) as f32,
                            Color::srgb(0.2, 1.0, 1.0),
                        );
                    }
                }
            }
        }
    }
}

fn as_point(p: [f64; 3]) -> Point3 {
    Point3::new(p[0], p[1], p[2])
}
fn array(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn parse_point(text: &str, plane: WorkPlane, reference: [f64; 3]) -> Result<[f64; 3], String> {
    let text = text.trim();
    let relative = text.starts_with('@');
    let text = text.strip_prefix('@').unwrap_or(text);
    if let Some((distance, angle)) = text.split_once('<') {
        if !relative {
            return Err("极坐标请使用 @长度<角度（角度为度）".into());
        }
        let distance = number(distance)?;
        let angle = number(angle)?.to_radians();
        let offset =
            plane.x_axis * (distance * angle.cos()) + plane.y_axis * (distance * angle.sin());
        return finite([
            reference[0] + offset.x,
            reference[1] + offset.y,
            reference[2] + offset.z,
        ]);
    }
    let world = text.starts_with("W:") || text.starts_with("w:");
    let text = if world { &text[2..] } else { text };
    let normalized = text.replace('，', ",");
    let values: Vec<f64> = if normalized.contains(',') {
        normalized
            .split(',')
            .map(number)
            .collect::<Result<_, _>>()?
    } else {
        normalized
            .split_whitespace()
            .map(number)
            .collect::<Result<_, _>>()?
    };
    let result = match values.as_slice() {
        [x, y, z] => {
            if relative {
                [reference[0] + x, reference[1] + y, reference[2] + z]
            } else {
                [*x, *y, *z]
            }
        }
        [u, v] if !world => {
            if relative {
                let offset = plane.x_axis * *u + plane.y_axis * *v;
                [
                    reference[0] + offset.x,
                    reference[1] + offset.y,
                    reference[2] + offset.z,
                ]
            } else {
                array(plane.to_world([*u, *v, 0.0]))
            }
        }
        _ => return Err("请输入 x,y,z（WCS）或 u,v（工作平面）；相对输入加 @".into()),
    };
    finite(result)
}

fn number(text: &str) -> Result<f64, String> {
    text.trim()
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| "坐标和角度必须为有限数值".into())
}
fn finite(point: [f64; 3]) -> Result<[f64; 3], String> {
    if point.iter().all(|value| value.is_finite()) {
        Ok(point)
    } else {
        Err("计算结果超出有效坐标范围".into())
    }
}
