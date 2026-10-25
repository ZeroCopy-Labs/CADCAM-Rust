//! Command steps and transient previews; only confirmation submits a CAD edit.

use std::collections::{BTreeMap, BTreeSet};

use super::cad_tools::{ToolOptions, TransformKind};
use bevy::prelude::*;
use bevy_egui::egui;
use cad_core::geometry_edit::{AffineTransform, Sketch, SketchKind};
use cad_core::model::{EntityId, Geometry, Plane, Point3, WorkPlane};
use cad_core::properties::{self, GripKind};

enum Step {
    Grip {
        id: EntityId,
        geometry: Geometry,
        kind: GripKind,
        base: Point3,
        target: Option<Point3>,
        dragging: bool,
    },
    Draw {
        sketch: Sketch,
        confirmed: bool,
    },
    TransformSelect {
        kind: TransformKind,
    },
    Transform {
        kind: TransformKind,
        ids: BTreeSet<u32>,
        base: Option<[f64; 3]>,
    },
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

pub enum CommandCommit {
    Geometry(BTreeMap<EntityId, Geometry>),
    Move(MoveCommit),
    Create {
        geometry: Vec<Geometry>,
        normal: cad_core::model::Vector3,
    },
    Transform {
        ids: BTreeSet<u32>,
        transforms: Vec<AffineTransform>,
        copies: bool,
    },
}
#[derive(Default)]
pub struct CommandState {
    pub tools: ToolOptions,
    copying: bool,
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
        matches!(self.step, Some(Step::Select | Step::TransformSelect { .. }))
    }
    pub fn takes_point(&self) -> bool {
        self.is_active()
            && !self.selecting()
            && !matches!(
                self.step,
                Some(
                    Step::Grip {
                        target: Some(_),
                        ..
                    } | Step::Confirm { .. }
                        | Step::Draw {
                            confirmed: true,
                            ..
                        }
                        | Step::Transform { base: Some(_), .. }
                )
            )
    }

    pub fn prompt(&self) -> String {
        match &self.step {
            Some(Step::Grip {
                target, dragging, ..
            }) => {
                return if *dragging {
                    "夹点：拖动预览，松开后等待确认；Esc 取消".into()
                } else if target.is_some() {
                    "夹点：Enter 提交；可输入坐标改目标；Esc 取消".into()
                } else {
                    "夹点：指定目标坐标或点选；Esc 取消".into()
                }
            }
            Some(Step::Draw { sketch, confirmed }) => {
                return if *confirmed {
                    format!(
                        "{}：Enter 提交；输入坐标修改最后一点；Esc 取消",
                        sketch.kind.label()
                    )
                } else {
                    format!(
                        "{}：已取 {} 点；{}；U 撤回一点，Esc 取消",
                        sketch.kind.label(),
                        sketch.points.len(),
                        if sketch.kind.continuous() {
                            "继续取点，Enter 结束，C 闭合"
                        } else {
                            "指定下一点；圆的半径/椭圆短半轴可输入长度"
                        }
                    )
                }
            }
            Some(Step::TransformSelect { kind }) => {
                return format!("{}：选择图元后 Enter；Esc 取消", kind.label())
            }
            Some(Step::Transform { kind, base, .. }) => {
                return format!(
                    "{}：{}",
                    kind.label(),
                    if base.is_some() {
                        "预览已固定，可修改左侧参数，Enter 提交"
                    } else {
                        "指定基点 / 中心 / 镜像平面经过的点"
                    }
                )
            }
            None => "就绪：选择绘图 / 变换工具；UCS XY / YZ / ZX / 3P 设置工作平面",
            Some(Step::Select) => "移动/复制 1/4：选择图元，Enter 确认选择；Esc 取消",
            Some(Step::Base { .. }) => {
                return format!(
                    "{} 2/4：指定基点（画布点选或输入坐标）；Esc 取消",
                    if self.copying { "复制" } else { "移动" }
                )
            }
            Some(Step::Target { .. }) => {
                return format!(
                    "{} 3/4：指定目标点；仅预览，不改图纸；Esc 取消",
                    if self.copying { "复制" } else { "移动" }
                )
            }
            Some(Step::Confirm { .. }) => {
                return format!(
                    "{} 4/4：Enter / 确认提交；可重新输入目标坐标；Esc 取消",
                    if self.copying { "复制" } else { "移动" }
                )
            }
            Some(Step::PlaneOrigin) => "三点平面 1/3：指定原点；Esc 取消",
            Some(Step::PlaneAxis { .. }) => "三点平面 2/3：指定 +U 方向上的点；Esc 取消",
            Some(Step::PlaneThird { .. }) => "三点平面 3/3：指定 +V 一侧的点；三点不能共线",
        }
        .to_string()
    }

    pub fn begin_grip(
        &mut self,
        id: EntityId,
        geometry: Geometry,
        kind: GripKind,
        target: Option<Point3>,
        dragging: bool,
    ) -> Result<(), String> {
        let base = properties::grips(&geometry)
            .into_iter()
            .find(|g| g.kind == kind)
            .ok_or("夹点已失效")?
            .point;
        if let Some(p) = target {
            properties::move_grip(&geometry, kind, p)?;
        }
        self.cancel();
        self.step = Some(Step::Grip {
            id,
            geometry,
            kind,
            base,
            target,
            dragging,
        });
        Ok(())
    }
    pub fn grip_dragging(&self) -> bool {
        matches!(self.step, Some(Step::Grip { dragging: true, .. }))
    }
    pub fn stop_grip_drag(&mut self) {
        if let Some(Step::Grip { dragging, .. }) = &mut self.step {
            *dragging = false;
        }
    }
    fn point_plane(&self) -> WorkPlane {
        if let Some(Step::Grip { geometry, base, .. }) = &self.step {
            let plane = match geometry {
                Geometry::Circle(c) => Some(c.plane),
                Geometry::Arc(a) => Some(a.plane),
                Geometry::Polyline(p) => p.plane,
                Geometry::Ellipse(e) => {
                    Some(Plane::new(e.major_axis.cross(e.minor_axis), e.major_axis))
                }
                _ => None,
            };
            return WorkPlane {
                origin: *base,
                x_axis: plane.map_or(self.plane.x_axis, |p| p.x_axis),
                y_axis: plane.map_or(self.plane.y_axis, |p| p.y_axis()),
            };
        }
        self.plane
    }

    pub fn begin_move(&mut self, ids: &BTreeSet<u32>) {
        self.copying = false;
        self.ray_error.clear();
        self.step = Some(if ids.is_empty() {
            Step::Select
        } else {
            Step::Base { ids: ids.clone() }
        });
        self.hover = None;
        self.error.clear();
    }

    pub fn begin_named(&mut self, name: &str, ids: &BTreeSet<u32>) -> bool {
        if matches!(name, "COPY" | "CO" | "复制") {
            self.begin_move(ids);
            self.copying = true;
            return true;
        }
        for kind in [
            SketchKind::Line,
            SketchKind::Polyline,
            SketchKind::Rectangle,
            SketchKind::Circle,
            SketchKind::Arc,
            SketchKind::Ellipse,
            SketchKind::Spline,
        ] {
            if name == kind.command()
                || matches!(
                    (name, kind),
                    ("L", SketchKind::Line)
                        | ("PL", SketchKind::Polyline)
                        | ("RECT", SketchKind::Rectangle)
                )
            {
                self.cancel();
                self.step = Some(Step::Draw {
                    sketch: Sketch::new(kind, self.plane, self.tools.degree),
                    confirmed: false,
                });
                return true;
            }
        }
        for kind in [
            TransformKind::Rotate,
            TransformKind::Scale,
            TransformKind::Mirror,
            TransformKind::RectArray,
            TransformKind::PolarArray,
        ] {
            if name == kind.command() {
                self.cancel();
                self.step = Some(if ids.is_empty() {
                    Step::TransformSelect { kind }
                } else {
                    Step::Transform {
                        kind,
                        ids: ids.clone(),
                        base: None,
                    }
                });
                return true;
            }
        }
        false
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
            Some(Step::Grip { base, .. }) => array(*base),
            Some(Step::Draw { sketch, .. }) => sketch
                .points
                .last()
                .copied()
                .map(array)
                .unwrap_or(array(self.plane.origin)),
            Some(Step::Transform {
                base: Some(base), ..
            }) => *base,
            Some(Step::Target { base, .. } | Step::Confirm { base, .. }) => *base,
            Some(Step::PlaneAxis { origin } | Step::PlaneThird { origin, .. }) => *origin,
            _ => array(self.plane.origin),
        }
    }

    fn snap_reference(&self) -> Option<Point3> {
        match &self.step {
            Some(Step::Grip { base, .. }) => Some(*base),
            Some(Step::Draw { sketch, .. }) if !sketch.points.is_empty() => {
                Some(*sketch.points.last().unwrap())
            }
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
    ) -> Result<Option<CommandCommit>, String> {
        if let Some(Step::Grip {
            id,
            geometry,
            kind,
            target: Some(target),
            ..
        }) = &self.step
        {
            if text.trim().is_empty() {
                return Ok(Some(CommandCommit::Geometry(BTreeMap::from([(
                    *id,
                    properties::move_grip(geometry, *kind, *target)?,
                )]))));
            }
        }
        if let Some(Step::TransformSelect { kind }) = self.step {
            if !text.trim().is_empty() || selected.is_empty() {
                return Err("请先选择图元，再按 Enter".into());
            }
            self.step = Some(Step::Transform {
                kind,
                ids: selected.clone(),
                base: None,
            });
            return Ok(None);
        }
        if let Some(Step::Transform {
            kind,
            ids,
            base: Some(base),
        }) = &self.step
        {
            if text.trim().is_empty() {
                let (transforms, copies) =
                    self.tools.transforms(*kind, *base, self.plane, ids.len())?;
                return Ok(Some(CommandCommit::Transform {
                    ids: ids.clone(),
                    transforms,
                    copies,
                }));
            }
            if matches!(kind, TransformKind::Rotate | TransformKind::Scale)
                && !text.contains([',', ' '])
            {
                let value = number(text)?;
                if *kind == TransformKind::Rotate {
                    self.tools.angle = value;
                } else {
                    self.tools.scale = value;
                }
                self.tools.transforms(*kind, *base, self.plane, ids.len())?;
                return Ok(None);
            }
        }
        if let Some(Step::Draw { sketch, confirmed }) = &mut self.step {
            let upper = text.trim().to_uppercase();
            if matches!(upper.as_str(), "U" | "UNDO") {
                sketch.points.pop().ok_or("没有可撤回的点")?;
                sketch.closed = false;
                *confirmed = false;
                return Ok(None);
            }
            if text.trim().is_empty() {
                let geometry = sketch.geometry()?;
                if *confirmed {
                    return Ok(Some(CommandCommit::Create {
                        geometry,
                        normal: sketch.plane.normal(),
                    }));
                }
                *confirmed = true;
                return Ok(None);
            }
            if matches!(upper.as_str(), "C" | "CLOSE" | "闭合") {
                if !sketch.kind.continuous() {
                    return Err("该工具不支持闭合选项".into());
                }
                let mut next = sketch.clone();
                next.closed = true;
                next.geometry()?;
                *sketch = next;
                *confirmed = true;
                return Ok(None);
            }
            if !text.contains([',', ' '])
                && ((sketch.kind == SketchKind::Circle && sketch.points.len() >= 1)
                    || (sketch.kind == SketchKind::Ellipse && sketch.points.len() >= 2))
            {
                let radius = number(text)?;
                if radius <= 1e-9 {
                    return Err("半径/半轴长度必须大于零".into());
                }
                let c = sketch.points[0];
                let direction = if sketch.kind == SketchKind::Circle {
                    sketch.plane.x_axis
                } else {
                    let a = sketch.points[1];
                    sketch.plane.normal().cross(
                        cad_core::model::Vector3::new(a.x - c.x, a.y - c.y, a.z - c.z).normalize(),
                    )
                };
                let p = [
                    c.x + direction.x * radius,
                    c.y + direction.y * radius,
                    c.z + direction.z * radius,
                ];
                self.accept_point(p)?;
                return Ok(None);
            }
        }
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
                let commit = MoveCommit {
                    ids: ids.clone(),
                    offset: std::array::from_fn(|i| target[i] - base[i]),
                };
                return Ok(Some(if self.copying {
                    CommandCommit::Transform {
                        ids: commit.ids,
                        transforms: vec![AffineTransform::translation(commit.offset)?],
                        copies: true,
                    }
                } else {
                    CommandCommit::Move(commit)
                }));
            }
            let point = self.hover.ok_or("请点选有效位置，或输入精确坐标")?;
            self.accept_point(point)?;
        } else {
            let point = parse_point(text, self.point_plane(), self.reference())?;
            self.accept_point(point)?;
        }
        Ok(None)
    }

    fn accept_point(&mut self, point: [f64; 3]) -> Result<(), String> {
        self.step = match self.step.as_ref().ok_or("没有正在取点的命令")? {
            Step::Grip {
                id,
                geometry,
                kind,
                base,
                ..
            } => {
                properties::move_grip(geometry, *kind, as_point(point))?;
                Some(Step::Grip {
                    id: *id,
                    geometry: geometry.clone(),
                    kind: *kind,
                    base: *base,
                    target: Some(as_point(point)),
                    dragging: false,
                })
            }
            Step::Draw { sketch, confirmed } => {
                let mut next = sketch.clone();
                if *confirmed {
                    next.points.pop();
                }
                next.push(as_point(point))?;
                let done = *confirmed
                    || !next.kind.continuous() && next.points.len() >= next.kind.required();
                if done {
                    next.geometry()?;
                }
                Some(Step::Draw {
                    sketch: next,
                    confirmed: done,
                })
            }
            Step::Transform { kind, ids, .. } => {
                self.tools.transforms(*kind, point, self.plane, ids.len())?;
                Some(Step::Transform {
                    kind: *kind,
                    ids: ids.clone(),
                    base: Some(point),
                })
            }
            Step::TransformSelect { .. } => return Err("请先确认选择".into()),
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
        let plane = self.point_plane();
        let point = frame
            .ray(camera, transform, cursor)
            .ok_or_else(|| "当前相机无法取点".to_string())
            .and_then(|ray| {
                self.aids.resolve(
                    document, frame, camera, transform, cursor, ray, plane, reference,
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
                        Err(error) => {
                            self.error = error;
                            if let Some(Step::Grip { dragging, .. }) = &mut self.step {
                                *dragging = false;
                            }
                        }
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

    pub fn drawing_preview(&self) -> Vec<Vec<[f64; 3]>> {
        if let Some(Step::Grip {
            geometry,
            kind,
            target,
            ..
        }) = &self.step
        {
            return target
                .or(self.hover.map(as_point))
                .and_then(|p| properties::move_grip(geometry, *kind, p).ok())
                .map(|g| super::display::tessellate_geometry(&g))
                .unwrap_or_default();
        }
        let Some(Step::Draw { sketch, confirmed }) = &self.step else {
            return Vec::new();
        };
        let mut sketch = sketch.clone();
        if !confirmed {
            if let Some(p) = self.hover {
                let _ = sketch.push(as_point(p));
            }
        }
        match sketch.geometry() {
            Ok(geometry) => geometry
                .iter()
                .flat_map(super::display::tessellate_geometry)
                .collect(),
            Err(_) => vec![sketch.points.iter().copied().map(array).collect()],
        }
    }
    pub fn transform_preview(&self) -> Option<(BTreeSet<u32>, Vec<AffineTransform>)> {
        let Some(Step::Transform { kind, ids, base }) = &self.step else {
            return None;
        };
        let (transforms, _) = self
            .tools
            .transforms(*kind, base.or(self.hover)?, self.plane, ids.len())
            .ok()?;
        Some((ids.clone(), transforms))
    }
    /// Return a command-bar submission, keeping all buttons on the same execution path.
    pub fn controls_ui(
        &mut self,
        ui: &mut egui::Ui,
        document: &cad_core::DrawingDocument,
    ) -> Option<String> {
        let mut command = None;
        ui.strong("绘图、变换与工作平面");
        self.aids.controls(ui);
        ui.add_enabled_ui(!self.is_active(), |ui| self.tools.style_ui(ui, document));
        if let Some(Step::Transform { kind, ids, base }) = &self.step {
            self.tools.transform_ui(ui, *kind);
            if let Some(base) = base {
                if let Err(error) = self.tools.transforms(*kind, *base, self.plane, ids.len()) {
                    ui.colored_label(egui::Color32::LIGHT_RED, error);
                }
            }
        }
        ui.label(self.prompt());
        if let Some(Step::Grip { base, .. }) = &self.step {
            ui.small(format!(
                "夹点临时平面原点：{:.4}, {:.4}, {:.4}；U/V 沿图元平面，三维输入仍为 WCS。",
                base.x, base.y, base.z
            ));
        }
        if self.is_active() {
            if !self.error_message().is_empty() {
                ui.colored_label(egui::Color32::LIGHT_RED, self.error_message());
            }
            if let Some(point) = self.hover {
                ui.label(format!(
                    "WCS：{:.4}, {:.4}, {:.4} mm",
                    point[0], point[1], point[2]
                ));
                let local = self.point_plane().to_local(as_point(point));
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
                if self.selecting() && ui.button("确认选择").clicked() {
                    command = Some(String::new());
                }
                if matches!(
                    self.step,
                    Some(
                        Step::Grip { target: Some(_), .. } | Step::Confirm { .. }
                            | Step::Draw {
                                confirmed: true,
                                ..
                            }
                            | Step::Transform { base: Some(_), .. }
                    )
                ) && ui.button("确认 / 提交").clicked()
                {
                    command = Some(String::new());
                }
                if matches!(
                    self.step,
                    Some(Step::Draw {
                        confirmed: false,
                        ..
                    })
                ) {
                    let continuous = matches!(&self.step, Some(Step::Draw { sketch, .. }) if sketch.kind.continuous());
                    if continuous && ui.button("结束取点").clicked() {
                        command = Some(String::new());
                    }
                    if ui.button("撤回一点").clicked() {
                        command = Some("U".into());
                    }
                    if continuous && ui.button("闭合").clicked() {
                        command = Some("C".into());
                    }
                }
                if ui.button("取消  Esc").clicked() {
                    command = Some("CANCEL".into());
                }
            });
            ui.small("x,y,z：WCS；u,v：平面；@：相对；@长度<角度：平面极坐标");
        } else {
            egui::CollapsingHeader::new("绘图与变换工具")
                .default_open(true)
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        for kind in [
                            SketchKind::Line,
                            SketchKind::Polyline,
                            SketchKind::Rectangle,
                            SketchKind::Circle,
                            SketchKind::Arc,
                            SketchKind::Ellipse,
                            SketchKind::Spline,
                        ] {
                            if ui.button(kind.label()).clicked() {
                                command = Some(kind.command().into());
                            }
                        }
                    });
                    ui.horizontal_wrapped(|ui| {
                        if ui.button("复制").clicked() {
                            command = Some("COPY".into());
                        }
                        for kind in [
                            TransformKind::Rotate,
                            TransformKind::Scale,
                            TransformKind::Mirror,
                            TransformKind::RectArray,
                            TransformKind::PolarArray,
                        ] {
                            if ui.button(kind.label()).clicked() {
                                command = Some(kind.command().into());
                            }
                        }
                    });
                });
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
        let plane = self.point_plane();
        self.aids.draw(gizmos, frame, plane);
        let length = (extent * 0.2).clamp(5.0, 1000.0);
        let origin = frame.point(array(plane.origin));
        if !origin.is_finite() {
            return;
        }
        for (local, color) in [
            ([length, 0.0, 0.0], Color::srgb(1.0, 0.4, 0.4)),
            ([0.0, length, 0.0], Color::srgb(0.3, 0.8, 1.0)),
            ([0.0, 0.0, length * 0.5], Color::srgb(0.4, 1.0, 0.5)),
        ] {
            gizmos.line(origin, frame.point(array(plane.to_world(local))), color);
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
                frame.point(array(plane.to_world(pair[0]))),
                frame.point(array(plane.to_world(pair[1]))),
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
