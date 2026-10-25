//! Command steps and transient previews; only confirmation submits a CAD edit.

use std::collections::{BTreeMap, BTreeSet};

use super::cad_tools::{ToolOptions, TransformKind};
use bevy::prelude::*;
use bevy_egui::egui;
use cad_core::geometry_edit::{AffineTransform, Sketch, SketchKind};
use cad_core::model::{EntityId, Geometry, Plane, Point3, Vector3, WorkPlane};
use cad_core::properties::{self, GripKind};

enum Step {
    TransformPick(super::cad_transform_command::Session),
    CurvePick(super::cad_curve_command::Session),
    PlaneSelect,
    EraseSelect,
    Utility {
        kind: super::cad_util::Kind,
        points: Vec<Point3>,
        confirmed: bool,
        ids: BTreeSet<u32>,
    },
    Paste {
        items: Vec<cad_core::geometry_edit::ClipboardItem>,
        base: [f64; 3],
        target: Option<[f64; 3]>,
    },
    ArraySpacing {
        ids: BTreeSet<u32>,
        base: [f64; 3],
    },
    Curves(cad_core::CurveEdit),
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

#[derive(Clone, Copy)]
enum RectangleDimensions {
    Length,
    Width(f64),
    Place([f64; 2]),
}

pub struct MoveCommit {
    pub ids: BTreeSet<u32>,
    pub offset: [f64; 3],
}

pub enum CommandCommit {
    Delete(BTreeSet<u32>),
    Report(String),
    Annotation(Vec<Geometry>),
    Clipboard {
        ids: BTreeSet<u32>,
        base: [f64; 3],
    },
    Paste {
        items: Vec<cad_core::geometry_edit::ClipboardItem>,
        offset: [f64; 3],
    },
    Curves(cad_core::CurveEdit),
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
    pub utility: super::cad_util::Options,
    pub last_command: String,
    pub last_measurement: String,
    copying: bool,
    copy_single: bool,
    copy_operations: Vec<u64>,
    rectangle_dimensions: Option<RectangleDimensions>,
    rectangle_angle: f64,
    rectangle_rotating: bool,
    pub aids: super::cad_aids::DraftingAids,
    pub plane: WorkPlane,
    step: Option<Step>,
    hover: Option<[f64; 3]>,
    pointer_point: Option<[f64; 3]>,
    pub error: String,
    ray_error: String,
    origin_input: [f64; 3],
}

impl CommandState {
    pub fn begin_curve_pick(
        &mut self,
        tool: cad_core::curve_edit::CurveTool,
        options: cad_core::curve_edit::CurveOptions,
    ) {
        self.cancel();
        self.step = Some(Step::CurvePick(super::cad_curve_command::Session::new(
            tool, options,
        )));
    }
    pub fn curve_length(&self) -> Option<f64> {
        if let Some(Step::CurvePick(s)) = &self.step {
            Some(s.length())
        } else {
            None
        }
    }
    pub fn continuous_curve(&self) -> bool {
        matches!(&self.step,Some(Step::CurvePick(s)) if s.continuous())
    }
    pub fn copy_mode(&self) -> Option<bool> {
        (self.copying
            && matches!(
                self.step,
                Some(Step::Select | Step::Base { .. } | Step::Target { .. } | Step::Confirm { .. })
            ))
        .then_some(!self.copy_single)
    }
    pub fn command_undo_active(&self) -> bool {
        self.continuous_curve() || self.copy_mode().is_some()
    }
    pub fn repeat_edit(&self) -> bool {
        self.continuous_curve() || self.copy_mode() == Some(true)
    }
    pub fn copy_waiting_target(&self) -> bool {
        self.copying && matches!(self.step, Some(Step::Target { .. }))
    }
    pub fn circle_drawing(&self) -> bool {
        matches!(&self.step, Some(Step::Draw {sketch,..}) if sketch.kind.is_circle())
    }
    pub fn arc_drawing(&self) -> bool {
        matches!(&self.step,Some(Step::Draw{sketch,..}) if sketch.kind.is_arc())
    }
    pub fn arc_options(&self) -> &'static [(&'static str, &'static str)] {
        if let Some(Step::Draw {
            sketch,
            confirmed: false,
        }) = &self.step
        {
            if sketch.kind.is_arc() {
                if sketch.points.is_empty() {
                    return &[
                        ("3P", "三点圆弧"),
                        ("C", "圆心 / 起点 / 终点方向"),
                        ("A", "圆心 / 起点 / 扫角"),
                    ];
                }
                if matches!(
                    sketch.kind,
                    SketchKind::ArcCenterEnd | SketchKind::ArcCenterAngle
                ) {
                    return &[("E", "终点方向方式"), ("A", "扫角方式")];
                }
            }
        }
        &[]
    }
    pub fn arc_sweep_input(&self) -> bool {
        matches!(&self.step,Some(Step::Draw{sketch,..}) if sketch.kind==SketchKind::ArcCenterAngle && sketch.points.len()==2)
    }
    pub fn rectangle_drawing(&self) -> bool {
        matches!(&self.step,Some(Step::Draw{sketch,..}) if sketch.kind==SketchKind::Rectangle)
    }
    pub fn rectangle_corner(&self) -> bool {
        matches!(&self.step,Some(Step::Draw{sketch,confirmed:false}) if sketch.kind==SketchKind::Rectangle && sketch.points.len()==1)
    }
    pub fn rectangle_placing(&self) -> bool {
        self.rectangle_corner()
            && matches!(
                self.rectangle_dimensions,
                Some(RectangleDimensions::Place(_))
            )
    }
    pub fn circle_options(&self) -> &'static [(&'static str, &'static str)] {
        if let Some(Step::Draw {
            sketch,
            confirmed: false,
        }) = &self.step
        {
            if sketch.kind.is_circle() {
                if sketch.points.is_empty() {
                    return &[
                        ("R", "圆心 / 半径"),
                        ("D", "圆心 / 直径"),
                        ("2P", "两点定直径"),
                        ("3P", "三点定圆"),
                    ];
                }
                if sketch.points.len() == 1
                    && matches!(sketch.kind, SketchKind::Circle | SketchKind::CircleDiameter)
                {
                    return &[("R", "按半径输入"), ("D", "按直径输入")];
                }
            }
        }
        &[]
    }
    pub fn curve_waiting_target(&self) -> bool {
        matches!(&self.step,Some(Step::CurvePick(s)) if s.waiting_target())
    }
    pub fn curve_change_distance(&self) -> bool {
        matches!(&self.step,Some(Step::CurvePick(s)) if s.can_change_distance())
    }
    pub fn transform_reference(&self) -> bool {
        matches!(&self.step,Some(Step::TransformPick(s)) if s.can_reference())
    }
    pub fn mirror_question(&self) -> bool {
        matches!(&self.step,Some(Step::TransformPick(s)) if s.mirror_question())
    }
    pub fn command_undo_id(&self) -> Option<u64> {
        if let Some(Step::CurvePick(s)) = &self.step {
            s.undo_id()
        } else if self.copy_mode().is_some() {
            self.copy_operations.last().copied()
        } else {
            None
        }
    }
    pub fn curve_boundaries(&self) -> Option<&BTreeSet<u32>> {
        if let Some(Step::CurvePick(s)) = &self.step {
            Some(s.boundaries())
        } else {
            None
        }
    }
    pub fn record_command_change(
        &mut self,
        change: &cad_core::DrawingChange,
        doc: &cad_core::DrawingDocument,
    ) {
        if let Some(Step::CurvePick(s)) = &mut self.step {
            if s.continuous() {
                s.committed(change, doc);
            }
        } else if self.copy_mode().is_some() {
            self.copy_operations.push(change.transaction_id);
        }
    }
    pub fn undo_command_change(&mut self, id: u64, doc: &cad_core::DrawingDocument) {
        if let Some(Step::CurvePick(s)) = &mut self.step {
            s.undone(id, doc);
        } else if self.copy_mode().is_some() && self.copy_operations.last() == Some(&id) {
            self.copy_operations.pop();
        }
        self.resume_edit();
    }
    pub fn resume_edit(&mut self) {
        if let Some(Step::CurvePick(s)) = &mut self.step {
            s.resume();
        } else if self.copying {
            if let Some(Step::Confirm { ids, base, .. }) = &self.step {
                self.step = Some(Step::Target {
                    ids: ids.clone(),
                    base: *base,
                });
            }
        }
        self.hover = None;
        self.pointer_point = None;
        self.aids.clear();
        self.ray_error.clear();
        self.error.clear();
    }
    pub fn picking_curve(&self) -> bool {
        matches!(&self.step,Some(Step::CurvePick(s)) if s.picking())
    }
    pub fn curve_viewport(
        &mut self,
        doc: &cad_core::DrawingDocument,
        id: Option<u32>,
        ray: (Point3, cad_core::model::Vector3),
        click: bool,
    ) -> Result<bool, String> {
        let result = if let Some(Step::CurvePick(s)) = &mut self.step {
            s.viewport(doc, id, ray, click)
        } else {
            Ok(false)
        };
        if matches!(result, Ok(true)) {
            self.hover = None;
            self.pointer_point = None;
            self.ray_error.clear();
        }
        result
    }
    pub fn begin_erase(&mut self) {
        self.cancel();
        self.step = Some(Step::EraseSelect);
    }
    pub fn begin_plane_select(&mut self) {
        self.cancel();
        self.step = Some(Step::PlaneSelect);
    }
    pub fn continuous_drawing(&self) -> bool {
        matches!(&self.step, Some(Step::Draw { sketch, .. }) if sketch.kind.continuous())
    }
    pub fn finish_on_cancel(&self) -> bool {
        matches!(&self.step, Some(Step::Draw { sketch, .. }) if sketch.kind.continuous() && sketch.geometry().is_ok())
    }
    pub fn ready(&self) -> bool {
        if let Some(Step::TransformPick(s)) = &self.step {
            return s.ready();
        }
        if let Some(Step::CurvePick(s)) = &self.step {
            return s.confirmed();
        }
        matches!(
            &self.step,
            Some(
                Step::Confirm { .. }
                    | Step::Draw {
                        confirmed: true,
                        ..
                    }
                    | Step::Paste {
                        target: Some(_),
                        ..
                    }
                    | Step::Grip {
                        target: Some(_),
                        dragging: false,
                        ..
                    }
            )
        ) || matches!(&self.step, Some(Step::Utility { kind, confirmed: true, .. }) if *kind != super::cad_util::Kind::Text)
    }
    fn finish_ready(&mut self, selected: &BTreeSet<u32>) -> Result<Option<CommandCommit>, String> {
        if self.ready() {
            self.input("", selected)
        } else {
            Ok(None)
        }
    }
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
    pub fn needs_options(&self) -> bool {
        matches!(
            self.step,
            Some(
                Step::Transform { .. }
                    | Step::TransformPick(_)
                    | Step::Curves(_)
                    | Step::CurvePick(_)
            )
        ) || matches!(self.step, Some(Step::Utility { kind, .. }) if !kind.measuring())
    }
    pub fn input_spec(&self) -> Option<super::cad_input::Spec> {
        use super::cad_input::{Kind, Spec};
        if self.arc_sweep_input() {
            let Some(Step::Draw { sketch, .. }) = &self.step else {
                unreachable!()
            };
            return Some(Spec {
                kind: Kind::ArcSweep,
                reference: None,
                plane: sketch.plane,
                point: None,
                value: sketch.sweep_degrees.unwrap_or(90.),
                secondary: 0.,
            });
        }
        if self.rectangle_corner() {
            if self.rectangle_rotating {
                return Some(Spec {
                    kind: Kind::RectangleAngle,
                    reference: None,
                    plane: self.plane,
                    point: None,
                    value: self.rectangle_angle,
                    secondary: 0.,
                });
            }
            let kind = match self.rectangle_dimensions {
                Some(RectangleDimensions::Length) => Some(Kind::RectangleLength),
                Some(RectangleDimensions::Width(_)) => Some(Kind::RectangleWidth),
                Some(RectangleDimensions::Place(_)) => return None,
                None => None,
            };
            if let Some(kind) = kind {
                return Some(Spec {
                    kind,
                    reference: None,
                    plane: self.plane,
                    point: None,
                    value: 0.,
                    secondary: 0.,
                });
            }
        }
        if let Some(Step::TransformPick(s)) = &self.step {
            if let Some(spec) = s.spec(self.hover.map(as_point), &self.tools, self.plane) {
                return Some(spec);
            }
        }
        if let Some(Step::CurvePick(s)) = &self.step {
            if !s.side() {
                return s.parameter_spec(self.point_plane());
            }
        }
        if let Some(Step::Grip {
            base,
            geometry,
            kind,
            target,
            ..
        }) = &self.step
        {
            let center = if *kind == GripKind::Radius {
                match geometry {
                    Geometry::Circle(c) => Some(c.center),
                    Geometry::Arc(a) => Some(a.center),
                    _ => None,
                }
            } else {
                None
            };
            return Some(Spec {
                kind: if center.is_some() {
                    Kind::Radius
                } else {
                    Kind::Polar
                },
                reference: Some(array(center.unwrap_or(*base))),
                plane: self.point_plane(),
                point: target.map(array).or(self.hover).or(Some(array(*base))),
                value: 0.,
                secondary: 0.,
            });
        }
        if let Some(Step::Transform {
            kind,
            base: Some(_),
            ..
        }) = self.step
        {
            let (kind, value) = match kind {
                TransformKind::Rotate => (Kind::Angle, self.tools.angle),
                TransformKind::Scale => (Kind::Scale, self.tools.scale),
                TransformKind::RectArray => (Kind::ArrayRect, self.tools.columns as f64),
                TransformKind::PolarArray => (Kind::Count, self.tools.count as f64),
                _ => return None,
            };
            return Some(Spec {
                kind,
                reference: None,
                plane: self.point_plane(),
                point: None,
                value,
                secondary: self.tools.rows as f64,
            });
        }
        if !self.takes_point() {
            return None;
        }
        let reference = self.snap_reference().map(array);
        let radius = matches!(&self.step, Some(Step::Draw { sketch, .. }) if matches!(sketch.kind,SketchKind::Circle | SketchKind::CircleDiameter) && !sketch.points.is_empty());
        Some(Spec {
            kind: if radius {
                if matches!(&self.step,Some(Step::Draw{sketch,..}) if sketch.kind==SketchKind::CircleDiameter)
                {
                    Kind::Diameter
                } else {
                    Kind::Radius
                }
            } else if reference.is_some() {
                Kind::Polar
            } else {
                Kind::Coordinates
            },
            reference,
            plane: self.point_plane(),
            point: self.hover,
            value: 0.,
            secondary: 0.,
        })
    }
    pub fn preview_scalar(&mut self, text: &str) {
        if self.arc_sweep_input() {
            if let Some(Step::Draw { sketch, confirmed }) = &mut self.step {
                if !text.trim().is_empty() || !*confirmed {
                    sketch.sweep_degrees = number(text).ok();
                }
            }
            return;
        }
        if let Some(Step::TransformPick(s)) = &mut self.step {
            s.preview_value(text, &mut self.tools);
            return;
        }
        let Some(Step::Transform {
            kind,
            ids,
            base: Some(base),
        }) = &self.step
        else {
            return;
        };
        let mut tools = self.tools.clone();
        if update_transform(&mut tools, *kind, text).is_err() {
            return;
        }
        if tools
            .transforms(*kind, *base, self.plane, ids.len())
            .is_ok()
        {
            self.tools = tools;
        }
    }
    pub fn refresh_input_preview(&mut self, dynamic: &super::cad_input::DynamicInput, text: &str) {
        if let Some(result) = self.circle_value_point(text) {
            match result {
                Ok(point) => {
                    self.hover = Some(point);
                    self.ray_error.clear();
                }
                Err(error) => {
                    self.hover = None;
                    self.ray_error = error;
                }
            }
            return;
        }
        if matches!(
            &self.step,
            Some(Step::Grip {
                target: Some(_),
                dragging: false,
                ..
            })
        ) && !text.trim().is_empty()
        {
            let point = if matches!(
                &self.step,
                Some(Step::Grip {
                    kind: GripKind::Radius,
                    ..
                })
            ) && !text.contains([',', ' ', '<'])
                && !text.starts_with(['@', '#'])
            {
                self.radius_grip_point(text)
            } else {
                parse_point(
                    &dynamic.submission(self, text),
                    self.point_plane(),
                    self.reference(),
                )
            };
            if let Ok(point) = point {
                if let Some(Step::Grip {
                    geometry,
                    kind,
                    target,
                    ..
                }) = &mut self.step
                {
                    if properties::move_grip(geometry, *kind, as_point(point)).is_ok() {
                        *target = Some(as_point(point));
                    }
                }
            }
        }
        if !self.takes_point() {
            return;
        }
        let literal = if dynamic.is_literal() {
            parse_point(
                &dynamic.submission(self, text),
                self.point_plane(),
                self.reference(),
            )
            .ok()
        } else {
            None
        };
        if let Some(point) = literal.or(self.pointer_point) {
            let result = if literal.is_some() {
                Ok(point)
            } else {
                dynamic.constrain(self, text, point)
            };
            match result.and_then(|point| self.rectangle_target(point)) {
                Ok(point) => {
                    self.hover = Some(point);
                    self.ray_error.clear();
                    if let Some(Step::CurvePick(s)) = &mut self.step {
                        if let Err(error) = s.preview_side(as_point(point), false) {
                            self.ray_error = error;
                        }
                    }
                }
                Err(error) => {
                    self.hover = None;
                    self.ray_error = error;
                }
            }
        }
    }
    fn circle_value_point(&self, text: &str) -> Option<Result<[f64; 3], String>> {
        if matches!(
            text.trim().to_uppercase().as_str(),
            "R" | "RADIUS" | "D" | "DIAMETER" | "U" | "UNDO"
        ) {
            return None;
        }
        let Some(Step::Draw {
            sketch,
            confirmed: false,
        }) = &self.step
        else {
            return None;
        };
        if !matches!(sketch.kind, SketchKind::Circle | SketchKind::CircleDiameter)
            || sketch.points.len() != 1
            || text.trim().is_empty()
            || text.contains([',', '，', '<', ' '])
            || text.starts_with(['@', '#'])
        {
            return None;
        }
        Some((|| {
            let value = number(text)?;
            if value <= 1e-9 {
                return Err("半径/直径须大于几何容差".into());
            }
            let c = sketch.points[0];
            let direction = self
                .pointer_point
                .map(as_point)
                .map(|p| Vector3::new(p.x - c.x, p.y - c.y, p.z - c.z))
                .map(|d| d - sketch.plane.normal() * d.dot(sketch.plane.normal()))
                .filter(|d| d.length() > 1e-9)
                .map(|d| d.normalize())
                .unwrap_or(sketch.plane.x_axis);
            let d = direction * value;
            finite([c.x + d.x, c.y + d.y, c.z + d.z])
        })())
    }
    fn rectangle_target(&self, point: [f64; 3]) -> Result<[f64; 3], String> {
        let Some(RectangleDimensions::Place(size)) = self.rectangle_dimensions else {
            return Ok(point);
        };
        let Some(Step::Draw {
            sketch,
            confirmed: false,
        }) = &self.step
        else {
            return Ok(point);
        };
        if sketch.kind != SketchKind::Rectangle || sketch.points.len() != 1 {
            return Ok(point);
        }
        let a = sketch.plane.to_local(sketch.points[0]);
        let b = sketch.plane.to_local(as_point(point));
        if !a.iter().chain(b.iter()).all(|v| v.is_finite()) {
            return Err("矩形方向超出有效数值范围".into());
        }
        if b[2].abs() > 1e-5 {
            return Err("矩形须在当前工作平面内放置；请启用投影捕捉或调整平面".into());
        }
        finite(array(sketch.plane.to_world([
            a[0] + if b[0] < a[0] { -size[0] } else { size[0] },
            a[1] + if b[1] < a[1] { -size[1] } else { size[1] },
            0.,
        ])))
    }
    fn radius_grip_point(&self, text: &str) -> Result<[f64; 3], String> {
        let Some(Step::Grip { geometry, base, .. }) = &self.step else {
            return Err("当前不是半径夹点".into());
        };
        let center = match geometry {
            Geometry::Circle(c) => c.center,
            Geometry::Arc(a) => a.center,
            _ => return Err("当前不是圆/圆弧半径夹点".into()),
        };
        let radius = number(text)?;
        if radius <= cad_core::curves::DISTANCE_TOLERANCE {
            return Err("半径须大于几何容差".into());
        }
        let d =
            cad_core::model::Vector3::new(base.x - center.x, base.y - center.y, base.z - center.z)
                .normalize()
                * radius;
        finite([center.x + d.x, center.y + d.y, center.z + d.z])
    }
    pub fn pause_pointer(&mut self) {
        // Keep the last geometric point while the user edits a numeric field.
        self.aids.clear();
    }
    pub fn selecting(&self) -> bool {
        if let Some(Step::TransformPick(s)) = &self.step {
            return s.selecting();
        }
        if let Some(Step::CurvePick(s)) = &self.step {
            return s.selecting();
        }
        matches!(
            self.step,
            Some(Step::EraseSelect | Step::Select | Step::TransformSelect { .. })
        ) || matches!(&self.step,Some(Step::Utility{kind:super::cad_util::Kind::CopyBase,ids,..}) if ids.is_empty())
    }
    pub fn takes_point(&self) -> bool {
        if self.arc_sweep_input() {
            return false;
        }
        if self.rectangle_corner() && self.rectangle_rotating {
            return false;
        }
        if self.rectangle_corner()
            && matches!(
                self.rectangle_dimensions,
                Some(RectangleDimensions::Length | RectangleDimensions::Width(_))
            )
        {
            return false;
        }
        if let Some(Step::TransformPick(s)) = &self.step {
            return s.takes_point();
        }
        if let Some(Step::CurvePick(s)) = &self.step {
            return s.side();
        }
        self.is_active()
            && !self.selecting()
            && !matches!(
                self.step,
                Some(
                    Step::Utility {
                        confirmed: true,
                        ..
                    } | Step::Paste {
                        target: Some(_),
                        ..
                    } | Step::Curves(_)
                        | Step::PlaneSelect
                        | Step::Grip {
                            target: Some(_),
                            ..
                        }
                        | Step::Confirm { .. }
                        | Step::Draw {
                            confirmed: true,
                            ..
                        }
                        | Step::Transform { base: Some(_), .. }
                )
            )
    }

    pub fn prompt(&self) -> String {
        if self.rectangle_corner() {
            if self.rectangle_rotating {
                return format!(
                    "矩形旋转：输入相对当前工作平面 U 轴的角度 <{:.4}°>；U 返回上一步，Esc 取消",
                    self.rectangle_angle
                );
            }
            let prefix = format!("矩形（旋转 {:.4}°）", self.rectangle_angle);
            return match self.rectangle_dimensions {
                Some(RectangleDimensions::Length)=>format!("{prefix}：输入沿旋转后 U 的正长度；R 设置角度，U 返回对角点方式"),
                Some(RectangleDimensions::Width(length))=>format!("{prefix}：长度 {length:.4} mm；输入沿旋转后 V 的正宽度；R 设置角度，U 重输长度"),
                Some(RectangleDimensions::Place([length,width]))=>format!("{prefix} {length:.4} × {width:.4} mm：移动鼠标选择旋转后象限，点击/Enter 确认；D 重输尺寸，R 设置角度，U 重输宽度"),
                None=>format!("{prefix}：指定对角点，或 D 输入长宽 / R 设置角度；U 撤回首角点，Esc 取消"),
            };
        }
        if let Some(multiple) = self.copy_mode() {
            if matches!(self.step, Some(Step::Base { .. } | Step::Target { .. })) {
                return format!("复制（{}，已完成 {} 次）：{}；M 多次 / S 单次；U 撤回本命令上次复制；Esc 结束保留结果",
                    if multiple {"多次"}else{"单次"}, self.copy_operations.len(),
                    if matches!(self.step,Some(Step::Base { .. })) {"指定基点"}else{"指定目标点；空输入 Enter/空格结束"});
            }
        }
        match &self.step {
            Some(Step::TransformPick(s)) => return s.prompt(),
            Some(Step::CurvePick(s)) => return s.prompt(),
            Some(Step::PlaneSelect) => {
                return "工作平面：输入 XY / YZ / ZX / 3P，Enter 默认为 XY；Esc 取消".into()
            }
            Some(Step::EraseSelect) => {
                return "删除：选择对象，Enter/空格确认；Shift 移除，Esc 取消".into()
            }
            Some(Step::Utility {
                kind,
                points,
                confirmed,
                ids,
            }) => {
                return if *kind == super::cad_util::Kind::CopyBase && ids.is_empty() {
                    "复制基点：选择图元后 Enter".into()
                } else {
                    format!(
                        "{}：已取 {}/{} 点；{}",
                        kind.label(),
                        points.len(),
                        kind.required(),
                        if *confirmed {
                            "Enter 完成；可输入坐标修改末点，Esc 取消"
                        } else {
                            "点选/输入坐标；Esc 取消"
                        }
                    )
                }
            }
            Some(Step::Paste { base, target, .. }) => {
                return format!(
                    "粘贴：原基点 ({:.3}, {:.3}, {:.3}) mm；{}",
                    base[0],
                    base[1],
                    base[2],
                    if target.is_some() {
                        "Enter 提交；可输入坐标改目标"
                    } else {
                        "指定目标点"
                    }
                )
            }
            Some(Step::ArraySpacing { .. }) => {
                return "矩形阵列：相对于阵列基点拾取列/行间距；Esc 取消".into()
            }
            Some(Step::Curves(edit)) => {
                return format!(
                    "预览：输出 {} 项，删除 {} 项{}；Enter 提交，Esc 取消。红色为待删除对象。",
                    edit.replacements.values().map(Vec::len).sum::<usize>(),
                    edit.remove.len(),
                    if edit.copies {
                        "（保留原件）"
                    } else {
                        ""
                    }
                )
            }
            Some(Step::Grip {
                target, dragging, ..
            }) => {
                return if *dragging {
                    "夹点：拖动预览，松开提交；Esc 取消".into()
                } else if target.is_some() {
                    "夹点：Enter 提交；可输入坐标改目标；Esc 取消".into()
                } else {
                    "夹点：指定目标坐标或点选；Esc 取消".into()
                }
            }
            Some(Step::Draw { sketch, confirmed }) => {
                if sketch.kind.is_arc() && !confirmed {
                    return format!(
                        "{}：{}；U 撤回取点，Esc 取消",
                        sketch.kind.label(),
                        match (sketch.kind, sketch.points.len()) {
                            (_, 0) => "指定首点；3P 三点 / C 圆心终点方向 / A 圆心扫角",
                            (SketchKind::Arc, 1) => "指定经过点",
                            (SketchKind::Arc, _) => "指定终点（三点不能共线）",
                            (_, 1) => "指定起点（与圆心确定半径）；E 终点方向 / A 扫角",
                            (SketchKind::ArcCenterEnd, _) =>
                                "指定从圆心引出的终点射线，默认逆时针；A 改为扫角",
                            _ => "输入扫角 <90°>，正角逆时针、负角顺时针；E 改为终点方向",
                        }
                    );
                }
                if sketch.kind.is_circle() && !confirmed {
                    return format!(
                        "{}：{}；U 撤回取点，Esc 取消",
                        sketch.kind.label(),
                        match (sketch.kind, sketch.points.len()) {
                            (_, 0) => "指定圆心/首点；R 圆心半径 / D 圆心直径 / 2P 两点 / 3P 三点",
                            (SketchKind::Circle, _) => "指定半径点或输入半径；D 改用直径",
                            (SketchKind::CircleDiameter, _) =>
                                "指定直径方向点或输入直径；R 改用半径",
                            (SketchKind::Circle2P, _) =>
                                "指定直径另一端点；数值为沿光标方向的直径长度",
                            (_, 1) => "指定圆周第二点",
                            _ => "指定圆周第三点（不能共线）",
                        }
                    );
                }
                return if *confirmed {
                    format!(
                        "{}：Enter 提交；输入坐标修改最后一点；Esc 取消",
                        sketch.kind.label()
                    )
                } else {
                    format!(
                        "{}：已取 {} 点；{}；U 撤回一点",
                        sketch.kind.label(),
                        sketch.points.len(),
                        if sketch.kind.continuous() {
                            "继续取点，Enter/空格结束，C 闭合；Esc 保留已完成部分"
                        } else {
                            "指定下一点；圆的半径/椭圆短半轴可输入长度"
                        }
                    )
                };
            }
            Some(Step::TransformSelect { kind }) => {
                return format!("{}：选择图元后 Enter；Esc 取消", kind.label())
            }
            Some(Step::Transform { kind, base, .. }) => {
                return format!(
                    "{}：{}",
                    kind.label(),
                    if base.is_some() {
                        match kind {
                            TransformKind::Rotate => "输入角度后确认即完成；可在右侧设置轴",
                            TransformKind::Scale => "输入比例后确认即完成",
                            TransformKind::RectArray => {
                                "输入列数,行数后确认；间距可在右侧拾取或设置"
                            }
                            TransformKind::PolarArray => "输入数量后确认；总角度在右侧设置",
                            _ => "在右侧工具页设置参数，Enter/空格提交",
                        }
                    } else {
                        "指定基点 / 中心 / 镜像平面经过的点"
                    }
                )
            }
            None => "就绪：选择绘图 / 变换工具；UCS XY / YZ / ZX / 3P 设置工作平面",
            Some(Step::Select) => "移动/复制 1/3：选择图元，Enter 确认选择；Shift 移除，Esc 取消",
            Some(Step::Base { .. }) => {
                return format!(
                    "{} 2/3：指定基点（画布点选或输入坐标）；Esc 取消",
                    if self.copying { "复制" } else { "移动" }
                )
            }
            Some(Step::Target { .. }) => {
                return format!(
                    "{} 3/3：指定目标点并完成；Esc 取消未确认操作",
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
    pub fn removed_preview_ids(&self) -> Option<&BTreeSet<EntityId>> {
        if let Some(Step::CurvePick(s)) = &self.step {
            return Some(s.affected());
        }
        if let Some(Step::Curves(edit)) = &self.step {
            Some(&edit.remove)
        } else {
            None
        }
    }
    pub fn begin_curves(&mut self, edit: cad_core::CurveEdit) {
        self.cancel();
        self.step = Some(Step::Curves(edit));
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
        if let Some(Step::TransformPick(s)) = &self.step {
            return s.point_plane(&self.tools, self.plane);
        }
        if let Some(Step::CurvePick(s)) = &self.step {
            if s.side() {
                return s.side_plane().unwrap_or(self.plane);
            }
        }
        if let Some(Step::ArraySpacing { base, .. }) = &self.step {
            return WorkPlane {
                origin: as_point(*base),
                ..self.plane
            };
        }
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
    pub fn begin_paste(
        &mut self,
        items: Vec<cad_core::geometry_edit::ClipboardItem>,
        base: [f64; 3],
    ) {
        self.cancel();
        self.step = Some(Step::Paste {
            items,
            base,
            target: None,
        });
    }
    pub fn begin_move(&mut self, ids: &BTreeSet<u32>) {
        self.cancel();
        self.step = Some(if ids.is_empty() {
            Step::Select
        } else {
            Step::Base { ids: ids.clone() }
        });
    }

    pub fn begin_named(&mut self, name: &str, ids: &BTreeSet<u32>) -> bool {
        if let Some(kind) = super::cad_util::Kind::ALL
            .into_iter()
            .find(|k| k.command() == name)
        {
            self.cancel();
            self.step = Some(Step::Utility {
                kind,
                points: vec![],
                confirmed: false,
                ids: ids.clone(),
            });
            return true;
        }
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
            SketchKind::CircleDiameter,
            SketchKind::Circle2P,
            SketchKind::Circle3P,
            SketchKind::Arc,
            SketchKind::ArcCenterEnd,
            SketchKind::ArcCenterAngle,
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
                if matches!(
                    kind,
                    TransformKind::Rotate | TransformKind::Scale | TransformKind::Mirror
                ) {
                    self.step = Some(Step::TransformPick(
                        super::cad_transform_command::Session::new(kind, ids),
                    ));
                    return true;
                }
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
        self.rectangle_dimensions = None;
        self.rectangle_angle = 0.;
        self.rectangle_rotating = false;
        self.copying = false;
        self.copy_single = false;
        self.copy_operations.clear();
        self.aids.clear();
        self.ray_error.clear();
        self.step = None;
        self.hover = None;
        self.pointer_point = None;
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
            Some(Step::TransformPick(s)) => array(s.reference_point().unwrap_or(self.plane.origin)),
            Some(Step::CurvePick(s)) if s.side() => s
                .side_plane()
                .map(|p| array(p.origin))
                .unwrap_or(array(self.plane.origin)),
            Some(Step::Utility { points, .. }) => points
                .last()
                .copied()
                .map(array)
                .unwrap_or(array(self.plane.origin)),
            Some(Step::Paste { base, .. } | Step::ArraySpacing { base, .. }) => *base,
            Some(Step::Grip {
                base,
                kind: GripKind::Radius,
                geometry,
                ..
            }) => array(match geometry {
                Geometry::Circle(c) => c.center,
                Geometry::Arc(a) => a.center,
                _ => *base,
            }),
            Some(Step::Grip { base, .. }) => array(*base),
            Some(Step::Draw { sketch, .. })
                if sketch.kind == SketchKind::ArcCenterEnd && sketch.points.len() == 2 =>
            {
                array(sketch.points[0])
            }
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
            Some(Step::TransformPick(s)) => s.reference_point(),
            Some(Step::Utility { points, .. }) => points.last().copied(),
            Some(Step::ArraySpacing { base, .. } | Step::Paste { base, .. }) => {
                Some(as_point(*base))
            }
            Some(Step::Grip { .. }) => Some(as_point(self.reference())),
            Some(Step::Draw { sketch, .. })
                if sketch.kind == SketchKind::ArcCenterEnd && sketch.points.len() == 2 =>
            {
                Some(sketch.points[0])
            }
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
                self.pointer_point = Some(array(point));
                match self.rectangle_target(array(point)) {
                    Ok(point) => {
                        self.hover = Some(point);
                        self.ray_error.clear();
                    }
                    Err(error) => {
                        self.hover = None;
                        self.ray_error = error;
                    }
                }
            }
        }
    }
    pub fn input(
        &mut self,
        text: &str,
        selected: &BTreeSet<u32>,
    ) -> Result<Option<CommandCommit>, String> {
        if self.rectangle_corner() {
            let upper = text.trim().to_uppercase();
            if matches!(upper.as_str(), "R" | "ROTATION") {
                self.rectangle_rotating = true;
                self.clear_hover();
                self.ray_error.clear();
                return Ok(None);
            }
            if matches!(upper.as_str(), "D" | "DIMENSIONS") {
                self.rectangle_rotating = false;
                self.rectangle_dimensions = Some(RectangleDimensions::Length);
                self.clear_hover();
                self.ray_error.clear();
                return Ok(None);
            }
            if self.rectangle_rotating {
                if !matches!(upper.as_str(), "U" | "UNDO") {
                    let angle = if text.trim().is_empty() {
                        self.rectangle_angle
                    } else {
                        number(text)?
                    }
                    .rem_euclid(360.);
                    let (s, c) = angle.to_radians().sin_cos();
                    if let Some(Step::Draw { sketch, .. }) = &mut self.step {
                        sketch.plane = WorkPlane {
                            origin: self.plane.origin,
                            x_axis: self.plane.x_axis * c + self.plane.y_axis * s,
                            y_axis: self.plane.y_axis * c - self.plane.x_axis * s,
                        };
                    }
                    self.rectangle_angle = angle;
                }
                self.rectangle_rotating = false;
                self.clear_hover();
                self.ray_error.clear();
                return Ok(None);
            }
            if matches!(upper.as_str(), "U" | "UNDO") {
                if let Some(phase) = self.rectangle_dimensions {
                    self.rectangle_dimensions = match phase {
                        RectangleDimensions::Length => None,
                        RectangleDimensions::Width(_) => Some(RectangleDimensions::Length),
                        RectangleDimensions::Place([length, _]) => {
                            Some(RectangleDimensions::Width(length))
                        }
                    };
                    self.clear_hover();
                    self.ray_error.clear();
                    return Ok(None);
                }
            }
            if let Some(phase @ (RectangleDimensions::Length | RectangleDimensions::Width(_))) =
                self.rectangle_dimensions
            {
                let value = number(text)?;
                if value <= 1e-9 {
                    return Err("矩形长度和宽度须为大于几何容差的正数".into());
                }
                self.rectangle_dimensions = Some(match phase {
                    RectangleDimensions::Length => RectangleDimensions::Width(value),
                    RectangleDimensions::Width(length) => {
                        RectangleDimensions::Place([length, value])
                    }
                    _ => unreachable!(),
                });
                self.clear_hover();
                self.ray_error.clear();
                return Ok(None);
            }
            if self.rectangle_placing() && text.trim().is_empty() {
                let point = self.hover.ok_or("请先在画布选择矩形的放置象限")?;
                self.accept_point(point)?;
                return self.finish_ready(selected);
            }
        }
        if self.copy_mode().is_some() && !self.selecting() {
            match text.trim().to_uppercase().as_str() {
                "M" | "MULTIPLE" | "多次" => {
                    self.copy_single = false;
                    return Ok(None);
                }
                "S" | "SINGLE" | "单次" => {
                    self.copy_single = true;
                    return Ok(None);
                }
                _ => {}
            }
            if text.trim().is_empty() && self.copy_waiting_target() {
                self.cancel();
                return Ok(None);
            }
        }
        if let Some(Step::TransformPick(s)) = &mut self.step {
            let point_expression = text.starts_with(['@', '#'])
                || text.contains([',', '，', '<'])
                || text.split_whitespace().count() > 1;
            if s.takes_point() && point_expression {
                s.preview_value("", &mut self.tools);
            }
            if !(s.takes_point()
                && (point_expression
                    || s.point_only(&self.tools)
                        && !text.trim().is_empty()
                        && number(text).is_ok()))
            {
                if text.trim().is_empty() && s.point_only(&self.tools) {
                    let point = self.hover.ok_or("请在画布指定点或输入坐标")?;
                    s.accept_point(as_point(point), &mut self.tools, self.plane)?;
                    let ready = s.ready();
                    self.hover = None;
                    self.pointer_point = None;
                    self.aids.clear();
                    return if ready {
                        self.finish_ready(selected)
                    } else {
                        Ok(None)
                    };
                }
                return s
                    .input(
                        text,
                        selected,
                        &mut self.tools,
                        self.plane,
                        self.hover.map(as_point),
                    )
                    .map(|c| {
                        c.map(|c| CommandCommit::Transform {
                            ids: c.ids,
                            transforms: vec![c.transform],
                            copies: c.copies,
                        })
                    });
            }
        }
        if let Some(Step::CurvePick(s)) = &mut self.step {
            if !s.side() {
                let result = s
                    .input(text, selected)
                    .map(|edit| edit.map(CommandCommit::Curves));
                if s.finished() {
                    self.cancel();
                }
                return result;
            }
            if s.can_change_distance()
                && matches!(
                    text.trim().to_uppercase().as_str(),
                    "D" | "DISTANCE" | "N" | "NEXT"
                )
            {
                let result = s
                    .input(text, selected)
                    .map(|edit| edit.map(CommandCommit::Curves));
                if result.is_ok() {
                    self.hover = None;
                    self.pointer_point = None;
                    self.aids.clear();
                    self.ray_error.clear();
                }
                return result;
            }
        }
        if matches!(
            &self.step,
            Some(Step::Grip {
                kind: GripKind::Radius,
                ..
            })
        ) && !text.trim().is_empty()
            && !text.contains([',', ' ', '<'])
            && !text.starts_with(['@', '#'])
        {
            let point = self.radius_grip_point(text)?;
            self.accept_point(point)?;
            return self.finish_ready(selected);
        }
        if matches!(self.step, Some(Step::PlaneSelect)) {
            let name = text.trim().to_uppercase();
            let name = if name.is_empty() { "XY" } else { name.as_str() };
            if !matches!(name, "XY" | "YZ" | "ZX" | "3P") {
                return Err("工作平面选项为 XY / YZ / ZX / 3P".into());
            }
            self.step = None;
            self.set_preset(name)?;
            return Ok(None);
        }
        if matches!(self.step, Some(Step::EraseSelect)) {
            if !text.trim().is_empty() || selected.is_empty() {
                return Err("请选择要删除的对象后确认".into());
            }
            return Ok(Some(CommandCommit::Delete(selected.clone())));
        }
        if let Some(Step::Utility {
            kind,
            points,
            confirmed,
            ids,
        }) = &mut self.step
        {
            if *kind == super::cad_util::Kind::CopyBase && ids.is_empty() {
                if !text.trim().is_empty() || selected.is_empty() {
                    return Err("请先选择图元再 Enter".into());
                }
                *ids = selected.clone();
                return Ok(None);
            }
            if *confirmed && text.trim().is_empty() {
                if kind.measuring() {
                    return Ok(Some(CommandCommit::Report(super::cad_util::result(
                        *kind, points, self.plane,
                    )?)));
                }
                if *kind == super::cad_util::Kind::CopyBase {
                    return Ok(Some(CommandCommit::Clipboard {
                        ids: ids.clone(),
                        base: array(points[0]),
                    }));
                }
                let geometry = self.utility.geometry(*kind, points, self.plane)?;
                return Ok(Some(if *kind == super::cad_util::Kind::Dimension {
                    CommandCommit::Annotation(geometry)
                } else {
                    CommandCommit::Create {
                        geometry,
                        normal: self.plane.normal(),
                    }
                }));
            }
        }
        if let Some(Step::Paste {
            items,
            base,
            target: Some(target),
        }) = &self.step
        {
            if text.trim().is_empty() {
                return Ok(Some(CommandCommit::Paste {
                    items: items.clone(),
                    offset: std::array::from_fn(|i| target[i] - base[i]),
                }));
            }
        }

        if let Some(Step::Curves(edit)) = &self.step {
            if text.trim().is_empty() {
                return Ok(Some(CommandCommit::Curves(edit.clone())));
            }
            return Err("曲线预览只接受 Enter 提交或 Esc 取消".into());
        }
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
            if matches!(
                kind,
                TransformKind::Rotate
                    | TransformKind::Scale
                    | TransformKind::RectArray
                    | TransformKind::PolarArray
            ) {
                update_transform(&mut self.tools, *kind, text)?;
                let (transforms, copies) =
                    self.tools.transforms(*kind, *base, self.plane, ids.len())?;
                return Ok(Some(CommandCommit::Transform {
                    ids: ids.clone(),
                    transforms,
                    copies,
                }));
            }
        }
        if let Some(Step::Draw { sketch, confirmed }) = &mut self.step {
            let upper = text.trim().to_uppercase();
            if sketch.kind.is_arc()
                && matches!(
                    upper.as_str(),
                    "3P" | "C" | "CENTER" | "A" | "ANGLE" | "E" | "END"
                )
            {
                let kind = match upper.as_str() {
                    "3P" => SketchKind::Arc,
                    "A" | "ANGLE" => SketchKind::ArcCenterAngle,
                    _ => SketchKind::ArcCenterEnd,
                };
                if !sketch.points.is_empty()
                    && (sketch.kind == SketchKind::Arc || kind == SketchKind::Arc || *confirmed)
                {
                    return Err("三点/圆心方式须在首点前选择；可 U 撤回后切换".into());
                }
                sketch.kind = kind;
                sketch.sweep_degrees = None;
                self.clear_hover();
                self.ray_error.clear();
                return Ok(None);
            }
            if sketch.kind.is_circle()
                && matches!(
                    upper.as_str(),
                    "R" | "RADIUS" | "D" | "DIAMETER" | "2P" | "3P"
                )
            {
                let kind = match upper.as_str() {
                    "R" | "RADIUS" => SketchKind::Circle,
                    "D" | "DIAMETER" => SketchKind::CircleDiameter,
                    "2P" => SketchKind::Circle2P,
                    _ => SketchKind::Circle3P,
                };
                if !sketch.points.is_empty()
                    && (!matches!(sketch.kind, SketchKind::Circle | SketchKind::CircleDiameter)
                        || !matches!(kind, SketchKind::Circle | SketchKind::CircleDiameter)
                        || *confirmed)
                {
                    return Err("两点/三点方式须在首点前选择；可用 U 撤回取点后切换".into());
                }
                sketch.kind = kind;
                self.hover = None;
                self.pointer_point = None;
                self.aids.clear();
                self.ray_error.clear();
                return Ok(None);
            }
            if matches!(upper.as_str(), "U" | "UNDO") {
                sketch.points.pop().ok_or("没有可撤回的点")?;
                sketch.sweep_degrees = None;
                if sketch.kind == SketchKind::Rectangle && sketch.points.is_empty() {
                    sketch.plane = self.plane;
                    self.rectangle_angle = 0.;
                    self.rectangle_rotating = false;
                }
                sketch.closed = false;
                *confirmed = false;
                return Ok(None);
            }
            if sketch.kind == SketchKind::ArcCenterAngle && sketch.points.len() == 2 {
                let mut next = sketch.clone();
                next.sweep_degrees = Some(if text.trim().is_empty() {
                    sketch.sweep_degrees.unwrap_or(90.)
                } else {
                    number(text)?
                });
                let geometry = next.geometry()?;
                *sketch = next;
                *confirmed = true;
                return Ok(Some(CommandCommit::Create {
                    geometry,
                    normal: sketch.plane.normal(),
                }));
            }
            if text.trim().is_empty() {
                if sketch.points.len()
                    < if sketch.kind == SketchKind::Spline {
                        sketch.degree as usize + 1
                    } else {
                        sketch.kind.required()
                    }
                {
                    self.cancel();
                    return Ok(None);
                }
                let geometry = sketch.geometry()?;
                return Ok(Some(CommandCommit::Create {
                    geometry,
                    normal: sketch.plane.normal(),
                }));
            }
            if matches!(upper.as_str(), "C" | "CLOSE" | "闭合") {
                if !sketch.kind.continuous() {
                    return Err("该工具不支持闭合选项".into());
                }
                let mut next = sketch.clone();
                next.closed = true;
                let geometry = next.geometry()?;
                *sketch = next;
                *confirmed = true;
                return Ok(Some(CommandCommit::Create {
                    geometry,
                    normal: sketch.plane.normal(),
                }));
            }
            if !text.contains([',', ' ', '<'])
                && !text.starts_with(['@', '#'])
                && ((matches!(sketch.kind, SketchKind::Circle | SketchKind::CircleDiameter)
                    && sketch.points.len() >= 1)
                    || (sketch.kind == SketchKind::Ellipse && sketch.points.len() >= 2))
            {
                let radius = number(text)?;
                if radius <= 1e-9 {
                    return Err("半径/半轴长度必须大于零".into());
                }
                let c = sketch.points[0];
                let direction =
                    if matches!(sketch.kind, SketchKind::Circle | SketchKind::CircleDiameter) {
                        sketch.plane.x_axis
                    } else {
                        let a = sketch.points[1];
                        sketch.plane.normal().cross(
                            cad_core::model::Vector3::new(a.x - c.x, a.y - c.y, a.z - c.z)
                                .normalize(),
                        )
                    };
                let p = [
                    c.x + direction.x * radius,
                    c.y + direction.y * radius,
                    c.z + direction.z * radius,
                ];
                self.accept_point(p)?;
                return self.finish_ready(selected);
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
            let point = if !text.contains([',', '，', '<', ' ']) && !text.starts_with(['@', '#']) {
                let distance = number(text)?;
                let base = self.snap_reference().ok_or("请指定完整坐标")?;
                let target = self.hover.ok_or("请先用鼠标指定方向，或输入完整坐标")?;
                let delta = cad_core::model::Vector3::new(
                    target[0] - base.x,
                    target[1] - base.y,
                    target[2] - base.z,
                );
                if delta.length() <= 1e-9 {
                    return Err("请先用鼠标指定非零方向".into());
                }
                let delta = delta.normalize() * distance;
                finite([base.x + delta.x, base.y + delta.y, base.z + delta.z])?
            } else {
                parse_point(text, self.point_plane(), self.reference())?
            };
            self.accept_point(point)?;
        }
        self.finish_ready(selected)
    }
    fn accept_point(&mut self, point: [f64; 3]) -> Result<(), String> {
        let point = self.rectangle_target(point)?;
        self.step = match self.step.as_ref().ok_or("没有正在取点的命令")? {
            Step::TransformPick(_) => {
                if let Some(Step::TransformPick(s)) = &mut self.step {
                    s.accept_point(as_point(point), &mut self.tools, self.plane)?;
                }
                self.hover = None;
                self.pointer_point = None;
                self.aids.clear();
                self.error.clear();
                self.ray_error.clear();
                return Ok(());
            }
            Step::CurvePick(_) => {
                if let Some(Step::CurvePick(s)) = &mut self.step {
                    s.preview_side(as_point(point), true)?;
                }
                self.hover = None;
                self.pointer_point = None;
                self.aids.clear();
                self.ray_error.clear();
                self.error.clear();
                return Ok(());
            }
            Step::Utility {
                kind,
                points,
                confirmed,
                ids,
            } => {
                let mut points = points.clone();
                if *confirmed {
                    points.pop();
                }
                points.push(as_point(point));
                let confirmed = points.len() >= kind.required();
                if confirmed {
                    if kind.measuring() {
                        super::cad_util::result(*kind, &points, self.plane)?;
                    } else {
                        self.utility.geometry(*kind, &points, self.plane)?;
                    }
                }
                Some(Step::Utility {
                    kind: *kind,
                    points,
                    confirmed,
                    ids: ids.clone(),
                })
            }
            Step::Paste { items, base, .. } => Some(Step::Paste {
                items: items.clone(),
                base: *base,
                target: Some(point),
            }),
            Step::ArraySpacing { ids, base } => {
                let a = self.plane.to_local(as_point(*base));
                let b = self.plane.to_local(as_point(point));
                self.tools.spacing = [b[0] - a[0], b[1] - a[1]];
                self.tools
                    .transforms(TransformKind::RectArray, *base, self.plane, ids.len())?;
                Some(Step::Transform {
                    kind: TransformKind::RectArray,
                    ids: ids.clone(),
                    base: Some(*base),
                })
            }
            Step::Curves(_) => return Err("当前为曲线整理确认步骤".into()),
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
                    // Center/angle arcs wait for a separate scalar confirmation.
                    || !next.kind.continuous() && next.kind!=SketchKind::ArcCenterAngle && next.points.len() >= next.kind.required();
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
            Step::Select | Step::EraseSelect | Step::PlaneSelect => {
                return Err("请先确认选择或工作平面选项".into())
            }
        };
        self.hover = None;
        self.aids.clear();
        self.pointer_point = None;
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
        dynamic: &super::cad_input::DynamicInput,
        draft: &str,
        click: bool,
    ) -> bool {
        if !self.takes_point() {
            return false;
        }
        if let Some(Step::TransformPick(s)) = &mut self.step {
            s.preview_value(draft, &mut self.tools);
        }
        let reference = self.snap_reference();
        let plane = self.point_plane();
        self.pointer_point = None;
        let point = frame
            .ray(camera, transform, cursor)
            .ok_or_else(|| "当前相机无法取点".to_string())
            .and_then(|ray| {
                self.aids.resolve(
                    document, frame, camera, transform, cursor, ray, plane, reference,
                )
            })
            .map(array)
            .and_then(finite)
            .inspect(|point| self.pointer_point = Some(*point))
            .and_then(|point| dynamic.constrain(self, draft, point))
            .and_then(|point| self.circle_value_point(draft).unwrap_or(Ok(point)))
            .and_then(|point| self.rectangle_target(point))
            .and_then(finite);
        match point {
            Ok(point) => {
                if let Some(Step::CurvePick(s)) = &mut self.step {
                    if s.side() {
                        if let Err(error) = s.preview_side(as_point(point), false) {
                            self.ray_error = error;
                            self.hover = Some(point);
                            return false;
                        }
                    }
                }
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
        self.pointer_point = None;
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
        if let Some(Step::CurvePick(s)) = &self.step {
            if let Some(edit) = s.preview() {
                return edit
                    .replacements
                    .values()
                    .flatten()
                    .flat_map(super::display::tessellate_geometry)
                    .collect();
            }
            return s
                .first_geometry()
                .map(super::display::tessellate_geometry)
                .unwrap_or_default();
        }
        if let Some(Step::Paste {
            items,
            base,
            target,
        }) = &self.step
        {
            if let Some(target) = target.or(self.hover) {
                let d = std::array::from_fn(|i| target[i] - base[i]);
                return items
                    .iter()
                    .filter_map(|i| cad_core::drafting::translated(&i.geometry, d).ok())
                    .flat_map(|g| super::display::tessellate_geometry(&g))
                    .collect();
            }
        }
        if let Some(Step::Utility {
            kind,
            points,
            confirmed,
            ..
        }) = &self.step
        {
            let mut points = points.clone();
            if !confirmed {
                if let Some(p) = self.hover {
                    points.push(as_point(p));
                }
            }
            return self
                .utility
                .geometry(*kind, &points, self.plane)
                .ok()
                .filter(|g| !g.is_empty())
                .map(|g| {
                    g.iter()
                        .flat_map(super::display::tessellate_geometry)
                        .collect()
                })
                .unwrap_or_else(|| vec![points.into_iter().map(array).collect()]);
        }
        if let Some(Step::Curves(edit)) = &self.step {
            return edit
                .replacements
                .values()
                .flatten()
                .flat_map(super::display::tessellate_geometry)
                .collect();
        }

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
        if sketch.kind == SketchKind::ArcCenterAngle && sketch.sweep_degrees.is_none() {
            sketch.sweep_degrees = Some(90.);
        }
        match sketch.geometry() {
            Ok(geometry) => geometry
                .iter()
                .flat_map(super::display::tessellate_geometry)
                .collect(),
            Err(_) => vec![sketch.points.iter().copied().map(array).collect()],
        }
    }
    pub fn utility_preview_geometry(&self) -> Vec<Geometry> {
        if let Some(Step::Utility {
            kind,
            points,
            confirmed,
            ..
        }) = &self.step
        {
            let mut points = points.clone();
            if !confirmed {
                if let Some(p) = self.hover {
                    points.push(as_point(p));
                }
            }
            return self
                .utility
                .geometry(*kind, &points, self.plane)
                .unwrap_or_default();
        }
        if let Some(Step::Paste {
            items,
            base,
            target,
        }) = &self.step
        {
            if let Some(target) = target.or(self.hover) {
                return items
                    .iter()
                    .filter_map(|i| {
                        cad_core::drafting::translated(
                            &i.geometry,
                            std::array::from_fn(|n| target[n] - base[n]),
                        )
                        .ok()
                    })
                    .collect();
            }
        }
        Vec::new()
    }
    pub fn transform_preview(&self) -> Option<(BTreeSet<u32>, Vec<AffineTransform>)> {
        if let Some(Step::TransformPick(s)) = &self.step {
            let c = s.preview(&self.tools, self.plane, self.hover.map(as_point))?;
            return Some((c.ids, vec![c.transform]));
        }
        if let Some(Step::ArraySpacing { ids, base }) = &self.step {
            let point = self.hover?;
            let a = self.plane.to_local(as_point(*base));
            let b = self.plane.to_local(as_point(point));
            let mut tools = self.tools.clone();
            tools.spacing = [b[0] - a[0], b[1] - a[1]];
            return tools
                .transforms(TransformKind::RectArray, *base, self.plane, ids.len())
                .ok()
                .map(|(t, _)| (ids.clone(), t));
        }
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
        ui.strong("命令参数与工作平面");
        self.aids.controls(ui);
        if self.rectangle_corner() && ui.button("D 矩形长度 / 宽度").clicked() {
            command = Some("D".into());
        }
        if self.rectangle_corner() && ui.button("R 矩形旋转角度").clicked() {
            command = Some("R".into());
        }
        if self.copy_mode().is_some() && !self.selecting() {
            ui.horizontal(|ui| {
                if ui.button("M 多次复制").clicked() {
                    command = Some("M".into());
                }
                if ui.button("S 单次复制").clicked() {
                    command = Some("S".into());
                }
            });
        }
        if let Some(Step::TransformPick(s)) = &self.step {
            ui.add_enabled_ui(!s.mirror_question(), |ui| {
                self.tools.advanced_transform_ui(ui, s.kind)
            });
            if s.can_reference() && ui.button("R 参照方向 / 长度").clicked() {
                command = Some("R".into());
            }
            if s.mirror_question() {
                ui.horizontal(|ui| {
                    if ui.button("N 保留源对象").clicked() {
                        command = Some("N".into());
                    }
                    if ui.button("Y 替换源对象").clicked() {
                        command = Some("Y".into());
                    }
                });
            }
        }
        ui.add_enabled_ui(!self.is_active(), |ui| self.tools.style_ui(ui, document));
        if let Some(Step::Transform { kind, ids, base }) = &self.step {
            self.tools.transform_ui(ui, *kind);
            if let Some(base) = base {
                if let Err(error) = self.tools.transforms(*kind, *base, self.plane, ids.len()) {
                    ui.colored_label(egui::Color32::LIGHT_RED, error);
                }
            }
        }
        if let Some(Step::Utility {
            kind,
            points,
            confirmed,
            ..
        }) = &self.step
        {
            self.utility.ui(ui, *kind);
            if *confirmed && kind.measuring() {
                if let Ok(result) = super::cad_util::result(*kind, points, self.plane) {
                    ui.label(result);
                }
            }
        }
        if !self.last_measurement.is_empty() {
            ui.label(&self.last_measurement);
        }
        ui.label(self.prompt());
        if let Some(Step::Grip { base, .. }) = &self.step {
            ui.small(format!(
                "夹点临时平面原点：{:.4}, {:.4}, {:.4}；U/V 沿图元平面，三维输入仍为 WCS。",
                base.x, base.y, base.z
            ));
        }
        if let Some(Step::Transform {
            kind: TransformKind::RectArray,
            ids,
            base: Some(base),
        }) = &self.step
        {
            let ids = ids.clone();
            let base = *base;
            if ui.button("拾取列/行间距").clicked() {
                self.step = Some(Step::ArraySpacing { ids, base });
            }
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
                        Step::Utility{confirmed:true,..} | Step::Paste{target:Some(_),..} | Step::Curves(_) | Step::Grip { target: Some(_), .. } | Step::Confirm { .. }
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
            Some(Step::TransformPick(s)) if s.takes_point() => s
                .reference_point()
                .zip(self.hover.map(as_point))
                .map(|(a, b)| (array(a), array(b))),
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
fn update_transform(
    tools: &mut ToolOptions,
    kind: TransformKind,
    text: &str,
) -> Result<(), String> {
    let count = |text: &str| -> Result<u32, String> {
        let value = number(text)?;
        if !(1.0..=1000.0).contains(&value) || value.fract() != 0. {
            return Err("阵列数量须为 1～1000 的整数".into());
        }
        Ok(value as u32)
    };
    match kind {
        TransformKind::Rotate => tools.angle = number(text)?,
        TransformKind::Scale => tools.scale = number(text)?,
        TransformKind::PolarArray => tools.count = count(text)?,
        TransformKind::RectArray => {
            let normalized = text.replace('，', ",");
            let values: Vec<_> = normalized.split(',').collect();
            let [column, row] = values.as_slice() else {
                return Err("请输入列数,行数".into());
            };
            let columns = count(column)?;
            let rows = count(row)?;
            tools.columns = columns;
            tools.rows = rows;
        }
        _ => return Err("该变换请在工具页设置参数".into()),
    }
    Ok(())
}

fn array(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn parse_point(text: &str, plane: WorkPlane, reference: [f64; 3]) -> Result<[f64; 3], String> {
    let text = text.trim();
    if text == "@" {
        return finite(reference);
    }
    let relative = text.starts_with('@');
    let text = text.strip_prefix('@').or_else(|| text.strip_prefix('#')).unwrap_or(text);
    if let Some((distance, angle)) = text.split_once('<') {
        let distance = number(distance)?;
        let angle = number(angle)?.to_radians();
        let offset =
            plane.x_axis * (distance * angle.cos()) + plane.y_axis * (distance * angle.sin());
        let origin = if relative { reference } else { array(plane.origin) };
        return finite([
            origin[0] + offset.x,
            origin[1] + offset.y,
            origin[2] + offset.z,
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
