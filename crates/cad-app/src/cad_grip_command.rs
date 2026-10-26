//! Grip modes share one immutable source and one coordinate-to-result path.
use std::collections::{BTreeMap, BTreeSet};

use cad_core::geometry_edit::AffineTransform;
use cad_core::model::{EntityId, Geometry, Plane, Point3, Vector3, WorkPlane};
use cad_core::properties::{self, GripKind};

use super::super::cad_input::{Kind, Spec};
use super::{array, as_point, finite, is_point_expression, number, parse_point, CommandCommit};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Stretch,
    Move,
}

pub struct Session {
    pub id: EntityId,
    pub kind: GripKind,
    geometry: Geometry,
    label: String,
    grip: Point3,
    base: Point3,
    plane: WorkPlane,
    mode: Mode,
    copies: bool,
    picking_base: bool,
    length: bool,
    target: Option<Point3>,
}

impl Session {
    pub fn new(
        id: EntityId,
        geometry: Geometry,
        kind: GripKind,
        target: Option<Point3>,
        plane: WorkPlane,
    ) -> Result<Self, String> {
        let grip = properties::grips(&geometry)
            .into_iter()
            .find(|g| g.kind == kind)
            .ok_or("夹点已失效")?;
        let own_plane = match &geometry {
            Geometry::Circle(c) => Some(c.plane),
            Geometry::Arc(a) => Some(a.plane),
            Geometry::Polyline(p) => p.plane,
            Geometry::Ellipse(e) => {
                Some(Plane::new(e.major_axis.cross(e.minor_axis), e.major_axis))
            }
            _ => None,
        };
        let session = Self {
            id,
            kind,
            mode: if kind == GripKind::Center
                || matches!((&geometry, kind), (Geometry::Line(_), GripKind::Midpoint))
            {
                Mode::Move
            } else {
                Mode::Stretch
            },
            geometry,
            label: grip
                .label
                .split('：')
                .next()
                .unwrap_or(&grip.label)
                .to_string(),
            grip: grip.point,
            base: grip.point,
            plane: WorkPlane {
                origin: grip.point,
                x_axis: own_plane.map_or(plane.x_axis, |p| p.x_axis),
                y_axis: own_plane.map_or(plane.y_axis, |p| p.y_axis()),
            },
            copies: false,
            picking_base: false,
            length: false,
            target,
        };
        if let Some(point) = target {
            session.geometry(point)?;
        }
        Ok(session)
    }
    pub fn ready(&self) -> bool {
        !self.picking_base && self.target.is_some()
    }
    pub fn confirmation_label(&self) -> &'static str {
        if self.picking_base {
            "确认新基点  Enter"
        } else if self.ready() {
            "提交固定预览  Enter"
        } else {
            "切换拉伸 / 移动  Enter / 空格"
        }
    }
    pub fn takes_point(&self) -> bool {
        self.target.is_none()
    }
    pub fn plane(&self) -> WorkPlane {
        WorkPlane {
            origin: self.base,
            ..self.plane
        }
    }
    fn whole_move(&self) -> bool {
        self.mode == Mode::Move
            || self.kind == GripKind::Center
            || matches!(
                (&self.geometry, self.kind),
                (Geometry::Line(_), GripKind::Midpoint)
            )
    }
    fn radius(&self) -> Option<Point3> {
        if self.mode == Mode::Stretch && !self.picking_base {
            if let (Geometry::Circle(c), GripKind::Radius(_)) = (&self.geometry, self.kind) {
                return Some(c.center);
            }
        }
        None
    }
    pub fn can_length(&self) -> bool {
        matches!(
            (&self.geometry, self.kind),
            (Geometry::Line(_), GripKind::Start | GripKind::End)
        )
    }
    fn opposite(&self) -> Point3 {
        let Geometry::Line(line) = &self.geometry else {
            unreachable!()
        };
        if self.kind == GripKind::Start {
            line.end
        } else {
            line.start
        }
    }
    fn command_point(&self, point: Point3) -> Point3 {
        Point3::new(
            self.base.x + point.x - self.grip.x,
            self.base.y + point.y - self.grip.y,
            self.base.z + point.z - self.grip.z,
        )
    }
    fn grip_point(&self, point: Point3) -> Point3 {
        Point3::new(
            self.grip.x + point.x - self.base.x,
            self.grip.y + point.y - self.base.y,
            self.grip.z + point.z - self.base.z,
        )
    }
    pub fn reference(&self) -> Point3 {
        if self.picking_base {
            self.base
        } else if self.length {
            self.command_point(self.opposite())
        } else if let Some(center) = self.radius() {
            self.command_point(center)
        } else {
            self.base
        }
    }
    pub fn spec(&self, hover: Option<[f64; 3]>) -> Spec {
        let point = self.target.map(array).or(hover).unwrap_or(array(self.base));
        Spec {
            kind: if self.picking_base {
                Kind::Coordinates
            } else if self.length {
                Kind::TargetLength
            } else if self.radius().is_some() {
                Kind::Radius
            } else {
                Kind::GripPolar
            },
            reference: Some(array(self.reference())),
            plane: self.plane(),
            point: Some(point),
            value: cad_core::curves::distance(self.reference(), as_point(point)),
            secondary: 0.,
        }
    }
    pub fn option(text: &str) -> bool {
        matches!(
            text.trim().to_uppercase().as_str(),
            "S" | "STRETCH"
                | "M"
                | "MOVE"
                | "C"
                | "COPY"
                | "B"
                | "BASE"
                | "L"
                | "LENGTH"
                | "D"
                | "DISPLACEMENT"
                | "GRIPAPPLY"
        )
    }
    pub fn prompt(&self) -> String {
        let mode = if self.mode == Mode::Move {
            "整体移动"
        } else if self.length {
            "端点最终长度"
        } else if self.radius().is_some() {
            "修改半径"
        } else {
            "拉伸"
        };
        let action = if self.picking_base {
            "指定新基点；仅更新基点，不修改图纸"
        } else if self.ready() {
            "Enter 提交固定预览，或点击重选目标"
        } else {
            "点击目标/输入确认；空 Enter/空格切换拉伸/移动"
        };
        format!(
            "{} · {mode}{}：{action}；S 拉伸/M 移动/C 复制/B 基点{}；Esc 取消",
            self.label,
            if self.copies {
                "（创建副本）"
            } else {
                ""
            },
            if self.can_length() {
                "/L 最终长度/D 位移"
            } else {
                ""
            }
        )
    }
    pub fn resolve(&self, text: &str, point: [f64; 3]) -> Result<[f64; 3], String> {
        let text = text.trim();
        if text.is_empty() {
            return finite(point);
        }
        if Self::option(text) {
            return Err("夹点选项请按 Enter 确认".into());
        }
        if is_point_expression(text) {
            return parse_point(text, self.plane(), array(self.reference()));
        }
        let value = number(text)?;
        let reference = self.reference();
        let mut direction = Vector3::new(
            point[0] - reference.x,
            point[1] - reference.y,
            point[2] - reference.z,
        );
        if self.radius().is_some() || self.length && direction.length() <= 1e-9 {
            let original = if let Some(center) = self.radius() {
                center
            } else {
                self.opposite()
            };
            direction = Vector3::new(
                self.grip.x - original.x,
                self.grip.y - original.y,
                self.grip.z - original.z,
            );
        }
        if (self.radius().is_some() || self.length) && value <= 1e-9 {
            return Err("半径或最终长度须大于零".into());
        }
        if direction.length() <= 1e-9 {
            return Err("请先用鼠标指定非零方向，或输入完整坐标".into());
        }
        let delta = direction.normalize() * value;
        finite([
            reference.x + delta.x,
            reference.y + delta.y,
            reference.z + delta.z,
        ])
    }
    fn geometry(&self, point: Point3) -> Result<Geometry, String> {
        if self.whole_move() {
            cad_core::drafting::translated(
                &self.geometry,
                [
                    point.x - self.base.x,
                    point.y - self.base.y,
                    point.z - self.base.z,
                ],
            )
        } else {
            properties::move_grip(&self.geometry, self.kind, self.grip_point(point))
        }
    }
    pub fn preview(&self, hover: Option<[f64; 3]>) -> Option<Geometry> {
        if self.picking_base {
            return None;
        }
        self.target
            .or(hover.map(as_point))
            .and_then(|point| self.geometry(point).ok())
    }
    pub fn marker(&self, hover: Option<[f64; 3]>) -> [f64; 3] {
        if self.picking_base {
            array(self.grip)
        } else {
            array(self.grip_point(self.target.or(hover.map(as_point)).unwrap_or(self.base)))
        }
    }
    pub fn accept(&mut self, point: Point3) -> Result<(), String> {
        if self.picking_base {
            self.base = point;
            self.picking_base = false;
        } else {
            self.geometry(point)?;
            self.target = Some(point);
        }
        Ok(())
    }
    pub fn preview_point(&mut self, point: Point3) -> Result<(), String> {
        if !self.picking_base {
            self.geometry(point)?;
        }
        if self.target.is_some() {
            self.target = Some(point);
        }
        Ok(())
    }
    fn commit(&self) -> Result<CommandCommit, String> {
        let point = self.target.ok_or("请先指定目标")?;
        let geometry = self.geometry(point)?;
        if self.whole_move() {
            Ok(CommandCommit::Transform {
                ids: BTreeSet::from([self.id.0]),
                transforms: vec![AffineTransform::translation([
                    point.x - self.base.x,
                    point.y - self.base.y,
                    point.z - self.base.z,
                ])?],
                copies: self.copies,
            })
        } else if self.copies {
            Ok(CommandCommit::Curves(cad_core::CurveEdit {
                replacements: BTreeMap::from([(self.id, vec![geometry])]),
                copies: true,
                ..Default::default()
            }))
        } else {
            Ok(CommandCommit::Geometry(BTreeMap::from([(
                self.id, geometry,
            )])))
        }
    }
    pub fn input(
        &mut self,
        text: &str,
        hover: Option<[f64; 3]>,
    ) -> Result<Option<CommandCommit>, String> {
        let upper = text.trim().to_uppercase();
        if upper == "GRIPAPPLY" {
            if !self.ready() {
                self.accept(as_point(hover.ok_or("请先指定有效目标")?))?;
            }
            return if self.ready() {
                self.commit().map(Some)
            } else {
                Ok(None)
            };
        }
        if upper.is_empty() && self.ready() {
            return self.commit().map(Some);
        }
        if upper.is_empty() && !self.picking_base {
            self.mode = if self.mode == Mode::Move {
                Mode::Stretch
            } else {
                Mode::Move
            };
            self.length = false;
            return Ok(None);
        }
        if Self::option(text) {
            match upper.as_str() {
                "S" | "STRETCH" => {
                    self.mode = Mode::Stretch;
                    self.length = false;
                    self.picking_base = false;
                }
                "M" | "MOVE" => {
                    self.mode = Mode::Move;
                    self.length = false;
                    self.picking_base = false;
                }
                "C" | "COPY" => self.copies = !self.copies,
                "B" | "BASE" => self.picking_base = true,
                "L" | "LENGTH" => {
                    if !self.can_length() {
                        return Err("最终长度仅适用于直线端点".into());
                    }
                    self.mode = Mode::Stretch;
                    self.length = true;
                    self.picking_base = false;
                }
                "D" | "DISPLACEMENT" => {
                    if !self.can_length() {
                        return Err("最终长度/位移切换仅适用于直线端点".into());
                    }
                    self.mode = Mode::Stretch;
                    self.length = false;
                    self.picking_base = false;
                }
                _ => unreachable!(),
            }
            self.target = None;
            return Ok(None);
        }
        let point = self.resolve(
            text,
            hover.or(self.target.map(array)).unwrap_or(array(self.base)),
        )?;
        self.accept(as_point(point))?;
        if self.ready() {
            self.commit().map(Some)
        } else {
            Ok(None)
        }
    }
}
