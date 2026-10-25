//! Point-driven transform steps. Geometry is only submitted by the existing native transaction.
use super::{
    cad_input::{Kind, Spec},
    cad_tools::{ToolOptions, TransformKind},
};
use cad_core::{
    geometry_edit::AffineTransform,
    model::{Point3, Vector3, WorkPlane},
};
use std::collections::BTreeSet;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Select,
    Base,
    Value,
    RefStart,
    RefEnd,
    RefValue,
    MirrorAxis,
    MirrorConfirm,
}
#[derive(Clone)]
pub struct Commit {
    pub ids: BTreeSet<u32>,
    pub transform: AffineTransform,
    pub copies: bool,
}
pub struct Session {
    pub kind: TransformKind,
    ids: BTreeSet<u32>,
    phase: Phase,
    base: Option<Point3>,
    ref_first: Option<Point3>,
    reference: Option<f64>,
    mirror_normal: Option<Vector3>,
    typed: Option<f64>,
    pending: Option<Commit>,
}
fn array(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}
fn vector(a: Point3, b: Point3) -> Vector3 {
    Vector3::new(b.x - a.x, b.y - a.y, b.z - a.z)
}
fn number(text: &str) -> Result<f64, String> {
    text.trim()
        .parse::<f64>()
        .ok()
        .filter(|v| v.is_finite())
        .ok_or("请输入有限数值".into())
}
impl Session {
    pub fn new(kind: TransformKind, ids: &BTreeSet<u32>) -> Self {
        Self {
            kind,
            ids: ids.clone(),
            phase: if ids.is_empty() {
                Phase::Select
            } else {
                Phase::Base
            },
            base: None,
            ref_first: None,
            reference: None,
            mirror_normal: None,
            typed: None,
            pending: None,
        }
    }
    pub fn selecting(&self) -> bool {
        self.phase == Phase::Select
    }
    pub fn ready(&self) -> bool {
        self.pending.is_some()
    }
    pub fn mirror_question(&self) -> bool {
        self.phase == Phase::MirrorConfirm
    }
    pub fn can_reference(&self) -> bool {
        self.base.is_some() && self.kind != TransformKind::Mirror
    }
    pub fn reference_point(&self) -> Option<Point3> {
        if self.phase == Phase::RefEnd {
            self.ref_first
        } else {
            self.base
        }
    }
    pub fn takes_point(&self) -> bool {
        self.pending.is_none()
            && match self.phase {
                Phase::Base
                | Phase::RefStart
                | Phase::RefEnd
                | Phase::RefValue
                | Phase::MirrorAxis => true,
                Phase::Value => self.kind == TransformKind::Rotate,
                _ => false,
            }
    }
    pub fn point_only(&self, tools: &ToolOptions) -> bool {
        self.pending.is_none()
            && (matches!(self.phase, Phase::Base | Phase::RefEnd)
                || self.phase == Phase::MirrorAxis && !tools.custom_axis && !tools.mirror_plane)
    }
    fn axes(tools: &ToolOptions, plane: WorkPlane) -> Result<(Vector3, Vector3, Vector3), String> {
        let n = if tools.custom_axis {
            Vector3::new(tools.axis[0], tools.axis[1], tools.axis[2])
        } else {
            plane.normal()
        };
        if !n.length().is_finite() || n.length() < 1e-9 {
            return Err("自定义旋转轴须有限且非零".into());
        }
        let n = n.normalize();
        let mut x = plane.x_axis - n * plane.x_axis.dot(n);
        if x.length() < 1e-9 {
            x = plane.y_axis - n * plane.y_axis.dot(n);
        }
        let x = x.normalize();
        Ok((x, n.cross(x), n))
    }
    pub fn point_plane(&self, tools: &ToolOptions, plane: WorkPlane) -> WorkPlane {
        let through = self.reference_point().unwrap_or(plane.origin);
        if self.kind == TransformKind::Rotate && self.base.is_some() {
            if let Ok((x, y, n)) = Self::axes(tools, plane) {
                let d = vector(plane.origin, through).dot(n);
                let origin = Point3::new(
                    plane.origin.x + n.x * d,
                    plane.origin.y + n.y * d,
                    plane.origin.z + n.z * d,
                );
                return WorkPlane {
                    origin,
                    x_axis: x,
                    y_axis: y,
                };
            }
        }
        let n = plane.normal();
        let d = vector(plane.origin, through).dot(n);
        WorkPlane {
            origin: Point3::new(
                plane.origin.x + n.x * d,
                plane.origin.y + n.y * d,
                plane.origin.z + n.z * d,
            ),
            ..plane
        }
    }
    fn angle(a: Point3, b: Point3, tools: &ToolOptions, plane: WorkPlane) -> Result<f64, String> {
        let (x, y, _) = Self::axes(tools, plane)?;
        let d = vector(a, b);
        let u = d.dot(x);
        let v = d.dot(y);
        if u.hypot(v) <= 1e-6 {
            return Err("所选方向在旋转轴法平面内长度为零".into());
        }
        Ok(v.atan2(u).to_degrees())
    }
    fn scalar_from_point(
        &self,
        p: Point3,
        tools: &ToolOptions,
        plane: WorkPlane,
    ) -> Result<f64, String> {
        let base = self.base.ok_or("请先指定基点")?;
        if self.kind == TransformKind::Rotate {
            Self::angle(base, p, tools, plane)
        } else {
            Ok(vector(base, p).length())
        }
    }
    fn default_value(&self, tools: &ToolOptions) -> f64 {
        if self.phase == Phase::RefStart {
            if self.kind == TransformKind::Rotate {
                0.
            } else {
                1.
            }
        } else if self.phase == Phase::RefValue {
            self.reference.unwrap_or(1.)
        } else if self.kind == TransformKind::Rotate {
            tools.angle
        } else {
            tools.scale
        }
    }
    pub fn spec(
        &self,
        hover: Option<Point3>,
        tools: &ToolOptions,
        plane: WorkPlane,
    ) -> Option<Spec> {
        if !matches!(self.phase, Phase::Value | Phase::RefStart | Phase::RefValue)
            || self.kind == TransformKind::Mirror
        {
            return None;
        }
        let kind = match (self.kind, self.phase) {
            (TransformKind::Rotate, Phase::RefStart) => Kind::ReferenceAngle,
            (TransformKind::Rotate, Phase::RefValue) => Kind::TargetAngle,
            (TransformKind::Scale, Phase::RefStart) => Kind::ReferenceLength,
            (TransformKind::Scale, Phase::RefValue) => Kind::TargetLength,
            (TransformKind::Rotate, _) => Kind::Angle,
            _ => Kind::Scale,
        };
        let value = self
            .typed
            .or_else(|| {
                if matches!(self.phase, Phase::Value | Phase::RefValue) && self.takes_point() {
                    hover.and_then(|p| self.scalar_from_point(p, tools, plane).ok())
                } else {
                    None
                }
            })
            .unwrap_or(self.default_value(tools));
        Some(Spec {
            kind,
            reference: self.reference_point().map(array),
            plane: self.point_plane(tools, plane),
            point: None,
            value,
            secondary: 0.,
        })
    }
    fn set_scalar(&self, tools: &mut ToolOptions, value: f64) -> Result<(), String> {
        if !value.is_finite() {
            return Err("变换参数须有限".into());
        }
        if self.kind == TransformKind::Rotate {
            tools.angle = if self.phase == Phase::RefValue {
                value - self.reference.ok_or("请先定义参照角度")?
            } else {
                value
            };
        } else {
            let scale = if self.phase == Phase::RefValue {
                value / self.reference.ok_or("请先定义参照长度")?
            } else {
                value
            };
            if !scale.is_finite() || scale <= 0. {
                return Err("长度及统一缩放比例须大于零".into());
            }
            tools.scale = scale;
        }
        Ok(())
    }
    pub fn preview_value(&mut self, text: &str, tools: &mut ToolOptions) {
        self.typed = None;
        if !matches!(self.phase, Phase::Value | Phase::RefValue | Phase::RefStart) {
            return;
        }
        if let Ok(value) = number(text) {
            let mut candidate = tools.clone();
            if self.phase == Phase::RefStart || self.set_scalar(&mut candidate, value).is_ok() {
                self.typed = Some(value);
                if self.phase != Phase::RefStart {
                    *tools = candidate;
                }
            }
        }
    }
    fn normal(
        &self,
        p: Option<Point3>,
        tools: &ToolOptions,
        plane: WorkPlane,
    ) -> Result<Vector3, String> {
        if let Some(n) = self.mirror_normal {
            return Ok(n);
        }
        if tools.custom_axis {
            let n = Vector3::new(tools.axis[0], tools.axis[1], tools.axis[2]);
            if !n.length().is_finite() || n.length() <= 1e-9 {
                return Err("镜像平面法向须有限且非零".into());
            }
            return Ok(n);
        }
        if tools.mirror_plane {
            return Ok(plane.normal());
        }
        let d = vector(
            self.base.ok_or("请先指定镜像轴第一点")?,
            p.ok_or("请指定镜像轴第二点")?,
        );
        let n = d.cross(plane.normal());
        if n.length() <= 1e-6 {
            return Err("镜像轴在工作平面内必须有非零方向".into());
        }
        Ok(n)
    }
    fn commit(
        &self,
        tools: &ToolOptions,
        plane: WorkPlane,
        hover: Option<Point3>,
        copies: bool,
    ) -> Result<Commit, String> {
        let base = array(self.base.ok_or("请先指定基点")?);
        let transform = match self.kind {
            TransformKind::Mirror => {
                let n = self.normal(hover, tools, plane)?;
                AffineTransform::mirror(base, [n.x, n.y, n.z])?
            }
            TransformKind::Rotate => {
                let (_, _, n) = Self::axes(tools, plane)?;
                AffineTransform::rotation(base, [n.x, n.y, n.z], tools.angle)?
            }
            _ => AffineTransform::scaling(base, tools.scale)?,
        };
        Ok(Commit {
            ids: self.ids.clone(),
            transform,
            copies,
        })
    }
    pub fn preview(
        &self,
        tools: &ToolOptions,
        plane: WorkPlane,
        hover: Option<Point3>,
    ) -> Option<Commit> {
        if let Some(pending) = &self.pending {
            return Some(pending.clone());
        }
        if !matches!(
            self.phase,
            Phase::Value | Phase::RefValue | Phase::MirrorAxis | Phase::MirrorConfirm
        ) {
            return None;
        }
        let mut tools = tools.clone();
        if self.kind != TransformKind::Mirror {
            let value = self
                .typed
                .or_else(|| {
                    hover
                        .filter(|_| self.takes_point())
                        .and_then(|p| self.scalar_from_point(p, &tools, plane).ok())
                })
                .unwrap_or(self.default_value(&tools));
            self.set_scalar(&mut tools, value).ok()?;
        }
        self.commit(
            &tools,
            plane,
            hover,
            if self.kind == TransformKind::Mirror {
                true
            } else {
                tools.copies
            },
        )
        .ok()
    }
    pub fn accept_point(
        &mut self,
        p: Point3,
        tools: &mut ToolOptions,
        plane: WorkPlane,
    ) -> Result<(), String> {
        match self.phase {
            Phase::Base => {
                self.base = Some(p);
                self.phase = if self.kind == TransformKind::Mirror {
                    if tools.custom_axis || tools.mirror_plane {
                        self.mirror_normal = Some(self.normal(None, tools, plane)?);
                        Phase::MirrorConfirm
                    } else {
                        Phase::MirrorAxis
                    }
                } else {
                    Phase::Value
                };
            }
            Phase::RefStart => {
                if let Some(value) = self.typed {
                    if self.kind == TransformKind::Scale && value <= 1e-6 {
                        return Err("参照长度须大于零".into());
                    }
                    self.reference = Some(value);
                    self.phase = Phase::RefValue;
                } else {
                    self.ref_first = Some(p);
                    self.phase = Phase::RefEnd;
                }
            }
            Phase::RefEnd => {
                let first = self.ref_first.ok_or("请指定参照第一点")?;
                let value = if self.kind == TransformKind::Rotate {
                    Self::angle(first, p, tools, plane)?
                } else {
                    vector(first, p).length()
                };
                if self.kind == TransformKind::Scale && value <= 1e-6 {
                    return Err("参照长度不能为零".into());
                }
                self.reference = Some(value);
                self.phase = Phase::RefValue;
            }
            Phase::Value | Phase::RefValue => {
                let value = if let Some(value) = self.typed {
                    value
                } else {
                    self.scalar_from_point(p, tools, plane)?
                };
                self.set_scalar(tools, value)?;
                self.pending = Some(self.commit(tools, plane, Some(p), tools.copies)?);
            }
            Phase::MirrorAxis => {
                self.mirror_normal = Some(self.normal(Some(p), tools, plane)?);
                self.phase = Phase::MirrorConfirm;
            }
            _ => return Err("当前步骤不接收点，请按提示输入".into()),
        }
        self.typed = None;
        Ok(())
    }
    pub fn input(
        &mut self,
        text: &str,
        selected: &BTreeSet<u32>,
        tools: &mut ToolOptions,
        plane: WorkPlane,
        hover: Option<Point3>,
    ) -> Result<Option<Commit>, String> {
        let text = text.trim();
        let upper = text.to_uppercase();
        if self.can_reference() && matches!(upper.as_str(), "R" | "REFERENCE") {
            self.phase = Phase::RefStart;
            self.reference = None;
            self.ref_first = None;
            self.typed = None;
            self.pending = None;
            return Ok(None);
        }
        if self.can_reference() && matches!(upper.as_str(), "C" | "COPY") {
            tools.copies = !tools.copies;
            if let Some(pending) = &mut self.pending {
                pending.copies = tools.copies;
            }
            return Ok(None);
        }
        if let Some(pending) = &self.pending {
            if text.is_empty() {
                return Ok(Some(pending.clone()));
            }
        }
        match self.phase {
            Phase::Select => {
                if !text.is_empty() || selected.is_empty() {
                    return Err("请选择对象后确认".into());
                }
                self.ids = selected.clone();
                self.phase = Phase::Base;
            }
            Phase::RefStart => {
                let value = if text.is_empty() {
                    self.default_value(tools)
                } else {
                    number(text)?
                };
                if self.kind == TransformKind::Scale && value <= 1e-6 {
                    return Err("参照长度须大于零".into());
                }
                self.reference = Some(value);
                self.phase = Phase::RefValue;
            }
            Phase::Value | Phase::RefValue => {
                let value = if text.is_empty() {
                    self.typed
                        .or_else(|| {
                            hover
                                .filter(|_| self.takes_point())
                                .and_then(|p| self.scalar_from_point(p, tools, plane).ok())
                        })
                        .unwrap_or(self.default_value(tools))
                } else {
                    number(text)?
                };
                self.set_scalar(tools, value)?;
                let commit = self.commit(tools, plane, hover, tools.copies)?;
                self.pending = Some(commit.clone());
                return Ok(Some(commit));
            }
            Phase::MirrorAxis if text.is_empty() && (tools.custom_axis || tools.mirror_plane) => {
                self.mirror_normal = Some(self.normal(None, tools, plane)?);
                self.phase = Phase::MirrorConfirm;
            }
            Phase::MirrorConfirm => {
                let copies = match upper.as_str() {
                    "" | "N" | "NO" | "否" => true,
                    "Y" | "YES" | "是" => false,
                    _ => return Err("输入 N 保留源对象，或 Y 以镜像结果替换源对象".into()),
                };
                let commit = self.commit(tools, plane, None, copies)?;
                self.pending = Some(commit.clone());
                return Ok(Some(commit));
            }
            _ => return Err("请在画布指定点或输入完整坐标".into()),
        }
        self.typed = None;
        Ok(None)
    }
    pub fn prompt(&self) -> String {
        format!(
            "{}：{}",
            self.kind.label(),
            match self.phase {
                Phase::Select => "选择对象后确认，Shift 移除",
                Phase::Base =>
                    if self.kind == TransformKind::Mirror {
                        "指定镜像轴第一点 / 高级镜像基点"
                    } else {
                        "指定变换基点"
                    },
                Phase::Value =>
                    if self.kind == TransformKind::Rotate {
                        "移动鼠标取角度或输入角度；R 参照，C 切换复制"
                    } else {
                        "输入比例；R 参照长度，C 切换复制"
                    },
                Phase::RefStart =>
                    if self.kind == TransformKind::Rotate {
                        "输入原角度或指定参照第一点"
                    } else {
                        "输入原长度或指定参照第一点"
                    },
                Phase::RefEnd => "指定参照第二点",
                Phase::RefValue =>
                    if self.kind == TransformKind::Rotate {
                        "输入新方向角度或点选新方向"
                    } else {
                        "输入新长度或点选距基点的新长度"
                    },
                Phase::MirrorAxis => "指定镜像轴第二点；高级平面模式可 Enter 确认",
                Phase::MirrorConfirm => "是否替换源对象 [Y 是 / N 否] <N 保留源对象>",
            }
        )
    }
}
