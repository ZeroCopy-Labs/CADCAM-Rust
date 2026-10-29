//! Analytic drawing construction and similarity transforms in workspace millimeters.
use crate::model::{
    Arc, Circle, Color, Ellipse, Geometry, LayerId, Line, Plane, Point3, Polyline, PolylineSegment,
    Spline, Vector3, WorkPlane,
};
use acadrust::{
    entities::{
        Arc as NArc, Circle as NCircle, Ellipse as NEllipse, Line as NLine, LwPolyline, LwVertex,
        Polyline3D, Spline as NSpline,
    },
    types::{Matrix3, Vector2, Vector3 as NV},
    EntityType,
};
use std::f64::consts::TAU;

pub(crate) fn array(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}
fn delta(a: Point3, b: Point3) -> Vector3 {
    Vector3::new(b.x - a.x, b.y - a.y, b.z - a.z)
}
fn add(p: Point3, v: Vector3) -> Point3 {
    Point3::new(p.x + v.x, p.y + v.y, p.z + v.z)
}
fn finite(values: impl IntoIterator<Item = f64>) -> bool {
    values.into_iter().all(f64::is_finite)
}
fn length(a: Point3, b: Point3) -> f64 {
    delta(a, b).length()
}

#[derive(Clone, Debug)]
pub struct NewEntity {
    pub geometry: Geometry,
    pub layer: LayerId,
    pub color: Color,
    pub normal: Option<Vector3>,
}

/// Geometry clipboard: world millimeters and common color/layer attributes only.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ClipboardItem {
    pub geometry: Geometry,
    pub layer: String,
    pub layer_color: Color,
    pub color: Color,
    pub reference: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SketchKind {
    Line,
    Polyline,
    Rectangle,
    Circle,
    CircleDiameter,
    Circle2P,
    Circle3P,
    Arc,
    ArcCenterEnd,
    ArcCenterAngle,
    Ellipse,
    EllipseArc,
    Spline,
}
impl SketchKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Line => "连续直线",
            Self::Polyline => "多段线",
            Self::Rectangle => "矩形",
            Self::Circle => "圆",
            Self::CircleDiameter => "圆（圆心/直径）",
            Self::Circle2P => "圆（两点直径）",
            Self::Circle3P => "圆（三点）",
            Self::Arc => "三点圆弧",
            Self::ArcCenterEnd => "圆弧（圆心/起点/终点方向）",
            Self::ArcCenterAngle => "圆弧（圆心/起点/扫角）",
            Self::Ellipse => "椭圆",
            Self::EllipseArc => "椭圆弧",
            Self::Spline => "控制点样条",
        }
    }
    pub fn command(self) -> &'static str {
        match self {
            Self::Line => "LINE",
            Self::Polyline => "PLINE",
            Self::Rectangle => "RECTANGLE",
            Self::Circle => "CIRCLE",
            Self::CircleDiameter => "CIRCLED",
            Self::Circle2P => "CIRCLE2P",
            Self::Circle3P => "CIRCLE3P",
            Self::Arc => "ARC",
            Self::ArcCenterEnd => "ARCCE",
            Self::ArcCenterAngle => "ARCCA",
            Self::Ellipse => "ELLIPSE",
            Self::EllipseArc => "ELLIPSEARC",
            Self::Spline => "SPLINE",
        }
    }
    pub fn continuous(self) -> bool {
        matches!(self, Self::Line | Self::Polyline | Self::Spline)
    }
    pub fn is_circle(self) -> bool {
        matches!(
            self,
            Self::Circle | Self::CircleDiameter | Self::Circle2P | Self::Circle3P
        )
    }
    pub fn is_arc(self) -> bool {
        matches!(self, Self::Arc | Self::ArcCenterEnd | Self::ArcCenterAngle)
    }
    pub fn is_ellipse(self) -> bool {
        matches!(self, Self::Ellipse | Self::EllipseArc)
    }
    pub fn required(self) -> usize {
        match self {
            Self::Line
            | Self::Polyline
            | Self::Rectangle
            | Self::Circle
            | Self::CircleDiameter
            | Self::Circle2P
            | Self::ArcCenterAngle => 2,
            Self::Arc | Self::ArcCenterEnd | Self::Ellipse | Self::Circle3P => 3,
            Self::Spline => 4,
            Self::EllipseArc => 5,
        }
    }
}
#[derive(Clone, Debug)]
pub struct Sketch {
    pub kind: SketchKind,
    pub plane: WorkPlane,
    pub points: Vec<Point3>,
    pub closed: bool,
    pub degree: u32,
    /// Signed sweep entered after the center and start picks for ArcCenterAngle.
    pub sweep_degrees: Option<f64>,
}
impl Sketch {
    pub fn ellipse(&self) -> Result<Ellipse, String> {
        if self.points.len() < 3 {
            return Err("椭圆需要中心、长半轴端点和短半轴点".into());
        }
        let major = delta(self.points[0], self.points[1]);
        let a = major.length();
        if !a.is_finite() || a <= 1e-9 {
            return Err("长半轴必须大于零".into());
        }
        let dir = self.plane.normal().cross(major * (1.0 / a));
        let b = delta(self.points[0], self.points[2]).dot(dir).abs();
        if !b.is_finite() || b <= 1e-9 || b > a {
            return Err("短半轴必须大于零且不超过长半轴".into());
        }
        Ok(Ellipse {
            center: self.points[0],
            major_axis: major,
            minor_axis: dir * b,
            start_parameter: 0.0,
            sweep_parameter: TAU,
        })
    }
    pub fn new(kind: SketchKind, plane: WorkPlane, degree: u32) -> Self {
        Self {
            kind,
            plane,
            points: Vec::new(),
            closed: false,
            degree,
            sweep_degrees: None,
        }
    }
    pub fn push(&mut self, p: Point3) -> Result<(), String> {
        if !finite(array(p)) {
            return Err("坐标必须为有限数值".into());
        }
        if self.kind != SketchKind::Line && self.plane.to_local(p)[2].abs() > 1e-5 {
            return Err("该工具在当前工作平面绘制；请切换平面或启用投影捕捉".into());
        }
        if self.points.last().is_some_and(|q| length(*q, p) <= 1e-9)
            && !(self.kind == SketchKind::EllipseArc && self.points.len() >= 3)
        {
            return Err("相邻点不能重合".into());
        }
        self.points.push(p);
        Ok(())
    }
    pub fn geometry(&self) -> Result<Vec<Geometry>, String> {
        let p = &self.points;
        let plane = Plane {
            normal: self.plane.normal(),
            x_axis: self.plane.x_axis,
        };
        let need = if self.kind == SketchKind::Spline {
            self.degree as usize + 1
        } else {
            self.kind.required()
        };
        if p.len() < need {
            return Err(format!("{}至少需要 {need} 个点", self.kind.label()));
        }
        let lines = |points: Vec<Point3>, closed: bool| -> Result<Vec<Geometry>, String> {
            let mut points = points;
            if closed && points.len() < 3 {
                return Err("闭合轮廓至少需要三个不同的点".into());
            }
            if closed && length(points[0], *points.last().unwrap()) > 1e-9 {
                points.push(points[0]);
            }
            let segments: Vec<_> = points
                .windows(2)
                .map(|w| PolylineSegment::Line {
                    start: w[0],
                    end: w[1],
                })
                .collect();
            if self.kind == SketchKind::Line {
                Ok(points
                    .windows(2)
                    .map(|w| {
                        Geometry::Line(Line {
                            start: w[0],
                            end: w[1],
                        })
                    })
                    .collect())
            } else {
                Ok(vec![Geometry::Polyline(Polyline {
                    segments,
                    closed,
                    plane: Some(plane),
                })])
            }
        };
        Ok(match self.kind {
            SketchKind::Line | SketchKind::Polyline => return lines(p.clone(), self.closed),
            SketchKind::Rectangle => {
                let a = self.plane.to_local(p[0]);
                let b = self.plane.to_local(p[1]);
                if (a[0] - b[0]).abs() < 1e-9 || (a[1] - b[1]).abs() < 1e-9 {
                    return Err("矩形的宽和高必须非零".into());
                }
                return lines(
                    vec![
                        p[0],
                        self.plane.to_world([b[0], a[1], 0.0]),
                        p[1],
                        self.plane.to_world([a[0], b[1], 0.0]),
                    ],
                    true,
                );
            }
            SketchKind::Circle | SketchKind::CircleDiameter => {
                let radius = length(p[0], p[1])
                    * if self.kind == SketchKind::CircleDiameter {
                        0.5
                    } else {
                        1.0
                    };
                if !radius.is_finite() || radius <= 1e-9 {
                    return Err("圆的半径须有限且大于几何容差".into());
                }
                vec![Geometry::Circle(Circle {
                    center: p[0],
                    radius,
                    plane,
                })]
            }
            SketchKind::Circle2P => {
                let d = delta(p[0], p[1]);
                let center = add(p[0], d * 0.5);
                let radius = d.length() * 0.5;
                if !finite(array(center)) || !radius.is_finite() || radius <= 1e-9 {
                    return Err("两点直径必须有限且非零".into());
                }
                vec![Geometry::Circle(Circle {
                    center,
                    radius,
                    plane,
                })]
            }
            SketchKind::ArcCenterEnd | SketchKind::ArcCenterAngle => {
                let start = delta(p[0], p[1]);
                let radius = start.length();
                if !radius.is_finite() || radius <= 1e-9 {
                    return Err("圆弧半径须有限且大于几何容差".into());
                }
                let start_angle = start
                    .dot(self.plane.y_axis)
                    .atan2(start.dot(self.plane.x_axis));
                let sweep_angle = if self.kind == SketchKind::ArcCenterAngle {
                    let degrees = self.sweep_degrees.ok_or("请输入圆弧扫角")?;
                    if !degrees.is_finite() || degrees.abs() >= 360. {
                        return Err("扫角须有限，且绝对值小于 360°；整圆请用 CIRCLE".into());
                    }
                    degrees.to_radians()
                } else {
                    let end = delta(p[0], p[2]);
                    let u = end.dot(self.plane.x_axis);
                    let v = end.dot(self.plane.y_axis);
                    if !u.hypot(v).is_finite() || u.hypot(v) <= 1e-9 {
                        return Err("终点方向不能与圆心重合".into());
                    }
                    (v.atan2(u) - start_angle).rem_euclid(TAU)
                };
                if !sweep_angle.is_finite()
                    || sweep_angle.abs() <= 1e-12
                    || self.kind == SketchKind::ArcCenterEnd && TAU - sweep_angle <= 1e-12
                {
                    return Err("圆弧扫角不能为零或起止方向相同".into());
                }
                vec![Geometry::Arc(Arc {
                    center: p[0],
                    radius,
                    start_angle,
                    sweep_angle,
                    plane,
                })]
            }
            SketchKind::Arc | SketchKind::Circle3P => {
                let a = self.plane.to_local(p[0]);
                let b = self.plane.to_local(p[1]);
                let c = self.plane.to_local(p[2]);
                let x = b[0] - a[0];
                let y = b[1] - a[1];
                let u = c[0] - a[0];
                let v = c[1] - a[1];
                let den = 2.0 * (x * v - y * u);
                if den.abs() <= 1e-12 * (x.hypot(y) * u.hypot(v)).max(1e-12) {
                    return Err("三点共线或重合，无法建立圆/圆弧".into());
                }
                let cx = ((x * x + y * y) * v - (u * u + v * v) * y) / den;
                let cy = (x * (u * u + v * v) - u * (x * x + y * y)) / den;
                let center = self.plane.to_world([a[0] + cx, a[1] + cy, 0.0]);
                if self.kind == SketchKind::Circle3P {
                    let radius = cx.hypot(cy);
                    if !finite(array(center)) || !radius.is_finite() || radius <= 1e-9 {
                        return Err("三点圆超出有效数值范围".into());
                    }
                    return Ok(vec![Geometry::Circle(Circle {
                        center,
                        radius,
                        plane,
                    })]);
                }
                let start = (-cy).atan2(-cx);
                let mid = (y - cy).atan2(x - cx);
                let end = (v - cy).atan2(u - cx);
                let positive = (end - start).rem_euclid(TAU);
                let sweep = if (mid - start).rem_euclid(TAU) <= positive {
                    positive
                } else {
                    positive - TAU
                };
                vec![Geometry::Arc(Arc {
                    center,
                    radius: cx.hypot(cy),
                    start_angle: start,
                    sweep_angle: sweep,
                    plane,
                })]
            }
            SketchKind::Ellipse | SketchKind::EllipseArc => {
                let mut ellipse = self.ellipse()?;
                if self.kind == SketchKind::EllipseArc {
                    ellipse.start_parameter = ellipse.parameter_at(p[3])?;
                    let end = ellipse.parameter_at(p[4])?;
                    ellipse.sweep_parameter = (end - ellipse.start_parameter).rem_euclid(TAU);
                    if ellipse.sweep_parameter < 1e-10 {
                        ellipse.sweep_parameter = TAU;
                    }
                }
                vec![Geometry::Ellipse(ellipse)]
            }
            SketchKind::Spline => {
                let mut control = p.clone();
                if self.closed && length(control[0], *control.last().unwrap()) > 1e-9 {
                    control.push(control[0]);
                }
                let n = control.len();
                let degree = self.degree as usize;
                let mut knots = vec![0.0; degree + 1];
                for i in 1..n - degree {
                    knots.push(i as f64 / (n - degree) as f64);
                }
                knots.extend(vec![1.0; degree + 1]);
                vec![Geometry::Spline(Spline {
                    degree: self.degree,
                    control_points: control,
                    knots,
                    weights: Vec::new(),
                    fit_points: Vec::new(),
                    closed: self.closed,
                    periodic: false,
                    rational: false,
                    planar: true,
                })]
            }
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AffineTransform {
    pub(crate) linear: [[f64; 3]; 3],
    pub(crate) origin: [f64; 3],
    pub(crate) offset: [f64; 3],
}
impl AffineTransform {
    pub fn translation(offset: [f64; 3]) -> Result<Self, String> {
        if !finite(offset) {
            return Err("位移必须有限".into());
        }
        Ok(Self {
            linear: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            origin: [0.0; 3],
            offset,
        })
    }
    pub fn rotation(origin: [f64; 3], axis: [f64; 3], degrees: f64) -> Result<Self, String> {
        let v = Vector3::new(axis[0], axis[1], axis[2]);
        let len = v.length();
        if !finite(origin) || !degrees.is_finite() || !len.is_finite() || len < 1e-12 {
            return Err("旋转基点/角度必须有限，旋转轴不能为零".into());
        }
        let n = v * (1.0 / len);
        let [x, y, z] = [n.x, n.y, n.z];
        let a = degrees.rem_euclid(360.0).to_radians();
        let (s, c) = a.sin_cos();
        let t = 1.0 - c;
        Ok(Self {
            linear: [
                [c + x * x * t, x * y * t - z * s, x * z * t + y * s],
                [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
                [z * x * t - y * s, z * y * t + x * s, c + z * z * t],
            ],
            origin,
            offset: [0.0; 3],
        })
    }
    pub fn scaling(origin: [f64; 3], scale: f64) -> Result<Self, String> {
        if !finite(origin) || !scale.is_finite() || scale <= 0.0 {
            return Err("基点须有限，统一缩放比例须大于零".into());
        }
        Ok(Self {
            linear: [[scale, 0.0, 0.0], [0.0, scale, 0.0], [0.0, 0.0, scale]],
            origin,
            offset: [0.0; 3],
        })
    }
    pub fn mirror(origin: [f64; 3], normal: [f64; 3]) -> Result<Self, String> {
        let n = Vector3::new(normal[0], normal[1], normal[2]);
        let len = n.length();
        if !finite(origin) || !len.is_finite() || len < 1e-12 {
            return Err("镜像平面原点须有限，法向不能为零".into());
        }
        let n = [n.x / len, n.y / len, n.z / len];
        let linear = std::array::from_fn(|i| {
            std::array::from_fn(|j| (if i == j { 1.0 } else { 0.0 }) - 2.0 * n[i] * n[j])
        });
        Ok(Self {
            linear,
            origin,
            offset: [0.0; 3],
        })
    }
    pub fn point(self, p: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|i| {
            self.origin[i]
                + self.offset[i]
                + (0..3)
                    .map(|j| self.linear[i][j] * (p[j] - self.origin[j]))
                    .sum::<f64>()
        })
    }
    pub(crate) fn identity(self) -> bool {
        self.linear == [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
            && self.offset == [0.0; 3]
    }
}

fn nv(p: Point3, scale: f64) -> NV {
    NV::new(p.x / scale, p.y / scale, p.z / scale)
}
fn normal(v: Vector3) -> NV {
    NV::new(v.x, v.y, v.z)
}
pub(crate) fn native_geometry(geometry: &Geometry, scale: f64) -> Result<EntityType, String> {
    Ok(match geometry {
        Geometry::Text(t) => {
            if t.content.is_empty()
                || t.content.contains(['\r', '\n'])
                || !t.height.is_finite()
                || t.height <= 0.0
                || ![t.position.x, t.position.y, t.position.z]
                    .iter()
                    .all(|v| v.is_finite())
                || (t.plane.normal.length() - 1.0).abs() > 1e-6
                || (t.plane.x_axis.length() - 1.0).abs() > 1e-6
                || t.plane.normal.dot(t.plane.x_axis).abs() > 1e-6
            {
                return Err("单行文字不能为空，位置有限、字高须大于零".into());
            }
            let n = normal(t.plane.normal);
            let basis = Matrix3::arbitrary_axis(n).transpose();
            let x = basis * normal(t.plane.x_axis);
            let mut text =
                acadrust::entities::Text::with_value(&t.content, basis * nv(t.position, scale))
                    .with_height(t.height / scale)
                    .with_rotation(x.y.atan2(x.x));
            text.normal = n;
            EntityType::Text(text)
        }
        Geometry::Line(l) => {
            EntityType::Line(NLine::from_points(nv(l.start, scale), nv(l.end, scale)))
        }
        Geometry::Circle(c) => {
            let n = normal(c.plane.normal);
            let mut e = NCircle::new();
            e.normal = n;
            e.center = Matrix3::arbitrary_axis(n).transpose() * nv(c.center, scale);
            e.radius = c.radius / scale;
            EntityType::Circle(e)
        }
        Geometry::Arc(a) => {
            let n = normal(a.plane.normal) * a.sweep_angle.signum();
            let basis = Matrix3::arbitrary_axis(n).transpose();
            let start = add(
                a.center,
                a.plane.x_axis * (a.radius * a.start_angle.cos())
                    + a.plane.y_axis() * (a.radius * a.start_angle.sin()),
            );
            let d = basis * (nv(start, scale) - nv(a.center, scale));
            let angle = d.y.atan2(d.x);
            let mut e = NArc::new();
            e.normal = n;
            e.center = basis * nv(a.center, scale);
            e.radius = a.radius / scale;
            e.start_angle = angle;
            e.end_angle = angle + a.sweep_angle.abs();
            EntityType::Arc(e)
        }
        Geometry::Ellipse(e) => {
            let mut n = NEllipse::new();
            n.center = nv(e.center, scale);
            n.major_axis = normal(e.major_axis) * (1.0 / scale);
            n.normal = normal(e.major_axis.cross(e.minor_axis).normalize());
            n.minor_axis_ratio = e.minor_axis.length() / e.major_axis.length();
            n.start_parameter = e.start_parameter;
            n.end_parameter = e.start_parameter + e.sweep_parameter;
            EntityType::Ellipse(n)
        }
        Geometry::Spline(s) => {
            let mut n = NSpline::new();
            n.degree = s.degree as i32;
            n.control_points = s.control_points.iter().map(|p| nv(*p, scale)).collect();
            n.knots = s.knots.clone();
            n.weights = s.weights.clone();
            n.fit_points = s.fit_points.iter().map(|p| nv(*p, scale)).collect();
            n.flags.closed = s.closed;
            n.flags.periodic = s.periodic;
            n.flags.rational = s.rational;
            n.flags.planar = s.planar;
            EntityType::Spline(n)
        }
        Geometry::Polyline(p) => {
            if p.segments.is_empty() {
                return Err("多段线不能为空".into());
            }
            let vertices: Vec<_> =
                p.segments
                    .iter()
                    .map(|s| match s {
                        PolylineSegment::Line { start, .. }
                        | PolylineSegment::Arc { start, .. } => *start,
                    })
                    .collect();
            if let Some(plane) = p.plane {
                let n = normal(plane.normal);
                let basis = Matrix3::arbitrary_axis(n).transpose();
                let mut e = LwPolyline::new();
                e.normal = n;
                e.is_closed = p.closed;
                e.elevation = (basis * nv(vertices[0], scale)).z;
                for (point, segment) in vertices.iter().zip(&p.segments) {
                    let q = basis * nv(*point, scale);
                    let mut v = LwVertex::new(Vector2::new(q.x, q.y));
                    if let PolylineSegment::Arc { sweep_angle, .. } = segment {
                        v.bulge = (sweep_angle / 4.0).tan();
                    }
                    e.vertices.push(v);
                }
                if !p.closed {
                    let end = match p.segments.last().unwrap() {
                        PolylineSegment::Line { end, .. } | PolylineSegment::Arc { end, .. } => {
                            *end
                        }
                    };
                    let q = basis * nv(end, scale);
                    e.vertices.push(LwVertex::new(Vector2::new(q.x, q.y)));
                }
                EntityType::LwPolyline(e)
            } else {
                if p.segments
                    .iter()
                    .any(|s| matches!(s, PolylineSegment::Arc { .. }))
                {
                    return Err("空间多段线不能包含未定义平面的圆弧".into());
                }
                let mut vertices = vertices;
                if !p.closed {
                    let end = match p.segments.last().unwrap() {
                        PolylineSegment::Line { end, .. } | PolylineSegment::Arc { end, .. } => {
                            *end
                        }
                    };
                    vertices.push(end);
                }
                let mut e =
                    Polyline3D::from_points(vertices.iter().map(|p| nv(*p, scale)).collect());
                e.flags.closed = p.closed;
                EntityType::Polyline3D(e)
            }
        }
    })
}
