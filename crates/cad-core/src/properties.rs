//! Geometry properties and grips operate on analytic WCS geometry, never display polylines.
use crate::geometry_edit::native_geometry;
use crate::model::{Geometry, Plane, Point3, PolylineSegment, Vector3};
use acadrust::{types::Vector3 as NV, EntityType};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GripKind {
    Start,
    End,
    Center,
    Radius,
    Major,
    Minor,
    Vertex(usize),
    Control(usize),
}
#[derive(Clone, Debug)]
pub struct Grip {
    pub kind: GripKind,
    pub label: String,
    pub point: Point3,
}
fn v(a: Point3, b: Point3) -> Vector3 {
    Vector3::new(b.x - a.x, b.y - a.y, b.z - a.z)
}
fn add(p: Point3, d: Vector3) -> Point3 {
    Point3::new(p.x + d.x, p.y + d.y, p.z + d.z)
}
fn arc_point(c: Point3, r: f64, plane: Plane, a: f64) -> Point3 {
    add(
        c,
        plane.x_axis * (r * a.cos()) + plane.y_axis() * (r * a.sin()),
    )
}
pub fn grips(g: &Geometry) -> Vec<Grip> {
    let mut out = Vec::new();
    let mut push = |kind, label: String, point| out.push(Grip { kind, label, point });
    match g {
        Geometry::Line(l) => {
            push(GripKind::Start, "起点".into(), l.start);
            push(GripKind::End, "终点".into(), l.end);
        }
        Geometry::Circle(c) => {
            push(GripKind::Center, "圆心".into(), c.center);
            push(
                GripKind::Radius,
                "半径".into(),
                arc_point(c.center, c.radius, c.plane, 0.0),
            );
        }
        Geometry::Arc(a) => {
            push(GripKind::Center, "圆心".into(), a.center);
            push(
                GripKind::Start,
                "弧起点".into(),
                arc_point(a.center, a.radius, a.plane, a.start_angle),
            );
            push(
                GripKind::End,
                "弧终点".into(),
                arc_point(a.center, a.radius, a.plane, a.start_angle + a.sweep_angle),
            );
            push(
                GripKind::Radius,
                "半径".into(),
                arc_point(
                    a.center,
                    a.radius,
                    a.plane,
                    a.start_angle + a.sweep_angle / 2.0,
                ),
            );
        }
        Geometry::Ellipse(e) => {
            push(GripKind::Center, "中心".into(), e.center);
            push(
                GripKind::Major,
                "长半轴端点".into(),
                add(e.center, e.major_axis),
            );
            push(
                GripKind::Minor,
                "短半轴端点".into(),
                add(e.center, e.minor_axis),
            );
        }
        Geometry::Polyline(p) => {
            for (i, s) in p.segments.iter().enumerate() {
                let (PolylineSegment::Line { start, .. } | PolylineSegment::Arc { start, .. }) = s;
                push(GripKind::Vertex(i), format!("顶点 {}", i + 1), *start);
            }
            if !p.closed {
                if let Some(PolylineSegment::Line { end, .. } | PolylineSegment::Arc { end, .. }) =
                    p.segments.last()
                {
                    push(GripKind::Vertex(p.segments.len()), "末顶点".into(), *end);
                }
            }
        }
        Geometry::Spline(s) => {
            if !s.periodic {
                for (i, p) in s.control_points.iter().enumerate() {
                    push(GripKind::Control(i), format!("控制点 {}", i + 1), *p);
                }
            }
        }
    }
    out
}
fn in_plane(origin: Point3, plane: Plane, p: Point3) -> Result<Vector3, String> {
    let d = v(origin, p);
    if d.dot(plane.normal).abs() > 1e-5 {
        return Err("该夹点必须保持在图元自身平面内；请切换工作平面或输入共面坐标".into());
    }
    Ok(d)
}
fn positive(r: f64) -> Result<f64, String> {
    if !r.is_finite() || r <= 1e-9 {
        Err("长度或半径必须为大于零的有限数值".into())
    } else {
        Ok(r)
    }
}
pub fn radius(g: &Geometry) -> Option<f64> {
    match g {
        Geometry::Circle(c) => Some(c.radius),
        Geometry::Arc(a) => Some(a.radius),
        _ => None,
    }
}
pub fn with_radius(g: &Geometry, r: f64) -> Result<Geometry, String> {
    let mut g = g.clone();
    let r = positive(r)?;
    match &mut g {
        Geometry::Circle(c) => c.radius = r,
        Geometry::Arc(a) => a.radius = r,
        _ => return Err("半径批量修改只支持圆和圆弧".into()),
    };
    Ok(g)
}
pub fn move_grip(g: &Geometry, kind: GripKind, p: Point3) -> Result<Geometry, String> {
    if ![p.x, p.y, p.z].into_iter().all(f64::is_finite) {
        return Err("夹点坐标必须有限".into());
    }
    let mut next = g.clone();
    match (&mut next, kind) {
        (Geometry::Line(l), GripKind::Start) => {
            positive(v(p, l.end).length())?;
            l.start = p;
        }
        (Geometry::Line(l), GripKind::End) => {
            positive(v(l.start, p).length())?;
            l.end = p;
        }
        (Geometry::Circle(c), GripKind::Center) => c.center = p,
        (Geometry::Circle(c), GripKind::Radius) => {
            c.radius = positive(in_plane(c.center, c.plane, p)?.length())?;
        }
        (Geometry::Arc(a), GripKind::Center) => a.center = p,
        (Geometry::Arc(a), GripKind::Radius) => {
            a.radius = positive(in_plane(a.center, a.plane, p)?.length())?;
        }
        (Geometry::Arc(a), GripKind::Start | GripKind::End) => {
            let old = arc_point(
                a.center,
                a.radius,
                a.plane,
                if kind == GripKind::Start {
                    a.start_angle
                } else {
                    a.start_angle + a.sweep_angle
                },
            );
            if old == p {
                return Ok(g.clone());
            }
            in_plane(a.center, a.plane, p)?;
            let start = if kind == GripKind::Start {
                p
            } else {
                arc_point(a.center, a.radius, a.plane, a.start_angle)
            };
            let middle = arc_point(
                a.center,
                a.radius,
                a.plane,
                a.start_angle + a.sweep_angle / 2.0,
            );
            let end = if kind == GripKind::End {
                p
            } else {
                arc_point(a.center, a.radius, a.plane, a.start_angle + a.sweep_angle)
            };
            let plane = crate::model::WorkPlane {
                origin: a.center,
                x_axis: a.plane.x_axis,
                y_axis: a.plane.y_axis(),
            };
            let mut sketch =
                crate::geometry_edit::Sketch::new(crate::geometry_edit::SketchKind::Arc, plane, 3);
            for point in [start, middle, end] {
                sketch.push(point)?;
            }
            let Geometry::Arc(result) = sketch.geometry()?.remove(0) else {
                unreachable!()
            };
            *a = result;
        }
        (Geometry::Ellipse(e), GripKind::Center) => e.center = p,
        (Geometry::Ellipse(e), GripKind::Major | GripKind::Minor) => {
            let normal = e.major_axis.cross(e.minor_axis).normalize();
            let plane = Plane::new(normal, e.major_axis);
            let d = in_plane(e.center, plane, p)?;
            if kind == GripKind::Major {
                let length = positive(d.length())?;
                let b = e.minor_axis.length();
                if length < b {
                    return Err("长半轴不能小于短半轴".into());
                }
                e.major_axis = d;
                e.minor_axis = normal.cross(d * (1.0 / length)) * b;
            } else {
                let direction = e.minor_axis.normalize();
                let length = positive(d.dot(direction).abs())?;
                if length > e.major_axis.length() {
                    return Err("短半轴不能超过长半轴".into());
                }
                e.minor_axis = direction * length;
            }
        }
        (Geometry::Polyline(poly), GripKind::Vertex(index)) => {
            let count = poly.segments.len();
            if count == 0 || index > count || (poly.closed && index == count) {
                return Err("多段线顶点已失效".into());
            }
            if let Some(plane) = poly.plane {
                let (PolylineSegment::Line { start, .. } | PolylineSegment::Arc { start, .. }) =
                    &poly.segments[0];
                in_plane(*start, plane, p)?;
            }
            for (i, s) in poly.segments.iter_mut().enumerate() {
                let affected =
                    i == index || i + 1 == index || (poly.closed && index == 0 && i + 1 == count);
                if !affected {
                    continue;
                }
                let (start, end) = match s {
                    PolylineSegment::Line { start, end }
                    | PolylineSegment::Arc { start, end, .. } => (start, end),
                };
                if i == index {
                    *start = p;
                }
                if i + 1 == index || (poly.closed && index == 0 && i + 1 == count) {
                    *end = p;
                }
                positive(v(*start, *end).length())?;
                if let PolylineSegment::Arc {
                    start,
                    end,
                    center,
                    radius,
                    start_angle,
                    sweep_angle,
                } = s
                {
                    let plane = poly.plane.ok_or("圆弧多段线缺少平面")?;
                    let chord = v(*start, *end);
                    let len = positive(chord.length())?;
                    let offset = len / (2.0 * (*sweep_angle / 2.0).tan());
                    let mid = add(*start, chord * 0.5);
                    *center = add(mid, plane.normal.cross(chord * (1.0 / len)) * offset);
                    let d = v(*center, *start);
                    *radius = positive(d.length())?;
                    *start_angle = d.dot(plane.y_axis()).atan2(d.dot(plane.x_axis));
                }
            }
        }
        (Geometry::Spline(s), GripKind::Control(index)) => {
            if s.periodic {
                return Err("周期样条暂不提供控制点编辑".into());
            }
            let old = *s.control_points.get(index).ok_or("样条控制点已失效")?;
            if s.planar {
                // Moving outside the old plane explicitly turns it into a spatial spline.
                let origin = s.control_points[0];
                let normal = s
                    .control_points
                    .iter()
                    .map(|p| v(origin, *p))
                    .find(|d| d.length() > 1e-9)
                    .and_then(|axis| {
                        s.control_points
                            .iter()
                            .map(|p| axis.cross(v(origin, *p)))
                            .find(|n| n.length() > 1e-9)
                    })
                    .map(Vector3::normalize);
                if normal.is_some_and(|n| v(old, p).dot(n).abs() > 1e-5) {
                    s.planar = false;
                }
            }
            let last = s.control_points.len() - 1;
            let tied = s.closed && v(s.control_points[0], s.control_points[last]).length() < 1e-9;
            s.control_points[index] = p;
            if tied && (index == 0 || index == last) {
                s.control_points[0] = p;
                s.control_points[last] = p;
            }
            // Control points are authoritative after a manual edit; stale fit data must not compete.
            s.fit_points.clear();
        }
        _ => return Err("该图元不支持所选夹点".into()),
    }
    Ok(next)
}

pub(crate) fn update_native(
    entity: &mut EntityType,
    g: &Geometry,
    scale: f64,
) -> Result<(), String> {
    let new = native_geometry(g, scale)?;
    entity.common_mut().raw_record = None;
    match (entity, new) {
        (EntityType::Line(e), EntityType::Line(n)) => {
            e.start = n.start;
            e.end = n.end;
        }
        (EntityType::Circle(e), EntityType::Circle(n)) => {
            e.center = n.center;
            e.radius = n.radius;
            e.normal = n.normal;
        }
        (EntityType::Arc(e), EntityType::Arc(n)) => {
            e.center = n.center;
            e.radius = n.radius;
            e.normal = n.normal;
            e.start_angle = n.start_angle;
            e.end_angle = n.end_angle;
        }
        (EntityType::Ellipse(e), EntityType::Ellipse(n)) => {
            e.center = n.center;
            e.major_axis = n.major_axis;
            e.normal = n.normal;
            e.minor_axis_ratio = n.minor_axis_ratio;
            e.start_parameter = n.start_parameter;
            e.end_parameter = n.end_parameter;
        }
        (EntityType::Spline(e), EntityType::Spline(n)) => {
            e.control_points = n.control_points;
            e.fit_points = n.fit_points;
            e.knots = n.knots;
            e.weights = n.weights;
            e.degree = n.degree;
            e.flags = n.flags;
            if e.flags.planar {
                if let Some(origin) = e.control_points.first() {
                    if e.control_points
                        .iter()
                        .any(|p| (*p - *origin).dot(&e.normal).abs() > 1e-5 / scale)
                    {
                        e.flags.planar = false;
                    }
                }
            }
            if !e.flags.planar {
                e.normal = NV::ZERO;
            }
        }
        (EntityType::LwPolyline(e), EntityType::LwPolyline(n))
            if e.vertices.len() == n.vertices.len() =>
        {
            e.normal = n.normal;
            e.elevation = n.elevation;
            for (v, p) in e.vertices.iter_mut().zip(n.vertices) {
                v.location = p.location;
                v.bulge = p.bulge;
            }
        }
        (EntityType::Polyline2D(e), EntityType::LwPolyline(n))
            if e.vertices.len() == n.vertices.len() =>
        {
            e.normal = n.normal;
            e.elevation = n.elevation;
            for (v, p) in e.vertices.iter_mut().zip(n.vertices) {
                v.location = NV::new(p.location.x, p.location.y, 0.0);
                v.bulge = p.bulge;
            }
        }
        (EntityType::Polyline3D(e), EntityType::Polyline3D(n))
            if e.vertices.len() == n.vertices.len() =>
        {
            for (v, p) in e.vertices.iter_mut().zip(n.vertices) {
                v.position = p.position;
            }
        }
        (EntityType::Polyline(e), EntityType::Polyline3D(n))
            if e.vertices.len() == n.vertices.len() =>
        {
            for (v, p) in e.vertices.iter_mut().zip(n.vertices) {
                v.location = p.position;
            }
        }
        _ => return Err("该原生图元不支持此几何修改，已取消整批操作".into()),
    }
    Ok(())
}
