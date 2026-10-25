//! Small measurement/drafting primitives. Dimensions are deliberately non-associative.
use crate::model::{Geometry, Line, Plane, Point3, Text, Vector3, WorkPlane};

pub const ANNOTATION_LAYER: &str = "_CADCAM_ANNOTATION";
pub fn is_annotation_layer(name: &str) -> bool {
    name.eq_ignore_ascii_case(ANNOTATION_LAYER)
}
fn add(p: Point3, d: Vector3) -> Point3 {
    Point3::new(p.x + d.x, p.y + d.y, p.z + d.z)
}
fn v(a: Point3, b: Point3) -> Vector3 {
    Vector3::new(b.x - a.x, b.y - a.y, b.z - a.z)
}

pub fn angle(a: Point3, vertex: Point3, b: Point3) -> Result<f64, String> {
    let u = v(vertex, a);
    let w = v(vertex, b);
    if u.length() <= 1e-9
        || w.length() <= 1e-9
        || !u.length().is_finite()
        || !w.length().is_finite()
    {
        return Err("角度两边不能为零长".into());
    }
    Ok(u.normalize()
        .dot(w.normalize())
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees())
}
pub fn text(
    position: Point3,
    content: String,
    height: f64,
    degrees: f64,
    plane: WorkPlane,
) -> Result<Geometry, String> {
    if content.trim().is_empty()
        || content.contains(['\n', '\r'])
        || !height.is_finite()
        || height <= 0.0
        || !degrees.is_finite()
    {
        return Err("请输入单行文字、正数字高和有限角度".into());
    }
    let a = degrees.to_radians();
    Ok(Geometry::Text(Text {
        position,
        content,
        height,
        plane: Plane {
            normal: plane.normal(),
            x_axis: plane.x_axis * a.cos() + plane.y_axis * a.sin(),
        },
    }))
}
pub fn aligned_dimension(
    a: Point3,
    b: Point3,
    offset: Point3,
    height: f64,
    plane: WorkPlane,
) -> Result<Vec<Geometry>, String> {
    if [a, b, offset]
        .iter()
        .any(|p| plane.to_local(*p)[2].abs() > 1e-6)
    {
        return Err("对齐标注的三点须在当前工作平面内".into());
    }
    let d = v(a, b);
    let len = d.length();
    if !len.is_finite() || len <= 1e-9 || !height.is_finite() || height <= 0.0 {
        return Err("标注长度和字高须大于零".into());
    }
    let u = d.normalize();
    let n = plane.normal().cross(u);
    let gap = v(a, offset).dot(n);
    let p = add(a, n * gap);
    let q = add(b, n * gap);
    let size = (height * 0.6).min(len * 0.2);
    let mut out = vec![];
    let mut line = |start, end| out.push(Geometry::Line(Line { start, end }));
    line(a, add(p, n * (height * 0.5 * gap.signum())));
    line(b, add(q, n * (height * 0.5 * gap.signum())));
    line(p, q);
    for (point, dir) in [(p, u), (q, u * (-1.0))] {
        line(point, add(add(point, dir * size), n * (size * 0.35)));
        line(point, add(add(point, dir * size), n * (-size * 0.35)));
    }
    let label = format!("{len:.3} mm");
    out.push(Geometry::Text(Text {
        position: add(
            add(p, u * (len * 0.5 - label.len() as f64 * height * 0.3)),
            n * (height * 0.3),
        ),
        content: label,
        height,
        plane: Plane {
            normal: plane.normal(),
            x_axis: u,
        },
    }));
    Ok(out)
}
pub fn hole(center: Point3, diameter: f64, plane: WorkPlane) -> Result<Vec<Geometry>, String> {
    if !diameter.is_finite() || diameter <= 0.0 {
        return Err("孔径须为正数".into());
    }
    Ok(vec![Geometry::Circle(crate::model::Circle {
        center,
        radius: diameter * 0.5,
        plane: Plane {
            normal: plane.normal(),
            x_axis: plane.x_axis,
        },
    })])
}
/// Two end-center points define the slot axis; diameter is its width.
pub fn slot(
    a: Point3,
    b: Point3,
    diameter: f64,
    plane: WorkPlane,
) -> Result<Vec<Geometry>, String> {
    if [a, b].iter().any(|p| plane.to_local(*p)[2].abs() > 1e-6) {
        return Err("长圆孔中心须在工作平面内".into());
    }
    let u = v(a, b).normalize();
    if v(a, b).length() <= 1e-9 || !diameter.is_finite() || diameter <= 0.0 {
        return Err("端圆中心不能重合，孔宽须大于零".into());
    }
    let n = plane.normal().cross(u);
    let r = diameter * 0.5;
    let basis = Plane {
        normal: plane.normal(),
        x_axis: u,
    };
    let p = add(a, n * r);
    let q = add(b, n * r);
    let t = add(b, n * (-r));
    let s = add(a, n * (-r));
    use crate::model::PolylineSegment as S;
    Ok(vec![Geometry::Polyline(crate::model::Polyline {
        closed: true,
        plane: Some(basis),
        segments: vec![
            S::Line { start: p, end: q },
            S::Arc {
                start: q,
                end: t,
                center: b,
                radius: r,
                start_angle: std::f64::consts::FRAC_PI_2,
                sweep_angle: -std::f64::consts::PI,
            },
            S::Line { start: t, end: s },
            S::Arc {
                start: s,
                end: p,
                center: a,
                radius: r,
                start_angle: -std::f64::consts::FRAC_PI_2,
                sweep_angle: -std::f64::consts::PI,
            },
        ],
    })])
}
pub fn translated(g: &Geometry, offset: [f64; 3]) -> Result<Geometry, String> {
    if !offset.iter().all(|v| v.is_finite()) {
        return Err("位移必须有限".into());
    }
    let mut g = g.clone();
    let shift = |p: &mut Point3| {
        p.x += offset[0];
        p.y += offset[1];
        p.z += offset[2];
    };
    match &mut g {
        Geometry::Text(t) => shift(&mut t.position),
        Geometry::Line(l) => {
            shift(&mut l.start);
            shift(&mut l.end);
        }
        Geometry::Circle(c) => shift(&mut c.center),
        Geometry::Arc(a) => shift(&mut a.center),
        Geometry::Ellipse(e) => shift(&mut e.center),
        Geometry::Spline(s) => {
            for p in s.control_points.iter_mut().chain(&mut s.fit_points) {
                shift(p);
            }
        }
        Geometry::Polyline(p) => {
            for s in &mut p.segments {
                match s {
                    crate::model::PolylineSegment::Line { start, end } => {
                        shift(start);
                        shift(end);
                    }
                    crate::model::PolylineSegment::Arc {
                        start, end, center, ..
                    } => {
                        shift(start);
                        shift(end);
                        shift(center);
                    }
                }
            }
        }
    }
    Ok(g)
}
