//! Analytic curve operations and machining sampling in millimeters.
use crate::model::{
    Arc, Geometry, Line, Plane, Point3, Polyline, PolylineSegment, Spline, Vector3,
};
use std::f64::consts::TAU;
pub const DISTANCE_TOLERANCE: f64 = 1e-6;
pub const ANGLE_TOLERANCE: f64 = 1e-8;
const MAX_POINTS: usize = 100_000;
#[derive(Clone, Copy, Debug)]
pub struct Sampling {
    pub chord: f64,
    pub angle: f64,
}
impl Default for Sampling {
    fn default() -> Self {
        Self {
            chord: 0.01,
            angle: 3f64.to_radians(),
        }
    }
}
pub(crate) fn delta(a: Point3, b: Point3) -> Vector3 {
    Vector3::new(b.x - a.x, b.y - a.y, b.z - a.z)
}
pub(crate) fn add(p: Point3, d: Vector3) -> Point3 {
    Point3::new(p.x + d.x, p.y + d.y, p.z + d.z)
}
pub fn distance(a: Point3, b: Point3) -> f64 {
    delta(a, b).length()
}
fn finite(p: Point3) -> bool {
    [p.x, p.y, p.z].into_iter().all(f64::is_finite)
}
pub(crate) fn arc_point(a: &Arc, t: f64) -> Point3 {
    let u = a.start_angle + a.sweep_angle * t;
    add(
        a.center,
        a.plane.x_axis * (a.radius * u.cos()) + a.plane.y_axis() * (a.radius * u.sin()),
    )
}
pub(crate) fn segment_geometry(
    s: &PolylineSegment,
    plane: Option<Plane>,
) -> Result<Geometry, String> {
    Ok(match *s {
        PolylineSegment::Line { start, end } => Geometry::Line(Line { start, end }),
        PolylineSegment::Arc {
            center,
            radius,
            start_angle,
            sweep_angle,
            ..
        } => Geometry::Arc(Arc {
            center,
            radius,
            start_angle,
            sweep_angle,
            plane: plane.ok_or("圆弧段缺少平面")?,
        }),
    })
}
pub fn evaluate(g: &Geometry, t: f64) -> Result<Point3, String> {
    if !t.is_finite() {
        return Err("曲线参数必须有限".into());
    }
    let t = t.clamp(0.0, 1.0);
    let p = match g {
        Geometry::Text(_) => return Err("文字不是可求值的切割曲线".into()),
        Geometry::Line(l) => add(l.start, delta(l.start, l.end) * t),
        Geometry::Arc(a) => arc_point(a, t),
        Geometry::Circle(c) => arc_point(
            &Arc {
                center: c.center,
                radius: c.radius,
                start_angle: 0.0,
                sweep_angle: TAU,
                plane: c.plane,
            },
            t,
        ),
        Geometry::Ellipse(e) => {
            let u = e.start_parameter + e.sweep_parameter * t;
            add(e.center, e.major_axis * u.cos() + e.minor_axis * u.sin())
        }
        Geometry::Spline(s) => {
            crate::snap::spline_point(s, t).ok_or("样条控制点/节点/权重无效，不能求值")?
        }
        Geometry::Polyline(p) => {
            if p.segments.is_empty() {
                return Err("多段线为空".into());
            }
            let u = t * p.segments.len() as f64;
            let i = (u.floor() as usize).min(p.segments.len() - 1);
            evaluate(&segment_geometry(&p.segments[i], p.plane)?, u - i as f64)?
        }
    };
    if finite(p) {
        Ok(p)
    } else {
        Err("曲线求值得到非有限坐标".into())
    }
}
pub fn nearest_parameter(g: &Geometry, p: Point3) -> Result<f64, String> {
    if let Geometry::Line(l) = g {
        let d = delta(l.start, l.end);
        let q = d.dot(d);
        if q < 1e-20 {
            return Err("零长线不能定位参数".into());
        }
        return Ok((delta(l.start, p).dot(d) / q).clamp(0.0, 1.0));
    }
    // Bounded numerical search for non-linear curves; not a proof of the global minimum.
    let divisions: usize = 128;
    let mut best = (f64::INFINITY, 0);
    for i in 0..=divisions {
        let d = distance(evaluate(g, i as f64 / divisions as f64)?, p);
        if d < best.0 {
            best = (d, i);
        }
    }
    let mut lo = best.1.saturating_sub(1) as f64 / divisions as f64;
    let mut hi = (best.1 + 1).min(divisions) as f64 / divisions as f64;
    for _ in 0..40 {
        let a = lo + (hi - lo) / 3.0;
        let b = hi - (hi - lo) / 3.0;
        if distance(evaluate(g, a)?, p) < distance(evaluate(g, b)?, p) {
            hi = b;
        } else {
            lo = a;
        }
    }
    let t = (lo + hi) / 2.0;
    let mut best = (f64::INFINITY, 0.0);
    for t in [0.0, t, 1.0] {
        let d = distance(evaluate(g, t)?, p);
        if d < best.0 {
            best = (d, t);
        }
    }
    Ok(best.1)
}

pub fn reverse(g: &Geometry) -> Result<Geometry, String> {
    let mut g = g.clone();
    match &mut g {
        Geometry::Text(_) => return Err("文字不支持曲线反向".into()),
        Geometry::Line(l) => std::mem::swap(&mut l.start, &mut l.end),
        Geometry::Arc(a) => {
            a.start_angle += a.sweep_angle;
            a.sweep_angle = -a.sweep_angle;
        }
        Geometry::Circle(c) => c.plane.normal = c.plane.normal * -1.0,
        Geometry::Ellipse(e) => {
            e.minor_axis = e.minor_axis * -1.0;
            e.start_parameter = -(e.start_parameter + e.sweep_parameter);
        }
        Geometry::Spline(s) => {
            s.control_points.reverse();
            s.fit_points.reverse();
            s.weights.reverse();
            let sum = s.knots.first().ok_or("样条节点为空")? + s.knots.last().unwrap();
            s.knots = s.knots.iter().rev().map(|k| sum - k).collect();
        }
        Geometry::Polyline(p) => {
            p.segments.reverse();
            for s in &mut p.segments {
                match s {
                    PolylineSegment::Line { start, end } => std::mem::swap(start, end),
                    PolylineSegment::Arc {
                        start,
                        end,
                        start_angle,
                        sweep_angle,
                        ..
                    } => {
                        std::mem::swap(start, end);
                        *start_angle += *sweep_angle;
                        *sweep_angle = -*sweep_angle;
                    }
                }
            }
        }
    }
    Ok(g)
}
pub fn slice(g: &Geometry, from: f64, to: f64) -> Result<Geometry, String> {
    if ![from, to].into_iter().all(f64::is_finite) || from < 0.0 || to > 1.0 || to - from < 1e-10 {
        return Err("保留参数须满足 0 ≤ 起始 < 结束 ≤ 1".into());
    }
    if from == 0.0 && to == 1.0 {
        return Ok(g.clone());
    }
    Ok(match g {
        Geometry::Line(_) => Geometry::Line(Line {
            start: evaluate(g, from)?,
            end: evaluate(g, to)?,
        }),
        Geometry::Arc(a) => Geometry::Arc(Arc {
            start_angle: a.start_angle + a.sweep_angle * from,
            sweep_angle: a.sweep_angle * (to - from),
            ..a.clone()
        }),
        Geometry::Circle(c) => Geometry::Arc(Arc {
            center: c.center,
            radius: c.radius,
            plane: c.plane,
            start_angle: TAU * from,
            sweep_angle: TAU * (to - from),
        }),
        Geometry::Polyline(p) => {
            let n = p.segments.len();
            let mut segments = Vec::new();
            for (i, s) in p.segments.iter().enumerate() {
                let a = (from * n as f64 - i as f64).max(0.0);
                let b = (to * n as f64 - i as f64).min(1.0);
                if b - a > 1e-10 {
                    segments.push(as_segment(&slice(&segment_geometry(s, p.plane)?, a, b)?)?);
                }
            }
            Geometry::Polyline(Polyline {
                segments,
                closed: false,
                plane: p.plane,
            })
        }
        _ => return Err("本批裁分仅支持直线、圆/圆弧和多段线".into()),
    })
}
pub(crate) fn as_segment(g: &Geometry) -> Result<PolylineSegment, String> {
    Ok(match g {
        Geometry::Line(l) => PolylineSegment::Line {
            start: l.start,
            end: l.end,
        },
        Geometry::Arc(a) => PolylineSegment::Arc {
            start: arc_point(a, 0.0),
            end: arc_point(a, 1.0),
            center: a.center,
            radius: a.radius,
            start_angle: a.start_angle,
            sweep_angle: a.sweep_angle,
        },
        _ => return Err("需要直线或圆弧段".into()),
    })
}
pub(crate) fn plane(g: &Geometry) -> Option<Plane> {
    match g {
        Geometry::Arc(a) => Some(a.plane),
        Geometry::Circle(c) => Some(c.plane),
        Geometry::Polyline(p) => p.plane,
        _ => None,
    }
}
fn circular(g: &Geometry) -> Option<Arc> {
    match g {
        Geometry::Arc(a) => Some(a.clone()),
        Geometry::Circle(c) => Some(Arc {
            center: c.center,
            radius: c.radius,
            plane: c.plane,
            start_angle: 0.0,
            sweep_angle: TAU,
        }),
        _ => None,
    }
}
fn parameter_unbounded(g: &Geometry, p: Point3) -> Option<f64> {
    match g {
        Geometry::Line(l) => {
            let d = delta(l.start, l.end);
            Some(delta(l.start, p).dot(d) / d.dot(d))
        }
        _ => {
            let a = circular(g)?;
            let d = delta(a.center, p);
            let angle = d.dot(a.plane.y_axis()).atan2(d.dot(a.plane.x_axis));
            let span = if a.sweep_angle > 0.0 {
                (angle - a.start_angle).rem_euclid(TAU)
            } else {
                -((a.start_angle - angle).rem_euclid(TAU))
            };
            Some(span / a.sweep_angle)
        }
    }
}
fn contains(g: &Geometry, p: Point3) -> bool {
    if [0.0, 1.0]
        .into_iter()
        .any(|t| evaluate(g, t).is_ok_and(|q| distance(q, p) <= DISTANCE_TOLERANCE))
    {
        return true;
    }
    parameter_unbounded(g, p).is_some_and(|t| (0.0..=1.0).contains(&t))
}

pub fn intersections(a: &Geometry, b: &Geometry, bounded: bool) -> Result<Vec<Point3>, String> {
    if let Geometry::Polyline(p) = a {
        let mut out = Vec::new();
        for s in &p.segments {
            out.extend(intersections(&segment_geometry(s, p.plane)?, b, bounded)?);
        }
        return Ok(out);
    }
    if let Geometry::Polyline(_) = b {
        return intersections(b, a, bounded);
    }
    let mut points = Vec::new();
    match (a, b) {
        (Geometry::Line(a), Geometry::Line(b)) => {
            let u = delta(a.start, a.end);
            let v = delta(b.start, b.end);
            let n = u.cross(v);
            let den = n.dot(n);
            if den > ANGLE_TOLERANCE.powi(2) * u.dot(u) * v.dot(v) {
                let w = delta(a.start, b.start);
                let t = w.cross(v).dot(n) / den;
                let s = w.cross(u).dot(n) / den;
                let p = add(a.start, u * t);
                if distance(p, add(b.start, v * s)) <= DISTANCE_TOLERANCE {
                    points.push(p);
                }
            }
        }
        (Geometry::Line(l), _) | (_, Geometry::Line(l)) => {
            let c = if matches!(a, Geometry::Line(_)) {
                circular(b)
            } else {
                circular(a)
            }
            .ok_or("求交暂不支持椭圆或样条")?;
            let d = delta(l.start, l.end);
            let w = delta(c.center, l.start);
            if w.dot(c.plane.normal).abs() > DISTANCE_TOLERANCE
                || d.dot(c.plane.normal).abs() > DISTANCE_TOLERANCE
            {
                return Ok(points);
            }
            let aa = d.dot(d);
            if aa < 1e-20 {
                return Err("零长线不能求交".into());
            }
            let middle = -w.dot(d) / aa;
            let closest = add(l.start, d * middle);
            let h2 = c.radius * c.radius - distance(closest, c.center).powi(2);
            if h2 >= -DISTANCE_TOLERANCE * c.radius.max(1.0) {
                let h = (h2.max(0.0) / aa).sqrt();
                points.push(add(l.start, d * (middle - h)));
                if h > 1e-12 {
                    points.push(add(l.start, d * (middle + h)));
                }
            }
        }
        _ => {
            let a = circular(a).ok_or("求交暂不支持椭圆或样条")?;
            let b = circular(b).ok_or("求交暂不支持椭圆或样条")?;
            let d = delta(a.center, b.center);
            let length = d.length();
            if a.plane.normal.cross(b.plane.normal).length() > ANGLE_TOLERANCE
                || d.dot(a.plane.normal).abs() > DISTANCE_TOLERANCE
            {
                return Ok(points);
            }
            if length <= DISTANCE_TOLERANCE {
                return Ok(points);
            }
            if length > a.radius + b.radius + DISTANCE_TOLERANCE
                || length < (a.radius - b.radius).abs() - DISTANCE_TOLERANCE
            {
                return Ok(points);
            }
            let along =
                (a.radius * a.radius - b.radius * b.radius + length * length) / (2.0 * length);
            let h2 = a.radius * a.radius - along * along;
            if h2 >= -DISTANCE_TOLERANCE * a.radius.max(1.0) {
                let center = add(a.center, d * (along / length));
                let side = a.plane.normal.cross(d * (1.0 / length)) * h2.max(0.0).sqrt();
                points.push(add(center, side));
                if side.length() > DISTANCE_TOLERANCE {
                    points.push(add(center, side * -1.0));
                }
            }
        }
    }
    if bounded {
        points.retain(|p| contains(a, *p) && contains(b, *p));
    }
    Ok(points)
}

pub fn sample(g: &Geometry, settings: Sampling) -> Result<Vec<Point3>, String> {
    if !settings.chord.is_finite()
        || settings.chord <= 0.0
        || !settings.angle.is_finite()
        || !(1e-6..=0.5).contains(&settings.angle)
    {
        return Err("加工采样公差或角度无效".into());
    }
    let count = |radius: f64, sweep: f64, ratio: f64| -> Result<usize, String> {
        if !radius.is_finite() || radius <= 0.0 || !sweep.is_finite() {
            return Err("无效曲线半径/扫角".into());
        }
        let chord_step = (8.0 * settings.chord / radius).sqrt().min(0.5);
        let step = chord_step.min(settings.angle * ratio);
        let n = (sweep.abs() / step).ceil().max(1.0);
        if n >= MAX_POINTS as f64 {
            return Err("加工曲线超出单图元 100000 点预算；请检查尺度与几何".into());
        }
        Ok(n as usize)
    };
    let mut out = match g {
        Geometry::Text(_) => return Err("文字不参加加工采样".into()),
        Geometry::Line(l) => vec![l.start, l.end],
        Geometry::Circle(c) => {
            let n = count(c.radius, TAU, 1.0)?;
            (0..=n)
                .map(|i| evaluate(g, i as f64 / n as f64))
                .collect::<Result<Vec<_>, _>>()?
        }
        Geometry::Arc(a) => {
            let n = count(a.radius, a.sweep_angle, 1.0)?;
            (0..=n)
                .map(|i| evaluate(g, i as f64 / n as f64))
                .collect::<Result<Vec<_>, _>>()?
        }
        Geometry::Ellipse(e) => {
            let a = e.major_axis.length().max(e.minor_axis.length());
            let b = e.major_axis.length().min(e.minor_axis.length());
            if b <= 0.0 {
                return Err("椭圆短轴无效".into());
            }
            let n = count(a, e.sweep_parameter, b / a)?;
            (0..=n)
                .map(|i| evaluate(g, i as f64 / n as f64))
                .collect::<Result<Vec<_>, _>>()?
        }
        Geometry::Polyline(p) => {
            let mut points = Vec::new();
            for s in &p.segments {
                let part = sample(&segment_geometry(s, p.plane)?, settings)?;
                if let Some(last) = points.last() {
                    if distance(*last, part[0]) > DISTANCE_TOLERANCE {
                        return Err("多段线段之间不连续".into());
                    }
                }
                points.extend(part.into_iter().skip(usize::from(!points.is_empty())));
                if points.len() > MAX_POINTS {
                    return Err("多段线采样超出点数预算".into());
                }
            }
            points
        }
        Geometry::Spline(s) => sample_spline(s, settings)?,
    };
    if out.len() < 2 || out.iter().any(|p| !finite(*p)) {
        return Err("加工采样为空或含非有限坐标".into());
    }
    let closed = match g {
        Geometry::Circle(_) => true,
        Geometry::Polyline(p) => p.closed,
        Geometry::Ellipse(e) => e.sweep_parameter.abs() >= TAU - ANGLE_TOLERANCE,
        Geometry::Spline(s) => s.closed,
        _ => false,
    };
    if closed {
        let first = out[0];
        if distance(first, *out.last().unwrap()) > DISTANCE_TOLERANCE {
            return Err("闭合曲线首尾不一致，不能补线冒充原曲线".into());
        }
        *out.last_mut().unwrap() = first;
    }
    Ok(out)
}
fn sample_spline(s: &Spline, settings: Sampling) -> Result<Vec<Point3>, String> {
    let p = s.degree as usize;
    let n = s.control_points.len();
    if p == 0
        || p > 16
        || n <= p
        || s.knots.len() != n + p + 1
        || s.knots.windows(2).any(|k| !k[0].is_finite() || k[0] > k[1])
        || s.knots.last().is_none_or(|v| !v.is_finite())
        || s.control_points.iter().any(|p| !finite(*p))
        || (!s.weights.is_empty()
            && (s.weights.len() != n || s.weights.iter().any(|w| !w.is_finite() || *w <= 0.0)))
    {
        return Err("加工样条需要有效节点、控制点及正权重".into());
    }
    let lo = s.knots[p];
    let hi = s.knots[n];
    if s.periodic
        || hi <= lo
        || s.knots[..=p].iter().any(|u| *u != lo)
        || s.knots[n..].iter().any(|u| *u != hi)
    {
        return Err("加工样条暂支持夹持、非周期 NURBS；未以控制多边形代替加工曲线".into());
    }
    let mut knots = s.knots.clone();
    let mut control: Vec<[f64; 4]> = s
        .control_points
        .iter()
        .enumerate()
        .map(|(i, q)| {
            let w = s.weights.get(i).copied().unwrap_or(1.0);
            [q.x * w, q.y * w, q.z * w, w]
        })
        .collect();
    let mut unique = Vec::new();
    for &u in &s.knots[p + 1..n] {
        if u > lo && u < hi && unique.last() != Some(&u) {
            unique.push(u);
        }
    }
    let insertions: usize = unique
        .iter()
        .map(|u| p.saturating_sub(knots.iter().filter(|v| *v == u).count()))
        .sum();
    if knots.len().saturating_mul(insertions) > 10_000_000 {
        return Err("样条分解工作量超出预算；未输出近似控制多边形".into());
    }
    for u in unique {
        let multiplicity = knots.iter().filter(|v| **v == u).count();
        if multiplicity > p {
            return Err("样条内部节点存在断裂".into());
        }
        for _ in multiplicity..p {
            let k = (p..control.len())
                .find(|i| knots[*i] <= u && u < knots[*i + 1])
                .ok_or("样条节点跨度无效")?;
            let m = knots.iter().filter(|v| **v == u).count();
            let mut next = vec![[0.0; 4]; control.len() + 1];
            next[..=k - p].copy_from_slice(&control[..=k - p]);
            for i in k - m..control.len() {
                next[i + 1] = control[i];
            }
            for i in k - p + 1..=k - m {
                let a = (u - knots[i]) / (knots[i + p] - knots[i]);
                for j in 0..4 {
                    next[i][j] = a * control[i][j] + (1.0 - a) * control[i - 1][j];
                }
            }
            knots.insert(k + 1, u);
            control = next;
            if control.len() > MAX_POINTS {
                return Err("样条分解超出点数预算".into());
            }
        }
    }
    let mut out = Vec::new();
    for k in p..control.len() {
        if knots[k + 1] > knots[k] {
            sample_bezier(&control[k - p..=k], settings, 0, &mut out)?;
        }
    }
    Ok(out)
}
fn sample_bezier(
    control: &[[f64; 4]],
    settings: Sampling,
    depth: u32,
    out: &mut Vec<Point3>,
) -> Result<(), String> {
    let points: Vec<_> = control
        .iter()
        .map(|q| Point3::new(q[0] / q[3], q[1] / q[3], q[2] / q[3]))
        .collect();
    if points.iter().any(|p| !finite(*p)) {
        return Err("样条齐次求值非有限".into());
    }
    let a = points[0];
    let b = *points.last().unwrap();
    let chord = delta(a, b);
    let len = chord.length();
    let near = |p: Point3| {
        let t = if len > 1e-15 {
            delta(a, p).dot(chord) / (len * len)
        } else {
            0.0
        };
        distance(p, add(a, chord * t.clamp(0.0, 1.0))) <= settings.chord
    };
    let cone = (settings.angle * 0.5).cos();
    let flat = len > 1e-12
        && points.iter().all(|p| near(*p))
        && points.windows(2).all(|w| {
            let d = delta(w[0], w[1]);
            d.length() < 1e-12 || d.dot(chord) >= cone * d.length() * len
        });
    if flat {
        if out.is_empty() {
            out.push(a);
        } else if distance(*out.last().unwrap(), a) > DISTANCE_TOLERANCE {
            return Err("样条分段不连续".into());
        }
        out.push(b);
        if out.len() > MAX_POINTS {
            return Err("样条采样超出点数预算".into());
        }
        return Ok(());
    }
    if depth >= 24 {
        return Err("样条在当前公差内无法稳定离散（可能存在尖点或退化段）".into());
    }
    let n = control.len();
    let mut work = control.to_vec();
    let mut left = vec![work[0]];
    let mut right = vec![work[n - 1]];
    for r in 1..n {
        for i in 0..n - r {
            for j in 0..4 {
                work[i][j] = (work[i][j] + work[i + 1][j]) * 0.5;
            }
        }
        left.push(work[0]);
        right.push(work[n - r - 1]);
    }
    right.reverse();
    sample_bezier(&left, settings, depth + 1, out)?;
    sample_bezier(&right, settings, depth + 1, out)
}
