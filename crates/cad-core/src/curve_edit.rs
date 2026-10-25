//! First contour tools: line, circular arc and polyline geometry. Results are preview recipes.
use crate::{
    curves::{
        self, add, arc_point, as_segment, delta, segment_geometry, ANGLE_TOLERANCE,
        DISTANCE_TOLERANCE as EPS,
    },
    model::{Geometry, Line, Plane, Point3, Polyline, PolylineSegment, Vector3},
};
use std::f64::consts::{PI, TAU};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CurveTool {
    Break,
    Trim,
    Extend,
    Join,
    Close,
    Chamfer,
    Fillet,
    Offset,
    Reverse,
}
impl CurveTool {
    pub fn label(self) -> &'static str {
        match self {
            Self::Break => "打断",
            Self::Trim => "参数修剪",
            Self::Extend => "定长延伸",
            Self::Join => "连接",
            Self::Close => "闭合",
            Self::Chamfer => "倒角",
            Self::Fillet => "圆角",
            Self::Offset => "偏移",
            Self::Reverse => "反向",
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub struct CurveOptions {
    pub from: f64,
    pub to: f64,
    pub length: f64,
    pub at_start: bool,
    pub tolerance: f64,
    pub plane: Plane,
}
impl Default for CurveOptions {
    fn default() -> Self {
        Self {
            from: 0.25,
            to: 0.75,
            length: 1.0,
            at_start: false,
            tolerance: 0.001,
            plane: Plane::new(Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0)),
        }
    }
}
fn positive(v: f64) -> Result<f64, String> {
    if v.is_finite() && v > EPS {
        Ok(v)
    } else {
        Err("长度或半径须大于几何容差且有限".into())
    }
}
pub fn apply(
    tool: CurveTool,
    input: &[Geometry],
    o: CurveOptions,
) -> Result<Vec<Geometry>, String> {
    if input.is_empty() {
        return Err("请先选择图元".into());
    }
    if !o.tolerance.is_finite() || o.tolerance < EPS {
        return Err("连接容差不能小于 0.000001 mm".into());
    }
    if tool == CurveTool::Join {
        return Ok(vec![join(input, o)?]);
    }
    if matches!(tool, CurveTool::Chamfer | CurveTool::Fillet) {
        return Ok(vec![corner(input, o, tool == CurveTool::Fillet)?]);
    }
    let mut output = Vec::new();
    for g in input {
        match tool {
            CurveTool::Break => {
                if !(1e-8..1.0 - 1e-8).contains(&o.from) {
                    return Err("打断参数须在 0 与 1 之间".into());
                }
                output.push(curves::slice(g, 0.0, o.from)?);
                output.push(curves::slice(g, o.from, 1.0)?);
            }
            CurveTool::Trim => output.push(curves::slice(g, o.from, o.to)?),
            CurveTool::Extend => output.push(extend(g, o.length, o.at_start)?),
            CurveTool::Close => output.push(close(g)?),
            CurveTool::Offset => output.push(offset(g, o.length, o.plane)?),
            CurveTool::Reverse => output.push(curves::reverse(g)?),
            _ => unreachable!(),
        }
    }
    Ok(output)
}
fn extend(g: &Geometry, length: f64, start: bool) -> Result<Geometry, String> {
    positive(length)?;
    Ok(match g {
        Geometry::Line(l) => {
            let mut l = l.clone();
            let d = delta(l.start, l.end);
            let len = positive(d.length())?;
            if start {
                l.start = add(l.start, d * (-length / len));
            } else {
                l.end = add(l.end, d * (length / len));
            }
            Geometry::Line(l)
        }
        Geometry::Arc(a) => {
            let mut a = a.clone();
            let extra = length / positive(a.radius)? * a.sweep_angle.signum();
            if (a.sweep_angle + extra).abs() >= TAU - ANGLE_TOLERANCE {
                return Err("延伸后的圆弧不能达到或超过整圆".into());
            }
            if start {
                a.start_angle -= extra;
            }
            a.sweep_angle += extra;
            Geometry::Arc(a)
        }
        Geometry::Polyline(p) => {
            if p.closed || p.segments.is_empty() {
                return Err("只能延伸开放多段线".into());
            }
            let mut p = p.clone();
            let i = if start { 0 } else { p.segments.len() - 1 };
            let g = segment_geometry(&p.segments[i], p.plane)?;
            p.segments[i] = as_segment(&extend(&g, length, start)?)?;
            Geometry::Polyline(p)
        }
        _ => return Err("定长延伸支持直线、非整圆圆弧及开放多段线".into()),
    })
}
fn vertices(s: &PolylineSegment) -> (Point3, Point3) {
    match s {
        PolylineSegment::Line { start, end } | PolylineSegment::Arc { start, end, .. } => {
            (*start, *end)
        }
    }
}
fn adjust(
    s: &PolylineSegment,
    start: Point3,
    end: Point3,
    plane: Option<Plane>,
) -> Result<PolylineSegment, String> {
    let chord = delta(start, end);
    let len = positive(chord.length())?;
    Ok(match s {
        PolylineSegment::Line { .. } => PolylineSegment::Line { start, end },
        PolylineSegment::Arc { sweep_angle, .. } => {
            let plane = plane.ok_or("圆弧需要共面条件")?;
            let offset = len / (2.0 * (*sweep_angle / 2.0).tan());
            let center = add(
                add(start, chord * 0.5),
                plane.normal.cross(chord * (1.0 / len)) * offset,
            );
            let d = delta(center, start);
            PolylineSegment::Arc {
                start,
                end,
                center,
                radius: d.length(),
                start_angle: d.dot(plane.y_axis()).atan2(d.dot(plane.x_axis)),
                sweep_angle: *sweep_angle,
            }
        }
    })
}
fn segments(g: &Geometry, plane: Option<Plane>) -> Result<Vec<PolylineSegment>, String> {
    match g {
        Geometry::Line(l) => Ok(vec![PolylineSegment::Line {
            start: l.start,
            end: l.end,
        }]),
        Geometry::Arc(a) => {
            if a.sweep_angle.abs() >= TAU - ANGLE_TOLERANCE {
                return Err("整圆弧不能作为连接端".into());
            }
            let target = plane.ok_or("圆弧连接缺少平面")?;
            if a.plane.normal.cross(target.normal).length() > ANGLE_TOLERANCE {
                return Err("圆弧必须共面连接".into());
            }
            let mut a = a.clone();
            let d = delta(a.center, arc_point(&a, 0.0));
            a.start_angle = d.dot(target.y_axis()).atan2(d.dot(target.x_axis));
            if a.plane.normal.dot(target.normal) < 0.0 {
                a.sweep_angle = -a.sweep_angle;
            }
            a.plane = target;
            Ok(vec![as_segment(&Geometry::Arc(a))?])
        }
        Geometry::Polyline(p) => {
            if p.closed {
                return Err("闭合多段线没有连接端点".into());
            }
            p.segments
                .iter()
                .map(|s| segment_geometry(s, p.plane).and_then(|g| segments(&g, plane)))
                .collect::<Result<Vec<_>, _>>()
                .map(|v| v.into_iter().flatten().collect())
        }
        _ => Err("连接/闭合只支持直线、圆弧和多段线".into()),
    }
}
fn is_closed(g: &Geometry) -> bool {
    match g {
        Geometry::Circle(_) => true,
        Geometry::Polyline(p) => p.closed,
        _ => false,
    }
}
fn join(input: &[Geometry], o: CurveOptions) -> Result<Geometry, String> {
    if input.len() < 2 {
        return Err("连接至少需要两个开放对象".into());
    }
    if input.iter().any(is_closed) {
        return Err("闭合对象不能参与开放端点连接".into());
    }
    let plane = input.iter().find_map(curves::plane).or(Some(o.plane));
    let mut pieces: Vec<Vec<PolylineSegment>> = input
        .iter()
        .map(|g| segments(g, plane))
        .collect::<Result<_, _>>()?;
    if pieces.iter().any(Vec::is_empty) {
        return Err("存在空多段线".into());
    }
    let endpoints: Vec<_> = pieces
        .iter()
        .map(|s| (vertices(&s[0]).0, vertices(s.last().unwrap()).1))
        .collect();
    let degrees: Vec<_> = endpoints
        .iter()
        .enumerate()
        .map(|(i, (a, b))| {
            [*a, *b].map(|p| {
                endpoints
                    .iter()
                    .enumerate()
                    .filter(|(j, (c, d))| {
                        *j != i
                            && (curves::distance(p, *c) <= o.tolerance
                                || curves::distance(p, *d) <= o.tolerance)
                    })
                    .count()
            })
        })
        .collect();
    if degrees.iter().flatten().any(|degree| *degree > 1) {
        return Err("端点有多个连接候选，存在分支或重复对象，请分批选择".into());
    }
    let (first, flip) = degrees
        .iter()
        .enumerate()
        .find_map(|(i, d)| {
            if d[0] == 0 {
                Some((i, false))
            } else if d[1] == 0 {
                Some((i, true))
            } else {
                None
            }
        })
        .unwrap_or((0, false));
    let mut result = if flip {
        segments(&curves::reverse(&input[first])?, plane)?
    } else {
        std::mem::take(&mut pieces[first])
    };
    let mut used = vec![false; input.len()];
    used[first] = true;
    for _ in 1..input.len() {
        let end = vertices(result.last().unwrap()).1;
        let next = endpoints
            .iter()
            .enumerate()
            .filter(|(i, _)| !used[*i])
            .find_map(|(i, (a, b))| {
                if curves::distance(end, *a) <= o.tolerance {
                    Some((i, false))
                } else if curves::distance(end, *b) <= o.tolerance {
                    Some((i, true))
                } else {
                    None
                }
            })
            .ok_or("所选对象不能连接成单一连续链")?;
        let mut part = if next.1 {
            segments(&curves::reverse(&input[next.0])?, plane)?
        } else {
            pieces[next.0].clone()
        };
        let last = vertices(&part[0]).1;
        part[0] = adjust(&part[0], end, last, plane)?;
        result.extend(part);
        used[next.0] = true;
    }
    let start = vertices(&result[0]).0;
    let end = vertices(result.last().unwrap()).1;
    let closed = curves::distance(start, end) <= o.tolerance;
    if closed {
        let i = result.len() - 1;
        result[i] = adjust(&result[i], vertices(&result[i]).0, start, plane)?;
    }
    let plane = derive_plane(&result, plane)?;
    Ok(Geometry::Polyline(Polyline {
        segments: result,
        closed,
        plane,
    }))
}
fn derive_plane(
    segments: &[PolylineSegment],
    candidate: Option<Plane>,
) -> Result<Option<Plane>, String> {
    let has_arc = segments
        .iter()
        .any(|s| matches!(s, PolylineSegment::Arc { .. }));
    if let Some(plane) = candidate {
        let origin = vertices(&segments[0]).0;
        if segments.iter().all(|s| {
            let (a, b) = vertices(s);
            delta(origin, a).dot(plane.normal).abs() <= EPS
                && delta(origin, b).dot(plane.normal).abs() <= EPS
        }) {
            return Ok(Some(plane));
        }
    }
    if has_arc {
        Err("带圆弧的连接结果必须共面".into())
    } else {
        Ok(None)
    }
}
fn close(g: &Geometry) -> Result<Geometry, String> {
    if is_closed(g) {
        return Ok(g.clone());
    }
    let plane = curves::plane(g);
    let mut s = segments(g, plane)?;
    if s.len() == 1 && matches!(s[0], PolylineSegment::Line { .. }) {
        return Err("单条直线无法形成有效闭合轮廓".into());
    }
    let start = vertices(&s[0]).0;
    let end = vertices(s.last().unwrap()).1;
    if curves::distance(start, end) > EPS {
        s.push(PolylineSegment::Line {
            start: end,
            end: start,
        });
    } else {
        let i = s.len() - 1;
        s[i] = adjust(&s[i], vertices(&s[i]).0, start, plane)?;
    }
    if s.len() < 2 {
        return Err("单条直线无法形成有效闭合轮廓".into());
    }
    Ok(Geometry::Polyline(Polyline {
        segments: s,
        closed: true,
        plane,
    }))
}
fn corner(input: &[Geometry], o: CurveOptions, fillet: bool) -> Result<Geometry, String> {
    let [Geometry::Line(a), Geometry::Line(b)] = input else {
        return Err("倒角/圆角首批支持两条在端点相接的直线".into());
    };
    let mut pair = (f64::INFINITY, a.start, a.end, b.start, b.end);
    for (p, q) in [(a.start, a.end), (a.end, a.start)] {
        for (r, s) in [(b.start, b.end), (b.end, b.start)] {
            let d = curves::distance(p, r);
            if d < pair.0 {
                pair = (d, p, q, r, s);
            }
        }
    }
    if pair.0 > o.tolerance {
        return Err("两条直线须在端点处连接，不能猜测内角侧".into());
    }
    let center = add(pair.1, delta(pair.1, pair.3) * 0.5);
    let u = delta(center, pair.2);
    let v = delta(center, pair.4);
    let lu = positive(u.length())?;
    let lv = positive(v.length())?;
    let u = u * (1.0 / lu);
    let v = v * (1.0 / lv);
    let normal = u.cross(v);
    if normal.length() <= ANGLE_TOLERANCE {
        return Err("倒角/圆角的两条边不能共线".into());
    }
    let theta = u.dot(v).clamp(-1.0, 1.0).acos();
    let value = positive(o.length)?;
    let d = if fillet {
        value / (theta / 2.0).tan()
    } else {
        value
    };
    if d >= lu - EPS || d >= lv - EPS {
        return Err("倒角距离/圆角半径过大，超过原线段范围".into());
    }
    let p = add(center, u * d);
    let q = add(center, v * d);
    let plane = Plane::new(normal, u);
    let middle = if fillet {
        let c = add(center, (u + v).normalize() * (value / (theta / 2.0).sin()));
        let r = delta(c, p);
        PolylineSegment::Arc {
            start: p,
            end: q,
            center: c,
            radius: value,
            start_angle: r.dot(plane.y_axis()).atan2(r.dot(plane.x_axis)),
            sweep_angle: -(PI - theta),
        }
    } else {
        PolylineSegment::Line { start: p, end: q }
    };
    Ok(Geometry::Polyline(Polyline {
        segments: vec![
            PolylineSegment::Line {
                start: pair.2,
                end: p,
            },
            middle,
            PolylineSegment::Line {
                start: q,
                end: pair.4,
            },
        ],
        closed: false,
        plane: Some(plane),
    }))
}
fn offset(g: &Geometry, d: f64, plane: Plane) -> Result<Geometry, String> {
    if !d.is_finite() {
        return Err("偏移距离必须有限".into());
    }
    if d == 0.0 {
        return Ok(g.clone());
    }
    Ok(match g {
        Geometry::Line(l) => {
            let direction = delta(l.start, l.end);
            let length = positive(direction.length())?;
            if direction.dot(plane.normal).abs() > EPS {
                return Err("直线偏移要求当前工作平面与该直线平行".into());
            }
            let shift = plane.normal.cross(direction * (1.0 / length)) * d;
            Geometry::Line(Line {
                start: add(l.start, shift),
                end: add(l.end, shift),
            })
        }
        Geometry::Circle(c) => {
            let mut c = c.clone();
            c.radius = positive(c.radius + d)?;
            Geometry::Circle(c)
        }
        Geometry::Arc(a) => {
            let mut a = a.clone();
            a.radius = positive(a.radius + d)?;
            Geometry::Arc(a)
        }
        Geometry::Polyline(p) => {
            let plane = p.plane.ok_or("空间多段线暂不支持平面偏移")?;
            let mut pieces = Vec::new();
            for s in &p.segments {
                let original = segment_geometry(s, Some(plane))?;
                let part = match &original {
                    Geometry::Arc(a) => {
                        let mut a = a.clone();
                        a.radius = positive(a.radius - d * a.sweep_angle.signum())?;
                        Geometry::Arc(a)
                    }
                    _ => offset(&original, d, plane)?,
                };
                pieces.push(part);
            }
            if pieces.is_empty() {
                return Err("多段线为空".into());
            }
            let mut starts: Vec<_> = pieces
                .iter()
                .map(|g| curves::evaluate(g, 0.0))
                .collect::<Result<_, _>>()?;
            let mut ends: Vec<_> = pieces
                .iter()
                .map(|g| curves::evaluate(g, 1.0))
                .collect::<Result<_, _>>()?;
            let count = pieces.len();
            let joins = if p.closed { count } else { count - 1 };
            for i in 0..joins {
                let j = (i + 1) % count;
                let vertex = vertices(&p.segments[i]).1;
                let point = if curves::distance(ends[i], starts[j]) <= EPS {
                    ends[i]
                } else {
                    curves::intersections(&pieces[i], &pieces[j], false)?
                        .into_iter()
                        .min_by(|a, b| {
                            curves::distance(*a, vertex).total_cmp(&curves::distance(*b, vertex))
                        })
                        .ok_or("相邻偏移段没有有效连接；未截断或丢弃局部轮廓")?
                };
                ends[i] = point;
                starts[j] = point;
            }
            let mut result = Vec::new();
            for i in 0..count {
                let part = match &pieces[i] {
                    Geometry::Line(original) => {
                        if delta(starts[i], ends[i]).dot(delta(original.start, original.end)) <= 0.0
                        {
                            return Err("偏移超过局部尺度，导致边反向或轮廓坍缩".into());
                        }
                        Geometry::Line(Line {
                            start: starts[i],
                            end: ends[i],
                        })
                    }
                    Geometry::Arc(a) => {
                        let mut a = a.clone();
                        let u = delta(a.center, starts[i]);
                        let v = delta(a.center, ends[i]);
                        let start = u.dot(plane.y_axis()).atan2(u.dot(plane.x_axis));
                        let end = v.dot(plane.y_axis()).atan2(v.dot(plane.x_axis));
                        let sweep = if a.sweep_angle > 0.0 {
                            (end - start).rem_euclid(TAU)
                        } else {
                            -((start - end).rem_euclid(TAU))
                        };
                        if !sweep.is_finite() || sweep.abs() <= ANGLE_TOLERANCE {
                            return Err("偏移后的圆弧角度退化".into());
                        }
                        if sweep.abs() > a.sweep_angle.abs() + PI {
                            return Err("偏移连接导致圆弧绕过大半圈，已取消".into());
                        }
                        a.start_angle = start;
                        a.sweep_angle = sweep;
                        Geometry::Arc(a)
                    }
                    _ => unreachable!(),
                };
                positive(curves::distance(starts[i], ends[i]))?;
                result.push(as_segment(&part)?);
            }
            for i in 0..count {
                for j in i + 1..count {
                    if j == i + 1 || p.closed && i == 0 && j + 1 == count {
                        continue;
                    }
                    if !curves::intersections(
                        &segment_geometry(&result[i], Some(plane))?,
                        &segment_geometry(&result[j], Some(plane))?,
                        true,
                    )?
                    .is_empty()
                    {
                        return Err("偏移结果自交，已取消整次偏移；请减小距离或先整理轮廓".into());
                    }
                }
            }
            Geometry::Polyline(Polyline {
                segments: result,
                closed: p.closed,
                plane: Some(plane),
            })
        }
        _ => return Err("偏移首批支持直线、圆/圆弧及平面多段线".into()),
    })
}
