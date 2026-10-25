//! Analytic geometry for boundary/side picking. Never modifies a drawing.
use crate::{
    curve_edit::{self, CurveOptions, CurveTool},
    curves::{self, delta, segment_geometry, DISTANCE_TOLERANCE as EPS},
    model::{Arc, Geometry, Line, Point3, Polyline},
};
use std::f64::consts::TAU;

fn supported(g: &Geometry) -> Result<(), String> {
    if matches!(
        g,
        Geometry::Line(_) | Geometry::Arc(_) | Geometry::Circle(_) | Geometry::Polyline(_)
    ) {
        Ok(())
    } else {
        Err("图形化曲线工具当前支持直线、圆/圆弧和多段线；椭圆/样条暂不支持".into())
    }
}
/// Exact parameter search for the supported line/arc pieces, including polyline seams.
pub fn parameter(g: &Geometry, point: Point3) -> Result<f64, String> {
    supported(g)?;
    match g {
        Geometry::Line(_) => curves::nearest_parameter(g, point),
        Geometry::Polyline(p) => {
            let mut best = (f64::INFINITY, 0.);
            if p.segments.is_empty() {
                return Err("多段线为空".into());
            }
            for (i, s) in p.segments.iter().enumerate() {
                let part = segment_geometry(s, p.plane)?;
                let t = parameter(&part, point)?;
                let d = curves::distance(curves::evaluate(&part, t)?, point);
                if d < best.0 {
                    best = (d, (i as f64 + t) / p.segments.len() as f64);
                }
            }
            Ok(best.1)
        }
        _ => {
            let (center, plane, start, sweep) = match g {
                Geometry::Arc(a) => (a.center, a.plane, a.start_angle, a.sweep_angle),
                Geometry::Circle(c) => (c.center, c.plane, 0., TAU),
                _ => unreachable!(),
            };
            if sweep.abs() < 1e-10 {
                return Err("圆弧转角为零".into());
            }
            let d = delta(center, point);
            let angle = d.dot(plane.y_axis()).atan2(d.dot(plane.x_axis));
            let t = ((angle - start) * sweep.signum()).rem_euclid(TAU) / sweep.abs();
            if t <= 1. {
                Ok(t)
            } else {
                Ok(
                    if curves::distance(curves::evaluate(g, 0.)?, point)
                        < curves::distance(curves::evaluate(g, 1.)?, point)
                    {
                        0.
                    } else {
                        1.
                    },
                )
            }
        }
    }
}
fn closed(g: &Geometry) -> bool {
    matches!(g, Geometry::Circle(_)) || matches!(g,Geometry::Polyline(p) if p.closed)
}
fn closed_slice(g: &Geometry, from: f64, to: f64) -> Result<Geometry, String> {
    if let Geometry::Circle(c) = g {
        return Ok(Geometry::Arc(Arc {
            center: c.center,
            radius: c.radius,
            plane: c.plane,
            start_angle: TAU * from,
            sweep_angle: TAU * (to - from),
        }));
    }
    if to <= 1. {
        return curves::slice(g, from, to);
    }
    let Geometry::Polyline(p) = g else {
        return Err("不支持该闭合曲线".into());
    };
    let mut segments = Vec::new();
    for (a, b) in [(from, 1.), (0., to - 1.)] {
        if b - a > 1e-10 {
            let Geometry::Polyline(part) = curves::slice(g, a, b)? else {
                unreachable!()
            };
            segments.extend(part.segments);
        }
    }
    Ok(Geometry::Polyline(Polyline {
        segments,
        closed: false,
        plane: p.plane,
    }))
}

/// Remove the interval containing the picked point, bounded by real intersections.
pub fn trim(g: &Geometry, boundaries: &[Geometry], pick: Point3) -> Result<Vec<Geometry>, String> {
    supported(g)?;
    let mut cuts = Vec::new();
    for boundary in boundaries {
        supported(boundary)?;
        for p in curves::intersections(g, boundary, true)? {
            cuts.push(parameter(g, p)?);
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    if cuts.is_empty() {
        return Err("对象与所选边界没有有效交点".into());
    }
    let t = parameter(g, pick)?;
    if cuts.iter().any(|c| {
        let d = (c - t).abs();
        d < 1e-8 || closed(g) && (1. - d).abs() < 1e-8
    }) {
        return Err("请点击要修剪的一侧，避开边界交点".into());
    }
    if closed(g) {
        if cuts.last().is_some_and(|c| *c > 1. - 1e-9) && cuts.first().is_some_and(|c| *c < 1e-9) {
            cuts.pop();
        }
        if cuts.len() < 2 {
            return Err("闭合轮廓修剪至少需要两个不同交点".into());
        }
        let lo = cuts
            .iter()
            .copied()
            .filter(|c| *c < t)
            .last()
            .unwrap_or(cuts[cuts.len() - 1] - 1.);
        let hi = cuts
            .iter()
            .copied()
            .find(|c| *c > t)
            .unwrap_or(cuts[0] + 1.);
        let from = hi.rem_euclid(1.);
        let length = 1. - (hi - lo);
        return Ok(vec![closed_slice(g, from, from + length)?]);
    }
    let lo = cuts.iter().copied().filter(|c| *c < t).last().unwrap_or(0.);
    let hi = cuts.iter().copied().find(|c| *c > t).unwrap_or(1.);
    if lo < 1e-9 && hi > 1. - 1e-9 && cuts.len() < 2 {
        return Err("没有可保留的有效区间，请重新选择边界".into());
    }
    let mut output = Vec::new();
    if lo > 1e-9 {
        output.push(curves::slice(g, 0., lo)?);
    }
    if hi < 1. - 1e-9 {
        output.push(curves::slice(g, hi, 1.)?);
    }
    Ok(output)
}

/// Extend only the picked terminal piece to the nearest bounded boundary intersection.
pub fn extend(
    g: &Geometry,
    boundaries: &[Geometry],
    pick: Point3,
    mut options: CurveOptions,
) -> Result<Geometry, String> {
    supported(g)?;
    if closed(g) {
        return Err("闭合轮廓不能延伸".into());
    }
    let start = curves::distance(curves::evaluate(g, 0.)?, pick)
        < curves::distance(curves::evaluate(g, 1.)?, pick);
    let piece = if let Geometry::Polyline(p) = g {
        let s = if start {
            p.segments.first()
        } else {
            p.segments.last()
        }
        .ok_or("多段线为空")?;
        segment_geometry(s, p.plane)?
    } else {
        g.clone()
    };
    let mut length = f64::INFINITY;
    for boundary in boundaries {
        supported(boundary)?;
        for point in curves::intersections(&piece, boundary, false)? {
            let t = parameter(boundary, point)?;
            if curves::distance(curves::evaluate(boundary, t)?, point) > EPS {
                continue;
            }
            let distance = match &piece {
                Geometry::Line(l) => {
                    let d = delta(l.start, l.end);
                    let len = d.length();
                    if len <= EPS {
                        return Err("零长线不能延伸".into());
                    }
                    let t = delta(l.start, point).dot(d) / len.powi(2);
                    if start {
                        -t * len
                    } else {
                        (t - 1.) * len
                    }
                }
                Geometry::Arc(a) => {
                    let d = delta(a.center, point);
                    let angle = d.dot(a.plane.y_axis()).atan2(d.dot(a.plane.x_axis));
                    let extra = if start {
                        (a.start_angle - angle) * a.sweep_angle.signum()
                    } else {
                        (angle - a.start_angle - a.sweep_angle) * a.sweep_angle.signum()
                    }
                    .rem_euclid(TAU);
                    if a.sweep_angle.abs() + extra >= TAU - 1e-8 {
                        continue;
                    }
                    extra * a.radius
                }
                _ => return Err("延伸支持直线、圆弧和开放多段线".into()),
            };
            if distance > EPS {
                length = length.min(distance);
            }
        }
    }
    if !length.is_finite() {
        return Err("选定端点的延伸方向没有有效边界交点".into());
    }
    options.at_start = start;
    options.length = length;
    curve_edit::apply(CurveTool::Extend, &[g.clone()], options)?
        .into_iter()
        .next()
        .ok_or("延伸没有结果".into())
}

pub fn offset(g: &Geometry, pick: Point3, mut options: CurveOptions) -> Result<Geometry, String> {
    supported(g)?;
    let side = match g {
        Geometry::Circle(c) => {
            let d = delta(c.center, pick);
            d.dot(c.plane.x_axis).hypot(d.dot(c.plane.y_axis())) - c.radius
        }
        Geometry::Arc(a) => {
            let d = delta(a.center, pick);
            d.dot(a.plane.x_axis).hypot(d.dot(a.plane.y_axis())) - a.radius
        }
        _ => {
            let t = parameter(g, pick)?;
            let (piece, t, plane) = if let Geometry::Polyline(p) = g {
                let at = t * p.segments.len() as f64;
                let i = (at.floor() as usize).min(p.segments.len() - 1);
                (
                    segment_geometry(&p.segments[i], p.plane)?,
                    at - i as f64,
                    p.plane.ok_or("空间多段线不能平面偏移")?,
                )
            } else {
                (g.clone(), t, options.plane)
            };
            let nearest = curves::evaluate(&piece, t)?;
            let tangent = match &piece {
                Geometry::Line(l) => delta(l.start, l.end),
                Geometry::Arc(a) => {
                    a.plane.normal.cross(delta(a.center, nearest)) * a.sweep_angle.signum()
                }
                _ => unreachable!(),
            };
            delta(nearest, pick).dot(plane.normal.cross(tangent).normalize())
        }
    };
    if side.abs() <= EPS {
        return Err("请在轮廓一侧选择偏移方向".into());
    }
    if !options.length.is_finite() || options.length <= EPS {
        return Err("偏移距离须大于几何容差且有限".into());
    }
    options.length *= side.signum();
    curve_edit::apply(CurveTool::Offset, &[g.clone()], options)?
        .into_iter()
        .next()
        .ok_or("偏移没有结果".into())
}

/// Picked sides disambiguate a corner even when the source lines are not connected.
pub fn corner(
    a: &Geometry,
    b: &Geometry,
    pa: Point3,
    pb: Point3,
    options: CurveOptions,
    fillet: bool,
) -> Result<[Geometry; 3], String> {
    let (Geometry::Line(la), Geometry::Line(lb)) = (a, b) else {
        return Err("点选圆角/倒角当前支持两条共面非平行直线".into());
    };
    let at = curves::intersections(a, b, false)?
        .into_iter()
        .next()
        .ok_or("两条直线没有唯一共面交点")?;
    let endpoint = |line: &Line, pick: Point3| -> Result<Point3, String> {
        let direction = delta(line.start, line.end);
        let side = delta(at, pick).dot(direction);
        if side.abs() <= EPS * direction.length() {
            return Err("请在交点之外点击要保留的线段一侧".into());
        }
        [line.start, line.end]
            .into_iter()
            .filter(|p| delta(at, *p).dot(direction) * side > 0.)
            .max_by(|x, y| curves::distance(at, *x).total_cmp(&curves::distance(at, *y)))
            .ok_or("选定侧没有可保留的线段".into())
    };
    let input = [
        Geometry::Line(Line {
            start: at,
            end: endpoint(la, pa)?,
        }),
        Geometry::Line(Line {
            start: at,
            end: endpoint(lb, pb)?,
        }),
    ];
    let mut output = curve_edit::apply(
        if fillet {
            CurveTool::Fillet
        } else {
            CurveTool::Chamfer
        },
        &input,
        options,
    )?;
    let Geometry::Polyline(p) = output.remove(0) else {
        return Err("圆角/倒角结果无效".into());
    };
    let mut parts: Vec<_> = p
        .segments
        .iter()
        .map(|s| segment_geometry(s, p.plane))
        .collect::<Result<_, _>>()?;
    for (i, original) in [(0, la), (2, lb)] {
        if let Geometry::Line(line) = &mut parts[i] {
            if delta(line.start, line.end).dot(delta(original.start, original.end)) < 0. {
                std::mem::swap(&mut line.start, &mut line.end);
            }
        }
    }
    Ok([parts.remove(0), parts.remove(0), parts.remove(0)])
}
