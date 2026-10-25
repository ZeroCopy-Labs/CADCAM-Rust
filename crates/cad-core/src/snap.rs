//! Geometric snap points, evaluated in f64 rather than from display polylines.
use crate::model::{Geometry, Plane, Point3, PolylineSegment, Spline, Vector3};
use std::collections::BTreeSet;
use std::f64::consts::{FRAC_PI_2, TAU};

const EPS: f64 = 1e-6;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SnapKind {
    Endpoint,
    Midpoint,
    Center,
    Quadrant,
    Intersection,
    Perpendicular,
    Tangent,
    Nearest,
}
impl SnapKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Endpoint => "端点",
            Self::Midpoint => "中点",
            Self::Center => "圆心/中心",
            Self::Quadrant => "象限点",
            Self::Intersection => "真实交点",
            Self::Perpendicular => "垂足",
            Self::Tangent => "切点",
            Self::Nearest => "最近点",
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub struct SnapPoint {
    pub point: Point3,
    pub kind: SnapKind,
}

#[derive(Clone, Copy)]
enum Curve<'a> {
    Line(Point3, Point3),
    Circle {
        center: Point3,
        radius: f64,
        plane: Plane,
        start: f64,
        sweep: f64,
    },
    Ellipse {
        center: Point3,
        u: Vector3,
        v: Vector3,
        start: f64,
        sweep: f64,
    },
    Spline(&'a Spline),
}
fn curves(geometry: &Geometry) -> Vec<Curve<'_>> {
    match geometry {
        Geometry::Line(l) => vec![Curve::Line(l.start, l.end)],
        Geometry::Circle(c) => vec![Curve::Circle {
            center: c.center,
            radius: c.radius,
            plane: c.plane,
            start: 0.0,
            sweep: TAU,
        }],
        Geometry::Arc(a) => vec![Curve::Circle {
            center: a.center,
            radius: a.radius,
            plane: a.plane,
            start: a.start_angle,
            sweep: a.sweep_angle,
        }],
        Geometry::Ellipse(e) => vec![Curve::Ellipse {
            center: e.center,
            u: e.major_axis,
            v: e.minor_axis,
            start: e.start_parameter,
            sweep: e.sweep_parameter,
        }],
        Geometry::Spline(s) => vec![Curve::Spline(s)],
        Geometry::Polyline(p) => p
            .segments
            .iter()
            .filter_map(|segment| match segment {
                PolylineSegment::Line { start, end } => Some(Curve::Line(*start, *end)),
                PolylineSegment::Arc {
                    center,
                    radius,
                    start_angle,
                    sweep_angle,
                    ..
                } => p.plane.map(|plane| Curve::Circle {
                    center: *center,
                    radius: *radius,
                    plane,
                    start: *start_angle,
                    sweep: *sweep_angle,
                }),
            })
            .collect(),
    }
}
pub fn vector(a: Point3, b: Point3) -> Vector3 {
    Vector3::new(b.x - a.x, b.y - a.y, b.z - a.z)
}
fn add(p: Point3, v: Vector3) -> Point3 {
    Point3::new(p.x + v.x, p.y + v.y, p.z + v.z)
}
fn distance(a: Point3, b: Point3) -> f64 {
    vector(a, b).length()
}
pub fn ray_distance(p: Point3, origin: Point3, direction: Vector3) -> f64 {
    let d = vector(origin, p);
    (d - direction * d.dot(direction)).length()
}
fn parameter(angle: f64, start: f64, sweep: f64) -> Option<f64> {
    if sweep.abs() < 1e-12 {
        return None;
    }
    let delta = if sweep > 0.0 {
        (angle - start).rem_euclid(TAU)
    } else {
        (start - angle).rem_euclid(TAU)
    };
    if sweep.abs() >= TAU - 1e-10 || delta <= sweep.abs() + 1e-10 {
        Some((delta / sweep.abs()).clamp(0.0, 1.0))
    } else {
        None
    }
}
pub(crate) fn spline_point(s: &Spline, t: f64) -> Option<Point3> {
    let n = s.control_points.len();
    let p = s.degree as usize;
    if n <= p
        || s.knots.len() != n + p + 1
        || s.knots.iter().any(|v| !v.is_finite())
        || s.knots.windows(2).any(|w| w[1] < w[0])
        || (!s.weights.is_empty() && s.weights.len() != n)
    {
        return None;
    }
    let lo = s.knots[p];
    let hi = s.knots[n];
    if hi <= lo {
        return None;
    }
    let u = lo + (hi - lo) * t.clamp(0.0, 1.0);
    let span = if u >= hi {
        n - 1
    } else {
        (p..n).find(|&i| s.knots[i] <= u && u < s.knots[i + 1])?
    };
    let mut d: Vec<[f64; 4]> = (span - p..=span)
        .map(|i| {
            let w = s.weights.get(i).copied().unwrap_or(1.0);
            let q = s.control_points[i];
            [q.x * w, q.y * w, q.z * w, w]
        })
        .collect();
    for r in 1..=p {
        for j in (r..=p).rev() {
            let i = span - p + j;
            let denominator = s.knots[i + p - r + 1] - s.knots[i];
            let a = if denominator.abs() > 1e-15 {
                (u - s.knots[i]) / denominator
            } else {
                0.0
            };
            for k in 0..4 {
                d[j][k] = (1.0 - a) * d[j - 1][k] + a * d[j][k];
            }
        }
    }
    let q = d[p];
    if !q.iter().all(|v| v.is_finite()) || q[3].abs() < 1e-15 {
        return None;
    }
    Some(Point3::new(q[0] / q[3], q[1] / q[3], q[2] / q[3]))
}
impl Curve<'_> {
    fn point(self, t: f64) -> Option<Point3> {
        Some(match self {
            Self::Line(a, b) => add(a, vector(a, b) * t),
            Self::Circle {
                center,
                radius,
                plane,
                start,
                sweep,
            } => {
                let a = start + sweep * t;
                add(
                    center,
                    plane.x_axis * (radius * a.cos()) + plane.y_axis() * (radius * a.sin()),
                )
            }
            Self::Ellipse {
                center,
                u,
                v,
                start,
                sweep,
            } => {
                let a = start + sweep * t;
                add(center, u * a.cos() + v * a.sin())
            }
            Self::Spline(s) => return spline_point(s, t),
        })
    }
    fn closed(self) -> bool {
        match self {
            Self::Circle { sweep, .. } | Self::Ellipse { sweep, .. } => sweep.abs() >= TAU - 1e-10,
            Self::Spline(s) => s.closed || s.periodic,
            _ => false,
        }
    }
    fn nearest(self, origin: Point3, direction: Vector3) -> Option<Point3> {
        if let Self::Line(a, b) = self {
            let d = vector(a, b);
            let q = vector(origin, a);
            let projected = d - direction * d.dot(direction);
            let q = q - direction * q.dot(direction);
            let l = projected.dot(projected);
            let t = if l > 1e-24 {
                (-q.dot(projected) / l).clamp(0.0, 1.0)
            } else {
                0.0
            };
            return self.point(t);
        }
        let score = |t: f64| {
            self.point(t)
                .map(|p| ray_distance(p, origin, direction))
                .unwrap_or(f64::INFINITY)
        };
        let best = (0_usize..=64)
            .min_by(|a, b| score(*a as f64 / 64.0).total_cmp(&score(*b as f64 / 64.0)))?;
        let mut left = (best.saturating_sub(1) as f64) / 64.0;
        let mut right = ((best + 1).min(64) as f64) / 64.0;
        for _ in 0..40 {
            let a = left + (right - left) / 3.0;
            let b = right - (right - left) / 3.0;
            if score(a) < score(b) {
                right = b;
            } else {
                left = a;
            }
        }
        let t = [0.0, 1.0, (left + right) * 0.5]
            .into_iter()
            .min_by(|a, b| score(*a).total_cmp(&score(*b)))?;
        self.point(t)
    }
}

pub fn snap_points(
    geometry: &Geometry,
    origin: Point3,
    direction: Vector3,
    reference: Option<Point3>,
    modes: &BTreeSet<SnapKind>,
) -> Vec<SnapPoint> {
    let mut points = Vec::new();
    let mut push = |point: Option<Point3>, kind: SnapKind| {
        if modes.contains(&kind) {
            if let Some(point) = point.filter(|p| [p.x, p.y, p.z].iter().all(|v| v.is_finite())) {
                if !points
                    .iter()
                    .any(|p: &SnapPoint| p.kind == kind && distance(p.point, point) < EPS)
                {
                    points.push(SnapPoint { point, kind });
                }
            }
        }
    };
    for curve in curves(geometry) {
        if !curve.closed() {
            push(curve.point(0.0), SnapKind::Endpoint);
            push(curve.point(1.0), SnapKind::Endpoint);
        }
        if !curve.closed() && !matches!(curve, Curve::Spline(_)) {
            push(curve.point(0.5), SnapKind::Midpoint);
        }
        match curve {
            Curve::Circle {
                center,
                radius,
                plane,
                start,
                sweep,
            } => {
                push(Some(center), SnapKind::Center);
                for i in 0..4 {
                    if let Some(t) = parameter(i as f64 * FRAC_PI_2, start, sweep) {
                        push(curve.point(t), SnapKind::Quadrant);
                    }
                }
                if let Some(reference) = reference {
                    let q = vector(center, reference);
                    let x = q.dot(plane.x_axis);
                    let y = q.dot(plane.y_axis());
                    let r = x.hypot(y);
                    let a = y.atan2(x);
                    if r > EPS {
                        for angle in [a, a + std::f64::consts::PI] {
                            if let Some(t) = parameter(angle, start, sweep) {
                                push(curve.point(t), SnapKind::Perpendicular);
                            }
                        }
                        if q.dot(plane.normal).abs() <= EPS && r > radius + EPS {
                            let delta = (radius / r).acos();
                            for angle in [a - delta, a + delta] {
                                if let Some(t) = parameter(angle, start, sweep) {
                                    push(curve.point(t), SnapKind::Tangent);
                                }
                            }
                        }
                    }
                }
            }
            Curve::Ellipse {
                center,
                start,
                sweep,
                ..
            } => {
                push(Some(center), SnapKind::Center);
                for i in 0..4 {
                    if let Some(t) = parameter(i as f64 * FRAC_PI_2, start, sweep) {
                        push(curve.point(t), SnapKind::Quadrant);
                    }
                }
            }
            Curve::Line(a, b) => {
                if let Some(reference) = reference {
                    let d = vector(a, b);
                    let length = d.dot(d);
                    if length > 1e-24 {
                        let t = vector(a, reference).dot(d) / length;
                        if (0.0..=1.0).contains(&t) {
                            push(curve.point(t), SnapKind::Perpendicular);
                        }
                    }
                }
            }
            Curve::Spline(_) => {}
        }
        if modes.contains(&SnapKind::Nearest) {
            push(curve.nearest(origin, direction), SnapKind::Nearest);
        }
    }
    points
}

/// Geometry-based shortlist; display tessellation is not used to decide curve proximity.
pub fn near_ray(geometry: &Geometry, origin: Point3, direction: Vector3, radius: f64) -> bool {
    curves(geometry).into_iter().any(|curve| {
        let bound = match curve {
            Curve::Circle { center, radius, .. } => Some((center, radius)),
            Curve::Ellipse { center, u, v, .. } => Some((center, u.length() + v.length())),
            _ => None,
        };
        if bound
            .is_some_and(|(center, r)| ray_distance(center, origin, direction) > r + radius + EPS)
        {
            return false;
        }
        curve
            .nearest(origin, direction)
            .is_some_and(|p| ray_distance(p, origin, direction) <= radius + EPS)
    })
}

fn line_line(a: Point3, b: Point3, c: Point3, d: Point3) -> Vec<Point3> {
    let u = vector(a, b);
    let v = vector(c, d);
    let w = vector(c, a);
    let aa = u.dot(u);
    let bb = u.dot(v);
    let cc = v.dot(v);
    let dd = u.dot(w);
    let ee = v.dot(w);
    let den = aa * cc - bb * bb;
    if den.abs() <= 1e-14 * aa * cc {
        return Vec::new();
    }
    let t = (bb * ee - cc * dd) / den;
    let s = (aa * ee - bb * dd) / den;
    if !(0.0..=1.0).contains(&t) || !(0.0..=1.0).contains(&s) {
        return Vec::new();
    }
    let p = add(a, u * t);
    let q = add(c, v * s);
    if distance(p, q) <= EPS {
        vec![add(p, vector(p, q) * 0.5)]
    } else {
        Vec::new()
    }
}
fn line_circle(a: Point3, b: Point3, circle: Curve<'_>) -> Vec<Point3> {
    let Curve::Circle {
        center,
        radius,
        plane,
        start,
        sweep,
    } = circle
    else {
        return Vec::new();
    };
    let d = vector(a, b);
    let q = vector(center, a);
    let den = d.dot(plane.normal);
    let mut parameters = Vec::new();
    if den.abs() > 1e-12 * d.length() {
        let t = -q.dot(plane.normal) / den;
        if (vector(center, add(a, d * t)).length() - radius).abs() <= EPS {
            parameters.push(t);
        }
    } else if q.dot(plane.normal).abs() <= EPS {
        let aa = d.dot(d);
        let bb = 2.0 * q.dot(d);
        let cc = q.dot(q) - radius * radius;
        let discriminant = bb * bb - 4.0 * aa * cc;
        if aa > 1e-24 && discriminant >= 0.0 {
            let root = discriminant.sqrt();
            parameters.extend([(-bb - root) / (2.0 * aa), (-bb + root) / (2.0 * aa)]);
        }
    }
    parameters
        .into_iter()
        .filter(|t| (0.0..=1.0).contains(t))
        .filter_map(|t| {
            let p = add(a, d * t);
            let q = vector(center, p);
            parameter(
                q.dot(plane.y_axis()).atan2(q.dot(plane.x_axis)),
                start,
                sweep,
            )
            .map(|_| p)
        })
        .collect()
}
fn circle_circle(a: Curve<'_>, b: Curve<'_>) -> Vec<Point3> {
    let (
        Curve::Circle {
            center: c1,
            radius: r1,
            plane: p1,
            start: s1,
            sweep: w1,
        },
        Curve::Circle {
            center: c2,
            radius: r2,
            plane: p2,
            start: s2,
            sweep: w2,
        },
    ) = (a, b)
    else {
        return Vec::new();
    };
    let delta = vector(c1, c2);
    let d = delta.length();
    if p1.normal.cross(p2.normal).length() > 1e-8
        || delta.dot(p1.normal).abs() > EPS
        || d <= EPS
        || d > r1 + r2 + EPS
        || d < (r1 - r2).abs() - EPS
    {
        return Vec::new();
    }
    let x = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    let h2 = r1 * r1 - x * x;
    if h2 < -EPS * EPS {
        return Vec::new();
    }
    let u = delta * (1.0 / d);
    let v = p1.normal.cross(u);
    let base = add(c1, u * x);
    let h = h2.max(0.0).sqrt();
    [add(base, v * h), add(base, v * (-h))]
        .into_iter()
        .filter(|p| {
            let q1 = vector(c1, *p);
            let q2 = vector(c2, *p);
            parameter(q1.dot(p1.y_axis()).atan2(q1.dot(p1.x_axis)), s1, w1).is_some()
                && parameter(q2.dot(p2.y_axis()).atan2(q2.dot(p2.x_axis)), s2, w2).is_some()
        })
        .collect()
}

/// True spatial intersections of nearby line/circle/arc primitives. Skew projections are excluded.
pub fn intersections(
    a: &Geometry,
    b: &Geometry,
    origin: Point3,
    direction: Vector3,
    radius: f64,
) -> Vec<Point3> {
    let nearby = |curve: &Curve<'_>| {
        curve
            .nearest(origin, direction)
            .is_some_and(|p| ray_distance(p, origin, direction) <= radius + EPS)
    };
    let aa: Vec<_> = curves(a).into_iter().filter(nearby).collect();
    let bb: Vec<_> = curves(b).into_iter().filter(nearby).collect();
    let mut result = Vec::new();
    for a in aa {
        for b in &bb {
            let points = match (a, *b) {
                (Curve::Line(p, q), Curve::Line(r, s)) => line_line(p, q, r, s),
                (Curve::Line(p, q), circle @ Curve::Circle { .. })
                | (circle @ Curve::Circle { .. }, Curve::Line(p, q)) => line_circle(p, q, circle),
                (Curve::Circle { .. }, Curve::Circle { .. }) => circle_circle(a, *b),
                _ => Vec::new(),
            };
            for p in points {
                if !result.iter().any(|q| distance(*q, p) < EPS) {
                    result.push(p);
                }
            }
        }
    }
    result
}
