//! Bounded, read-only diagnostics with explicit single-issue repair proposals.
use crate::{
    curves::{self, delta, DISTANCE_TOLERANCE as EPS},
    model::{DrawingDocument, EntityId, Geometry, Point3, Vector3},
    properties::{self, GripKind},
    CurveEdit,
};
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;
const PAIRS: usize = 200_000;
const ISSUES: usize = 512;
const SEGMENTS: usize = 20_000;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Duplicate,
    ZeroLength,
    Gap,
    OpenContour,
    Intersection,
    InvalidGeometry,
}
impl Kind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Duplicate => "几何重复",
            Self::ZeroLength => "零长段",
            Self::Gap => "端点间隙",
            Self::OpenContour => "开放/分支轮廓",
            Self::Intersection => "内部交叉",
            Self::InvalidGeometry => "几何/采样无效",
        }
    }
}
#[derive(Clone, Debug)]
pub struct Issue {
    pub kind: Kind,
    pub entities: Vec<EntityId>,
    pub point: Point3,
    pub detail: String,
    pub gap: Option<(Endpoint, Endpoint)>,
}
#[derive(Clone, Debug)]
pub struct Endpoint {
    pub id: EntityId,
    pub point: Point3,
    pub grip: Option<GripKind>,
}
#[derive(Clone, Debug)]
pub struct Contour {
    pub entities: Vec<EntityId>,
    pub closed: bool,
    pub length: f64,
    pub approximate: bool,
    pub direction: Option<(bool, Vector3)>,
    pub point: Point3,
}
#[derive(Clone, Debug, Default)]
pub struct Report {
    pub issues: Vec<Issue>,
    pub contours: Vec<Contour>,
    pub truncated: bool,
    pub omitted: usize,
    pub pairs: usize,
}
impl Report {
    fn push(&mut self, issue: Issue) {
        if self.issues.len() < ISSUES {
            self.issues.push(issue);
        } else {
            self.omitted += 1;
            self.truncated = true;
        }
    }
}
fn closed(g: &Geometry) -> bool {
    match g {
        Geometry::Circle(_) => true,
        Geometry::Polyline(p) => p.closed,
        Geometry::Ellipse(e) => e.sweep_parameter.abs() >= TAU - 1e-8,
        Geometry::Spline(s) => s.closed || s.periodic,
        _ => false,
    }
}
fn near(a: Point3, b: Point3) -> bool {
    curves::distance(a, b) <= EPS
}
fn same_geometry(a: &Geometry, b: &Geometry) -> bool {
    match (a, b) {
        (Geometry::Line(a), Geometry::Line(b)) => {
            (near(a.start, b.start) && near(a.end, b.end))
                || (near(a.start, b.end) && near(a.end, b.start))
        }
        (Geometry::Circle(a), Geometry::Circle(b)) => {
            near(a.center, b.center)
                && (a.radius - b.radius).abs() <= EPS
                && a.plane.normal.cross(b.plane.normal).length() <= 1e-8
        }
        (Geometry::Arc(a), Geometry::Arc(b)) => {
            near(a.center, b.center)
                && (a.radius - b.radius).abs() <= EPS
                && (a.sweep_angle.abs() - b.sweep_angle.abs()).abs() <= 1e-8
                && [false, true].into_iter().any(|rev| {
                    [0.0, 0.5, 1.0].into_iter().all(|t| {
                        curves::evaluate(&Geometry::Arc(a.clone()), t)
                            .ok()
                            .zip(
                                curves::evaluate(
                                    &Geometry::Arc(b.clone()),
                                    if rev { 1.0 - t } else { t },
                                )
                                .ok(),
                            )
                            .is_some_and(|(p, q)| near(p, q))
                    })
                })
        }
        (Geometry::Ellipse(a), Geometry::Ellipse(b)) => {
            if !near(a.center, b.center)
                || (a.major_axis.length() - b.major_axis.length()).abs() > EPS
                || (a.minor_axis.length() - b.minor_axis.length()).abs() > EPS
                || (a.sweep_parameter - b.sweep_parameter).abs() > 1e-8
            {
                return false;
            }
            [false, true].into_iter().any(|rev| {
                [0.0, 0.25, 0.5, 0.75, 1.0].into_iter().all(|t| {
                    curves::evaluate(&Geometry::Ellipse(a.clone()), t)
                        .ok()
                        .zip(
                            curves::evaluate(
                                &Geometry::Ellipse(b.clone()),
                                if rev { 1.0 - t } else { t },
                            )
                            .ok(),
                        )
                        .is_some_and(|(p, q)| near(p, q))
                })
            })
        }
        (Geometry::Spline(a), Geometry::Spline(b)) => {
            let match_data = |a: &crate::model::Spline, b: &crate::model::Spline| {
                a.degree == b.degree
                    && a.closed == b.closed
                    && a.periodic == b.periodic
                    && a.fit_points.len() == b.fit_points.len()
                    && a.fit_points
                        .iter()
                        .zip(&b.fit_points)
                        .all(|(p, q)| near(*p, *q))
                    && a.knots == b.knots
                    && a.weights == b.weights
                    && a.control_points.len() == b.control_points.len()
                    && a.control_points
                        .iter()
                        .zip(&b.control_points)
                        .all(|(p, q)| near(*p, *q))
            };
            match_data(a, b)
                || curves::reverse(&Geometry::Spline(b.clone()))
                    .ok()
                    .is_some_and(|g| matches!(g,Geometry::Spline(ref b) if match_data(a,b)))
        }
        (Geometry::Polyline(a), Geometry::Polyline(b)) => {
            if a.closed != b.closed
                || a.segments.len() != b.segments.len()
                || a.segments.len() > 1024
            {
                return false;
            }
            let compare = |b: &crate::model::Polyline| {
                let n = a.segments.len();
                let shifts = if a.closed { n } else { 1 };
                (0..shifts).any(|shift| {
                    a.segments.iter().enumerate().all(|(i, s)| {
                        curves::segment_geometry(s, a.plane)
                            .ok()
                            .zip(
                                curves::segment_geometry(&b.segments[(i + shift) % n], b.plane)
                                    .ok(),
                            )
                            .is_some_and(|(x, y)| same_geometry(&x, &y))
                    })
                })
            };
            compare(b)
                || curves::reverse(&Geometry::Polyline(b.clone()))
                    .ok()
                    .is_some_and(|g| matches!(g,Geometry::Polyline(ref b) if compare(b)))
        }
        _ => false,
    }
}
pub fn length(g: &Geometry) -> Result<(f64, bool), String> {
    Ok(match g {
        Geometry::Line(l) => (curves::distance(l.start, l.end), false),
        Geometry::Circle(c) => (TAU * c.radius, false),
        Geometry::Arc(a) => (a.radius * a.sweep_angle.abs(), false),
        Geometry::Polyline(p) => {
            let mut sum = 0.0;
            for s in &p.segments {
                sum += length(&curves::segment_geometry(s, p.plane)?)?.0;
            }
            (sum, false)
        }
        _ => {
            let points = curves::sample(
                g,
                curves::Sampling {
                    chord: 0.05,
                    angle: 5f64.to_radians(),
                },
            )?;
            (
                points
                    .windows(2)
                    .map(|p| curves::distance(p[0], p[1]))
                    .sum(),
                true,
            )
        }
    })
}
fn location(g: &Geometry) -> Point3 {
    curves::evaluate(g, 0.0).unwrap_or_else(|_| match g {
        Geometry::Spline(s) => s
            .control_points
            .first()
            .or(s.fit_points.first())
            .copied()
            .unwrap_or(Point3::new(0.0, 0.0, 0.0)),
        _ => Point3::new(0.0, 0.0, 0.0),
    })
}
fn root(parent: &mut [usize], i: usize) -> usize {
    let mut r = i;
    while parent[r] != r {
        r = parent[r];
    }
    let mut node = i;
    while parent[node] != node {
        let next = parent[node];
        parent[node] = r;
        node = next;
    }
    r
}
fn issue(kind: Kind, ids: Vec<EntityId>, point: Point3, detail: String) -> Issue {
    Issue {
        kind,
        entities: ids,
        point,
        detail,
        gap: None,
    }
}
struct Segment {
    id: EntityId,
    index: usize,
    count: usize,
    closed: bool,
    g: Geometry,
    approx: bool,
    min: [f64; 3],
    max: [f64; 3],
}
fn bounds(g: &Geometry) -> ([f64; 3], [f64; 3]) {
    match g {
        Geometry::Line(l) => {
            let a = [l.start.x, l.start.y, l.start.z];
            let b = [l.end.x, l.end.y, l.end.z];
            (
                std::array::from_fn(|i| a[i].min(b[i]) - EPS),
                std::array::from_fn(|i| a[i].max(b[i]) + EPS),
            )
        }
        Geometry::Arc(a) => {
            let c = [a.center.x, a.center.y, a.center.z];
            let x = [a.plane.x_axis.x, a.plane.x_axis.y, a.plane.x_axis.z];
            let v = a.plane.y_axis();
            let y = [v.x, v.y, v.z];
            let r: [f64; 3] = std::array::from_fn(|i| a.radius * x[i].hypot(y[i]) + EPS);
            (
                std::array::from_fn(|i| c[i] - r[i]),
                std::array::from_fn(|i| c[i] + r[i]),
            )
        }
        Geometry::Circle(c) => bounds(&Geometry::Arc(crate::model::Arc {
            center: c.center,
            radius: c.radius,
            plane: c.plane,
            start_angle: 0.0,
            sweep_angle: TAU,
        })),
        _ => unreachable!("diagnostic primitives are lines or circular curves"),
    }
}
pub fn analyze(doc: &DrawingDocument, gap_limit: f64) -> Report {
    let mut report = Report::default();
    let gap_limit = if gap_limit.is_finite() {
        gap_limit.clamp(EPS, 1.0)
    } else {
        0.01
    };
    let mut entities: Vec<_> = doc
        .entities
        .iter()
        .filter(|e| !matches!(e.geometry, Geometry::Text(_)))
        .take(50_000)
        .collect();
    report.truncated = entities.len()
        != doc
            .entities
            .iter()
            .filter(|e| !matches!(e.geometry, Geometry::Text(_)))
            .count();
    let mut endpoints = Vec::new();
    let mut lengths = Vec::new();
    let mut segments = Vec::new();
    for (owner, e) in entities.iter().enumerate() {
        if segments.len() >= SEGMENTS {
            report.truncated = true;
            break;
        }

        let g = &e.geometry;
        let point = location(g);
        match length(g) {
            Ok(value) => lengths.push(value),
            Err(error) => {
                lengths.push((0.0, true));
                report.push(issue(Kind::InvalidGeometry, vec![e.id], point, error));
            }
        }
        if !closed(g) {
            for t in [0.0, 1.0] {
                if let Ok(point) = curves::evaluate(g, t) {
                    let grip = properties::grips(g)
                        .into_iter()
                        .find(|h| {
                            matches!(
                                h.kind,
                                GripKind::Start
                                    | GripKind::End
                                    | GripKind::Vertex(_)
                                    | GripKind::Control(_)
                            ) && near(h.point, point)
                        })
                        .map(|h| h.kind);
                    endpoints.push((
                        owner,
                        Endpoint {
                            id: e.id,
                            point,
                            grip,
                        },
                    ));
                }
            }
        }
        let pieces: Vec<_> = match g {
            Geometry::Text(_) => Vec::new(),
            Geometry::Line(_) | Geometry::Arc(_) | Geometry::Circle(_) => vec![(g.clone(), false)],
            Geometry::Polyline(p) => p
                .segments
                .iter()
                .filter_map(|s| {
                    curves::segment_geometry(s, p.plane)
                        .ok()
                        .map(|g| (g, false))
                })
                .collect(),
            Geometry::Ellipse(_) | Geometry::Spline(_) => curves::sample(
                g,
                curves::Sampling {
                    chord: 0.05,
                    angle: 0.1,
                },
            )
            .map(|p| {
                p.windows(2)
                    .map(|w| {
                        (
                            Geometry::Line(crate::model::Line {
                                start: w[0],
                                end: w[1],
                            }),
                            true,
                        )
                    })
                    .collect()
            })
            .unwrap_or_default(),
        };
        let count = pieces.len();
        for (index, (g, approx)) in pieces.into_iter().enumerate() {
            if length(&g).is_ok_and(|v| v.0 <= EPS) {
                report.push(issue(
                    Kind::ZeroLength,
                    vec![e.id],
                    location(&g),
                    format!("第 {} 段长度不超过 {} mm", index + 1, EPS),
                ));
            }
            if segments.len() >= SEGMENTS {
                report.truncated = true;
                break;
            }
            let (min, max) = bounds(&g);
            segments.push(Segment {
                id: e.id,
                index,
                count,
                closed: closed(&e.geometry),
                g,
                approx,
                min,
                max,
            });
        }
    }
    entities.truncate(lengths.len());
    let mut checks = 0;
    'duplicates: for i in 0..entities.len() {
        for j in i + 1..entities.len() {
            checks += 1;
            if checks > PAIRS {
                report.truncated = true;
                break 'duplicates;
            }
            let (a, b) = (entities[i], entities[j]);
            if same_geometry(&a.geometry, &b.geometry) {
                report.push(issue(
                    Kind::Duplicate,
                    vec![a.id, b.id],
                    location(&a.geometry),
                    "几何重合；需核对图层、颜色及用途后再决定删除".into(),
                ));
            }
        }
    }
    report.pairs += checks.min(PAIRS);
    let mut parent: Vec<_> = (0..entities.len()).collect();
    let mut degree = vec![0usize; endpoints.len()];
    checks = 0;
    'gaps: for i in 0..endpoints.len() {
        for j in i + 1..endpoints.len() {
            checks += 1;
            if checks > PAIRS {
                report.truncated = true;
                break 'gaps;
            }
            let (a, b) = (&endpoints[i].1, &endpoints[j].1);
            let d = curves::distance(a.point, b.point);
            if d <= EPS {
                degree[i] += 1;
                degree[j] += 1;
                let x = root(&mut parent, endpoints[i].0);
                let y = root(&mut parent, endpoints[j].0);
                parent[x] = y;
            } else if d <= gap_limit {
                let mut item = issue(
                    Kind::Gap,
                    vec![a.id, b.id],
                    a.point,
                    format!("间隙 {d:.6} mm，候选上限 {gap_limit:.6} mm；修复仅针对当前候选"),
                );
                item.gap = Some((a.clone(), b.clone()));
                report.push(item);
            }
        }
    }
    report.pairs += checks.min(PAIRS);
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..entities.len() {
        let r = root(&mut parent, i);
        groups.entry(r).or_default().push(i);
    }
    for members in groups.into_values() {
        let ids: Vec<_> = members.iter().map(|i| entities[*i].id).collect();
        let indices: BTreeSet<_> = members.iter().copied().collect();
        let closed = members.iter().all(|i| closed(&entities[*i].geometry))
            || endpoints
                .iter()
                .enumerate()
                .filter(|(_, e)| indices.contains(&e.0))
                .all(|(i, _)| degree[i] == 1)
                && endpoints.iter().any(|e| indices.contains(&e.0));
        let point = location(&entities[members[0]].geometry);
        let length: f64 = members.iter().map(|i| lengths[*i].0).sum();
        let closed = closed && length > EPS;
        let approximate = members.iter().any(|i| lengths[*i].1);
        let direction = if closed {
            direction(
                &members
                    .iter()
                    .map(|i| entities[*i].geometry.clone())
                    .collect::<Vec<_>>(),
            )
        } else {
            None
        };
        if !closed {
            report.push(issue(
                Kind::OpenContour,
                ids.clone(),
                point,
                "未形成端点唯一闭合的轮廓（可能开放、分支或求值失败）".into(),
            ));
        }
        report.contours.push(Contour {
            entities: ids,
            closed,
            length,
            approximate,
            direction,
            point,
        });
    }
    checks = 0;
    'crossings: for i in 0..segments.len() {
        for j in i + 1..segments.len() {
            checks += 1;
            if checks > PAIRS {
                report.truncated = true;
                break 'crossings;
            }
            let (a, b) = (&segments[i], &segments[j]);
            if a.approx
                && b.approx
                && a.id == b.id
                && (a.index.abs_diff(b.index) == 1
                    || a.closed && a.index.min(b.index) == 0 && a.index.max(b.index) + 1 == a.count)
            {
                continue;
            }
            if (0..3).any(|k| a.max[k] < b.min[k] || b.max[k] < a.min[k]) {
                continue;
            }
            if let Ok(points) = curves::intersections(&a.g, &b.g, true) {
                for point in points {
                    let endpoint = |g: &Geometry| {
                        !closed(g)
                            && [0.0, 1.0]
                                .into_iter()
                                .any(|t| curves::evaluate(g, t).is_ok_and(|p| near(p, point)))
                    };
                    if endpoint(&a.g) && endpoint(&b.g) {
                        continue;
                    }
                    let detail = if a.approx || b.approx {
                        "离散曲线交叉候选，需放大确认；不提供自动解交"
                    } else {
                        "解析线/圆弧在段内部相交，需核对是否为有效轮廓"
                    };
                    report.push(issue(
                        Kind::Intersection,
                        vec![a.id, b.id],
                        point,
                        detail.into(),
                    ));
                }
            }
        }
    }
    report.pairs += checks.min(PAIRS);
    report
}
fn direction(input: &[Geometry]) -> Option<(bool, Vector3)> {
    let g = if input.len() == 1 {
        input[0].clone()
    } else {
        crate::curve_edit::apply(
            crate::curve_edit::CurveTool::Join,
            input,
            crate::curve_edit::CurveOptions {
                tolerance: EPS,
                ..Default::default()
            },
        )
        .ok()?
        .remove(0)
    };
    let p = curves::sample(
        &g,
        curves::Sampling {
            chord: 0.05,
            angle: 0.1,
        },
    )
    .ok()?;
    let origin = *p.first()?;
    let axis = p
        .iter()
        .map(|p| delta(origin, *p))
        .find(|d| d.length() > EPS)?;
    let mut normal = p
        .iter()
        .map(|p| axis.cross(delta(origin, *p)))
        .find(|n| n.length() > EPS * axis.length())?
        .normalize();
    let values = [normal.x, normal.y, normal.z];
    let i = (0..3).max_by(|a, b| values[*a].abs().total_cmp(&values[*b].abs()))?;
    if values[i] < 0.0 {
        normal = normal * -1.0;
    }
    if p.iter().any(|p| delta(origin, *p).dot(normal).abs() > EPS) {
        return None;
    }
    let area: f64 = p
        .windows(2)
        .map(|p| delta(origin, p[0]).cross(delta(origin, p[1])).dot(normal) * 0.5)
        .sum();
    if area.abs() < EPS * EPS {
        None
    } else {
        Some((area > 0.0, normal))
    }
}
pub fn repair(doc: &DrawingDocument, item: &Issue, gap_limit: f64) -> Result<CurveEdit, String> {
    let mut edit = CurveEdit::default();
    let entity = |id| doc.entity(id).ok_or("诊断对象已失效，请重新扫描");
    match item.kind {
        Kind::Duplicate => {
            let keep = *item.entities.iter().min().ok_or("缺少重复对象")?;
            let first = entity(keep)?;
            for id in &item.entities {
                let other = entity(*id)?;
                if !same_geometry(&first.geometry, &other.geometry)
                    || first.color != other.color
                    || first.visible != other.visible
                    || first.layer_id != other.layer_id
                {
                    return Err("候选属性或几何不同，请人工核对后手动处理".into());
                }
                if *id != keep {
                    edit.remove.insert(*id);
                }
            }
        }
        Kind::ZeroLength => {
            let e = entity(item.entities[0])?;
            match &e.geometry {
                Geometry::Line(l) if curves::distance(l.start, l.end) <= EPS => {
                    edit.remove.insert(e.id);
                }
                Geometry::Polyline(p) => {
                    let mut p = p.clone();
                    p.segments.retain(|s| {
                        curves::segment_geometry(s, p.plane)
                            .and_then(|g| length(&g))
                            .is_ok_and(|v| v.0 > EPS)
                    });
                    if p.segments.is_empty() {
                        edit.remove.insert(e.id);
                    } else {
                        edit.replacements.insert(e.id, vec![Geometry::Polyline(p)]);
                    }
                }
                _ => return Err("该零长候选暂不支持自动修复".into()),
            }
        }
        Kind::Gap => {
            let (a, b) = item.gap.as_ref().ok_or("缺少间隙端点")?;
            if !gap_limit.is_finite() || curves::distance(a.point, b.point) > gap_limit {
                return Err("间隙超过当前候选上限，禁止补齐".into());
            }
            let e = entity(b.id)?;
            if a.id == b.id {
                let g = crate::curve_edit::apply(
                    crate::curve_edit::CurveTool::Close,
                    &[e.geometry.clone()],
                    Default::default(),
                )?;
                edit.replacements.insert(e.id, g);
            } else {
                let kind = b.grip.ok_or("该曲线端点没有可编辑夹点，请人工修整")?;
                let g = properties::move_grip(&e.geometry, kind, a.point)?;
                edit.replacements.insert(e.id, vec![g]);
            }
        }
        _ => return Err("该问题提供定位与选择，请使用曲线工具人工整理".into()),
    }
    Ok(edit)
}
