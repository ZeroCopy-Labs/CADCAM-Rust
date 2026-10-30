//! Split planar X-monotone contours into outward and return passes.
use crate::{
    cam::CamModel,
    cam_analysis::{oriented_unit_path, PlanGroup, PlanUnit, TubeZone},
    display::DisplayData,
};
use serde::{Deserialize, Serialize};

type Point = [f64; 3];

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct FlyConfig {
    pub enabled: bool,
    pub row_tolerance: f64,
    pub max_gap: f64,
    pub link_tolerance: f64,
}
impl Default for FlyConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            row_tolerance: 0.5,
            max_gap: 100.0,
            link_tolerance: 0.02,
        }
    }
}
impl FlyConfig {
    pub fn validation_error(self) -> Option<&'static str> {
        if ![self.row_tolerance, self.max_gap, self.link_tolerance]
            .iter()
            .all(|v| v.is_finite() && *v > 0.0)
        {
            Some("飞扫行容差、最大孔间距及连接公差必须为有限正数")
        } else {
            None
        }
    }
}

pub struct Operation {
    pub unit: PlanUnit,
    pub path: Option<Vec<Point>>,
    /// Prefix includes the previous endpoint and the next cutting start.
    pub gap_points: usize,
    pub flying: bool,
}
fn ordinary(unit: &PlanUnit) -> Operation {
    Operation {
        unit: unit.clone(),
        path: None,
        gap_points: 0,
        flying: false,
    }
}
struct Hole {
    unit: PlanUnit,
    forward: Vec<Point>,
    backward: Vec<Point>,
    row: f64,
    surface: f64,
    width: f64,
}
fn hole(unit: &PlanUnit, display: &DisplayData, cam: &CamModel) -> Option<Hole> {
    if unit.machining_cut || !cam.unit(unit.unit_id)?.closed {
        return None;
    }
    let (across, normal) = match unit.zone {
        TubeZone::TopFace | TubeZone::BottomFace => (1, 2),
        TubeZone::LeftFace | TubeZone::RightFace => (2, 1),
        _ => return None,
    };
    let mut path = oriented_unit_path(display, cam, unit.unit_id, unit.direction)?;
    if path.len() < 4 {
        return None;
    }
    if distance(path[0], *path.last()?) < 1e-6 {
        path.pop();
    }
    let surface = path[0][normal];
    if path.iter().any(|p| (p[normal] - surface).abs() > 1e-5) {
        return None;
    }
    let min_x = path.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    let max_x = path.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
    let min = path.iter().map(|p| p[across]).fold(f64::INFINITY, f64::min);
    let max = path
        .iter()
        .map(|p| p[across])
        .fold(f64::NEG_INFINITY, f64::max);
    let row = (min + max) * 0.5;
    // Split flat extreme edges at their middle, so rectangles turn on a side
    // rather than adding a gap connector at a corner.
    let mut split = Vec::new();
    for i in 0..path.len() {
        let a = path[i];
        let b = path[(i + 1) % path.len()];
        split.push(a);
        if (a[0] - b[0]).abs() < 1e-9
            && ((a[0] - min_x).abs() < 1e-9 || (a[0] - max_x).abs() < 1e-9)
        {
            split.push(std::array::from_fn(|k| (a[k] + b[k]) * 0.5));
        }
    }
    path = split;
    let extreme = |x: f64| {
        (0..path.len())
            .filter(|i| (path[*i][0] - x).abs() < 1e-9)
            .min_by(|a, b| {
                (path[*a][across] - row)
                    .abs()
                    .total_cmp(&(path[*b][across] - row).abs())
            })
    };
    let left = extreme(min_x)?;
    let right = extreme(max_x)?;
    if path[right][0] - path[left][0] < 1e-6 {
        return None;
    }
    let chain = |reverse: bool| {
        let mut result = vec![path[left]];
        let mut i = left;
        while i != right {
            i = (i + if reverse { path.len() - 1 } else { 1 }) % path.len();
            result.push(path[i]);
        }
        result
    };
    let forward = chain(false);
    let mut backward = chain(true);
    if [&forward, &backward]
        .iter()
        .any(|p| p.windows(2).any(|w| w[1][0] < w[0][0] - 1e-6))
    {
        return None;
    }
    backward.reverse();
    Some(Hole {
        unit: unit.clone(),
        forward,
        backward,
        row,
        surface,
        width: max - min,
    })
}

pub fn operations(
    group: &PlanGroup,
    display: &DisplayData,
    cam: &CamModel,
    config: FlyConfig,
    overcut: f64,
) -> Vec<Operation> {
    if !config.enabled || overcut > 1e-9 {
        return group.units.iter().map(ordinary).collect();
    }
    let mut output = Vec::new();
    let mut i = 0;
    while i < group.units.len() {
        let Some(first) = hole(&group.units[i], display, cam) else {
            output.push(ordinary(&group.units[i]));
            i += 1;
            continue;
        };
        let input_start = i;
        let output_start = output.len();
        let mut holes = vec![first];
        i += 1;
        while i < group.units.len() {
            let Some(next) = hole(&group.units[i], display, cam) else {
                break;
            };
            holes.push(next);
            i += 1;
        }
        let mut surfaces: Vec<Vec<Hole>> = Vec::new();
        for h in holes {
            if let Some(surface) = surfaces
                .iter_mut()
                .find(|s| s[0].unit.zone == h.unit.zone && (s[0].surface - h.surface).abs() <= 1e-5)
            {
                surface.push(h);
            } else {
                surfaces.push(vec![h]);
            }
        }
        for mut holes in surfaces {
            holes.sort_by(|a, b| a.row.total_cmp(&b.row));
            let mut rows: Vec<Vec<Hole>> = Vec::new();
            for h in holes {
                if rows
                    .last()
                    .is_none_or(|r| (h.row - r[0].row).abs() > config.row_tolerance)
                {
                    rows.push(Vec::new());
                }
                rows.last_mut().unwrap().push(h);
            }
            for mut row in rows {
                row.sort_by(|a, b| a.forward[0][0].total_cmp(&b.forward[0][0]));
                let mut batch: Vec<Hole> = Vec::new();
                for h in row {
                    if let Some(previous) = batch.last() {
                        let gap = h.forward[0][0] - previous.forward.last().unwrap()[0];
                        if gap <= 1e-6 || gap > config.max_gap {
                            emit(&mut output, &batch, config);
                            batch.clear();
                        }
                    }
                    batch.push(h);
                }
                emit(&mut output, &batch, config);
            }
        }
        if output[output_start..].iter().all(|op| !op.flying) {
            output.truncate(output_start);
            output.extend(group.units[input_start..i].iter().map(ordinary));
        }
    }
    output
}
fn emit(output: &mut Vec<Operation>, batch: &[Hole], config: FlyConfig) {
    if batch.len() < 2 {
        output.extend(batch.iter().map(|h| ordinary(&h.unit)));
        return;
    }
    let mut previous: Option<Vec<Point>> = None;
    let mut previous_width = 0.0;
    let batch_start = output.len();
    for (h, returning) in batch
        .iter()
        .map(|h| (h, false))
        .chain(batch[..batch.len() - 1].iter().rev().map(|h| (h, true)))
    {
        let mut path = if returning {
            h.backward.clone()
        } else {
            h.forward.clone()
        };
        // Turn along the farthest hole itself, keeping the laser on at its seam.
        if !returning && h.unit.unit_id == batch.last().unwrap().unit.unit_id {
            path.extend(h.backward.iter().skip(1));
        }
        let cut = path.clone();
        let mut gap_points = 0;
        if let Some(previous) = &previous {
            let Some(link) = link(
                previous,
                &path,
                h.width.min(previous_width),
                config.link_tolerance,
            ) else {
                output.truncate(batch_start);
                output.extend(batch.iter().map(|h| ordinary(&h.unit)));
                return;
            };
            gap_points = link.len();
            path = link.into_iter().chain(path.into_iter().skip(1)).collect();
        }
        output.push(Operation {
            unit: h.unit.clone(),
            path: Some(path),
            gap_points,
            flying: true,
        });
        previous = Some(cut);
        previous_width = h.width;
    }
}
fn distance(a: Point, b: Point) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}
fn link(previous: &[Point], next: &[Point], width: f64, tolerance: f64) -> Option<Vec<Point>> {
    let a = previous[previous.len() - 1];
    let b = next[0];
    let length = ((a[0] - b[0]).abs() / 3.0).min(width * 0.5);
    let control = |p: Point, q: Point, origin: Point, sign: f64| {
        let scale = sign * length / distance(p, q).max(1e-12);
        std::array::from_fn(|i| origin[i] + (q[i] - p[i]) * scale)
    };
    let c = control(previous[previous.len() - 2], a, a, 1.0);
    let d = control(b, next[1], b, -1.0);
    let mut result = vec![a];
    flatten([a, c, d, b], tolerance, 0, &mut result).then_some(result)
}
fn flatten(p: [Point; 4], tolerance: f64, depth: u32, output: &mut Vec<Point>) -> bool {
    let deviation = |q: Point| {
        let v: Point = std::array::from_fn(|i| p[3][i] - p[0][i]);
        let length2 = v.iter().map(|x| x * x).sum::<f64>();
        let t = ((0..3).map(|i| (q[i] - p[0][i]) * v[i]).sum::<f64>() / length2.max(1e-24))
            .clamp(0.0, 1.0);
        distance(q, std::array::from_fn(|i| p[0][i] + v[i] * t))
    };
    if deviation(p[1]).max(deviation(p[2])) <= tolerance {
        output.push(p[3]);
        return true;
    }
    if depth >= 16 {
        return false;
    }
    let mid = |a: Point, b: Point| std::array::from_fn(|i| (a[i] + b[i]) * 0.5);
    let a = mid(p[0], p[1]);
    let b = mid(p[1], p[2]);
    let c = mid(p[2], p[3]);
    let d = mid(a, b);
    let e = mid(b, c);
    let f = mid(d, e);
    flatten([p[0], a, d, f], tolerance, depth + 1, output)
        && flatten([f, e, c, p[3]], tolerance, depth + 1, output)
}
