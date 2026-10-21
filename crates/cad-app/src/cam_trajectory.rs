//! Deterministic four-axis motion preview.
//!
//! The first implementation keeps the tube axis on X and maps each planar path
//! to X/Y/Z/A using the unit's owner zone. It is a preview layer, not yet a
//! validated machine post-processor.

use crate::cam::CamModel;
use crate::cam_analysis::{oriented_unit_path, CamAnalysis, TubeSection, TubeZone};
use crate::display::DisplayData;

#[derive(Clone, Copy, Debug)]
pub struct RotaryConfig {
    pub a_offset_degrees: f64,
    pub machining_z_offset: f64,
    pub rotation_safety_clearance: f64,
    pub overcut_distance: f64,
    pub feed: f64,
}

impl Default for RotaryConfig {
    fn default() -> Self {
        Self {
            a_offset_degrees: 0.0,
            machining_z_offset: 0.0,
            rotation_safety_clearance: 5.0,
            overcut_distance: 0.0,
            feed: 1000.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MachinePose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub a: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MachineMoveKind {
    Rapid,
    Cutting,
}

#[derive(Clone, Copy, Debug)]
pub struct MachineMove {
    pub kind: MachineMoveKind,
    pub pose: MachinePose,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TrajectoryUnitRange {
    pub order: u32,
    pub start: usize,
    pub end: usize,
}

#[derive(Clone, Debug, Default)]
pub struct MachineTrajectory {
    pub moves: Vec<MachineMove>,
    pub unit_ranges: Vec<TrajectoryUnitRange>,
    pub cutting_move_count: usize,
    pub safe_z: f64,
    pub unit_count: usize,
    pub a_min: f64,
    pub a_max: f64,
}

/// Build a deterministic four-axis preview from the current CAM plan.
pub fn build_trajectory(
    display: &DisplayData,
    cam: &CamModel,
    analysis: &CamAnalysis,
    config: RotaryConfig,
) -> MachineTrajectory {
    let Some(section) = &analysis.tube_section else {
        return MachineTrajectory::default();
    };
    let Some(bounds) = display.bounds else {
        return MachineTrajectory::default();
    };

    let center_y = section.center_y;
    let center_z = section.center_z;
    let max_radius = [
        (bounds.min_y - center_y).abs(),
        (bounds.max_y - center_y).abs(),
        (bounds.min_z - center_z).abs(),
        (bounds.max_z - center_z).abs(),
    ]
    .into_iter()
    .fold(0.0f64, f64::max)
    .max(1.0);
    let safe_z = center_z + max_radius + config.rotation_safety_clearance;
    let mut moves = Vec::new();
    let mut unit_ranges = Vec::new();
    let mut cutting_move_count = 0usize;
    let mut unit_count = 0usize;
    let mut previous_a = 0.0f64;

    for group in &analysis.plan.groups {
        for unit in &group.units {
            let Some(mut path) = oriented_unit_path(display, cam, unit.unit_id, unit.direction)
            else {
                continue;
            };
            if path.len() < 2 {
                continue;
            }
            let closed = cam
                .unit(unit.unit_id)
                .map(|machining_unit| machining_unit.closed)
                .unwrap_or(false);
            if closed {
                // Close the path if the source tessellation omitted the final
                // coincident point.
                if distance3(path[0], *path.last().unwrap()) > 1.0e-6 {
                    path.push(path[0]);
                }
                // Optional overcut continues along the initial cutting tangent.
                if config.overcut_distance > 1.0e-9 && path.len() >= 2 {
                    let start = path[0];
                    let direction = [
                        path[1][0] - start[0],
                        path[1][1] - start[1],
                        path[1][2] - start[2],
                    ];
                    let length = (direction[0] * direction[0]
                        + direction[1] * direction[1]
                        + direction[2] * direction[2])
                        .sqrt();
                    if length > 1.0e-9 {
                        path.push([
                            start[0] + direction[0] / length * config.overcut_distance,
                            start[1] + direction[1] / length * config.overcut_distance,
                            start[2] + direction[2] / length * config.overcut_distance,
                        ]);
                    }
                }
            }

            let point_poses: Vec<(f64, PoseMode)> = if unit.machining_cut {
                // A section loop contains both straight face pieces and corner
                // arcs. Classify every point so straight face pieces keep a
                // fixed A angle instead of being emitted as curved motion.
                path.iter()
                    .map(|point| section_point_pose(*point, section))
                    .collect()
            } else {
                let mode = pose_mode(unit, analysis, section);
                path.iter()
                    .map(|point| {
                        let angle = if mode == PoseMode::Radial {
                            radial_angle_degrees(*point, section)
                        } else {
                            raw_angle_degrees(unit.zone, *point, section)
                        };
                        (angle, mode)
                    })
                    .collect()
            };

            let mut poses = Vec::with_capacity(path.len());
            for (point, (raw_angle, mode)) in path.iter().zip(point_poses.iter()) {
                let angle = *raw_angle + config.a_offset_degrees;
                let aligned = unwrap_angle(previous_a, angle);
                let pose = match mode {
                    PoseMode::Radial => radial_pose(
                        *point,
                        center_y,
                        center_z,
                        aligned,
                        config.machining_z_offset,
                    ),
                    PoseMode::Rotating => pose_from_point(
                        *point,
                        center_y,
                        center_z,
                        aligned,
                        config.machining_z_offset,
                    ),
                };
                poses.push(pose);
                previous_a = aligned;
            }

            let Some(first) = poses.first().copied() else {
                continue;
            };
            let last = poses.last().copied().unwrap_or(first);
            let start_index = moves.len();

            // Rotate at a safe height, approach, cut, then retract before the
            // next unit can perform a rapid transfer.
            moves.push(MachineMove {
                kind: MachineMoveKind::Rapid,
                pose: MachinePose {
                    x: first.x,
                    y: first.y,
                    z: safe_z,
                    a: first.a,
                },
            });
            moves.push(MachineMove {
                kind: MachineMoveKind::Rapid,
                pose: first,
            });
            for pose in poses.iter().skip(1) {
                moves.push(MachineMove {
                    kind: MachineMoveKind::Cutting,
                    pose: *pose,
                });
                cutting_move_count += 1;
            }
            moves.push(MachineMove {
                kind: MachineMoveKind::Rapid,
                pose: MachinePose {
                    x: last.x,
                    y: last.y,
                    z: safe_z,
                    a: last.a,
                },
            });
            unit_ranges.push(TrajectoryUnitRange {
                order: unit.order,
                start: start_index,
                end: moves.len(),
            });
            unit_count += 1;
        }
    }

    let a_min = moves
        .iter()
        .map(|movement| movement.pose.a)
        .fold(f64::INFINITY, f64::min);
    let a_max = moves
        .iter()
        .map(|movement| movement.pose.a)
        .fold(f64::NEG_INFINITY, f64::max);

    MachineTrajectory {
        moves,
        unit_ranges,
        cutting_move_count,
        safe_z,
        unit_count,
        a_min: if a_min.is_finite() { a_min } else { 0.0 },
        a_max: if a_max.is_finite() { a_max } else { 0.0 },
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PoseMode {
    Rotating,
    Radial,
}

fn pose_mode(
    unit: &crate::cam_analysis::PlanUnit,
    analysis: &CamAnalysis,
    _section: &TubeSection,
) -> PoseMode {
    // A machining cut is a profile going around the tube. Its tool axis must
    // follow the local radial normal, not a fixed owner-face angle.
    if unit.machining_cut {
        return PoseMode::Radial;
    }
    let touched = analysis
        .zone_profiles
        .get(&unit.unit_id)
        .map(|profile| profile.touched_zones.iter().filter(|hit| **hit).count())
        .unwrap_or(1);
    if touched > 2 {
        PoseMode::Radial
    } else {
        PoseMode::Rotating
    }
}

fn radial_angle_degrees(point: [f64; 3], section: &TubeSection) -> f64 {
    let dy = point[1] - section.center_y;
    let dz = point[2] - section.center_z;
    dy.atan2(dz).to_degrees()
}

fn section_point_pose(point: [f64; 3], section: &TubeSection) -> (f64, PoseMode) {
    let half_y = section.half_y();
    let half_z = section.half_z();
    let radius = section.corner_radius();
    let local_y = point[1] - section.center_y;
    let local_z = point[2] - section.center_z;
    let tolerance = (half_y.max(half_z) * 0.01).clamp(0.01, 0.1);
    let face_z_limit = half_z - radius + tolerance;
    let face_y_limit = half_y - radius + tolerance;

    let mut best_face: Option<(f64, f64)> = None;
    let candidates = [
        (half_z - local_z, 0.0, local_y.abs() <= face_y_limit),
        (local_y - half_y, 90.0, local_z.abs() <= face_z_limit),
        (local_z + half_z, 180.0, local_y.abs() <= face_y_limit),
        (local_y + half_y, -90.0, local_z.abs() <= face_z_limit),
    ];
    for (distance, angle, in_face) in candidates {
        if in_face && distance.abs() <= tolerance {
            if best_face
                .map(|(best_distance, _)| distance.abs() < best_distance)
                .unwrap_or(true)
            {
                best_face = Some((distance.abs(), angle));
            }
        }
    }
    if let Some((_, angle)) = best_face {
        return (angle, PoseMode::Rotating);
    }

    // Corner arcs are classified around their real corner center.
    let sign_y = if local_y >= 0.0 { 1.0 } else { -1.0 };
    let sign_z = if local_z >= 0.0 { 1.0 } else { -1.0 };
    let corner_y = sign_y * (half_y - radius);
    let corner_z = sign_z * (half_z - radius);
    let raw_angle = (local_y - corner_y).atan2(local_z - corner_z).to_degrees();
    (raw_angle, PoseMode::Rotating)
}

fn radial_pose(
    point: [f64; 3],
    center_y: f64,
    center_z: f64,
    angle_degrees: f64,
    machining_z_offset: f64,
) -> MachinePose {
    let dy = point[1] - center_y;
    let dz = point[2] - center_z;
    let radius = dy.hypot(dz);
    MachinePose {
        x: point[0],
        y: center_y,
        z: center_z + radius + machining_z_offset,
        a: angle_degrees,
    }
}

fn raw_angle_degrees(zone: TubeZone, point: [f64; 3], section: &TubeSection) -> f64 {
    match zone {
        TubeZone::TopFace => 0.0,
        TubeZone::RightFace => 90.0,
        TubeZone::BottomFace => 180.0,
        TubeZone::LeftFace => -90.0,
        TubeZone::TopToTopRightBoundary => 0.0,
        TubeZone::TopRightToRightBoundary => 90.0,
        TubeZone::RightToBottomRightBoundary => 90.0,
        TubeZone::BottomRightToBottomBoundary => 180.0,
        TubeZone::BottomToBottomLeftBoundary => 180.0,
        TubeZone::BottomLeftToLeftBoundary => -90.0,
        TubeZone::LeftToTopLeftBoundary => -90.0,
        TubeZone::TopLeftToTopBoundary => 0.0,
        TubeZone::TopRightCorner
        | TubeZone::BottomRightCorner
        | TubeZone::BottomLeftCorner
        | TubeZone::TopLeftCorner => {
            let (center_y, center_z) = corner_center(zone, section);
            let dy = point[1] - section.center_y - center_y;
            let dz = point[2] - section.center_z - center_z;
            dy.atan2(dz).to_degrees()
        }
    }
}

fn corner_center(zone: TubeZone, section: &TubeSection) -> (f64, f64) {
    let half_y = section.half_y();
    let half_z = section.half_z();
    let radius = section.corner_radius();
    match zone {
        TubeZone::TopRightCorner => (half_y - radius, half_z - radius),
        TubeZone::BottomRightCorner => (half_y - radius, -half_z + radius),
        TubeZone::BottomLeftCorner => (-half_y + radius, -half_z + radius),
        TubeZone::TopLeftCorner => (-half_y + radius, half_z - radius),
        _ => (0.0, 0.0),
    }
}

fn pose_from_point(
    point: [f64; 3],
    center_y: f64,
    center_z: f64,
    angle_degrees: f64,
    machining_z_offset: f64,
) -> MachinePose {
    let angle = angle_degrees.to_radians();
    let (sin, cos) = angle.sin_cos();
    let local_y = point[1] - center_y;
    let local_z = point[2] - center_z;
    let machine_y = local_y * cos - local_z * sin;
    let machine_z = local_y * sin + local_z * cos;
    MachinePose {
        x: point[0],
        y: center_y + machine_y,
        z: center_z + machine_z + machining_z_offset,
        a: angle_degrees,
    }
}

fn unwrap_angle(previous: f64, target: f64) -> f64 {
    let mut value = target;
    while value - previous > 180.0 {
        value -= 360.0;
    }
    while value - previous < -180.0 {
        value += 360.0;
    }
    value
}

fn distance3(left: [f64; 3], right: [f64; 3]) -> f64 {
    let dx = right[0] - left[0];
    let dy = right[1] - left[1];
    let dz = right[2] - left[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}
