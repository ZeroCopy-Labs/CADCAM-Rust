//! Deterministic four-axis motion preview.
//!
//! The first implementation keeps the tube axis on X and maps each planar path
//! to X/Y/Z/A using the unit's owner zone. It is a preview layer, not yet a
//! validated machine post-processor.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

use crate::cam::CamModel;
use crate::cam_analysis::{
    check_boundary_pair, oriented_unit_path, oriented_unit_path_from, CamAnalysis, EndCutSide,
    TubeSection, TubeZone,
};
use crate::display::DisplayData;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TransferConfig {
    pub enabled: bool,
    pub retract_height: f64,
    pub approach_height: f64,
    pub height: f64,
    pub auto_height: bool,
    pub rise_ratio: f64,
    pub fall_ratio: f64,
    pub smooth_enabled: bool,
    pub smooth_tolerance: f64,
}

impl Default for TransferConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            retract_height: 2.0,
            approach_height: 2.0,
            height: 4.0,
            auto_height: false,
            rise_ratio: 0.5,
            fall_ratio: 0.5,
            smooth_enabled: false,
            smooth_tolerance: 0.05,
        }
    }
}

impl TransferConfig {
    pub fn total_height(self) -> f64 {
        if self.auto_height {
            self.retract_height.max(self.approach_height)
        } else {
            self.height
        }
    }

    pub fn validation_error(self) -> Option<&'static str> {
        if ![
            self.retract_height,
            self.approach_height,
            self.height,
            self.rise_ratio,
            self.fall_ratio,
            self.smooth_tolerance,
        ]
        .iter()
        .all(|v| v.is_finite())
        {
            return Some("联动转移参数必须为有限数值");
        }
        if self.smooth_tolerance <= 0.0 {
            return Some("平滑允许偏差必须大于零");
        }
        if self.retract_height < 0.0 || self.approach_height < 0.0 || self.height < 0.0 {
            return Some("联动抬落刀高度不能为负数");
        }
        if self.total_height() < self.retract_height.max(self.approach_height) {
            return Some("联动总高度不能小于最小抬刀/落刀高度");
        }
        if !(0.0..=1.0).contains(&self.rise_ratio)
            || !(0.0..=1.0).contains(&self.fall_ratio)
            || self.rise_ratio + self.fall_ratio > 1.0 + 1.0e-9
        {
            return Some("联动水平占比须在 0–100% 内且合计不超过 100%");
        }
        None
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RotaryConfig {
    pub a_offset_degrees: f64,
    pub machining_z_offset: f64,
    pub wall_thickness: f64,
    pub end_cut_correction: bool,
    pub rotation_safety_clearance: f64,
    pub same_zone_clearance: f64,
    pub transfer: TransferConfig,
    pub initial_position: Option<MachinePose>,
    pub return_position: Option<MachinePose>,
    pub overcut_distance: f64,
    pub keep_continuous_angle: bool,
    pub output_decimals: usize,
}

impl Default for RotaryConfig {
    fn default() -> Self {
        Self {
            a_offset_degrees: 0.0,
            machining_z_offset: 0.0,
            wall_thickness: 0.0,
            end_cut_correction: false,
            rotation_safety_clearance: 5.0,
            same_zone_clearance: 0.0,
            transfer: TransferConfig::default(),
            initial_position: None,
            return_position: None,
            overcut_distance: 0.0,
            keep_continuous_angle: true,
            output_decimals: 3,
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
pub enum ArcPlane {
    XY,
    XZ,
    YZ,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MachineMoveKind {
    Rapid,
    Cutting,
    Arc {
        clockwise: bool,
        plane: ArcPlane,
        center: [f64; 3],
    },
}

impl MachineMoveKind {
    pub fn is_cutting(self) -> bool {
        matches!(self, Self::Cutting | Self::Arc { .. })
    }
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

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct UnitRuleKeys {
    pub entity_type: String,
    pub layer: String,
    pub color: String,
}

#[derive(Clone, Debug, Default)]
pub struct MachineTrajectory {
    pub error: Option<String>,
    pub moves: Vec<MachineMove>,
    pub unit_ranges: Vec<TrajectoryUnitRange>,
    pub unit_rules: Vec<Vec<UnitRuleKeys>>,
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
    if let Some(error) = analysis.grouping_errors.first() {
        return MachineTrajectory {
            error: Some(error.clone()),
            ..Default::default()
        };
    }
    if !config.wall_thickness.is_finite()
        || config.wall_thickness < 0.0
        || (config.end_cut_correction && config.wall_thickness <= 0.0)
    {
        return MachineTrajectory {
            error: Some("壁厚必须为有限非负值，开启端面补偿时必须大于零".to_string()),
            ..Default::default()
        };
    }
    if !valid_config(config) {
        return MachineTrajectory::default();
    }
    let Some(section) = &analysis.tube_section else {
        if config.end_cut_correction {
            return MachineTrajectory {
                error: Some("壁厚端面补偿需要有效的方管截面和配对断面".to_string()),
                ..Default::default()
            };
        }
        return build_without_tube_section(display, cam, analysis, config);
    };
    if !valid_section(section) {
        return MachineTrajectory::default();
    }
    if config.end_cut_correction && config.wall_thickness >= section.half_y().min(section.half_z())
    {
        return MachineTrajectory {
            error: Some("壁厚达到半截面尺寸，内腔无效".to_string()),
            ..Default::default()
        };
    }
    let Some(bounds) = display.bounds else {
        return MachineTrajectory::default();
    };
    if ![
        bounds.min_x,
        bounds.min_y,
        bounds.min_z,
        bounds.max_x,
        bounds.max_y,
        bounds.max_z,
    ]
    .iter()
    .all(|value| value.is_finite())
    {
        return MachineTrajectory::default();
    }

    let center_y = section.center_y;
    let center_z = section.center_z;
    let radius_y = (bounds.min_y - center_y)
        .abs()
        .max((bounds.max_y - center_y).abs())
        .max(section.half_y());
    let radius_z = (bounds.min_z - center_z)
        .abs()
        .max((bounds.max_z - center_z).abs())
        .max(section.half_z());
    let safe_z = center_z
        + radius_y.hypot(radius_z)
        + config.machining_z_offset.max(0.0)
        + config.rotation_safety_clearance;
    if !safe_z.is_finite() {
        return MachineTrajectory::default();
    }
    let mut moves = Vec::new();
    let mut unit_ranges = Vec::new();
    let mut unit_rules = Vec::new();
    let mut unit_count = 0usize;
    let mut previous_end = config.initial_position;
    let mut previous_surface: Option<TubeZone> = None;
    let mut corrected_sections = BTreeMap::new();

    for group in &analysis.plan.groups {
        for unit in &group.units {
            let side = analysis.end_cut_side(unit.unit_id);
            let wall_cut = config.end_cut_correction && side != EndCutSide::Section;
            if config.end_cut_correction
                && unit.machining_cut
                && side == EndCutSide::Section
                && !analysis.trim_sections.contains(&unit.unit_id)
            {
                return MachineTrajectory {
                    error: Some(format!(
                        "断面单元 {} 尚未配对，请配对或显式标记为修边",
                        unit.order
                    )),
                    ..Default::default()
                };
            }
            let Some(mut path) = oriented_unit_path_from(
                display,
                cam,
                unit.unit_id,
                unit.direction,
                Some(unit.start_point),
            ) else {
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
            }
            let source_point_count = path.len();
            if closed {
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
                let mode = pose_mode(unit, analysis, section, cam);
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
            let mut previous_a = previous_end.map(|pose| pose.a).unwrap_or(0.0);
            for (point, (raw_angle, mode)) in path.iter().zip(point_poses.iter()) {
                let angle = wrap_angle_180(*raw_angle + wrap_angle_180(config.a_offset_degrees));
                let aligned = if poses.is_empty() {
                    align_entry_angle(previous_a, angle, config.keep_continuous_angle)
                } else {
                    unwrap_angle(previous_a, angle)
                };
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
            let mut cut_slopes = None;
            if wall_cut
                || (config.machining_z_offset != 0.0
                    && poses
                        .iter()
                        .any(|pose| (pose.a - poses[0].a).abs() > 1.0e-9))
            {
                let source = &path[..source_point_count];
                let direction = if unit.machining_cut {
                    section_cut_direction(source, section)
                } else {
                    fit_cut_direction(source).map_err(str::to_string)
                };
                let [y_slope, z_slope] = match direction {
                    Ok(direction) => direction,
                    Err(error) => {
                        return MachineTrajectory {
                            error: Some(format!(
                                "加工单元 {} 的修正方向求解失败：{error}",
                                unit.order
                            )),
                            ..Default::default()
                        }
                    }
                };
                cut_slopes = Some([y_slope, z_slope]);
            }
            if wall_cut {
                // Correct only geometric samples; extend the resulting cutting tangent for overcut.
                for (source, pose) in path.iter().zip(&mut poses).take(source_point_count) {
                    match crate::cam_wall::end_cut_allowance(
                        section,
                        config.wall_thickness,
                        *source,
                        [pose.x, pose.y, pose.z, pose.a],
                        cut_slopes.unwrap(),
                        side,
                    ) {
                        Ok(extra) => pose.x += extra,
                        Err(error) => {
                            return MachineTrajectory {
                                error: Some(format!(
                                    "加工单元 {} 的壁厚补偿失败：{error}",
                                    unit.order
                                )),
                                ..Default::default()
                            }
                        }
                    }
                }
            } else if let Some([y_slope, z_slope]) = cut_slopes {
                // End cuts already account for Z in their inner-wall projection.
                for pose in &mut poses {
                    let (sin, cos) = pose.a.to_radians().sin_cos();
                    // In workpiece coordinates the machine-Z offset is (dY, dZ) = h*(sin A, cos A).
                    pose.x += config.machining_z_offset * (y_slope * sin + z_slope * cos);
                }
            }
            if wall_cut {
                corrected_sections.insert(
                    unit.unit_id,
                    path.iter()
                        .zip(&poses)
                        .take(source_point_count)
                        .map(|(p, m)| [m.x, p[1], p[2]])
                        .collect::<Vec<_>>(),
                );
                if poses.len() > source_point_count {
                    let start = poses[0];
                    let next = poses[1];
                    let end = poses[source_point_count - 1];
                    let length = distance3(
                        [start.x, path[0][1], path[0][2]],
                        [next.x, path[1][1], path[1][2]],
                    );
                    if length > 1.0e-9 {
                        poses[source_point_count] = MachinePose {
                            x: end.x + (next.x - start.x) / length * config.overcut_distance,
                            y: end.y + (next.y - start.y) / length * config.overcut_distance,
                            z: end.z + (next.z - start.z) / length * config.overcut_distance,
                            a: end.a + (next.a - start.a) / length * config.overcut_distance,
                        };
                    }
                }
            }
            if poses.iter().any(|pose| !finite_pose(*pose)) {
                return MachineTrajectory::default();
            }

            let Some(first) = poses.first().copied() else {
                continue;
            };
            let last = poses.last().copied().unwrap_or(first);
            let start_index = moves.len();

            if unit_count == 0 {
                if let Some(initial) = config.initial_position {
                    push_rapid(&mut moves, initial);
                }
            }
            let surface = if !unit.machining_cut
                && flat_surface(unit.zone, section, &poses, config.machining_z_offset)
            {
                Some(unit.zone)
            } else {
                None
            };
            let local = surface.is_some() && surface == previous_surface;
            moves.extend(transfer_moves(
                previous_end,
                first,
                safe_z,
                local,
                config.transfer,
                config.same_zone_clearance,
                config.output_decimals,
            ));
            if let Some(arc) = arc_cut_info(
                display,
                cam,
                unit,
                &poses,
                center_y,
                center_z,
                config.machining_z_offset,
            ) {
                if arc.full_circle && poses.len() >= 4 {
                    let mid = poses[arc.mid_index];
                    moves.push(MachineMove {
                        kind: MachineMoveKind::Arc {
                            clockwise: arc.clockwise,
                            plane: arc.plane,
                            center: arc.center,
                        },
                        pose: mid,
                    });
                    moves.push(MachineMove {
                        kind: MachineMoveKind::Arc {
                            clockwise: arc.clockwise,
                            plane: arc.plane,
                            center: arc.center,
                        },
                        pose: first,
                    });
                    // A closed circle may have an overcut point appended after
                    // the geometric start. Emit it as a real move so the next
                    // transfer starts from the position actually reached.
                    let end_delta = ((first.x - last.x).powi(2)
                        + (first.y - last.y).powi(2)
                        + (first.z - last.z).powi(2))
                    .sqrt();
                    if end_delta > 1.0e-6 {
                        moves.push(MachineMove {
                            kind: MachineMoveKind::Cutting,
                            pose: last,
                        });
                    }
                } else if !arc.full_circle {
                    moves.push(MachineMove {
                        kind: MachineMoveKind::Arc {
                            clockwise: arc.clockwise,
                            plane: arc.plane,
                            center: arc.center,
                        },
                        pose: last,
                    });
                } else {
                    for pose in poses.iter().skip(1) {
                        moves.push(MachineMove {
                            kind: MachineMoveKind::Cutting,
                            pose: *pose,
                        });
                    }
                }
            } else {
                for pose in poses.iter().skip(1) {
                    moves.push(MachineMove {
                        kind: MachineMoveKind::Cutting,
                        pose: *pose,
                    });
                }
            }
            unit_ranges.push(TrajectoryUnitRange {
                order: unit.order,
                start: start_index,
                end: moves.len(),
            });
            unit_rules.push(unit_rule_keys(display, cam, unit));
            previous_end = moves.last().map(|movement| movement.pose);
            previous_surface = surface;
            unit_count += 1;
        }
    }

    if config.end_cut_correction {
        for group in &analysis.groups {
            if let (Some(left), Some(right)) = (group.left_section, group.right_section) {
                let (Some(l), Some(r)) = (
                    corrected_sections.get(&left),
                    corrected_sections.get(&right),
                ) else {
                    return MachineTrajectory {
                        error: Some("成品段的端面轨迹不完整".to_string()),
                        ..Default::default()
                    };
                };
                if let Err(error) = check_boundary_pair(l, r) {
                    return MachineTrajectory {
                        error: Some(format!("成品组 {} 补偿后空间不足：{error}", group.id + 1)),
                        ..Default::default()
                    };
                }
            }
        }
    }

    if let Some(previous) = previous_end {
        let mut target = config.return_position.unwrap_or(MachinePose {
            z: previous.z.max(safe_z),
            ..previous
        });
        target.a = unwrap_angle(previous.a, target.a);
        moves.extend(transfer_moves(
            Some(previous),
            target,
            safe_z,
            false,
            config.transfer,
            config.same_zone_clearance,
            config.output_decimals,
        ));
        if let Some(last_range) = unit_ranges.last_mut() {
            last_range.end = moves.len();
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

    let cutting_move_count = moves
        .iter()
        .filter(|movement| movement.kind.is_cutting())
        .count();
    MachineTrajectory {
        error: None,
        moves,
        unit_ranges,
        unit_rules,
        cutting_move_count,
        safe_z,
        unit_count,
        a_min: if a_min.is_finite() { a_min } else { 0.0 },
        a_max: if a_max.is_finite() { a_max } else { 0.0 },
    }
}

fn finite_pose(pose: MachinePose) -> bool {
    [pose.x, pose.y, pose.z, pose.a]
        .iter()
        .all(|v| v.is_finite())
}

fn fit_cut_direction(points: &[[f64; 3]]) -> Result<[f64; 2], &'static str> {
    if points.len() < 3 {
        return Err("方向采样不足，至少需要三个非共线点");
    }
    let mut mean = [0.0; 3];
    for point in points {
        for (value, coordinate) in mean.iter_mut().zip(point) {
            *value += coordinate / points.len() as f64;
        }
    }
    let (mut yy, mut yz, mut zz, mut xy, mut xz) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for point in points {
        let [x, y, z] = [point[0] - mean[0], point[1] - mean[1], point[2] - mean[2]];
        yy += y * y;
        yz += y * z;
        zz += z * z;
        xy += x * y;
        xz += x * z;
    }
    let determinant = yy * zz - yz * yz;
    if !determinant.is_finite() || yy <= 0.0 || zz <= 0.0 || determinant <= yy * zz * 1.0e-6 {
        return Err("YZ 采样退化，无法确定断面方向");
    }
    let slopes = [
        (xy * zz - xz * yz) / determinant,
        (xz * yy - xy * yz) / determinant,
    ];
    if !slopes.iter().all(|v| v.is_finite()) {
        return Err("断面方向包含非法数值");
    }
    if points.iter().any(|p| {
        (p[0] - mean[0] - slopes[0] * (p[1] - mean[1]) - slopes[1] * (p[2] - mean[2])).abs() > 0.01
    }) {
        return Err("采样不共面，不能可靠确定联动修正方向");
    }
    Ok(slopes)
}

fn section_cut_direction(path: &[[f64; 3]], section: &TubeSection) -> Result<[f64; 2], String> {
    let points = if path.len() > 1 && distance3(path[0], *path.last().unwrap()) <= 1.0e-9 {
        &path[..path.len() - 1]
    } else {
        path
    };
    if points.is_empty() {
        return Err("断面采样为空".to_string());
    }
    if points.iter().all(|p| (p[0] - points[0][0]).abs() <= 1.0e-9) {
        return Ok([0.0, 0.0]);
    }
    let r = section.corner_radius();
    let corners = [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)];
    let names = ["右上", "右下", "左下", "左上"];
    let mut directions = [[0.0; 2]; 4];
    for (corner, (sy, sz)) in corners.iter().enumerate() {
        let cy = section.center_y + sy * (section.half_y() - r);
        let cz = section.center_z + sz * (section.half_z() - r);
        let mut samples = Vec::new();
        for (index, point) in points.iter().enumerate() {
            let dy = point[1] - cy;
            let dz = point[2] - cz;
            if r > 1.0e-9 {
                if dy * sy >= -1.0e-8 && dz * sz >= -1.0e-8 && (dy.hypot(dz) - r).abs() <= 0.01 {
                    samples.push(*point);
                }
            } else if dy.hypot(dz) <= 1.0e-8 {
                // A sharp corner obtains its direction from the two adjacent edge segments.
                samples.extend_from_slice(&[
                    points[(index + points.len() - 1) % points.len()],
                    *point,
                    points[(index + 1) % points.len()],
                ]);
            }
        }
        directions[corner] = fit_cut_direction(&samples)
            .map_err(|error| format!("{}圆角/转角：{error}", names[corner]))?;
    }
    for i in 0..4 {
        for j in 0..i {
            let [a, b] = directions[i];
            let [c, d] = directions[j];
            let cosine =
                (1.0 + a * c + b * d) / (1.0_f64.hypot(a.hypot(b)) * 1.0_f64.hypot(c.hypot(d)));
            if cosine < 1.0_f64.to_radians().cos() {
                return Err(format!("{}与{}转角的断面方向不一致", names[i], names[j]));
            }
        }
    }
    // Equal corner weights keep dense sampling on one side from biasing the direction.
    Ok([
        directions.iter().map(|d| d[0] / 4.0).sum(),
        directions.iter().map(|d| d[1] / 4.0).sum(),
    ])
}

fn align_entry_angle(previous: f64, target: f64, continuous: bool) -> f64 {
    let aligned = unwrap_angle(previous, target);
    if continuous || (aligned - previous).abs() <= 1.0e-9 {
        aligned
    } else {
        target
    }
}

pub(crate) fn push_rapid(moves: &mut Vec<MachineMove>, pose: MachinePose) {
    if moves
        .last()
        .map(|last| {
            distance3(
                [last.pose.x, last.pose.y, last.pose.z],
                [pose.x, pose.y, pose.z],
            ) <= 1.0e-9
                && (last.pose.a - pose.a).abs() <= 1.0e-9
        })
        .unwrap_or(false)
    {
        return;
    }
    moves.push(MachineMove {
        kind: MachineMoveKind::Rapid,
        pose,
    });
}

/// Generate the same profile used by the settings diagram and real transfers.
/// `local` requires a known common horizontal surface, with no release boundary.
pub fn transfer_moves(
    start: Option<MachinePose>,
    end: MachinePose,
    safe_z: f64,
    local: bool,
    config: TransferConfig,
    legacy_clearance: f64,
    output_decimals: usize,
) -> Vec<MachineMove> {
    let mut moves = Vec::new();
    if config.validation_error().is_some()
        || !safe_z.is_finite()
        || !finite_pose(end)
        || start.is_some_and(|p| !finite_pose(p))
        || !legacy_clearance.is_finite()
        || legacy_clearance < 0.0
        || output_decimals > 9
    {
        return moves;
    }
    let Some(start) = start else {
        push_rapid(
            &mut moves,
            MachinePose {
                z: safe_z.max(end.z),
                ..end
            },
        );
        push_rapid(&mut moves, end);
        return moves;
    };
    // Equivalent angles describe the same pose, even with continuity disabled.
    let end = MachinePose {
        a: align_entry_angle(start.a, end.a, false),
        ..end
    };
    if config.smooth_enabled {
        if let Some(smoothed) = crate::cam_transfer::smooth_transfer(
            start,
            end,
            safe_z,
            local && (start.a - end.a).abs() <= 1.0e-6,
            config,
            output_decimals,
        ) {
            return smoothed;
        }
    }
    if local && (start.a - end.a).abs() <= 1.0e-6 {
        let distance = (end.x - start.x).hypot(end.y - start.y);
        if config.enabled && distance > 1.0e-9 {
            let peak = start.z.max(end.z) + config.total_height();
            let at = |t: f64, z: f64| MachinePose {
                x: start.x + (end.x - start.x) * t,
                y: start.y + (end.y - start.y) * t,
                z,
                a: start.a,
            };
            push_rapid(&mut moves, at(0.0, start.z + config.retract_height));
            push_rapid(&mut moves, at(config.rise_ratio, peak));
            push_rapid(&mut moves, at(1.0 - config.fall_ratio, peak));
            push_rapid(&mut moves, at(1.0, end.z + config.approach_height));
        } else {
            let lift = if config.enabled {
                (start.z + config.retract_height).max(end.z + config.approach_height)
            } else {
                start.z.max(end.z) + legacy_clearance
            };
            push_rapid(&mut moves, MachinePose { z: lift, ..start });
            push_rapid(&mut moves, MachinePose { z: lift, ..end });
        }
    } else {
        let clearance = safe_z.max(start.z).max(end.z);
        push_rapid(
            &mut moves,
            MachinePose {
                z: clearance,
                ..start
            },
        );
        push_rapid(
            &mut moves,
            MachinePose {
                z: clearance,
                ..end
            },
        );
    }
    push_rapid(&mut moves, end);
    if moves.len() > 1 && moves[0].pose == start {
        moves.remove(0);
    }
    moves
}

fn flat_surface(
    zone: TubeZone,
    section: &TubeSection,
    poses: &[MachinePose],
    z_offset: f64,
) -> bool {
    let (angle, height, width) = match zone {
        TubeZone::TopFace => (0.0, section.half_z(), section.half_y()),
        TubeZone::RightFace => (90.0, section.half_y(), section.half_z()),
        TubeZone::BottomFace => (180.0, section.half_z(), section.half_y()),
        TubeZone::LeftFace => (-90.0, section.half_y(), section.half_z()),
        _ => return false,
    };
    poses.iter().all(|p| {
        let delta = (p.a - angle).to_radians();
        delta.sin().abs() <= 1.0e-9
            && delta.cos() > 0.0
            && (p.z - (section.center_z + height + z_offset)).abs() <= 1.0e-6
            && (p.y - section.center_y).abs() <= width - section.corner_radius() + 1.0e-6
    })
}

fn unit_rule_keys(
    display: &DisplayData,
    cam: &CamModel,
    unit: &crate::cam_analysis::PlanUnit,
) -> Vec<UnitRuleKeys> {
    let Some(machining_unit) = cam.unit(unit.unit_id) else {
        return Vec::new();
    };
    let mut keys = Vec::new();
    for member in &machining_unit.members {
        let Some(entity) = display.entities.iter().find(|entity| entity.id == *member) else {
            continue;
        };
        let key = UnitRuleKeys {
            entity_type: entity.kind.to_string(),
            layer: entity.layer.clone(),
            color: entity.color_key.clone(),
        };
        if !keys.contains(&key) {
            keys.push(key);
        }
    }
    keys
}

fn valid_config(config: RotaryConfig) -> bool {
    [
        config.a_offset_degrees,
        config.machining_z_offset,
        config.rotation_safety_clearance,
        config.same_zone_clearance,
        config.overcut_distance,
    ]
    .iter()
    .all(|value| value.is_finite())
        && config.rotation_safety_clearance >= 0.0
        && config.same_zone_clearance >= 0.0
        && config.overcut_distance >= 0.0
        && config.output_decimals <= 9
        && config.transfer.validation_error().is_none()
        && config
            .initial_position
            .into_iter()
            .chain(config.return_position)
            .all(finite_pose)
}

fn valid_section(section: &TubeSection) -> bool {
    section.center_y.is_finite()
        && section.center_z.is_finite()
        && section.size_y.is_finite()
        && section.size_z.is_finite()
        && section.radius.is_finite()
        && section.size_y > 0.0
        && section.size_z > 0.0
        && section.radius >= 0.0
}

fn build_without_tube_section(
    display: &DisplayData,
    cam: &CamModel,
    analysis: &CamAnalysis,
    config: RotaryConfig,
) -> MachineTrajectory {
    let Some(bounds) = display.bounds else {
        return MachineTrajectory::default();
    };
    let safe_z = bounds.max_z + config.rotation_safety_clearance.max(1.0);
    let mut moves = Vec::new();
    let mut unit_ranges = Vec::new();
    let mut unit_rules = Vec::new();
    let mut cutting_move_count = 0usize;
    let mut unit_count = 0usize;
    let mut previous_end = config.initial_position;

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
            if closed && distance3(path[0], *path.last().unwrap()) > 1.0e-6 {
                path.push(path[0]);
            }

            let angle = align_entry_angle(
                previous_end.map(|pose| pose.a).unwrap_or(0.0),
                wrap_angle_180(config.a_offset_degrees),
                config.keep_continuous_angle,
            );
            let to_pose = |point: [f64; 3]| MachinePose {
                x: point[0],
                y: point[1],
                z: point[2],
                a: angle,
            };
            let first = to_pose(path[0]);
            let last = to_pose(*path.last().unwrap());
            let start_index = moves.len();
            if unit_count == 0 {
                if let Some(initial) = config.initial_position {
                    push_rapid(&mut moves, initial);
                }
            }
            moves.extend(transfer_moves(
                previous_end,
                first,
                safe_z,
                false,
                config.transfer,
                config.same_zone_clearance,
                config.output_decimals,
            ));
            for point in path.iter().skip(1) {
                moves.push(MachineMove {
                    kind: MachineMoveKind::Cutting,
                    pose: to_pose(*point),
                });
                cutting_move_count += 1;
            }
            unit_ranges.push(TrajectoryUnitRange {
                order: unit.order,
                start: start_index,
                end: moves.len(),
            });
            unit_rules.push(unit_rule_keys(display, cam, unit));
            previous_end = Some(last);
            unit_count += 1;
        }
    }

    if let Some(previous) = previous_end {
        let mut target = config.return_position.unwrap_or(MachinePose {
            z: previous.z.max(safe_z),
            ..previous
        });
        target.a = unwrap_angle(previous.a, target.a);
        moves.extend(transfer_moves(
            Some(previous),
            target,
            safe_z,
            false,
            config.transfer,
            config.same_zone_clearance,
            config.output_decimals,
        ));
        if let Some(last_range) = unit_ranges.last_mut() {
            last_range.end = moves.len();
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
        error: None,
        moves,
        unit_ranges,
        unit_rules,
        cutting_move_count,
        safe_z,
        unit_count,
        a_min: if a_min.is_finite() { a_min } else { 0.0 },
        a_max: if a_max.is_finite() { a_max } else { 0.0 },
    }
}

#[derive(Clone, Copy, Debug)]
struct ArcCutInfo {
    center: [f64; 3],
    plane: ArcPlane,
    clockwise: bool,
    full_circle: bool,
    mid_index: usize,
}

fn arc_cut_info(
    display: &DisplayData,
    cam: &CamModel,
    unit: &crate::cam_analysis::PlanUnit,
    poses: &[MachinePose],
    center_y: f64,
    center_z: f64,
    machining_z_offset: f64,
) -> Option<ArcCutInfo> {
    if unit.machining_cut || poses.len() < 3 {
        return None;
    }
    let members = &cam.unit(unit.unit_id)?.members;
    if members.len() != 1 {
        return None;
    }
    let arc = display.arcs.get(&members[0])?;
    let first_a = poses.first()?.a;
    if poses.iter().any(|pose| (pose.a - first_a).abs() > 1.0e-6) {
        return None;
    }

    let angle = first_a.to_radians();
    let (sin, cos) = angle.sin_cos();
    let center = [
        arc.center[0],
        center_y + (arc.center[1] - center_y) * cos - (arc.center[2] - center_z) * sin,
        center_z
            + (arc.center[1] - center_y) * sin
            + (arc.center[2] - center_z) * cos
            + machining_z_offset,
    ];
    let normal = [
        arc.normal[0],
        arc.normal[1] * cos - arc.normal[2] * sin,
        arc.normal[1] * sin + arc.normal[2] * cos,
    ];
    let absolute = [normal[0].abs(), normal[1].abs(), normal[2].abs()];
    let (plane, canonical) = if absolute[2] >= absolute[0] && absolute[2] >= absolute[1] {
        (ArcPlane::XY, [0.0, 0.0, 1.0])
    } else if absolute[1] >= absolute[0] {
        (ArcPlane::XZ, [0.0, 1.0, 0.0])
    } else {
        (ArcPlane::YZ, [1.0, 0.0, 0.0])
    };

    let mid_index = poses.len() / 2;
    if mid_index == 0 || mid_index >= poses.len() - 1 {
        return None;
    }
    let start = poses[0];
    let mid = poses[mid_index];
    let start_vector = [
        start.x - center[0],
        start.y - center[1],
        start.z - center[2],
    ];
    let mid_vector = [mid.x - center[0], mid.y - center[1], mid.z - center[2]];
    let cross = [
        start_vector[1] * mid_vector[2] - start_vector[2] * mid_vector[1],
        start_vector[2] * mid_vector[0] - start_vector[0] * mid_vector[2],
        start_vector[0] * mid_vector[1] - start_vector[1] * mid_vector[0],
    ];
    let sense = cross[0] * canonical[0] + cross[1] * canonical[1] + cross[2] * canonical[2];
    if sense.abs() <= 1.0e-12 {
        return None;
    }

    Some(ArcCutInfo {
        center,
        plane,
        clockwise: sense < 0.0,
        full_circle: arc.full_circle,
        mid_index,
    })
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
    cam: &CamModel,
) -> PoseMode {
    // A machining cut is a profile going around the tube. Its tool axis must
    // follow the local radial normal, not a fixed owner-face angle.
    if unit.machining_cut {
        return PoseMode::Radial;
    }
    // Open paths are single straight/curved strokes. Keep one tool orientation
    // for the whole stroke so the machine does not interpolate A while cutting.
    if cam
        .unit(unit.unit_id)
        .map(|machining_unit| !machining_unit.closed)
        .unwrap_or(false)
    {
        return PoseMode::Rotating;
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

fn wrap_angle_180(value: f64) -> f64 {
    let value = value.rem_euclid(360.0);
    if value > 180.0 {
        value - 360.0
    } else {
        value
    }
}

fn unwrap_angle(previous: f64, target: f64) -> f64 {
    let mut delta = (target - previous) % 360.0;
    if delta > 180.0 {
        delta -= 360.0;
    }
    if delta < -180.0 {
        delta += 360.0;
    }
    previous + delta
}

fn distance3(left: [f64; 3], right: [f64; 3]) -> f64 {
    let dx = right[0] - left[0];
    let dy = right[1] - left[1];
    let dz = right[2] - left[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}
