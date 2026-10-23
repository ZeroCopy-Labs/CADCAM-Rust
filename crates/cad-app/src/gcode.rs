//! TOML-driven G-code profile and text rendering.
//!
//! The renderer consumes the semantic machine trajectory. It never changes the
//! machining order, directions or starts.

use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::cam_trajectory::{
    ArcPlane, MachineMove, MachineMoveKind, MachinePose, MachineTrajectory, UnitRuleKeys,
};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct CodeRule {
    pub header: String,
    pub footer: String,
    pub comment: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct GCodeProfile {
    pub name: String,
    pub header: String,
    pub footer: String,
    pub unit_start: String,
    pub unit_end: String,
    pub rapid: String,
    pub cutting: String,
    pub feed: f64,
    pub decimals: usize,
    pub line_ending: String,
    pub overcut: f64,
    pub arc_tolerance: f64,
    pub a_offset_degrees: f64,
    pub machining_z_offset: f64,
    pub rotation_safety_clearance: f64,
    pub same_zone_clearance: f64,
    pub arc_output: bool,
    pub keep_continuous_angle: bool,
    pub use_initial_position: bool,
    pub initial_x: f64,
    pub initial_y: f64,
    pub initial_z: f64,
    pub initial_a: f64,
    pub return_to_initial: bool,
    pub entity_type_rules: BTreeMap<String, CodeRule>,
    pub color_rules: BTreeMap<String, CodeRule>,
    pub layer_rules: BTreeMap<String, CodeRule>,
}

impl Default for GCodeProfile {
    fn default() -> Self {
        let mut entity_type_rules = BTreeMap::new();
        for key in ["LINE", "ARC", "CIRCLE", "ELLIPSE", "POLYLINE", "SPLINE"] {
            entity_type_rules.insert(key.to_string(), CodeRule::default());
        }
        let mut color_rules = BTreeMap::new();
        for key in [
            "BYLAYER", "BYBLOCK", "ACI:1", "ACI:2", "ACI:3", "ACI:4", "ACI:5", "ACI:6", "ACI:7",
        ] {
            color_rules.insert(key.to_string(), CodeRule::default());
        }
        let mut layer_rules = BTreeMap::new();
        layer_rules.insert("0".to_string(), CodeRule::default());

        Self {
            name: "default".to_string(),
            header: "G90\nG21".to_string(),
            footer: "M30".to_string(),
            unit_start: "M03".to_string(),
            unit_end: "M05".to_string(),
            rapid: "G00".to_string(),
            cutting: "G01".to_string(),
            feed: 1000.0,
            decimals: 3,
            line_ending: "crlf".to_string(),
            overcut: 0.0,
            arc_tolerance: 0.1,
            a_offset_degrees: 0.0,
            machining_z_offset: 0.0,
            rotation_safety_clearance: 5.0,
            same_zone_clearance: 0.0,
            arc_output: true,
            keep_continuous_angle: true,
            use_initial_position: false,
            initial_x: 0.0,
            initial_y: 0.0,
            initial_z: 0.0,
            initial_a: 0.0,
            return_to_initial: false,
            entity_type_rules,
            color_rules,
            layer_rules,
        }
    }
}

impl GCodeProfile {
    pub fn rotary_config(&self) -> crate::cam_trajectory::RotaryConfig {
        crate::cam_trajectory::RotaryConfig {
            a_offset_degrees: self.a_offset_degrees,
            machining_z_offset: self.machining_z_offset,
            rotation_safety_clearance: self.rotation_safety_clearance,
            same_zone_clearance: self.same_zone_clearance,
            overcut_distance: self.overcut,
            keep_continuous_angle: self.keep_continuous_angle,
            feed: self.feed,
        }
    }

    pub fn load(path: &Path) -> Result<Self, String> {
        let text = fs::read_to_string(path).map_err(|error| error.to_string())?;
        toml::from_str(&text).map_err(|error| error.to_string())
    }

    pub fn save(&self, path: &Path) -> Result<(), String> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|error| error.to_string())?;
        }
        let text = toml::to_string_pretty(self).map_err(|error| error.to_string())?;
        fs::write(path, text).map_err(|error| error.to_string())
    }

    pub fn load_or_create(path: &Path) -> Self {
        if path.exists() {
            match Self::load(path) {
                Ok(profile) => {
                    // Rewrite valid files so newly added fields appear in TOML.
                    let _ = profile.save(path);
                    profile
                }
                // Keep an unreadable file untouched; the user may have a
                // hand-edited or newer-format profile worth recovering.
                Err(_) => Self::default(),
            }
        } else {
            let profile = Self::default();
            let _ = profile.save(path);
            profile
        }
    }
}

pub fn render_gcode(trajectory: &MachineTrajectory, profile: &GCodeProfile) -> String {
    let mut lines: Vec<String> = Vec::new();
    push_block(&mut lines, &profile.header);
    let mut previous_pose: Option<MachinePose> = None;
    if profile.use_initial_position {
        let pose = MachinePose {
            x: profile.initial_x,
            y: profile.initial_y,
            z: profile.initial_z,
            a: profile.initial_a,
        };
        lines.push(format!(
            "{} {}",
            profile.rapid,
            format_axis(pose, profile.decimals)
        ));
        previous_pose = Some(pose);
    }

    for (index, range) in trajectory.unit_ranges.iter().enumerate() {
        let start = range.start.min(trajectory.moves.len());
        let end = range.end.min(trajectory.moves.len());
        if start >= end {
            continue;
        }
        let keys = trajectory
            .unit_rules
            .get(index)
            .cloned()
            .unwrap_or_default();
        let moves = &trajectory.moves[start..end];
        let first_cut = moves.iter().position(|movement| movement.kind.is_cutting());
        let last_cut = moves
            .iter()
            .rposition(|movement| movement.kind.is_cutting());

        for movement in moves.iter().take(first_cut.unwrap_or(moves.len())) {
            push_movement(&mut lines, movement, profile, &mut previous_pose);
        }

        if let (Some(first_cut), Some(last_cut)) = (first_cut, last_cut) {
            push_rule_headers(&mut lines, profile, &keys);
            push_block(&mut lines, &profile.unit_start);
            for movement in &moves[first_cut..=last_cut] {
                push_movement(&mut lines, movement, profile, &mut previous_pose);
            }
            push_block(&mut lines, &profile.unit_end);
            push_rule_footers(&mut lines, profile, &keys);
            for movement in moves.iter().skip(last_cut + 1) {
                push_movement(&mut lines, movement, profile, &mut previous_pose);
            }
        }
    }

    if profile.return_to_initial {
        let pose = MachinePose {
            x: profile.initial_x,
            y: profile.initial_y,
            z: profile.initial_z,
            a: profile.initial_a,
        };
        lines.push(format!(
            "{} {}",
            profile.rapid,
            format_axis(pose, profile.decimals)
        ));
    }

    push_block(&mut lines, &profile.footer);

    let ending = if profile.line_ending.eq_ignore_ascii_case("lf") {
        "\n"
    } else {
        "\r\n"
    };
    let mut text = lines.join(ending);
    text.push_str(ending);
    text
}

pub fn validate_trajectory(trajectory: &MachineTrajectory, profile: &GCodeProfile) -> Vec<String> {
    let mut issues = Vec::new();
    if !profile.feed.is_finite() || profile.feed <= 0.0 {
        issues.push("进给速度必须是有限正数".to_string());
    }
    if !profile.rotation_safety_clearance.is_finite() || profile.rotation_safety_clearance < 0.0 {
        issues.push("旋转安全抬刀距离非法".to_string());
    }
    if !profile.same_zone_clearance.is_finite() || profile.same_zone_clearance < 0.0 {
        issues.push("同区离面距离非法".to_string());
    }
    if !profile.overcut.is_finite() || profile.overcut < 0.0 {
        issues.push("过切距离非法".to_string());
    }
    if !profile.arc_tolerance.is_finite() || profile.arc_tolerance <= 0.0 {
        issues.push("圆弧折线公差必须是有限正数".to_string());
    }
    if profile.decimals > 9 {
        issues.push("小数位不能超过 9".to_string());
    }
    if trajectory.unit_ranges.is_empty() || trajectory.moves.is_empty() {
        issues.push("没有可导出的加工单元或运动段".to_string());
        return issues;
    }
    if (profile.use_initial_position || profile.return_to_initial)
        && (!profile.initial_x.is_finite()
            || !profile.initial_y.is_finite()
            || !profile.initial_z.is_finite()
            || !profile.initial_a.is_finite())
    {
        issues.push("初始机床位置包含非法数值".to_string());
    }

    for range in &trajectory.unit_ranges {
        let start = range.start.min(trajectory.moves.len());
        let end = range.end.min(trajectory.moves.len());
        if start >= end {
            issues.push(format!("加工单元 {} 的运动范围为空", range.order));
            continue;
        }
        let moves = &trajectory.moves[start..end];
        if !moves.iter().any(|movement| movement.kind.is_cutting()) {
            issues.push(format!("加工单元 {} 没有切削移动", range.order));
        }
        if moves.iter().any(|movement| {
            !movement.pose.x.is_finite()
                || !movement.pose.y.is_finite()
                || !movement.pose.z.is_finite()
                || !movement.pose.a.is_finite()
        }) {
            issues.push(format!("加工单元 {} 包含非法坐标", range.order));
        }
        for pair in moves.windows(2) {
            if pair[0].kind == MachineMoveKind::Rapid
                && pair[1].kind == MachineMoveKind::Rapid
                && (pair[0].pose.a - pair[1].pose.a).abs() > 1.0e-6
                && (pair[0].pose.z < trajectory.safe_z - 1.0e-6
                    || pair[1].pose.z < trajectory.safe_z - 1.0e-6)
            {
                issues.push(format!("加工单元 {} 在非安全高度改变 A 轴", range.order));
            }
        }
    }
    issues
}

fn push_block(lines: &mut Vec<String>, block: &str) {
    for line in block.lines() {
        let line = line.trim();
        if !line.is_empty() {
            lines.push(line.to_string());
        }
    }
}

fn push_rule_headers(lines: &mut Vec<String>, profile: &GCodeProfile, keys: &[UnitRuleKeys]) {
    for key in keys {
        let rules = [
            profile.entity_type_rules.get(&key.entity_type),
            profile.layer_rules.get(&key.layer),
            profile.color_rules.get(&key.color),
        ];
        for rule in rules.into_iter().flatten() {
            push_block(lines, &rule.header);
        }
    }
}

fn push_rule_footers(lines: &mut Vec<String>, profile: &GCodeProfile, keys: &[UnitRuleKeys]) {
    for key in keys.iter().rev() {
        let rules = [
            profile.color_rules.get(&key.color),
            profile.layer_rules.get(&key.layer),
            profile.entity_type_rules.get(&key.entity_type),
        ];
        for rule in rules.into_iter().flatten() {
            push_block(lines, &rule.footer);
        }
    }
}

fn push_movement(
    lines: &mut Vec<String>,
    movement: &MachineMove,
    profile: &GCodeProfile,
    previous_pose: &mut Option<MachinePose>,
) {
    match movement.kind {
        MachineMoveKind::Rapid => lines.push(rapid_line(movement, profile)),
        MachineMoveKind::Cutting => lines.push(cutting_line(movement, profile)),
        MachineMoveKind::Arc {
            clockwise,
            plane,
            center,
        } => {
            let Some(start) = *previous_pose else {
                lines.push(cutting_line(movement, profile));
                *previous_pose = Some(movement.pose);
                return;
            };
            if !profile.arc_output {
                push_arc_polyline(lines, movement, start, center, clockwise, plane, profile);
                *previous_pose = Some(movement.pose);
                return;
            }
            lines.push(
                match plane {
                    ArcPlane::XY => "G17",
                    ArcPlane::XZ => "G18",
                    ArcPlane::YZ => "G19",
                }
                .to_string(),
            );
            lines.push(arc_line(movement, start, center, clockwise, plane, profile));
        }
    }
    *previous_pose = Some(movement.pose);
}

fn push_arc_polyline(
    lines: &mut Vec<String>,
    movement: &MachineMove,
    start: MachinePose,
    center: [f64; 3],
    clockwise: bool,
    plane: ArcPlane,
    profile: &GCodeProfile,
) {
    let end = movement.pose;
    let (start_u, start_v, end_u, end_v, radius, fixed) = match plane {
        ArcPlane::XY => (
            start.x - center[0],
            start.y - center[1],
            end.x - center[0],
            end.y - center[1],
            (start.x - center[0]).hypot(start.y - center[1]),
            [start.z, 0.0, 0.0],
        ),
        ArcPlane::XZ => (
            start.x - center[0],
            start.z - center[2],
            end.x - center[0],
            end.z - center[2],
            (start.x - center[0]).hypot(start.z - center[2]),
            [start.y, 1.0, 0.0],
        ),
        ArcPlane::YZ => (
            start.y - center[1],
            start.z - center[2],
            end.y - center[1],
            end.z - center[2],
            (start.y - center[1]).hypot(start.z - center[2]),
            [start.x, 2.0, 0.0],
        ),
    };
    if !radius.is_finite() || radius <= 1.0e-9 {
        lines.push(cutting_line(movement, profile));
        return;
    }
    let start_angle = start_v.atan2(start_u);
    let end_angle = end_v.atan2(end_u);
    let mut delta = end_angle - start_angle;
    if clockwise {
        while delta >= 0.0 {
            delta -= std::f64::consts::TAU;
        }
    } else {
        while delta <= 0.0 {
            delta += std::f64::consts::TAU;
        }
    }
    if delta.abs() <= 1.0e-9 {
        lines.push(cutting_line(movement, profile));
        return;
    }
    let tolerance = profile.arc_tolerance.max(1.0e-6);
    let max_step = if radius > tolerance {
        2.0 * (1.0 - tolerance / radius).clamp(-1.0, 1.0).acos()
    } else {
        std::f64::consts::FRAC_PI_2
    };
    let steps = (delta.abs() / max_step.max(1.0e-4)).ceil().max(1.0) as usize;
    for step in 1..steps {
        let angle = start_angle + delta * step as f64 / steps as f64;
        let (x, y, z) = match plane {
            ArcPlane::XY => (
                center[0] + radius * angle.cos(),
                center[1] + radius * angle.sin(),
                fixed[0],
            ),
            ArcPlane::XZ => (
                center[0] + radius * angle.cos(),
                fixed[0],
                center[2] + radius * angle.sin(),
            ),
            ArcPlane::YZ => (
                fixed[0],
                center[1] + radius * angle.cos(),
                center[2] + radius * angle.sin(),
            ),
        };
        let point = MachineMove {
            kind: MachineMoveKind::Cutting,
            pose: MachinePose { x, y, z, a: end.a },
        };
        lines.push(cutting_line(&point, profile));
    }
    lines.push(cutting_line(movement, profile));
}

fn rapid_line(movement: &MachineMove, profile: &GCodeProfile) -> String {
    let axis = format_axis(movement.pose, profile.decimals);
    format!("{} {}", profile.rapid, axis)
}

fn cutting_line(movement: &MachineMove, profile: &GCodeProfile) -> String {
    let axis = format_axis(movement.pose, profile.decimals);
    format!(
        "{} {} F{}",
        profile.cutting,
        axis,
        format_number(profile.feed, profile.decimals)
    )
}

fn arc_line(
    movement: &MachineMove,
    start: MachinePose,
    center: [f64; 3],
    clockwise: bool,
    plane: ArcPlane,
    profile: &GCodeProfile,
) -> String {
    let code = if clockwise { "G02" } else { "G03" };
    let i = format_number(center[0] - start.x, profile.decimals);
    let j = format_number(center[1] - start.y, profile.decimals);
    let k = format_number(center[2] - start.z, profile.decimals);
    let offsets = match plane {
        ArcPlane::XY => format!("I{i} J{j}"),
        ArcPlane::XZ => format!("I{i} K{k}"),
        ArcPlane::YZ => format!("J{j} K{k}"),
    };
    format!(
        "{} {} {} F{}",
        code,
        format_axis_xyz(movement.pose, profile.decimals),
        offsets,
        format_number(profile.feed, profile.decimals)
    )
}

fn format_axis_xyz(pose: MachinePose, decimals: usize) -> String {
    format!(
        "X{} Y{} Z{}",
        format_number(pose.x, decimals),
        format_number(pose.y, decimals),
        format_number(pose.z, decimals)
    )
}

fn format_axis(pose: MachinePose, decimals: usize) -> String {
    format!(
        "X{} Y{} Z{} A{}",
        format_number(pose.x, decimals),
        format_number(pose.y, decimals),
        format_number(pose.z, decimals),
        format_number(pose.a, decimals)
    )
}

fn format_number(value: f64, decimals: usize) -> String {
    let decimals = decimals.min(9);
    if decimals == 0 {
        let formatted = format!("{value:.0}");
        return if formatted == "-0" {
            "0".to_string()
        } else {
            formatted
        };
    }
    let formatted = format!("{value:.decimals$}");
    let trimmed = formatted.trim_end_matches('0').trim_end_matches('.');
    if trimmed.is_empty() || trimmed == "-" {
        "0".to_string()
    } else {
        trimmed.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cam_trajectory::TrajectoryUnitRange;

    #[test]
    fn zero_decimal_format_keeps_integer_zeros() {
        assert_eq!(format_number(1000.0, 0), "1000");
        assert_eq!(format_number(0.0, 0), "0");
        assert_eq!(format_number(-0.0, 0), "0");
        assert_eq!(format_number(1000.0, 2), "1000");
    }

    #[test]
    fn disabled_arc_output_uses_polyline() {
        let trajectory = MachineTrajectory {
            moves: vec![
                MachineMove {
                    kind: MachineMoveKind::Rapid,
                    pose: MachinePose {
                        x: 10.0,
                        y: 0.0,
                        z: 0.0,
                        a: 0.0,
                    },
                },
                MachineMove {
                    kind: MachineMoveKind::Arc {
                        clockwise: false,
                        plane: ArcPlane::XY,
                        center: [0.0, 0.0, 0.0],
                    },
                    pose: MachinePose {
                        x: 0.0,
                        y: 10.0,
                        z: 0.0,
                        a: 0.0,
                    },
                },
            ],
            unit_ranges: vec![TrajectoryUnitRange {
                order: 1,
                start: 0,
                end: 2,
            }],
            unit_rules: vec![Vec::new()],
            cutting_move_count: 1,
            safe_z: 10.0,
            unit_count: 1,
            a_min: 0.0,
            a_max: 0.0,
        };
        let mut profile = GCodeProfile::default();
        profile.arc_output = false;
        profile.arc_tolerance = 0.05;
        let text = render_gcode(&trajectory, &profile);
        assert!(!text.contains("G02"));
        assert!(!text.contains("G03"));
        assert!(text.lines().filter(|line| line.starts_with("G01")).count() > 1);
    }

    #[test]
    fn invalid_profile_file_is_not_overwritten() {
        let path = std::env::temp_dir().join("cadcam-invalid-gcode.toml");
        fs::write(&path, "not = [valid").unwrap();
        let before = fs::read_to_string(&path).unwrap();
        let _ = GCodeProfile::load_or_create(&path);
        let after = fs::read_to_string(&path).unwrap();
        assert_eq!(before, after);
        let _ = fs::remove_file(path);
    }
}
