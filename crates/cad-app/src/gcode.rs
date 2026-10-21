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
            let profile = Self::load(path).unwrap_or_default();
            // Rewrite once so newly added default fields are visible in TOML.
            let _ = profile.save(path);
            profile
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
    if trajectory.unit_ranges.is_empty() || trajectory.moves.is_empty() {
        issues.push("没有可导出的加工单元或运动段".to_string());
        return issues;
    }
    if profile.use_initial_position
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
            if !profile.arc_output {
                lines.push(cutting_line(movement, profile));
                *previous_pose = Some(movement.pose);
                return;
            }
            let Some(start) = *previous_pose else {
                lines.push(cutting_line(movement, profile));
                *previous_pose = Some(movement.pose);
                return;
            };
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
    let formatted = format!("{value:.decimals$}");
    formatted
        .trim_end_matches('0')
        .trim_end_matches('.')
        .to_string()
}
