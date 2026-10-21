//! TOML-driven G-code profile and text rendering.
//!
//! The renderer consumes the semantic machine trajectory. It never changes the
//! machining order, directions or starts.

use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::cam_trajectory::{MachineMoveKind, MachineTrajectory};

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
}

impl Default for GCodeProfile {
    fn default() -> Self {
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
        }
    }
}

impl GCodeProfile {
    pub fn rotary_config(&self) -> crate::cam_trajectory::RotaryConfig {
        crate::cam_trajectory::RotaryConfig {
            a_offset_degrees: self.a_offset_degrees,
            machining_z_offset: self.machining_z_offset,
            rotation_safety_clearance: self.rotation_safety_clearance,
            overcut_distance: self.overcut,
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

    for range in &trajectory.unit_ranges {
        let start = range.start.min(trajectory.moves.len());
        let end = range.end.min(trajectory.moves.len());
        if start >= end {
            continue;
        }
        let moves = &trajectory.moves[start..end];
        let first_cut = moves
            .iter()
            .position(|movement| movement.kind == MachineMoveKind::Cutting);
        let last_cut = moves
            .iter()
            .rposition(|movement| movement.kind == MachineMoveKind::Cutting);

        for movement in moves.iter().take(first_cut.unwrap_or(moves.len())) {
            lines.push(rapid_line(movement, profile));
        }

        if let (Some(first_cut), Some(last_cut)) = (first_cut, last_cut) {
            push_block(&mut lines, &profile.unit_start);
            for movement in &moves[first_cut..=last_cut] {
                lines.push(cutting_line(movement, profile));
            }
            push_block(&mut lines, &profile.unit_end);
            for movement in moves.iter().skip(last_cut + 1) {
                lines.push(rapid_line(movement, profile));
            }
        }
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

fn push_block(lines: &mut Vec<String>, block: &str) {
    for line in block.lines() {
        let line = line.trim();
        if !line.is_empty() {
            lines.push(line.to_string());
        }
    }
}

fn rapid_line(movement: &crate::cam_trajectory::MachineMove, profile: &GCodeProfile) -> String {
    let axis = format_axis(movement.pose, profile.decimals);
    format!("{} {}", profile.rapid, axis)
}

fn cutting_line(movement: &crate::cam_trajectory::MachineMove, profile: &GCodeProfile) -> String {
    let axis = format_axis(movement.pose, profile.decimals);
    format!(
        "{} {} F{}",
        profile.cutting,
        axis,
        format_number(profile.feed, profile.decimals)
    )
}

fn format_axis(pose: crate::cam_trajectory::MachinePose, decimals: usize) -> String {
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
