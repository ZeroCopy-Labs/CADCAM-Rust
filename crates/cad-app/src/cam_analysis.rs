//! First CAM analysis layer: tube sections, machining-section candidates,
//! machining intervals and the 16 circumferential tube zones.
//!
//! This module is intentionally independent from Bevy/egui. It consumes the
//! display paths and machining units already produced by `display.rs` / `cam.rs`.

use std::collections::{BTreeMap, BTreeSet};

use crate::cam::{CamModel, UnitId};
use crate::display::DisplayData;

pub const SURFACE_TOLERANCE: f64 = 0.25;
pub const JOIN_TOLERANCE: f64 = 1.0e-3;
const BOUNDARY_BINS: usize = 512;
const ENTRY_ANGLE_TOLERANCE: f64 = 0.5_f64.to_radians();

/// The 16 circumferential zones of a square/rounded tube.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum TubeZone {
    TopFace,
    TopToTopRightBoundary,
    TopRightCorner,
    TopRightToRightBoundary,
    RightFace,
    RightToBottomRightBoundary,
    BottomRightCorner,
    BottomRightToBottomBoundary,
    BottomFace,
    BottomToBottomLeftBoundary,
    BottomLeftCorner,
    BottomLeftToLeftBoundary,
    LeftFace,
    LeftToTopLeftBoundary,
    TopLeftCorner,
    TopLeftToTopBoundary,
}

impl TubeZone {
    pub const ALL: [TubeZone; 16] = [
        TubeZone::TopFace,
        TubeZone::TopToTopRightBoundary,
        TubeZone::TopRightCorner,
        TubeZone::TopRightToRightBoundary,
        TubeZone::RightFace,
        TubeZone::RightToBottomRightBoundary,
        TubeZone::BottomRightCorner,
        TubeZone::BottomRightToBottomBoundary,
        TubeZone::BottomFace,
        TubeZone::BottomToBottomLeftBoundary,
        TubeZone::BottomLeftCorner,
        TubeZone::BottomLeftToLeftBoundary,
        TubeZone::LeftFace,
        TubeZone::LeftToTopLeftBoundary,
        TubeZone::TopLeftCorner,
        TubeZone::TopLeftToTopBoundary,
    ];

    pub fn label(self) -> &'static str {
        match self {
            TubeZone::TopFace => "上表面",
            TubeZone::TopToTopRightBoundary => "上→左上过渡",
            TubeZone::TopRightCorner => "左上圆角",
            TubeZone::TopRightToRightBoundary => "左上→左过渡",
            TubeZone::RightFace => "左表面",
            TubeZone::RightToBottomRightBoundary => "左→左下过渡",
            TubeZone::BottomRightCorner => "左下圆角",
            TubeZone::BottomRightToBottomBoundary => "左下→下过渡",
            TubeZone::BottomFace => "下表面",
            TubeZone::BottomToBottomLeftBoundary => "下→右下过渡",
            TubeZone::BottomLeftCorner => "右下圆角",
            TubeZone::BottomLeftToLeftBoundary => "右下→右过渡",
            TubeZone::LeftFace => "右表面",
            TubeZone::LeftToTopLeftBoundary => "右→右上过渡",
            TubeZone::TopLeftCorner => "右上圆角",
            TubeZone::TopLeftToTopBoundary => "右上→上过渡",
        }
    }

    pub fn index(self) -> usize {
        self as usize
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SectionSource {
    Auto,
    Manual,
}

/// A candidate automatically fitted from the drawing's machining units.
#[derive(Clone, Debug)]
pub struct TubeCandidate {
    pub center_y: f64,
    pub center_z: f64,
    pub size_y: f64,
    pub size_z: f64,
    pub radius: f64,
    pub score: f64,
}

impl TubeCandidate {
    pub fn to_section(&self, source: SectionSource) -> TubeSection {
        TubeSection {
            center_y: self.center_y,
            center_z: self.center_z,
            size_y: self.size_y,
            size_z: self.size_z,
            radius: self.radius,
            source: source,
        }
    }
}

/// The tube cross-section used as the reference for all CAM analysis.
#[derive(Clone, Debug)]
pub struct TubeSection {
    pub center_y: f64,
    pub center_z: f64,
    pub size_y: f64,
    pub size_z: f64,
    pub radius: f64,
    pub source: SectionSource,
}

impl Default for TubeSection {
    fn default() -> Self {
        Self {
            center_y: 0.0,
            center_z: 0.0,
            size_y: 38.0,
            size_z: 25.0,
            radius: 2.0,
            source: SectionSource::Manual,
        }
    }
}

impl TubeSection {
    pub fn half_y(&self) -> f64 {
        self.size_y.max(1.0e-9) * 0.5
    }

    pub fn half_z(&self) -> f64 {
        self.size_z.max(1.0e-9) * 0.5
    }

    pub fn corner_radius(&self) -> f64 {
        self.radius.clamp(0.0, self.half_y().min(self.half_z()))
    }

    pub fn zone_for_point(&self, point: [f64; 3]) -> Option<TubeZone> {
        self.boundary()
            .nearest(point[1], point[2])
            .map(|(_, _, zone)| zone)
    }

    fn boundary(&self) -> TubeBoundary {
        TubeBoundary::new(self)
    }
}

/// One point on the tube perimeter, already shifted to section-local YZ.
#[derive(Clone, Copy, Debug)]
struct BoundarySample {
    y: f64,
    z: f64,
    perimeter: f64,
    zone: TubeZone,
}

#[derive(Clone, Debug)]
struct TubeBoundary {
    samples: Vec<BoundarySample>,
    perimeter: f64,
}

impl TubeBoundary {
    fn new(section: &TubeSection) -> Self {
        let half_y = section.half_y();
        let half_z = section.half_z();
        let radius = section.corner_radius();
        let mut samples = Vec::new();
        let mut perimeter = 0.0;
        let face_steps = 24usize;
        let corner_steps = 20usize;
        macro_rules! push {
            ($y:expr, $z:expr, $zone:expr $(,)?) => {
                samples.push(BoundarySample {
                    y: $y,
                    z: $z,
                    perimeter,
                    zone: $zone,
                })
            };
        }

        // Right face, bottom to top.
        for index in 0..=face_steps {
            let t = index as f64 / face_steps as f64;
            let z = -half_z + radius + (2.0 * (half_z - radius)) * t;
            push!(half_y, z, TubeZone::RightFace);
            if index < face_steps {
                perimeter += 2.0 * (half_z - radius) / face_steps as f64;
            }
        }
        push!(half_y, half_z - radius, TubeZone::TopRightToRightBoundary);
        perimeter += std::f64::consts::FRAC_PI_2 * radius / corner_steps as f64;
        // Top-right corner.
        let corner_center_y = half_y - radius;
        let corner_center_z = half_z - radius;
        for index in 1..=corner_steps {
            let angle = std::f64::consts::FRAC_PI_2 * index as f64 / corner_steps as f64;
            let (sin, cos) = angle.sin_cos();
            push!(
                corner_center_y + radius * cos,
                corner_center_z + radius * sin,
                TubeZone::TopRightCorner,
            );
            if index < corner_steps {
                perimeter += std::f64::consts::FRAC_PI_2 * radius / corner_steps as f64;
            }
        }
        push!(half_y - radius, half_z, TubeZone::TopToTopRightBoundary);
        perimeter += 2.0 * (half_y - radius) / face_steps as f64;
        // Top face, right to left.
        for index in 1..=face_steps {
            let t = index as f64 / face_steps as f64;
            let y = half_y - radius - 2.0 * (half_y - radius) * t;
            push!(y, half_z, TubeZone::TopFace);
            if index < face_steps {
                perimeter += 2.0 * (half_y - radius) / face_steps as f64;
            }
        }
        push!(-half_y + radius, half_z, TubeZone::TopLeftToTopBoundary);
        perimeter += std::f64::consts::FRAC_PI_2 * radius / corner_steps as f64;
        // Top-left corner.
        let corner_center_y = -half_y + radius;
        for index in 1..=corner_steps {
            let angle = std::f64::consts::FRAC_PI_2
                + std::f64::consts::FRAC_PI_2 * index as f64 / corner_steps as f64;
            let (sin, cos) = angle.sin_cos();
            push!(
                corner_center_y + radius * cos,
                corner_center_z + radius * sin,
                TubeZone::TopLeftCorner,
            );
            if index < corner_steps {
                perimeter += std::f64::consts::FRAC_PI_2 * radius / corner_steps as f64;
            }
        }
        push!(-half_y, half_z - radius, TubeZone::LeftToTopLeftBoundary);
        perimeter += 2.0 * (half_z - radius) / face_steps as f64;
        // Left face, top to bottom.
        for index in 1..=face_steps {
            let t = index as f64 / face_steps as f64;
            let z = half_z - radius - 2.0 * (half_z - radius) * t;
            push!(-half_y, z, TubeZone::LeftFace);
            if index < face_steps {
                perimeter += 2.0 * (half_z - radius) / face_steps as f64;
            }
        }
        push!(
            -half_y,
            -half_z + radius,
            TubeZone::BottomLeftToLeftBoundary,
        );
        perimeter += std::f64::consts::FRAC_PI_2 * radius / corner_steps as f64;
        // Bottom-left corner.
        let corner_center_z = -half_z + radius;
        for index in 1..=corner_steps {
            let angle = std::f64::consts::PI
                + std::f64::consts::FRAC_PI_2 * index as f64 / corner_steps as f64;
            let (sin, cos) = angle.sin_cos();
            push!(
                corner_center_y + radius * cos,
                corner_center_z + radius * sin,
                TubeZone::BottomLeftCorner,
            );
            if index < corner_steps {
                perimeter += std::f64::consts::FRAC_PI_2 * radius / corner_steps as f64;
            }
        }
        push!(
            -half_y + radius,
            -half_z,
            TubeZone::BottomToBottomLeftBoundary,
        );
        perimeter += 2.0 * (half_y - radius) / face_steps as f64;
        // Bottom face, left to right.
        for index in 1..=face_steps {
            let t = index as f64 / face_steps as f64;
            let y = -half_y + radius + 2.0 * (half_y - radius) * t;
            push!(y, -half_z, TubeZone::BottomFace);
            if index < face_steps {
                perimeter += 2.0 * (half_y - radius) / face_steps as f64;
            }
        }
        push!(
            half_y - radius,
            -half_z,
            TubeZone::BottomRightToBottomBoundary,
        );
        perimeter += std::f64::consts::FRAC_PI_2 * radius / corner_steps as f64;
        let corner_center_y = half_y - radius;
        for index in 1..=corner_steps {
            let angle = 3.0 * std::f64::consts::FRAC_PI_2
                + std::f64::consts::FRAC_PI_2 * index as f64 / corner_steps as f64;
            let (sin, cos) = angle.sin_cos();
            push!(
                corner_center_y + radius * cos,
                corner_center_z + radius * sin,
                TubeZone::BottomRightCorner,
            );
            if index < corner_steps {
                perimeter += std::f64::consts::FRAC_PI_2 * radius / corner_steps as f64;
            }
        }
        push!(
            half_y,
            -half_z + radius,
            TubeZone::RightToBottomRightBoundary,
        );
        perimeter += 0.0;

        // Add a final closing position at the same point as the first sample.
        if let Some(first) = samples.first().copied() {
            samples.push(BoundarySample {
                y: first.y,
                z: first.z,
                perimeter,
                zone: TubeZone::RightFace,
            });
        }

        Self { samples, perimeter }
    }

    fn nearest(&self, y: f64, z: f64) -> Option<(f64, f64, TubeZone)> {
        let mut best: Option<(f64, f64, TubeZone)> = None;
        for index in 0..self.samples.len().saturating_sub(1) {
            let start = self.samples[index];
            let end = self.samples[index + 1];
            let dy = end.y - start.y;
            let dz = end.z - start.z;
            let length_squared = dy * dy + dz * dz;
            if length_squared <= 1.0e-18 {
                continue;
            }
            let t = (((y - start.y) * dy + (z - start.z) * dz) / length_squared).clamp(0.0, 1.0);
            let py = start.y + dy * t;
            let pz = start.z + dz * t;
            let distance = (y - py).hypot(z - pz);
            if best
                .map(|(_, best_distance, _)| distance < best_distance)
                .unwrap_or(true)
            {
                let perimeter = start.perimeter + (end.perimeter - start.perimeter) * t;
                best = Some((perimeter, distance, start.zone));
            }
        }
        best
    }
}

/// Result of comparing a candidate loop with the tube perimeter.
#[derive(Clone, Debug)]
pub struct MachiningSectionCandidate {
    pub unit_id: UnitId,
    pub x_min: f64,
    pub x_max: f64,
    pub winding: i32,
    pub maximum_deviation: f64,
    pub coverage_gap_bins: usize,
    pub zone_mask: [bool; 16],
}

#[derive(Clone, Debug)]
pub struct MachiningSection {
    pub unit_id: UnitId,
    pub boundary_units: Vec<UnitId>,
    pub x_min: f64,
    pub x_max: f64,
    pub winding: i32,
    pub zone_mask: [bool; 16],
}

#[derive(Clone, Debug, Default)]
pub struct MachiningInterval {
    pub id: u32,
    pub x_start: f64,
    pub x_end: f64,
    pub units: Vec<UnitId>,
}

/// One 16-zone bucket inside a section-delimited machining group.
#[derive(Clone, Debug)]
pub struct ZoneUnitGroup {
    pub zone: TubeZone,
    pub units: Vec<UnitId>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EndCutSide {
    #[default]
    Section,
    Left,
    Right,
}

impl EndCutSide {
    pub fn label(self) -> &'static str {
        match self {
            Self::Section => "未配对/修边断面",
            Self::Left => "左端面（+X 修正）",
            Self::Right => "右端面（−X 修正）",
        }
    }
}

/// A finished part has two boundaries. Units outside paired parts stay visible
/// in a separate group rather than being silently attached to a cut.
#[derive(Clone, Debug)]
pub struct MachiningGroup {
    pub id: u32,
    pub x_start: f64,
    pub x_end: f64,
    pub left_section: Option<UnitId>,
    pub right_section: Option<UnitId>,
    pub units: Vec<UnitId>,
    pub zone_groups: Vec<ZoneUnitGroup>,
}

/// One unit in the deterministic machining order preview.
#[derive(Clone, Debug)]
pub struct PlanUnit {
    pub unit_id: UnitId,
    pub zone: TubeZone,
    pub order: u32,
    pub machining_cut: bool,
    pub direction: UnitDirection,
    pub start_point: [f64; 3],
    pub end_point: [f64; 3],
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum UnitDirection {
    #[default]
    Forward,
    Reverse,
}

/// Ordered units for one cut-delimited machining group.
#[derive(Clone, Debug)]
pub struct PlanGroup {
    pub group_id: u32,
    pub x_start: f64,
    pub x_end: f64,
    pub units: Vec<PlanUnit>,
}

/// Deterministic machining order with derived directions and entry points.
#[derive(Clone, Debug, Default)]
pub struct CamPlan {
    pub groups: Vec<PlanGroup>,
}

#[derive(Clone, Debug)]
pub struct UnitZoneProfile {
    pub zone_lengths: [f64; 16],
    pub touched_zones: [bool; 16],
    pub owner_zone: Option<TubeZone>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RotationDirection {
    #[default]
    CounterClockwise,
    Clockwise,
}

#[derive(Clone, Debug)]
pub struct CamAnalysis {
    pub tube_candidates: Vec<TubeCandidate>,
    pub tube_section: Option<TubeSection>,
    pub machining_candidates: Vec<MachiningSectionCandidate>,
    pub machining_sections: Vec<MachiningSection>,
    pub intervals: Vec<MachiningInterval>,
    pub groups: Vec<MachiningGroup>,
    pub plan: CamPlan,
    pub zone_profiles: BTreeMap<UnitId, UnitZoneProfile>,
    pub lazy_rotation: bool,
    pub rotation_direction: RotationDirection,
    pub machine_start: [f64; 3],
    pub section_pairs: Vec<(UnitId, UnitId)>,
    pub trim_sections: BTreeSet<UnitId>,
    pub grouping_errors: Vec<String>,
}

impl Default for CamAnalysis {
    fn default() -> Self {
        Self {
            tube_candidates: Vec::new(),
            tube_section: None,
            machining_candidates: Vec::new(),
            machining_sections: Vec::new(),
            intervals: Vec::new(),
            groups: Vec::new(),
            plan: CamPlan::default(),
            zone_profiles: BTreeMap::new(),
            lazy_rotation: true,
            rotation_direction: RotationDirection::CounterClockwise,
            machine_start: [0.0; 3],
            section_pairs: Vec::new(),
            trim_sections: BTreeSet::new(),
            grouping_errors: Vec::new(),
        }
    }
}

impl CamAnalysis {
    pub fn end_cut_side(&self, unit: UnitId) -> EndCutSide {
        for group in &self.groups {
            if group.left_section.is_some() && group.right_section.is_some() {
                if group.left_section == Some(unit) {
                    return EndCutSide::Left;
                }
                if group.right_section == Some(unit) {
                    return EndCutSide::Right;
                }
            }
        }
        EndCutSide::Section
    }

    pub fn pair_sections(
        &mut self,
        left: UnitId,
        right: UnitId,
        display: &DisplayData,
        cam: &CamModel,
    ) -> Result<(), String> {
        if left == right
            || ![left, right]
                .iter()
                .all(|id| self.machining_sections.iter().any(|s| s.unit_id == *id))
        {
            return Err("请选择两个不同的加工断面".to_string());
        }
        let l = ordered_unit_points(display, cam, left).ok_or("左断面路径不完整")?;
        let r = ordered_unit_points(display, cam, right).ok_or("右断面路径不完整")?;
        check_boundary_pair(&l, &r)?;
        self.section_pairs
            .retain(|(a, b)| ![left, right].contains(a) && ![left, right].contains(b));
        self.section_pairs.push((left, right));
        self.trim_sections.remove(&left);
        self.trim_sections.remove(&right);
        self.rebuild_groups(display, cam);
        self.rebuild_intervals();
        self.rebuild_plan(display, cam);
        Ok(())
    }

    pub fn reset_section_pairs(&mut self, display: &DisplayData, cam: &CamModel) {
        self.section_pairs.clear();
        self.rebuild_groups(display, cam);
        self.rebuild_intervals();
        self.rebuild_plan(display, cam);
    }

    pub fn set_trim_section(
        &mut self,
        unit: UnitId,
        trim: bool,
        display: &DisplayData,
        cam: &CamModel,
    ) {
        if trim {
            self.trim_sections.insert(unit);
        } else {
            self.trim_sections.remove(&unit);
        }
        self.section_pairs.retain(|(l, r)| *l != unit && *r != unit);
        self.rebuild_groups(display, cam);
        self.rebuild_intervals();
        self.rebuild_plan(display, cam);
    }

    pub fn set_machine_start(&mut self, display: &DisplayData, cam: &CamModel, start: [f64; 3]) {
        if self.machine_start != start {
            self.machine_start = start;
            self.rebuild_plan(display, cam);
        }
    }
    pub fn build(display: &DisplayData, cam: &CamModel) -> Self {
        let mut analysis = Self::default();
        analysis.tube_candidates = detect_tube_candidates(display, cam);
        analysis.try_auto_tube_section(display, cam);
        if analysis.tube_section.is_none() {
            analysis.rebuild_without_tube_section(display, cam);
        }
        analysis
    }

    pub fn apply_tube_candidate(&mut self, index: usize, display: &DisplayData, cam: &CamModel) {
        let Some(candidate) = self.tube_candidates.get(index).cloned() else {
            return;
        };
        self.tube_section = Some(candidate.to_section(SectionSource::Auto));
        self.rebuild_after_tube(display, cam);
    }

    pub fn apply_manual_tube(
        &mut self,
        mut section: TubeSection,
        display: &DisplayData,
        cam: &CamModel,
    ) {
        section.source = SectionSource::Manual;
        self.tube_section = Some(section);
        self.rebuild_after_tube(display, cam);
    }

    pub fn apply_machining_candidate(
        &mut self,
        index: usize,
        display: &DisplayData,
        cam: &CamModel,
    ) {
        let Some(candidate) = self.machining_candidates.get(index).cloned() else {
            return;
        };
        if self
            .machining_sections
            .iter()
            .any(|section| section.unit_id == candidate.unit_id)
        {
            return;
        }
        self.machining_sections.push(MachiningSection {
            unit_id: candidate.unit_id,
            boundary_units: vec![candidate.unit_id],
            x_min: candidate.x_min,
            x_max: candidate.x_max,
            winding: candidate.winding,
            zone_mask: candidate.zone_mask,
        });
        self.machining_sections.sort_by(|left, right| {
            left.x_min
                .partial_cmp(&right.x_min)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        self.rebuild_groups(display, cam);
        self.rebuild_intervals();
        self.rebuild_plan(display, cam);
    }

    /// Mark a closed unit as a machining cut manually.
    pub fn apply_manual_machining_section(
        &mut self,
        unit_id: UnitId,
        display: &DisplayData,
        cam: &CamModel,
    ) -> bool {
        if self.tube_section.is_none()
            || self
                .machining_sections
                .iter()
                .any(|section| section.unit_id == unit_id)
        {
            return false;
        }
        let Some(unit) = cam.unit(unit_id) else {
            return false;
        };
        if !unit.closed {
            return false;
        }
        let Some((x_min, x_max)) = unit_x_bounds(display, cam, unit_id) else {
            return false;
        };
        let zone_mask = self
            .zone_profiles
            .get(&unit_id)
            .map(|profile| profile.touched_zones)
            .unwrap_or([false; 16]);
        self.machining_sections.push(MachiningSection {
            unit_id,
            boundary_units: vec![unit_id],
            x_min,
            x_max,
            winding: 1,
            zone_mask,
        });
        self.machining_sections.sort_by(|left, right| {
            left.x_min
                .partial_cmp(&right.x_min)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        self.rebuild_groups(display, cam);
        self.rebuild_intervals();
        self.rebuild_plan(display, cam);
        true
    }

    /// Remove a machining section assignment.
    pub fn remove_machining_section(
        &mut self,
        unit_id: UnitId,
        display: &DisplayData,
        cam: &CamModel,
    ) -> bool {
        let before = self.machining_sections.len();
        self.machining_sections
            .retain(|section| section.unit_id != unit_id);
        if self.machining_sections.len() == before {
            return false;
        }
        self.rebuild_groups(display, cam);
        self.rebuild_intervals();
        self.rebuild_plan(display, cam);
        true
    }

    pub fn clear_tube_section(&mut self, display: &DisplayData, cam: &CamModel) {
        self.tube_section = None;
        self.rebuild_without_tube_section(display, cam);
    }

    /// Fallback planning when no tube section can be determined: process the
    /// machining units directly in X order without section/zone analysis.
    fn rebuild_without_tube_section(&mut self, display: &DisplayData, cam: &CamModel) {
        self.machining_candidates.clear();
        self.machining_sections.clear();
        self.section_pairs.clear();
        self.trim_sections.clear();
        self.grouping_errors.clear();
        self.zone_profiles.clear();
        self.intervals.clear();
        self.groups.clear();
        self.plan.groups.clear();

        let mut unit_ids: Vec<UnitId> = cam.units.iter().map(|unit| unit.id).collect();
        if unit_ids.is_empty() {
            return;
        }
        unit_ids.sort_by(|left, right| {
            let left_x = unit_x_bounds(display, cam, *left)
                .map(|(minimum, _)| minimum)
                .unwrap_or(f64::INFINITY);
            let right_x = unit_x_bounds(display, cam, *right)
                .map(|(minimum, _)| minimum)
                .unwrap_or(f64::INFINITY);
            left_x
                .partial_cmp(&right_x)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| left.cmp(right))
        });

        let mut planned = Vec::new();
        for (index, unit_id) in unit_ids.iter().enumerate() {
            planned.push(PlanUnit {
                unit_id: *unit_id,
                zone: TubeZone::TopFace,
                order: index as u32 + 1,
                machining_cut: false,
                direction: UnitDirection::Forward,
                start_point: [0.0; 3],
                end_point: [0.0; 3],
            });
        }
        let mut previous_end: Option<[f64; 3]> = None;
        for unit in &mut planned {
            if let Some((direction, start, end)) =
                choose_unit_direction(display, cam, unit.unit_id, previous_end)
            {
                unit.direction = direction;
                unit.start_point = start;
                unit.end_point = end;
                previous_end = Some(end);
            }
        }

        let x_start = unit_ids
            .iter()
            .filter_map(|unit_id| unit_x_bounds(display, cam, *unit_id))
            .map(|(minimum, _)| minimum)
            .fold(f64::INFINITY, f64::min);
        let x_end = unit_ids
            .iter()
            .filter_map(|unit_id| unit_x_bounds(display, cam, *unit_id))
            .map(|(_, maximum)| maximum)
            .fold(f64::NEG_INFINITY, f64::max);
        let x_start = if x_start.is_finite() { x_start } else { 0.0 };
        let x_end = if x_end.is_finite() { x_end } else { 0.0 };

        self.groups.push(MachiningGroup {
            id: 0,
            x_start,
            x_end,
            left_section: None,
            right_section: None,
            units: unit_ids.clone(),
            zone_groups: Vec::new(),
        });
        self.intervals.push(MachiningInterval {
            id: 0,
            x_start,
            x_end,
            units: unit_ids,
        });
        self.plan.groups.push(PlanGroup {
            group_id: 0,
            x_start,
            x_end,
            units: planned,
        });
    }

    fn try_auto_tube_section(&mut self, display: &DisplayData, cam: &CamModel) {
        let Some(best) = self.tube_candidates.first().cloned() else {
            return;
        };
        let equivalent = self.tube_candidates.iter().all(|candidate| {
            (candidate.center_y - best.center_y).abs() <= 0.1
                && (candidate.center_z - best.center_z).abs() <= 0.1
                && (candidate.size_y - best.size_y).abs() <= 0.1
                && (candidate.size_z - best.size_z).abs() <= 0.1
                && (candidate.radius - best.radius).abs() <= 0.1
        });
        if self.tube_candidates.len() == 1 || equivalent {
            self.tube_section = Some(best.to_section(SectionSource::Auto));
            self.rebuild_after_tube(display, cam);
        }
    }

    fn rebuild_after_tube(&mut self, display: &DisplayData, cam: &CamModel) {
        self.machining_candidates.clear();
        self.machining_sections.clear();
        self.intervals.clear();
        self.groups.clear();
        self.plan.groups.clear();
        self.zone_profiles.clear();
        let Some(section) = self.tube_section.clone() else {
            self.rebuild_without_tube_section(display, cam);
            return;
        };
        self.machining_candidates = detect_machining_candidates(display, cam, &section);
        // High-confidence section candidates are applied automatically. The
        // candidate already passed winding/coverage checks; here we only require
        // a tight surface deviation and no meaningful uncovered gap.
        for candidate in &self.machining_candidates {
            if candidate.maximum_deviation <= 0.05 && candidate.coverage_gap_bins <= 1 {
                self.machining_sections.push(MachiningSection {
                    unit_id: candidate.unit_id,
                    boundary_units: vec![candidate.unit_id],
                    x_min: candidate.x_min,
                    x_max: candidate.x_max,
                    winding: candidate.winding,
                    zone_mask: candidate.zone_mask,
                });
            }
        }
        self.machining_sections.sort_by(|left, right| {
            left.x_min
                .partial_cmp(&right.x_min)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        self.zone_profiles = classify_units(display, cam, &section);
        self.rebuild_groups(display, cam);
        self.rebuild_intervals();
        self.rebuild_plan(display, cam);
    }

    fn rebuild_intervals(&mut self) {
        self.intervals = self
            .groups
            .iter()
            .map(|group| MachiningInterval {
                id: group.id,
                x_start: group.x_start,
                x_end: group.x_end,
                units: group
                    .units
                    .iter()
                    .copied()
                    .filter(|id| !self.machining_sections.iter().any(|s| s.unit_id == *id))
                    .collect(),
            })
            .collect();
    }

    fn rebuild_groups(&mut self, display: &DisplayData, cam: &CamModel) {
        self.groups.clear();
        self.grouping_errors.clear();
        let ids: BTreeSet<_> = self.machining_sections.iter().map(|s| s.unit_id).collect();
        self.trim_sections.retain(|id| ids.contains(id));
        self.section_pairs.retain(|(l, r)| {
            ids.contains(l)
                && ids.contains(r)
                && !self.trim_sections.contains(l)
                && !self.trim_sections.contains(r)
        });
        let paths: BTreeMap<_, _> = ids
            .iter()
            .filter_map(|id| ordered_unit_points(display, cam, *id).map(|p| (*id, p)))
            .collect();
        for id in &ids {
            if !paths.contains_key(id) {
                self.grouping_errors
                    .push(format!("断面 #{:08X} 的路径不完整", id.0));
            }
        }
        let paired: BTreeSet<_> = self
            .section_pairs
            .iter()
            .flat_map(|(l, r)| [*l, *r])
            .collect();
        let mut automatic: Vec<_> = self
            .machining_sections
            .iter()
            .filter(|s| !paired.contains(&s.unit_id) && !self.trim_sections.contains(&s.unit_id))
            .map(|s| s.unit_id)
            .collect();
        automatic.sort_by(|l, r| {
            unit_x_center(display, cam, *l)
                .unwrap_or(0.0)
                .total_cmp(&unit_x_center(display, cam, *r).unwrap_or(0.0))
                .then_with(|| l.cmp(r))
        });
        let mut pairs = self.section_pairs.clone();
        pairs.extend(automatic.chunks_exact(2).map(|p| (p[0], p[1])));
        pairs.sort_by(|(l, _), (r, _)| {
            unit_x_center(display, cam, *l)
                .unwrap_or(0.0)
                .total_cmp(&unit_x_center(display, cam, *r).unwrap_or(0.0))
        });
        for (left, right) in pairs {
            let (Some(l), Some(r)) = (paths.get(&left), paths.get(&right)) else {
                continue;
            };
            if let Err(error) = check_boundary_pair(l, r) {
                self.grouping_errors.push(format!(
                    "断面 #{:08X}/#{:08X} 无法组成成品段：{error}",
                    left.0, right.0
                ));
            }
            self.groups.push(MachiningGroup {
                id: self.groups.len() as u32,
                x_start: l.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
                x_end: r.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max),
                left_section: Some(left),
                right_section: Some(right),
                units: vec![left, right],
                zone_groups: Vec::new(),
            });
        }
        for i in 0..self.groups.len() {
            for j in 0..i {
                let (a, b) = (&self.groups[i], &self.groups[j]);
                let (al, ar, bl, br) = (
                    &paths[&a.left_section.unwrap()],
                    &paths[&a.right_section.unwrap()],
                    &paths[&b.left_section.unwrap()],
                    &paths[&b.right_section.unwrap()],
                );
                if al.iter().chain(ar).chain(bl).chain(br).any(|p| {
                    let y = p[1];
                    let z = p[2];
                    match (
                        boundary_x_at(al, y, z),
                        boundary_x_at(ar, y, z),
                        boundary_x_at(bl, y, z),
                        boundary_x_at(br, y, z),
                    ) {
                        (Some(a), Some(b), Some(c), Some(d)) => {
                            b.min(d) - a.max(c) > JOIN_TOLERANCE
                        }
                        _ => false,
                    }
                }) {
                    self.grouping_errors.push(format!(
                        "成品组 {} 与 {} 重叠，不支持自动共切",
                        j + 1,
                        i + 1
                    ));
                }
            }
        }
        let mut loose: Vec<_> = ids
            .iter()
            .filter(|id| !self.groups.iter().any(|g| g.units.contains(id)))
            .copied()
            .collect();
        for unit in cam.units.iter().filter(|u| !ids.contains(&u.id)) {
            let Some(points) = ordered_unit_points(display, cam, unit.id) else {
                self.grouping_errors
                    .push(format!("加工单元 #{:08X} 的路径不完整", unit.id.0));
                loose.push(unit.id);
                continue;
            };
            let mut owners = Vec::new();
            let mut partial = false;
            for group in &self.groups {
                let (l, r) = (
                    &paths[&group.left_section.unwrap()],
                    &paths[&group.right_section.unwrap()],
                );
                let inside: Vec<_> = points
                    .iter()
                    .map(
                        |p| match (boundary_x_at(l, p[1], p[2]), boundary_x_at(r, p[1], p[2])) {
                            (Some(l), Some(r)) => {
                                p[0] >= l - JOIN_TOLERANCE && p[0] <= r + JOIN_TOLERANCE
                            }
                            _ => false,
                        },
                    )
                    .collect();
                if !inside.is_empty() && inside.iter().all(|v| *v) {
                    owners.push(group.id as usize);
                } else if inside.iter().any(|v| *v) {
                    partial = true;
                }
            }
            if owners.len() == 1 && !partial {
                self.groups[owners[0]].units.push(unit.id);
            } else {
                if partial || owners.len() > 1 {
                    self.grouping_errors.push(format!(
                        "加工单元 #{:08X} 跨越成品边界或归属不唯一",
                        unit.id.0
                    ));
                }
                loose.push(unit.id);
            }
        }
        if !loose.is_empty() {
            let bounds = loose
                .iter()
                .filter_map(|id| unit_x_bounds(display, cam, *id))
                .collect::<Vec<_>>();
            self.groups.push(MachiningGroup {
                id: self.groups.len() as u32,
                x_start: bounds.iter().map(|(x, _)| *x).fold(f64::INFINITY, f64::min),
                x_end: bounds
                    .iter()
                    .map(|(_, x)| *x)
                    .fold(f64::NEG_INFINITY, f64::max),
                left_section: None,
                right_section: None,
                units: loose,
                zone_groups: Vec::new(),
            });
        }
        for group in &mut self.groups {
            group.zone_groups = TubeZone::ALL
                .iter()
                .copied()
                .map(|zone| ZoneUnitGroup {
                    zone,
                    units: Vec::new(),
                })
                .collect();
            for id in &group.units {
                let zone = self
                    .zone_profiles
                    .get(id)
                    .and_then(|p| p.owner_zone)
                    .unwrap_or(TubeZone::TopFace);
                group.zone_groups[zone.index()].units.push(*id);
            }
        }
    }

    fn rebuild_plan(&mut self, display: &DisplayData, cam: &CamModel) {
        if self.tube_section.is_none() {
            self.rebuild_without_tube_section(display, cam);
            return;
        }
        self.plan.groups.clear();
        let mut next_order = 1u32;
        for group in self
            .groups
            .iter()
            .filter(|g| g.left_section.is_none())
            .chain(self.groups.iter().filter(|g| g.left_section.is_some()))
        {
            let mut planned = Vec::new();
            if self.lazy_rotation {
                for zone in ordered_zones(self.rotation_direction) {
                    let zone_group = &group.zone_groups[zone.index()];
                    let mut units = zone_group.units.clone();
                    units.sort_by(|left, right| {
                        let left_x = unit_x_center(display, cam, *left).unwrap_or(f64::INFINITY);
                        let right_x = unit_x_center(display, cam, *right).unwrap_or(f64::INFINITY);
                        left_x
                            .partial_cmp(&right_x)
                            .unwrap_or(std::cmp::Ordering::Equal)
                            .then_with(|| left.cmp(right))
                    });
                    for unit_id in units {
                        if self.machining_sections.iter().any(|s| s.unit_id == unit_id) {
                            continue;
                        }
                        planned.push(PlanUnit {
                            unit_id,
                            zone: zone_group.zone,
                            order: next_order,
                            machining_cut: false,
                            direction: UnitDirection::Forward,
                            start_point: [0.0; 3],
                            end_point: [0.0; 3],
                        });
                        next_order += 1;
                    }
                }
            } else {
                let mut units: Vec<(UnitId, TubeZone)> = Vec::new();
                for zone_group in &group.zone_groups {
                    for unit_id in &zone_group.units {
                        if !self
                            .machining_sections
                            .iter()
                            .any(|s| s.unit_id == *unit_id)
                        {
                            units.push((*unit_id, zone_group.zone));
                        }
                    }
                }
                units.sort_by(|(left_id, left_zone), (right_id, right_zone)| {
                    let left_x = unit_x_bounds(display, cam, *left_id)
                        .map(|(minimum, _)| minimum)
                        .unwrap_or(f64::INFINITY);
                    let right_x = unit_x_bounds(display, cam, *right_id)
                        .map(|(minimum, _)| minimum)
                        .unwrap_or(f64::INFINITY);
                    left_x
                        .partial_cmp(&right_x)
                        .unwrap_or(std::cmp::Ordering::Equal)
                        .then_with(|| {
                            zone_rank(*left_zone, self.rotation_direction)
                                .cmp(&zone_rank(*right_zone, self.rotation_direction))
                        })
                        .then_with(|| left_id.cmp(right_id))
                });
                for (unit_id, zone) in units {
                    planned.push(PlanUnit {
                        unit_id,
                        zone,
                        order: next_order,
                        machining_cut: false,
                        direction: UnitDirection::Forward,
                        start_point: [0.0; 3],
                        end_point: [0.0; 3],
                    });
                    next_order += 1;
                }
            }

            let cuts = if let (Some(left), Some(right)) = (group.left_section, group.right_section)
            {
                vec![left, right]
            } else {
                group
                    .units
                    .iter()
                    .copied()
                    .filter(|id| self.machining_sections.iter().any(|s| s.unit_id == *id))
                    .collect()
            };
            for section_unit in cuts {
                let zone = self
                    .zone_profiles
                    .get(&section_unit)
                    .and_then(|profile| profile.owner_zone)
                    .unwrap_or(TubeZone::TopFace);
                let cut = PlanUnit {
                    unit_id: section_unit,
                    zone,
                    order: next_order,
                    machining_cut: true,
                    direction: UnitDirection::Forward,
                    start_point: [0.0; 3],
                    end_point: [0.0; 3],
                };
                if group.left_section == Some(section_unit) {
                    planned.insert(0, cut);
                } else {
                    planned.push(cut);
                }
                next_order += 1;
            }

            if !planned.is_empty() {
                self.plan.groups.push(PlanGroup {
                    group_id: group.id,
                    x_start: group.x_start,
                    x_end: group.x_end,
                    units: planned,
                });
            }
        }
        self.refresh_plan_entries(display, cam);
    }

    /// Change lazy-rotation / rotation-direction planning policy.
    pub fn set_plan_options(
        &mut self,
        lazy_rotation: bool,
        rotation_direction: RotationDirection,
        display: &DisplayData,
        cam: &CamModel,
    ) -> bool {
        if self.lazy_rotation == lazy_rotation && self.rotation_direction == rotation_direction {
            return false;
        }
        self.lazy_rotation = lazy_rotation;
        self.rotation_direction = rotation_direction;
        self.rebuild_plan(display, cam);
        true
    }

    /// Move one non-section plan unit inside its group and refresh direction/start.
    pub fn move_plan_unit(
        &mut self,
        display: &DisplayData,
        cam: &CamModel,
        group_id: u32,
        index: usize,
        delta: isize,
    ) -> bool {
        let Some(group) = self
            .plan
            .groups
            .iter_mut()
            .find(|group| group.group_id == group_id)
        else {
            return false;
        };
        let target = index as isize + delta;
        if index >= group.units.len() || target < 0 || target >= group.units.len() as isize {
            return false;
        }
        let target = target as usize;
        if group.units[index].machining_cut || group.units[target].machining_cut {
            return false;
        }
        group.units.swap(index, target);

        self.refresh_plan_entries(display, cam);
        true
    }

    /// Move one non-section plan unit to a new index inside its group.
    pub fn move_plan_unit_to(
        &mut self,
        display: &DisplayData,
        cam: &CamModel,
        group_id: u32,
        from: usize,
        to: usize,
    ) -> bool {
        let left_section = self
            .groups
            .iter()
            .find(|g| g.id == group_id)
            .and_then(|g| g.left_section);
        let Some(group) = self
            .plan
            .groups
            .iter_mut()
            .find(|group| group.group_id == group_id)
        else {
            return false;
        };
        if from >= group.units.len()
            || to > group.units.len()
            || from == to
            || group.units[from].machining_cut
        {
            return false;
        }
        let unit = group.units.remove(from);
        let first = usize::from(
            left_section.is_some()
                && group
                    .units
                    .first()
                    .is_some_and(|u| Some(u.unit_id) == left_section),
        );
        let last = group
            .units
            .iter()
            .enumerate()
            .skip(first)
            .find(|(_, u)| u.machining_cut)
            .map(|(i, _)| i)
            .unwrap_or(group.units.len());
        let insert = to.clamp(first, last);
        group.units.insert(insert, unit);

        self.refresh_plan_entries(display, cam);
        true
    }

    fn refresh_plan_entries(&mut self, display: &DisplayData, cam: &CamModel) {
        let first_section_id = self
            .plan
            .groups
            .iter()
            .flat_map(|g| &g.units)
            .find(|u| u.machining_cut)
            .map(|u| u.unit_id);
        let last_section_id = self
            .plan
            .groups
            .iter()
            .flat_map(|g| &g.units)
            .rfind(|u| u.machining_cut)
            .map(|u| u.unit_id);
        let mut previous_end = None;
        let mut previous_direction = None;
        let mut previous_zone = None;
        let mut order = 1;
        for group in &mut self.plan.groups {
            if self.lazy_rotation {
                previous_end = None;
                previous_direction = None;
                previous_zone = None;
            }
            for unit in &mut group.units {
                unit.order = order;
                order += 1;
                let entry = if let Some(section) = &self.tube_section {
                    if unit.machining_cut {
                        choose_section_entry(
                            display,
                            cam,
                            section,
                            unit,
                            previous_end,
                            previous_direction,
                            previous_zone,
                            self.machine_start,
                            first_section_id == Some(unit.unit_id),
                            last_section_id == Some(unit.unit_id),
                        )
                        .or_else(|| choose_unit_direction(display, cam, unit.unit_id, previous_end))
                    } else {
                        choose_unit_entry(
                            display,
                            cam,
                            section,
                            unit.unit_id,
                            unit.zone,
                            self.lazy_rotation,
                            self.rotation_direction,
                        )
                    }
                } else {
                    choose_unit_direction(display, cam, unit.unit_id, previous_end)
                };
                if let Some((direction, start, end)) = entry {
                    unit.direction = direction;
                    unit.start_point = start;
                    unit.end_point = end;
                    previous_end = Some(end);
                    previous_direction =
                        unit_exit_direction(display, cam, unit.unit_id, direction, start);
                }
                previous_zone = Some(unit.zone);
            }
        }
    }
}

pub fn boundary_x_at(path: &[[f64; 3]], y: f64, z: f64) -> Option<f64> {
    if path.len() < 2 {
        return None;
    }
    let mut best = None;
    for i in 0..path.len() {
        let (a, b) = (path[i], path[(i + 1) % path.len()]);
        let (dy, dz) = (b[1] - a[1], b[2] - a[2]);
        let length = dy * dy + dz * dz;
        if length <= 1.0e-18 {
            continue;
        }
        let t = (((y - a[1]) * dy + (z - a[2]) * dz) / length).clamp(0.0, 1.0);
        let distance = (y - a[1] - t * dy).hypot(z - a[2] - t * dz);
        if best.is_none_or(|(d, _)| distance < d) {
            best = Some((distance, a[0] + t * (b[0] - a[0])));
        }
    }
    best.map(|(_, x)| x)
}

pub fn check_boundary_pair(left: &[[f64; 3]], right: &[[f64; 3]]) -> Result<(), String> {
    if left.len() < 3 || right.len() < 3 {
        return Err("断面采样不足".to_string());
    }
    for p in left.iter().chain(right) {
        match (
            boundary_x_at(left, p[1], p[2]),
            boundary_x_at(right, p[1], p[2]),
        ) {
            (Some(l), Some(r)) if l.is_finite() && r.is_finite() && r - l > JOIN_TOLERANCE => {}
            _ => return Err("左右断面顺序颠倒、相交或间距不足".to_string()),
        }
    }
    Ok(())
}

pub fn detect_tube_candidates(display: &DisplayData, cam: &CamModel) -> Vec<TubeCandidate> {
    // The tube section is inferred from the YZ envelope of all closed
    // machining units. Open helpers do not define the tube section.
    let closed_units: std::collections::BTreeSet<UnitId> = cam
        .units
        .iter()
        .filter(|unit| unit.closed)
        .map(|unit| unit.id)
        .collect();
    let mut points = Vec::new();
    for path in &display.paths {
        if !cam
            .unit_for_entity(path.entity_id)
            .map(|unit_id| closed_units.contains(&unit_id))
            .unwrap_or(false)
        {
            continue;
        }
        for point in &path.points {
            points.push((point[1], point[2]));
        }
    }
    if points.len() < 8 {
        return Vec::new();
    }

    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;
    for (y, z) in &points {
        min_y = min_y.min(*y);
        max_y = max_y.max(*y);
        min_z = min_z.min(*z);
        max_z = max_z.max(*z);
    }
    let size_y = max_y - min_y;
    let size_z = max_z - min_z;
    if size_y <= JOIN_TOLERANCE || size_z <= JOIN_TOLERANCE {
        return Vec::new();
    }
    let center_y = (min_y + max_y) * 0.5;
    let center_z = (min_z + max_z) * 0.5;
    let radius = estimate_corner_radius(&points, center_y, center_z, size_y, size_z);

    let mut candidates = vec![TubeCandidate {
        center_y,
        center_z,
        size_y,
        size_z,
        radius,
        score: 1.0,
    }];
    candidates.sort_by(|left, right| {
        right
            .score
            .partial_cmp(&left.score)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    candidates
}

fn estimate_corner_radius(
    points: &[(f64, f64)],
    center_y: f64,
    center_z: f64,
    size_y: f64,
    size_z: f64,
) -> f64 {
    let half_y = size_y * 0.5;
    let half_z = size_z * 0.5;
    let corner_points = [
        (center_y + half_y, center_z + half_z),
        (center_y + half_y, center_z - half_z),
        (center_y - half_y, center_z - half_z),
        (center_y - half_y, center_z + half_z),
    ];
    let mut estimates = Vec::new();
    for (corner_y, corner_z) in corner_points {
        let mut nearest = f64::INFINITY;
        for (y, z) in points {
            if (y - corner_y).abs() <= half_y && (z - corner_z).abs() <= half_z {
                let distance = (y - corner_y).hypot(z - corner_z);
                nearest = nearest.min(distance);
            }
        }
        if nearest.is_finite() {
            estimates.push(nearest / (std::f64::consts::SQRT_2 - 1.0));
        }
    }
    if estimates.is_empty() {
        0.0
    } else {
        let average = estimates.iter().sum::<f64>() / estimates.len() as f64;
        average.clamp(0.0, half_y.min(half_z) * 0.49)
    }
}

pub fn detect_machining_candidates(
    display: &DisplayData,
    cam: &CamModel,
    section: &TubeSection,
) -> Vec<MachiningSectionCandidate> {
    let boundary = section.boundary();
    let mut candidates = Vec::new();
    for unit in &cam.units {
        if !unit.closed {
            continue;
        }
        let Some(points) = ordered_unit_points(display, cam, unit.id) else {
            continue;
        };
        if points.len() < 3 {
            continue;
        }
        let mut x_min = f64::INFINITY;
        let mut x_max = f64::NEG_INFINITY;
        let mut previous_perimeter: Option<f64> = None;
        let mut total_travel = 0.0;
        let mut max_deviation: f64 = 0.0;
        let mut bins = [false; BOUNDARY_BINS];
        let mut zone_mask = [false; 16];
        let mut valid = true;

        for point in &points {
            x_min = x_min.min(point[0]);
            x_max = x_max.max(point[0]);
            let local_y = point[1] - section.center_y;
            let local_z = point[2] - section.center_z;
            let Some((perimeter, deviation, zone)) = boundary.nearest(local_y, local_z) else {
                valid = false;
                break;
            };
            if deviation > SURFACE_TOLERANCE {
                valid = false;
                break;
            }
            max_deviation = max_deviation.max(deviation);
            zone_mask[zone.index()] = true;
            bins[((perimeter / boundary.perimeter).clamp(0.0, 1.0) * (BOUNDARY_BINS - 1) as f64)
                as usize] = true;
            if let Some(previous) = previous_perimeter {
                let mut delta = perimeter - previous;
                if delta > boundary.perimeter * 0.5 {
                    delta -= boundary.perimeter;
                } else if delta < -boundary.perimeter * 0.5 {
                    delta += boundary.perimeter;
                }
                total_travel += delta;
                mark_travel_bins(&mut bins, previous, delta, boundary.perimeter);
            }
            previous_perimeter = Some(perimeter);
        }

        if !valid {
            continue;
        }
        let winding = (total_travel / boundary.perimeter).round() as i32;
        if winding.abs() != 1 {
            continue;
        }
        let coverage_gap_bins = maximum_coverage_gap(&bins);
        if coverage_gap_bins > 3 {
            continue;
        }
        candidates.push(MachiningSectionCandidate {
            unit_id: unit.id,
            x_min,
            x_max,
            winding,
            maximum_deviation: max_deviation,
            coverage_gap_bins,
            zone_mask,
        });
    }
    candidates.sort_by(|left, right| {
        left.x_min
            .partial_cmp(&right.x_min)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    candidates
}

fn mark_travel_bins(bins: &mut [bool; BOUNDARY_BINS], start: f64, delta: f64, perimeter: f64) {
    if perimeter <= f64::EPSILON {
        return;
    }
    let steps = ((delta.abs() / perimeter) * BOUNDARY_BINS as f64 * 2.0).ceil() as usize;
    for step in 0..=steps.max(1) {
        let t = step as f64 / steps.max(1) as f64;
        let position = (start + delta * t).rem_euclid(perimeter);
        let index = ((position / perimeter) * (BOUNDARY_BINS - 1) as f64) as usize;
        bins[index.min(BOUNDARY_BINS - 1)] = true;
    }
}

fn maximum_coverage_gap(bins: &[bool; BOUNDARY_BINS]) -> usize {
    let mut first = None;
    let mut last = None;
    for (index, hit) in bins.iter().enumerate() {
        if *hit {
            first.get_or_insert(index);
            last = Some(index);
        }
    }
    let (Some(first), Some(last)) = (first, last) else {
        return BOUNDARY_BINS;
    };
    let mut gap = 0;
    let mut current = 0;
    for index in 0..BOUNDARY_BINS {
        if bins[index] {
            current = 0;
        } else {
            current += 1;
            gap = gap.max(current);
        }
    }
    // Circular gap across the end/start boundary.
    let mut leading = 0;
    for hit in bins.iter() {
        if *hit {
            break;
        }
        leading += 1;
    }
    let mut trailing = 0;
    for hit in bins.iter().rev() {
        if *hit {
            break;
        }
        trailing += 1;
    }
    gap.max(leading + trailing)
        .max(BOUNDARY_BINS - (last - first + 1))
}

fn ordered_unit_points(
    display: &DisplayData,
    cam: &CamModel,
    unit_id: UnitId,
) -> Option<Vec<[f64; 3]>> {
    let mut paths: Vec<&crate::display::DisplayPath> = display
        .paths
        .iter()
        .filter(|path| cam.unit_for_entity(path.entity_id) == Some(unit_id))
        .collect();
    if paths.is_empty() {
        return None;
    }
    let mut ordered: Vec<[f64; 3]> = paths.remove(0).points.clone();
    while !paths.is_empty() {
        let end = *ordered.last()?;
        let mut best: Option<(usize, bool, f64)> = None;
        for (index, path) in paths.iter().enumerate() {
            let start = *path.points.first()?;
            let last = *path.points.last()?;
            let start_distance = distance3(end, start);
            let end_distance = distance3(end, last);
            let candidate = if start_distance <= end_distance {
                (index, false, start_distance)
            } else {
                (index, true, end_distance)
            };
            if best
                .map(|(_, _, best_distance)| candidate.2 < best_distance)
                .unwrap_or(true)
            {
                best = Some(candidate);
            }
        }
        let (index, reverse, distance) = best?;
        if distance > 1.0 {
            return None;
        }
        let mut points = paths.remove(index).points.clone();
        if reverse {
            points.reverse();
        }
        for point in points.into_iter().skip(1) {
            ordered.push(point);
        }
    }
    Some(ordered)
}

fn choose_unit_direction(
    display: &DisplayData,
    cam: &CamModel,
    unit_id: UnitId,
    previous_end: Option<[f64; 3]>,
) -> Option<(UnitDirection, [f64; 3], [f64; 3])> {
    let path = ordered_unit_points(display, cam, unit_id)?;
    let start = *path.first()?;
    let end = *path.last()?;
    if let Some(previous_end) = previous_end {
        if distance3(previous_end, end) < distance3(previous_end, start) {
            return Some((UnitDirection::Reverse, end, start));
        }
    }
    Some((UnitDirection::Forward, start, end))
}

pub fn oriented_unit_path(
    display: &DisplayData,
    cam: &CamModel,
    unit_id: UnitId,
    direction: UnitDirection,
) -> Option<Vec<[f64; 3]>> {
    oriented_unit_path_from(display, cam, unit_id, direction, None)
}

pub fn oriented_unit_path_from(
    display: &DisplayData,
    cam: &CamModel,
    unit_id: UnitId,
    direction: UnitDirection,
    start_point: Option<[f64; 3]>,
) -> Option<Vec<[f64; 3]>> {
    let mut path = ordered_unit_points(display, cam, unit_id)?;
    let closed = cam
        .unit(unit_id)
        .map(|machining_unit| machining_unit.closed)
        .unwrap_or(false);
    if direction == UnitDirection::Reverse {
        path.reverse();
    }
    if closed {
        if path.len() >= 2 && distance3(path[0], *path.last()?) > 1.0e-9 {
            path.push(path[0]);
        }
        if let Some(start_point) = start_point {
            let span = path.len().saturating_sub(1);
            if span >= 2 {
                let index = (0..span)
                    .min_by(|left, right| {
                        distance3(path[*left], start_point)
                            .partial_cmp(&distance3(path[*right], start_point))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .unwrap_or(0);
                if index > 0 {
                    let mut rotated = Vec::with_capacity(path.len());
                    rotated.extend_from_slice(&path[index..]);
                    rotated.extend_from_slice(&path[1..=index]);
                    path = rotated;
                }
            }
        }
    }
    Some(path)
}

fn entry_score(
    start: [f64; 3],
    tangent_point: [f64; 3],
    previous_end: Option<[f64; 3]>,
    previous_direction: Option<[f64; 3]>,
) -> Option<(f64, f64, f64)> {
    let previous_end = previous_end?;
    let approach = [
        start[0] - previous_end[0],
        start[1] - previous_end[1],
        start[2] - previous_end[2],
    ];
    let tangent = [
        tangent_point[0] - start[0],
        tangent_point[1] - start[1],
        tangent_point[2] - start[2],
    ];
    let approach_length = vector_length(approach);
    let tangent_length = vector_length(tangent);
    if approach_length <= 1.0e-9 || tangent_length <= 1.0e-9 {
        return None;
    }
    let approach_cos =
        (approach[0] * tangent[0] + approach[1] * tangent[1] + approach[2] * tangent[2])
            / (approach_length * tangent_length);
    let previous_angle = previous_direction
        .map(|previous_direction| {
            let length = vector_length(previous_direction);
            if length <= 1.0e-9 {
                return 0.0;
            }
            let cosine = (approach[0] * previous_direction[0]
                + approach[1] * previous_direction[1]
                + approach[2] * previous_direction[2])
                / (approach_length * length);
            cosine.clamp(-1.0, 1.0).acos()
        })
        .unwrap_or(0.0);
    let distance = approach_length;
    Some((
        previous_angle,
        approach_cos.clamp(-1.0, 1.0).acos(),
        distance,
    ))
}

fn entry_obtuse(previous_angle: f64, tangent_angle: f64) -> f64 {
    let limit = std::f64::consts::FRAC_PI_2 + 1.0e-6;
    if previous_angle > limit || tangent_angle > limit {
        1.0
    } else {
        0.0
    }
}

fn better_entry(
    candidate: (f64, f64, f64, f64, f64),
    current: Option<(f64, f64, f64, f64, f64)>,
) -> bool {
    let Some((
        current_obtuse,
        current_prev_angle,
        current_tangent_angle,
        current_a_change,
        current_distance,
    )) = current
    else {
        return true;
    };
    if candidate.0 < current_obtuse - 1.0e-9 {
        return true;
    }
    if (candidate.0 - current_obtuse).abs() > 1.0e-9 {
        return false;
    }
    if candidate.1 < current_prev_angle - ENTRY_ANGLE_TOLERANCE {
        return true;
    }
    if (candidate.1 - current_prev_angle).abs() > ENTRY_ANGLE_TOLERANCE {
        return false;
    }
    if candidate.2 < current_tangent_angle - ENTRY_ANGLE_TOLERANCE {
        return true;
    }
    if (candidate.2 - current_tangent_angle).abs() > ENTRY_ANGLE_TOLERANCE {
        return false;
    }
    if candidate.3 < current_a_change - 1.0e-9 {
        return true;
    }
    (candidate.3 - current_a_change).abs() <= 1.0e-9 && candidate.4 < current_distance
}

fn zone_local_vertical(zone: TubeZone) -> [f64; 2] {
    // Local north is +Y after rotating this zone to its upward cutting pose.
    match zone {
        TubeZone::TopFace | TubeZone::TopToTopRightBoundary | TubeZone::TopLeftToTopBoundary => {
            [1.0, 0.0]
        }
        TubeZone::RightFace
        | TubeZone::TopRightToRightBoundary
        | TubeZone::RightToBottomRightBoundary => [0.0, -1.0],
        TubeZone::BottomFace
        | TubeZone::BottomRightToBottomBoundary
        | TubeZone::BottomToBottomLeftBoundary => [-1.0, 0.0],
        TubeZone::LeftFace
        | TubeZone::BottomLeftToLeftBoundary
        | TubeZone::LeftToTopLeftBoundary => [0.0, 1.0],
        TubeZone::TopRightCorner => [1.0, -1.0],
        TubeZone::BottomRightCorner => [-1.0, -1.0],
        TubeZone::BottomLeftCorner => [-1.0, 1.0],
        TubeZone::TopLeftCorner => [1.0, 1.0],
    }
}

fn zone_is_compatible(candidate: TubeZone, desired: TubeZone) -> bool {
    if candidate == desired {
        return true;
    }
    match desired {
        TubeZone::TopFace => matches!(
            candidate,
            TubeZone::TopToTopRightBoundary
                | TubeZone::TopLeftToTopBoundary
                | TubeZone::TopLeftCorner
                | TubeZone::TopRightCorner
        ),
        TubeZone::RightFace => matches!(
            candidate,
            TubeZone::TopRightToRightBoundary
                | TubeZone::RightToBottomRightBoundary
                | TubeZone::TopRightCorner
                | TubeZone::BottomRightCorner
        ),
        TubeZone::BottomFace => matches!(
            candidate,
            TubeZone::BottomRightToBottomBoundary
                | TubeZone::BottomToBottomLeftBoundary
                | TubeZone::BottomRightCorner
                | TubeZone::BottomLeftCorner
        ),
        TubeZone::LeftFace => matches!(
            candidate,
            TubeZone::BottomLeftToLeftBoundary
                | TubeZone::LeftToTopLeftBoundary
                | TubeZone::TopLeftCorner
                | TubeZone::BottomLeftCorner
        ),
        _ => false,
    }
}

fn on_zone_face_plane(zone: TubeZone, point: [f64; 3], section: &TubeSection) -> bool {
    let tolerance = 0.05;
    match zone {
        TubeZone::TopFace => (point[2] - (section.center_z + section.half_z())).abs() <= tolerance,
        TubeZone::BottomFace => {
            (point[2] - (section.center_z - section.half_z())).abs() <= tolerance
        }
        TubeZone::LeftFace => (point[1] - (section.center_y - section.half_y())).abs() <= tolerance,
        TubeZone::RightFace => {
            (point[1] - (section.center_y + section.half_y())).abs() <= tolerance
        }
        _ => false,
    }
}

fn choose_section_entry(
    display: &DisplayData,
    cam: &CamModel,
    section: &TubeSection,
    unit: &PlanUnit,
    previous_end: Option<[f64; 3]>,
    previous_direction: Option<[f64; 3]>,
    previous_zone: Option<TubeZone>,
    machine_start: [f64; 3],
    first_unit: bool,
    last_unit: bool,
) -> Option<(UnitDirection, [f64; 3], [f64; 3])> {
    let path = ordered_unit_points(display, cam, unit.unit_id)?;
    if path.len() < 2 {
        return None;
    }
    let closed = cam
        .unit(unit.unit_id)
        .map(|machining_unit| machining_unit.closed)
        .unwrap_or(false);
    let span = if closed {
        path.len().saturating_sub(1)
    } else {
        path.len()
    };
    if span < 2 {
        return None;
    }
    if first_unit {
        // The first machining section starts on the tube top surface.
        let collect = |predicate: &dyn Fn(usize) -> bool| -> Vec<usize> {
            if closed {
                (0..span).filter(|index| predicate(*index)).collect()
            } else {
                [0usize, path.len() - 1]
                    .into_iter()
                    .filter(|index| predicate(*index))
                    .collect()
            }
        };
        let mut candidate_indices =
            collect(&|index| on_zone_face_plane(TubeZone::TopFace, path[index], section));
        if candidate_indices.is_empty() {
            candidate_indices = collect(&|index| {
                section
                    .zone_for_point(path[index])
                    .map(|point_zone| zone_is_compatible(point_zone, TubeZone::TopFace))
                    .unwrap_or(false)
            });
        }
        let index = candidate_indices.into_iter().min_by(|left, right| {
            distance3(path[*left], machine_start)
                .partial_cmp(&distance3(path[*right], machine_start))
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        let start = path[index];
        let direction = if closed {
            let forward = path[(index + 1) % span];
            let reverse = path[(index + span - 1) % span];
            if reverse[0] - start[0] > forward[0] - start[0] + 1.0e-9 {
                UnitDirection::Reverse
            } else {
                UnitDirection::Forward
            }
        } else if index == 0 {
            UnitDirection::Forward
        } else {
            UnitDirection::Reverse
        };
        let end = if closed {
            start
        } else if direction == UnitDirection::Forward {
            path[path.len() - 1]
        } else {
            path[0]
        };
        return Some((direction, start, end));
    }
    if last_unit {
        let previous_zone = previous_zone?;
        let collect = |predicate: &dyn Fn(usize) -> bool| -> Vec<usize> {
            if closed {
                (0..span).filter(|index| predicate(*index)).collect()
            } else {
                [0usize, path.len() - 1]
                    .into_iter()
                    .filter(|index| predicate(*index))
                    .collect()
            }
        };
        let mut candidate_indices =
            collect(&|index| on_zone_face_plane(previous_zone, path[index], section));
        if candidate_indices.is_empty() {
            candidate_indices =
                collect(&|index| section.zone_for_point(path[index]) == Some(previous_zone));
        }
        if candidate_indices.is_empty() {
            candidate_indices = collect(&|index| {
                section
                    .zone_for_point(path[index])
                    .map(|point_zone| zone_is_compatible(point_zone, previous_zone))
                    .unwrap_or(false)
            });
        }
        let mut best: Option<(f64, f64, f64, f64, f64, UnitDirection, [f64; 3], [f64; 3])> = None;
        for index in candidate_indices {
            let start = path[index];
            let directions: &[UnitDirection] = if closed {
                &[UnitDirection::Forward, UnitDirection::Reverse]
            } else if index == 0 {
                &[UnitDirection::Forward]
            } else {
                &[UnitDirection::Reverse]
            };
            for direction in directions {
                let tangent_point = if closed {
                    match direction {
                        UnitDirection::Forward => path[(index + 1) % span],
                        UnitDirection::Reverse => path[(index + span - 1) % span],
                    }
                } else if index == 0 {
                    path[1]
                } else {
                    path[path.len() - 2]
                };
                let Some((previous_angle, tangent_angle, distance)) =
                    entry_score(start, tangent_point, previous_end, previous_direction)
                else {
                    continue;
                };
                let end = if closed {
                    start
                } else if *direction == UnitDirection::Forward {
                    path[path.len() - 1]
                } else {
                    path[0]
                };
                let candidate = (
                    entry_obtuse(previous_angle, tangent_angle),
                    previous_angle,
                    tangent_angle,
                    0.0,
                    distance,
                    *direction,
                    start,
                    end,
                );
                if better_entry(
                    (
                        candidate.0,
                        candidate.1,
                        candidate.2,
                        candidate.3,
                        candidate.4,
                    ),
                    best.map(|current| (current.0, current.1, current.2, current.3, current.4)),
                ) {
                    best = Some(candidate);
                }
            }
        }
        if let Some((_, _, _, _, _, direction, start, end)) = best {
            return Some((direction, start, end));
        }
    }
    None
}

fn choose_unit_entry(
    display: &DisplayData,
    cam: &CamModel,
    section: &TubeSection,
    unit_id: UnitId,
    zone: TubeZone,
    lazy_rotation: bool,
    rotation_direction: RotationDirection,
) -> Option<(UnitDirection, [f64; 3], [f64; 3])> {
    let path = ordered_unit_points(display, cam, unit_id)?;
    if path.len() < 2 {
        return None;
    }
    let closed = cam.unit(unit_id)?.closed;
    let span = if closed && distance3(path[0], *path.last()?) <= 1.0e-9 {
        path.len() - 1
    } else {
        path.len()
    };
    if span < 2 {
        return None;
    }
    let north = zone_local_vertical(zone);
    let height = |point: [f64; 3]| point[1] * north[0] + point[2] * north[1];
    let pole = |index: usize| {
        if lazy_rotation {
            height(path[index])
        } else {
            path[index][0]
        }
    };
    // An open stroke may only start at an endpoint; never drop its other half.
    let candidates: Vec<usize> = if closed {
        (0..span).collect()
    } else {
        vec![0, path.len() - 1]
    };
    let maximum = candidates
        .iter()
        .map(|index| pole(*index))
        .fold(f64::NEG_INFINITY, f64::max);
    let minimum = candidates
        .iter()
        .map(|index| pole(*index))
        .fold(f64::INFINITY, f64::min);
    let in_zone = |index: usize| section.zone_for_point(path[index]) == Some(zone);
    let mut eligible: Vec<usize> = candidates
        .iter()
        .copied()
        .filter(|index| in_zone(*index) && (pole(*index) - maximum).abs() <= 1.0e-7)
        .collect();
    if eligible.is_empty() && lazy_rotation {
        eligible = candidates
            .iter()
            .copied()
            .filter(|index| in_zone(*index) && (pole(*index) - minimum).abs() <= 1.0e-7)
            .collect();
    }
    if eligible.is_empty() {
        eligible = candidates
            .iter()
            .copied()
            .filter(|index| in_zone(*index))
            .collect();
    }
    if eligible.is_empty() {
        eligible = candidates;
    }
    let index = eligible.into_iter().max_by(|left, right| {
        pole(*left)
            .total_cmp(&pole(*right))
            .then_with(|| path[*left][0].total_cmp(&path[*right][0]))
            .then_with(|| height(path[*left]).total_cmp(&height(path[*right])))
    })?;
    let start = path[index];
    let direction = if closed {
        let forward = path[(index + 1) % span];
        let reverse = path[(index + span - 1) % span];
        let score = |point: [f64; 3]| {
            let length = distance3(start, point);
            if length <= 1.0e-9 {
                return f64::NEG_INFINITY;
            }
            if lazy_rotation {
                (point[0] - start[0]) / length
            } else {
                let sign = if rotation_direction == RotationDirection::CounterClockwise {
                    1.0
                } else {
                    -1.0
                };
                sign * (height(point) - height(start)) / length
            }
        };
        if score(reverse) > score(forward) + 1.0e-9 {
            UnitDirection::Reverse
        } else {
            UnitDirection::Forward
        }
    } else if index == 0 {
        UnitDirection::Forward
    } else {
        UnitDirection::Reverse
    };
    let end = if closed {
        start
    } else if direction == UnitDirection::Forward {
        *path.last()?
    } else {
        path[0]
    };
    Some((direction, start, end))
}

fn unit_exit_direction(
    display: &DisplayData,
    cam: &CamModel,
    unit_id: UnitId,
    direction: UnitDirection,
    start_point: [f64; 3],
) -> Option<[f64; 3]> {
    let path = oriented_unit_path_from(display, cam, unit_id, direction, Some(start_point))?;
    if path.len() < 2 {
        return None;
    }
    let last = path[path.len() - 1];
    let previous = path[path.len() - 2];
    let vector = [
        last[0] - previous[0],
        last[1] - previous[1],
        last[2] - previous[2],
    ];
    let length = vector_length(vector);
    (length > 1.0e-9).then(|| [vector[0] / length, vector[1] / length, vector[2] / length])
}

fn vector_length(vector: [f64; 3]) -> f64 {
    (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
}

fn ordered_zones(direction: RotationDirection) -> Vec<TubeZone> {
    match direction {
        RotationDirection::CounterClockwise => TubeZone::ALL.to_vec(),
        RotationDirection::Clockwise => std::iter::once(TubeZone::ALL[0])
            .chain(TubeZone::ALL[1..].iter().rev().copied())
            .collect(),
    }
}

fn zone_rank(zone: TubeZone, direction: RotationDirection) -> usize {
    match direction {
        RotationDirection::CounterClockwise => zone.index(),
        RotationDirection::Clockwise => {
            if zone.index() == 0 {
                0
            } else {
                TubeZone::ALL.len() - zone.index()
            }
        }
    }
}

fn unit_x_bounds(display: &DisplayData, cam: &CamModel, unit_id: UnitId) -> Option<(f64, f64)> {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    for path in &display.paths {
        if cam.unit_for_entity(path.entity_id) != Some(unit_id) {
            continue;
        }
        for point in &path.points {
            min_x = min_x.min(point[0]);
            max_x = max_x.max(point[0]);
        }
    }
    (min_x.is_finite() && max_x.is_finite()).then_some((min_x, max_x))
}

fn unit_x_center(display: &DisplayData, cam: &CamModel, unit_id: UnitId) -> Option<f64> {
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    for path in &display.paths {
        if cam.unit_for_entity(path.entity_id) != Some(unit_id) {
            continue;
        }
        for point in &path.points {
            min_x = min_x.min(point[0]);
            max_x = max_x.max(point[0]);
        }
    }
    if min_x.is_finite() {
        Some((min_x + max_x) * 0.5)
    } else {
        None
    }
}

pub fn classify_units(
    display: &DisplayData,
    cam: &CamModel,
    section: &TubeSection,
) -> BTreeMap<UnitId, UnitZoneProfile> {
    let boundary = section.boundary();
    let mut profiles = BTreeMap::new();
    for unit in &cam.units {
        let mut zone_lengths = [0.0; 16];
        let mut touched_zones = [false; 16];
        let mut has_data = false;
        for path in &display.paths {
            if cam.unit_for_entity(path.entity_id) != Some(unit.id) {
                continue;
            }
            for pair in path.points.windows(2) {
                let mut previous: Option<(f64, TubeZone)> = None;
                const PROJECTION_STEPS: usize = 6;
                for step in 0..=PROJECTION_STEPS {
                    let t = step as f64 / PROJECTION_STEPS as f64;
                    let y = pair[0][1] + (pair[1][1] - pair[0][1]) * t;
                    let z = pair[0][2] + (pair[1][2] - pair[0][2]) * t;
                    let Some((perimeter, _, zone)) =
                        boundary.nearest(y - section.center_y, z - section.center_z)
                    else {
                        continue;
                    };
                    touched_zones[zone.index()] = true;
                    has_data = true;
                    if let Some((previous_perimeter, previous_zone)) = previous {
                        let mut delta = perimeter - previous_perimeter;
                        if delta > boundary.perimeter * 0.5 {
                            delta -= boundary.perimeter;
                        } else if delta < -boundary.perimeter * 0.5 {
                            delta += boundary.perimeter;
                        }
                        zone_lengths[previous_zone.index()] += delta.abs();
                    }
                    previous = Some((perimeter, zone));
                }
            }
        }
        if !has_data {
            continue;
        }
        let owner_zone = TubeZone::ALL.iter().copied().max_by(|left, right| {
            zone_lengths[left.index()]
                .partial_cmp(&zone_lengths[right.index()])
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        profiles.insert(
            unit.id,
            UnitZoneProfile {
                zone_lengths,
                touched_zones,
                owner_zone,
            },
        );
    }
    profiles
}

fn distance3(left: [f64; 3], right: [f64; 3]) -> f64 {
    let dx = right[0] - left[0];
    let dy = right[1] - left[1];
    let dz = right[2] - left[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}
