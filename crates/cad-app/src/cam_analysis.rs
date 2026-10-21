//! First CAM analysis layer: tube sections, machining-section candidates,
//! machining intervals and the 16 circumferential tube zones.
//!
//! This module is intentionally independent from Bevy/egui. It consumes the
//! display paths and machining units already produced by `display.rs` / `cam.rs`.

use std::collections::BTreeMap;

use crate::cam::{CamModel, UnitId};
use crate::display::DisplayData;

pub const SURFACE_TOLERANCE: f64 = 0.25;
pub const JOIN_TOLERANCE: f64 = 1.0e-3;
const BOUNDARY_BINS: usize = 512;

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
            TubeZone::TopToTopRightBoundary => "上→右上过渡",
            TubeZone::TopRightCorner => "右上圆角",
            TubeZone::TopRightToRightBoundary => "右上→右过渡",
            TubeZone::RightFace => "右表面",
            TubeZone::RightToBottomRightBoundary => "右→右下过渡",
            TubeZone::BottomRightCorner => "右下圆角",
            TubeZone::BottomRightToBottomBoundary => "右下→下过渡",
            TubeZone::BottomFace => "下表面",
            TubeZone::BottomToBottomLeftBoundary => "下→左下过渡",
            TubeZone::BottomLeftCorner => "左下圆角",
            TubeZone::BottomLeftToLeftBoundary => "左下→左过渡",
            TubeZone::LeftFace => "左表面",
            TubeZone::LeftToTopLeftBoundary => "左→左上过渡",
            TubeZone::TopLeftCorner => "左上圆角",
            TubeZone::TopLeftToTopBoundary => "左上→上过渡",
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
    pub unit_id: Option<UnitId>,
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
            source_unit: self.unit_id,
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
    pub source_unit: Option<UnitId>,
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
    pub score: f64,
}

#[derive(Clone, Debug)]
pub struct MachiningSection {
    pub unit_id: UnitId,
    pub boundary_units: Vec<UnitId>,
    pub x_min: f64,
    pub x_max: f64,
    pub winding: i32,
    pub zone_mask: [bool; 16],
    pub source: SectionSource,
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

/// A group delimited by machining cuts. Machining reaches `section_unit` at the
/// end of the group; everything before it is then released.
#[derive(Clone, Debug)]
pub struct MachiningGroup {
    pub id: u32,
    pub x_start: f64,
    pub x_end: f64,
    pub section_unit: Option<UnitId>,
    pub units: Vec<UnitId>,
    pub zone_groups: Vec<ZoneUnitGroup>,
}

#[derive(Clone, Debug)]
pub struct UnitZoneProfile {
    pub unit_id: UnitId,
    pub zone_lengths: [f64; 16],
    pub touched_zones: [bool; 16],
    pub owner_zone: Option<TubeZone>,
}

#[derive(Clone, Debug, Default)]
pub struct CamAnalysis {
    pub tube_candidates: Vec<TubeCandidate>,
    pub tube_section: Option<TubeSection>,
    pub machining_candidates: Vec<MachiningSectionCandidate>,
    pub machining_sections: Vec<MachiningSection>,
    pub intervals: Vec<MachiningInterval>,
    pub groups: Vec<MachiningGroup>,
    pub zone_profiles: BTreeMap<UnitId, UnitZoneProfile>,
}

impl CamAnalysis {
    pub fn build(display: &DisplayData, cam: &CamModel) -> Self {
        let mut analysis = Self::default();
        analysis.tube_candidates = detect_tube_candidates(display, cam);
        analysis.try_auto_tube_section(display, cam);
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
            source: SectionSource::Auto,
        });
        self.machining_sections.sort_by(|left, right| {
            left.x_min
                .partial_cmp(&right.x_min)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        self.rebuild_intervals(display, cam);
        self.rebuild_groups(display, cam);
    }

    pub fn clear_tube_section(&mut self) {
        self.tube_section = None;
        self.machining_candidates.clear();
        self.machining_sections.clear();
        self.intervals.clear();
        self.groups.clear();
        self.zone_profiles.clear();
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
        self.zone_profiles.clear();
        let Some(section) = self.tube_section.clone() else {
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
                    source: SectionSource::Auto,
                });
            }
        }
        self.machining_sections.sort_by(|left, right| {
            left.x_min
                .partial_cmp(&right.x_min)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        self.zone_profiles = classify_units(display, cam, &section);
        self.rebuild_intervals(display, cam);
        self.rebuild_groups(display, cam);
    }

    fn rebuild_intervals(&mut self, display: &DisplayData, cam: &CamModel) {
        self.intervals.clear();
        let Some(bounds) = display.bounds else {
            return;
        };
        let mut boundaries: Vec<f64> = self
            .machining_sections
            .iter()
            .map(|section| section.x_min)
            .collect();
        boundaries
            .sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
        boundaries.dedup_by(|left, right| (*left - *right).abs() <= JOIN_TOLERANCE);

        let section_units: std::collections::BTreeSet<UnitId> = self
            .machining_sections
            .iter()
            .map(|section| section.unit_id)
            .collect();

        let mut edges = Vec::with_capacity(boundaries.len() + 2);
        edges.push(bounds.min_x);
        edges.extend(boundaries);
        edges.push(bounds.max_x);

        let mut intervals = Vec::new();
        for index in 0..edges.len().saturating_sub(1) {
            let x_start = edges[index];
            let x_end = edges[index + 1];
            if x_end - x_start <= JOIN_TOLERANCE {
                continue;
            }
            let units = cam
                .units
                .iter()
                .filter(|unit| !section_units.contains(&unit.id))
                .filter(|unit| {
                    unit_x_center(display, cam, unit.id)
                        .map(|x| x > x_start && x <= x_end)
                        .unwrap_or(false)
                })
                .map(|unit| unit.id)
                .collect();
            intervals.push(MachiningInterval {
                id: intervals.len() as u32,
                x_start,
                x_end,
                units,
            });
        }
        self.intervals = intervals;
    }

    fn rebuild_groups(&mut self, display: &DisplayData, cam: &CamModel) {
        self.groups.clear();
        let Some(bounds) = display.bounds else {
            return;
        };

        let mut sections: Vec<(f64, UnitId)> = self
            .machining_sections
            .iter()
            .map(|section| (section.x_min, section.unit_id))
            .collect();
        sections.sort_by(|left, right| {
            left.0
                .partial_cmp(&right.0)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        sections.dedup_by(|left, right| (left.0 - right.0).abs() <= JOIN_TOLERANCE);

        let section_units: std::collections::BTreeSet<UnitId> =
            sections.iter().map(|(_, unit_id)| *unit_id).collect();

        let mut edges = Vec::with_capacity(sections.len() + 2);
        edges.push(bounds.min_x);
        edges.extend(sections.iter().map(|(x, _)| *x));
        edges.push(bounds.max_x);

        let mut groups = Vec::new();
        for index in 0..edges.len().saturating_sub(1) {
            let x_start = edges[index];
            let x_end = edges[index + 1];
            let section_unit = sections.get(index).map(|(_, unit_id)| *unit_id);

            let mut units: Vec<UnitId> = cam
                .units
                .iter()
                .filter(|unit| !section_units.contains(&unit.id))
                .filter(|unit| {
                    unit_x_center(display, cam, unit.id)
                        .map(|x| x > x_start - JOIN_TOLERANCE && x <= x_end + JOIN_TOLERANCE)
                        .unwrap_or(false)
                })
                .map(|unit| unit.id)
                .collect();
            if let Some(section_unit) = section_unit {
                units.push(section_unit);
            }
            if units.is_empty() && section_unit.is_none() {
                continue;
            }
            units.sort_unstable();
            units.dedup();

            let mut zone_groups: Vec<ZoneUnitGroup> = TubeZone::ALL
                .iter()
                .copied()
                .map(|zone| ZoneUnitGroup {
                    zone,
                    units: Vec::new(),
                })
                .collect();
            for unit_id in &units {
                let zone = self
                    .zone_profiles
                    .get(unit_id)
                    .and_then(|profile| profile.owner_zone)
                    .unwrap_or(TubeZone::TopFace);
                zone_groups[zone.index()].units.push(*unit_id);
            }

            groups.push(MachiningGroup {
                id: groups.len() as u32,
                x_start,
                x_end,
                section_unit,
                units,
                zone_groups,
            });
        }
        self.groups = groups;
    }
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
        unit_id: None,
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

fn unit_yz_points(display: &DisplayData, cam: &CamModel, unit_id: UnitId) -> Vec<(f64, f64)> {
    let mut points = Vec::new();
    for path in &display.paths {
        if cam.unit_for_entity(path.entity_id) != Some(unit_id) {
            continue;
        }
        for point in &path.points {
            points.push((point[1], point[2]));
        }
    }
    points
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
        let score = 1.0 / (1.0 + max_deviation + coverage_gap_bins as f64);
        candidates.push(MachiningSectionCandidate {
            unit_id: unit.id,
            x_min,
            x_max,
            winding,
            maximum_deviation: max_deviation,
            coverage_gap_bins,
            zone_mask,
            score,
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
                unit_id: unit.id,
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
