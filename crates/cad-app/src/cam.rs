//! Derived CAM model: group CAD entities into continuous machining units.
//!
//! This module only groups topology. It does not modify the CAD document and
//! does not generate machining plans or toolpaths yet.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use cad_core::model::{DrawingDocument, Geometry, Plane, Point3, PolylineSegment};

/// Model-space tolerance used when deciding whether two entity endpoints touch.
pub const JOIN_TOLERANCE: f64 = 1.0e-3;

/// Stable identity of a machining unit.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub struct UnitId(pub u64);

/// One group of topologically continuous entities.
#[derive(Clone, Debug)]
pub struct MachiningUnit {
    pub id: UnitId,
    /// Sorted entity IDs. The member order is intentionally not a machining
    /// direction yet; direction/order belongs to the later CAM planning stage.
    pub members: Vec<u32>,
    /// A closed loop, or a standalone full-circle/full-ellipse/closed spline.
    pub closed: bool,
}

/// Derived CAM topology for the current CAD document.
#[derive(Clone, Debug, Default)]
pub struct CamModel {
    pub units: Vec<MachiningUnit>,
    pub entity_to_unit: BTreeMap<u32, UnitId>,
}

impl CamModel {
    pub fn unit(&self, id: UnitId) -> Option<&MachiningUnit> {
        self.units.iter().find(|unit| unit.id == id)
    }

    pub fn unit_for_entity(&self, entity_id: u32) -> Option<UnitId> {
        self.entity_to_unit.get(&entity_id).copied()
    }
}

/// Build machining units from endpoint continuity.
///
/// Open entities connect when an endpoint lies within [`JOIN_TOLERANCE`].
/// Full circles, closed polylines, full ellipses and closed splines are always
/// standalone units because they have no endpoints to join.
pub fn build_cam_model(document: &DrawingDocument, excluded: &BTreeSet<u32>) -> CamModel {
    let mut endpoints = EndpointIndex::new(JOIN_TOLERANCE);
    let mut union_find = UnionFind::default();
    let mut open_records: Vec<OpenRecord> = Vec::new();
    let mut closed_entities: Vec<u32> = Vec::new();

    for entity in &document.entities {
        if excluded.contains(&entity.id.0) {
            continue;
        }
        let entity_id = entity.id.0;
        match entity_topology(&entity.geometry) {
            Some(Topology::Closed) => closed_entities.push(entity_id),
            Some(Topology::Open { start, end }) => {
                let start_node = endpoints.intern(start);
                let end_node = endpoints.intern(end);
                union_find.ensure(start_node);
                union_find.ensure(end_node);
                union_find.union(start_node, end_node);
                open_records.push(OpenRecord {
                    entity_id,
                    start_node,
                    end_node,
                });
            }
            None => {}
        }
    }

    let mut groups: BTreeMap<usize, Vec<OpenRecord>> = BTreeMap::new();
    for record in open_records {
        let root = union_find.find(record.start_node);
        groups.entry(root).or_default().push(record);
    }

    let mut units: Vec<MachiningUnit> = Vec::new();

    for records in groups.into_values() {
        let mut members: Vec<u32> = records.iter().map(|record| record.entity_id).collect();
        members.sort_unstable();

        let mut node_degrees: BTreeMap<usize, u32> = BTreeMap::new();
        for record in &records {
            *node_degrees.entry(record.start_node).or_default() += 1;
            *node_degrees.entry(record.end_node).or_default() += 1;
        }
        let closed = !node_degrees.is_empty() && node_degrees.values().all(|degree| *degree == 2);

        units.push(MachiningUnit {
            id: unit_id(&members),
            members,
            closed,
        });
    }

    for entity_id in closed_entities {
        let members = vec![entity_id];
        units.push(MachiningUnit {
            id: unit_id(&members),
            members,
            closed: true,
        });
    }

    units.sort_by_key(|unit| unit.members.first().copied().unwrap_or(u32::MAX));

    let mut entity_to_unit = BTreeMap::new();
    for unit in &units {
        for entity_id in &unit.members {
            entity_to_unit.insert(*entity_id, unit.id);
        }
    }

    CamModel {
        units,
        entity_to_unit,
    }
}

fn unit_id(members: &[u32]) -> UnitId {
    // FNV-1a over the canonical sorted member set. This keeps the identity
    // stable across reloads and independent of display indices.
    let mut hash = 14695981039346656037u64;
    for entity_id in members {
        hash ^= u64::from(*entity_id);
        hash = hash.wrapping_mul(1099511628211);
    }
    UnitId(hash)
}

enum Topology {
    Closed,
    Open { start: Point3, end: Point3 },
}

fn entity_topology(geometry: &Geometry) -> Option<Topology> {
    match geometry {
        Geometry::Text(_) => None,
        Geometry::Line(line) => Some(Topology::Open {
            start: line.start,
            end: line.end,
        }),
        Geometry::Polyline(polyline) => {
            if polyline.closed {
                return Some(Topology::Closed);
            }
            let first = polyline.segments.first()?;
            let last = polyline.segments.last()?;
            Some(Topology::Open {
                start: segment_start(first),
                end: segment_end(last),
            })
        }
        Geometry::Circle(_) => Some(Topology::Closed),
        Geometry::Arc(arc) => {
            let start = point_on_arc(arc.center, arc.radius, arc.start_angle, arc.plane);
            let end = point_on_arc(
                arc.center,
                arc.radius,
                arc.start_angle + arc.sweep_angle,
                arc.plane,
            );
            Some(Topology::Open { start, end })
        }
        Geometry::Ellipse(ellipse) => {
            if ellipse.sweep_parameter.abs() >= std::f64::consts::TAU - 1.0e-9 {
                return Some(Topology::Closed);
            }
            let start = point_on_ellipse(
                ellipse.center,
                ellipse.major_axis,
                ellipse.minor_axis,
                ellipse.start_parameter,
            );
            let end = point_on_ellipse(
                ellipse.center,
                ellipse.major_axis,
                ellipse.minor_axis,
                ellipse.start_parameter + ellipse.sweep_parameter,
            );
            Some(Topology::Open { start, end })
        }
        Geometry::Spline(spline) => {
            if spline.closed || spline.periodic {
                return Some(Topology::Closed);
            }
            let start = cad_core::curves::evaluate(geometry, 0.0).ok()?;
            let end = cad_core::curves::evaluate(geometry, 1.0).ok()?;
            Some(Topology::Open { start, end })
        }
    }
}

fn segment_start(segment: &PolylineSegment) -> Point3 {
    match segment {
        PolylineSegment::Line { start, .. } => *start,
        PolylineSegment::Arc { start, .. } => *start,
    }
}

fn segment_end(segment: &PolylineSegment) -> Point3 {
    match segment {
        PolylineSegment::Line { end, .. } => *end,
        PolylineSegment::Arc { end, .. } => *end,
    }
}

fn point_on_arc(center: Point3, radius: f64, angle: f64, plane: Plane) -> Point3 {
    let y_axis = plane.y_axis();
    let (sin, cos) = angle.sin_cos();
    Point3::new(
        center.x + radius * (cos * plane.x_axis.x + sin * y_axis.x),
        center.y + radius * (cos * plane.x_axis.y + sin * y_axis.y),
        center.z + radius * (cos * plane.x_axis.z + sin * y_axis.z),
    )
}

fn point_on_ellipse(
    center: Point3,
    major_axis: cad_core::model::Vector3,
    minor_axis: cad_core::model::Vector3,
    parameter: f64,
) -> Point3 {
    let (sin, cos) = parameter.sin_cos();
    Point3::new(
        center.x + major_axis.x * cos + minor_axis.x * sin,
        center.y + major_axis.y * cos + minor_axis.y * sin,
        center.z + major_axis.z * cos + minor_axis.z * sin,
    )
}

struct OpenRecord {
    entity_id: u32,
    start_node: usize,
    end_node: usize,
}

#[derive(Default)]
struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn ensure(&mut self, node: usize) {
        while self.parent.len() <= node {
            let next = self.parent.len();
            self.parent.push(next);
        }
    }

    fn find(&mut self, node: usize) -> usize {
        self.ensure(node);
        if self.parent[node] != node {
            self.parent[node] = self.find(self.parent[node]);
        }
        self.parent[node]
    }

    fn union(&mut self, left: usize, right: usize) {
        let left_root = self.find(left);
        let right_root = self.find(right);
        if left_root != right_root {
            self.parent[right_root] = left_root;
        }
    }
}

/// Spatial hash for endpoint welding.
struct EndpointIndex {
    tolerance: f64,
    nodes: Vec<Point3>,
    cells: HashMap<(i64, i64, i64), Vec<usize>>,
}

impl EndpointIndex {
    fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            nodes: Vec::new(),
            cells: HashMap::new(),
        }
    }

    fn intern(&mut self, point: Point3) -> usize {
        let cell = self.cell(point);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let key = (cell.0 + dx, cell.1 + dy, cell.2 + dz);
                    if let Some(indices) = self.cells.get(&key) {
                        for &index in indices {
                            if distance_squared(self.nodes[index], point)
                                <= self.tolerance * self.tolerance
                            {
                                return index;
                            }
                        }
                    }
                }
            }
        }

        let index = self.nodes.len();
        self.nodes.push(point);
        self.cells.entry(cell).or_default().push(index);
        index
    }

    fn cell(&self, point: Point3) -> (i64, i64, i64) {
        (
            (point.x / self.tolerance).floor() as i64,
            (point.y / self.tolerance).floor() as i64,
            (point.z / self.tolerance).floor() as i64,
        )
    }
}

fn distance_squared(left: Point3, right: Point3) -> f64 {
    let dx = left.x - right.x;
    let dy = left.y - right.y;
    let dz = left.z - right.z;
    dx * dx + dy * dy + dz * dz
}
