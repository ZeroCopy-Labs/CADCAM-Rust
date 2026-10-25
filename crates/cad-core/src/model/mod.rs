//! Internal CAD document and entity model.
//!
//! This module is independent of any particular file parser. It contains only
//! the semantics required by the current phase: a flat entity list with layers,
//! common entity attributes and six geometry kinds.
//!
//! It is deliberately **not** a full AutoCAD-style document model: block
//! definitions, annotations, hatches, dictionaries and 3D solids are not
//! represented yet.
//!
//! File layout:
//! - `ids.rs` — stable internal identifiers;
//! - `entity.rs` — entity common attributes and colors;
//! - `geometry.rs` — value-only geometric primitives;
//! - `document.rs` — the aggregate document and import report.

mod document;
mod entity;
mod geometry;
mod ids;
mod work_plane;

// Re-export the complete model surface so callers do not depend on file layout.
pub use document::{
    BlockExpansionReport, BlockMapping, DrawingDocument, FileFormat, ImportReport, InsertSource,
    Layer, LengthUnit, SkipReason, SkippedEntity, SourceInfo, MAX_SKIPPED_SAMPLES,
};
pub use entity::{Color, Entity, EntityColor, LayerColor};
pub use geometry::{
    Arc, Circle, Ellipse, Geometry, Line, Plane, Point3, Polyline, PolylineSegment, Spline, Text,
    Vector3, GEOMETRY_EPSILON,
};
pub use ids::{EntityId, LayerId};
pub use work_plane::WorkPlane;
