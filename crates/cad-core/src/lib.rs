//! `cad-core`: DXF/DWG import and the internal CAD document model.
//!
//! The crate is split into three responsibilities:
//!
//! - `import` (private): detect the file format, run the third-party parser in
//!   failsafe mode and reject unusable input;
//! - `convert` (private): translate parser entities into the internal model and
//!   expose [`read_document`];
//! - [`model`]: the parser-independent document/entity/geometry data model.
//!
//! The only public read entry point is [`read_document`]. Parser types never
//! leave the crate.

mod convert;
mod import;

pub mod model;

pub use convert::read_document;
pub use import::ImportError;
pub use model::{
    DrawingDocument, Entity, EntityId, FileFormat, Geometry, ImportReport, Layer, LayerId,
    LengthUnit, SkipReason, SkippedEntity, SourceInfo,
};
