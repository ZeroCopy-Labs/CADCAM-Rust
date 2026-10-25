//! `cad-core`: DXF/DWG import and the internal CAD document model.
//!
//! The crate separates parsing, conversion, editing and domain data:
//!
//! - `import` (private): detect the file format, run the third-party parser in
//!   failsafe mode and reject unusable input;
//! - `convert` (private): translate parser entities into the internal model and
//!   expose [`read_document`];
//! - [`model`]: the parser-independent document/entity/geometry data model.
//! - `edit` (private): retain native records, apply undoable batch edits and export DXF.
//!
//! [`read_document`] returns the model; [`read_editable_document`] retains an
//! editable native document behind [`EditableDrawing`]. Parser types remain private.

mod convert;
mod edit;
mod file_io;
mod import;

pub mod model;

pub use convert::{aci_color_rgb, read_document, read_editable_document};
pub use edit::{DrawingChange, DrawingEdit, EditableDrawing};
pub use import::ImportError;
pub use model::{
    DrawingDocument, Entity, EntityId, FileFormat, Geometry, ImportReport, Layer, LayerId,
    LengthUnit, SkipReason, SkippedEntity, SourceInfo,
};
