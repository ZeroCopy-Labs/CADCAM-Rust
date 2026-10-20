//! Stable internal identifiers.
//!
//! These newtypes keep entity/layer identities distinct at the type level and
//! prevent parser handles from leaking into business code. They are assigned by
//! the importer, not read from the file.

/// Identifier of an entity inside a [`DrawingDocument`](crate::model::DrawingDocument).
///
/// This is an internal identifier, not a DXF/DWG handle. For the current
/// version it is assigned sequentially and is equal to the entity's index in
/// `DrawingDocument::entities`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EntityId(
    /// Zero-based index into `DrawingDocument::entities`.
    pub u32,
);

/// Identifier of a layer inside a [`DrawingDocument`](crate::model::DrawingDocument).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct LayerId(
    /// Zero-based index into `DrawingDocument::layers`.
    pub u32,
);
