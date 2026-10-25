//! Stable internal identifiers.
//!
//! These newtypes keep entity/layer identities distinct at the type level and
//! prevent parser handles from leaking into business code. They are assigned by
//! the importer, not read from the file.

/// Identifier of an entity inside a [`DrawingDocument`](crate::model::DrawingDocument).
///
/// This is an internal identifier, not a DXF/DWG handle. For the current
/// version it is assigned sequentially on import. Editing preserves it, so after
/// deletion it need not equal an index in `DrawingDocument::entities`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EntityId(
    /// Stable zero-based identity assigned on import.
    pub u32,
);

/// Identifier of a layer inside a [`DrawingDocument`](crate::model::DrawingDocument).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct LayerId(
    /// Stable identity preserved across edits, undo and redo; not an array index.
    pub u32,
);
