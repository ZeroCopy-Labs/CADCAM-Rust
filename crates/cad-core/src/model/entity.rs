//! Entity types of the internal CAD model.
//!
//! An entity is a stable identity plus common display attributes and exactly one
//! geometry value. Editing state, selection state and CAM state deliberately do
//! not live here.

use super::geometry::Geometry;
use super::ids::{EntityId, LayerId};

/// Color of an entity or a layer.
///
/// `ByLayer` and `ByBlock` only make sense for entity colors; layers normally
/// carry a concrete ACI/RGB color.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Color {
    /// Inherit the color from the entity's layer.
    ByLayer,
    /// Inherit the color from the containing block reference.
    ByBlock,
    /// Color explicitly set to "none".
    None,
    /// AutoCAD Color Index.
    Aci(u8),
    /// 24-bit true color.
    Rgb { r: u8, g: u8, b: u8 },
}

/// Alias used by layer colors; layers normally use concrete ACI/RGB values.
pub type LayerColor = Color;

/// Alias used by entity colors; ByLayer/ByBlock are only meaningful here.
pub type EntityColor = Color;

/// One entity of the internal document.
///
/// Common properties (identity, layer, color) are managed here; `geometry` only
/// describes the geometric meaning and carries no render/selection/CAM state.
#[derive(Debug, Clone, PartialEq)]
pub struct Entity {
    /// Default reference role retained in this application's DXF XData.
    pub reference: bool,
    /// Stable internal identity. Not a DXF/DWG handle.
    pub id: EntityId,
    /// Native handle in the current editable drawing.
    pub native_handle: u64,
    pub visible: bool,
    /// Layer this entity belongs to.
    pub layer_id: LayerId,
    /// Display color, possibly resolved through the layer.
    pub color: EntityColor,
    /// One of the six supported geometry values.
    pub geometry: Geometry,
}
