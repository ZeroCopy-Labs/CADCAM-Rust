//! Internal drawing document and import report.
//!
//! The document aggregates parser-independent data: source information, units,
//! layers, entities and a report of what the importer did. It intentionally does
//! not contain block definitions, layouts or editing history yet.

use std::collections::BTreeMap;
use std::fmt;
use std::path::PathBuf;

use super::entity::{Entity, LayerColor};
use super::ids::{EntityId, LayerId};

/// The on-disk CAD format a document was read from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum FileFormat {
    /// Text-based DXF.
    AsciiDxf,
    /// Binary DXF (starts with the `AutoCAD Binary DXF` sentinel).
    BinaryDxf,
    /// Native binary DWG.
    Dwg,
}

impl FileFormat {
    /// Whether this is a DXF variant.
    pub fn is_dxf(self) -> bool {
        matches!(self, Self::AsciiDxf | Self::BinaryDxf)
    }
}

impl fmt::Display for FileFormat {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AsciiDxf => formatter.write_str("ASCII DXF"),
            Self::BinaryDxf => formatter.write_str("Binary DXF"),
            Self::Dwg => formatter.write_str("DWG"),
        }
    }
}

/// Drawing length unit, based on the DXF `$INSUNITS` value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LengthUnit {
    /// Inches.
    Inches,
    /// Feet.
    Feet,
    /// Miles.
    Miles,
    /// Millimeters (most common for mechanical drawings).
    Millimeters,
    /// Centimeters.
    Centimeters,
    /// Meters.
    Meters,
    /// Kilometers.
    Kilometers,
    /// Microinches.
    Microinches,
    /// Mils (thousandths of an inch).
    Mils,
    /// Yards.
    Yards,
    /// Angstroms.
    Angstroms,
    /// Nanometers.
    Nanometers,
    /// Microns.
    Microns,
    /// Decimeters.
    Decimeters,
    /// Decameters.
    Decameters,
    /// Hectometers.
    Hectometers,
    /// Gigameters.
    Gigameters,
    /// Astronomical units.
    AstronomicalUnits,
    /// Light years.
    LightYears,
    /// Parsecs.
    Parsecs,
    /// US survey feet.
    UsSurveyFeet,
    /// US survey inches.
    UsSurveyInch,
    /// US survey yards.
    UsSurveyYard,
    /// US survey miles.
    UsSurveyMile,
}

impl LengthUnit {
    /// Conversion from source coordinates to the application's millimeter workspace.
    pub fn millimeters_per_unit(self) -> f64 {
        match self {
            Self::Inches => 25.4,
            Self::Feet => 304.8,
            Self::Miles => 1_609_344.0,
            Self::Millimeters => 1.0,
            Self::Centimeters => 10.0,
            Self::Meters => 1_000.0,
            Self::Kilometers => 1_000_000.0,
            Self::Microinches => 0.000_025_4,
            Self::Mils => 0.0254,
            Self::Yards => 914.4,
            Self::Angstroms => 1e-7,
            Self::Nanometers => 1e-6,
            Self::Microns => 0.001,
            Self::Decimeters => 100.0,
            Self::Decameters => 10_000.0,
            Self::Hectometers => 100_000.0,
            Self::Gigameters => 1e12,
            Self::AstronomicalUnits => 149_597_870_700_000.0,
            Self::LightYears => 9.460_730_472_580_8e18,
            Self::Parsecs => 3.085_677_581_491_367e19,
            Self::UsSurveyFeet => 1_200_000.0 / 3937.0,
            Self::UsSurveyInch => 100_000.0 / 3937.0,
            Self::UsSurveyYard => 3_600_000.0 / 3937.0,
            Self::UsSurveyMile => 6_336_000_000.0 / 3937.0,
        }
    }
}

impl fmt::Display for LengthUnit {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let symbol = match self {
            Self::Inches => "in",
            Self::Feet => "ft",
            Self::Miles => "mi",
            Self::Millimeters => "mm",
            Self::Centimeters => "cm",
            Self::Meters => "m",
            Self::Kilometers => "km",
            Self::Microinches => "uin",
            Self::Mils => "mil",
            Self::Yards => "yd",
            Self::Angstroms => "angstrom",
            Self::Nanometers => "nm",
            Self::Microns => "um",
            Self::Decimeters => "dm",
            Self::Decameters => "dam",
            Self::Hectometers => "hm",
            Self::Gigameters => "Gm",
            Self::AstronomicalUnits => "AU",
            Self::LightYears => "ly",
            Self::Parsecs => "pc",
            Self::UsSurveyFeet => "ftUS",
            Self::UsSurveyInch => "inUS",
            Self::UsSurveyYard => "ydUS",
            Self::UsSurveyMile => "miUS",
        };
        formatter.write_str(symbol)
    }
}

/// A layer of the internal document.
#[derive(Debug, Clone, PartialEq)]
pub struct Layer {
    /// Stable internal layer identity.
    pub id: LayerId,
    /// Layer name as read from the drawing.
    pub name: String,
    /// Layer display color.
    pub color: LayerColor,
    /// Off or frozen layers remain in CAM and DXF, but are not displayed.
    pub visible: bool,
    pub locked: bool,
}

/// Source information of an imported drawing.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SourceInfo {
    /// File name (without the directory).
    pub file_name: String,
    /// Original path when available; may be `None` for constructed documents.
    pub file_path: Option<PathBuf>,
    /// Detected on-disk format.
    pub format: FileFormat,
    /// Drawing version (`AC10xx`, or the DWG header version).
    pub version: String,
}

/// Why an imported entity was not represented in the internal model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SkipReason {
    /// Intentionally ignored (for example an entity outside model space).
    Ignored,
    /// Entity type / subtype is not supported by the internal model yet.
    Unsupported,
    /// Entity was supported in principle, but its data could not be converted.
    ConversionFailed,
}

impl fmt::Display for SkipReason {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Ignored => formatter.write_str("ignored"),
            Self::Unsupported => formatter.write_str("unsupported"),
            Self::ConversionFailed => formatter.write_str("conversion failed"),
        }
    }
}

/// A sample of an entity that was not converted.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SkippedEntity {
    /// Original DXF/DWG type name, e.g. `"TEXT"` or `"ACAD_PROXY_ENTITY"`.
    pub source_type: String,
    /// Layer name from the source entity (not the internal layer id).
    pub layer: String,
    /// Original file handle, for tracing only.
    pub handle: Option<u64>,
    /// Broad category of why the entity was skipped.
    pub reason: SkipReason,
    /// Human-readable detail for diagnostics.
    pub message: String,
}

/// Maximum number of individual skipped-entity samples kept in the report.
pub const MAX_SKIPPED_SAMPLES: usize = 256;

/// One original INSERT cell in an expanded instance path.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct InsertSource {
    pub handle: u64,
    pub row: u16,
    pub column: u16,
}
/// Mapping retained for the current import report, not a persistent block relationship.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BlockMapping {
    pub handle: u64,
    pub path: Vec<InsertSource>,
    pub source_handle: u64,
    pub segment: usize,
}
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BlockExpansionReport {
    pub original_entities: usize,
    pub original_model_space: usize,
    pub instances: usize,
    pub entities: usize,
    pub mappings: Vec<BlockMapping>,
    pub mappings_omitted: usize,
    pub failures: Vec<String>,
    pub failures_omitted: usize,
}

/// Summary of what the import converter did.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ImportReport {
    pub blocks: BlockExpansionReport,
    pub parser_review_required: bool,
    pub parser_skipped_records: usize,
    /// All parsed entities, including block definitions and paper space.
    pub source_entities: usize,
    /// Entities presented to the model-space converter.
    pub model_space_entities: usize,
    /// Parsed entities outside model space.
    pub ignored_non_model_space: usize,
    /// Number of successfully converted entities per internal geometry kind.
    pub converted: BTreeMap<String, usize>,
    /// Samples of entities that were ignored, unsupported or failed.
    pub skipped: Vec<SkippedEntity>,
    /// Number of skipped entities that were not kept as samples.
    pub skipped_omitted: usize,
    /// Human-readable warnings produced during conversion.
    pub warnings: Vec<String>,
}

impl ImportReport {
    pub fn has_omissions(&self) -> bool {
        self.parser_review_required
            || !self.skipped.is_empty()
            || self.skipped_omitted > 0
            || !self.blocks.failures.is_empty()
            || self.blocks.failures_omitted > 0
    }

    /// Increment the converted counter for a geometry kind.
    pub(crate) fn record_converted(&mut self, kind: &'static str) {
        *self.converted.entry(kind.to_string()).or_insert(0) += 1;
    }

    /// Record a skipped entity, keeping only the first samples.
    pub(crate) fn record_skipped(
        &mut self,
        source_type: String,
        layer: String,
        handle: Option<u64>,
        reason: SkipReason,
        message: String,
    ) {
        if self.skipped.len() < MAX_SKIPPED_SAMPLES {
            self.skipped.push(SkippedEntity {
                source_type,
                layer,
                handle,
                reason,
                message,
            });
        } else {
            self.skipped_omitted += 1;
        }
    }

    /// Append a non-fatal import warning.
    pub(crate) fn warn(&mut self, message: impl Into<String>) {
        self.warnings.push(message.into());
    }
}

/// The program's own CAD document.
///
/// This is not the parser's document type: it is a parser-independent model
/// built by the import module.
#[derive(Debug, Clone, PartialEq)]
pub struct DrawingDocument {
    /// Workspace unit: millimeters after source units have been resolved.
    pub units: Option<LengthUnit>,
    /// Source coordinates' unit, retained for native editing and DXF output.
    pub source_units: Option<LengthUnit>,
    /// Current layers; stable IDs need not match their array positions.
    pub layers: Vec<Layer>,
    /// Native current layer, used for new geometry.
    pub current_layer: Option<LayerId>,
    /// Model-space entities only, in conversion order.
    pub entities: Vec<Entity>,
    /// File/source metadata.
    pub source: SourceInfo,
    /// What the importer converted, skipped or warned about.
    pub import_report: ImportReport,
}

impl DrawingDocument {
    /// Number of entities in the internal model.
    pub fn entity_count(&self) -> usize {
        self.entities.len()
    }

    /// Look up a layer by internal id.
    pub fn layer(&self, id: LayerId) -> Option<&Layer> {
        self.layers.iter().find(|layer| layer.id == id)
    }

    /// Look up a stable entity id; deletion can leave gaps in the id sequence.
    pub fn entity(&self, id: EntityId) -> Option<&Entity> {
        self.entities.iter().find(|entity| entity.id == id)
    }
}
