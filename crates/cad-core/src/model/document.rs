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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
}

/// Source information of an imported drawing.
#[derive(Debug, Clone, PartialEq)]
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
#[derive(Debug, Clone, PartialEq)]
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

/// Summary of what the import converter did.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ImportReport {
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
    /// Drawing unit from `$INSUNITS`; `None` when absent or unknown.
    pub units: Option<LengthUnit>,
    /// Layers in internal-id order.
    pub layers: Vec<Layer>,
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
        self.layers.get(id.0 as usize)
    }

    /// Look up an entity by internal id (ids are sequential in this version).
    pub fn entity(&self, id: EntityId) -> Option<&Entity> {
        self.entities.get(id.0 as usize)
    }
}
