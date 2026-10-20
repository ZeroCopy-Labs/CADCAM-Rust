//! Parser boundary for DXF/DWG files.
//!
//! This module only *reads* files: format detection, failsafe parser setup,
//! I/O error translation and a usability gate. It deliberately does not know
//! about `DrawingDocument`; conversion lives in `crate::convert`.
//!
//! `acadrust` is a private implementation detail of this module. The rest of
//! the crate never sees parser entity/document types.

mod error;
mod format;

use std::fs::File;
use std::io::Read;
use std::path::Path;

use acadrust::{
    CadDocument, DwgReadOptions, DwgReader, DxfReader, DxfReaderConfiguration, ReadOutcome,
    ReadStats,
};

use crate::model::FileFormat;

pub use error::ImportError;

/// Native parser result shared with the conversion layer.
///
/// The fields are crate-visible but the parser document is never exposed outside
/// cad-core.
pub(crate) struct NativeRead {
    /// Format detection result (format, diagnostics, version hint).
    pub(crate) detection: format::FormatDetection,
    /// Parser-owned document. Only the conversion module consumes it.
    pub(crate) document: CadDocument,
    /// Parser statistics used for warnings and the usability gate.
    pub(crate) stats: ReadStats,
}

/// Read a file far enough to detect its format and run the parser.
///
/// This is the only function that touches the parser readers. It performs the
/// "no usable drawing data" check before handing data to the converter.
pub(crate) fn read_native(path: &Path) -> Result<NativeRead, ImportError> {
    let header = read_header(path)?;
    if header.is_empty() {
        return Err(ImportError::EmptyFile {
            path: path.to_path_buf(),
        });
    }

    let detection = format::detect_format(path, &header)?;
    let outcome = parse_drawing(path, detection.format)?;
    let ReadOutcome { document, stats } = outcome;

    // Failsafe parsing may return a structurally valid but empty document for
    // a corrupt or non-drawing file (e.g. a truncated DWG header). Reject that
    // instead of reporting a successful read with no usable drawing data.
    if !stats.has_usable_drawing_data() {
        let detail = stats
            .diagnostics
            .first()
            .map(|diagnostic| format!("{}: {}", diagnostic.code, diagnostic.message))
            .unwrap_or_else(|| "no parse diagnostics were reported".to_string());
        return Err(ImportError::Parse {
            path: path.to_path_buf(),
            format: Some(detection.format),
            message: format!(
                "parser produced no usable drawing records ({} diagnostics; first: {detail})",
                stats.diagnostics.len()
            ),
        });
    }

    Ok(NativeRead {
        detection,
        document,
        stats,
    })
}

/// Read the leading bytes used by the format detector.
fn read_header(path: &Path) -> Result<Vec<u8>, ImportError> {
    let mut file = File::open(path).map_err(|error| ImportError::io(path, error))?;
    let mut buffer = vec![0u8; 64 * 1024];
    let read = file
        .read(&mut buffer)
        .map_err(|error| ImportError::io(path, error))?;
    buffer.truncate(read);
    Ok(buffer)
}

/// Dispatch to the DXF or DWG reader in failsafe mode.
fn parse_drawing(path: &Path, format: FileFormat) -> Result<ReadOutcome, ImportError> {
    match format {
        FileFormat::Dwg => {
            let mut reader = DwgReader::from_file_with_options(path, DwgReadOptions::failsafe())
                .map_err(|error| map_parser_error(path, Some(format), error))?;
            reader
                .read_with_stats()
                .map_err(|error| map_parser_error(path, Some(format), error))
        }
        FileFormat::AsciiDxf | FileFormat::BinaryDxf => {
            let configuration = DxfReaderConfiguration {
                failsafe: true,
                default_encoding: None,
            };
            DxfReader::from_file(path)
                .map_err(|error| map_parser_error(path, Some(format), error))?
                .with_configuration(configuration)
                .read_with_stats()
                .map_err(|error| map_parser_error(path, Some(format), error))
        }
    }
}

/// Translate parser errors into the crate's stable import error type.
fn map_parser_error(
    path: &Path,
    format: Option<FileFormat>,
    error: acadrust::DxfError,
) -> ImportError {
    use acadrust::DxfError;

    match error {
        DxfError::Io(source) => ImportError::Io {
            path: path.to_path_buf(),
            source,
        },
        DxfError::UnsupportedVersion(version) => ImportError::UnsupportedVersion {
            path: path.to_path_buf(),
            version,
        },
        other => ImportError::Parse {
            path: path.to_path_buf(),
            format,
            message: other.to_string(),
        },
    }
}
