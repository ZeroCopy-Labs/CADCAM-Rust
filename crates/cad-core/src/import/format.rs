//! DXF/DWG format detection.
//!
//! Detection uses file signatures and content heuristics; the extension is only
//! used as a fallback and for mismatch diagnostics.
use std::path::Path;

use crate::model::FileFormat;

use super::error::ImportError;

/// Internal result of signature/content based format detection.
///
/// Only the information consumed by the reader/converter is retained; detailed
/// "how we recognized it" strings were useful for the old inspection CLI but are
/// no longer part of the public API.
pub(crate) struct FormatDetection {
    pub format: FileFormat,
    /// Version string taken from the DWG file header (`AC10xx`), when present.
    pub magic_version: Option<String>,
    /// Diagnostic when the file extension and the detected content disagree.
    pub extension_mismatch: Option<String>,
}

const BINARY_DXF_SENTINEL: &[u8] = b"AutoCAD Binary DXF\r\n\x1a\0";
const DXF_SECTION_MARKERS: [&str; 6] = [
    "HEADER", "CLASSES", "TABLES", "BLOCKS", "ENTITIES", "OBJECTS",
];

/// Detect the CAD format from content signatures, using the extension only as a
/// secondary signal (and to report mismatches).
pub(crate) fn detect_format(path: &Path, bytes: &[u8]) -> Result<FormatDetection, ImportError> {
    let extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .map(|extension| extension.to_ascii_lowercase());

    if let Some(version) = dwg_version_from_header(bytes) {
        return Ok(FormatDetection {
            format: FileFormat::Dwg,
            magic_version: Some(version),
            extension_mismatch: dwg_extension_mismatch(extension.as_deref()),
        });
    }

    if bytes.starts_with(BINARY_DXF_SENTINEL) {
        return Ok(FormatDetection {
            format: FileFormat::BinaryDxf,
            magic_version: None,
            extension_mismatch: dxf_extension_mismatch(extension.as_deref(), FileFormat::BinaryDxf),
        });
    }

    if looks_like_ascii_dxf(bytes) {
        return Ok(FormatDetection {
            format: FileFormat::AsciiDxf,
            magic_version: None,
            extension_mismatch: dxf_extension_mismatch(extension.as_deref(), FileFormat::AsciiDxf),
        });
    }

    // Fallback: an explicit .dxf extension plus text-like content is handed to
    // the DXF parser so that it can report the concrete parsing problem.
    if extension.as_deref() == Some("dxf") && is_mostly_text(bytes) {
        return Ok(FormatDetection {
            format: FileFormat::AsciiDxf,
            magic_version: None,
            extension_mismatch: None,
        });
    }

    Err(ImportError::UnrecognizedFormat {
        path: path.to_path_buf(),
        detail: describe_unrecognized(bytes, extension.as_deref()),
    })
}

fn dwg_version_from_header(bytes: &[u8]) -> Option<String> {
    if bytes.len() < 6 {
        return None;
    }
    let code = &bytes[..6];
    let looks_like_dwg = &code[..2] == b"AC"
        && &code[2..4] == b"10"
        && code[4].is_ascii_digit()
        && code[5].is_ascii_digit();
    if !looks_like_dwg {
        return None;
    }
    std::str::from_utf8(code).ok().map(str::to_string)
}

fn looks_like_ascii_dxf(bytes: &[u8]) -> bool {
    let sample = &bytes[..bytes.len().min(64 * 1024)];
    if sample.contains(&0) {
        return false;
    }

    let text = String::from_utf8_lossy(sample);
    let text = text.trim_start_matches('\u{feff}').trim_start();
    if text.is_empty() {
        return false;
    }

    let upper = text.to_ascii_uppercase();
    upper.contains("SECTION")
        && DXF_SECTION_MARKERS
            .iter()
            .any(|marker| upper.contains(marker))
}

fn is_mostly_text(bytes: &[u8]) -> bool {
    let sample = &bytes[..bytes.len().min(8 * 1024)];
    if sample.is_empty() || sample.contains(&0) {
        return false;
    }

    let disallowed = sample
        .iter()
        .filter(|&&byte| byte < 0x20 && byte != b'\r' && byte != b'\n' && byte != b'\t')
        .count();
    disallowed * 20 <= sample.len()
}

fn dwg_extension_mismatch(extension: Option<&str>) -> Option<String> {
    match extension {
        Some("dxf") => Some(
            "file extension is .dxf but the content starts with a DWG signature; parsed as DWG"
                .to_string(),
        ),
        _ => None,
    }
}

fn dxf_extension_mismatch(extension: Option<&str>, detected: FileFormat) -> Option<String> {
    match extension {
        Some("dwg") => Some(format!(
            "file extension is .dwg but the content is {detected}; parsed as {detected}"
        )),
        _ => None,
    }
}

fn describe_unrecognized(bytes: &[u8], extension: Option<&str>) -> String {
    let extension = extension
        .map(|extension| format!(".{extension}"))
        .unwrap_or_else(|| "no extension".to_string());
    let prefix: Vec<String> = bytes
        .iter()
        .take(16)
        .map(|byte| format!("{byte:02X}"))
        .collect();

    if bytes.contains(&0) {
        format!(
            "binary content ({extension}, first bytes: {})",
            prefix.join(" ")
        )
    } else {
        format!("text content ({extension}) without DXF section markers")
    }
}
