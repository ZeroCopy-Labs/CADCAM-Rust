//! Errors returned by the DXF/DWG import boundary.
//!
//! Parser-specific error types are translated here so callers do not depend on
//! the third-party parser.
use std::fmt;
use std::io;
use std::path::{Path, PathBuf};

use crate::model::FileFormat;

/// Errors that can occur while importing a DXF/DWG drawing.
#[derive(Debug)]
pub enum ImportError {
    /// The file could not be opened or read from disk.
    Io { path: PathBuf, source: io::Error },

    /// The file exists but contains no data.
    EmptyFile { path: PathBuf },

    /// The file content does not look like a supported CAD file format.
    UnrecognizedFormat { path: PathBuf, detail: String },

    /// The file was recognized, but its CAD version is not supported.
    UnsupportedVersion { path: PathBuf, version: String },

    /// The file was recognized, but parsing failed.
    Parse {
        path: PathBuf,
        format: Option<FileFormat>,
        message: String,
    },
}

impl ImportError {
    pub(crate) fn io(path: &Path, source: io::Error) -> Self {
        Self::Io {
            path: path.to_path_buf(),
            source,
        }
    }
}

impl fmt::Display for ImportError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, source } => {
                write!(formatter, "{}: I/O error: {source}", path.display())
            }
            Self::EmptyFile { path } => {
                write!(formatter, "{}: file is empty", path.display())
            }
            Self::UnrecognizedFormat { path, detail } => write!(
                formatter,
                "{}: unsupported or unrecognized CAD file format ({detail})",
                path.display()
            ),
            Self::UnsupportedVersion { path, version } => write!(
                formatter,
                "{}: unsupported CAD version: {version}",
                path.display()
            ),
            Self::Parse {
                path,
                format,
                message,
            } => match format {
                Some(format) => write!(
                    formatter,
                    "{}: failed to parse {format}: {message}",
                    path.display()
                ),
                None => write!(
                    formatter,
                    "{}: failed to parse drawing: {message}",
                    path.display()
                ),
            },
        }
    }
}

impl std::error::Error for ImportError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}
