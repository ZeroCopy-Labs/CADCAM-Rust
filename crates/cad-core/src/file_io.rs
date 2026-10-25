//! Replace a file only after its sibling temporary file has been fully written.

use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};

static TEMP_SERIAL: AtomicU64 = AtomicU64::new(0);

fn replace(path: &Path, write: impl FnOnce(&mut File) -> io::Result<()>) -> io::Result<()> {
    let name = path
        .file_name()
        .ok_or_else(|| io::Error::other("目标文件名为空"))?;
    let mut temporary_name = name.to_os_string();
    temporary_name.push(format!(
        ".{}.{}.tmp",
        std::process::id(),
        TEMP_SERIAL.fetch_add(1, Ordering::Relaxed)
    ));
    let temporary = path.with_file_name(temporary_name);
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&temporary)?;
    let result = (|| {
        write(&mut file)?;
        file.sync_all()?;
        drop(file);
        fs::rename(&temporary, path)
    })();
    if result.is_err() {
        let _ = fs::remove_file(&temporary);
    }
    result
}

pub(crate) fn write_dxf(path: &Path, bytes: &[u8], backup: bool) -> io::Result<()> {
    write_file(path, bytes, backup)
}

/// Write a complete snapshot using a sibling temporary file and optional backup.
pub fn write_file(path: &Path, bytes: &[u8], backup: bool) -> io::Result<()> {
    if backup && path.try_exists()? {
        let mut name = path
            .file_name()
            .ok_or_else(|| io::Error::other("目标文件名为空"))?
            .to_os_string();
        name.push(".bak");
        replace(&path.with_file_name(name), |output| {
            io::copy(&mut File::open(path)?, output).map(|_| ())
        })?;
    }
    replace(path, |output| output.write_all(bytes))
}
