//! Versioned, self-contained CAD/CAM project files. Derived geometry is never serialized.

use crate::{cam_analysis::MachiningSetup, gcode::GCodeProfile, ViewerState};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeSet,
    path::{Path, PathBuf},
};

#[derive(Serialize, Deserialize)]
struct Project {
    format: String,
    version: u32,
    drawing: cad_core::ProjectDrawing,
    machining: MachiningSetup,
    profile: GCodeProfile,
    excluded: BTreeSet<u32>,
}

struct Checkpoint {
    revision: u64,
    source: cad_core::SourceInfo,
    machining: MachiningSetup,
    profile: GCodeProfile,
    excluded: BTreeSet<u32>,
}

#[derive(Default)]
pub struct ProjectState {
    pub path: Option<PathBuf>,
    pub native_project: bool,
    saved: Option<Checkpoint>,
}

fn checkpoint(state: &ViewerState) -> Option<Checkpoint> {
    state.drawing.as_ref().map(|d| Checkpoint {
        revision: d.revision(),
        source: d.document().source.clone(),
        machining: state.analysis.setup(),
        profile: state.gcode_profile.clone(),
        excluded: state.excluded.clone(),
    })
}

pub fn reset(state: &mut ViewerState) {
    state.project = ProjectState {
        saved: checkpoint(state),
        ..Default::default()
    };
}

pub fn machining_modified(state: &ViewerState) -> bool {
    state.drawing.is_some()
        && state.project.saved.as_ref().is_none_or(|s| {
            s.machining != state.analysis.setup()
                || s.profile != state.gcode_profile
                || s.excluded != state.excluded
        })
}

pub fn modified(state: &ViewerState) -> bool {
    state.drawing.is_some()
        && (machining_modified(state)
            || state.project.saved.as_ref().is_none_or(|s| {
                state
                    .drawing
                    .as_ref()
                    .is_none_or(|d| s.revision != d.revision() || s.source != d.document().source)
            }))
}

pub fn save(state: &mut ViewerState, save_as: bool) -> bool {
    if state.drawing.is_some() && !state.derived.sources_current() {
        crate::cad_jobs::ensure(state, crate::cad_jobs::Follow::SaveProject(save_as));
        return false;
    }
    let Some(drawing) = state.drawing.as_ref() else {
        return false;
    };
    let path = if save_as || state.project.path.is_none() {
        let name = Path::new(&state.file_name).with_extension("cadcam");
        let Some(path) = rfd::FileDialog::new()
            .set_title("保存 CAD/CAM 项目")
            .add_filter("CAD/CAM 项目", &["cadcam"])
            .set_directory(crate::dialog_directory(state))
            .set_file_name(name.to_string_lossy())
            .save_file()
        else {
            return false;
        };
        path
    } else {
        state.project.path.clone().unwrap()
    };
    let result = (|| {
        if !path
            .extension()
            .is_some_and(|e| e.eq_ignore_ascii_case("cadcam"))
        {
            return Err("项目文件必须使用 .cadcam 扩展名".to_string());
        }
        let project = Project {
            format: "CADCAM-Rust".into(),
            version: 1,
            drawing: drawing.project_snapshot()?,
            machining: state.analysis.setup(),
            profile: state.gcode_profile.clone(),
            excluded: state.excluded.clone(),
        };
        let bytes = serde_json::to_vec_pretty(&project).map_err(|e| e.to_string())?;
        drop(project);
        // Reject a snapshot that cannot restore its formal values/identities before replacing a file.
        let verified: Project = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
        cad_core::EditableDrawing::from_project(verified.drawing)?;
        cad_core::write_file(&path, &bytes, true).map_err(|e| e.to_string())
    })();
    match result {
        Ok(()) => {
            state.project = ProjectState {
                path: Some(path.clone()),
                native_project: true,
                saved: checkpoint(state),
            };
            crate::remember_recent(state, &std::fs::canonicalize(&path).unwrap_or(path.clone()));
            crate::push_message(
                state,
                format!(
                    "已保存项目：{}；图纸、安排及工艺快照已保存，DXF 和全局配置文件保持独立",
                    path.display()
                ),
            );
            true
        }
        Err(error) => {
            crate::push_message(state, format!("项目保存失败：{error}"));
            false
        }
    }
}

pub struct ProjectCandidate {
    pub drawing: cad_core::EditableDrawing,
    pub profile: GCodeProfile,
    pub excluded: BTreeSet<u32>,
    pub setup: MachiningSetup,
    pub source: Option<PathBuf>,
    pub migrated: bool,
}
pub fn read(path: &Path) -> Result<ProjectCandidate, String> {
    let project: Project = serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    if project.format != "CADCAM-Rust" || project.version != 1 {
        return Err("不支持此项目格式或版本，当前工作已保留".to_string());
    }
    let known: BTreeSet<_> = project.drawing.ids.iter().map(|(_, id)| *id).collect();
    if project.excluded.iter().any(|id| !known.contains(id)) {
        return Err("项目参考对象包含未知图元 ID".to_string());
    }
    let source = project.drawing.source.file_path.clone();
    let previous_count = project.drawing.entities.len();
    let drawing = cad_core::EditableDrawing::from_project(project.drawing)?;
    let migrated = drawing.document().entities.len() != previous_count;
    let mut excluded = project.excluded;
    excluded.extend(
        drawing
            .document()
            .entities
            .iter()
            .filter(|e| matches!(e.geometry, cad_core::Geometry::Text(_)))
            .map(|e| e.id.0),
    );
    Ok(ProjectCandidate {
        drawing,
        profile: project.profile,
        excluded,
        setup: project.machining,
        source,
        migrated,
    })
}

pub fn finish_open(
    state: &mut ViewerState,
    path: &Path,
    source: Option<PathBuf>,
    migrated: bool,
    different_profile: bool,
) {
    let normal = path
        .extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("cadcam"));
    state.project = ProjectState {
        path: normal.then(|| path.to_path_buf()),
        native_project: true,
        saved: (normal && !migrated).then(|| checkpoint(state)).flatten(),
    };
    crate::remember_recent(
        state,
        &std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf()),
    );
    crate::push_message(
        state,
        format!(
            "已打开项目：{}；加工数据已按内嵌图纸重算，撤销历史从本次打开开始",
            path.display()
        ),
    );
    if let Some(source) = source.filter(|p| !p.is_file()) {
        crate::push_message(
            state,
            format!("原始来源不存在：{}；当前使用项目内嵌图纸", source.display()),
        );
    }
    if different_profile {
        crate::push_message(
            state,
            "已采用项目内嵌工艺配置，与打开前的会话配置不同；未覆盖全局 gcode.toml",
        );
    }
    if !normal {
        crate::push_message(state, "已恢复项目备份，请另存为新的 .cadcam 项目");
    }
}
