//! CAD file lifecycle, import diagnostics and recovery snapshots.

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use bevy::{app::AppExit, prelude::*, window::WindowCloseRequested};
use bevy_egui::egui;
use cad_core::{EditableDrawing, ImportReport};

use super::{cam, cam_analysis, display, push_message, remember_recent, ViewerState};

#[derive(Default)]
pub(super) struct FileState {
    pub report_open: bool,
    pub coverage_acknowledged: bool,
    pub exit_requested: bool,
    report: Option<ImportReport>,
    report_name: String,
    recovery_path: Option<PathBuf>,
    recovery_revision: Option<u64>,
    recovery_elapsed: f32,
}

pub(super) fn open_drawing(state: &mut ViewerState, raw_path: &str) {
    let path = PathBuf::from(super::normalize_path(raw_path));
    if !path.as_os_str().is_empty() {
        super::cad_jobs::open(state, path);
    }
}

pub(super) fn install_prepared(state: &mut ViewerState, prepared: super::cad_jobs::Prepared) {
    let c = prepared.candidate;
    let different = state.gcode_profile != c.profile;
    state.gcode_profile = c.profile;
    let origin = c.project;
    let path = c.path;
    let recovered = c.recovered;
    install_data(
        state,
        Some(c.drawing),
        Some((prepared.display, prepared.calculated, c.excluded)),
    );
    if let Some((source, migrated)) = origin {
        super::cad_project::finish_open(state, &path, source, migrated, different);
    } else {
        if !recovered {
            remember_recent(state, &std::fs::canonicalize(&path).unwrap_or(path));
        }
        push_message(
            state,
            format!(
                "已加载 {} 个图元、{} 个加工单元；{}",
                state.entity_count,
                state.cam.units.len(),
                state.unit
            ),
        );
        if recovered {
            push_message(state, "已恢复图纸备份，请另存 DXF；不包含加工安排或历史");
        }
    }
}

pub(super) fn install_drawing(state: &mut ViewerState, drawing: Option<EditableDrawing>) {
    install_data(state, drawing, None);
}
fn install_data(
    state: &mut ViewerState,
    drawing: Option<EditableDrawing>,
    ready: Option<(
        display::DisplayData,
        super::cad_jobs::Calculated,
        std::collections::BTreeSet<u32>,
    )>,
) {
    state.tasks.cancel();
    clear_recovery(state);
    let prepared = ready.is_some();
    let mut ready = ready;
    state.excluded = drawing
        .as_ref()
        .map(|d| {
            d.document()
                .entities
                .iter()
                .filter(|e| e.reference)
                .map(|e| e.id.0)
                .collect()
        })
        .unwrap_or_default();
    state.file_ui.coverage_acknowledged = false;
    state.file_ui.report = drawing
        .as_ref()
        .map(|drawing| drawing.document().import_report.clone());
    state.file_ui.report_open = state
        .file_ui
        .report
        .as_ref()
        .is_some_and(|report| !report.warnings.is_empty() || report.has_omissions());
    if let Some(document) = drawing.as_ref().map(EditableDrawing::document) {
        state.file_name = document.source.file_name.clone();
        state.file_ui.report_name = state.file_name.clone();
        state.format = document.source.format.to_string();
        state.version = document.source.version.clone();
        state.unit = format!(
            "工作 mm · 源 {}",
            document
                .source_units
                .map(|unit| unit.to_string())
                .unwrap_or_else(|| "未指定".into())
        );
        state.entity_count = document.entity_count();
        state.layer_count = document.layers.len();
        state.warning_count = document.import_report.warnings.len();
        if let Some((display, calculated, excluded)) = ready.take() {
            state.display = display;
            state.machining = calculated.machining;
            state.machining_error = calculated.error;
            state.cam = calculated.cam;
            state.analysis = calculated.analysis;
            state.trajectory = Some(calculated.trajectory);
            state.excluded = excluded;
        } else {
            state.display = display::build_display(document);
            match display::build_machining(document, &state.excluded) {
                Ok(data) => {
                    state.machining = data;
                    state.machining_error = None;
                }
                Err(error) => {
                    state.machining = Default::default();
                    state.machining_error = Some(error);
                }
            }
            state.cam = cam::build_cam_model(document, &state.excluded);
        }
    } else {
        state.file_name.clear();
        state.format.clear();
        state.version.clear();
        state.unit.clear();
        state.entity_count = 0;
        state.layer_count = 0;
        state.warning_count = 0;
        state.display = Default::default();
        state.machining = Default::default();
        state.machining_error = None;
        state.cam = Default::default();
    }
    if !prepared {
        state.analysis = cam_analysis::CamAnalysis::build(&state.machining, &state.cam);
        state.analysis.set_machine_start(
            &state.machining,
            &state.cam,
            [
                state.gcode_profile.initial_x,
                state.gcode_profile.initial_y,
                state.gcode_profile.initial_z,
            ],
        );
    }
    state.manual_tube = state.analysis.tube_section.clone().unwrap_or_default();
    state.manual_pair = [None, None];
    if !prepared {
        state.trajectory = None;
    }
    state.drawing = drawing;
    state.derived = Default::default();
    if let Some(drawing) = &state.drawing {
        state.derived.document_changed(drawing.revision(), true);
        state.derived.geometry_ready();
    }
    state.load_version += 1;
    if state.drawing.is_some() {
        state.derived.observe_profile(&state.gcode_profile);
        if prepared {
            state.derived.trajectory = Some(state.derived.inputs);
        } else {
            super::rebuild_state_trajectory(state);
        }
    }
    state.performance.paths(&state.display, &state.machining);
    state.cad_controls = Default::default();
    state.cad_curves = Default::default();
    state.cad_diagnostics = Default::default();
    state.cad_layers = Default::default();
    state.text_edit = Default::default();
    state.measurements = Default::default();
    state.pending_copy = None;
    state.cad_command = Default::default();
    state.dynamic_input.reset_fields();
    state.trajectory_view = Default::default();
    state.command_input.clear();
    state.cad_history.clear();

    state.fit_dirty = true;
    super::cad_project::reset(state);
}

pub(super) fn new_drawing(state: &mut ViewerState) {
    state.tasks.cancel();
    if confirm_replace(state) {
        install_drawing(state, Some(EditableDrawing::blank()));
        push_message(state, "已新建毫米图纸");
    } else {
        state
            .tasks
            .defer_after_save(super::cad_jobs::AfterSave::New);
    }
}

pub(super) fn close_drawing(state: &mut ViewerState) {
    state.tasks.cancel();
    if confirm_replace(state) {
        install_drawing(state, None);
    } else {
        state
            .tasks
            .defer_after_save(super::cad_jobs::AfterSave::Close);
    }
}

pub(super) fn confirm_replace(state: &mut ViewerState) -> bool {
    if state.project.native_project || super::cad_project::machining_modified(state) {
        if !super::cad_project::modified(state) {
            return true;
        }
        return match rfd::MessageDialog::new().set_title("保存当前 CAD/CAM 项目？")
            .set_description("图纸、加工安排或工艺配置有未保存修改。是：保存项目后继续；否：放弃；取消：保留当前工作。DXF 保存不包含加工安排。")
            .set_buttons(rfd::MessageButtons::YesNoCancel).show() {
                rfd::MessageDialogResult::Yes => super::cad_project::save(state, false),
                rfd::MessageDialogResult::No => true,
                _ => false,
            };
    }
    if !state
        .drawing
        .as_ref()
        .is_some_and(EditableDrawing::is_modified)
    {
        return true;
    }
    match rfd::MessageDialog::new().set_title("保存当前 CAD 修改？")
        .set_description("是：保存后继续；否：放弃修改；取消：留在当前图纸。保存失败或取消另存为都不会关闭当前图纸。")
        .set_buttons(rfd::MessageButtons::YesNoCancel).show() {
        rfd::MessageDialogResult::Yes => save_drawing(state, false),
        rfd::MessageDialogResult::No => true,
        _ => false,
    }
}

pub(super) fn save_drawing(state: &mut ViewerState, save_as: bool) -> bool {
    let Some(drawing) = state.drawing.as_ref() else {
        return false;
    };
    let source = &drawing.document().source;
    let existing = source.file_path.as_ref().filter(|path| {
        source.format.is_dxf()
            && path
                .extension()
                .is_some_and(|extension| extension.eq_ignore_ascii_case("dxf"))
    });
    let path = if save_as || existing.is_none() {
        let file_name = Path::new(&source.file_name).with_extension("dxf");
        let Some(path) = rfd::FileDialog::new()
            .set_title("保存 CAD 图纸为 DXF")
            .add_filter("DXF 图纸", &["dxf"])
            .set_directory(super::dialog_directory(state))
            .set_file_name(file_name.to_string_lossy())
            .save_file()
        else {
            return false;
        };
        path
    } else {
        existing.unwrap().clone()
    };
    match state.drawing.as_mut().unwrap().save_dxf(&path) {
        Ok(()) => {
            let source = &state.drawing.as_ref().unwrap().document().source;
            state.file_name = source.file_name.clone();
            state.format = source.format.to_string();
            clear_recovery(state);
            remember_recent(
                state,
                &std::fs::canonicalize(&path).unwrap_or_else(|_| path.clone()),
            );
            push_message(
                state,
                format!(
                    "已保存 {}；覆盖已有文件时保留同目录 .bak 备份",
                    path.display()
                ),
            );
            true
        }
        Err(error) => {
            push_message(state, format!("保存失败：{error}"));
            false
        }
    }
}

pub(super) fn close_requested(
    mut events: MessageReader<WindowCloseRequested>,
    mut state: ResMut<ViewerState>,
    mut exit: MessageWriter<AppExit>,
) {
    let requested = events.read().count() > 0 || state.file_ui.exit_requested;
    state.file_ui.exit_requested = false;
    if requested && confirm_replace(&mut state) {
        clear_recovery(&mut state);
        exit.write(AppExit::Success);
    } else if requested {
        state
            .tasks
            .defer_after_save(super::cad_jobs::AfterSave::Exit);
    }
}

pub(super) fn recovery_directory() -> PathBuf {
    std::env::var_os("LOCALAPPDATA")
        .or_else(|| std::env::var_os("APPDATA"))
        .map(PathBuf::from)
        .unwrap_or_else(|| std::env::current_dir().unwrap_or_default())
        .join("CADCAM-Rust")
        .join("Recovery")
}

fn clear_recovery(state: &mut ViewerState) {
    if let Some(path) = state.file_ui.recovery_path.take() {
        if let Err(error) = std::fs::remove_file(&path) {
            if error.kind() != std::io::ErrorKind::NotFound {
                push_message(state, format!("恢复快照未能清理：{error}"));
            }
        }
    }
    state.file_ui.recovery_revision = None;
    state.file_ui.recovery_elapsed = 0.0;
}

pub(super) fn maintain_recovery(time: Res<Time>, mut state: ResMut<ViewerState>) {
    state.file_ui.recovery_elapsed += time.delta_secs();
    if state.project.native_project && !super::cad_project::modified(&state) {
        if state.file_ui.recovery_path.is_some() {
            clear_recovery(&mut state);
        }
        return;
    }
    let Some(drawing) = state.drawing.as_ref() else {
        return;
    };
    if !drawing.is_modified() {
        if state.file_ui.recovery_path.is_some() {
            clear_recovery(&mut state);
        }
        return;
    }
    let revision = drawing.revision();
    if state.file_ui.recovery_revision == Some(revision)
        || state.file_ui.recovery_path.is_some() && state.file_ui.recovery_elapsed < 30.0
    {
        return;
    }
    let directory = recovery_directory();
    if state.file_ui.recovery_path.is_none() {
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        let stem = Path::new(&state.file_name)
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy();
        state.file_ui.recovery_path = Some(directory.join(format!(
            "{stem}.recovery-{}-{stamp}.dxf",
            std::process::id()
        )));
    }
    state.file_ui.recovery_elapsed = 0.0;
    let result = std::fs::create_dir_all(&directory)
        .map_err(|error| error.to_string())
        .and_then(|_| {
            state
                .drawing
                .as_ref()
                .unwrap()
                .write_recovery(state.file_ui.recovery_path.as_ref().unwrap())
        });
    match result {
        Ok(()) => state.file_ui.recovery_revision = Some(revision),
        Err(error) => push_message(&mut state, format!("自动恢复快照写入失败：{error}")),
    }
}

pub(super) fn recover_drawing(state: &mut ViewerState) {
    let directory = recovery_directory();
    if let Err(error) = std::fs::create_dir_all(&directory) {
        push_message(state, format!("无法访问恢复目录：{error}"));
        return;
    }
    if let Some(path) = rfd::FileDialog::new()
        .set_title("恢复图纸快照（不含 CAM 安排和撤销历史）")
        .add_filter("恢复 DXF / 备份", &["dxf", "bak"])
        .set_directory(&directory)
        .pick_file()
    {
        if state.file_ui.recovery_path.as_ref() == Some(&path) {
            push_message(state, "该快照属于当前会话，请先保存或另存为当前图纸");
            return;
        }
        open_drawing(state, &path.to_string_lossy());
    }
}

pub(super) fn import_report_ui(context: &egui::Context, state: &mut ViewerState) {
    if !state.file_ui.report_open {
        return;
    }
    let Some(report) = state.file_ui.report.as_ref() else {
        state.file_ui.report_open = false;
        return;
    };
    let mut close = false;
    let modal = egui::Modal::new(egui::Id::new("cad_import_report")).show(context, |ui| {
        ui.set_width((context.viewport_rect().width() - 60.0).clamp(280.0, 960.0));
        ui.heading("导入报告");
        ui.label(&state.file_ui.report_name);
        ui.label(&state.unit);
        ui.label(format!("解析对象 {} · 模型空间 {} · 已转换 {} · 跳过 {} · 模型空间外 {}", report.source_entities, report.model_space_entities, report.converted.values().sum::<usize>(), report.skipped.len() + report.skipped_omitted, report.ignored_non_model_space));
        ui.separator();
        egui::ScrollArea::vertical().max_height((context.viewport_rect().height() - 230.0).max(140.0)).show(ui, |ui| {
            ui.strong("已转换图元");
            for (kind, count) in &report.converted { ui.label(format!("{kind}：{count}")); }
            ui.separator(); ui.strong("未转换对象（仅列出模型空间中的样本）");
            if report.skipped.is_empty() { ui.label("无跳过样本"); }
            for (index, item) in report.skipped.iter().enumerate() {
                ui.push_id(index, |ui| {
                    ui.group(|ui| {
                        ui.label(format!("{} · 图层 {} · 句柄 {} · {}", item.source_type, item.layer, item.handle.map(|handle| format!("{handle:X}")).unwrap_or_else(|| "无".into()), item.reason));
                        ui.label(&item.message);
                    });
                });
            }
            if report.skipped_omitted > 0 { ui.label(format!("另有 {} 个跳过对象未逐项列出", report.skipped_omitted)); }
            ui.separator();ui.strong("块实例展开与源映射");
            ui.label(format!("原始对象 {} · 原始模型空间 {} · 已展开实例 {} · 独立图元 {}",report.blocks.original_entities,report.blocks.original_model_space,report.blocks.instances,report.blocks.entities));
            for failure in &report.blocks.failures{ui.colored_label(egui::Color32::LIGHT_RED,failure);}
            if report.blocks.failures_omitted>0{ui.label(format!("另有 {} 个未展开实例未逐项列出",report.blocks.failures_omitted));}
            ui.label(format!("解析跳过原始记录 {}；需核对恢复/截断提示：{}",report.parser_skipped_records,report.parser_review_required));
            for map in &report.blocks.mappings{
                let path=map.path.iter().map(|s|format!("{:X}[{},{}]",s.handle,s.row,s.column)).collect::<Vec<_>>().join(" → ");
                ui.small(format!("新句柄 {:X} ← {} → 源 {:X} / 段 {}",map.handle,path,map.source_handle,map.segment));
            }
            if report.blocks.mappings_omitted>0{ui.label(format!("另有 {} 条映射未列出",report.blocks.mappings_omitted));}
            ui.separator(); ui.strong("解析及单位提示");
            for warning in &report.warnings { ui.label(warning); }
            ui.label("报告保留本次打开时的统计；后续编辑不改写该报告。未转换对象不会进入当前加工模型，DXF 保留范围取决于解析库。");
        });
        if report.has_omissions(){ui.separator();ui.colored_label(egui::Color32::LIGHT_RED,"存在未转换对象或解析恢复/截断记录，请核对加工范围。");ui.checkbox(&mut state.file_ui.coverage_acknowledged,"我已核对遗漏，允许仅导出当前已转换图元的加工代码");}
        ui.separator();
        close = ui.button("关闭").clicked();
    });
    if close || modal.should_close() {
        state.file_ui.report_open = false;
    }
}
