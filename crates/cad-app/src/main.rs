//! Bevy + wgpu desktop viewer for the CAD core.
//!
//! cad-core owns parsing/model data, this crate tessellates it into 2D display
//! paths, Bevy renders them with gizmos, and egui provides the desktop UI.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use bevy::{
    prelude::*,
    window::{PrimaryWindow, WindowPlugin, WindowTheme},
};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};

use cad_core::read_document;

mod display;

const BACKGROUND: Color = Color::srgb(0.035, 0.045, 0.060);
const RECENT_LIMIT: usize = 10;
const MESSAGE_LIMIT: usize = 200;
const COMMAND_MAX_HISTORY_LINES: usize = 4;
const COMMAND_ROW_HEIGHT: f32 = 21.0;
const COMMAND_VERTICAL_PADDING: f32 = 12.0;
const TOP_BAR_HEIGHT: f32 = 36.0;
const BOTTOM_BAR_HEIGHT: f32 = 30.0;
const MENU_SIDE_WIDTH: f32 = 140.0;

/// Everything the viewer keeps after a drawing is loaded.
#[derive(Resource, Default)]
struct ViewerState {
    file_name: String,
    format: String,
    version: String,
    unit: String,
    entity_count: usize,
    layer_count: usize,
    warning_count: usize,
    display: display::DisplayData,
    fit_dirty: bool,
    rotation_deg: f32,
    recent: Vec<PathBuf>,
    messages: Vec<String>,
    command_open: bool,
    command_focus: bool,
    command_input: String,
}

/// Marker for the single 2D camera.
#[derive(Component)]
struct MainCamera;

fn main() {
    let initial_path = env::args().nth(1).unwrap_or_default();
    let mut state = ViewerState {
        recent: load_recent(),
        ..Default::default()
    };
    push_message(&mut state, "使用“文件 → 打开文件”选择 DXF/DWG 图纸");
    if !initial_path.trim().is_empty() {
        open_drawing(&mut state, &initial_path);
    }

    App::new()
        .insert_resource(ClearColor(BACKGROUND))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CAD/CAM — Bevy Viewer".to_string(),
                resolution: (1280, 800).into(),
                window_theme: Some(WindowTheme::Dark),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .insert_resource(state)
        .add_systems(Startup, setup_camera)
        .add_systems(Update, (fit_camera, draw_display))
        .add_systems(EguiPrimaryContextPass, drawing_ui)
        .run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn((Camera2d, MainCamera));
}

fn open_drawing(state: &mut ViewerState, raw_path: &str) {
    let path = normalize_path(raw_path);
    if path.is_empty() {
        push_message(state, "请输入 DXF/DWG 路径");
        return;
    }

    push_message(state, format!("正在读取 {path}"));
    match read_document(&path) {
        Ok(document) => {
            let display = display::build_display(&document);
            state.file_name = document.source.file_name.clone();
            state.format = document.source.format.to_string();
            state.version = document.source.version.clone();
            state.unit = document
                .units
                .map(|unit| unit.to_string())
                .unwrap_or_else(|| "未指定".to_string());
            state.entity_count = document.entity_count();
            state.layer_count = document.layers.len();
            state.warning_count = document.import_report.warnings.len();
            state.display = display;
            state.rotation_deg = 0.0;
            state.fit_dirty = true;

            push_message(state, format!("已加载 {} 个实体", state.entity_count));
            push_message(
                state,
                format!(
                    "实体 {} | 图层 {} | {} | {} | {} | 警告 {}",
                    state.entity_count,
                    state.layer_count,
                    state.format,
                    state.version,
                    state.unit,
                    state.warning_count
                ),
            );

            let mut recent_path = PathBuf::from(&path);
            if !recent_path.is_absolute() {
                recent_path = env::current_dir().unwrap_or_default().join(recent_path);
            }
            remember_recent(state, &recent_path);
        }
        Err(error) => {
            push_message(state, format!("读取失败：{error}"));
        }
    }
}

fn push_message(state: &mut ViewerState, message: impl Into<String>) {
    state.messages.push(message.into());
    if state.messages.len() > MESSAGE_LIMIT {
        state.messages.remove(0);
    }
}

/// Current command implementation: only echo the entered text.
fn submit_command(state: &mut ViewerState, raw: &str) {
    let command = raw.trim();
    if command.is_empty() {
        return;
    }
    push_message(state, command.to_string());
}

/// Fit the 2D camera to the current display bounds.
fn fit_camera(
    mut state: ResMut<ViewerState>,
    camera: Single<(&mut Transform, &mut Projection), With<MainCamera>>,
    window: Single<&Window, With<PrimaryWindow>>,
) {
    if !state.fit_dirty {
        return;
    }
    let Some(bounds) = state.display.bounds else {
        state.fit_dirty = false;
        return;
    };

    let width = (bounds.max_x - bounds.min_x).abs().max(1.0e-6);
    let height = (bounds.max_y - bounds.min_y).abs().max(1.0e-6);
    // Keep the drawing inside the menu/status bars instead of hiding it
    // underneath the external UI panes.
    let viewport_width = window.width().max(1.0) as f64;
    let viewport_height = (window.height() - TOP_BAR_HEIGHT - BOTTOM_BAR_HEIGHT).max(1.0) as f64;
    let scale = ((width / viewport_width).max(height / viewport_height) * 1.12) as f32;

    let (mut transform, mut projection) = camera.into_inner();
    transform.translation.x = ((bounds.min_x + bounds.max_x) * 0.5) as f32;
    transform.translation.y = ((bounds.min_y + bounds.max_y) * 0.5) as f32
        + (TOP_BAR_HEIGHT - BOTTOM_BAR_HEIGHT) * 0.5 * scale;
    transform.translation.z = 1000.0;
    transform.rotation = Quat::from_rotation_z(state.rotation_deg.to_radians());
    if let Projection::Orthographic(orthographic) = &mut *projection {
        orthographic.scale = scale.max(1.0e-6);
    }
    state.fit_dirty = false;
}

/// Draw all display paths as 2D line segments.
fn draw_display(state: Res<ViewerState>, mut gizmos: Gizmos) {
    for path in &state.display.paths {
        let color = Color::srgb(path.color[0], path.color[1], path.color[2]);
        for pair in path.points.windows(2) {
            let [x0, y0] = pair[0];
            let [x1, y1] = pair[1];
            gizmos.line_2d(
                Vec2::new(x0 as f32, y0 as f32),
                Vec2::new(x1 as f32, y1 as f32),
                color,
            );
        }
    }
}

fn drawing_ui(
    mut contexts: EguiContexts,
    mut state: ResMut<ViewerState>,
    mut fonts_loaded: Local<bool>,
) -> Result {
    let context = contexts.ctx_mut()?;
    install_cjk_font(context, &mut fonts_loaded);
    context.set_visuals(egui::Visuals::dark());

    let mut open_dialog = false;
    let mut pending_path: Option<PathBuf> = None;
    let mut fit_clicked = false;
    let current_name = if state.file_name.is_empty() {
        "未打开文件".to_string()
    } else {
        state.file_name.clone()
    };

    let mut root = egui::Ui::new(
        context.clone(),
        egui::Id::new("cad_root"),
        egui::UiBuilder::new()
            .layer_id(egui::LayerId::background())
            .max_rect(context.viewport_rect()),
    );

    egui::Panel::top("menu_bar").show(&mut root, |ui| {
        ui.horizontal(|ui| {
            ui.allocate_ui_with_layout(
                egui::Vec2::new(MENU_SIDE_WIDTH, 24.0),
                egui::Layout::left_to_right(egui::Align::Center),
                |ui| {
                    ui.menu_button("文件", |ui| {
                        if ui.button("打开文件").clicked() {
                            open_dialog = true;
                            ui.close();
                        }
                        ui.separator();
                        ui.menu_button("最近打开", |ui| {
                            if state.recent.is_empty() {
                                ui.label("暂无记录");
                            } else {
                                for path in &state.recent {
                                    let name = path
                                        .file_name()
                                        .map(|name| name.to_string_lossy().into_owned())
                                        .unwrap_or_else(|| path.display().to_string());
                                    let response = ui.button(name);
                                    let clicked = response.clicked();
                                    response.on_hover_text(path.display().to_string());
                                    if clicked {
                                        pending_path = Some(path.clone());
                                        ui.close();
                                    }
                                }
                            }
                        });
                    });
                    ui.menu_button("视图", |ui| {
                        if ui.button("适应视图").clicked() {
                            fit_clicked = true;
                            ui.close();
                        }
                        if ui
                            .add(
                                egui::Slider::new(&mut state.rotation_deg, 0.0..=360.0)
                                    .text("旋转"),
                            )
                            .changed()
                        {
                            fit_clicked = true;
                        }
                    });
                },
            );

            let center_width = (ui.available_width() - MENU_SIDE_WIDTH).max(0.0);
            ui.allocate_ui_with_layout(
                egui::Vec2::new(center_width, 24.0),
                egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                |ui| {
                    ui.strong(&current_name);
                },
            );
            ui.allocate_ui_with_layout(
                egui::Vec2::new(MENU_SIDE_WIDTH, 24.0),
                egui::Layout::left_to_right(egui::Align::Center),
                |_ui| {},
            );
        });
    });

    let command_dimensions = state.command_open.then(|| {
        let history_lines = state.messages.len().min(COMMAND_MAX_HISTORY_LINES);
        let history_height = history_lines as f32 * COMMAND_ROW_HEIGHT;
        let total_height =
            (history_lines + 1) as f32 * COMMAND_ROW_HEIGHT + COMMAND_VERTICAL_PADDING;
        (total_height, history_height)
    });
    let mut command_panel = egui::Panel::bottom("command_line").frame(
        egui::Frame::new()
            .inner_margin(egui::Margin::symmetric(0, 6))
            .fill(egui::Color32::from_rgb(12, 17, 24)),
    );
    if let Some((outer_height, _)) = command_dimensions {
        command_panel = command_panel.min_size(outer_height).max_size(outer_height);
    }
    let command_panel = command_panel.show(&mut root, |ui| {
        if state.command_open {
            let (_, history_height) = command_dimensions.unwrap_or((0.0, 0.0));
            if history_height > 0.0 {
                egui::ScrollArea::vertical()
                    .max_height(history_height)
                    .auto_shrink([false, true])
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for message in &state.messages {
                            ui.horizontal(|ui| {
                                ui.add_space(8.0);
                                ui.label(format!("> {message}"));
                            });
                        }
                    });
            }

            let focus_input = std::mem::take(&mut state.command_focus);
            ui.horizontal(|ui| {
                ui.add_space(8.0);
                let response = ui.add_sized(
                    egui::Vec2::new(ui.available_width(), COMMAND_ROW_HEIGHT - 2.0),
                    egui::TextEdit::singleline(&mut state.command_input).hint_text("输入命令"),
                );
                if focus_input {
                    response.request_focus();
                }
                let enter = ui.input(|input| input.key_pressed(egui::Key::Enter));
                if enter && (response.has_focus() || response.lost_focus()) {
                    let command = std::mem::take(&mut state.command_input);
                    submit_command(&mut state, &command);
                    response.request_focus();
                }
            });
        } else {
            let latest = state
                .messages
                .last()
                .cloned()
                .unwrap_or_else(|| "就绪".to_string());
            let (rect, response) = ui.allocate_exact_size(
                egui::Vec2::new(ui.available_width(), COMMAND_ROW_HEIGHT),
                egui::Sense::click(),
            );
            ui.painter().text(
                rect.left_center() + egui::vec2(8.0, 0.0),
                egui::Align2::LEFT_CENTER,
                format!("> {latest}"),
                egui::TextStyle::Body.resolve(ui.style()),
                ui.visuals().text_color(),
            );
            if response.clicked() {
                state.command_open = true;
                state.command_focus = true;
            }
        }
    });
    let command_rect = command_panel.response.rect;
    if state.command_open {
        let clicked = context.input(|input| input.pointer.any_click());
        let pointer = context.input(|input| input.pointer.interact_pos());
        if clicked {
            if let Some(position) = pointer {
                if !command_rect.contains(position) {
                    state.command_open = false;
                }
            }
        }
    }

    if open_dialog {
        if let Some(path) = choose_drawing_file(&state) {
            pending_path = Some(path);
        }
    }
    if let Some(path) = pending_path {
        open_drawing(&mut state, &path.to_string_lossy());
    }
    if fit_clicked {
        state.fit_dirty = true;
    }
    Ok(())
}

/// Install a Windows CJK font so the Chinese menu text is not rendered as boxes.
fn install_cjk_font(context: &egui::Context, loaded: &mut bool) {
    if *loaded {
        return;
    }
    *loaded = true;

    let candidates = [r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\msyh.ttc"];
    let Some(bytes) = candidates.iter().find_map(|path| fs::read(path).ok()) else {
        return;
    };

    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "cjk".to_string(),
        Arc::new(egui::FontData::from_owned(bytes)),
    );
    fonts
        .families
        .entry(egui::FontFamily::Proportional)
        .or_default()
        .insert(0, "cjk".to_string());
    fonts
        .families
        .entry(egui::FontFamily::Monospace)
        .or_default()
        .push("cjk".to_string());
    context.set_fonts(fonts);
}

/// Native file picker, starting in the last used directory.
fn choose_drawing_file(state: &ViewerState) -> Option<PathBuf> {
    rfd::FileDialog::new()
        .set_title("打开 CAD 图纸")
        .add_filter("CAD 图纸 (*.dxf, *.dwg)", &["dxf", "dwg"])
        .set_directory(dialog_directory(state))
        .pick_file()
}

fn dialog_directory(state: &ViewerState) -> PathBuf {
    state
        .recent
        .iter()
        .find_map(|path| path.parent())
        .filter(|directory| directory.is_dir())
        .map(Path::to_path_buf)
        .unwrap_or_else(desktop_dir)
}

/// First launch defaults to the user's Desktop.
fn desktop_dir() -> PathBuf {
    env::var_os("USERPROFILE")
        .map(PathBuf::from)
        .map(|home| home.join("Desktop"))
        .filter(|desktop| desktop.is_dir())
        .unwrap_or_else(|| env::current_dir().unwrap_or_default())
}

fn recent_store_path() -> PathBuf {
    let base = env::var_os("APPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(|| env::current_dir().unwrap_or_default());
    base.join("CADCAM-Rust").join("recent.txt")
}

fn load_recent() -> Vec<PathBuf> {
    let text = fs::read_to_string(recent_store_path()).unwrap_or_default();
    let mut recent = Vec::new();
    for line in text.lines() {
        let path = PathBuf::from(line.trim());
        if path.as_os_str().is_empty() || recent.contains(&path) {
            continue;
        }
        recent.push(path);
        if recent.len() == RECENT_LIMIT {
            break;
        }
    }
    recent
}

fn save_recent(recent: &[PathBuf]) {
    let path = recent_store_path();
    if let Some(parent) = path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    let contents = recent
        .iter()
        .map(|entry| entry.to_string_lossy())
        .collect::<Vec<_>>()
        .join("\n");
    let _ = fs::write(path, contents);
}

fn remember_recent(state: &mut ViewerState, path: &Path) {
    state.recent.retain(|existing| existing != path);
    state.recent.insert(0, path.to_path_buf());
    state.recent.truncate(RECENT_LIMIT);
    save_recent(&state.recent);
}

/// Normalize pasted paths: trim quotes, accept `file://` URLs and remove
/// invisible formatting characters that make Windows reject the path.
fn normalize_path(raw: &str) -> String {
    let mut value = raw.trim().trim_matches('"').trim().to_string();
    if let Some(rest) = value.strip_prefix("file://") {
        let mut path = rest.to_string();
        if path.starts_with('/') && path.len() >= 3 && path.as_bytes()[2] == b':' {
            path.remove(0);
        }
        value = path.replace('/', "\\");
    }
    value
        .chars()
        .filter(|character| {
            !character.is_control()
                && !matches!(
                    character,
                    '\u{200b}'..='\u{200f}'
                        | '\u{202a}'..='\u{202e}'
                        | '\u{2060}'..='\u{206f}'
                        | '\u{feff}'
                )
        })
        .collect()
}
