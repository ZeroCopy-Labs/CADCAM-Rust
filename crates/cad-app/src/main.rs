//! Bevy + wgpu desktop viewer for the CAD core.
//!
//! cad-core owns parsing/model data, this crate tessellates it into 2D display
//! paths, Bevy renders them with gizmos, and egui provides the desktop UI.

use std::env;
use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use bevy::{
    gizmos::config::{GizmoConfigStore, GizmoLineStyle},
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
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
    recent: Vec<PathBuf>,
    messages: Vec<String>,
    command_open: bool,
    command_focus: bool,
    command_input: String,
}

/// Orbit camera state, independent from the loaded drawing.
#[derive(Resource)]
struct CameraRig {
    target: Vec3,
    yaw: f32,
    pitch: f32,
    distance: f32,
    ortho_scale: f32,
}

impl Default for CameraRig {
    fn default() -> Self {
        Self {
            target: Vec3::ZERO,
            yaw: -FRAC_PI_4,
            pitch: 0.55,
            distance: 100.0,
            ortho_scale: 1.0,
        }
    }
}

impl CameraRig {
    /// Unit direction from the orbit target toward the camera.
    fn direction(&self) -> Vec3 {
        let cos_pitch = self.pitch.cos();
        Vec3::new(
            cos_pitch * self.yaw.sin(),
            self.pitch.sin(),
            cos_pitch * self.yaw.cos(),
        )
    }
}

/// Marker for the single 3D camera.
#[derive(Component)]
struct MainCamera;

/// Dashed negative half-axes.
#[derive(Default, Reflect, GizmoConfigGroup)]
struct DashedAxisGizmos;

fn main() {
    let initial_path = env::args().nth(1).unwrap_or_default();
    let mut state = ViewerState {
        recent: load_recent(),
        ..Default::default()
    };
    push_message(&mut state, "使用“文件 → 打开文件”选择 DXF/DWG 图纸");
    push_message(
        &mut state,
        "视角：Shift+中键旋转 | 中键拖动平移 | 滚轮缩放 | 右上角魔方切换",
    );
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
        .insert_resource(CameraRig::default())
        .init_gizmo_group::<DashedAxisGizmos>()
        .add_systems(Startup, (setup_camera, configure_gizmos))
        .add_systems(
            Update,
            (orbit_camera, fit_camera, update_camera, draw_scene).chain(),
        )
        .add_systems(EguiPrimaryContextPass, drawing_ui)
        .run();
}

fn setup_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Projection::Orthographic(OrthographicProjection {
            scale: 1.0,
            near: 0.01,
            far: 1.0e7,
            ..OrthographicProjection::default_3d()
        }),
        MainCamera,
    ));
}

fn configure_gizmos(mut store: ResMut<GizmoConfigStore>) {
    let (negative_axes, _) = store.config_mut::<DashedAxisGizmos>();
    negative_axes.line.width = 1.5;
    negative_axes.line.style = GizmoLineStyle::Dashed {
        gap_scale: 3.0,
        line_scale: 3.0,
    };
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

/// Orbit/pan/zoom controls for the 3D view.
fn orbit_camera(
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut rig: ResMut<CameraRig>,
) {
    if mouse.pressed(MouseButton::Middle) {
        let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
        if shift {
            // Shift + middle drag: orbit the 3D view.
            rig.yaw -= motion.delta.x * 0.008;
            rig.pitch = (rig.pitch + motion.delta.y * 0.008).clamp(-1.55, 1.55);
        } else {
            // Plain middle drag: pan the orbit target.
            let forward = -rig.direction();
            let right = forward.cross(Vec3::Y).normalize_or_zero();
            if right.length_squared() > 0.0 {
                let up = right.cross(forward).normalize();
                let speed = rig.ortho_scale;
                rig.target -= right * motion.delta.x * speed;
                rig.target += up * motion.delta.y * speed;
            }
        }
    }

    // Middle mouse wheel: zoom the orthographic projection.
    let wheel = scroll.delta.y;
    if wheel != 0.0 {
        rig.ortho_scale = (rig.ortho_scale * (-wheel * 0.12).exp()).clamp(1.0e-5, 1.0e7);
    }
}

/// Fit the orbit target and distance to the drawing bounds.
fn fit_camera(
    mut state: ResMut<ViewerState>,
    mut rig: ResMut<CameraRig>,
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
    let depth = (bounds.max_y - bounds.min_y).abs().max(1.0e-6);
    let height = (bounds.max_z - bounds.min_z).abs().max(1.0e-6);
    let center = Vec3::new(
        ((bounds.min_x + bounds.max_x) * 0.5) as f32,
        ((bounds.min_z + bounds.max_z) * 0.5) as f32,
        -((bounds.min_y + bounds.max_y) * 0.5) as f32,
    );
    rig.target = center;

    // Fit the camera-aligned bounding box, so Z heights are included.
    let direction = rig.direction();
    let forward = -direction;
    let mut right = forward.cross(Vec3::Y);
    if right.length_squared() < 1.0e-6 {
        right = Vec3::X;
    } else {
        right = right.normalize();
    }
    let up = right.cross(forward).normalize_or_zero();
    let mut max_right = 0.0f32;
    let mut max_up = 0.0f32;
    for x in [bounds.min_x, bounds.max_x] {
        for y in [bounds.min_y, bounds.max_y] {
            for z in [bounds.min_z, bounds.max_z] {
                let corner = Vec3::new(x as f32, z as f32, -(y as f32)) - center;
                max_right = max_right.max(corner.dot(right).abs());
                max_up = max_up.max(corner.dot(up).abs());
            }
        }
    }
    let viewport_width = window.width().max(1.0);
    let viewport_height = window.height().max(1.0);
    let scale = (2.0 * max_right / viewport_width)
        .max(2.0 * max_up / viewport_height)
        .max(1.0e-6)
        * 1.15;
    rig.ortho_scale = scale;
    rig.distance = ((width.max(depth).max(height) * 2.0).max(10.0)) as f32;
    state.fit_dirty = false;
}

fn update_camera(
    rig: Res<CameraRig>,
    camera: Single<(&mut Transform, &mut Projection), With<MainCamera>>,
) {
    let eye = rig.target + rig.direction() * rig.distance;
    let (mut transform, mut projection) = camera.into_inner();
    *transform = Transform::from_translation(eye).looking_at(rig.target, Vec3::Y);
    if let Projection::Orthographic(orthographic) = &mut *projection {
        orthographic.scale = rig.ortho_scale;
    }
}

/// Draw the drawing paths and solid/dashed coordinate axes.
fn draw_scene(
    state: Res<ViewerState>,
    mut gizmos: Gizmos,
    mut negative_axes: Gizmos<DashedAxisGizmos>,
) {
    for path in &state.display.paths {
        let color = Color::srgb(path.color[0], path.color[1], path.color[2]);
        for pair in path.points.windows(2) {
            let [x0, y0, z0] = pair[0];
            let [x1, y1, z1] = pair[1];
            gizmos.line(
                Vec3::new(x0 as f32, z0 as f32, -(y0 as f32)),
                Vec3::new(x1 as f32, z1 as f32, -(y1 as f32)),
                color,
            );
        }
    }

    let Some(bounds) = state.display.bounds else {
        return;
    };

    // Keep the axes at the CAD world origin, never at the drawing center.
    let extent = ((bounds.max_x - bounds.min_x)
        .max(bounds.max_y - bounds.min_y)
        .max(bounds.max_z - bounds.min_z)
        .max(1.0)) as f32;
    let axis = extent * 0.6;
    let z_axis = axis * 0.5;
    let red = Color::srgb(0.85, 0.25, 0.25);
    let green = Color::srgb(0.30, 0.85, 0.35);
    let blue = Color::srgb(0.25, 0.45, 0.95);
    let origin = Vec3::ZERO;

    // CAD +X, +Y and +Z in Bevy coordinates: CAD Y maps to -Bevy Z.
    let positive_x = Vec3::X * axis;
    let positive_y = -Vec3::Z * axis;
    let positive_z = Vec3::Y * z_axis;
    gizmos.line(origin, positive_x, red);
    gizmos.line(origin, positive_y, blue);
    gizmos.line(origin, positive_z, green);
    negative_axes.line(origin, -positive_x, red);
    negative_axes.line(origin, -positive_y, blue);
    negative_axes.line(origin, -positive_z, green);
}

fn drawing_ui(
    mut contexts: EguiContexts,
    mut state: ResMut<ViewerState>,
    mut camera: ResMut<CameraRig>,
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
                        if ui.button("等轴测").clicked() {
                            camera.yaw = -FRAC_PI_4;
                            camera.pitch = 0.55;
                            ui.close();
                        }
                        if ui.button("俯视图").clicked() {
                            camera.yaw = 0.0;
                            camera.pitch = 1.55;
                            ui.close();
                        }
                        if ui.button("前视图").clicked() {
                            camera.yaw = 0.0;
                            camera.pitch = 0.0;
                            ui.close();
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
    view_cube_ui(context, &mut camera);
    Ok(())
}

/// Small clickable orientation cube in the top-right corner.
fn view_cube_ui(context: &egui::Context, rig: &mut CameraRig) {
    egui::Area::new(egui::Id::new("view_cube"))
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-12.0, 48.0))
        .show(context, |ui| {
            let size = 108.0;
            let (rect, response) =
                ui.allocate_exact_size(egui::Vec2::splat(size), egui::Sense::click());
            let painter = ui.painter().clone();
            painter.rect_filled(
                rect,
                6.0,
                egui::Color32::from_rgba_unmultiplied(12, 17, 24, 235),
            );

            let dir = rig.direction();
            let forward = -dir;
            let mut right = forward.cross(Vec3::Y);
            if right.length_squared() < 1.0e-6 {
                right = Vec3::X;
            } else {
                right = right.normalize();
            }
            let up = right.cross(forward).normalize_or_zero();
            let center = rect.center();
            let scale = size * 0.28;
            let project =
                |point: Vec3| center + egui::vec2(point.dot(right), -point.dot(up)) * scale;

            let pointer = ui.input(|input| input.pointer.hover_pos());
            let visible: Vec<_> = cube_faces()
                .into_iter()
                .filter(|(_, normal, _, _)| normal.dot(dir) > 0.02)
                .map(|(vertices, normal, label, view)| {
                    let projected: Vec<egui::Pos2> =
                        vertices.iter().map(|point| project(*point)).collect();
                    (projected, normal, label, view)
                })
                .collect();
            let hovered = pointer.and_then(|position| {
                visible
                    .iter()
                    .position(|(polygon, _, _, _)| point_in_polygon(position, polygon))
            });

            for (index, (polygon, normal, label, _)) in visible.iter().enumerate() {
                let base = if normal.x.abs() > 0.5 {
                    egui::Color32::from_rgb(52, 62, 78)
                } else if normal.y.abs() > 0.5 {
                    egui::Color32::from_rgb(58, 70, 76)
                } else {
                    egui::Color32::from_rgb(48, 58, 74)
                };
                let fill = if Some(index) == hovered {
                    egui::Color32::from_rgb(76, 105, 140)
                } else {
                    base
                };
                painter.add(egui::Shape::convex_polygon(
                    polygon.clone(),
                    fill,
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(100, 120, 145)),
                ));
                let center = polygon
                    .iter()
                    .fold(egui::Pos2::ZERO, |acc, point| acc + point.to_vec2())
                    / polygon.len() as f32;
                painter.text(
                    center,
                    egui::Align2::CENTER_CENTER,
                    *label,
                    egui::FontId::proportional(10.0),
                    egui::Color32::from_rgb(225, 232, 240),
                );
            }

            if response.clicked() {
                if let Some(index) = hovered {
                    let (yaw, pitch) = visible[index].3;
                    rig.yaw = yaw;
                    rig.pitch = pitch;
                }
            }
        });
}

/// Six cube faces: corners, outward normal, label, and (yaw, pitch) preset.
fn cube_faces() -> [([Vec3; 4], Vec3, &'static str, (f32, f32)); 6] {
    let v = Vec3::new;
    [
        (
            [
                v(1.0, -1.0, -1.0),
                v(1.0, 1.0, -1.0),
                v(1.0, 1.0, 1.0),
                v(1.0, -1.0, 1.0),
            ],
            Vec3::X,
            "右",
            (FRAC_PI_2, 0.0),
        ),
        (
            [
                v(-1.0, -1.0, 1.0),
                v(-1.0, 1.0, 1.0),
                v(-1.0, 1.0, -1.0),
                v(-1.0, -1.0, -1.0),
            ],
            Vec3::NEG_X,
            "左",
            (-FRAC_PI_2, 0.0),
        ),
        (
            [
                v(-1.0, 1.0, -1.0),
                v(1.0, 1.0, -1.0),
                v(1.0, 1.0, 1.0),
                v(-1.0, 1.0, 1.0),
            ],
            Vec3::Y,
            "上",
            (0.0, 1.55),
        ),
        (
            [
                v(-1.0, -1.0, 1.0),
                v(1.0, -1.0, 1.0),
                v(1.0, -1.0, -1.0),
                v(-1.0, -1.0, -1.0),
            ],
            Vec3::NEG_Y,
            "下",
            (0.0, -1.55),
        ),
        (
            [
                v(-1.0, -1.0, 1.0),
                v(-1.0, 1.0, 1.0),
                v(1.0, 1.0, 1.0),
                v(1.0, -1.0, 1.0),
            ],
            Vec3::Z,
            "前",
            (0.0, 0.0),
        ),
        (
            [
                v(1.0, -1.0, -1.0),
                v(1.0, 1.0, -1.0),
                v(-1.0, 1.0, -1.0),
                v(-1.0, -1.0, -1.0),
            ],
            Vec3::NEG_Z,
            "后",
            (PI, 0.0),
        ),
    ]
}

fn point_in_polygon(point: egui::Pos2, polygon: &[egui::Pos2]) -> bool {
    let mut inside = false;
    let mut previous = polygon.len().saturating_sub(1);
    for current in 0..polygon.len() {
        let a = polygon[current];
        let b = polygon[previous];
        if (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x
        {
            inside = !inside;
        }
        previous = current;
    }
    inside
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
