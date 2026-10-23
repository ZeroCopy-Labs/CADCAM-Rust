//! Bevy + wgpu desktop viewer for the CAD core.
//!
//! cad-core owns parsing/model data, this crate tessellates it into 2D display
//! paths, Bevy renders them with gizmos, and egui provides the desktop UI.

use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use bevy::{
    gizmos::config::{GizmoConfigStore, GizmoLineStyle},
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll},
    log::{Level, LogPlugin},
    prelude::*,
    window::{PrimaryWindow, WindowPlugin, WindowTheme},
};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};

use cad_core::read_document;

mod cam;
mod cam_analysis;
mod cam_trajectory;
mod display;
mod gcode;

const BACKGROUND: Color = Color::srgb(0.035, 0.045, 0.060);
const RECENT_LIMIT: usize = 10;
const MESSAGE_LIMIT: usize = 200;
const COMMAND_MAX_HISTORY_LINES: usize = 4;
const COMMAND_ROW_HEIGHT: f32 = 21.0;
const COMMAND_VERTICAL_PADDING: f32 = 12.0;
const MENU_SIDE_WIDTH: f32 = 300.0;
const PICK_TOLERANCE: f32 = 12.0;

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
    cam: cam::CamModel,
    analysis: cam_analysis::CamAnalysis,
    manual_tube: cam_analysis::TubeSection,
    trajectory: Option<cam_trajectory::MachineTrajectory>,
    gcode_profile: gcode::GCodeProfile,
    gcode_preview: String,
    gcode_settings_open: bool,
    gcode_settings_tab: usize,
    gcode_selected_type: String,
    gcode_selected_color: String,
    gcode_selected_layer: String,
    fit_dirty: bool,
    recent: Vec<PathBuf>,
    messages: Vec<String>,
    command_open: bool,
    command_focus: bool,
    command_input: String,
    load_version: u64,
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

/// Thick yellow overlay for selected entities.
#[derive(Default, Reflect, GizmoConfigGroup)]
struct SelectedGizmos;

/// Current selection and in-progress rubber-band box.
#[derive(Resource, Default)]
struct SelectionState {
    selected: BTreeSet<u32>,
    cam_selected: BTreeSet<cam::UnitId>,
    drag_start: Option<Vec2>,
    drag_current: Option<Vec2>,
    reorder_clicks: Option<ReorderClicks>,
    load_version: u64,
}

#[derive(Clone, Debug)]
struct ReorderClicks {
    group_id: u32,
    start: usize,
    length: usize,
    clicked: Vec<cam::UnitId>,
}

/// CAD space is the authoritative drawing; CAM space is the derived unit view.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum SpaceMode {
    #[default]
    Cad,
    Cam,
}

#[derive(Resource, Default)]
struct SpaceState {
    mode: SpaceMode,
}

/// Last-frame UI occupancy, used to stop mouse/keyboard input leaking into the
/// viewport while interacting with the feature panel or command bar.
#[derive(Resource, Default)]
struct UiState {
    pointer_over_ui: bool,
}

fn main() {
    let initial_path = env::args().nth(1).unwrap_or_default();
    let gcode_path = gcode_config_path();
    let mut state = ViewerState {
        recent: load_recent(),
        gcode_profile: gcode::GCodeProfile::load_or_create(&gcode_path),
        ..Default::default()
    };
    push_message(&mut state, "使用“文件 → 打开文件”选择 DXF/DWG 图纸");
    push_message(
        &mut state,
        "视角：Shift+中键旋转 | 中键拖动平移 | 滚轮缩放 | 右上角魔方切换",
    );
    push_message(&mut state, format!("G代码配置：{}", gcode_path.display()));
    if !initial_path.trim().is_empty() {
        open_drawing(&mut state, &initial_path);
    }

    App::new()
        .insert_resource(ClearColor(BACKGROUND))
        .add_plugins(
            DefaultPlugins
                .set(LogPlugin {
                    level: Level::WARN,
                    ..default()
                })
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "CAD/CAM — Bevy Viewer".to_string(),
                        resolution: (1280, 800).into(),
                        window_theme: Some(WindowTheme::Dark),
                        ..default()
                    }),
                    ..default()
                }),
        )
        .add_plugins(EguiPlugin::default())
        .insert_resource(state)
        .insert_resource(CameraRig::default())
        .insert_resource(SelectionState::default())
        .insert_resource(SpaceState::default())
        .insert_resource(UiState::default())
        .init_gizmo_group::<DashedAxisGizmos>()
        .init_gizmo_group::<SelectedGizmos>()
        .add_systems(Startup, (setup_camera, configure_gizmos))
        .add_systems(
            Update,
            (
                orbit_camera,
                fit_camera,
                update_camera,
                selection_input,
                draw_scene,
            )
                .chain(),
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

    let (selected, _) = store.config_mut::<SelectedGizmos>();
    selected.line.width = 3.5;
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
            let cam_model = cam::build_cam_model(&document);
            let mut analysis = cam_analysis::CamAnalysis::build(&display, &cam_model);
            analysis.set_machine_start(
                &display,
                &cam_model,
                [
                    state.gcode_profile.initial_x,
                    state.gcode_profile.initial_y,
                    state.gcode_profile.initial_z,
                ],
            );
            let trajectory = Some(cam_trajectory::build_trajectory(
                &display,
                &cam_model,
                &analysis,
                state.gcode_profile.rotary_config(),
            ));
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
            state.cam = cam_model;
            state.analysis = analysis;
            state.trajectory = trajectory;
            state.load_version += 1;
            state.fit_dirty = true;

            push_message(state, format!("已加载 {} 个实体", state.entity_count));
            push_message(
                state,
                format!(
                    "生成 {} 个加工单元 | 实体 {} | 图层 {} | {} | {} | {} | 警告 {}",
                    state.cam.units.len(),
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
    ui_state: Res<UiState>,
    mut rig: ResMut<CameraRig>,
) {
    if ui_state.pointer_over_ui {
        return;
    }
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

fn switch_space(
    space: &mut SpaceState,
    selection: &mut SelectionState,
    cam: &cam::CamModel,
    mode: SpaceMode,
) {
    if space.mode == mode {
        return;
    }

    match mode {
        SpaceMode::Cam => {
            selection.cam_selected.clear();
            for entity_id in &selection.selected {
                if let Some(unit_id) = cam.unit_for_entity(*entity_id) {
                    selection.cam_selected.insert(unit_id);
                }
            }
        }
        SpaceMode::Cad => {
            selection.selected.clear();
            for unit_id in &selection.cam_selected {
                if let Some(unit) = cam.unit(*unit_id) {
                    selection.selected.extend(unit.members.iter().copied());
                }
            }
        }
    }

    space.mode = mode;
    selection.drag_start = None;
    selection.drag_current = None;
}

fn space_switch_ui(
    ui: &mut egui::Ui,
    space: &mut SpaceState,
    selection: &mut SelectionState,
    cam: &cam::CamModel,
) {
    let (rect, response) = ui.allocate_exact_size(egui::vec2(112.0, 24.0), egui::Sense::click());
    let t = ui.ctx().animate_bool_with_time(
        egui::Id::new("space_switch"),
        space.mode == SpaceMode::Cam,
        0.18,
    );
    let painter = ui.painter();

    painter.rect_filled(rect, 12.0, egui::Color32::from_rgb(24, 31, 41));
    let inner = rect.shrink(3.0);
    let knob_width = (inner.width() - 4.0) * 0.5;
    let left_x = inner.left() + 2.0;
    let right_x = inner.right() - 2.0 - knob_width;
    let knob_x = left_x + (right_x - left_x) * t;
    let knob = egui::Rect::from_min_size(
        egui::pos2(knob_x, inner.top()),
        egui::vec2(knob_width, inner.height()),
    );
    painter.rect_filled(knob, 9.0, egui::Color32::from_rgb(44, 116, 196));

    let active = egui::Color32::from_rgb(245, 248, 252);
    let inactive = egui::Color32::from_rgb(135, 145, 160);
    let font = egui::FontId::proportional(13.0);
    painter.text(
        egui::pos2(rect.left() + rect.width() * 0.25, rect.center().y),
        egui::Align2::CENTER_CENTER,
        "CAD",
        font.clone(),
        if t < 0.5 { active } else { inactive },
    );
    painter.text(
        egui::pos2(rect.left() + rect.width() * 0.75, rect.center().y),
        egui::Align2::CENTER_CENTER,
        "CAM",
        font,
        if t >= 0.5 { active } else { inactive },
    );

    if response.clicked() {
        let target = if space.mode == SpaceMode::Cad {
            SpaceMode::Cam
        } else {
            SpaceMode::Cad
        };
        switch_space(space, selection, cam, target);
    }
    response.on_hover_cursor(egui::CursorIcon::PointingHand);
}

/// Draw the drawing paths and solid/dashed coordinate axes.
fn draw_scene(
    state: Res<ViewerState>,
    selection: Res<SelectionState>,
    space: Res<SpaceState>,
    mut gizmos: Gizmos,
    mut selected_gizmos: Gizmos<SelectedGizmos>,
    mut negative_axes: Gizmos<DashedAxisGizmos>,
) {
    for path in &state.display.paths {
        let unit = state.cam.unit_for_entity(path.entity_id);
        let is_selected = match space.mode {
            SpaceMode::Cad => selection.selected.contains(&path.entity_id),
            SpaceMode::Cam => unit
                .map(|unit_id| selection.cam_selected.contains(&unit_id))
                .unwrap_or(false),
        };
        let color = if is_selected {
            Color::srgb(1.0, 0.82, 0.22)
        } else {
            Color::srgb(path.color[0], path.color[1], path.color[2])
        };
        for pair in path.points.windows(2) {
            let [x0, y0, z0] = pair[0];
            let [x1, y1, z1] = pair[1];
            let start = Vec3::new(x0 as f32, z0 as f32, -(y0 as f32));
            let end = Vec3::new(x1 as f32, z1 as f32, -(y1 as f32));
            if is_selected {
                selected_gizmos.line(start, end, color);
            } else {
                gizmos.line(start, end, color);
            }
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

/// Mouse selection: click picking, Shift+click add, and AutoCAD-style
/// left-to-right window / right-to-left crossing box selection.
fn selection_input(
    mut contexts: EguiContexts,
    mouse: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    window: Single<&Window, With<PrimaryWindow>>,
    camera: Single<(&Camera, &GlobalTransform), With<MainCamera>>,
    mut state: ResMut<ViewerState>,
    mut selection: ResMut<SelectionState>,
    space: Res<SpaceState>,
    ui_state: Res<UiState>,
) -> Result {
    let valid: BTreeSet<u32> = state
        .display
        .entities
        .iter()
        .map(|entity| entity.id)
        .collect();
    if selection.load_version != state.load_version {
        selection.selected.clear();
        selection.cam_selected.clear();
        selection.drag_start = None;
        selection.drag_current = None;
        selection.reorder_clicks = None;
        selection.load_version = state.load_version;
    }
    selection.selected.retain(|id| valid.contains(id));
    let valid_units: BTreeSet<cam::UnitId> = state.cam.units.iter().map(|unit| unit.id).collect();
    selection.cam_selected.retain(|id| valid_units.contains(id));

    let Some(cursor) = window.cursor_position() else {
        return Ok(());
    };
    let context = contexts.ctx_mut()?;
    let ui_active = ui_state.pointer_over_ui
        || context.egui_wants_pointer_input()
        || context.egui_is_using_pointer();
    if ui_active {
        // A click that starts on the panel resize handle must be owned by the
        // panel, even if the first press frame already reached this system.
        selection.drag_start = None;
        selection.drag_current = None;
        selection.reorder_clicks = None;
        return Ok(());
    }
    let (camera, camera_transform) = camera.into_inner();
    let ctrl = keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight);
    if !ctrl {
        selection.reorder_clicks = None;
    }

    if mouse.just_pressed(MouseButton::Left) {
        if space.mode == SpaceMode::Cam && ctrl {
            if let Some(unit_id) = pick_entity(&state.display, camera, camera_transform, cursor)
                .and_then(|entity_id| state.cam.unit_for_entity(entity_id))
            {
                let mut reorder = selection.reorder_clicks.take();
                if reorder.is_none() && selection.cam_selected.contains(&unit_id) {
                    if let Some((group_id, start, length)) =
                        consecutive_plan_block(&state.analysis, &selection.cam_selected)
                    {
                        reorder = Some(ReorderClicks {
                            group_id,
                            start,
                            length,
                            clicked: Vec::new(),
                        });
                    }
                }
                if let Some(mut active) = reorder {
                    if apply_reorder_click(&mut state, &mut active, unit_id) {
                        selection.reorder_clicks = Some(active);
                        selection.drag_start = None;
                        selection.drag_current = None;
                        return Ok(());
                    }
                }
            }
        }
        selection.drag_start = Some(cursor);
        selection.drag_current = Some(cursor);
    }
    if mouse.pressed(MouseButton::Left) && selection.drag_start.is_some() {
        selection.drag_current = Some(cursor);
    }
    if mouse.just_released(MouseButton::Left) {
        let Some(start) = selection.drag_start.take() else {
            return Ok(());
        };
        let end = selection.drag_current.take().unwrap_or(cursor);
        let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);

        if (end - start).length() < 4.0 {
            match space.mode {
                SpaceMode::Cad => {
                    let hit = pick_entity(&state.display, camera, camera_transform, cursor);
                    if shift {
                        if let Some(id) = hit {
                            selection.selected.insert(id);
                        }
                    } else {
                        selection.selected.clear();
                        if let Some(id) = hit {
                            selection.selected.insert(id);
                        }
                    }
                }
                SpaceMode::Cam => {
                    let hit = pick_entity(&state.display, camera, camera_transform, cursor)
                        .and_then(|entity_id| state.cam.unit_for_entity(entity_id));
                    if shift {
                        if let Some(id) = hit {
                            selection.cam_selected.insert(id);
                        }
                    } else {
                        selection.cam_selected.clear();
                        if let Some(id) = hit {
                            selection.cam_selected.insert(id);
                        }
                    }
                }
            }
        } else {
            match space.mode {
                SpaceMode::Cad => {
                    let chosen = box_select(&state.display, camera, camera_transform, start, end);
                    if shift {
                        selection.selected.extend(chosen);
                    } else {
                        selection.selected = chosen;
                    }
                }
                SpaceMode::Cam => {
                    let chosen = box_select_units(
                        &state.display,
                        &state.cam,
                        camera,
                        camera_transform,
                        start,
                        end,
                    );
                    if shift {
                        selection.cam_selected.extend(chosen);
                    } else {
                        selection.cam_selected = chosen;
                    }
                }
            }
        }
    }
    Ok(())
}

fn to_render_point(point: [f64; 3]) -> Vec3 {
    Vec3::new(point[0] as f32, point[2] as f32, -(point[1] as f32))
}

fn project_entity_point(
    camera: &Camera,
    camera_transform: &GlobalTransform,
    point: [f64; 3],
) -> Option<Vec2> {
    camera
        .world_to_viewport(camera_transform, to_render_point(point))
        .ok()
}

fn pick_entity(
    display: &display::DisplayData,
    camera: &Camera,
    camera_transform: &GlobalTransform,
    cursor: Vec2,
) -> Option<u32> {
    let mut best: Option<(f32, u32)> = None;
    for path in &display.paths {
        let projected: Vec<Vec2> = path
            .points
            .iter()
            .filter_map(|point| project_entity_point(camera, camera_transform, *point))
            .collect();
        for pair in projected.windows(2) {
            let distance = distance_to_segment(cursor, pair[0], pair[1]);
            if distance <= PICK_TOLERANCE
                && best
                    .map(|(best_distance, _)| distance < best_distance)
                    .unwrap_or(true)
            {
                best = Some((distance, path.entity_id));
            }
        }
    }
    best.map(|(_, id)| id)
}

fn box_select(
    display: &display::DisplayData,
    camera: &Camera,
    camera_transform: &GlobalTransform,
    start: Vec2,
    end: Vec2,
) -> BTreeSet<u32> {
    let rect = egui::Rect::from_two_pos(egui::pos2(start.x, start.y), egui::pos2(end.x, end.y));
    let window_mode = end.x >= start.x;
    let mut selected = BTreeSet::new();

    for path in &display.paths {
        let projected: Vec<Vec2> = path
            .points
            .iter()
            .filter_map(|point| project_entity_point(camera, camera_transform, *point))
            .collect();
        if projected.len() < 2 {
            continue;
        }

        let hit = if window_mode {
            projected
                .iter()
                .all(|point| rect.contains(egui::pos2(point.x, point.y)))
        } else {
            projected
                .iter()
                .any(|point| rect.contains(egui::pos2(point.x, point.y)))
                || projected
                    .windows(2)
                    .any(|pair| segment_intersects_rect(pair[0], pair[1], rect))
        };
        if hit {
            selected.insert(path.entity_id);
        }
    }
    selected
}

fn box_select_units(
    display: &display::DisplayData,
    cam: &cam::CamModel,
    camera: &Camera,
    camera_transform: &GlobalTransform,
    start: Vec2,
    end: Vec2,
) -> BTreeSet<cam::UnitId> {
    let rect = egui::Rect::from_two_pos(egui::pos2(start.x, start.y), egui::pos2(end.x, end.y));
    let window_mode = end.x >= start.x;
    let mut selected = BTreeSet::new();

    for unit in &cam.units {
        let mut has_projection = false;
        let mut all_inside = true;
        let mut intersects = false;

        for path in &display.paths {
            if cam.entity_to_unit.get(&path.entity_id) != Some(&unit.id) {
                continue;
            }
            let projected: Vec<Vec2> = path
                .points
                .iter()
                .filter_map(|point| project_entity_point(camera, camera_transform, *point))
                .collect();
            if projected.len() < 2 {
                continue;
            }
            has_projection = true;
            all_inside &= projected
                .iter()
                .all(|point| rect.contains(egui::pos2(point.x, point.y)));
            intersects |= projected
                .iter()
                .any(|point| rect.contains(egui::pos2(point.x, point.y)))
                || projected
                    .windows(2)
                    .any(|pair| segment_intersects_rect(pair[0], pair[1], rect));
        }

        let hit = has_projection && if window_mode { all_inside } else { intersects };
        if hit {
            selected.insert(unit.id);
        }
    }

    selected
}

fn consecutive_plan_block(
    analysis: &cam_analysis::CamAnalysis,
    selected: &BTreeSet<cam::UnitId>,
) -> Option<(u32, usize, usize)> {
    if selected.is_empty() {
        return None;
    }
    let mut group_id = None;
    let mut entries = Vec::new();
    for group in &analysis.plan.groups {
        for (index, unit) in group.units.iter().enumerate() {
            if !selected.contains(&unit.unit_id) {
                continue;
            }
            if unit.machining_cut {
                return None;
            }
            if let Some(existing) = group_id {
                if existing != group.group_id {
                    return None;
                }
            } else {
                group_id = Some(group.group_id);
            }
            entries.push((index, unit.order));
        }
    }
    if entries.len() != selected.len() {
        return None;
    }
    entries.sort_by_key(|(_, order)| *order);
    if entries.windows(2).any(|pair| pair[1].1 != pair[0].1 + 1) {
        return None;
    }
    let start = entries.iter().map(|(index, _)| *index).min()?;
    let end = entries.iter().map(|(index, _)| *index).max()?;
    if end + 1 - start != entries.len() {
        return None;
    }
    Some((group_id?, start, entries.len()))
}

fn plan_unit_location(
    analysis: &cam_analysis::CamAnalysis,
    unit_id: cam::UnitId,
) -> Option<(u32, usize)> {
    for group in &analysis.plan.groups {
        for (index, unit) in group.units.iter().enumerate() {
            if unit.unit_id == unit_id {
                return Some((group.group_id, index));
            }
        }
    }
    None
}

fn apply_reorder_click(
    state: &mut ViewerState,
    reorder: &mut ReorderClicks,
    unit_id: cam::UnitId,
) -> bool {
    if reorder.clicked.contains(&unit_id) {
        return true;
    }
    let Some((group_id, current)) = plan_unit_location(&state.analysis, unit_id) else {
        return false;
    };
    if group_id != reorder.group_id
        || current < reorder.start
        || current >= reorder.start + reorder.length
    {
        return false;
    }
    let target = reorder.start + reorder.clicked.len();
    if target >= reorder.start + reorder.length {
        return false;
    }
    if current != target {
        let ViewerState {
            display,
            cam,
            analysis,
            ..
        } = &mut *state;
        if !analysis.move_plan_unit_to(display, cam, group_id, current, target) {
            return false;
        }
    }
    reorder.clicked.push(unit_id);
    rebuild_state_trajectory(state);
    refresh_gcode_preview(state);
    true
}

fn distance_to_segment(point: Vec2, start: Vec2, end: Vec2) -> f32 {
    let segment = end - start;
    let length_squared = segment.length_squared();
    if length_squared <= f32::EPSILON {
        return point.distance(start);
    }
    let t = ((point - start).dot(segment) / length_squared).clamp(0.0, 1.0);
    point.distance(start + segment * t)
}

fn segment_intersects_rect(start: Vec2, end: Vec2, rect: egui::Rect) -> bool {
    let min = Vec2::new(rect.min.x, rect.min.y);
    let max = Vec2::new(rect.max.x, rect.max.y);
    if rect.contains(egui::pos2(start.x, start.y)) || rect.contains(egui::pos2(end.x, end.y)) {
        return true;
    }
    let corners = [min, Vec2::new(max.x, min.y), max, Vec2::new(min.x, max.y)];
    (0..4).any(|index| segments_intersect(start, end, corners[index], corners[(index + 1) % 4]))
}

fn segments_intersect(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    fn cross(a: Vec2, b: Vec2, c: Vec2) -> f32 {
        (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
    }
    let ab_c = cross(a, b, c);
    let ab_d = cross(a, b, d);
    let cd_a = cross(c, d, a);
    let cd_b = cross(c, d, b);
    ab_c * ab_d < 0.0 && cd_a * cd_b < 0.0
}

fn entity_info_ui(ui: &mut egui::Ui, entity: &display::DisplayEntity, scope: &str) {
    ui.horizontal(|ui| {
        ui.colored_label(color32(entity.color), "■");
        ui.strong(format!("{}  #{}", entity.kind, entity.id));
    });
    ui.add_space(4.0);
    ui.label(egui::RichText::new("基本信息").strong());

    egui::Grid::new(format!("common_{scope}_{}", entity.id))
        .num_columns(2)
        .striped(true)
        .spacing([10.0, 4.0])
        .show(ui, |ui| {
            ui.label("类型");
            ui.label(entity.kind);
            ui.end_row();
            ui.label("实体 ID");
            ui.label(entity.id.to_string());
            ui.end_row();
            ui.label("图层");
            ui.label(&entity.layer);
            ui.end_row();
            ui.label("图层颜色");
            color_label(ui, entity.layer_color);
            ui.end_row();
            ui.label("图元颜色");
            color_label(ui, entity.color);
            ui.end_row();
        });

    ui.add_space(6.0);
    ui.label(egui::RichText::new("几何信息").strong());
    egui::Grid::new(format!("geometry_{scope}_{}", entity.id))
        .num_columns(2)
        .striped(true)
        .spacing([10.0, 4.0])
        .show(ui, |ui| {
            for (label, value) in &entity.properties {
                ui.label(*label);
                ui.label(value);
                ui.end_row();
            }
        });
}

#[derive(Default)]
struct CamAnalysisUiAction {
    apply_tube: Option<usize>,
    manual_tube: Option<cam_analysis::TubeSection>,
    clear_tube: bool,
    apply_machining: Option<usize>,
    manual_machining: Option<cam::UnitId>,
    remove_machining: Option<cam::UnitId>,
    move_unit: Option<(u32, usize, isize)>,
    set_lazy_rotation: Option<bool>,
    set_rotation_direction: Option<cam_analysis::RotationDirection>,
}

fn cam_analysis_ui(
    ui: &mut egui::Ui,
    analysis: &cam_analysis::CamAnalysis,
    trajectory: Option<&cam_trajectory::MachineTrajectory>,
    manual_tube: &mut cam_analysis::TubeSection,
) -> CamAnalysisUiAction {
    let mut action = CamAnalysisUiAction::default();

    egui::CollapsingHeader::new("方管截面")
        .default_open(true)
        .show(ui, |ui| {
            if let Some(section) = &analysis.tube_section {
                egui::Grid::new("tube_section_table")
                    .num_columns(2)
                    .striped(true)
                    .spacing([8.0, 3.0])
                    .show(ui, |ui| {
                        ui.label("中心 Y");
                        ui.label(format!("{:.3}", section.center_y));
                        ui.end_row();
                        ui.label("中心 Z");
                        ui.label(format!("{:.3}", section.center_z));
                        ui.end_row();
                        ui.label("Y 尺寸");
                        ui.label(format!("{:.3}", section.size_y));
                        ui.end_row();
                        ui.label("Z 尺寸");
                        ui.label(format!("{:.3}", section.size_z));
                        ui.end_row();
                        ui.label("圆角");
                        ui.label(format!("{:.3}", section.radius));
                        ui.end_row();
                        ui.label("来源");
                        ui.label(match section.source {
                            cam_analysis::SectionSource::Auto => "自动",
                            cam_analysis::SectionSource::Manual => "手动",
                        });
                        ui.end_row();
                    });
                if ui.small_button("清除/重新识别").clicked() {
                    action.clear_tube = true;
                }
                ui.separator();
                ui.label("手动参数");
                let mut edited = section.clone();
                ui.horizontal(|ui| {
                    ui.label("中心 Y");
                    ui.add(egui::DragValue::new(&mut edited.center_y).speed(0.1));
                    ui.label("Z");
                    ui.add(egui::DragValue::new(&mut edited.center_z).speed(0.1));
                });
                ui.horizontal(|ui| {
                    ui.label("尺寸 Y");
                    ui.add(egui::DragValue::new(&mut edited.size_y).speed(0.1));
                    ui.label("Z");
                    ui.add(egui::DragValue::new(&mut edited.size_z).speed(0.1));
                });
                ui.horizontal(|ui| {
                    ui.label("圆角");
                    ui.add(egui::DragValue::new(&mut edited.radius).speed(0.1));
                    if ui.small_button("应用手动参数").clicked() {
                        action.manual_tube = Some(edited);
                    }
                });
            } else {
                if analysis.tube_candidates.is_empty() {
                    ui.label("未识别到方管截面候选");
                } else {
                    ui.label(format!(
                        "候选 {} 个，请选择",
                        analysis.tube_candidates.len()
                    ));
                    for (index, candidate) in analysis.tube_candidates.iter().take(8).enumerate() {
                        ui.horizontal(|ui| {
                            ui.label(format!(
                                "Y{:.2} Z{:.2} {:.2}×{:.2} R{:.2}",
                                candidate.center_y,
                                candidate.center_z,
                                candidate.size_y,
                                candidate.size_z,
                                candidate.radius
                            ));
                            if ui.small_button("应用").clicked() {
                                action.apply_tube = Some(index);
                            }
                        });
                    }
                }
                ui.separator();
                ui.label("无截面时可直接按加工单元导出；也可手动指定方管截面。");
                ui.label("手动截面参数");
                ui.horizontal(|ui| {
                    ui.label("中心 Y");
                    ui.add(egui::DragValue::new(&mut manual_tube.center_y).speed(0.1));
                    ui.label("中心 Z");
                    ui.add(egui::DragValue::new(&mut manual_tube.center_z).speed(0.1));
                });
                ui.horizontal(|ui| {
                    ui.label("方管宽 Y");
                    ui.add(egui::DragValue::new(&mut manual_tube.size_y).speed(0.1));
                    ui.label("方管高 Z");
                    ui.add(egui::DragValue::new(&mut manual_tube.size_z).speed(0.1));
                });
                ui.horizontal(|ui| {
                    ui.label("圆角");
                    ui.add(egui::DragValue::new(&mut manual_tube.radius).speed(0.1));
                    if ui.small_button("应用手动截面").clicked() {
                        action.manual_tube = Some(manual_tube.clone());
                    }
                });
            }
        });

    if analysis.tube_section.is_some() || !analysis.plan.groups.is_empty() {
        egui::CollapsingHeader::new("加工断面与区间")
            .default_open(true)
            .show(ui, |ui| {
                if analysis.tube_section.is_none() {
                    ui.label("未识别方管截面：当前按加工单元直接排序和导出。");
                    ui.separator();
                }
                if analysis.machining_candidates.is_empty() {
                    ui.label("未识别到加工断面候选");
                } else {
                    let existing: BTreeSet<cam::UnitId> = analysis
                        .machining_sections
                        .iter()
                        .map(|section| section.unit_id)
                        .collect();
                    for (index, candidate) in
                        analysis.machining_candidates.iter().take(8).enumerate()
                    {
                        if existing.contains(&candidate.unit_id) {
                            continue;
                        }
                        ui.horizontal(|ui| {
                            ui.label(format!(
                                "X {:.2}..{:.2} 绕数 {} 缺口 {} 偏差 {:.2}",
                                candidate.x_min,
                                candidate.x_max,
                                candidate.winding,
                                candidate.coverage_gap_bins,
                                candidate.maximum_deviation
                            ));
                            if ui.small_button("应用").clicked() {
                                action.apply_machining = Some(index);
                            }
                        });
                    }
                }

                if !analysis.machining_sections.is_empty() {
                    ui.separator();
                    ui.label(format!(
                        "已应用加工断面 {} 个",
                        analysis.machining_sections.len()
                    ));
                    for section in &analysis.machining_sections {
                        let covered = section.zone_mask.iter().filter(|value| **value).count();
                        ui.label(format!(
                            "X {:.2}..{:.2}  绕数 {}  成员 {}  区位 {}",
                            section.x_min,
                            section.x_max,
                            section.winding,
                            section.boundary_units.len(),
                            covered
                        ));
                    }
                }

                if !analysis.intervals.is_empty() {
                    ui.separator();
                    ui.label("加工区间");
                    for interval in &analysis.intervals {
                        ui.label(format!(
                            "{}: X {:.2}..{:.2}  {} 个单元",
                            interval.id + 1,
                            interval.x_start,
                            interval.x_end,
                            interval.units.len()
                        ));
                    }
                }

                if !analysis.groups.is_empty() {
                    ui.separator();
                    ui.label("断面加工组");
                    for group in &analysis.groups {
                        let title = format!(
                            "组 {} · X {:.2}..{:.2} · {} 个单元",
                            group.id + 1,
                            group.x_start,
                            group.x_end,
                            group.units.len()
                        );
                        egui::CollapsingHeader::new(title)
                            .id_salt(("machining_group", group.id))
                            .default_open(false)
                            .show(ui, |ui| {
                                if let Some(section_unit) = group.section_unit {
                                    let section_index = analysis
                                        .machining_sections
                                        .iter()
                                        .position(|section| section.unit_id == section_unit);
                                    ui.label(format!(
                                        "组末加工断面：{}",
                                        section_index
                                            .map(|index| format!("#{}", index + 1))
                                            .unwrap_or_else(|| "未编号".to_string())
                                    ));
                                }
                                for zone_group in &group.zone_groups {
                                    if zone_group.units.is_empty() {
                                        continue;
                                    }
                                    ui.label(format!(
                                        "{}：{} 个单元",
                                        zone_group.zone.label(),
                                        zone_group.units.len()
                                    ));
                                }
                            });
                    }
                }

                if !analysis.plan.groups.is_empty() {
                    ui.separator();
                    ui.label("排序策略");
                    let mut lazy = analysis.lazy_rotation;
                    if ui.checkbox(&mut lazy, "懒旋转").changed() {
                        action.set_lazy_rotation = Some(lazy);
                    }
                    let mut direction = analysis.rotation_direction;
                    ui.horizontal(|ui| {
                        if ui
                            .radio_value(
                                &mut direction,
                                cam_analysis::RotationDirection::CounterClockwise,
                                "逆时针",
                            )
                            .changed()
                        {
                            action.set_rotation_direction = Some(direction);
                        }
                        if ui
                            .radio_value(
                                &mut direction,
                                cam_analysis::RotationDirection::Clockwise,
                                "顺时针",
                            )
                            .changed()
                        {
                            action.set_rotation_direction = Some(direction);
                        }
                    });
                    ui.label(
                        "懒旋转：完成一个区位后再转下一个区位；关闭后按 YZ 平面沿 +X 扫描顺序。",
                    );
                    ui.label(
                        "落刀点：懒旋转优先局部北极点，非懒旋转取 +X 极点；开放路径只选端点。",
                    );
                    ui.separator();
                    ui.label("加工顺序（预览）");
                    ui.label(
                        "框选连续编号的加工单元后，按住 Ctrl 依次点击单元，点击顺序即最终顺序。",
                    );
                    ui.label("加工断面固定在组末，不参与排序。");
                    for plan_group in &analysis.plan.groups {
                        let title = format!(
                            "组 {} · X {:.2}..{:.2}",
                            plan_group.group_id + 1,
                            plan_group.x_start,
                            plan_group.x_end
                        );
                        egui::CollapsingHeader::new(title)
                            .id_salt(("plan_group", plan_group.group_id))
                            .default_open(false)
                            .show(ui, |ui| {
                                for (unit_index, unit) in plan_group.units.iter().enumerate() {
                                    let kind = if unit.machining_cut {
                                        "断面".to_string()
                                    } else {
                                        unit.zone.label().to_string()
                                    };
                                    let direction = match unit.direction {
                                        cam_analysis::UnitDirection::Forward => "正向",
                                        cam_analysis::UnitDirection::Reverse => "反向",
                                    };
                                    ui.horizontal(|ui| {
                                        if !unit.machining_cut {
                                            let can_up = unit_index > 0
                                                && !plan_group.units[unit_index - 1].machining_cut;
                                            let can_down = unit_index + 1 < plan_group.units.len()
                                                && !plan_group.units[unit_index + 1].machining_cut;
                                            if ui
                                                .add_enabled(can_up, egui::Button::new("↑").small())
                                                .clicked()
                                            {
                                                action.move_unit =
                                                    Some((plan_group.group_id, unit_index, -1));
                                            }
                                            if ui
                                                .add_enabled(
                                                    can_down,
                                                    egui::Button::new("↓").small(),
                                                )
                                                .clicked()
                                            {
                                                action.move_unit =
                                                    Some((plan_group.group_id, unit_index, 1));
                                            }
                                        }
                                        ui.add(
                                            egui::Label::new(format!(
                                                "{}. {}  #{:08X}  {}  起点 ({:.2}, {:.2}, {:.2})",
                                                unit.order,
                                                kind,
                                                unit.unit_id.0 as u32,
                                                direction,
                                                unit.start_point[0],
                                                unit.start_point[1],
                                                unit.start_point[2]
                                            ))
                                            .wrap(),
                                        );
                                    });
                                }
                            });
                    }
                }
            });

        if let Some(trajectory) = trajectory {
            if trajectory.moves.is_empty() {
                ui.label("四轴轨迹：暂无");
            } else {
                egui::CollapsingHeader::new("四轴轨迹（预览）")
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.label(format!("加工单元：{}", trajectory.unit_count));
                        ui.label(format!("运动段：{}", trajectory.moves.len()));
                        ui.label(format!("切削段：{}", trajectory.cutting_move_count));
                        ui.label(format!("旋转安全 Z：{:.3}", trajectory.safe_z));
                        ui.label(format!(
                            "A 范围：{:.2}° .. {:.2}°",
                            trajectory.a_min, trajectory.a_max
                        ));
                        if let Some(last) = trajectory.moves.last() {
                            ui.label(format!(
                                "末姿态 X{:.2} Y{:.2} Z{:.2} A{:.2}",
                                last.pose.x, last.pose.y, last.pose.z, last.pose.a
                            ));
                        }
                    });
            }
        }
    }

    action
}

fn cam_unit_info_ui(
    ui: &mut egui::Ui,
    unit: &cam::MachiningUnit,
    display: &display::DisplayData,
    zone_profile: Option<&cam_analysis::UnitZoneProfile>,
    machining_section_index: Option<usize>,
    machining_candidate: bool,
    scope: &str,
) {
    ui.horizontal(|ui| {
        ui.strong("加工单元");
    });
    ui.add_space(4.0);
    ui.label(egui::RichText::new("基本信息").strong());
    egui::Grid::new(format!("unit_common_{scope}_{:016X}", unit.id.0))
        .num_columns(2)
        .striped(true)
        .spacing([10.0, 4.0])
        .show(ui, |ui| {
            ui.label("单元 ID");
            ui.label(format!("{:016X}", unit.id.0));
            ui.end_row();
            ui.label("类型");
            ui.label(if unit.closed { "闭合" } else { "开放" });
            ui.end_row();
            ui.label("成员数");
            ui.label(unit.members.len().to_string());
            ui.end_row();
            ui.label("加工断面");
            ui.label(match machining_section_index {
                Some(index) => format!("是（#{}）", index + 1),
                None if machining_candidate => "候选（未应用）".to_string(),
                None => "否".to_string(),
            });
            ui.end_row();
            ui.label("显示颜色");
            ui.label("跟随成员图元");
            ui.end_row();
        });

    if let Some(profile) = zone_profile {
        ui.add_space(6.0);
        ui.label(egui::RichText::new("加工区位").strong());
        egui::Grid::new(format!("unit_zones_{scope}_{:016X}", unit.id.0))
            .num_columns(2)
            .striped(true)
            .spacing([10.0, 4.0])
            .show(ui, |ui| {
                ui.label("主区位");
                ui.label(
                    profile
                        .owner_zone
                        .map(|zone| zone.label())
                        .unwrap_or("未判定"),
                );
                ui.end_row();
                ui.label("主区位投影长度");
                ui.label(
                    profile
                        .owner_zone
                        .map(|zone| format!("{:.3}", profile.zone_lengths[zone.index()]))
                        .unwrap_or_else(|| "—".to_string()),
                );
                ui.end_row();
            });

        let touched: Vec<&'static str> = cam_analysis::TubeZone::ALL
            .iter()
            .filter(|zone| profile.touched_zones[zone.index()])
            .map(|zone| zone.label())
            .collect();
        ui.label("经过区位");
        ui.add(
            egui::Label::new(if touched.is_empty() {
                "—".to_string()
            } else {
                touched.join("、")
            })
            .wrap(),
        );
    }

    ui.add_space(6.0);
    ui.label(egui::RichText::new("成员图元").strong());
    egui::Grid::new(format!("unit_members_{scope}_{:016X}", unit.id.0))
        .num_columns(3)
        .striped(true)
        .spacing([10.0, 4.0])
        .show(ui, |ui| {
            for member in &unit.members {
                if let Some(entity) = display.entities.iter().find(|entity| entity.id == *member) {
                    ui.label(entity.kind);
                    ui.label(format!("#{}", entity.id));
                    ui.label(&entity.layer);
                    ui.end_row();
                }
            }
        });
}

fn color_label(ui: &mut egui::Ui, rgb: [f32; 3]) {
    ui.horizontal(|ui| {
        ui.colored_label(color32(rgb), "■");
        ui.label(format!(
            "#{:02X}{:02X}{:02X}",
            (rgb[0] * 255.0).round() as u8,
            (rgb[1] * 255.0).round() as u8,
            (rgb[2] * 255.0).round() as u8
        ));
    });
}

fn color32(rgb: [f32; 3]) -> egui::Color32 {
    egui::Color32::from_rgb(
        (rgb[0] * 255.0).round() as u8,
        (rgb[1] * 255.0).round() as u8,
        (rgb[2] * 255.0).round() as u8,
    )
}

fn draw_machining_order_labels(
    context: &egui::Context,
    state: &ViewerState,
    camera: &Camera,
    camera_transform: &GlobalTransform,
    blocked: &[egui::Rect],
) {
    if state.analysis.plan.groups.is_empty() {
        return;
    }
    let painter = context.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("machining_order_labels"),
    ));
    let viewport = context.viewport_rect();
    let blue = egui::Color32::from_rgb(64, 146, 255);
    let orange = egui::Color32::from_rgb(255, 170, 64);

    for group in &state.analysis.plan.groups {
        for unit in &group.units {
            // Draw the entry arrow independently of the unit-center label, so
            // a section whose center is off-screen or under a panel still gets
            // its start marker when the start itself is visible.
            if let Some(path) = cam_analysis::oriented_unit_path_from(
                &state.display,
                &state.cam,
                unit.unit_id,
                unit.direction,
                Some(unit.start_point),
            ) {
                if let Some(start_point) = path.first() {
                    if let Ok(start_screen) = camera.world_to_viewport(
                        camera_transform,
                        Vec3::new(
                            start_point[0] as f32,
                            start_point[2] as f32,
                            -(start_point[1] as f32),
                        ),
                    ) {
                        let start = egui::pos2(start_screen.x, start_screen.y);
                        let mut direction = None;
                        for point in path.iter().skip(1) {
                            if let Ok(next_screen) = camera.world_to_viewport(
                                camera_transform,
                                Vec3::new(point[0] as f32, point[2] as f32, -(point[1] as f32)),
                            ) {
                                let delta = egui::vec2(
                                    next_screen.x - start_screen.x,
                                    next_screen.y - start_screen.y,
                                );
                                if delta.length() > 2.0 {
                                    direction = Some(delta.normalized());
                                    break;
                                }
                            }
                        }
                        if let Some(direction) = direction {
                            let tip = start + direction * 18.0;
                            let free = viewport.contains(start)
                                && viewport.contains(tip)
                                && !blocked.iter().any(|rect| rect.expand(4.0).contains(start));
                            if free {
                                let stroke = egui::Stroke::new(2.0, orange);
                                let perpendicular = egui::vec2(-direction.y, direction.x);
                                painter.circle_filled(start, 3.0, orange);
                                painter.line_segment([start, tip], stroke);
                                painter.line_segment(
                                    [tip, tip - direction * 7.0 + perpendicular * 4.5],
                                    stroke,
                                );
                                painter.line_segment(
                                    [tip, tip - direction * 7.0 - perpendicular * 4.5],
                                    stroke,
                                );
                            }
                        }
                    }
                }
            }

            let Some(center) = unit_center(&state.display, &state.cam, unit.unit_id) else {
                continue;
            };
            let world = Vec3::new(center[0] as f32, center[2] as f32, -(center[1] as f32));
            let Ok(screen) = camera.world_to_viewport(camera_transform, world) else {
                continue;
            };
            let position = egui::pos2(screen.x, screen.y);
            if !viewport.contains(position)
                || blocked
                    .iter()
                    .any(|rect| rect.expand(4.0).contains(position))
            {
                continue;
            }

            if !unit.machining_cut {
                painter.circle_filled(position, 2.5, blue);
            }
            let galley = painter.layout_no_wrap(
                unit.order.to_string(),
                egui::FontId::monospace(12.0),
                egui::Color32::WHITE,
            );
            let anchor = egui::pos2(position.x + 7.0, position.y - galley.size().y * 0.5);
            let background = egui::Rect::from_min_size(
                anchor - egui::vec2(3.0, 2.0),
                galley.size() + egui::vec2(6.0, 4.0),
            );
            if blocked
                .iter()
                .any(|rect| rect.expand(4.0).contains(background.center()))
            {
                continue;
            }
            painter.rect_filled(
                background,
                3.0,
                egui::Color32::from_rgba_unmultiplied(8, 12, 18, 210),
            );
            painter.galley(anchor, galley, egui::Color32::WHITE);
        }
    }
}

fn draw_machining_section_labels(
    context: &egui::Context,
    state: &ViewerState,
    camera: &Camera,
    camera_transform: &GlobalTransform,
    blocked: &[egui::Rect],
) {
    if state.analysis.machining_sections.is_empty() {
        return;
    }
    let painter = context.layer_painter(egui::LayerId::new(
        egui::Order::Foreground,
        egui::Id::new("machining_section_labels"),
    ));
    let viewport = context.viewport_rect();

    for (index, section) in state.analysis.machining_sections.iter().enumerate() {
        let Some(center) = unit_center(&state.display, &state.cam, section.unit_id) else {
            continue;
        };
        let world = Vec3::new(center[0] as f32, center[2] as f32, -(center[1] as f32));
        let Ok(screen) = camera.world_to_viewport(camera_transform, world) else {
            continue;
        };
        let position = egui::pos2(screen.x, screen.y);
        if !viewport.contains(position)
            || blocked
                .iter()
                .any(|rect| rect.expand(4.0).contains(position))
        {
            continue;
        }

        painter.circle_filled(position, 3.0, egui::Color32::from_rgb(255, 214, 80));
        let text = format!("#{}", index + 1);
        let galley =
            painter.layout_no_wrap(text, egui::FontId::monospace(14.0), egui::Color32::WHITE);
        let anchor = egui::pos2(
            position.x - 7.0 - galley.size().x,
            position.y - galley.size().y * 0.5,
        );
        let background = egui::Rect::from_min_size(
            anchor - egui::vec2(3.0, 2.0),
            galley.size() + egui::vec2(6.0, 4.0),
        );
        if blocked
            .iter()
            .any(|rect| rect.expand(4.0).contains(background.center()))
        {
            continue;
        }
        painter.rect_filled(
            background,
            3.0,
            egui::Color32::from_rgba_unmultiplied(8, 12, 18, 210),
        );
        painter.galley(anchor, galley, egui::Color32::WHITE);
    }
}

fn unit_center(
    display: &display::DisplayData,
    cam: &cam::CamModel,
    unit_id: cam::UnitId,
) -> Option<[f64; 3]> {
    let mut sum = [0.0f64; 3];
    let mut count = 0usize;
    for path in &display.paths {
        if cam.unit_for_entity(path.entity_id) != Some(unit_id) {
            continue;
        }
        for point in &path.points {
            sum[0] += point[0];
            sum[1] += point[1];
            sum[2] += point[2];
            count += 1;
        }
    }
    if count == 0 {
        return None;
    }
    Some([
        sum[0] / count as f64,
        sum[1] / count as f64,
        sum[2] / count as f64,
    ])
}

fn drawing_ui(
    mut contexts: EguiContexts,
    mut state: ResMut<ViewerState>,
    mut camera: ResMut<CameraRig>,
    mut selection: ResMut<SelectionState>,
    mut space: ResMut<SpaceState>,
    view_camera: Single<(&Camera, &GlobalTransform), With<MainCamera>>,
    mut fonts_loaded: Local<bool>,
    mut ui_state: ResMut<UiState>,
) -> Result {
    let context = contexts.ctx_mut()?;
    install_cjk_font(context, &mut fonts_loaded);
    context.set_visuals(egui::Visuals::dark());

    let mut open_dialog = false;
    let mut export_gcode = false;
    let mut quick_export_gcode = false;
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

    let menu_panel = egui::Panel::top("menu_bar").show(&mut root, |ui| {
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
                        if ui.button("导出 G 代码").clicked() {
                            export_gcode = true;
                            ui.close();
                        }
                        if ui.button("G代码配置与预览").clicked() {
                            state.gcode_settings_open = true;
                            ui.close();
                        }
                        if ui.button("快速导出到 build").clicked() {
                            quick_export_gcode = true;
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
                    ui.separator();
                    space_switch_ui(ui, &mut space, &mut selection, &state.cam);
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
    let menu_rect = menu_panel.response.rect;
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

    let mut cam_action = CamAnalysisUiAction::default();
    let feature_panel = egui::Panel::left("feature_info")
        .default_size(300.0)
        .resizable(true)
        .show(&mut root, |ui| {
            ui.add_space(6.0);
            ui.heading("特征");
            ui.separator();

            match space.mode {
                SpaceMode::Cad => {
                    let selected_entities: Vec<&display::DisplayEntity> = state
                        .display
                        .entities
                        .iter()
                        .filter(|entity| selection.selected.contains(&entity.id))
                        .collect();

                    if selected_entities.is_empty() {
                        ui.label("未选择图元");
                        ui.label("左键点选 / 拖拽框选");
                        ui.label("Shift + 左键追加选择");
                    } else {
                        ui.label(format!("已选择 {} 个图元", selected_entities.len()));
                        ui.separator();
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            if selected_entities.len() == 1 {
                                entity_info_ui(ui, selected_entities[0], "single");
                            } else {
                                for entity in selected_entities {
                                    egui::CollapsingHeader::new(format!(
                                        "{}  #{}",
                                        entity.kind, entity.id
                                    ))
                                    .id_salt(entity.id)
                                    .default_open(false)
                                    .show(ui, |ui| entity_info_ui(ui, entity, "multi"));
                                }
                            }
                        });
                    }
                }
                SpaceMode::Cam => {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        let ViewerState {
                            analysis,
                            trajectory,
                            manual_tube,
                            ..
                        } = &mut *state;
                        cam_action =
                            cam_analysis_ui(ui, analysis, trajectory.as_ref(), manual_tube);
                        ui.separator();

                        let selected_units: Vec<&cam::MachiningUnit> = state
                            .cam
                            .units
                            .iter()
                            .filter(|unit| selection.cam_selected.contains(&unit.id))
                            .collect();

                        if selected_units.is_empty() {
                            ui.label("未选择加工单元");
                            ui.label("左键点选 / 拖拽框选");
                            ui.label("Shift + 左键追加选择");
                        } else {
                            ui.label(format!("已选择 {} 个加工单元", selected_units.len()));
                            ui.separator();
                            if selected_units.len() == 1 {
                                let zone_profile =
                                    state.analysis.zone_profiles.get(&selected_units[0].id);
                                let section_index = state
                                    .analysis
                                    .machining_sections
                                    .iter()
                                    .position(|section| section.unit_id == selected_units[0].id);
                                let machining_candidate = state
                                    .analysis
                                    .machining_candidates
                                    .iter()
                                    .any(|candidate| candidate.unit_id == selected_units[0].id);
                                cam_unit_info_ui(
                                    ui,
                                    selected_units[0],
                                    &state.display,
                                    zone_profile,
                                    section_index,
                                    machining_candidate,
                                    "single",
                                );
                                ui.separator();
                                if section_index.is_some() {
                                    if ui.button("取消加工断面").clicked() {
                                        cam_action.remove_machining = Some(selected_units[0].id);
                                    }
                                } else if selected_units[0].closed
                                    && state.analysis.tube_section.is_some()
                                {
                                    if ui.button("设为加工断面").clicked() {
                                        cam_action.manual_machining = Some(selected_units[0].id);
                                    }
                                }
                            } else {
                                for unit in selected_units {
                                    egui::CollapsingHeader::new(format!(
                                        "加工单元  #{:016X}",
                                        unit.id.0
                                    ))
                                    .id_salt(unit.id)
                                    .default_open(false)
                                    .show(ui, |ui| {
                                        let zone_profile =
                                            state.analysis.zone_profiles.get(&unit.id);
                                        let section_index = state
                                            .analysis
                                            .machining_sections
                                            .iter()
                                            .position(|section| section.unit_id == unit.id);
                                        let machining_candidate = state
                                            .analysis
                                            .machining_candidates
                                            .iter()
                                            .any(|candidate| candidate.unit_id == unit.id);
                                        cam_unit_info_ui(
                                            ui,
                                            unit,
                                            &state.display,
                                            zone_profile,
                                            section_index,
                                            machining_candidate,
                                            "multi",
                                        )
                                    });
                                }
                            }
                        }
                    });
                }
            }
        });

    let feature_rect = feature_panel.response.rect;

    let mut cam_input_changed = false;
    {
        let ViewerState {
            display,
            cam,
            analysis,
            ..
        } = &mut *state;
        if cam_action.clear_tube {
            analysis.clear_tube_section(display, cam);
            cam_input_changed = true;
        }
        if let Some(index) = cam_action.apply_tube {
            analysis.apply_tube_candidate(index, display, cam);
            cam_input_changed = true;
        }
        if let Some(section) = cam_action.manual_tube {
            analysis.apply_manual_tube(section, display, cam);
            cam_input_changed = true;
        }
        if let Some(index) = cam_action.apply_machining {
            analysis.apply_machining_candidate(index, display, cam);
            cam_input_changed = true;
        }
        if let Some(unit_id) = cam_action.manual_machining {
            if analysis.apply_manual_machining_section(unit_id, display, cam) {
                cam_input_changed = true;
            }
        }
        if let Some(unit_id) = cam_action.remove_machining {
            if analysis.remove_machining_section(unit_id, display, cam) {
                cam_input_changed = true;
            }
        }
        if let Some((group_id, index, delta)) = cam_action.move_unit {
            if analysis.move_plan_unit(display, cam, group_id, index, delta) {
                cam_input_changed = true;
            }
        }
        if let Some(lazy) = cam_action.set_lazy_rotation {
            if analysis.set_plan_options(lazy, analysis.rotation_direction, display, cam) {
                cam_input_changed = true;
            }
        }
        if let Some(direction) = cam_action.set_rotation_direction {
            if analysis.set_plan_options(analysis.lazy_rotation, direction, display, cam) {
                cam_input_changed = true;
            }
        }
    }
    if cam_input_changed {
        let ViewerState {
            display,
            cam,
            analysis,
            trajectory,
            gcode_profile,
            ..
        } = &mut *state;
        *trajectory = Some(cam_trajectory::build_trajectory(
            display,
            cam,
            analysis,
            gcode_profile.rotary_config(),
        ));
    }

    if let (Some(start), Some(current)) = (selection.drag_start, selection.drag_current) {
        if (current - start).length() > 2.0 {
            let rect = egui::Rect::from_two_pos(
                egui::pos2(start.x, start.y),
                egui::pos2(current.x, current.y),
            );
            let painter = context.layer_painter(egui::LayerId::new(
                egui::Order::Foreground,
                egui::Id::new("selection_rectangle"),
            ));
            painter.rect_filled(
                rect,
                0.0,
                egui::Color32::from_rgba_unmultiplied(80, 140, 255, 28),
            );
            let stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(120, 180, 255));
            painter.line_segment([rect.left_top(), rect.right_top()], stroke);
            painter.line_segment([rect.right_top(), rect.right_bottom()], stroke);
            painter.line_segment([rect.right_bottom(), rect.left_bottom()], stroke);
            painter.line_segment([rect.left_bottom(), rect.left_top()], stroke);
        }
    }

    if let Some(position) = context.input(|input| input.pointer.hover_pos()) {
        let over_ui = menu_rect.expand(6.0).contains(position)
            || feature_rect.expand(8.0).contains(position)
            || command_rect.expand(6.0).contains(position)
            || context.egui_wants_pointer_input();
        if !over_ui {
            let size = PICK_TOLERANCE * 2.0;
            let rect = egui::Rect::from_center_size(position, egui::vec2(size, size));
            let painter = context.layer_painter(egui::LayerId::new(
                egui::Order::Foreground,
                egui::Id::new("pick_box"),
            ));
            let stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(245, 205, 90));
            painter.line_segment([rect.left_top(), rect.right_top()], stroke);
            painter.line_segment([rect.right_top(), rect.right_bottom()], stroke);
            painter.line_segment([rect.right_bottom(), rect.left_bottom()], stroke);
            painter.line_segment([rect.left_bottom(), rect.left_top()], stroke);
        }
    }

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
    if export_gcode {
        export_gcode_file(&mut state);
    }
    if quick_export_gcode {
        quick_export_gcode_file(&mut state);
    }
    if fit_clicked {
        state.fit_dirty = true;
    }

    if space.mode == SpaceMode::Cam && !state.gcode_settings_open {
        let (view_camera, view_transform) = view_camera.into_inner();
        draw_machining_order_labels(
            context,
            &state,
            view_camera,
            view_transform,
            &[menu_rect, feature_rect, command_rect],
        );
        draw_machining_section_labels(
            context,
            &state,
            view_camera,
            view_transform,
            &[menu_rect, feature_rect, command_rect],
        );
    }

    view_cube_ui(context, &mut camera);

    if state.gcode_settings_open {
        if state.gcode_preview.is_empty() {
            refresh_gcode_preview(&mut state);
        }
        let mut window_open = true;
        let mut save_profile = false;
        let mut reload_profile = false;
        let mut validate_gcode = false;
        let config_path = gcode_config_path();
        let window_size = egui::vec2(960.0, 540.0);
        let default_pos = context.content_rect().center() - window_size * 0.5;
        egui::Window::new("G代码配置")
            .fixed_size(window_size)
            .default_pos(default_pos)
            .movable(true)
            .drag_area(egui::WindowDrag::Anywhere)
            .open(&mut window_open)
            .show(context, |ui| {
                ui.label(format!("配置文件：{}", config_path.display()));
                ui.separator();
                let tabs = [
                    "文件级",
                    "图元类型",
                    "图元颜色",
                    "所属图层",
                    "加工安全",
                    "加工规划",
                ];
                ui.horizontal_wrapped(|ui| {
                    for (index, title) in tabs.iter().enumerate() {
                        if ui
                            .selectable_label(state.gcode_settings_tab == index, *title)
                            .clicked()
                        {
                            state.gcode_settings_tab = index;
                        }
                    }
                });
                ui.separator();
                let body_height = (ui.available_height() - 90.0).max(180.0);
                egui::ScrollArea::vertical()
                    .id_salt("gcode_settings_body")
                    .max_height(body_height)
                    .show(ui, |ui| {
                        ui.set_min_height(body_height);
                        match state.gcode_settings_tab {
                            0 => {
                                egui::Grid::new("gcode_file_tab")
                                    .num_columns(4)
                                    .spacing([10.0, 5.0])
                                    .show(ui, |ui| {
                                        ui.label("配置名称");
                                        ui.text_edit_singleline(&mut state.gcode_profile.name);
                                        ui.label("单元开始");
                                        ui.text_edit_singleline(
                                            &mut state.gcode_profile.unit_start,
                                        );
                                        ui.end_row();
                                        ui.label("单元结束");
                                        ui.text_edit_singleline(&mut state.gcode_profile.unit_end);
                                        ui.label("快速代码");
                                        ui.text_edit_singleline(&mut state.gcode_profile.rapid);
                                        ui.end_row();
                                        ui.label("切削代码");
                                        ui.text_edit_singleline(&mut state.gcode_profile.cutting);
                                        ui.label("进给");
                                        ui.add(
                                            egui::DragValue::new(&mut state.gcode_profile.feed)
                                                .speed(1.0),
                                        );
                                        ui.end_row();
                                        ui.label("小数位");
                                        ui.add(
                                            egui::DragValue::new(&mut state.gcode_profile.decimals)
                                                .range(0..=6),
                                        );
                                        ui.label("");
                                        ui.label("");
                                        ui.end_row();
                                    });
                                ui.horizontal(|ui| {
                                    ui.checkbox(
                                        &mut state.gcode_profile.arc_output,
                                        "输出圆弧 G02/G03",
                                    );
                                    ui.checkbox(
                                        &mut state.gcode_profile.keep_continuous_angle,
                                        "A 轴保持连续角度",
                                    );
                                });
                                let available = ui.available_height();
                                let label_height = 18.0;
                                let gap = 6.0;
                                let editor_height =
                                    ((available - 2.0 * label_height - 2.0 * gap) / 2.0).max(40.0);
                                ui.label("文件头");
                                ui.add_sized(
                                    egui::vec2(ui.available_width(), editor_height),
                                    egui::TextEdit::multiline(&mut state.gcode_profile.header)
                                        .desired_rows(1),
                                );
                                ui.add_space(gap);
                                ui.label("文件尾");
                                ui.add_sized(
                                    egui::vec2(ui.available_width(), editor_height),
                                    egui::TextEdit::multiline(&mut state.gcode_profile.footer)
                                        .desired_rows(1),
                                );
                            }
                            1 => {
                                let mut selected = state.gcode_selected_type.clone();
                                gcode_rule_tab(
                                    ui,
                                    &mut state.gcode_profile.entity_type_rules,
                                    &mut selected,
                                    &[],
                                    &BTreeMap::new(),
                                    &BTreeMap::new(),
                                    RuleTab::EntityType,
                                );
                                state.gcode_selected_type = selected;
                            }
                            2 => {
                                let mut available =
                                    vec!["BYLAYER".to_string(), "BYBLOCK".to_string()];
                                for index in 1..=9 {
                                    available.push(format!("ACI:{index}"));
                                }
                                let mut colors = BTreeMap::new();
                                for key in state.gcode_profile.color_rules.keys() {
                                    if let Some(color) = color_rule_swatch(key) {
                                        colors.insert(key.clone(), color);
                                    }
                                }
                                let mut selected = state.gcode_selected_color.clone();
                                gcode_rule_tab(
                                    ui,
                                    &mut state.gcode_profile.color_rules,
                                    &mut selected,
                                    &available,
                                    &colors,
                                    &BTreeMap::new(),
                                    RuleTab::Color,
                                );
                                state.gcode_selected_color = selected;
                            }
                            3 => {
                                let mut layers: Vec<String> = Vec::new();
                                let mut colors = BTreeMap::new();
                                let mut indices = BTreeMap::new();
                                for layer in &state.display.layers {
                                    layers.push(layer.name.clone());
                                    colors.insert(layer.name.clone(), layer.color);
                                    if let Some(index) = layer.color_index {
                                        indices.insert(layer.name.clone(), index);
                                    }
                                }
                                layers.push("0".to_string());
                                layers.sort();
                                layers.dedup();
                                let mut selected = state.gcode_selected_layer.clone();
                                gcode_rule_tab(
                                    ui,
                                    &mut state.gcode_profile.layer_rules,
                                    &mut selected,
                                    &layers,
                                    &colors,
                                    &indices,
                                    RuleTab::Layer,
                                );
                                state.gcode_selected_layer = selected;
                            }
                            4 => {
                                egui::Grid::new("gcode_safety_tab")
                                    .num_columns(2)
                                    .spacing([10.0, 5.0])
                                    .show(ui, |ui| {
                                        ui.label("旋转安全抬刀");
                                        ui.add(
                                            egui::DragValue::new(
                                                &mut state.gcode_profile.rotation_safety_clearance,
                                            )
                                            .speed(0.1),
                                        );
                                        ui.end_row();
                                        ui.label("同区离面");
                                        ui.add(
                                            egui::DragValue::new(
                                                &mut state.gcode_profile.same_zone_clearance,
                                            )
                                            .speed(0.1),
                                        );
                                        ui.end_row();
                                    });
                                ui.label("同区离面为 0 时，同区位、同 A 角直接在表面快速转移。");
                                ui.label(
                                "大于 0 时执行局部抬刀、平移、落刀；跨区始终先抬到旋转安全高度。",
                            );
                                ui.separator();
                                ui.checkbox(
                                    &mut state.gcode_profile.use_initial_position,
                                    "启用初始机床位置",
                                );
                                if state.gcode_profile.use_initial_position {
                                    egui::Grid::new("gcode_initial_position")
                                        .num_columns(4)
                                        .spacing([8.0, 4.0])
                                        .show(ui, |ui| {
                                            ui.label("X");
                                            ui.add(
                                                egui::DragValue::new(
                                                    &mut state.gcode_profile.initial_x,
                                                )
                                                .speed(0.1),
                                            );
                                            ui.label("Y");
                                            ui.add(
                                                egui::DragValue::new(
                                                    &mut state.gcode_profile.initial_y,
                                                )
                                                .speed(0.1),
                                            );
                                            ui.end_row();
                                            ui.label("Z");
                                            ui.add(
                                                egui::DragValue::new(
                                                    &mut state.gcode_profile.initial_z,
                                                )
                                                .speed(0.1),
                                            );
                                            ui.label("A");
                                            ui.add(
                                                egui::DragValue::new(
                                                    &mut state.gcode_profile.initial_a,
                                                )
                                                .speed(0.1),
                                            );
                                            ui.end_row();
                                        });
                                }
                                ui.checkbox(
                                    &mut state.gcode_profile.return_to_initial,
                                    "程序结束返回初始位置",
                                );
                            }
                            5 => {
                                egui::Grid::new("gcode_planning_tab")
                                    .num_columns(2)
                                    .spacing([10.0, 5.0])
                                    .show(ui, |ui| {
                                        ui.label("过切距离");
                                        ui.add(
                                            egui::DragValue::new(&mut state.gcode_profile.overcut)
                                                .speed(0.1),
                                        );
                                        ui.end_row();
                                        ui.label("圆弧折线公差");
                                        ui.add(
                                            egui::DragValue::new(
                                                &mut state.gcode_profile.arc_tolerance,
                                            )
                                            .speed(0.01)
                                            .range(0.001..=10.0),
                                        );
                                        ui.end_row();
                                        ui.label("A 轴偏移");
                                        ui.add(
                                            egui::DragValue::new(
                                                &mut state.gcode_profile.a_offset_degrees,
                                            )
                                            .speed(0.1),
                                        );
                                        ui.end_row();
                                        ui.label("加工面 Z 修正");
                                        ui.add(
                                            egui::DragValue::new(
                                                &mut state.gcode_profile.machining_z_offset,
                                            )
                                            .speed(0.1),
                                        );
                                        ui.end_row();
                                    });
                                ui.label(
                                    "加工规划参数会改变四轴姿态或轨迹，保存后会重新计算轨迹。",
                                );
                            }
                            _ => {}
                        }
                    });
                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("保存配置并重算").clicked() {
                        save_profile = true;
                    }
                    if ui.button("重新加载").clicked() {
                        reload_profile = true;
                    }
                    if ui.button("刷新预览").clicked() {
                        refresh_gcode_preview(&mut state);
                    }
                    if ui.button("校验 G 代码").clicked() {
                        validate_gcode = true;
                    }
                });
                ui.separator();
                let preview: String = state
                    .gcode_preview
                    .lines()
                    .take(300)
                    .collect::<Vec<_>>()
                    .join("\n");
                egui::CollapsingHeader::new("G代码预览（前 300 行）")
                    .default_open(false)
                    .show(ui, |ui| {
                        egui::ScrollArea::vertical()
                            .id_salt("gcode_preview")
                            .max_height(220.0)
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                ui.monospace(preview);
                            });
                    });
            });
        state.gcode_settings_open = window_open;

        if reload_profile {
            state.gcode_profile = gcode::GCodeProfile::load(&config_path).unwrap_or_default();
            sync_machine_start(&mut state);
            rebuild_state_trajectory(&mut state);
            refresh_gcode_preview(&mut state);
            push_message(&mut state, "已重新加载 G 代码配置");
        }
        if save_profile {
            match state.gcode_profile.save(&config_path) {
                Ok(()) => {
                    sync_machine_start(&mut state);
                    rebuild_state_trajectory(&mut state);
                    refresh_gcode_preview(&mut state);
                    push_message(&mut state, "G 代码配置已保存，轨迹已重新计算");
                }
                Err(error) => push_message(&mut state, format!("保存配置失败：{error}")),
            }
        }
        if validate_gcode {
            if let Some(trajectory) = state.trajectory.as_ref() {
                let issues = gcode::validate_trajectory(trajectory, &state.gcode_profile);
                if issues.is_empty() {
                    push_message(&mut state, "G 代码校验通过");
                } else {
                    for issue in issues.into_iter().take(5) {
                        push_message(&mut state, format!("G 代码校验：{issue}"));
                    }
                }
            } else {
                push_message(&mut state, "没有可校验的四轴轨迹");
            }
        }
    }

    let pointer = context.input(|input| input.pointer.hover_pos());
    let over_panels = pointer
        .map(|position| {
            menu_rect.expand(6.0).contains(position)
                || feature_rect.expand(8.0).contains(position)
                || command_rect.expand(6.0).contains(position)
        })
        .unwrap_or(false);
    ui_state.pointer_over_ui =
        over_panels || context.egui_is_using_pointer() || context.egui_wants_pointer_input();
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

fn gcode_config_path() -> PathBuf {
    let base = env::var_os("APPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(|| env::current_dir().unwrap_or_default());
    base.join("CADCAM-Rust").join("gcode.toml")
}

fn refresh_gcode_preview(state: &mut ViewerState) {
    state.gcode_preview = state
        .trajectory
        .as_ref()
        .map(|trajectory| gcode::render_gcode(trajectory, &state.gcode_profile))
        .unwrap_or_default();
}

fn color_rule_swatch(key: &str) -> Option<[f32; 3]> {
    match key {
        "BYLAYER" => Some([0.86, 0.89, 0.94]),
        "BYBLOCK" => Some([0.36, 0.40, 0.46]),
        _ => key
            .strip_prefix("ACI:")
            .and_then(|index| index.parse().ok())
            .map(display::aci_color),
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum RuleTab {
    EntityType,
    Color,
    Layer,
}

fn entity_type_label(key: &str) -> String {
    let name = match key {
        "LINE" => "直线",
        "ARC" => "圆弧",
        "CIRCLE" => "圆",
        "ELLIPSE" => "椭圆",
        "POLYLINE" | "LWPOLYLINE" => "多段线",
        "SPLINE" => "样条曲线",
        "POINT" => "点",
        _ => "未知类型",
    };
    format!("{key}  {name}")
}

fn color_key_label(key: &str) -> String {
    let name = match key {
        "BYLAYER" => "随层",
        "BYBLOCK" => "随块",
        "ACI:1" => "红色",
        "ACI:2" => "黄色",
        "ACI:3" => "绿色",
        "ACI:4" => "青色",
        "ACI:5" => "蓝色",
        "ACI:6" => "洋红",
        "ACI:7" => "白色",
        "ACI:8" => "深灰",
        "ACI:9" => "浅灰",
        _ => "索引色",
    };
    format!("{key}  {name}")
}

fn gcode_rule_tab(
    ui: &mut egui::Ui,
    rules: &mut BTreeMap<String, gcode::CodeRule>,
    selected: &mut String,
    available: &[String],
    key_colors: &BTreeMap<String, [f32; 3]>,
    key_indices: &BTreeMap<String, u8>,
    tab: RuleTab,
) {
    for name in available {
        rules.entry(name.clone()).or_default();
    }
    let keys: Vec<String> = rules.keys().cloned().collect();
    if keys.is_empty() {
        ui.label("暂无规则");
        return;
    }
    if selected.is_empty() || !rules.contains_key(selected) {
        *selected = keys[0].clone();
    }
    let panel_height = ui.available_height();

    ui.horizontal(|ui| {
        ui.vertical(|ui| {
            ui.set_min_width(190.0);
            for key in &keys {
                let mut label = match tab {
                    RuleTab::EntityType => entity_type_label(key),
                    RuleTab::Color => color_key_label(key),
                    RuleTab::Layer => key.clone(),
                };
                if tab == RuleTab::Layer {
                    if let Some(index) = key_indices.get(key) {
                        label = format!("{label}  ACI {index}");
                    }
                }
                ui.horizontal(|ui| {
                    if let Some(color) = key_colors.get(key) {
                        ui.colored_label(color32(*color), "■");
                    }
                    if ui
                        .selectable_label(selected.as_str() == key.as_str(), label)
                        .clicked()
                    {
                        *selected = key.clone();
                    }
                });
            }
        });
        ui.separator();
        ui.vertical(|ui| {
            ui.set_min_width(360.0);
            ui.spacing_mut().item_spacing.y = 2.0;
            ui.label(format!("规则：{selected}"));
            if let Some(rule) = rules.get_mut(selected) {
                let editor_height =
                    ((panel_height - 4.0 * 18.0 - 3.0 * 5.0 - 22.0) / 2.0).max(45.0);
                ui.label("头部代码");
                ui.add_sized(
                    egui::vec2(ui.available_width(), editor_height),
                    egui::TextEdit::multiline(&mut rule.header).desired_rows(1),
                );
                ui.label("尾部代码");
                ui.add_sized(
                    egui::vec2(ui.available_width(), editor_height),
                    egui::TextEdit::multiline(&mut rule.footer).desired_rows(1),
                );
                ui.label("说明");
                ui.add_sized(
                    egui::vec2(ui.available_width(), 22.0),
                    egui::TextEdit::singleline(&mut rule.comment),
                );
            }
        });
    });
}

fn sync_machine_start(state: &mut ViewerState) {
    let start = [
        state.gcode_profile.initial_x,
        state.gcode_profile.initial_y,
        state.gcode_profile.initial_z,
    ];
    let ViewerState {
        display,
        cam,
        analysis,
        ..
    } = &mut *state;
    analysis.set_machine_start(display, cam, start);
}

fn rebuild_state_trajectory(state: &mut ViewerState) {
    let ViewerState {
        display,
        cam,
        analysis,
        trajectory,
        gcode_profile,
        ..
    } = &mut *state;
    *trajectory = Some(cam_trajectory::build_trajectory(
        display,
        cam,
        analysis,
        gcode_profile.rotary_config(),
    ));
}

fn export_gcode_file(state: &mut ViewerState) {
    let Some(trajectory) = state.trajectory.as_ref() else {
        push_message(state, "没有可导出的四轴轨迹，请先完成 CAM 分析");
        return;
    };
    if trajectory.unit_ranges.is_empty() {
        push_message(state, "没有可导出的加工单元");
        return;
    }
    let issues = gcode::validate_trajectory(trajectory, &state.gcode_profile);
    if !issues.is_empty() {
        for issue in issues.into_iter().take(5) {
            push_message(state, format!("导出前校验：{issue}"));
        }
        return;
    }

    let stem = Path::new(&state.file_name)
        .file_stem()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_else(|| "output".to_string());
    let directory = dialog_directory(state);
    let Some(path) = rfd::FileDialog::new()
        .set_title("导出 G 代码")
        .add_filter("G 代码 (*.nc, *.gcode)", &["nc", "gcode", "txt"])
        .set_directory(&directory)
        .set_file_name(format!("{stem}.nc"))
        .save_file()
    else {
        return;
    };

    let text = gcode::render_gcode(trajectory, &state.gcode_profile);
    match fs::write(&path, text) {
        Ok(()) => push_message(state, format!("已导出 G 代码：{}", path.display())),
        Err(error) => push_message(state, format!("导出 G 代码失败：{error}")),
    }
}

fn quick_export_gcode_file(state: &mut ViewerState) {
    let Some(trajectory) = state.trajectory.as_ref() else {
        push_message(state, "没有可导出的四轴轨迹，请先完成 CAM 分析");
        return;
    };
    if trajectory.unit_ranges.is_empty() {
        push_message(state, "没有可导出的加工单元");
        return;
    }
    let issues = gcode::validate_trajectory(trajectory, &state.gcode_profile);
    if !issues.is_empty() {
        for issue in issues.into_iter().take(5) {
            push_message(state, format!("导出前校验：{issue}"));
        }
        return;
    }
    let stem = Path::new(&state.file_name)
        .file_stem()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_else(|| "output".to_string());
    let path = env::current_dir()
        .unwrap_or_default()
        .join("build")
        .join(format!("{stem}-quick.nc"));
    if let Some(parent) = path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    let text = gcode::render_gcode(trajectory, &state.gcode_profile);
    match fs::write(&path, text) {
        Ok(()) => push_message(state, format!("快速导出 G 代码：{}", path.display())),
        Err(error) => push_message(state, format!("快速导出 G 代码失败：{error}")),
    }
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
