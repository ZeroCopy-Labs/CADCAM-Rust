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

use cad_core::{DrawingChange, DrawingEdit};
use cad_file::open_drawing;

mod cad_aids;
mod cad_command;
mod cad_curves;
mod cad_diagnostics;
mod cad_edit;
mod cad_file;
mod cad_layers;
mod cad_selection;
mod cad_tools;
mod cad_view;
mod cam;
mod cam_analysis;
mod cam_trajectory;
mod cam_transfer;
mod cam_wall;
mod display;
mod gcode;

const BACKGROUND: Color = Color::srgb(0.035, 0.045, 0.060);
const RECENT_LIMIT: usize = 10;
const MESSAGE_LIMIT: usize = 200;
const COMMAND_MAX_HISTORY_LINES: usize = 4;
const COMMAND_ROW_HEIGHT: f32 = 21.0;
const COMMAND_VERTICAL_PADDING: f32 = 12.0;
const PICK_TOLERANCE: f32 = 12.0;

/// Everything the viewer keeps after a drawing is loaded.
#[derive(Resource, Default)]
struct ViewerState {
    drawing: Option<cad_core::EditableDrawing>,
    file_ui: cad_file::FileState,
    cad_controls: cad_edit::EditControls,
    cad_curves: cad_curves::CurveControls,
    cad_diagnostics: cad_diagnostics::Controls,
    cad_layers: cad_layers::LayerControls,
    cad_command: cad_command::CommandState,
    cad_history: BTreeMap<u64, CadUndoState>,
    file_name: String,
    format: String,
    version: String,
    unit: String,
    entity_count: usize,
    layer_count: usize,
    warning_count: usize,
    display: display::DisplayData,
    machining: display::DisplayData,
    machining_error: Option<String>,
    view: cad_view::ViewFrame,
    cam: cam::CamModel,
    analysis: cam_analysis::CamAnalysis,
    manual_tube: cam_analysis::TubeSection,
    manual_pair: [Option<cam::UnitId>; 2],
    trajectory: Option<cam_trajectory::MachineTrajectory>,
    gcode_profile: gcode::GCodeProfile,
    gcode_settings_message: String,
    gcode_settings_error: bool,
    transfer_example: TransferExample,
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

struct CadUndoState {
    layer_rules: BTreeMap<String, Option<gcode::CodeRule>>,
    selected: BTreeSet<u32>,
    cam_selected: BTreeSet<cam::UnitId>,
    machining: Option<(
        cam_analysis::CamAnalysis,
        cam_analysis::TubeSection,
        [Option<cam::UnitId>; 2],
    )>,
}

#[derive(Default)]
struct TransferExample {
    distance: f64,
    delta_z: f64,
    cross_zone: bool,
    delta_a: f64,
}

/// Orbit camera state, independent from the loaded drawing.
#[derive(Resource)]
struct CameraRig {
    target: [f64; 3],
    yaw: f32,
    pitch: f32,
    distance: f32,
    ortho_scale: f32,
}

impl Default for CameraRig {
    fn default() -> Self {
        Self {
            target: [0.0; 3],
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
    filter: cad_selection::SelectionFilter,
    hover_hits: Vec<u32>,
    hover_index: usize,
    hover_cursor: Option<Vec2>,
}
impl SelectionState {
    fn hovered(&self) -> Option<u32> {
        self.hover_hits.get(self.hover_index).copied()
    }
    fn cycle(&mut self, reverse: bool) {
        let n = self.hover_hits.len();
        if n > 0 {
            self.hover_index = (self.hover_index + if reverse { n - 1 } else { 1 }) % n;
        }
    }
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
        transfer_example: TransferExample {
            distance: 100.0,
            delta_a: 90.0,
            ..Default::default()
        },
        gcode_profile: gcode::GCodeProfile::load_or_create(&gcode_path),
        ..Default::default()
    };
    push_message(&mut state, "使用“文件 → 打开文件”选择 DXF/DWG 图纸");
    push_message(
        &mut state,
        "视角：Shift+中键旋转 | 中键拖动平移 | 滚轮缩放 | 右上角魔方切换",
    );
    push_message(&mut state, format!("G代码配置：{}", gcode_path.display()));
    if std::fs::read_dir(cad_file::recovery_directory()).is_ok_and(|entries| {
        entries.filter_map(std::result::Result::ok).any(|entry| {
            entry
                .path()
                .extension()
                .is_some_and(|extension| extension.eq_ignore_ascii_case("dxf"))
        })
    }) {
        push_message(
            &mut state,
            "发现图纸恢复快照，可通过“文件 → 恢复图纸快照…”打开",
        );
    }
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
                    close_when_requested: false,
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
                cad_file::maintain_recovery,
                cad_file::close_requested,
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

fn push_message(state: &mut ViewerState, message: impl Into<String>) {
    state.messages.push(message.into());
    if state.messages.len() > MESSAGE_LIMIT {
        state.messages.remove(0);
    }
}

fn submit_command(
    state: &mut ViewerState,
    selection: &mut SelectionState,
    mode: SpaceMode,
    raw: &str,
) {
    let command = raw.trim();
    let upper = command.to_uppercase();
    if matches!(upper.as_str(), "CANCEL" | "取消") {
        state.cad_command.cancel();
        state.command_input.clear();
        push_message(state, "已取消命令，图纸未改动");
        return;
    }
    if mode != SpaceMode::Cad {
        push_message(state, "CAD 命令请在 CAD 工作区执行");
        return;
    }
    if state.drawing.is_none() {
        push_message(state, "请先打开或新建图纸");
        return;
    }
    let was_active = state.cad_command.is_active();
    let result = if was_active {
        state.cad_command.input(command, &selection.selected)
    } else {
        match upper.as_str() {
            "MOVE" | "M" | "移动" => {
                state.command_input.clear();
                state.cad_command.begin_move(&selection.selected);
                Ok(None)
            }
            "UNDO" | "U" | "撤销" => {
                apply_cad_action(state, selection, cad_edit::EditAction::Undo);
                Ok(None)
            }
            "REDO" | "重做" => {
                apply_cad_action(state, selection, cad_edit::EditAction::Redo);
                Ok(None)
            }
            "" => return,
            _ if upper.starts_with("UCS ") => state
                .cad_command
                .set_preset(upper[4..].trim())
                .map(|_| None),
            _ if cad_curves::TOOLS.iter().any(|(_,name)|*name==upper)=>{
                state.cad_curves.tool=cad_curves::TOOLS.iter().find(|(_,name)|*name==upper).map(|(tool,_)|*tool);
                match state.cad_curves.recipe(state.drawing.as_ref().unwrap().document(),&selection.selected,state.cad_command.plane){
                    Ok(edit)=>{state.cad_command.begin_curves(edit);Ok(None)},Err(error)=>Err(error),
                }
            },
            _ if state.cad_command.begin_named(&upper, &selection.selected) => Ok(None),
            _ => Err("可用命令见左侧工具或绘图/编辑菜单：LINE、PLINE、RECTANGLE、CIRCLE、ARC、ELLIPSE、SPLINE、COPY、ROTATE、SCALE、MIRROR、ARRAYRECT、ARRAYPOLAR".into()),
        }
    };
    match result {
        Ok(Some(commit)) => {
            let action = match commit {
                cad_command::CommandCommit::Curves(edit) => cad_edit::EditAction::Curves(edit),
                cad_command::CommandCommit::Geometry(updates) => {
                    cad_edit::EditAction::Geometry(updates)
                }
                cad_command::CommandCommit::Move(commit) => {
                    selection.selected = commit.ids;
                    cad_edit::EditAction::Apply(DrawingEdit::Translate(commit.offset))
                }
                cad_command::CommandCommit::Create { geometry, normal } => {
                    let Some(layer) = state
                        .drawing
                        .as_ref()
                        .and_then(|d| d.document().current_layer)
                    else {
                        push_message(state, "请先选择新图元所属图层");
                        return;
                    };
                    cad_edit::EditAction::Create(
                        geometry
                            .into_iter()
                            .map(|geometry| cad_core::geometry_edit::NewEntity {
                                geometry,
                                layer,
                                color: state.cad_command.tools.color,
                                normal: Some(normal),
                            })
                            .collect(),
                    )
                }
                cad_command::CommandCommit::Transform {
                    ids,
                    transforms,
                    copies,
                } => {
                    selection.selected = ids;
                    cad_edit::EditAction::Transform { transforms, copies }
                }
            };
            if apply_cad_action(state, selection, action) {
                state.cad_command.cancel();
            } else {
                state.cad_command.error = "操作未提交，请调整参数或取消".into();
            }
        }
        Ok(None) => {
            state.cad_command.error.clear();
            if !state.cad_command.is_active() && (was_active || upper.starts_with("UCS ")) {
                push_message(state, "工作平面已设置，图元坐标保持不变");
            }
        }
        Err(error) => {
            state.cad_command.error = error.clone();
            state.command_input = command.to_string();
            push_message(state, error);
        }
    }
    if state.cad_command.is_active() {
        state.command_open = true;
        let prompt = state.cad_command.prompt();
        push_message(state, prompt);
    }
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
                let delta = -right * motion.delta.x * speed + up * motion.delta.y * speed;
                rig.target[0] += delta.x as f64;
                rig.target[1] -= delta.z as f64;
                rig.target[2] += delta.y as f64;
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
    let center = [
        bounds.min_x + (bounds.max_x - bounds.min_x) * 0.5,
        bounds.min_y + (bounds.max_y - bounds.min_y) * 0.5,
        bounds.min_z + (bounds.max_z - bounds.min_z) * 0.5,
    ];
    let frame = cad_view::ViewFrame {
        origin: center,
        pixel_size: 1.0,
    };
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
                let corner = frame.point([x, y, z]);
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
    mut state: ResMut<ViewerState>,
    rig: Res<CameraRig>,
    camera: Single<(&mut Transform, &mut Projection), With<MainCamera>>,
) {
    state.view.origin = rig.target;
    state.view.pixel_size = rig.ortho_scale as f64;
    let eye = rig.direction() * rig.distance;
    let (mut transform, mut projection) = camera.into_inner();
    *transform = Transform::from_translation(eye).looking_at(Vec3::ZERO, Vec3::Y);
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
    let preview = (space.mode == SpaceMode::Cad)
        .then(|| state.cad_command.preview())
        .flatten();
    for path in &state.display.paths {
        if !path.visible {
            continue;
        }
        let unit = state.cam.unit_for_entity(path.entity_id);
        let is_selected = match space.mode {
            SpaceMode::Cad => selection.selected.contains(&path.entity_id),
            SpaceMode::Cam => unit
                .map(|unit_id| selection.cam_selected.contains(&unit_id))
                .unwrap_or(false),
        };
        let is_hovered =
            space.mode == SpaceMode::Cad && selection.hovered() == Some(path.entity_id);
        let pending_delete = state
            .cad_command
            .removed_preview_ids()
            .is_some_and(|ids| ids.contains(&cad_core::EntityId(path.entity_id)));
        let color = if pending_delete {
            Color::srgb(1.0, 0.18, 0.18)
        } else if is_hovered && !is_selected {
            Color::srgb(1.0, 0.5, 0.12)
        } else if is_selected {
            Color::srgb(1.0, 0.82, 0.22)
        } else {
            Color::srgb(path.color[0], path.color[1], path.color[2])
        };
        for pair in path.points.windows(2) {
            let [x0, y0, z0] = pair[0];
            let [x1, y1, z1] = pair[1];
            let start = state.view.point([x0, y0, z0]);
            let end = state.view.point([x1, y1, z1]);
            if is_selected || is_hovered || pending_delete {
                selected_gizmos.line(start, end, color);
            } else {
                gizmos.line(start, end, color);
            }
        }
        if let Some(preview) = preview
            .as_ref()
            .filter(|preview| preview.ids.contains(&path.entity_id))
        {
            for pair in path.points.windows(2) {
                let start = state
                    .view
                    .point(std::array::from_fn(|i| pair[0][i] + preview.offset[i]));
                let end = state
                    .view
                    .point(std::array::from_fn(|i| pair[1][i] + preview.offset[i]));
                if start.is_finite() && end.is_finite() {
                    selected_gizmos.line(start, end, Color::srgb(0.15, 1.0, 1.0));
                }
            }
        }
    }

    if space.mode == SpaceMode::Cad
        && !state.cad_command.is_active()
        && selection.selected.len() == 1
    {
        if let Some(doc) = state.drawing.as_ref().map(|d| d.document()) {
            if let Some(entity) =
                doc.entity(cad_core::EntityId(*selection.selected.first().unwrap()))
            {
                if entity.visible
                    && doc
                        .layer(entity.layer_id)
                        .is_none_or(|l| l.visible && !l.locked)
                {
                    for grip in cad_core::properties::grips(&entity.geometry) {
                        let position = state.view.point([grip.point.x, grip.point.y, grip.point.z]);
                        selected_gizmos.cube(
                            Transform::from_translation(position)
                                .with_scale(Vec3::splat((state.view.pixel_size * 6.0) as f32)),
                            Color::srgb(0.2, 1.0, 0.5),
                        );
                    }
                }
            }
        }
    }

    if space.mode == SpaceMode::Cad {
        for path in state.cad_command.drawing_preview() {
            for pair in path.windows(2) {
                let a = state.view.point(pair[0]);
                let b = state.view.point(pair[1]);
                if a.is_finite() && b.is_finite() {
                    selected_gizmos.line(a, b, Color::srgb(0.15, 1.0, 1.0));
                }
            }
        }
        if let Some((ids, transforms)) = state.cad_command.transform_preview() {
            for path in state
                .display
                .paths
                .iter()
                .filter(|p| ids.contains(&p.entity_id))
            {
                for transform in &transforms {
                    for pair in path.points.windows(2) {
                        let a = state.view.point(transform.point(pair[0]));
                        let b = state.view.point(transform.point(pair[1]));
                        if a.is_finite() && b.is_finite() {
                            selected_gizmos.line(a, b, Color::srgb(0.15, 1.0, 1.0));
                        }
                    }
                }
            }
        }
    }
    // Keep the axes at the CAD world origin, never at the drawing center.
    let extent = state
        .display
        .bounds
        .map(|bounds| {
            ((bounds.max_x - bounds.min_x)
                .max(bounds.max_y - bounds.min_y)
                .max(bounds.max_z - bounds.min_z)
                .max(1.0)) as f32
        })
        .unwrap_or(100.0);
    if space.mode == SpaceMode::Cad {
        state
            .cad_command
            .draw_plane(&mut gizmos, extent as f64, state.view);
    }
    let axis = extent * 0.6;
    let z_axis = axis * 0.5;
    let red = Color::srgb(0.85, 0.25, 0.25);
    let green = Color::srgb(0.30, 0.85, 0.35);
    let blue = Color::srgb(0.25, 0.45, 0.95);
    let origin = state.view.point([0.0; 3]);

    // CAD +X, +Y and +Z in Bevy coordinates: CAD Y maps to -Bevy Z.
    let positive_x = Vec3::X * axis;
    let positive_y = -Vec3::Z * axis;
    let positive_z = Vec3::Y * z_axis;
    gizmos.line(origin, origin + positive_x, red);
    gizmos.line(origin, origin + positive_y, blue);
    gizmos.line(origin, origin + positive_z, green);
    negative_axes.line(origin, origin - positive_x, red);
    negative_axes.line(origin, origin - positive_y, blue);
    negative_axes.line(origin, origin - positive_z, green);
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
        .filter(|entity| entity.visible)
        .map(|entity| entity.id)
        .collect();
    if selection.load_version != state.load_version {
        selection.selected.clear();
        selection.cam_selected.clear();
        selection.drag_start = None;
        selection.drag_current = None;
        selection.reorder_clicks = None;
        selection.load_version = state.load_version;
        selection.hover_hits.clear();
        selection.hover_cursor = None;
        selection.hover_index = 0;
        selection.filter = Default::default();
    }
    selection.selected.retain(|id| valid.contains(id));
    let valid_units: BTreeSet<cam::UnitId> = state.cam.units.iter().map(|unit| unit.id).collect();
    selection.cam_selected.retain(|id| valid_units.contains(id));

    let Some(cursor) = window.cursor_position() else {
        if mouse.just_released(MouseButton::Left) {
            state.cad_command.stop_grip_drag();
        }
        state.cad_command.clear_hover();
        selection.hover_hits.clear();
        selection.hover_cursor = None;
        selection.hover_index = 0;
        return Ok(());
    };
    let context = contexts.ctx_mut()?;
    let ui_active = ui_state.pointer_over_ui
        || context.egui_wants_pointer_input()
        || context.egui_is_using_pointer();
    if ui_active {
        if mouse.just_released(MouseButton::Left) {
            state.cad_command.stop_grip_drag();
        }
        selection.hover_hits.clear();
        selection.hover_cursor = None;
        selection.hover_index = 0;
        state.cad_command.clear_hover();
        // A click that starts on the panel resize handle must be owned by the
        // panel, even if the first press frame already reached this system.
        selection.drag_start = None;
        selection.drag_current = None;
        selection.reorder_clicks = None;
        return Ok(());
    }
    let (camera, camera_transform) = camera.into_inner();
    if space.mode == SpaceMode::Cad
        && !state.cad_command.is_active()
        && mouse.just_pressed(MouseButton::Left)
    {
        if let Some((id, geometry, kind)) =
            pick_grip(&state, &selection, camera, camera_transform, cursor)
        {
            if state
                .cad_command
                .begin_grip(id, geometry, kind, None, true)
                .is_ok()
            {
                state.command_open = true;
                state.command_input.clear();
                selection.drag_start = None;
                selection.drag_current = None;
                return Ok(());
            }
        }
    }
    let nearby = pick_entities(
        &state.display,
        &state.view,
        camera,
        camera_transform,
        cursor,
    );
    if space.mode == SpaceMode::Cad
        && state.cad_command.is_active()
        && !state.cad_command.selecting()
    {
        selection.hover_hits.clear();
        let accepted = {
            let ViewerState {
                drawing,
                cad_command,
                view,
                ..
            } = &mut *state;
            drawing.as_ref().is_some_and(|drawing| {
                cad_command.viewport_point(
                    camera,
                    camera_transform,
                    cursor,
                    *view,
                    drawing.document(),
                    if cad_command.grip_dragging() {
                        mouse.just_released(MouseButton::Left)
                    } else {
                        mouse.just_pressed(MouseButton::Left)
                    },
                )
            })
        };
        if mouse.just_released(MouseButton::Left) {
            state.cad_command.stop_grip_drag();
        }
        if accepted {
            state.command_input.clear();
            if !state.cad_command.is_active() {
                push_message(&mut state, "工作平面已按三点设置，图元坐标保持不变");
            }
        }
        selection.drag_start = None;
        selection.drag_current = None;
        return Ok(());
    }
    if space.mode == SpaceMode::Cad {
        let hits: Vec<_> = nearby
            .into_iter()
            .filter(|id| {
                state
                    .display
                    .entities
                    .iter()
                    .find(|e| e.id == *id)
                    .is_some_and(|e| selection.filter.matches(e))
            })
            .collect();
        if selection
            .hover_cursor
            .is_none_or(|p| p.distance(cursor) > 3.0)
            || selection.hover_hits != hits
        {
            selection.hover_index = 0;
        }
        selection.hover_cursor = Some(cursor);
        selection.hover_hits = hits;
        if selection.hover_hits.is_empty() {
            selection.hover_index = 0;
        } else {
            selection.hover_index %= selection.hover_hits.len();
        }
    } else {
        selection.hover_hits.clear();
    }
    let ctrl = keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight);
    if !ctrl {
        selection.reorder_clicks = None;
    }

    if mouse.just_pressed(MouseButton::Left) {
        if space.mode == SpaceMode::Cam && ctrl {
            if let Some(unit_id) = pick_entity(
                &state.display,
                &state.view,
                camera,
                camera_transform,
                cursor,
            )
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
                    let chosen = selection.hovered().into_iter().collect();
                    let alt = keys.pressed(KeyCode::AltLeft) || keys.pressed(KeyCode::AltRight);
                    let SelectionState {
                        filter, selected, ..
                    } = &mut *selection;
                    filter.apply(selected, chosen, shift, alt, ctrl);
                }
                SpaceMode::Cam => {
                    let hit = pick_entity(
                        &state.display,
                        &state.view,
                        camera,
                        camera_transform,
                        cursor,
                    )
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
                    let chosen = box_select(
                        &state.display,
                        &state.view,
                        camera,
                        camera_transform,
                        start,
                        end,
                    )
                    .into_iter()
                    .filter(|id| {
                        state
                            .display
                            .entities
                            .iter()
                            .find(|e| e.id == *id)
                            .is_some_and(|e| selection.filter.matches(e))
                    })
                    .collect();
                    let alt = keys.pressed(KeyCode::AltLeft) || keys.pressed(KeyCode::AltRight);
                    let SelectionState {
                        filter, selected, ..
                    } = &mut *selection;
                    filter.apply(selected, chosen, shift, alt, ctrl);
                }
                SpaceMode::Cam => {
                    let chosen = box_select_units(
                        &state.display,
                        &state.cam,
                        &state.view,
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

fn pick_grip(
    state: &ViewerState,
    selection: &SelectionState,
    camera: &Camera,
    transform: &GlobalTransform,
    cursor: Vec2,
) -> Option<(
    cad_core::EntityId,
    cad_core::Geometry,
    cad_core::properties::GripKind,
)> {
    if selection.selected.len() != 1 {
        return None;
    }
    let doc = state.drawing.as_ref()?.document();
    let id = cad_core::EntityId(*selection.selected.first()?);
    let entity = doc.entity(id)?;
    if !entity.visible {
        return None;
    }
    if doc
        .layer(entity.layer_id)
        .is_some_and(|l| l.locked || !l.visible)
    {
        return None;
    }
    cad_core::properties::grips(&entity.geometry)
        .into_iter()
        .filter_map(|g| {
            state
                .view
                .project(camera, transform, [g.point.x, g.point.y, g.point.z])
                .map(|screen| (screen.distance(cursor), g.kind))
        })
        .filter(|(distance, _)| *distance <= 8.0)
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, kind)| (id, entity.geometry.clone(), kind))
}

fn project_entity_point(
    camera: &Camera,
    camera_transform: &GlobalTransform,
    point: [f64; 3],
    frame: &cad_view::ViewFrame,
) -> Option<Vec2> {
    frame.project(camera, camera_transform, point)
}

fn pick_entities(
    display: &display::DisplayData,
    frame: &cad_view::ViewFrame,
    camera: &Camera,
    transform: &GlobalTransform,
    cursor: Vec2,
) -> Vec<u32> {
    let mut hits: BTreeMap<u32, (f32, f32)> = BTreeMap::new();
    for path in &display.paths {
        if !path.visible {
            continue;
        }
        for pair in path.points.windows(2) {
            let (Some(a), Some(b)) = (
                frame.project(camera, transform, pair[0]),
                frame.project(camera, transform, pair[1]),
            ) else {
                continue;
            };
            let distance = distance_to_segment(cursor, a, b);
            if distance > PICK_TOLERANCE {
                continue;
            }
            let segment = b - a;
            let t = if segment.length_squared() > 1e-12 {
                ((cursor - a).dot(segment) / segment.length_squared()).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let point = frame.point(std::array::from_fn(|i| {
                pair[0][i] + (pair[1][i] - pair[0][i]) * t as f64
            }));
            let depth = (point - transform.translation()).length_squared();
            let hit = hits.entry(path.entity_id).or_insert((distance, depth));
            if distance < hit.0 || distance == hit.0 && depth < hit.1 {
                *hit = (distance, depth);
            }
        }
    }
    let mut hits: Vec<_> = hits.into_iter().collect();
    hits.sort_by(|a, b| {
        a.1 .0
            .total_cmp(&b.1 .0)
            .then(a.1 .1.total_cmp(&b.1 .1))
            .then(a.0.cmp(&b.0))
    });
    hits.into_iter().map(|(id, _)| id).collect()
}
fn pick_entity(
    display: &display::DisplayData,
    frame: &cad_view::ViewFrame,
    camera: &Camera,
    transform: &GlobalTransform,
    cursor: Vec2,
) -> Option<u32> {
    pick_entities(display, frame, camera, transform, cursor)
        .into_iter()
        .next()
}

fn box_select(
    display: &display::DisplayData,
    frame: &cad_view::ViewFrame,
    camera: &Camera,
    camera_transform: &GlobalTransform,
    start: Vec2,
    end: Vec2,
) -> BTreeSet<u32> {
    let rect = egui::Rect::from_two_pos(egui::pos2(start.x, start.y), egui::pos2(end.x, end.y));
    let window_mode = end.x >= start.x;
    let mut selected = BTreeSet::new();

    for path in &display.paths {
        if !path.visible {
            continue;
        }
        let projected: Vec<Vec2> = path
            .points
            .iter()
            .filter_map(|point| project_entity_point(camera, camera_transform, *point, frame))
            .collect();
        if projected.len() < 2 {
            continue;
        }

        let hit = if window_mode {
            projected.len() == path.points.len()
                && projected
                    .iter()
                    .all(|point| rect.contains(egui::pos2(point.x, point.y)))
        } else {
            projected
                .iter()
                .any(|point| rect.contains(egui::pos2(point.x, point.y)))
                || path.points.windows(2).any(|pair| {
                    match (
                        frame.project(camera, camera_transform, pair[0]),
                        frame.project(camera, camera_transform, pair[1]),
                    ) {
                        (Some(a), Some(b)) => segment_intersects_rect(a, b, rect),
                        _ => false,
                    }
                })
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
    frame: &cad_view::ViewFrame,
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
            if !path.visible {
                continue;
            }
            if cam.entity_to_unit.get(&path.entity_id) != Some(&unit.id) {
                continue;
            }
            let projected: Vec<Vec2> = path
                .points
                .iter()
                .filter_map(|point| project_entity_point(camera, camera_transform, *point, frame))
                .collect();
            if projected.len() < 2 {
                all_inside = false;
                continue;
            }
            has_projection = true;
            all_inside &= projected.len() == path.points.len()
                && projected
                    .iter()
                    .all(|point| rect.contains(egui::pos2(point.x, point.y)));
            intersects |= projected
                .iter()
                .any(|point| rect.contains(egui::pos2(point.x, point.y)))
                || path.points.windows(2).any(|pair| {
                    match (
                        frame.project(camera, camera_transform, pair[0]),
                        frame.project(camera, camera_transform, pair[1]),
                    ) {
                        (Some(a), Some(b)) => segment_intersects_rect(a, b, rect),
                        _ => false,
                    }
                });
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
            machining: display,
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
    pair_sections: Option<(cam::UnitId, cam::UnitId)>,
    reset_pairs: bool,
    trim_section: Option<(cam::UnitId, bool)>,
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
    manual_pair: &mut [Option<cam::UnitId>; 2],
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
                    for (index, section) in analysis.machining_sections.iter().enumerate() {
                        let covered = section.zone_mask.iter().filter(|value| **value).count();
                        ui.label(format!(
                            "断面 {} · {} · X {:.2}..{:.2}  绕数 {}  成员 {}  区位 {}",
                            index + 1,
                            analysis.end_cut_side(section.unit_id).label(),
                            section.x_min,
                            section.x_max,
                            section.winding,
                            section.boundary_units.len(),
                            covered
                        ));
                        let mut trim = analysis.trim_sections.contains(&section.unit_id);
                        if ui
                            .checkbox(
                                &mut trim,
                                format!("断面 {} 仅作修边（不参与配对和壁厚补偿）", index + 1),
                            )
                            .changed()
                        {
                            action.trim_section = Some((section.unit_id, trim));
                        }
                    }
                    ui.separator();
                    ui.label("手动成品段配对");
                    for (side, selected) in manual_pair.iter_mut().enumerate() {
                        let label = if side == 0 { "左断面" } else { "右断面" };
                        let text = analysis
                            .machining_sections
                            .iter()
                            .position(|s| Some(s.unit_id) == *selected)
                            .map(|i| format!("断面 {}", i + 1))
                            .unwrap_or_else(|| "请选择".to_string());
                        egui::ComboBox::from_id_salt(("part_pair", side))
                            .selected_text(format!("{label}：{text}"))
                            .show_ui(ui, |ui| {
                                for (i, s) in analysis.machining_sections.iter().enumerate() {
                                    ui.selectable_value(
                                        selected,
                                        Some(s.unit_id),
                                        format!(
                                            "断面 {} · X {:.2}..{:.2}",
                                            i + 1,
                                            s.x_min,
                                            s.x_max
                                        ),
                                    );
                                }
                            });
                    }
                    ui.horizontal(|ui| {
                        if ui.button("应用左右配对").clicked() {
                            if let [Some(left), Some(right)] = *manual_pair {
                                action.pair_sections = Some((left, right));
                            }
                        }
                        if ui.button("恢复自动配对").clicked() {
                            action.reset_pairs = true;
                        }
                    });
                    ui.label(
                        "自动按轴向位置两两配对；相邻成品不共用断面。未配对断面可指定为修边。",
                    );
                }

                for error in &analysis.grouping_errors {
                    ui.colored_label(egui::Color32::RED, error);
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
                            "{} {} · X {:.2}..{:.2} · {} 个单元",
                            if group.left_section.is_some() {
                                "成品段"
                            } else {
                                "组外/修边组"
                            },
                            group.id + 1,
                            group.x_start,
                            group.x_end,
                            group.units.len()
                        );
                        egui::CollapsingHeader::new(title)
                            .id_salt(("machining_group", group.id))
                            .default_open(false)
                            .show(ui, |ui| {
                                for (label, boundary) in [
                                    ("左端面", group.left_section),
                                    ("右端面", group.right_section),
                                ] {
                                    if let Some(section_unit) = boundary {
                                        let section_index = analysis
                                            .machining_sections
                                            .iter()
                                            .position(|section| section.unit_id == section_unit);
                                        ui.label(format!(
                                            "{}：{}",
                                            label,
                                            section_index
                                                .map(|index| format!("#{}", index + 1))
                                                .unwrap_or_else(|| "未编号".to_string())
                                        ));
                                    }
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
                    ui.label("成品段顺序固定：左端面 → 各区位加工单元 → 右端面。");
                    ui.label("左右端面固定在组首、组末；普通单元仅在两者之间排序。");
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
                                        analysis.end_cut_side(unit.unit_id).label().to_string()
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
            if let Some(error) = &trajectory.error {
                ui.colored_label(egui::Color32::RED, error);
            } else if trajectory.moves.is_empty() {
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
                &state.machining,
                &state.cam,
                unit.unit_id,
                unit.direction,
                Some(unit.start_point),
            ) {
                if let Some(start_point) = path.first() {
                    if let Ok(start_screen) =
                        camera.world_to_viewport(camera_transform, state.view.point(*start_point))
                    {
                        let start = egui::pos2(start_screen.x, start_screen.y);
                        let mut direction = None;
                        for point in path.iter().skip(1) {
                            if let Ok(next_screen) =
                                camera.world_to_viewport(camera_transform, state.view.point(*point))
                            {
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

            let Some(center) = unit_center(&state.machining, &state.cam, unit.unit_id) else {
                continue;
            };
            let world = state.view.point(center);
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
        let Some(center) = unit_center(&state.machining, &state.cam, section.unit_id) else {
            continue;
        };
        let world = state.view.point(center);
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
        if !path.visible {
            continue;
        }
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
    let (view_camera, view_transform) = view_camera.into_inner();
    let context = contexts.ctx_mut()?;
    install_cjk_font(context, &mut fonts_loaded);
    context.set_visuals(egui::Visuals::dark());

    let mut command_request = None;
    if !state.gcode_settings_open && !state.file_ui.report_open {
        let wants_keyboard = context.egui_wants_keyboard_input();
        context.input_mut(|input| {
            if state.cad_command.is_active()
                && input.consume_key(egui::Modifiers::NONE, egui::Key::Escape)
            {
                command_request = Some("CANCEL".to_string());
            } else if !wants_keyboard {
                if state.cad_command.is_active()
                    && input.consume_key(egui::Modifiers::NONE, egui::Key::Enter)
                {
                    command_request = Some(String::new());
                } else if space.mode == SpaceMode::Cad
                    && !state.cad_command.is_active()
                    && input.consume_key(egui::Modifiers::CTRL, egui::Key::M)
                {
                    command_request = Some("MOVE".to_string());
                }
            }
        });
    }
    let command_field =
        context.memory(|memory| memory.has_focus(egui::Id::new("cad_command_input")));
    if space.mode == SpaceMode::Cad
        && !state.gcode_settings_open
        && !state.file_ui.report_open
        && (!context.egui_wants_keyboard_input() || command_field && state.command_input.is_empty())
    {
        context.input_mut(|input| {
            if input.consume_key(egui::Modifiers::NONE, egui::Key::F3) {
                state.cad_command.aids.enabled = !state.cad_command.aids.enabled;
            }
            if input.consume_key(egui::Modifiers::NONE, egui::Key::F7) {
                state.cad_command.aids.grid = !state.cad_command.aids.grid;
            }
            if input.consume_key(egui::Modifiers::NONE, egui::Key::F9) {
                state.cad_command.aids.grid_snap = !state.cad_command.aids.grid_snap;
            }
            if input.consume_key(egui::Modifiers::NONE, egui::Key::F8) {
                let aids = &mut state.cad_command.aids;
                aids.ortho = !aids.ortho;
                if aids.ortho {
                    aids.polar = false;
                }
            }
            if input.consume_key(egui::Modifiers::NONE, egui::Key::F10) {
                let aids = &mut state.cad_command.aids;
                aids.polar = !aids.polar;
                if aids.polar {
                    aids.ortho = false;
                }
            }
            let can_cycle = if state.cad_command.takes_point() {
                state.cad_command.aids.has_candidates()
            } else {
                (!state.cad_command.is_active() || state.cad_command.selecting())
                    && !selection.hover_hits.is_empty()
            };
            let reverse = can_cycle && input.consume_key(egui::Modifiers::SHIFT, egui::Key::Tab);
            if can_cycle && (reverse || input.consume_key(egui::Modifiers::NONE, egui::Key::Tab)) {
                if state.cad_command.takes_point() {
                    state.cad_command.cycle_snap(reverse);
                } else if !state.cad_command.is_active() || state.cad_command.selecting() {
                    selection.cycle(reverse);
                }
            }
        });
    }
    let mut export_dxf = false;
    let mut new_drawing = false;
    let mut save_drawing = false;
    let mut save_as = false;
    let mut close_drawing = false;
    let mut recover_drawing = false;
    let mut open_dialog = false;
    if !state.gcode_settings_open
        && !state.file_ui.report_open
        && !context.egui_wants_keyboard_input()
    {
        context.input_mut(|input| {
            new_drawing = input.consume_key(egui::Modifiers::CTRL, egui::Key::N);
            open_dialog = input.consume_key(egui::Modifiers::CTRL, egui::Key::O);
            save_as =
                input.consume_key(egui::Modifiers::CTRL | egui::Modifiers::SHIFT, egui::Key::S);
            save_drawing = save_as || input.consume_key(egui::Modifiers::CTRL, egui::Key::S);
            if state.cad_command.is_active() {
                save_drawing = false;
            }
        });
    }
    let mut cad_action = None;
    let mut select_all = false;
    if space.mode == SpaceMode::Cad
        && !state.gcode_settings_open
        && !state.file_ui.report_open
        && !context.egui_wants_keyboard_input()
    {
        context.input_mut(|input| {
            if !state.cad_command.is_active()
                && (input.consume_key(egui::Modifiers::CTRL | egui::Modifiers::SHIFT, egui::Key::Z)
                    || input.consume_key(egui::Modifiers::CTRL, egui::Key::Y))
            {
                cad_action = Some(cad_edit::EditAction::Redo);
            } else if !state.cad_command.is_active()
                && input.consume_key(egui::Modifiers::CTRL, egui::Key::Z)
            {
                cad_action = Some(cad_edit::EditAction::Undo);
            }
            if (!state.cad_command.is_active() || state.cad_command.selecting())
                && input.consume_key(egui::Modifiers::CTRL, egui::Key::A)
            {
                select_all = true;
            }
            if !state.cad_command.is_active()
                && input.consume_key(egui::Modifiers::NONE, egui::Key::Delete)
            {
                cad_action = Some(cad_edit::EditAction::Apply(DrawingEdit::Delete));
            }
        });
    }
    let mut export_gcode = false;
    let mut quick_export_gcode = false;
    let mut pending_path: Option<PathBuf> = None;
    let mut fit_clicked = false;
    let current_name = if state.file_name.is_empty() {
        "未打开文件".to_string()
    } else {
        format!(
            "{}{}",
            state.file_name,
            if state
                .drawing
                .as_ref()
                .is_some_and(|drawing| drawing.is_modified())
            {
                " *"
            } else {
                ""
            }
        )
    };

    let mut root = egui::Ui::new(
        context.clone(),
        egui::Id::new("cad_root"),
        egui::UiBuilder::new()
            .layer_id(egui::LayerId::background())
            .max_rect(context.viewport_rect()),
    );

    let menu_panel = egui::Panel::top("menu_bar").show(&mut root, |ui| {
        ui.horizontal_wrapped(|ui| {
            ui.menu_button("文件", |ui| {
                if ui.button("新建图纸  Ctrl+N").clicked() {
                    new_drawing = true;
                    ui.close();
                }
                if ui.button("打开文件  Ctrl+O").clicked() {
                    open_dialog = true;
                    ui.close();
                }
                ui.separator();
                if ui
                    .add_enabled(
                        state.drawing.is_some() && !state.cad_command.is_active(),
                        egui::Button::new("保存  Ctrl+S"),
                    )
                    .clicked()
                {
                    save_drawing = true;
                    ui.close();
                }
                if ui
                    .add_enabled(
                        state.drawing.is_some() && !state.cad_command.is_active(),
                        egui::Button::new("另存为 DXF…  Ctrl+Shift+S"),
                    )
                    .clicked()
                {
                    save_drawing = true;
                    save_as = true;
                    ui.close();
                }
                if ui
                    .add_enabled(
                        state.drawing.is_some() && !state.cad_command.is_active(),
                        egui::Button::new("导出 DXF…"),
                    )
                    .clicked()
                {
                    export_dxf = true;
                    ui.close();
                }
                if ui
                    .add_enabled(
                        !state.cad_command.is_active(),
                        egui::Button::new("快速导出到 build"),
                    )
                    .clicked()
                {
                    quick_export_gcode = true;
                    ui.close();
                }
                ui.separator();
                if ui
                    .add_enabled(state.drawing.is_some(), egui::Button::new("导入报告"))
                    .clicked()
                {
                    state.file_ui.report_open = true;
                    ui.close();
                }
                if ui.button("恢复图纸快照…").clicked() {
                    recover_drawing = true;
                    ui.close();
                }
                if ui
                    .add_enabled(state.drawing.is_some(), egui::Button::new("关闭图纸"))
                    .clicked()
                {
                    close_drawing = true;
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
                ui.separator();
                if ui.button("退出").clicked() {
                    state.file_ui.exit_requested = true;
                    ui.close();
                }
            });
            ui.add_enabled_ui(
                space.mode == SpaceMode::Cad
                    && state.drawing.is_some()
                    && !state.cad_command.is_active(),
                |ui| {
                    ui.menu_button("绘图", |ui| {
                        for kind in [
                            cad_core::geometry_edit::SketchKind::Line,
                            cad_core::geometry_edit::SketchKind::Polyline,
                            cad_core::geometry_edit::SketchKind::Rectangle,
                            cad_core::geometry_edit::SketchKind::Circle,
                            cad_core::geometry_edit::SketchKind::Arc,
                            cad_core::geometry_edit::SketchKind::Ellipse,
                            cad_core::geometry_edit::SketchKind::Spline,
                        ] {
                            if ui.button(kind.label()).clicked() {
                                command_request = Some(kind.command().into());
                                ui.close();
                            }
                        }
                    });
                },
            );
            ui.add_enabled_ui(space.mode == SpaceMode::Cad, |ui| {
                ui.menu_button("编辑", |ui| {
                    ui.add_enabled_ui(
                        state.drawing.is_some() && !state.cad_command.is_active(),
                        |ui| {
                            ui.menu_button("曲线整理", |ui| {
                                for (tool, name) in cad_curves::TOOLS {
                                    if ui.button(tool.label()).clicked() {
                                        command_request = Some(name.into());
                                        ui.close();
                                    }
                                }
                            });
                            if ui.button("复制（基点/目标点）").clicked() {
                                command_request = Some("COPY".into());
                                ui.close();
                            }
                            for kind in [
                                cad_tools::TransformKind::Rotate,
                                cad_tools::TransformKind::Scale,
                                cad_tools::TransformKind::Mirror,
                                cad_tools::TransformKind::RectArray,
                                cad_tools::TransformKind::PolarArray,
                            ] {
                                if ui.button(kind.label()).clicked() {
                                    command_request = Some(kind.command().into());
                                    ui.close();
                                }
                            }
                        },
                    );
                    if ui
                        .add_enabled(
                            state.drawing.is_some() && !state.cad_command.is_active(),
                            egui::Button::new("基点 → 目标点移动  Ctrl+M"),
                        )
                        .clicked()
                    {
                        command_request = Some("MOVE".into());
                        ui.close();
                    }
                    if ui
                        .add_enabled(
                            state
                                .drawing
                                .as_ref()
                                .is_some_and(|drawing| drawing.can_undo())
                                && !state.cad_command.is_active(),
                            egui::Button::new("撤销  Ctrl+Z"),
                        )
                        .clicked()
                    {
                        cad_action = Some(cad_edit::EditAction::Undo);
                        ui.close();
                    }
                    if ui
                        .add_enabled(
                            state
                                .drawing
                                .as_ref()
                                .is_some_and(|drawing| drawing.can_redo())
                                && !state.cad_command.is_active(),
                            egui::Button::new("重做  Ctrl+Y / Ctrl+Shift+Z"),
                        )
                        .clicked()
                    {
                        cad_action = Some(cad_edit::EditAction::Redo);
                        ui.close();
                    }
                    if ui
                        .add_enabled(
                            !state.cad_command.is_active() || state.cad_command.selecting(),
                            egui::Button::new("全选  Ctrl+A"),
                        )
                        .clicked()
                    {
                        select_all = true;
                        ui.close();
                    }
                    if ui
                        .add_enabled(
                            !selection.selected.is_empty() && !state.cad_command.is_active(),
                            egui::Button::new("删除所选  Delete"),
                        )
                        .clicked()
                    {
                        cad_action = Some(cad_edit::EditAction::Apply(DrawingEdit::Delete));
                        ui.close();
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
            if ui
                .add_enabled(
                    !state.cad_command.is_active(),
                    egui::Button::new("导出 G 代码"),
                )
                .clicked()
            {
                export_gcode = true;
            }
            if ui.button("G代码配置").clicked() {
                state.gcode_settings_open = true;
            }
            ui.separator();
            let old_mode = space.mode;
            space_switch_ui(ui, &mut space, &mut selection, &state.cam);
            if old_mode != space.mode {
                state.cad_command.cancel();
                state.command_input.clear();
            }
            ui.separator();
            ui.add(egui::Label::new(&current_name).truncate());
        });
    });
    let menu_rect = menu_panel.response.rect;
    if select_all {
        selection.selected = state
            .display
            .entities
            .iter()
            .filter(|entity| selection.filter.matches(entity))
            .map(|entity| entity.id)
            .collect();
    }
    if let Some(command) = command_request.take() {
        submit_command(&mut state, &mut selection, space.mode, &command);
        state.command_focus = state.cad_command.is_active();
    }
    if state.cad_command.is_active() {
        state.command_open = true;
        if !context.egui_wants_keyboard_input()
            && context.input(|input| {
                input
                    .events
                    .iter()
                    .any(|event| matches!(event, egui::Event::Text(_)))
            })
        {
            state.command_focus = true;
        }
    }
    let command_dimensions = state.command_open.then(|| {
        let history_lines = state.messages.len().min(COMMAND_MAX_HISTORY_LINES);
        let history_height = history_lines as f32 * COMMAND_ROW_HEIGHT;
        let total_height = (history_lines + 1 + usize::from(state.cad_command.is_active())) as f32
            * COMMAND_ROW_HEIGHT
            + COMMAND_VERTICAL_PADDING;
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
            if state.cad_command.is_active() {
                let status = format!(
                    "{} {}",
                    state.cad_command.prompt(),
                    state.cad_command.error_message()
                );
                ui.add(egui::Label::new(&status).truncate())
                    .on_hover_text(status);
            }
            let input_id = egui::Id::new("cad_command_input");
            if focus_input {
                ui.memory_mut(|memory| memory.request_focus(input_id));
            }
            ui.horizontal(|ui| {
                ui.add_space(8.0);
                let response = ui.add_sized(
                    egui::Vec2::new(ui.available_width(), COMMAND_ROW_HEIGHT - 2.0),
                    egui::TextEdit::singleline(&mut state.command_input)
                        .id(input_id)
                        .hint_text("MOVE / UCS；坐标 x,y,z / u,v；相对 @；极坐标 @长度<角度"),
                );
                if focus_input {
                    response.request_focus();
                }
                let enter = ui.input(|input| input.key_pressed(egui::Key::Enter));
                if enter && (response.has_focus() || response.lost_focus()) {
                    let command = std::mem::take(&mut state.command_input);
                    submit_command(&mut state, &mut selection, space.mode, &command);
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
                    egui::ScrollArea::vertical()
                        .id_salt("cad_features")
                        .show(ui, |ui| {
                            {
                                let ViewerState {
                                    drawing,
                                    cad_layers,
                                    cad_command,
                                    ..
                                } = &mut *state;
                                if let Some(drawing) = drawing.as_ref() {
                                    ui.add_enabled_ui(!cad_command.is_active(), |ui| {
                                        if let Some(edit) = cad_layers.ui(ui, drawing.document()) {
                                            cad_action = Some(cad_edit::EditAction::Layer(edit));
                                        }
                                    });
                                }
                            }
                            {
                                let ViewerState {
                                    drawing,
                                    cad_diagnostics,
                                    cad_command,
                                    ..
                                } = &mut *state;
                                if let Some(drawing) = drawing.as_ref() {
                                    ui.add_enabled_ui(!cad_command.is_active(), |ui| {
                                        if let Some(action) = cad_diagnostics.ui(
                                            ui,
                                            drawing.document(),
                                            drawing.revision(),
                                        ) {
                                            match action {
                                                cad_diagnostics::Action::Locate { ids, point } => {
                                                    selection.selected =
                                                        ids.iter().map(|id| id.0).collect();
                                                    camera.target = [point.x, point.y, point.z];
                                                }
                                                cad_diagnostics::Action::Preview(edit) => {
                                                    cad_command.begin_curves(edit);
                                                }
                                            }
                                        }
                                    });
                                }
                            }
                            let selection_enabled =
                                !state.cad_command.is_active() || state.cad_command.selecting();
                            ui.add_enabled_ui(selection_enabled, |ui| {
                                if selection.filter.ui(ui, &state.display) {
                                    let ids: Vec<_> = state
                                        .display
                                        .entities
                                        .iter()
                                        .filter(|e| selection.filter.matches(e))
                                        .map(|e| e.id)
                                        .collect();
                                    for id in ids {
                                        if !selection.selected.remove(&id) {
                                            selection.selected.insert(id);
                                        }
                                    }
                                }
                            });
                            {
                                let ViewerState {
                                    drawing,
                                    cad_command,
                                    ..
                                } = &mut *state;
                                if let Some(drawing) = drawing.as_ref() {
                                    if let Some(command) =
                                        cad_command.controls_ui(ui, drawing.document())
                                    {
                                        command_request = Some(command);
                                    }
                                }
                            }
                            {
                                let ViewerState {
                                    cad_curves,
                                    cad_command,
                                    drawing,
                                    ..
                                } = &mut *state;
                                ui.add_enabled_ui(!cad_command.is_active(), |ui| {
                                    if cad_curves.ui(ui) {
                                        if let Some(doc) = drawing.as_ref().map(|d| d.document()) {
                                            match cad_curves.recipe(
                                                doc,
                                                &selection.selected,
                                                cad_command.plane,
                                            ) {
                                                Ok(edit) => cad_command.begin_curves(edit),
                                                Err(error) => cad_command.error = error,
                                            }
                                        }
                                    }
                                });
                            }
                            let editing_enabled = !state.cad_command.is_active()
                                && !state
                                    .display
                                    .entities
                                    .iter()
                                    .any(|e| selection.selected.contains(&e.id) && e.locked);
                            if !state.cad_command.is_active() && !editing_enabled {
                                ui.weak("选择中含锁定图层，属性只读；请先在图层管理解锁。");
                            }
                            let ViewerState {
                                drawing,
                                cad_controls,
                                ..
                            } = &mut *state;
                            if let Some(drawing) = drawing.as_ref() {
                                ui.add_enabled_ui(editing_enabled, |ui| {
                                    if let Some(action) = cad_edit::editing_ui(
                                        ui,
                                        drawing.document(),
                                        &selection.selected,
                                        cad_controls,
                                    ) {
                                        cad_action = Some(action);
                                    }
                                });
                            }
                            let selected_entities: Vec<_> = state
                                .display
                                .entities
                                .iter()
                                .filter(|entity| selection.selected.contains(&entity.id))
                                .collect();
                            if selected_entities.is_empty() {
                                ui.label("左键点选 / 拖拽框选；Shift 追加选择");
                            } else if selected_entities.len() == 1 {
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
                SpaceMode::Cam => {
                    ui.small("加工采样：弦差 0.01 mm · 曲线内转角 3°；独立于显示折线。");
                    if state
                        .drawing
                        .as_ref()
                        .is_some_and(|d| d.document().import_report.has_omissions())
                        && !state.file_ui.coverage_acknowledged
                    {
                        ui.colored_label(
                            egui::Color32::YELLOW,
                            "有未转换对象，导出前需核对加工范围。",
                        );
                        if ui.button("打开导入报告并确认范围").clicked() {
                            state.file_ui.report_open = true;
                        }
                    }
                    if let Some(error) = &state.machining_error {
                        ui.colored_label(egui::Color32::LIGHT_RED, error);
                    }
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        let ViewerState {
                            analysis,
                            trajectory,
                            manual_tube,
                            manual_pair,
                            ..
                        } = &mut *state;
                        cam_action = cam_analysis_ui(
                            ui,
                            analysis,
                            trajectory.as_ref(),
                            manual_tube,
                            manual_pair,
                        );
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
                                    &state.machining,
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
                                            &state.machining,
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
    if let Some(command) = command_request {
        submit_command(&mut state, &mut selection, space.mode, &command);
        state.command_open = true;
        state.command_focus = state.cad_command.is_active();
    }
    if let Some(action) = cad_action {
        if space.mode == SpaceMode::Cad && !state.cad_command.is_active() {
            apply_cad_action(&mut state, &mut selection, action);
        }
    }

    let mut cam_input_changed = false;
    let mut pairing_error = None;
    {
        let ViewerState {
            machining: display,
            cam,
            analysis,
            ..
        } = &mut *state;
        if cam_action.clear_tube {
            analysis.clear_tube_section(display, cam);
            cam_input_changed = true;
        }
        if cam_action.reset_pairs {
            analysis.reset_section_pairs(display, cam);
            cam_input_changed = true;
        }
        if let Some((left, right)) = cam_action.pair_sections {
            match analysis.pair_sections(left, right, display, cam) {
                Ok(()) => cam_input_changed = true,
                Err(error) => pairing_error = Some(error),
            }
        }
        if let Some((unit, trim)) = cam_action.trim_section {
            analysis.set_trim_section(unit, trim, display, cam);
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
    if let Some(error) = pairing_error {
        push_message(&mut state, format!("断面配对失败：{error}"));
    }
    if cam_input_changed {
        rebuild_state_trajectory(&mut state);
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
            let crossing = current.x < start.x;
            let color = if crossing {
                egui::Color32::from_rgb(80, 220, 140)
            } else {
                egui::Color32::from_rgb(120, 180, 255)
            };
            painter.rect_filled(rect, 0.0, color.gamma_multiply(0.12));
            painter.text(
                rect.left_top() + egui::vec2(6.0, 6.0),
                egui::Align2::LEFT_TOP,
                if crossing {
                    "交叉：相交即选"
                } else {
                    "窗口：全部包含"
                },
                egui::FontId::proportional(13.0),
                color,
            );
            let stroke = egui::Stroke::new(1.0, color);
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

    if state.command_open && !state.cad_command.is_active() {
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

    if state.cad_command.is_active() {
        save_drawing = false;
        export_dxf = false;
        export_gcode = false;
        quick_export_gcode = false;
    }
    if new_drawing {
        cad_file::new_drawing(&mut state);
    }
    if save_drawing {
        cad_file::save_drawing(&mut state, save_as);
    }
    if close_drawing {
        cad_file::close_drawing(&mut state);
    }
    if recover_drawing {
        cad_file::recover_drawing(&mut state);
    }
    if open_dialog {
        if let Some(path) = choose_drawing_file(&state) {
            pending_path = Some(path);
        }
    }
    if let Some(path) = pending_path {
        open_drawing(&mut state, &path.to_string_lossy());
    }
    if export_dxf {
        export_dxf_file(&mut state);
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

    if space.mode == SpaceMode::Cad && !state.gcode_settings_open && !state.file_ui.report_open {
        state
            .cad_command
            .aids
            .marker_ui(context, state.view, view_camera, view_transform);
        if let Some(entity) = selection
            .hovered()
            .and_then(|id| state.display.entities.iter().find(|e| e.id == id))
        {
            if let Some(cursor) = context.input(|input| input.pointer.hover_pos()) {
                let painter = context.layer_painter(egui::LayerId::new(
                    egui::Order::Foreground,
                    egui::Id::new("selection_hover"),
                ));
                painter.text(
                    cursor + egui::vec2(18.0, 18.0),
                    egui::Align2::LEFT_TOP,
                    format!(
                        "{} · {} · {}/{} Tab切换",
                        entity.kind,
                        entity.layer,
                        selection.hover_index + 1,
                        selection.hover_hits.len()
                    ),
                    egui::FontId::proportional(14.0),
                    egui::Color32::LIGHT_YELLOW,
                );
            }
        }
    }
    view_cube_ui(context, &mut camera);

    if state.gcode_settings_open {
        gcode_settings_ui(context, &mut state);
    }
    cad_file::import_report_ui(context, &mut state);

    let pointer = context.input(|input| input.pointer.hover_pos());
    let over_panels = pointer
        .map(|position| {
            menu_rect.expand(6.0).contains(position)
                || feature_rect.expand(8.0).contains(position)
                || command_rect.expand(6.0).contains(position)
        })
        .unwrap_or(false);
    ui_state.pointer_over_ui = state.gcode_settings_open
        || state.file_ui.report_open
        || over_panels
        || context.egui_is_using_pointer()
        || context.egui_wants_pointer_input();
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

fn gcode_settings_ui(context: &egui::Context, state: &mut ViewerState) {
    let mut window_open = true;
    let mut action = None;
    let tabs = [
        "文件级",
        "图元类型",
        "图元颜色",
        "所属图层",
        "加工安全",
        "加工规划",
    ];
    let hints = [
        "设置程序头尾、加工指令和输出格式",
        "按图元类型设置头尾代码；类型头最先输出，类型尾最后输出",
        "按图元颜色设置头尾代码；颜色头在图层头之后，颜色尾在图层尾之前",
        "按所属图层设置头尾代码；执行顺序位于类型规则与颜色规则之间",
        "设置安全净空、同面转移和初始位置",
        "设置过切、旋转偏移及端面补偿",
    ];
    let modal = egui::Modal::new(egui::Id::new("gcode_settings_modal"))
        .frame(egui::Frame::new().fill(egui::Color32::from_rgb(18, 24, 33)))
        .show(context, |ui| {
            let size = context.content_rect().size();
            ui.set_min_size(size);
            ui.set_max_size(size);
            ui.spacing_mut().item_spacing = egui::vec2(12.0, 6.0);
            egui::Panel::top("gcode_settings_header")
                .exact_size(108.0)
                .resizable(false)
                .frame(egui::Frame::new().inner_margin(egui::Margin::symmetric(18, 10)))
                .show(ui, |ui| {
                    ui.columns(2, |columns| {
                        columns[0].heading("G代码配置");
                        columns[0].add(egui::Label::new(egui::RichText::new(format!(
                            "保存位置：{}", gcode_config_path().display()
                        )).size(16.0)).truncate())
                            .on_hover_text(gcode_config_path().display().to_string());
                        columns[0].horizontal(|ui| {
                            for (label, selected_action, hint) in [
                                ("导入配置…", GCodeConfigAction::Import, "从 TOML 文件导入当前配置，不自动覆盖已保存的配置"),
                                ("导出配置…", GCodeConfigAction::Export, "将当前配置另存为 TOML 文件"),
                                ("重新加载", GCodeConfigAction::Reload, "从默认保存位置重新加载配置并重算"),
                                ("恢复默认", GCodeConfigAction::Reset, "恢复内置默认值，点击保存后才写入磁盘"),
                            ] {
                                if ui.button(label).on_hover_text(hint).clicked() {
                                    action = Some(selected_action);
                                }
                            }
                        });
                        columns[1].horizontal(|ui| {
                            ui.strong("规则代码执行顺序");
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui.button("关闭  ×").clicked() { window_open = false; }
                            });
                        });
                        columns[1].label("前置：图元类型 → 所属图层 → 图元颜色 → 单元开始");
                        columns[1].label("后置：单元结束 → 图元颜色 → 所属图层 → 图元类型")
                            .on_hover_text("每个连续切削段使用此顺序；多个类型/图层/颜色组合的尾部按组合逆序输出。文件头和文件尾位于整个程序最外层。");
                    });
                });
            egui::Panel::bottom("gcode_settings_footer")
                .exact_size(70.0)
                .resizable(false)
                .frame(egui::Frame::new()
                    .fill(egui::Color32::from_rgb(24, 32, 44))
                    .inner_margin(egui::Margin::symmetric(18, 8)))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new("保存配置并重算")
                            .fill(egui::Color32::from_rgb(40, 100, 170)))
                            .on_hover_text("写入默认配置位置并重新计算轨迹，下次启动使用此配置")
                            .clicked() {
                            action = Some(GCodeConfigAction::Save);
                        }
                        if ui.button("校验 G 代码")
                            .on_hover_text("按当前内存配置重算并校验轨迹，不保存配置或输出文件")
                            .clicked() {
                            action = Some(GCodeConfigAction::Validate);
                        }
                    });
                    let (message, error) = if let Some(error) = state.gcode_profile.validation_error() {
                        (error, true)
                    } else {
                        (state.gcode_settings_message.as_str(), state.gcode_settings_error)
                    };
                    let color = if error { egui::Color32::LIGHT_RED } else {
                        egui::Color32::from_rgb(120, 210, 170)
                    };
                    ui.add(egui::Label::new(egui::RichText::new(message).color(color)).truncate())
                        .on_hover_text(message);
                });
            egui::Panel::left("gcode_settings_navigation")
                .exact_size(148.0)
                .resizable(false)
                .frame(egui::Frame::new().inner_margin(8))
                .show(ui, |ui| {
                    let gap = ui.spacing().item_spacing.y;
                    let height = ((ui.available_height() - gap * 5.0) / 6.0).max(1.0);
                    for (index, title) in tabs.iter().enumerate() {
                        if ui.add_sized([ui.available_width(), height],
                            egui::Button::selectable(state.gcode_settings_tab == index, *title))
                            .on_hover_text(hints[index]).clicked() {
                            state.gcode_settings_tab = index;
                        }
                    }
                });
            egui::CentralPanel::default()
                .frame(egui::Frame::new().inner_margin(16))
                .show(ui, |ui| {
                    ui.push_id(state.gcode_settings_tab, |ui| {
                        match state.gcode_settings_tab {
                            0 => gcode_file_tab(ui, &mut state.gcode_profile),
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
                            4 => gcode_safety_tab(ui, state),
                            5 => gcode_planning_tab(ui, &mut state.gcode_profile),
                            _ => {}
                        }
                    });
                });
        });
    if modal.should_close() {
        window_open = false;
    }
    state.gcode_settings_open = window_open;
    if let Some(action) = action {
        let (message, error) = match gcode_config_action(state, action) {
            Ok(Some(message)) => (message, false),
            Ok(None) => (String::new(), false),
            Err(error) => (error, true),
        };
        if !message.is_empty() {
            state.gcode_settings_message = message.clone();
            state.gcode_settings_error = error;
            push_message(state, message);
        }
    }
    if !window_open {
        rebuild_state_trajectory(state);
    }
}

fn gcode_code_pair(
    ui: &mut egui::Ui,
    header: &mut String,
    footer: &mut String,
    comment: &mut String,
    file: bool,
) {
    let gap = ui.spacing().item_spacing.y;
    let label_height = ui.text_style_height(&egui::TextStyle::Body);
    let comment_height = 28.0;
    let height =
        ((ui.available_height() - label_height * 2.0 - comment_height - gap * 4.0) / 2.0).max(24.0);
    let titles = if file {
        ["文件头", "文件尾"]
    } else {
        ["头部代码", "尾部代码"]
    };
    for (title, text) in titles.into_iter().zip([header, footer]) {
        ui.label(title).on_hover_text(if file {
            "按原样输出到整个程序的开头或结尾，支持多行代码"
        } else {
            "按顶栏说明的规则顺序输出，支持多行代码"
        });
        egui::ScrollArea::vertical()
            .id_salt(title)
            .max_height(height)
            .min_scrolled_height(height)
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.add_sized(
                    [ui.available_width(), (height - 2.0).max(20.0)],
                    egui::TextEdit::multiline(text).code_editor(),
                );
            });
    }
    ui.horizontal(|ui| {
        ui.label("说明")
            .on_hover_text("仅作为配置备注保存，不输出到 G 代码");
        ui.add_sized(
            [ui.available_width(), comment_height],
            egui::TextEdit::singleline(comment).hint_text("配置备注，不输出到 G 代码"),
        );
    });
}

fn gcode_file_tab(ui: &mut egui::Ui, profile: &mut gcode::GCodeProfile) {
    ui.columns(2, |columns| {
        egui::ScrollArea::vertical()
            .id_salt("file_options")
            .auto_shrink([false, false])
            .show(&mut columns[0], |ui| {
                ui.spacing_mut().text_edit_width = (ui.available_width() - 100.0).max(80.0);
                egui::Grid::new("gcode_file_options")
                    .num_columns(2)
                    .spacing([12.0, 8.0])
                    .min_row_height(24.0)
                    .show(ui, |ui| {
                        for (label, value, hint) in [
                            ("配置名称", &mut profile.name, "用于识别和管理当前配置"),
                            (
                                "单元开始",
                                &mut profile.unit_start,
                                "在类型、图层、颜色前置规则之后输出，例如开启激光 M03",
                            ),
                            (
                                "单元结束",
                                &mut profile.unit_end,
                                "在颜色、图层、类型后置规则之前输出，例如关闭激光 M05",
                            ),
                            (
                                "快速代码",
                                &mut profile.rapid,
                                "空走使用的运动指令，通常为 G00",
                            ),
                            (
                                "切削代码",
                                &mut profile.cutting,
                                "直线切削使用的运动指令，通常为 G01",
                            ),
                        ] {
                            ui.label(label).on_hover_text(hint);
                            ui.text_edit_singleline(value).on_hover_text(hint);
                            ui.end_row();
                        }
                        ui.label("小数位")
                            .on_hover_text("输出坐标的小数位数，范围为 0–9");
                        ui.add(egui::DragValue::new(&mut profile.decimals).range(0..=9));
                        ui.end_row();
                        ui.label("换行格式")
                            .on_hover_text("根据控制器要求选择 CRLF 或 LF");
                        egui::ComboBox::from_id_salt("gcode_line_ending")
                            .selected_text(if profile.line_ending.eq_ignore_ascii_case("lf") {
                                "LF"
                            } else {
                                "CRLF"
                            })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    &mut profile.line_ending,
                                    "crlf".to_string(),
                                    "CRLF",
                                );
                                ui.selectable_value(
                                    &mut profile.line_ending,
                                    "lf".to_string(),
                                    "LF",
                                );
                            });
                        ui.end_row();
                    });
                ui.separator();
                ui.checkbox(&mut profile.arc_output, "输出圆弧 G02/G03")
                    .on_hover_text("关闭后按圆弧折线公差展开已识别的圆弧运动");
                ui.checkbox(&mut profile.keep_continuous_angle, "A 轴保持连续角度")
                    .on_hover_text("跨单元选择最接近上一姿态的等价 A 角，避免多绕整圈");
            });
        gcode_code_pair(
            &mut columns[1],
            &mut profile.header,
            &mut profile.footer,
            &mut profile.comment,
            true,
        );
    });
}

fn gcode_number_row(ui: &mut egui::Ui, label: &str, value: &mut f64, speed: f64, hint: &str) {
    ui.label(label).on_hover_text(hint);
    ui.add(egui::DragValue::new(value).speed(speed))
        .on_hover_text(hint);
    ui.end_row();
}

fn gcode_safety_tab(ui: &mut egui::Ui, state: &mut ViewerState) {
    let ViewerState {
        gcode_profile: profile,
        transfer_example,
        ..
    } = state;
    ui.columns(2, |columns| {
        egui::ScrollArea::vertical()
            .id_salt("safety_options")
            .auto_shrink([false, false])
            .show(&mut columns[0], |ui| {
                egui::Grid::new("gcode_safety_options")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .min_row_height(22.0)
                    .show(ui, |ui| {
                        gcode_number_row(
                            ui,
                            "旋转安全抬刀",
                            &mut profile.rotation_safety_clearance,
                            0.1,
                            "跨区位或旋转时，在管材最大碰撞半径之外增加的安全净空",
                        );
                        gcode_number_row(
                            ui,
                            "同区离面",
                            &mut profile.same_zone_clearance,
                            0.1,
                            "关闭同面联动后，同面且 A 不变的空走离面高度；0 表示沿表面转移",
                        );
                    });
                transfer_settings_ui(ui, &mut profile.transfer);
                ui.separator();
                ui.checkbox(&mut profile.use_initial_position, "启用初始机床位置")
                    .on_hover_text("从下方配置的机床位置经安全净空进入第一个加工单元");
                ui.checkbox(&mut profile.return_to_initial, "结束返回初始位置")
                    .on_hover_text("程序结束返回配置的 XYZ，A 选择靠近末姿态的等价角度");
                egui::Grid::new("gcode_initial_position")
                    .num_columns(2)
                    .spacing([12.0, 6.0])
                    .min_row_height(22.0)
                    .show(ui, |ui| {
                        for (label, value) in [
                            ("初始 X", &mut profile.initial_x),
                            ("初始 Y", &mut profile.initial_y),
                            ("初始 Z", &mut profile.initial_z),
                            ("初始 A", &mut profile.initial_a),
                        ] {
                            gcode_number_row(
                                ui,
                                label,
                                value,
                                0.1,
                                "初始及返回位置使用此坐标；初始 XYZ 也作为加工规划的参考位置",
                            );
                        }
                    });
            });
        transfer_diagram_ui(
            &mut columns[1],
            &profile.transfer,
            transfer_example,
            profile.decimals,
        );
    });
}

fn gcode_planning_tab(ui: &mut egui::Ui, profile: &mut gcode::GCodeProfile) {
    let field_width = ((ui.available_width() - 16.0) * 0.5).max(80.0);
    egui::Grid::new("gcode_planning_options")
        .num_columns(2)
        .min_col_width(field_width)
        .spacing([16.0, 12.0])
        .min_row_height(40.0)
        .show(ui, |ui| {
            for (label, value, hint) in [
                (
                    "过切距离",
                    &mut profile.overcut,
                    "闭合轮廓加工一周后沿切向继续切削的距离，0 表示不过切",
                ),
                (
                    "A 轴偏移",
                    &mut profile.a_offset_degrees,
                    "附加到几何姿态的 A 角偏移，按 360° 周期解释",
                ),
                (
                    "加工面 Z 修正",
                    &mut profile.machining_z_offset,
                    "调整加工高度；启用壁厚补偿时纳入内壁投影统一求 X，其余跨区位单元采用 Z/X 联动",
                ),
                (
                    "方管壁厚",
                    &mut profile.wall_thickness,
                    "启用端面补偿时必须大于零，并小于截面较小尺寸的一半",
                ),
            ] {
                ui.label(label).on_hover_text(hint);
                ui.add_sized([field_width, 32.0], egui::DragValue::new(value).speed(0.1))
                    .on_hover_text(hint);
                ui.end_row();
            }
            ui.label("圆弧折线公差")
                .on_hover_text("关闭圆弧输出时，将语义圆弧展开为折线所用的公差，范围为 0.001–10");
            ui.add_sized(
                [field_width, 32.0],
                egui::DragValue::new(&mut profile.arc_tolerance)
                    .speed(0.01)
                    .range(0.001..=10.0),
            );
            ui.end_row();
            ui.label("成品端面壁厚补偿")
                .on_hover_text("仅补偿已配对的左右端面，修边断面不补偿；刀头轨迹允许弯折或不共面");
            ui.checkbox(&mut profile.end_cut_correction, "启用（左 +X / 右 −X）")
                .on_hover_text("从含 Z 偏移的刀头姿态求内壁作用位置，一次性修正 X");
            ui.end_row();
        });
}

#[derive(Clone, Copy)]
enum GCodeConfigAction {
    Import,
    Export,
    Save,
    Reload,
    Reset,
    Validate,
}

fn gcode_config_action(
    state: &mut ViewerState,
    action: GCodeConfigAction,
) -> Result<Option<String>, String> {
    let config_path = gcode_config_path();
    let directory = config_path.parent().unwrap_or_else(|| Path::new("."));
    let message = match action {
        GCodeConfigAction::Import | GCodeConfigAction::Reload => {
            let path = if matches!(action, GCodeConfigAction::Import) {
                let Some(path) = rfd::FileDialog::new()
                    .set_title("导入 G 代码配置")
                    .add_filter("TOML 配置", &["toml"])
                    .set_directory(directory)
                    .pick_file()
                else {
                    return Ok(None);
                };
                path
            } else {
                config_path.clone()
            };
            let profile = gcode::GCodeProfile::load(&path)
                .map_err(|error| format!("加载配置失败，当前配置已保留：{error}"))?;
            state.gcode_profile = profile;
            rebuild_state_trajectory(state);
            if matches!(action, GCodeConfigAction::Import) {
                format!(
                    "已导入配置并重算：{}；点击保存以供下次启动使用",
                    path.display()
                )
            } else {
                "已重新加载保存的配置并重算".to_string()
            }
        }
        GCodeConfigAction::Export | GCodeConfigAction::Save => {
            let path = if matches!(action, GCodeConfigAction::Export) {
                let Some(path) = rfd::FileDialog::new()
                    .set_title("导出 G 代码配置")
                    .add_filter("TOML 配置", &["toml"])
                    .set_directory(directory)
                    .set_file_name("gcode.toml")
                    .save_file()
                else {
                    return Ok(None);
                };
                path
            } else {
                config_path
            };
            state
                .gcode_profile
                .save(&path)
                .map_err(|error| format!("写入配置失败：{error}"))?;
            if matches!(action, GCodeConfigAction::Save) {
                rebuild_state_trajectory(state);
                format!("配置已保存，轨迹已重新计算：{}", path.display())
            } else {
                format!("配置已导出：{}", path.display())
            }
        }
        GCodeConfigAction::Reset => {
            state.gcode_profile = gcode::GCodeProfile::default();
            rebuild_state_trajectory(state);
            "已恢复全部默认配置并重算；点击保存以供下次启动使用".to_string()
        }
        GCodeConfigAction::Validate => {
            rebuild_state_trajectory(state);
            let trajectory = state
                .trajectory
                .as_ref()
                .ok_or_else(|| "没有可校验的四轴轨迹".to_string())?;
            let issues = gcode::validate_trajectory(trajectory, &state.gcode_profile);
            if !issues.is_empty() {
                return Err(format!("G 代码校验：{}", issues.join("；")));
            }
            "G 代码校验通过".to_string()
        }
    };
    Ok(Some(message))
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
    ui.columns(2, |columns| {
        let list_height = columns[0].available_height().max(24.0);
        egui::ScrollArea::vertical()
            .id_salt("gcode_rule_keys")
            .max_height(list_height)
            .min_scrolled_height(list_height)
            .auto_shrink([false, false])
            .show(&mut columns[0], |ui| {
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
                            let (rect, response) = ui
                                .allocate_exact_size(egui::vec2(18.0, 18.0), egui::Sense::hover());
                            ui.painter()
                                .rect_filled(rect, 3.0, egui::Color32::from_gray(150));
                            let color = color32(*color);
                            ui.painter().rect_filled(rect.shrink(1.0), 2.0, color);
                            response.on_hover_text(format!(
                                "#{:02X}{:02X}{:02X}",
                                color.r(),
                                color.g(),
                                color.b()
                            ));
                        }
                        if ui
                            .add_sized(
                                [ui.available_width(), 28.0],
                                egui::Button::selectable(selected == key, label),
                            )
                            .on_hover_text(format!("编辑 {key} 的头部代码、尾部代码和说明"))
                            .clicked()
                        {
                            *selected = key.clone();
                        }
                    });
                }
            });
        if let Some(rule) = rules.get_mut(selected) {
            gcode_code_pair(
                &mut columns[1],
                &mut rule.header,
                &mut rule.footer,
                &mut rule.comment,
                false,
            );
        }
    });
}

fn transfer_settings_ui(ui: &mut egui::Ui, config: &mut cam_trajectory::TransferConfig) {
    ui.separator();
    ui.checkbox(&mut config.enabled, "启用同面联动抬落刀")
        .on_hover_text("同区位且 A 不变时使用联动抬刀、平移和落刀；跨面或旋转使用安全净空");
    egui::Grid::new("transfer_settings")
        .num_columns(2)
        .spacing([12.0, 6.0])
        .min_row_height(22.0)
        .show(ui, |ui| {
            gcode_number_row(
                ui,
                "最小安全抬刀高度",
                &mut config.retract_height,
                0.1,
                "离开上一加工单元前垂直抬刀的最小高度",
            );
            gcode_number_row(
                ui,
                "最小安全落刀高度",
                &mut config.approach_height,
                0.1,
                "进入下一加工单元前垂直落刀的最小高度",
            );
            ui.label("联动总高度")
                .on_hover_text("同面为相对两端较高点的峰高；跨区位平滑时作为旋转净空上方的额外过渡高度。不能小于最小抬刀和落刀高度。");
            ui.add_enabled(
                !config.auto_height,
                egui::DragValue::new(&mut config.height).speed(0.1),
            )
            .on_hover_text("启用自动总高度时由两个最小高度的较大值决定");
            ui.end_row();
            let mut rise = config.rise_ratio * 100.0;
            ui.label("抬刀水平占比")
                .on_hover_text("抬升阶段占 XY 总行程的百分比，与落刀占比之和不能超过 100%");
            if ui
                .add(egui::DragValue::new(&mut rise).speed(1.0).suffix(" %"))
                .changed()
            {
                config.rise_ratio = rise / 100.0;
            }
            ui.end_row();
            let mut fall = config.fall_ratio * 100.0;
            ui.label("落刀水平占比")
                .on_hover_text("下降阶段占 XY 总行程的百分比，与抬刀占比之和不能超过 100%");
            if ui
                .add(egui::DragValue::new(&mut fall).speed(1.0).suffix(" %"))
                .changed()
            {
                config.fall_ratio = fall / 100.0;
            }
            ui.end_row();
        });
    ui.checkbox(&mut config.auto_height, "总高度取最小抬落高度的较大值")
        .on_hover_text("自动总高度不再单独读取联动总高度输入值");
    ui.separator();
    ui.checkbox(&mut config.smooth_enabled, "启用空移路径平滑（同区位 / 跨区位）")
        .on_hover_text("最小安全抬落段保持竖直。同面需启用联动；跨区位在旋转净空以上增加过渡高度，XY/Z/A 联动后输出离散线段。空间不足时保留原路。");
    ui.horizontal(|ui| {
        ui.label("平滑允许偏差")
            .on_hover_text("曲线离散及 XYZ 输出舍入的总允许偏差，必须大于输出精度产生的舍入误差");
        ui.add_enabled(
            config.smooth_enabled,
            egui::DragValue::new(&mut config.smooth_tolerance).speed(0.01),
        );
    });
}

fn transfer_diagram_ui(
    ui: &mut egui::Ui,
    config: &cam_trajectory::TransferConfig,
    example: &mut TransferExample,
    decimals: usize,
) {
    let TransferExample {
        distance,
        delta_z,
        cross_zone,
        delta_a,
    } = example;
    ui.label("空移剖面示意").on_hover_text(
        "横轴为 XY 行程，纵轴为 Z，非等比例；与实际转移共用几何计算，不含控制器加减速或碰撞模型",
    );
    ui.checkbox(cross_zone, "示例：跨区位转位")
        .on_hover_text("仅切换示意。同面与跨区位使用相同的实际转移函数，示例参数不保存到工艺。");
    egui::Grid::new("transfer_example")
        .num_columns(2)
        .spacing([12.0, 6.0])
        .show(ui, |ui| {
            ui.label("示例 XY 距离")
                .on_hover_text("仅用于右侧示意，不保存到配置");
            ui.add(egui::DragValue::new(distance).range(0.0..=1.0e6).speed(1.0));
            ui.end_row();
            ui.label("终点 Z 差")
                .on_hover_text("仅用于右侧示意，不保存到配置");
            ui.add(egui::DragValue::new(delta_z).speed(0.1));
            ui.end_row();
            if *cross_zone {
                ui.label("示例 A 转角");
                ui.add(egui::DragValue::new(delta_a).speed(1.0).suffix(" °"));
                ui.end_row();
            }
        });
    if let Some(error) = config.validation_error() {
        ui.colored_label(egui::Color32::LIGHT_RED, error);
        return;
    }
    let start = cam_trajectory::MachinePose {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        a: 0.0,
    };
    let end = cam_trajectory::MachinePose {
        x: *distance,
        y: 0.0,
        z: *delta_z,
        a: if *cross_zone { *delta_a } else { 0.0 },
    };
    let moves = cam_trajectory::transfer_moves(
        Some(start),
        end,
        config.total_height().max(*delta_z) + 1.0,
        !*cross_zone,
        *config,
        0.0,
        decimals,
    );
    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), ui.available_height().max(80.0)),
        egui::Sense::hover(),
    );
    response.on_hover_text("示意参数不保存；横轴 XY 行程、纵轴 Z 非等比。蓝色为安全抬刀，橙色为联动抬升，青色为联动下降，绿色为安全落刀。");
    let painter = ui.painter();
    painter.rect_filled(rect, 4.0, egui::Color32::from_rgb(24, 30, 38));
    let plot = rect.shrink(22.0);
    let points: Vec<_> = std::iter::once(start)
        .chain(moves.iter().map(|m| m.pose))
        .collect();
    let min_z = points.iter().map(|p| p.z).fold(0.0f64, f64::min);
    let max_z = points.iter().map(|p| p.z).fold(1.0f64, f64::max);
    let project = |p: cam_trajectory::MachinePose| {
        egui::pos2(
            plot.left() + (p.x / distance.max(1.0)) as f32 * plot.width(),
            plot.bottom() - ((p.z - min_z) / (max_z - min_z).max(1.0)) as f32 * plot.height(),
        )
    };
    for (i, pair) in points.windows(2).enumerate() {
        let color = if i == 0 {
            egui::Color32::from_rgb(60, 130, 255)
        } else if i + 2 == points.len() {
            egui::Color32::GREEN
        } else if pair[1].z > pair[0].z + 1.0e-9 {
            egui::Color32::from_rgb(255, 160, 40)
        } else if pair[1].z < pair[0].z - 1.0e-9 {
            egui::Color32::from_rgb(40, 190, 220)
        } else {
            egui::Color32::LIGHT_GRAY
        };
        painter.line_segment(
            [project(pair[0]), project(pair[1])],
            egui::Stroke::new(2.0, color),
        );
    }
    painter.text(
        plot.left_bottom() + egui::vec2(0.0, 8.0),
        egui::Align2::LEFT_TOP,
        "加工结束",
        egui::FontId::proportional(11.0),
        egui::Color32::WHITE,
    );
    painter.text(
        plot.right_bottom() + egui::vec2(0.0, 8.0),
        egui::Align2::RIGHT_TOP,
        "下一入口",
        egui::FontId::proportional(11.0),
        egui::Color32::WHITE,
    );
}

fn sync_machine_start(state: &mut ViewerState) {
    let start = [
        state.gcode_profile.initial_x,
        state.gcode_profile.initial_y,
        state.gcode_profile.initial_z,
    ];
    let ViewerState {
        machining: display,
        cam,
        analysis,
        ..
    } = &mut *state;
    analysis.set_machine_start(display, cam, start);
}

fn rebuild_state_trajectory(state: &mut ViewerState) {
    if let Some(error) = &state.machining_error {
        state.trajectory = Some(cam_trajectory::MachineTrajectory {
            error: Some(error.clone()),
            ..Default::default()
        });
        return;
    }
    sync_machine_start(state);
    let ViewerState {
        machining: display,
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

fn apply_cad_action(
    state: &mut ViewerState,
    selection: &mut SelectionState,
    action: cad_edit::EditAction,
) -> bool {
    if state.drawing.is_none() {
        return false;
    }
    match action {
        cad_edit::EditAction::BeginGrip { id, kind, target } => {
            let doc = state.drawing.as_ref().unwrap().document();
            let Some(entity) = doc.entity(id) else {
                return false;
            };
            if !entity.visible
                || doc
                    .layer(entity.layer_id)
                    .is_some_and(|l| l.locked || !l.visible)
            {
                push_message(state, "请先显示并解锁图层");
                return false;
            }
            match state.cad_command.begin_grip(
                id,
                entity.geometry.clone(),
                kind,
                Some(cad_core::model::Point3::new(
                    target[0], target[1], target[2],
                )),
                false,
            ) {
                Ok(()) => {
                    state.command_open = true;
                    state.command_input.clear();
                    return true;
                }
                Err(error) => {
                    push_message(state, error);
                    return false;
                }
            }
        }
        action @ (cad_edit::EditAction::Apply(_)
        | cad_edit::EditAction::Create(_)
        | cad_edit::EditAction::Transform { .. }
        | cad_edit::EditAction::Geometry(_)
        | cad_edit::EditAction::Curves(_)
        | cad_edit::EditAction::Layer(_)) => {
            let geometry_changed = match &action {
                cad_edit::EditAction::Apply(edit) => edit.changes_geometry(),
                cad_edit::EditAction::Layer(_) => false,
                _ => true,
            };
            let curve_results = if let cad_edit::EditAction::Curves(edit) = &action {
                if edit.copies {
                    Some(BTreeSet::new())
                } else {
                    Some(
                        edit.replacements
                            .keys()
                            .map(|id| id.0)
                            .collect::<BTreeSet<_>>(),
                    )
                }
            } else {
                None
            };
            let mut before = capture_cad_context(state, selection, geometry_changed);
            let rename =
                if let cad_edit::EditAction::Layer(cad_core::LayerEdit::Rename { id, name }) =
                    &action
                {
                    let Some(layer) = state.drawing.as_ref().unwrap().document().layer(*id) else {
                        return false;
                    };
                    let old = layer.name.clone();
                    let new = name.clone();
                    if old != new
                        && state.gcode_profile.layer_rules.contains_key(&old)
                        && state.gcode_profile.layer_rules.contains_key(&new)
                    {
                        push_message(
                            state,
                            "图层改名取消：目标名称已有 G 代码规则，请先处理规则冲突",
                        );
                        return false;
                    }
                    for key in [&old, &new] {
                        before.layer_rules.insert(
                            key.clone(),
                            state.gcode_profile.layer_rules.get(key).cloned(),
                        );
                    }
                    Some((old, new))
                } else {
                    None
                };
            let ids = cad_edit::entity_ids(&selection.selected);
            let drawing = state.drawing.as_mut().unwrap();
            let result = match action {
                cad_edit::EditAction::Apply(edit) => drawing.apply(&ids, edit),
                cad_edit::EditAction::Create(entities) => drawing.create(&entities),
                cad_edit::EditAction::Geometry(updates) => drawing.set_geometry(&updates),
                cad_edit::EditAction::Curves(edit) => drawing.apply_curves(&edit),
                cad_edit::EditAction::Layer(edit) => drawing.edit_layer(edit),
                cad_edit::EditAction::Transform { transforms, copies } => {
                    drawing.transform(&ids, &transforms, copies)
                }
                _ => unreachable!(),
            };
            match result {
                Ok(None) => {
                    push_message(state, "没有需要修改的图元");
                    return true;
                }
                Ok(Some(change)) => {
                    if let Some((old, new)) = rename {
                        if old != new {
                            if let Some(rule) = state.gcode_profile.layer_rules.remove(&old) {
                                state.gcode_profile.layer_rules.insert(new.clone(), rule);
                            }
                        }
                        if state.gcode_selected_layer == old {
                            state.gcode_selected_layer = new;
                        }
                    }
                    state.cad_history.insert(change.transaction_id, before);
                    refresh_edited_drawing(state, &change, None);
                    selection.cam_selected.clear();
                    if !change.created.is_empty() {
                        selection.selected = change.created.iter().map(|id| id.0).collect();
                    }
                    if let Some(mut results) = curve_results {
                        results.extend(change.created.iter().map(|id| id.0));
                        selection.selected = results;
                    }
                    let operation = change.kind.label();
                    push_message(
                        state,
                        format!(
                            "已{operation} {} 个图元，可用 Ctrl+Z 撤销本次批量操作",
                            change.affected
                        ),
                    );
                    if change.kind.changes_geometry() {
                        push_message(
                            state,
                            "已根据当前几何重建 CAM；手动断面、配对和排序需重新确认",
                        );
                    }
                }
                Err(error) => {
                    push_message(state, format!("CAD 编辑失败：{error}"));
                    return false;
                }
            }
        }
        cad_edit::EditAction::Undo | cad_edit::EditAction::Redo => {
            let redo = matches!(action, cad_edit::EditAction::Redo);
            let drawing = state.drawing.as_mut().unwrap();
            let change = if redo { drawing.redo() } else { drawing.undo() };
            let Some(change) = change else {
                return false;
            };
            // Swap the context for this transaction so undo and redo stay symmetric.
            let mut current = capture_cad_context(state, selection, change.kind.changes_geometry());
            if let Some(context) = state.cad_history.get(&change.transaction_id) {
                for key in context.layer_rules.keys() {
                    current.layer_rules.insert(
                        key.clone(),
                        state.gcode_profile.layer_rules.get(key).cloned(),
                    );
                }
            }
            let restored = state
                .cad_history
                .insert(change.transaction_id, current)
                .expect("every committed CAD transaction has an application context");
            for (key, rule) in restored.layer_rules {
                match rule {
                    Some(rule) => {
                        state.gcode_profile.layer_rules.insert(key, rule);
                    }
                    None => {
                        state.gcode_profile.layer_rules.remove(&key);
                    }
                }
            }
            selection.selected = restored.selected;
            selection.cam_selected = restored.cam_selected;
            refresh_edited_drawing(state, &change, restored.machining);
            push_message(
                state,
                if redo {
                    "已重做上一次 CAD 批量修改"
                } else {
                    "已撤销上一次 CAD 批量修改"
                },
            );
        }
    }
    let retained: BTreeSet<_> = state.drawing.as_ref().unwrap().history_ids().collect();
    state.cad_history.retain(|id, _| retained.contains(id));
    let valid: BTreeSet<_> = state
        .display
        .entities
        .iter()
        .filter(|entity| entity.visible)
        .map(|entity| entity.id)
        .collect();
    selection.selected.retain(|id| valid.contains(id));
    selection
        .cam_selected
        .retain(|id| state.cam.unit(*id).is_some());
    selection.drag_start = None;
    selection.drag_current = None;
    selection.reorder_clicks = None;
    selection.hover_hits.clear();
    selection.hover_cursor = None;
    selection.hover_index = 0;
    state.cad_controls = cad_edit::EditControls::default();
    state
        .cad_layers
        .reset(state.drawing.as_ref().unwrap().document());
    true
}

fn capture_cad_context(
    state: &ViewerState,
    selection: &SelectionState,
    geometry_changed: bool,
) -> CadUndoState {
    CadUndoState {
        layer_rules: BTreeMap::new(),
        selected: selection.selected.clone(),
        cam_selected: selection.cam_selected.clone(),
        machining: geometry_changed.then(|| {
            (
                state.analysis.clone(),
                state.manual_tube.clone(),
                state.manual_pair,
            )
        }),
    }
}

fn refresh_edited_drawing(
    state: &mut ViewerState,
    change: &DrawingChange,
    restored: Option<(
        cam_analysis::CamAnalysis,
        cam_analysis::TubeSection,
        [Option<cam::UnitId>; 2],
    )>,
) {
    let drawing = state.drawing.as_ref().unwrap();
    let document = drawing.document();
    state.display = display::build_display(document);
    match display::build_machining(document) {
        Ok(data) => {
            state.machining = data;
            state.machining_error = None;
        }
        Err(error) => {
            state.machining = Default::default();
            state.machining_error = Some(error);
        }
    }
    state.entity_count = document.entity_count();
    state.layer_count = document.layers.len();
    state.warning_count = document.import_report.warnings.len();
    if change.kind.changes_geometry() {
        state.file_ui.coverage_acknowledged = false;
        state.cam = cam::build_cam_model(document);
        if let Some((analysis, manual_tube, manual_pair)) = restored {
            state.analysis = analysis;
            state.manual_tube = manual_tube;
            state.manual_pair = manual_pair;
        } else {
            let lazy = state.analysis.lazy_rotation;
            let direction = state.analysis.rotation_direction;
            state.analysis = cam_analysis::CamAnalysis::build(&state.machining, &state.cam);
            state
                .analysis
                .set_plan_options(lazy, direction, &state.machining, &state.cam);
            state.manual_pair = [None, None];
        }
    }
    rebuild_state_trajectory(state);
}

fn export_dxf_file(state: &mut ViewerState) {
    if state.drawing.is_none() {
        return;
    }
    let stem = Path::new(&state.file_name)
        .file_stem()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_else(|| "drawing".to_string());
    let Some(path) = rfd::FileDialog::new()
        .set_title("导出 CAD 图纸为 DXF")
        .add_filter("DXF 图纸", &["dxf"])
        .set_directory(dialog_directory(state))
        .set_file_name(format!("{stem}-edited.dxf"))
        .save_file()
    else {
        return;
    };
    match state.drawing.as_mut().unwrap().export_dxf(&path) {
        Ok(()) => push_message(state, format!("已导出当前 CAD 图纸：{}", path.display())),
        Err(error) => push_message(state, format!("DXF 导出失败：{error}")),
    }
}

fn confirm_import_coverage(state: &mut ViewerState) -> bool {
    if state
        .drawing
        .as_ref()
        .is_some_and(|d| d.document().import_report.has_omissions())
        && !state.file_ui.coverage_acknowledged
    {
        state.file_ui.report_open = true;
        push_message(
            state,
            "导出暂停：请在导入报告中核对遗漏对象并明确确认加工范围",
        );
        return false;
    }
    true
}

fn export_gcode_file(state: &mut ViewerState) {
    if !confirm_import_coverage(state) {
        return;
    }

    rebuild_state_trajectory(state);
    let Some(trajectory) = state.trajectory.as_ref() else {
        push_message(state, "没有可导出的四轴轨迹，请先完成 CAM 分析");
        return;
    };
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
    if !confirm_import_coverage(state) {
        return;
    }
    rebuild_state_trajectory(state);
    let Some(trajectory) = state.trajectory.as_ref() else {
        push_message(state, "没有可导出的四轴轨迹，请先完成 CAM 分析");
        return;
    };
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
        .add_filter(
            "CAD 图纸与备份 (*.dxf, *.dwg, *.bak)",
            &["dxf", "dwg", "bak"],
        )
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
