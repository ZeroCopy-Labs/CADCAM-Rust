//! Bevy + wgpu desktop viewer for the CAD core.
//!
//! The application is deliberately thin: cad-core owns parsing/model data, this
//! crate tessellates it into 2D display paths, Bevy renders them with gizmos,
//! and egui provides the small amount of desktop UI.

use bevy::{
    prelude::*,
    window::{PrimaryWindow, WindowPlugin},
};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};

use cad_core::read_document;

mod display;

const BACKGROUND: Color = Color::srgb(0.035, 0.045, 0.060);

/// Everything the viewer keeps after a drawing is loaded.
#[derive(Resource, Default)]
struct ViewerState {
    path_input: String,
    file_name: String,
    format: String,
    version: String,
    unit: String,
    entity_count: usize,
    layer_count: usize,
    warning_count: usize,
    status: String,
    display: display::DisplayData,
    fit_dirty: bool,
    rotation_deg: f32,
}

/// Marker for the single 2D camera.
#[derive(Component)]
struct MainCamera;

fn main() {
    let initial_path = std::env::args().nth(1).unwrap_or_default();
    let mut state = ViewerState {
        path_input: initial_path.clone(),
        status: "Open a DXF/DWG file to begin".to_string(),
        ..Default::default()
    };
    if !initial_path.trim().is_empty() {
        open_drawing(&mut state, &initial_path);
    }

    App::new()
        .insert_resource(ClearColor(BACKGROUND))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CAD/CAM — Bevy Viewer".to_string(),
                resolution: (1440, 900).into(),
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
        state.status = "Enter a DXF/DWG path".to_string();
        return;
    }

    state.path_input = path.clone();
    state.status = format!("Loading {path}");
    match read_document(&path) {
        Ok(document) => {
            let display = display::build_display(&document);
            state.file_name = document.source.file_name.clone();
            state.format = document.source.format.to_string();
            state.version = document.source.version.clone();
            state.unit = document
                .units
                .map(|unit| unit.to_string())
                .unwrap_or_else(|| "unspecified".to_string());
            state.entity_count = document.entity_count();
            state.layer_count = document.layers.len();
            state.warning_count = document.import_report.warnings.len();
            state.display = display;
            state.status = format!("Loaded {} entities", state.entity_count);
            state.rotation_deg = 0.0;
            state.fit_dirty = true;
        }
        Err(error) => {
            state.status = format!("Load failed: {error}");
        }
    }
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
    let viewport_width = window.width().max(1.0) as f64;
    let viewport_height = window.height().max(1.0) as f64;
    let scale = ((width / viewport_width).max(height / viewport_height) * 1.12) as f32;

    let (mut transform, mut projection) = camera.into_inner();
    transform.translation.x = ((bounds.min_x + bounds.max_x) * 0.5) as f32;
    transform.translation.y = ((bounds.min_y + bounds.max_y) * 0.5) as f32;
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

fn drawing_ui(mut contexts: EguiContexts, mut state: ResMut<ViewerState>) -> Result {
    let context = contexts.ctx_mut()?;
    context.set_visuals(egui::Visuals::dark());

    let mut open_clicked = false;
    let mut fit_clicked = false;

    egui::Window::new("CAD/CAM")
        .default_pos([16.0, 16.0])
        .default_width(420.0)
        .resizable(true)
        .show(context, |ui| {
            ui.label("Drawing path");
            ui.horizontal(|ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut state.path_input)
                        .hint_text("D:\\CAD\\part.dxf")
                        .desired_width(300.0),
                );
                open_clicked = ui.button("Open").clicked();
            });

            ui.horizontal(|ui| {
                fit_clicked = ui.button("Fit view").clicked();
                if ui.button("Clear log").clicked() {
                    state.status = "Open a DXF/DWG file to begin".to_string();
                }
            });

            let rotation =
                ui.add(egui::Slider::new(&mut state.rotation_deg, 0.0..=360.0).text("Rotation"));
            if rotation.changed() {
                fit_clicked = true;
            }

            ui.separator();
            ui.label(format!(
                "File: {}",
                if state.file_name.is_empty() {
                    "—"
                } else {
                    &state.file_name
                }
            ));
            ui.label(format!(
                "Entities: {}    Layers: {}",
                state.entity_count, state.layer_count
            ));
            ui.label(format!(
                "Format: {}    Version: {}",
                if state.format.is_empty() {
                    "—"
                } else {
                    &state.format
                },
                if state.version.is_empty() {
                    "—"
                } else {
                    &state.version
                }
            ));
            ui.label(format!(
                "Units: {}    Import warnings: {}",
                state.unit, state.warning_count
            ));
            ui.separator();
            ui.label("Status");
            ui.label(&state.status);
        });

    if open_clicked {
        let path = state.path_input.clone();
        open_drawing(&mut state, &path);
    }
    if fit_clicked {
        state.fit_dirty = true;
    }
    Ok(())
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
