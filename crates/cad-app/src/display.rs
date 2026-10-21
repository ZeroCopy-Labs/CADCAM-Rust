//! Display tessellation for the Bevy viewport.
//!
//! This is a viewer adapter: it converts the internal CAD document into simple
//! 3D line paths. It is not a geometry kernel and does not persist or modify
//! the CAD model.

use std::f64::consts::TAU;

use cad_core::model::{
    Color, DrawingDocument, Entity, Geometry, Plane, Point3, Polyline, PolylineSegment, Spline,
};

/// World bounds of all display paths, in CAD coordinates.
#[derive(Clone, Copy, Debug)]
pub struct Bounds {
    pub min_x: f64,
    pub min_y: f64,
    pub min_z: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub max_z: f64,
}

/// One drawable polyline with its display color.
#[derive(Clone, Debug)]
pub struct DisplayPath {
    pub color: [f32; 3],
    pub points: Vec<[f64; 3]>,
}

/// All data the viewport needs after a drawing has been loaded.
#[derive(Clone, Debug, Default)]
pub struct DisplayData {
    pub paths: Vec<DisplayPath>,
    pub bounds: Option<Bounds>,
}

/// Convert a drawing into 3D line paths for the Bevy scene.
///
/// Arc/circle/ellipse points use their stored plane basis; splines are evaluated
/// in 3D. The viewer only does the final CAD-to-render axis conversion.
pub fn build_display(document: &DrawingDocument) -> DisplayData {
    let mut paths = Vec::new();
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut min_z = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut max_z = f64::NEG_INFINITY;
    let mut has_points = false;

    for entity in &document.entities {
        let color = resolve_color(entity, document);

        for path in tessellate_geometry(&entity.geometry) {
            let points: Vec<[f64; 3]> = path
                .into_iter()
                .filter(|point| point.iter().all(|value| value.is_finite()))
                .collect();
            if points.len() < 2 {
                continue;
            }

            for point in &points {
                min_x = min_x.min(point[0]);
                min_y = min_y.min(point[1]);
                min_z = min_z.min(point[2]);
                max_x = max_x.max(point[0]);
                max_y = max_y.max(point[1]);
                max_z = max_z.max(point[2]);
            }
            has_points = true;

            paths.push(DisplayPath { color, points });
        }
    }

    let bounds = has_points.then_some(Bounds {
        min_x,
        min_y,
        min_z,
        max_x,
        max_y,
        max_z,
    });

    DisplayData { paths, bounds }
}

/// Resolve the display color from the entity, falling back to its layer color.
fn resolve_color(entity: &Entity, document: &DrawingDocument) -> [f32; 3] {
    let layer_color = document
        .layers
        .iter()
        .find(|layer| layer.id == entity.layer_id)
        .map(|layer| layer.color)
        .unwrap_or(Color::Aci(7));

    match entity.color {
        Color::ByLayer | Color::ByBlock | Color::None => color_to_rgb(layer_color),
        color => color_to_rgb(color),
    }
}

/// Convert the model's color representation to normalized RGB.
fn color_to_rgb(color: Color) -> [f32; 3] {
    match color {
        Color::Rgb { r, g, b } => [
            f32::from(r) / 255.0,
            f32::from(g) / 255.0,
            f32::from(b) / 255.0,
        ],
        Color::Aci(index) => aci_color(index),
        Color::ByLayer | Color::ByBlock | Color::None => [0.86, 0.89, 0.94],
    }
}

/// Small ACI lookup used by the viewer. Unknown indices fall back to gray.
fn aci_color(index: u8) -> [f32; 3] {
    match index {
        1 => [1.00, 0.32, 0.32],
        2 => [1.00, 0.84, 0.28],
        3 => [0.36, 0.86, 0.42],
        4 => [0.32, 0.84, 0.94],
        5 => [0.42, 0.56, 1.00],
        6 => [0.88, 0.44, 0.94],
        7 => [0.92, 0.94, 0.98],
        8 => [0.56, 0.60, 0.66],
        9 => [0.72, 0.76, 0.82],
        _ => [0.82, 0.86, 0.92],
    }
}

/// Dispatch one internal geometry value to its display tessellator.
fn tessellate_geometry(geometry: &Geometry) -> Vec<Vec<[f64; 3]>> {
    match geometry {
        Geometry::Line(line) => vec![vec![to_array(line.start), to_array(line.end)]],
        Geometry::Polyline(polyline) => polyline_paths(polyline),
        Geometry::Circle(circle) => vec![sample_arc(
            circle.center,
            circle.radius,
            0.0,
            TAU,
            circle.plane,
            64,
        )],
        Geometry::Arc(arc) => vec![sample_arc(
            arc.center,
            arc.radius,
            arc.start_angle,
            arc.sweep_angle,
            arc.plane,
            segments_for_sweep(arc.sweep_angle),
        )],
        Geometry::Ellipse(ellipse) => vec![sample_ellipse(
            ellipse.center,
            ellipse.major_axis,
            ellipse.minor_axis,
            ellipse.start_parameter,
            ellipse.sweep_parameter,
            64,
        )],
        Geometry::Spline(spline) => vec![sample_spline(spline)],
    }
}

/// Walk a polyline and merge its line/arc segments into one 3D display path.
/// Arc segments use the polyline's shared plane; 3D polylines have line segments
/// only by construction.
fn polyline_paths(polyline: &Polyline) -> Vec<Vec<[f64; 3]>> {
    let mut points = Vec::new();
    let plane = polyline.plane;

    for segment in &polyline.segments {
        match segment {
            PolylineSegment::Line { start, end } => {
                if points.is_empty() {
                    points.push(to_array(*start));
                }
                points.push(to_array(*end));
            }
            PolylineSegment::Arc {
                start,
                end,
                center,
                radius,
                start_angle,
                sweep_angle,
            } => {
                if points.is_empty() {
                    points.push(to_array(*start));
                }

                if let Some(plane) = plane {
                    let arc = sample_arc(
                        *center,
                        *radius,
                        *start_angle,
                        *sweep_angle,
                        plane,
                        segments_for_sweep(*sweep_angle),
                    );
                    for point in arc.into_iter().skip(1) {
                        points.push(point);
                    }
                } else {
                    // A 3D-source polyline has line segments only; this is a
                    // defensive fallback for malformed input.
                    points.push(to_array(*end));
                }
            }
        }
    }

    if points.len() >= 2 {
        vec![points]
    } else {
        Vec::new()
    }
}

/// Sample a 3D circular arc using the plane basis.
fn sample_arc(
    center: Point3,
    radius: f64,
    start_angle: f64,
    sweep_angle: f64,
    plane: Plane,
    segments: usize,
) -> Vec<[f64; 3]> {
    if !radius.is_finite() || radius.abs() <= 1.0e-12 || !sweep_angle.is_finite() {
        return Vec::new();
    }

    let sweep_angle = sweep_angle.abs().min(TAU) * sweep_angle.signum();
    if sweep_angle.abs() <= 1.0e-12 {
        return Vec::new();
    }

    let y_axis = plane.y_axis();
    let segments = segments.max(2);
    let mut points = Vec::with_capacity(segments + 1);

    for index in 0..=segments {
        let angle = start_angle + sweep_angle * (index as f64 / segments as f64);
        let (sin, cos) = angle.sin_cos();
        let point = Point3::new(
            center.x + radius * (cos * plane.x_axis.x + sin * y_axis.x),
            center.y + radius * (cos * plane.x_axis.y + sin * y_axis.y),
            center.z + radius * (cos * plane.x_axis.z + sin * y_axis.z),
        );
        points.push(to_array(point));
    }

    points
}

/// Sample an ellipse parameter range in 3D.
fn sample_ellipse(
    center: Point3,
    major_axis: cad_core::model::Vector3,
    minor_axis: cad_core::model::Vector3,
    start_parameter: f64,
    sweep_parameter: f64,
    segments: usize,
) -> Vec<[f64; 3]> {
    if !sweep_parameter.is_finite() || sweep_parameter.abs() <= 1.0e-12 {
        return Vec::new();
    }

    let segments = segments.max(2);
    let mut points = Vec::with_capacity(segments + 1);

    for index in 0..=segments {
        let parameter = start_parameter + sweep_parameter * (index as f64 / segments as f64);
        let (sin, cos) = parameter.sin_cos();
        let point = Point3::new(
            center.x + major_axis.x * cos + minor_axis.x * sin,
            center.y + major_axis.y * cos + minor_axis.y * sin,
            center.z + major_axis.z * cos + minor_axis.z * sin,
        );
        points.push(to_array(point));
    }

    points
}

/// Sample a NURBS spline in 3D. Falls back to the control polygon when the knot
/// or degree data is incomplete; this keeps malformed drawings visible.
fn sample_spline(spline: &Spline) -> Vec<[f64; 3]> {
    let control = &spline.control_points;
    let control_count = control.len();
    let degree = spline.degree as usize;

    if control_count < degree + 1 || control_count < 2 {
        return control.iter().map(|point| to_array(*point)).collect();
    }

    if spline.knots.len() < control_count + degree + 1 {
        return control.iter().map(|point| to_array(*point)).collect();
    }

    let start = spline.knots[degree];
    let end = spline.knots[control_count];
    if !start.is_finite() || !end.is_finite() || end <= start {
        return control.iter().map(|point| to_array(*point)).collect();
    }

    let segments = 64usize;
    let mut points = Vec::with_capacity(segments + 1);

    for index in 0..=segments {
        let parameter = if index == segments {
            end - (end - start) * 1.0e-9
        } else {
            start + (end - start) * (index as f64 / segments as f64)
        };

        if let Some(point) =
            evaluate_spline(control, &spline.weights, &spline.knots, degree, parameter)
        {
            points.push(to_array(point));
        }
    }

    if points.len() >= 2 {
        points
    } else {
        control.iter().map(|point| to_array(*point)).collect()
    }
}

/// Evaluate a rational B-spline point with the de Boor algorithm.
fn evaluate_spline(
    control: &[Point3],
    weights: &[f64],
    knots: &[f64],
    degree: usize,
    parameter: f64,
) -> Option<Point3> {
    let control_count = control.len();
    if control_count < degree + 1 || knots.len() < control_count + degree + 1 {
        return None;
    }

    let last_span = control_count - 1;
    let mut span = degree;
    for candidate in degree..control_count {
        span = candidate;
        if parameter >= knots[candidate] && parameter < knots[candidate + 1] {
            break;
        }
    }
    span = span.min(last_span);

    let mut values = Vec::with_capacity(degree + 1);
    for index in 0..=degree {
        let control_index = span.saturating_sub(degree) + index;
        if control_index >= control_count {
            return None;
        }

        let weight = weights
            .get(control_index)
            .copied()
            .filter(|value| value.is_finite() && value.abs() > 1.0e-12)
            .unwrap_or(1.0);
        let point = control[control_index];
        values.push([point.x * weight, point.y * weight, point.z * weight, weight]);
    }

    for level in 1..=degree {
        for index in (level..=degree).rev() {
            let knot_index = span.saturating_sub(degree) + index;
            let denominator = knots[knot_index + degree - level + 1] - knots[knot_index];
            let alpha = if denominator.abs() > 1.0e-12 {
                (parameter - knots[knot_index]) / denominator
            } else {
                0.0
            };

            for component in 0..4 {
                values[index][component] =
                    (1.0 - alpha) * values[index - 1][component] + alpha * values[index][component];
            }
        }
    }

    let weight = values[degree][3];
    if !weight.is_finite() || weight.abs() <= 1.0e-12 {
        return Some(control[span.min(last_span)]);
    }

    Some(Point3::new(
        values[degree][0] / weight,
        values[degree][1] / weight,
        values[degree][2] / weight,
    ))
}

/// Fixed screen-space-ish sampling density for arcs. A future version should
/// derive this from zoom/view tolerance.
fn segments_for_sweep(sweep_angle: f64) -> usize {
    let fraction = (sweep_angle.abs() / TAU).clamp(0.0, 1.0);
    ((fraction * 96.0).ceil() as usize).clamp(8, 256)
}

fn to_array(point: Point3) -> [f64; 3] {
    [point.x, point.y, point.z]
}
