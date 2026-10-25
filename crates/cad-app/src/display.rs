//! Display tessellation for the Bevy viewport.
//!
//! This is a viewer adapter: it converts the internal CAD document into simple
//! 3D line paths. It is not a geometry kernel and does not persist or modify
//! the CAD model.

use std::collections::{BTreeMap, BTreeSet};
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

/// One drawable polyline with its display color and owning entity.
#[derive(Clone, Debug)]
pub struct DisplayPath {
    pub entity_id: u32,
    pub visible: bool,
    pub color: [f32; 3],
    pub points: Vec<[f64; 3]>,
}

/// One drawing layer, retained so configuration UIs can list every layer.
#[derive(Clone, Debug)]
pub struct DisplayLayer {
    pub name: String,
    pub color: [f32; 3],
    pub color_index: Option<u8>,
}

/// Basic display metadata for one CAD entity.
#[derive(Clone, Debug)]
pub struct DisplayEntity {
    pub id: u32,
    pub kind: &'static str,
    pub visible: bool,
    pub locked: bool,
    pub layer: String,
    pub color: [f32; 3],
    pub layer_color: [f32; 3],
    pub color_key: String,
    /// Geometry-specific table rows, independent of the common entity fields.
    pub properties: Vec<(&'static str, String)>,
}

/// Source arc/circle metadata retained for G02/G03 output of planar units.
#[derive(Clone, Debug)]
pub struct DisplayArc {
    pub center: [f64; 3],
    pub normal: [f64; 3],
    pub full_circle: bool,
}

/// All data the viewport needs after a drawing has been loaded.
#[derive(Clone, Debug, Default)]
pub struct DisplayData {
    pub paths: Vec<DisplayPath>,
    pub entities: Vec<DisplayEntity>,
    pub layers: Vec<DisplayLayer>,
    pub arcs: BTreeMap<u32, DisplayArc>,
    pub bounds: Option<Bounds>,
}

/// Convert a drawing into 3D line paths for the Bevy scene.
///
/// Arc/circle/ellipse points use their stored plane basis; splines are evaluated
/// in 3D. The viewer only does the final CAD-to-render axis conversion.
pub fn build_display(document: &DrawingDocument) -> DisplayData {
    let mut paths = Vec::new();
    let mut entities = Vec::new();
    let mut arcs = BTreeMap::new();
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut min_z = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut max_z = f64::NEG_INFINITY;
    let mut has_points = false;

    for entity in &document.entities {
        let color = resolve_color(entity, document);
        let mut entity_paths = Vec::new();

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
            entity_paths.push(DisplayPath {
                entity_id: entity.id.0,
                visible: entity.visible
                    && document.layer(entity.layer_id).is_none_or(|l| l.visible),
                color,
                points,
            });
        }

        if entity_paths.is_empty() {
            continue;
        }
        paths.extend(entity_paths);
        match &entity.geometry {
            Geometry::Circle(circle) => {
                arcs.insert(
                    entity.id.0,
                    DisplayArc {
                        center: [circle.center.x, circle.center.y, circle.center.z],
                        normal: [
                            circle.plane.normal.x,
                            circle.plane.normal.y,
                            circle.plane.normal.z,
                        ],
                        full_circle: true,
                    },
                );
            }
            Geometry::Arc(arc) => {
                arcs.insert(
                    entity.id.0,
                    DisplayArc {
                        center: [arc.center.x, arc.center.y, arc.center.z],
                        normal: [arc.plane.normal.x, arc.plane.normal.y, arc.plane.normal.z],
                        full_circle: false,
                    },
                );
            }
            _ => {}
        }
        let layer = document
            .layers
            .iter()
            .find(|layer| layer.id == entity.layer_id);
        let layer_name = layer
            .map(|layer| layer.name.clone())
            .unwrap_or_else(|| "0".to_string());
        let layer_color = layer
            .map(|layer| color_to_rgb(layer.color))
            .unwrap_or([0.86, 0.89, 0.94]);
        let color_key = match entity.color {
            Color::ByLayer | Color::None => "BYLAYER".to_string(),
            Color::ByBlock => "BYBLOCK".to_string(),
            Color::Aci(index) => format!("ACI:{index}"),
            Color::Rgb { r, g, b } => format!("#{r:02X}{g:02X}{b:02X}"),
        };
        entities.push(DisplayEntity {
            id: entity.id.0,
            visible: entity.visible && document.layer(entity.layer_id).is_none_or(|l| l.visible),
            locked: document.layer(entity.layer_id).is_some_and(|l| l.locked),
            kind: entity.geometry.kind(),
            layer: layer_name,
            color,
            layer_color,
            color_key,
            properties: {
                let mut rows = entity_properties(&entity.geometry);
                rows.insert(0, ("原生句柄", format!("{:X}", entity.native_handle)));
                rows
            },
        });
    }

    let bounds = has_points.then_some(Bounds {
        min_x,
        min_y,
        min_z,
        max_x,
        max_y,
        max_z,
    });
    let layers = document
        .layers
        .iter()
        .map(|layer| DisplayLayer {
            name: layer.name.clone(),
            color: color_to_rgb(layer.color),
            color_index: match layer.color {
                Color::Aci(index) => Some(index),
                _ => None,
            },
        })
        .collect();

    DisplayData {
        paths,
        entities,
        layers,
        arcs,
        bounds,
    }
}

/// Refresh rule keys and viewport attributes without re-sampling any geometry.
pub fn refresh_properties(data: &mut DisplayData, document: &DrawingDocument) {
    let sources: BTreeMap<_, _> = document.entities.iter().map(|e| (e.id.0, e)).collect();
    let mut attributes = BTreeMap::new();
    for entity in &mut data.entities {
        let Some(source) = sources.get(&entity.id) else {
            continue;
        };
        let layer = document.layer(source.layer_id);
        entity.visible = source.visible && layer.is_none_or(|l| l.visible);
        entity.locked = layer.is_some_and(|l| l.locked);
        entity.layer = layer.map(|l| l.name.clone()).unwrap_or_else(|| "0".into());
        entity.color = resolve_color(source, document);
        entity.layer_color = layer
            .map(|l| color_to_rgb(l.color))
            .unwrap_or([0.86, 0.89, 0.94]);
        entity.color_key = match source.color {
            Color::ByLayer | Color::None => "BYLAYER".into(),
            Color::ByBlock => "BYBLOCK".into(),
            Color::Aci(i) => format!("ACI:{i}"),
            Color::Rgb { r, g, b } => format!("#{r:02X}{g:02X}{b:02X}"),
        };
        attributes.insert(entity.id, (entity.visible, entity.color));
    }
    for path in &mut data.paths {
        if let Some((visible, color)) = attributes.get(&path.entity_id) {
            path.visible = *visible;
            path.color = *color;
        }
    }
    data.layers = document
        .layers
        .iter()
        .map(|layer| DisplayLayer {
            name: layer.name.clone(),
            color: color_to_rgb(layer.color),
            color_index: match layer.color {
                Color::Aci(i) => Some(i),
                _ => None,
            },
        })
        .collect();
}

/// CAM paths are sampled from analytic geometry, independently of viewport tessellation.
pub fn build_machining(
    document: &DrawingDocument,
    excluded: &BTreeSet<u32>,
) -> Result<DisplayData, String> {
    let mut data = build_display(document);
    data.paths.clear();
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    let mut total = 0;
    for entity in &document.entities {
        if excluded.contains(&entity.id.0) {
            continue;
        }
        let points =
            cad_core::curves::sample(&entity.geometry, cad_core::curves::Sampling::default())
                .map_err(|e| format!("图元 #{} 加工采样失败：{e}", entity.id.0))?;
        total += points.len();
        if total > 2_000_000 {
            return Err("加工采样超出整图 2000000 点预算，未输出部分刀路".into());
        }
        for p in &points {
            for (i, v) in [p.x, p.y, p.z].into_iter().enumerate() {
                min[i] = min[i].min(v);
                max[i] = max[i].max(v);
            }
        }
        data.paths.push(DisplayPath {
            entity_id: entity.id.0,
            visible: entity.visible && document.layer(entity.layer_id).is_none_or(|l| l.visible),
            color: resolve_color(entity, document),
            points: points.into_iter().map(to_array).collect(),
        });
    }
    data.bounds = (!data.paths.is_empty()).then_some(Bounds {
        min_x: min[0],
        min_y: min[1],
        min_z: min[2],
        max_x: max[0],
        max_y: max[1],
        max_z: max[2],
    });
    Ok(data)
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
pub(crate) fn color_to_rgb(color: Color) -> [f32; 3] {
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

/// Resolve the complete ACI palette used by both DXF output and CAD editing.
pub fn aci_color(index: u8) -> [f32; 3] {
    cad_core::aci_color_rgb(index)
        .map(|rgb| rgb.map(|value| f32::from(value) / 255.0))
        .unwrap_or([0.82, 0.86, 0.92])
}

/// Type-specific table rows for the selection panel.
fn entity_properties(geometry: &Geometry) -> Vec<(&'static str, String)> {
    match geometry {
        Geometry::Line(line) => vec![
            ("起点", format_point(line.start)),
            ("终点", format_point(line.end)),
            ("长度", format!("{:.3}", distance(line.start, line.end))),
        ],
        Geometry::Polyline(polyline) => {
            let start = polyline.segments.first().map(|segment| match segment {
                PolylineSegment::Line { start, .. } => *start,
                PolylineSegment::Arc { start, .. } => *start,
            });
            let end = polyline.segments.last().map(|segment| match segment {
                PolylineSegment::Line { end, .. } => *end,
                PolylineSegment::Arc { end, .. } => *end,
            });
            vec![
                ("段数", polyline.segments.len().to_string()),
                ("闭合", yes_no(polyline.closed)),
                (
                    "起点",
                    start.map(format_point).unwrap_or_else(|| "—".to_string()),
                ),
                (
                    "终点",
                    end.map(format_point).unwrap_or_else(|| "—".to_string()),
                ),
                (
                    "平面法向",
                    polyline
                        .plane
                        .map(|plane| format_vector(plane.normal))
                        .unwrap_or_else(|| "3D（无线段平面）".to_string()),
                ),
            ]
        }
        Geometry::Circle(circle) => vec![
            ("圆心", format_point(circle.center)),
            ("半径", format!("{:.3}", circle.radius)),
            ("平面法向", format_vector(circle.plane.normal)),
        ],
        Geometry::Arc(arc) => vec![
            ("圆心", format_point(arc.center)),
            ("半径", format!("{:.3}", arc.radius)),
            ("起始角", format!("{:.3}°", arc.start_angle.to_degrees())),
            ("扫掠角", format!("{:.3}°", arc.sweep_angle.to_degrees())),
            ("平面法向", format_vector(arc.plane.normal)),
        ],
        Geometry::Ellipse(ellipse) => vec![
            ("中心", format_point(ellipse.center)),
            ("长半轴", format!("{:.3}", ellipse.major_axis.length())),
            ("短半轴", format!("{:.3}", ellipse.minor_axis.length())),
            (
                "起始参数",
                format!("{:.3}°", ellipse.start_parameter.to_degrees()),
            ),
            (
                "扫掠参数",
                format!("{:.3}°", ellipse.sweep_parameter.to_degrees()),
            ),
        ],
        Geometry::Spline(spline) => vec![
            ("阶数", spline.degree.to_string()),
            ("控制点数", spline.control_points.len().to_string()),
            ("节点数", spline.knots.len().to_string()),
            ("闭合", yes_no(spline.closed)),
            ("周期", yes_no(spline.periodic)),
            ("有理", yes_no(spline.rational)),
            ("平面", yes_no(spline.planar)),
        ],
    }
}

fn distance(start: Point3, end: Point3) -> f64 {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let dz = end.z - start.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn yes_no(value: bool) -> String {
    if value { "是" } else { "否" }.to_string()
}

fn format_point(point: Point3) -> String {
    format!("({:.2}, {:.2}, {:.2})", point.x, point.y, point.z)
}

fn format_vector(vector: cad_core::model::Vector3) -> String {
    format!("({:.3}, {:.3}, {:.3})", vector.x, vector.y, vector.z)
}

/// Dispatch one internal geometry value to its display tessellator.
pub(crate) fn tessellate_geometry(geometry: &Geometry) -> Vec<Vec<[f64; 3]>> {
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
