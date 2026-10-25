//! Conversion from the parser document into the internal CAD model.
//!
//! This module is the bridge between `crate::import` (reading/parsing) and
//! `crate::model` (data model). It is the only place where `acadrust` entity
//! types are translated into `DrawingDocument`. The public entry point is
//! [`read_document`]: parse a file, then convert only model-space entities.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::f64::consts::TAU;
use std::path::Path;

use acadrust::entities::{
    Arc as NativeArc, Circle as NativeCircle, Ellipse as NativeEllipse, Line as NativeLine,
    LwPolyline as NativeLwPolyline, Polyline as NativePolyline, Polyline2D as NativePolyline2D,
    Polyline3D as NativePolyline3D, Spline as NativeSpline,
};
use acadrust::types::Vector3 as NativeVector3;
use acadrust::{CadDocument, Color as NativeColor, EntityType, Handle, ReadStats};

use crate::edit::EditableDrawing;

use crate::import::{read_native, ImportError};
use crate::model::{
    Arc, Circle, Color, DrawingDocument, Ellipse, Entity, EntityId, FileFormat, Geometry,
    ImportReport, Layer, LayerId, LengthUnit, Line, Plane, Point3, Polyline, PolylineSegment,
    SkipReason, SourceInfo, Spline, Vector3, GEOMETRY_EPSILON,
};

const BULGE_EPSILON: f64 = 1e-12;

/// Resolve an ACI entry using the parser's complete AutoCAD color table.
pub fn aci_color_rgb(index: u8) -> Option<[u8; 3]> {
    NativeColor::Index(index).rgb().map(|(r, g, b)| [r, g, b])
}

/// Read a DXF or DWG file and convert it into the parser-independent model.
///
/// This is the only public read API of the crate. It:
/// 1. lets `crate::import` detect and parse the file;
/// 2. converts model-space entities with `convert_document`;
/// 3. folds parser diagnostics/notifications into `ImportReport::warnings`.
pub fn read_document<P: AsRef<Path>>(path: P) -> Result<DrawingDocument, ImportError> {
    Ok(read_editable_document(path)?.into_document())
}

/// Read an editable drawing while retaining the native records for DXF export.
pub fn read_editable_document<P: AsRef<Path>>(path: P) -> Result<EditableDrawing, ImportError> {
    let path = path.as_ref();
    let native = read_native(path)?;
    let mut ids = BTreeMap::new();
    let mut layer_identities = BTreeMap::new();
    let mut document = convert_document(
        path,
        native.detection.format,
        &native.document,
        &mut ids,
        &mut layer_identities,
    );

    let mut warnings = Vec::new();
    if let Some(mismatch) = &native.detection.extension_mismatch {
        warnings.push(mismatch.clone());
    }
    append_stats_warnings(&mut warnings, &native.stats);
    append_notification_warnings(&mut warnings, &native.document);

    // The DWG header may carry a more useful version than the parser's
    // `UNKNOWN` fallback; only fill that gap, never override a parsed version.
    if document.source.version == "UNKNOWN" {
        if let Some(magic_version) = &native.detection.magic_version {
            document.source.version = magic_version.clone();
        }
    }

    document.import_report.warnings.extend(warnings);
    deduplicate(&mut document.import_report.warnings);
    Ok(EditableDrawing::new(
        document,
        native.document,
        ids,
        layer_identities,
    ))
}

/// Convert the parser document for `path` into the internal model.
pub(crate) fn convert_document(
    path: &Path,
    format: FileFormat,
    native: &CadDocument,
    ids: &mut BTreeMap<Handle, EntityId>,
    layer_identities: &mut BTreeMap<String, LayerId>,
) -> DrawingDocument {
    let model_space: Vec<&EntityType> = native.model_space_entities().collect();
    let total_entities = native.entities().count();
    let ignored_non_model_space = total_entities.saturating_sub(model_space.len());

    let mut report = ImportReport {
        source_entities: total_entities,
        model_space_entities: model_space.len(),
        ignored_non_model_space,
        ..Default::default()
    };
    if ignored_non_model_space > 0 {
        report.warn(format!(
            "ignored {ignored_non_model_space} entity(ies) outside model space \
             (paper space or block definitions)"
        ));
    }

    let mut layers = Vec::new();
    let mut layer_ids: HashMap<String, LayerId> = HashMap::new();
    for native_layer in native.layers.iter() {
        let next_id = LayerId(
            layer_identities
                .values()
                .map(|id| id.0)
                .max()
                .map_or(0, |id| id + 1),
        );
        let id = *layer_identities
            .entry(if native_layer.handle.is_valid() {
                format!("handle:{}", native_layer.handle.value())
            } else {
                format!("name:{}", layer_key(&native_layer.name))
            })
            .or_insert(next_id);
        layer_ids.insert(layer_key(&native_layer.name), id);
        layers.push(Layer {
            id,
            name: native_layer.name.clone(),
            color: convert_color(native_layer.color),
            visible: !native_layer.flags.off && !native_layer.flags.frozen,
            locked: native_layer.flags.locked,
        });
    }

    // Some drawings reference layers that are missing from the layer table.
    // Add them so that every entity has a valid layer id.
    for native_entity in &model_space {
        ensure_layer(
            &mut layers,
            &mut layer_ids,
            layer_identities,
            entity_layer_name(native_entity),
            &mut report,
        );
    }

    let source_units = convert_units(native.header.insertion_units, &mut report);
    let scale = source_units
        .map(LengthUnit::millimeters_per_unit)
        .unwrap_or(1.0);

    let mut entities = Vec::new();
    let mut next_id = ids.values().map(|id| id.0).max().map_or(0, |id| id + 1);
    for native_entity in model_space {
        let key = layer_key(entity_layer_name(native_entity));
        let layer_id = *layer_ids
            .get(&key)
            .expect("layer pre-pass must register every referenced layer");
        let color = convert_color(native_entity.as_entity().color());

        if let Some(mut geometry) = convert_geometry(native_entity, &mut report) {
            if !scale_geometry(&mut geometry, scale) {
                record_skip(
                    native_entity,
                    SkipReason::ConversionFailed,
                    "coordinates are non-finite after unit conversion",
                    &mut report,
                );
                continue;
            }
            let id = *ids.entry(native_entity.common().handle).or_insert_with(|| {
                let id = EntityId(next_id);
                next_id += 1;
                id
            });
            report.record_converted(geometry.kind());
            entities.push(Entity {
                id,
                layer_id,
                color,
                geometry,
            });
        }
    }

    let file_name = path
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.display().to_string());

    DrawingDocument {
        units: source_units.map(|_| LengthUnit::Millimeters),
        source_units,
        current_layer: layer_ids
            .get(&layer_key(&native.header.current_layer_name))
            .copied()
            .or_else(|| layer_ids.get("0").copied())
            .or_else(|| layers.first().map(|l| l.id)),
        layers,
        entities,
        source: SourceInfo {
            file_name,
            file_path: Some(path.to_path_buf()),
            format,
            version: native.version.to_string(),
        },
        import_report: report,
    }
}

fn scale_geometry(geometry: &mut Geometry, scale: f64) -> bool {
    let point = |p: &mut Point3| {
        p.x *= scale;
        p.y *= scale;
        p.z *= scale;
        p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
    };
    let radius = |r: &mut f64| {
        *r *= scale;
        r.is_finite() && *r > 0.0
    };
    match geometry {
        Geometry::Line(line) => point(&mut line.start) & point(&mut line.end),
        Geometry::Polyline(polyline) => polyline.segments.iter_mut().all(|segment| match segment {
            PolylineSegment::Line { start, end } => point(start) & point(end),
            PolylineSegment::Arc {
                start,
                end,
                center,
                radius,
                ..
            } => {
                point(start) & point(end) & point(center) & {
                    *radius *= scale;
                    radius.is_finite() && *radius > 0.0
                }
            }
        }),
        Geometry::Circle(circle) => point(&mut circle.center) & radius(&mut circle.radius),
        Geometry::Arc(arc) => point(&mut arc.center) & radius(&mut arc.radius),
        Geometry::Ellipse(ellipse) => {
            ellipse.major_axis = ellipse.major_axis * scale;
            ellipse.minor_axis = ellipse.minor_axis * scale;
            point(&mut ellipse.center)
                && [
                    ellipse.major_axis.x,
                    ellipse.major_axis.y,
                    ellipse.major_axis.z,
                    ellipse.minor_axis.x,
                    ellipse.minor_axis.y,
                    ellipse.minor_axis.z,
                ]
                .iter()
                .all(|value| value.is_finite())
        }
        Geometry::Spline(spline) => spline
            .control_points
            .iter_mut()
            .chain(&mut spline.fit_points)
            .all(point),
    }
}

fn convert_geometry(native: &EntityType, report: &mut ImportReport) -> Option<Geometry> {
    let result: Result<Geometry, String> = match native {
        EntityType::Line(line) => Ok(convert_line(line)),
        EntityType::Circle(circle) => convert_circle(circle),
        EntityType::Arc(arc) => convert_arc(arc),
        EntityType::Ellipse(ellipse) => convert_ellipse(ellipse),
        EntityType::Spline(spline) => convert_spline(spline),
        EntityType::LwPolyline(polyline) => convert_lw_polyline(polyline),
        EntityType::Polyline2D(polyline) => convert_polyline2d(polyline),
        EntityType::Polyline(polyline) => convert_polyline_3d_heavy(polyline),
        EntityType::Polyline3D(polyline) => convert_polyline3d(polyline),
        _ => {
            record_skip(
                native,
                SkipReason::Unsupported,
                "entity type is not represented by the internal model yet",
                report,
            );
            return None;
        }
    };

    match result {
        Ok(geometry) => Some(geometry),
        Err(message) => {
            record_skip(native, SkipReason::ConversionFailed, &message, report);
            None
        }
    }
}

fn convert_line(native: &NativeLine) -> Geometry {
    Geometry::Line(Line {
        start: point(native.start),
        end: point(native.end),
    })
}

fn convert_circle(native: &NativeCircle) -> Result<Geometry, String> {
    let plane = plane_from_normal(native.normal)?;
    if !native.radius.is_finite() || native.radius <= 0.0 {
        return Err("circle radius must be positive and finite".to_string());
    }
    Ok(Geometry::Circle(Circle {
        center: ocs_point(native.center, plane),
        radius: native.radius,
        plane,
    }))
}

fn convert_arc(native: &NativeArc) -> Result<Geometry, String> {
    let plane = plane_from_normal(native.normal)?;
    if !native.radius.is_finite() || native.radius <= 0.0 {
        return Err("arc radius must be positive and finite".to_string());
    }
    if !native.start_angle.is_finite() || !native.end_angle.is_finite() {
        return Err("arc angles must be finite".to_string());
    }
    Ok(Geometry::Arc(Arc {
        center: ocs_point(native.center, plane),
        radius: native.radius,
        start_angle: native.start_angle,
        sweep_angle: positive_sweep(native.end_angle - native.start_angle),
        plane,
    }))
}

fn convert_ellipse(native: &NativeEllipse) -> Result<Geometry, String> {
    let major_axis = vector(native.major_axis);
    let major_length = major_axis.length();
    if !major_length.is_finite() || major_length <= GEOMETRY_EPSILON {
        return Err("ellipse major axis has zero length".to_string());
    }
    if !native.minor_axis_ratio.is_finite() || native.minor_axis_ratio <= 0.0 {
        return Err("ellipse minor axis ratio must be positive and finite".to_string());
    }
    if !native.start_parameter.is_finite() || !native.end_parameter.is_finite() {
        return Err("ellipse parameters must be finite".to_string());
    }

    let normal = unit_normal(native.normal)?;
    let major_unit = major_axis * (1.0 / major_length);
    let minor_axis = normal.cross(major_unit) * (major_length * native.minor_axis_ratio);

    Ok(Geometry::Ellipse(Ellipse {
        center: point(native.center),
        major_axis,
        minor_axis,
        start_parameter: native.start_parameter,
        sweep_parameter: positive_sweep(native.end_parameter - native.start_parameter),
    }))
}

fn convert_spline(native: &NativeSpline) -> Result<Geometry, String> {
    if native.degree < 1 {
        return Err("spline degree must be at least 1".to_string());
    }
    if native.control_points.is_empty() && native.fit_points.is_empty() {
        return Err("spline has neither control points nor fit points".to_string());
    }

    Ok(Geometry::Spline(Spline {
        degree: native.degree as u32,
        control_points: native.control_points.iter().copied().map(point).collect(),
        knots: native.knots.clone(),
        weights: native.weights.clone(),
        fit_points: native.fit_points.iter().copied().map(point).collect(),
        closed: native.flags.closed,
        periodic: native.flags.periodic,
        rational: native.flags.rational,
        planar: native.flags.planar,
    }))
}

fn convert_lw_polyline(native: &NativeLwPolyline) -> Result<Geometry, String> {
    let vertices: Vec<(f64, f64, f64)> = native
        .vertices
        .iter()
        .map(|vertex| (vertex.location.x, vertex.location.y, native.elevation))
        .collect();
    let bulges: Vec<f64> = native.vertices.iter().map(|vertex| vertex.bulge).collect();

    Ok(Geometry::Polyline(build_planar_polyline(
        &vertices,
        &bulges,
        native.is_closed,
        native.normal,
    )?))
}

fn convert_polyline2d(native: &NativePolyline2D) -> Result<Geometry, String> {
    if native.flags.bits() & (16 | 64) != 0 {
        return Err("polygon mesh / polyface mesh polylines are not supported".to_string());
    }

    let vertices: Vec<(f64, f64, f64)> = native
        .vertices
        .iter()
        .map(|vertex| {
            let z = if vertex.location.z != 0.0 {
                vertex.location.z
            } else {
                native.elevation
            };
            (vertex.location.x, vertex.location.y, z)
        })
        .collect();
    let bulges: Vec<f64> = native.vertices.iter().map(|vertex| vertex.bulge).collect();

    Ok(Geometry::Polyline(build_planar_polyline(
        &vertices,
        &bulges,
        native.flags.is_closed(),
        native.normal,
    )?))
}

fn convert_polyline_3d_heavy(native: &NativePolyline) -> Result<Geometry, String> {
    if native.flags.bits() & (16 | 64) != 0 {
        return Err("polygon mesh / polyface mesh polylines are not supported".to_string());
    }

    let points: Vec<NativeVector3> = native
        .vertices
        .iter()
        .map(|vertex| vertex.location)
        .collect();
    build_linear_polyline(points, native.flags.is_closed())
}

fn convert_polyline3d(native: &NativePolyline3D) -> Result<Geometry, String> {
    if native.flags.is_3d_mesh || native.flags.is_polyface_mesh {
        return Err("polygon mesh / polyface mesh polylines are not supported".to_string());
    }

    let points: Vec<NativeVector3> = native
        .vertices
        .iter()
        .map(|vertex| vertex.position)
        .collect();
    build_linear_polyline(points, native.flags.closed)
}

fn build_planar_polyline(
    vertices: &[(f64, f64, f64)],
    bulges: &[f64],
    closed: bool,
    normal: NativeVector3,
) -> Result<Polyline, String> {
    if vertices.len() < 2 {
        return Err("polyline has fewer than two vertices".to_string());
    }

    let plane = plane_from_normal(normal)?;
    let mut segments = Vec::new();
    let count = vertices.len();
    let segment_count = if closed { count } else { count - 1 };

    for index in 0..segment_count {
        let (x0, y0, z0) = vertices[index];
        let (x1, y1, z1) = vertices[(index + 1) % count];
        let bulge = bulges.get(index).copied().unwrap_or(0.0);
        let chord = ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt();

        if chord <= GEOMETRY_EPSILON {
            if bulge.abs() <= BULGE_EPSILON {
                continue;
            }
            return Err("polyline contains a zero-length bulge segment".to_string());
        }

        let start = ocs_xy_point(x0, y0, z0, plane);
        let end = ocs_xy_point(x1, y1, z1, plane);

        if bulge.abs() <= BULGE_EPSILON {
            segments.push(PolylineSegment::Line { start, end });
        } else {
            let ((center_x, center_y), radius, start_angle, sweep_angle) =
                bulge_arc_2d((x0, y0), (x1, y1), bulge)
                    .ok_or_else(|| "polyline bulge produces a degenerate arc".to_string())?;
            segments.push(PolylineSegment::Arc {
                start,
                end,
                center: ocs_xy_point(center_x, center_y, z0, plane),
                radius,
                start_angle,
                sweep_angle,
            });
        }
    }

    if segments.is_empty() {
        return Err("polyline has no convertible segments".to_string());
    }

    Ok(Polyline {
        segments,
        closed,
        plane: Some(plane),
    })
}

fn build_linear_polyline(points: Vec<NativeVector3>, closed: bool) -> Result<Geometry, String> {
    if points.len() < 2 {
        return Err("polyline has fewer than two vertices".to_string());
    }

    let mut segments = Vec::new();
    let count = points.len();
    let segment_count = if closed { count } else { count - 1 };

    for index in 0..segment_count {
        let start = point(points[index]);
        let end = point(points[(index + 1) % count]);
        segments.push(PolylineSegment::Line { start, end });
    }

    Ok(Geometry::Polyline(Polyline {
        segments,
        closed,
        plane: None,
    }))
}

/// Convert a DXF bulge (stored on the segment start vertex) into a 2D arc.
///
/// Returns `(center, radius, start_angle, sweep_angle)`; positive sweep means
/// counter-clockwise.
fn bulge_arc_2d(
    start: (f64, f64),
    end: (f64, f64),
    bulge: f64,
) -> Option<((f64, f64), f64, f64, f64)> {
    if !bulge.is_finite() {
        return None;
    }

    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    let chord = (dx * dx + dy * dy).sqrt();
    if chord <= GEOMETRY_EPSILON {
        return None;
    }

    let sweep = 4.0 * bulge.atan();
    let half = sweep / 2.0;
    let tan_half = half.tan();
    if !tan_half.is_finite() {
        return None;
    }

    let offset = chord / (2.0 * tan_half);
    let normal_x = -dy / chord;
    let normal_y = dx / chord;
    let center = (
        (start.0 + end.0) * 0.5 + normal_x * offset,
        (start.1 + end.1) * 0.5 + normal_y * offset,
    );
    let radius = ((start.0 - center.0).powi(2) + (start.1 - center.1).powi(2)).sqrt();
    if !radius.is_finite() || radius <= GEOMETRY_EPSILON {
        return None;
    }

    let start_angle = (start.1 - center.1).atan2(start.0 - center.0);
    Some((center, radius, start_angle, sweep))
}

fn positive_sweep(delta: f64) -> f64 {
    let wrapped = delta.rem_euclid(TAU);
    if wrapped.abs() < GEOMETRY_EPSILON && delta.abs() >= TAU - GEOMETRY_EPSILON {
        TAU
    } else {
        wrapped
    }
}

fn unit_normal(normal: NativeVector3) -> Result<Vector3, String> {
    let vector = vector(normal);
    let length = vector.length();
    if !length.is_finite() || length <= GEOMETRY_EPSILON {
        return Err("invalid extrusion/normal vector".to_string());
    }
    Ok(vector * (1.0 / length))
}

fn plane_from_normal(normal: NativeVector3) -> Result<Plane, String> {
    let normal = unit_normal(normal)?;
    let x_axis = if normal.x.abs() < 1.0 / 64.0 && normal.y.abs() < 1.0 / 64.0 {
        Vector3::new(0.0, 1.0, 0.0).cross(normal)
    } else {
        Vector3::new(0.0, 0.0, 1.0).cross(normal)
    }
    .normalize();

    if x_axis.length() <= GEOMETRY_EPSILON {
        return Err("failed to derive an OCS x-axis from the extrusion vector".to_string());
    }

    Ok(Plane::new(normal, x_axis))
}

fn ocs_point(point: NativeVector3, plane: Plane) -> Point3 {
    ocs_xy_point(point.x, point.y, point.z, plane)
}

fn ocs_xy_point(x: f64, y: f64, z: f64, plane: Plane) -> Point3 {
    Point3::new(
        plane.x_axis.x * x + plane.y_axis().x * y + plane.normal.x * z,
        plane.x_axis.y * x + plane.y_axis().y * y + plane.normal.y * z,
        plane.x_axis.z * x + plane.y_axis().z * y + plane.normal.z * z,
    )
}

fn point(vector: NativeVector3) -> Point3 {
    Point3::new(vector.x, vector.y, vector.z)
}

fn vector(vector: NativeVector3) -> Vector3 {
    Vector3::new(vector.x, vector.y, vector.z)
}

fn convert_color(color: NativeColor) -> Color {
    match color {
        NativeColor::ByLayer => Color::ByLayer,
        NativeColor::ByBlock => Color::ByBlock,
        NativeColor::None => Color::None,
        NativeColor::Index(index) => Color::Aci(index),
        NativeColor::Rgb { r, g, b } => Color::Rgb { r, g, b },
    }
}

fn convert_units(insunits: i16, report: &mut ImportReport) -> Option<LengthUnit> {
    let unit = match insunits {
        1 => LengthUnit::Inches,
        2 => LengthUnit::Feet,
        3 => LengthUnit::Miles,
        4 => LengthUnit::Millimeters,
        5 => LengthUnit::Centimeters,
        6 => LengthUnit::Meters,
        7 => LengthUnit::Kilometers,
        8 => LengthUnit::Microinches,
        9 => LengthUnit::Mils,
        10 => LengthUnit::Yards,
        11 => LengthUnit::Angstroms,
        12 => LengthUnit::Nanometers,
        13 => LengthUnit::Microns,
        14 => LengthUnit::Decimeters,
        15 => LengthUnit::Decameters,
        16 => LengthUnit::Hectometers,
        17 => LengthUnit::Gigameters,
        18 => LengthUnit::AstronomicalUnits,
        19 => LengthUnit::LightYears,
        20 => LengthUnit::Parsecs,
        21 => LengthUnit::UsSurveyFeet,
        22 => LengthUnit::UsSurveyInch,
        23 => LengthUnit::UsSurveyYard,
        24 => LengthUnit::UsSurveyMile,
        0 => {
            report.warn(
                "drawing units are unspecified (INSUNITS=0); downstream CAM must choose units explicitly",
            );
            return None;
        }
        _ => {
            report.warn(format!(
                "unknown INSUNITS value {insunits}; treating drawing units as unspecified"
            ));
            return None;
        }
    };
    Some(unit)
}

fn ensure_layer(
    layers: &mut Vec<Layer>,
    layer_ids: &mut HashMap<String, LayerId>,
    layer_identities: &mut BTreeMap<String, LayerId>,
    name: &str,
    report: &mut ImportReport,
) -> LayerId {
    let key = layer_key(name);
    if let Some(id) = layer_ids.get(&key) {
        return *id;
    }

    let next_id = LayerId(
        layer_identities
            .values()
            .map(|id| id.0)
            .max()
            .map_or(0, |id| id + 1),
    );
    let id = *layer_identities
        .entry(format!("name:{key}"))
        .or_insert(next_id);
    layers.push(Layer {
        id,
        name: name.to_string(),
        color: Color::None,
        visible: true,
        locked: false,
    });
    layer_ids.insert(key, id);
    report.warn(format!(
        "layer '{name}' is referenced by an entity but missing from the layer table; \
         added an implicit layer"
    ));
    id
}

fn entity_layer_name(entity: &EntityType) -> &str {
    let layer = entity.as_entity().layer();
    if layer.trim().is_empty() {
        "0"
    } else {
        layer
    }
}

fn layer_key(name: &str) -> String {
    name.to_lowercase()
}

fn record_skip(native: &EntityType, reason: SkipReason, message: &str, report: &mut ImportReport) {
    let layer = entity_layer_name(native).to_string();
    let handle = native.as_entity().handle();
    let handle = if handle.is_valid() {
        Some(handle.value())
    } else {
        None
    };
    report.record_skipped(
        source_type(native),
        layer,
        handle,
        reason,
        message.to_string(),
    );
}

fn source_type(native: &EntityType) -> String {
    match native {
        EntityType::Unknown(unknown) => unknown.dxf_name.clone(),
        _ => native.as_entity().entity_type().to_string(),
    }
}

/// Append structured parser diagnostics as human-readable import warnings.
fn append_stats_warnings(warnings: &mut Vec<String>, stats: &ReadStats) {
    for diagnostic in &stats.diagnostics {
        let mut message = format!(
            "[{}] {}: {}",
            diagnostic.stage, diagnostic.code, diagnostic.message
        );
        if let Some(section) = &diagnostic.section {
            message.push_str(&format!(" (section: {section})"));
        }
        if let Some(record_type) = &diagnostic.record_type {
            message.push_str(&format!(" (record type: {record_type})"));
        }
        warnings.push(message);
    }
}

/// Append non-fatal parser notifications (encoding/format recovery notes).
fn append_notification_warnings(warnings: &mut Vec<String>, document: &CadDocument) {
    for notification in document.notifications.iter() {
        warnings.push(notification.to_string());
    }
}

/// Remove duplicate warnings while preserving their first-seen order.
fn deduplicate(warnings: &mut Vec<String>) {
    let mut seen = HashSet::new();
    warnings.retain(|warning| seen.insert(warning.clone()));
}
