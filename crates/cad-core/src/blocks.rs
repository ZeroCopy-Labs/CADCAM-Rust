//! Materialize supported INSERTs in native coordinates before unit conversion.
use crate::{
    convert::convert_geometry,
    model::{BlockExpansionReport, BlockMapping, ImportReport, InsertSource},
};
use acadrust::{
    entities::{Arc, Circle, Ellipse, Insert},
    types::{Matrix3, Transform, Vector3},
    CadDocument, EntityType, Handle,
};
use std::f64::consts::TAU;
const LIMIT: usize = 100_000;
struct Leaf {
    entity: EntityType,
    path: Vec<InsertSource>,
    source: u64,
    segment: usize,
    circular: bool,
}
pub(crate) fn materialize(doc: &mut CadDocument) -> BlockExpansionReport {
    let mut report = BlockExpansionReport {
        original_entities: doc.entities().count(),
        original_model_space: doc.model_space_entities().count(),
        ..Default::default()
    };
    let inserts: Vec<_> = doc
        .model_space_entities()
        .filter_map(|e| {
            if let EntityType::Insert(i) = e {
                Some(i.clone())
            } else {
                None
            }
        })
        .collect();
    for insert in inserts {
        if report.entities >= LIMIT {
            report.failures_omitted += 1;
            continue;
        }
        let mut work = 0;
        let result = expand(doc, &insert, None, &mut Vec::new(), &mut work).and_then(|leaves| {
            if report.entities + leaves.len() > LIMIT {
                return Err("整图块展开超过 100000 个实体预算".into());
            }
            let mut candidate = doc.clone();
            let mut mappings = Vec::new();
            for mut leaf in leaves {
                let mut probe = ImportReport::default();
                convert_geometry(&leaf.entity, &mut probe).ok_or_else(|| {
                    format!(
                        "变换后图元无法转换：{:?}",
                        probe.skipped.first().map(|s| &s.message)
                    )
                })?;
                let common = leaf.entity.common_mut();
                common.handle = Handle::NULL;
                common.owner_handle = Handle::NULL;
                common.raw_record = None;
                common.layer_handle = None;
                let handle = candidate
                    .add_entity(leaf.entity)
                    .map_err(|e| e.to_string())?;
                mappings.push(BlockMapping {
                    handle: handle.value(),
                    path: leaf.path,
                    source_handle: leaf.source,
                    segment: leaf.segment,
                });
            }
            candidate.remove_entity(insert.common.handle);
            *doc = candidate;
            Ok(mappings)
        });
        match result {
            Ok(mappings) => {
                report.instances += 1;
                report.entities += mappings.len();
                for map in mappings {
                    if report.mappings.len() < 2048 {
                        report.mappings.push(map);
                    } else {
                        report.mappings_omitted += 1;
                    }
                }
            }
            Err(error) => {
                if report.failures.len() < 256 {
                    report.failures.push(format!(
                        "INSERT {:X}（{}）：{error}；源实例保留，未部分展开。",
                        insert.common.handle.value(),
                        insert.block_name
                    ));
                } else {
                    report.failures_omitted += 1;
                }
            }
        }
    }
    report
}
fn expand(
    doc: &CadDocument,
    original: &Insert,
    parent: Option<&acadrust::entities::EntityCommon>,
    stack: &mut Vec<String>,
    work: &mut usize,
) -> Result<Vec<Leaf>, String> {
    let key = original.block_name.to_lowercase();
    if stack.len() >= 24 || stack.contains(&key) {
        return Err("块引用递归/循环或深度超过 24".into());
    }
    if !original.attributes.is_empty() {
        return Err("含实例属性的块暂不展开".into());
    }
    let mut insert = original.clone();
    if insert.common.layer.trim().is_empty() {
        insert.common.layer = "0".into();
    }
    if let Some(parent) = parent {
        if insert.common.layer == "0" {
            insert.common.layer = parent.layer.clone();
        }
        if insert.common.color == acadrust::Color::ByBlock {
            insert.common.color = parent.color;
        }
        if insert.common.line_weight == acadrust::types::LineWeight::ByBlock {
            insert.common.line_weight = parent.line_weight;
        }
        if insert.common.linetype.eq_ignore_ascii_case("byblock") {
            insert.common.linetype = parent.linetype.clone();
            insert.common.linetype_handle = None;
        }
        if insert.common.transparency.is_by_block() {
            insert.common.transparency = parent.transparency;
        }
        insert.common.invisible |= parent.invisible;
    }
    let layer = doc.layers.get(&insert.common.layer);
    if insert.common.color == acadrust::Color::ByBlock {
        insert.common.color = acadrust::Color::Index(7);
    }
    if insert.common.line_weight == acadrust::types::LineWeight::ByLayer {
        if let Some(layer) = layer {
            insert.common.line_weight = layer.line_weight;
        }
    }
    if insert.common.linetype.eq_ignore_ascii_case("bylayer") {
        if let Some(layer) = layer {
            insert.common.linetype = layer.line_type.clone();
            insert.common.linetype_handle = None;
        }
    }
    if insert.common.transparency.is_by_layer() {
        if let Some(layer) = layer {
            insert.common.transparency = layer.transparency;
        }
    }
    if insert.common.color == acadrust::Color::ByLayer {
        insert.common.color = layer.map(|l| l.color).unwrap_or(acadrust::Color::Index(7));
    }
    insert.common.invisible |= layer.is_some_and(|l| l.flags.off || l.flags.frozen);
    let values = [
        insert.x_scale(),
        insert.y_scale(),
        insert.z_scale(),
        insert.rotation,
        insert.insert_point.x,
        insert.insert_point.y,
        insert.insert_point.z,
        insert.column_spacing,
        insert.row_spacing,
        insert.normal.length(),
    ];
    if values.iter().any(|v| !v.is_finite())
        || insert.normal.length() < 1e-12
        || [insert.x_scale(), insert.y_scale(), insert.z_scale()]
            .iter()
            .any(|v| v.abs() < 1e-12)
        || insert.row_count == 0
        || insert.column_count == 0
    {
        return Err("块的法向、缩放或阵列参数无效".into());
    }
    let block = doc
        .block_records
        .get(&insert.block_name)
        .ok_or("找不到块定义")?;
    if !block.explodable
        || block.flags.is_external
        || block.flags.is_xref
        || block.flags.is_xref_overlay
    {
        return Err("外部参照或禁止分解的块暂不展开".into());
    }
    let base = doc
        .get_entity(block.block_entity_handle)
        .and_then(|e| {
            if let EntityType::Block(b) = e {
                Some(b.base_point)
            } else {
                None
            }
        })
        .unwrap_or(block.base_point);
    stack.push(key);
    let mut children = Vec::new();
    for handle in &block.entity_handles {
        *work += 1;
        if *work > LIMIT {
            return Err("块遍历超过预算".into());
        }
        let entity = doc.get_entity(*handle).ok_or("块定义含失效实体引用")?;
        match entity {
            EntityType::Block(_) | EntityType::BlockEnd(_) => continue,
            EntityType::Insert(nested) => {
                children.extend(expand(doc, nested, Some(&insert.common), stack, work)?)
            }
            _ => {
                let thick = match entity {
                    EntityType::Line(e) => e.thickness != 0.0,
                    EntityType::Circle(e) => e.thickness != 0.0,
                    EntityType::Arc(e) => e.thickness != 0.0,
                    _ => false,
                };
                if thick {
                    return Err("带挤出厚度的块内图元暂不展开".into());
                }
                let vertices = match entity {
                    EntityType::LwPolyline(p) => p.vertices.len(),
                    EntityType::Polyline2D(p) => p.vertices.len(),
                    EntityType::Polyline3D(p) => p.vertices.len(),
                    EntityType::Polyline(p) => p.vertices.len(),
                    _ => 0,
                };
                if vertices > LIMIT {
                    return Err("块内多段线顶点超过展开预算".into());
                }
                let mut probe = ImportReport::default();
                let geometry = convert_geometry(entity, &mut probe).ok_or_else(|| {
                    format!(
                        "块内类型 {} 暂不支持可靠展开",
                        entity.as_entity().entity_type()
                    )
                })?;
                match geometry {
                    crate::model::Geometry::Polyline(p) => {
                        let wide = match entity {
                            EntityType::LwPolyline(p) => {
                                p.constant_width != 0.0
                                    || p.thickness != 0.0
                                    || p.vertices
                                        .iter()
                                        .any(|v| v.start_width != 0.0 || v.end_width != 0.0)
                            }
                            EntityType::Polyline2D(p) => {
                                p.start_width != 0.0
                                    || p.end_width != 0.0
                                    || p.thickness != 0.0
                                    || p.flags.is_spline_fit()
                            }
                            EntityType::Polyline3D(p) => {
                                p.flags.spline_fit
                                    || p.default_start_width != 0.0
                                    || p.default_end_width != 0.0
                            }
                            _ => false,
                        };
                        if wide {
                            return Err("带宽度/厚度/拟合标记的块内多段线暂不展开".into());
                        }
                        for (segment, s) in p.segments.iter().enumerate() {
                            let g = crate::curves::segment_geometry(s, p.plane)?;
                            let circular = matches!(g, crate::model::Geometry::Arc(_));
                            let mut e = crate::geometry_edit::native_geometry(&g, 1.0)?;
                            *e.common_mut() = entity.common().clone();
                            children.push(Leaf {
                                entity: e,
                                path: Vec::new(),
                                source: handle.value(),
                                segment,
                                circular,
                            });
                        }
                    }
                    _ => children.push(Leaf {
                        entity: entity.clone(),
                        path: Vec::new(),
                        source: handle.value(),
                        segment: 0,
                        circular: matches!(entity, EntityType::Circle(_) | EntityType::Arc(_)),
                    }),
                }
            }
        }
    }
    stack.pop();
    let cells = insert.row_count as usize * insert.column_count as usize;
    if children.len().saturating_mul(cells) > LIMIT
        || *work + children.len().saturating_mul(cells) > LIMIT
    {
        return Err("块阵列展开超过预算".into());
    }
    let mut out = Vec::new();
    for mut leaf in children {
        // Convert circular primitives first: non-uniform transforms of tilted arcs must use a conic.
        leaf.entity = as_conic(leaf.entity);
        leaf.entity
            .apply_transform(&Transform::from_translation(base * -1.0));
        if leaf.entity.common().layer.trim().is_empty() {
            leaf.entity.common_mut().layer = "0".into();
        }
        let by_block = leaf.entity.common().color == acadrust::Color::ByBlock;
        for (cell, mut entity) in insert.explode(&[leaf.entity]).into_iter().enumerate() {
            *work += 1;
            entity.common_mut().invisible |= insert.common.invisible;
            if by_block {
                entity.common_mut().color_name = None;
                entity.common_mut().color_book_handle = None;
            }
            if leaf.circular {
                entity = restore_circular(entity);
            }
            let mut path = vec![InsertSource {
                handle: original.common.handle.value(),
                row: (cell / insert.column_count as usize) as u16,
                column: (cell % insert.column_count as usize) as u16,
            }];
            path.extend(leaf.path.clone());
            out.push(Leaf {
                entity,
                path,
                source: leaf.source,
                segment: leaf.segment,
                circular: leaf.circular,
            });
        }
    }
    Ok(out)
}
fn as_conic(entity: EntityType) -> EntityType {
    match entity {
        EntityType::Circle(c) => {
            let basis = Matrix3::arbitrary_axis(c.normal);
            EntityType::Ellipse(Ellipse {
                common: c.common,
                center: basis * c.center,
                major_axis: basis * (Vector3::UNIT_X * c.radius),
                minor_axis_ratio: 1.0,
                start_parameter: 0.0,
                end_parameter: TAU,
                normal: c.normal,
            })
        }
        EntityType::Arc(a) => {
            let basis = Matrix3::arbitrary_axis(a.normal);
            EntityType::Ellipse(Ellipse {
                common: a.common,
                center: basis * a.center,
                major_axis: basis * (Vector3::UNIT_X * a.radius),
                minor_axis_ratio: 1.0,
                start_parameter: a.start_angle,
                end_parameter: a.end_angle,
                normal: a.normal,
            })
        }
        e => e,
    }
}
fn restore_circular(entity: EntityType) -> EntityType {
    let EntityType::Ellipse(e) = entity else {
        return entity;
    };
    if (e.minor_axis_ratio - 1.0).abs() > 1e-10 {
        return EntityType::Ellipse(e);
    }
    let basis = Matrix3::arbitrary_axis(e.normal).transpose();
    let radius = e.major_axis.length();
    if e.is_full() {
        let mut c = Circle::new();
        c.common = e.common;
        c.center = basis * e.center;
        c.normal = e.normal;
        c.radius = radius;
        return EntityType::Circle(c);
    }
    let minor = e.normal.normalize().cross(&e.major_axis) * e.minor_axis_ratio;
    let d = basis * (e.major_axis * e.start_parameter.cos() + minor * e.start_parameter.sin());
    let start = d.y.atan2(d.x);
    let mut a = Arc::new();
    a.common = e.common;
    a.center = basis * e.center;
    a.radius = radius;
    a.normal = e.normal;
    a.start_angle = start;
    a.end_angle = start + (e.end_parameter - e.start_parameter).rem_euclid(TAU);
    EntityType::Arc(a)
}
