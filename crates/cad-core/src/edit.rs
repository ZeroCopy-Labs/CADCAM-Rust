//! Batch editing with stable model identities and native DXF record retention.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::path::Path;

use acadrust::{CadDocument, Color as NativeColor, DxfWriter, Handle};

use crate::convert::convert_document;
use crate::geometry_edit::{native_geometry, AffineTransform, NewEntity};
use crate::model::{Color, DrawingDocument, EntityId, FileFormat, LayerId, LengthUnit};

const HISTORY_LIMIT: usize = 100;

/// One previewed structural edit; primary replacements retain their IDs.
#[derive(Clone, Debug, Default)]
pub struct CurveEdit {
    pub replacements: BTreeMap<EntityId, Vec<crate::model::Geometry>>,
    pub remove: BTreeSet<EntityId>,
    pub copies: bool,
}

#[derive(Clone, Debug)]
pub enum LayerEdit {
    Create {
        name: String,
        color: Color,
    },
    Rename {
        id: LayerId,
        name: String,
    },
    Delete {
        id: LayerId,
        migrate: Option<LayerId>,
    },
    Color {
        id: LayerId,
        color: Color,
    },
    Visible {
        id: LayerId,
        visible: bool,
    },
    Locked {
        id: LayerId,
        locked: bool,
    },
    Current(LayerId),
}

/// One committed batch, shared by apply, undo and redo notifications.
#[derive(Clone, Copy, Debug)]
pub enum DrawingChangeKind {
    Color,
    Layer,
    Move,
    Delete,
    Create,
    Transform,
    Geometry,
    Layers,
}
impl DrawingChangeKind {
    pub fn changes_geometry(self) -> bool {
        !matches!(self, Self::Color | Self::Layer | Self::Layers)
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Color => "修改颜色",
            Self::Layer => "修改所属图层",
            Self::Move => "三维移动",
            Self::Delete => "删除",
            Self::Create => "创建",
            Self::Transform => "批量变换/复制",
            Self::Geometry => "修改几何属性",
            Self::Layers => "修改图层",
        }
    }
}
#[derive(Clone, Debug)]
pub struct DrawingChange {
    /// Unique within this loaded drawing, including branches after undo.
    pub transaction_id: u64,
    /// Number of source entities affected by this batch.
    pub affected: usize,
    /// Original operation, also returned when this transaction is undone/redone.
    pub kind: DrawingChangeKind,
    pub created: Vec<EntityId>,
}

struct HistoryEntry {
    native: CadDocument,
    revision: u64,
    change: DrawingChange,
}

#[derive(Clone, Copy, Debug)]
pub enum DrawingEdit {
    SetColor(Color),
    SetLayer(LayerId),
    Translate([f64; 3]),
    Delete,
}

impl DrawingEdit {
    pub fn changes_geometry(self) -> bool {
        matches!(self, Self::Translate(_) | Self::Delete)
    }
}

pub struct EditableDrawing {
    document: DrawingDocument,
    native: CadDocument,
    ids: BTreeMap<Handle, EntityId>,
    layer_ids: BTreeMap<String, LayerId>,
    history: VecDeque<HistoryEntry>,
    redo: Vec<HistoryEntry>,
    serial: u64,
    revision: u64,
    saved_revision: u64,
}

impl EditableDrawing {
    /// An empty millimeter drawing; source records are created by the parser library.
    pub fn blank() -> Self {
        let mut native = CadDocument::new();
        native.header.insertion_units = 4;
        let mut ids = BTreeMap::new();
        let mut layer_ids = BTreeMap::new();
        let mut document = convert_document(
            Path::new("未命名.dxf"),
            FileFormat::AsciiDxf,
            &native,
            &mut ids,
            &mut layer_ids,
        );
        document.source.file_path = None;
        Self::new(document, native, ids, layer_ids)
    }

    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Assign the interpretation of a unitless source before it enters CAD/CAM.
    pub fn resolve_units(&mut self, unit: LengthUnit) -> Result<(), String> {
        if self.document.source_units.is_some() {
            return Err("图纸已有单位，不能重复解释".into());
        }
        self.native.header.insertion_units = match unit {
            LengthUnit::Millimeters => 4,
            LengthUnit::Inches => 1,
            _ => return Err("无单位图纸请选择毫米或英寸".into()),
        };
        self.serial += 1;
        self.revision = self.serial;
        self.refresh_document();
        self.document.import_report.warnings.push(format!(
            "无单位图纸由用户指定为 {unit}；工作区单位为 mm，保存 DXF 后保留该声明"
        ));
        Ok(())
    }

    pub fn mark_recovered(&mut self) {
        self.document.source.file_path = None;
        let name = Path::new(&self.document.source.file_name);
        if name
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("bak"))
        {
            self.document.source.file_name =
                name.file_stem().unwrap().to_string_lossy().into_owned();
        }
        self.serial += 1;
        self.revision = self.serial;
    }

    pub(crate) fn new(
        document: DrawingDocument,
        native: CadDocument,
        ids: BTreeMap<Handle, EntityId>,
        layer_ids: BTreeMap<String, LayerId>,
    ) -> Self {
        Self {
            document,
            native,
            ids,
            layer_ids,
            history: VecDeque::new(),
            redo: Vec::new(),
            serial: 0,
            revision: 0,
            saved_revision: 0,
        }
    }

    pub fn document(&self) -> &DrawingDocument {
        &self.document
    }

    pub fn into_document(self) -> DrawingDocument {
        self.document
    }

    pub fn can_undo(&self) -> bool {
        !self.history.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }

    /// IDs whose application-side selection/CAM snapshots are still needed.
    pub fn history_ids(&self) -> impl Iterator<Item = u64> + '_ {
        self.history
            .iter()
            .chain(&self.redo)
            .map(|entry| entry.change.transaction_id)
    }

    pub fn is_modified(&self) -> bool {
        self.revision != self.saved_revision
    }

    pub fn apply(
        &mut self,
        selected: &BTreeSet<EntityId>,
        edit: DrawingEdit,
    ) -> Result<Option<DrawingChange>, String> {
        if selected.is_empty() {
            return Ok(None);
        }
        self.ensure_editable(selected)?;
        let layer = match edit {
            DrawingEdit::SetLayer(id) => {
                self.ensure_layer_editable(id)?;
                Some(
                    self.document
                        .layer(id)
                        .ok_or("目标图层不存在")?
                        .name
                        .clone(),
                )
            }
            _ => None,
        };
        if let DrawingEdit::Translate(offset) = edit {
            if !offset.iter().all(|value| value.is_finite()) {
                return Err("移动量必须是有限的三维坐标".to_string());
            }
            if offset == [0.0; 3] {
                return Ok(None);
            }
        }
        let entities: BTreeMap<_, _> = self
            .document
            .entities
            .iter()
            .map(|entity| (entity.id, entity))
            .collect();
        let native_handles: BTreeMap<_, _> =
            self.ids.iter().map(|(handle, id)| (*id, *handle)).collect();
        let mut handles = Vec::new();
        for id in selected {
            let entity = entities.get(id).ok_or("选择中存在已失效的图元")?;
            let unchanged = match edit {
                DrawingEdit::SetColor(color) => entity.color == color,
                DrawingEdit::SetLayer(layer) => entity.layer_id == layer,
                _ => false,
            };
            if unchanged {
                continue;
            }
            let handle = *native_handles.get(id).ok_or("图元缺少源文档记录")?;
            if self.native.get_entity(handle).is_none() {
                return Err("源图元记录已失效".to_string());
            }
            handles.push(handle);
        }
        if handles.is_empty() {
            return Ok(None);
        }
        // Mutate and convert a candidate before replacing the authoritative document.
        let mut candidate = self.native.clone();
        for handle in &handles {
            if matches!(edit, DrawingEdit::Delete) {
                candidate.remove_entity(*handle);
                continue;
            }
            let entity = candidate
                .get_entity_mut(*handle)
                .expect("validated native entity");
            match edit {
                DrawingEdit::SetColor(color) => {
                    let common = entity.common_mut();
                    common.color = match color {
                        Color::ByLayer => NativeColor::ByLayer,
                        Color::ByBlock => NativeColor::ByBlock,
                        Color::None => NativeColor::None,
                        Color::Aci(index) => NativeColor::Index(index),
                        Color::Rgb { r, g, b } => NativeColor::Rgb { r, g, b },
                    };
                    common.color_name = None;
                    common.color_book_handle = None;
                }
                DrawingEdit::SetLayer(_) => {
                    let common = entity.common_mut();
                    common.layer = layer.as_ref().unwrap().clone();
                    common.layer_handle = None;
                }
                DrawingEdit::Translate([x, y, z]) => {
                    let scale = self
                        .document
                        .source_units
                        .ok_or("请先明确图纸单位")?
                        .millimeters_per_unit();
                    let offset = acadrust::types::Vector3::new(x / scale, y / scale, z / scale);
                    if let acadrust::EntityType::Polyline2D(polyline) = entity {
                        // Heavy 2D polylines store their depth in the header elevation.
                        let offset = acadrust::types::Matrix3::arbitrary_axis(polyline.normal)
                            .transpose()
                            * offset;
                        let elevation = polyline
                            .vertices
                            .first()
                            .map(|vertex| vertex.location.z)
                            .filter(|value| *value != 0.0)
                            .unwrap_or(polyline.elevation);
                        polyline.elevation = elevation + offset.z;
                        for vertex in &mut polyline.vertices {
                            vertex.location.x += offset.x;
                            vertex.location.y += offset.y;
                            vertex.location.z = 0.0;
                        }
                    } else {
                        entity.translate(offset);
                    }
                }
                DrawingEdit::Delete => unreachable!(),
            }
        }
        let kind = match edit {
            DrawingEdit::SetColor(_) => DrawingChangeKind::Color,
            DrawingEdit::SetLayer(_) => DrawingChangeKind::Layer,
            DrawingEdit::Translate(_) => DrawingChangeKind::Move,
            DrawingEdit::Delete => DrawingChangeKind::Delete,
        };
        self.commit(
            candidate,
            self.ids.clone(),
            kind,
            handles.len(),
            if matches!(edit, DrawingEdit::Delete) {
                selected.clone()
            } else {
                BTreeSet::new()
            },
            Vec::new(),
        )
    }

    pub fn create(&mut self, entities: &[NewEntity]) -> Result<Option<DrawingChange>, String> {
        if entities.is_empty() {
            return Ok(None);
        }
        if entities.len() > 50_000 {
            return Err("单次创建不能超过 50000 个图元".into());
        }
        let scale = self.unit_scale()?;
        let mut candidate = self.native.clone();
        let mut ids = self.ids.clone();
        let mut created = Vec::new();
        let mut next_id = Self::next_id(&ids)?;
        for item in entities {
            self.ensure_layer_editable(item.layer)?;
            let layer = self
                .document
                .layer(item.layer)
                .ok_or("创建目标图层不存在")?;
            let mut entity = native_geometry(&item.geometry, scale)?;
            if let (acadrust::EntityType::Spline(spline), Some(n)) = (&mut entity, item.normal) {
                spline.normal = acadrust::types::Vector3::new(n.x, n.y, n.z);
            }
            entity.common_mut().layer = layer.name.clone();
            entity.common_mut().color = match item.color {
                Color::ByLayer => NativeColor::ByLayer,
                Color::ByBlock => NativeColor::ByBlock,
                Color::None => NativeColor::None,
                Color::Aci(i) => NativeColor::Index(i),
                Color::Rgb { r, g, b } => NativeColor::Rgb { r, g, b },
            };
            if let acadrust::EntityType::Polyline3D(p) = &mut entity {
                for vertex in &mut p.vertices {
                    vertex.handle = candidate.allocate_handle();
                }
            }
            let handle = candidate.add_entity(entity).map_err(|e| e.to_string())?;
            let id = Self::fresh_id(&mut ids, handle, &mut next_id)?;
            created.push(id);
        }
        self.commit(
            candidate,
            ids,
            DrawingChangeKind::Create,
            entities.len(),
            BTreeSet::new(),
            created,
        )
    }

    pub fn transform(
        &mut self,
        selected: &BTreeSet<EntityId>,
        transforms: &[AffineTransform],
        copies: bool,
    ) -> Result<Option<DrawingChange>, String> {
        if selected.is_empty() || transforms.is_empty() {
            return Ok(None);
        }
        self.ensure_editable(selected)?;
        if !copies && transforms.len() != 1 {
            return Err("原位变换只接受一个变换".into());
        }
        if selected
            .len()
            .checked_mul(transforms.len())
            .is_none_or(|n| n > 50_000)
        {
            return Err("单次变换最多影响 50000 个图元".into());
        }
        if !copies && transforms[0].identity() {
            return Ok(None);
        }
        let scale = self.unit_scale()?;
        let handles: BTreeMap<_, _> = self.ids.iter().map(|(h, id)| (*id, *h)).collect();
        let mut sources = Vec::new();
        for id in selected {
            self.document.entity(*id).ok_or("选择中存在失效图元")?;
            let handle = *handles.get(id).ok_or("图元缺少源记录")?;
            sources.push((
                handle,
                self.native
                    .get_entity(handle)
                    .ok_or("源记录已失效")?
                    .clone(),
            ));
        }
        let mut candidate = self.native.clone();
        let mut ids = self.ids.clone();
        let mut created = Vec::new();
        let mut next_id = Self::next_id(&ids)?;
        for transform in transforms {
            for (handle, source) in &sources {
                let mut entity = source.clone();
                Self::transform_native(&mut entity, *transform, scale);
                if copies {
                    let common = entity.common_mut();
                    common.handle = Handle::NULL;
                    common.owner_handle = Handle::NULL;
                    common.raw_record = None;
                    if let acadrust::EntityType::Polyline3D(p) = &mut entity {
                        for vertex in &mut p.vertices {
                            vertex.handle = candidate.allocate_handle();
                        }
                    }
                    let handle = candidate.add_entity(entity).map_err(|e| e.to_string())?;
                    created.push(Self::fresh_id(&mut ids, handle, &mut next_id)?);
                } else {
                    *candidate.get_entity_mut(*handle).ok_or("源记录失效")? = entity;
                }
            }
        }
        self.commit(
            candidate,
            ids,
            DrawingChangeKind::Transform,
            selected.len() * transforms.len(),
            BTreeSet::new(),
            created,
        )
    }

    fn ensure_layer_editable(&self, id: LayerId) -> Result<(), String> {
        let layer = self.document.layer(id).ok_or("图层不存在")?;
        if layer.locked || !layer.visible {
            return Err(format!("图层“{}”已锁定或隐藏，请先解锁并显示", layer.name));
        }
        Ok(())
    }

    fn ensure_editable(&self, ids: &BTreeSet<EntityId>) -> Result<(), String> {
        for id in ids {
            let entity = self.document.entity(*id).ok_or("选择中存在失效图元")?;
            self.ensure_layer_editable(entity.layer_id)?;
        }
        Ok(())
    }

    pub fn apply_curves(&mut self, edit: &CurveEdit) -> Result<Option<DrawingChange>, String> {
        if edit.replacements.is_empty() && edit.remove.is_empty() {
            return Ok(None);
        }
        if edit.copies && !edit.remove.is_empty() {
            return Err("创建副本不能同时删除源对象".into());
        }
        let selected: BTreeSet<_> = edit
            .replacements
            .keys()
            .copied()
            .chain(edit.remove.iter().copied())
            .collect();
        self.ensure_editable(&selected)?;
        if edit
            .remove
            .iter()
            .any(|id| edit.replacements.contains_key(id))
        {
            return Err("替换与删除对象重复".into());
        }
        let output: usize = edit.replacements.values().map(Vec::len).sum();
        if output > 50_000 {
            return Err("单次曲线整理结果不能超过 50000 个图元".into());
        }
        let scale = self.unit_scale()?;
        let mut candidate = self.native.clone();
        let mut ids = self.ids.clone();
        let mut next = Self::next_id(&ids)?;
        let mut created = Vec::new();
        let handles: BTreeMap<_, _> = ids.iter().map(|(h, id)| (*id, *h)).collect();
        for id in &selected {
            let source = self
                .native
                .get_entity(*handles.get(id).ok_or("源记录失效")?)
                .ok_or("源记录失效")?;
            let unsupported = match source {
                acadrust::EntityType::Line(e) => e.thickness != 0.0,
                acadrust::EntityType::Circle(e) => e.thickness != 0.0,
                acadrust::EntityType::Arc(e) => e.thickness != 0.0,
                acadrust::EntityType::LwPolyline(p) => {
                    p.constant_width != 0.0
                        || p.thickness != 0.0
                        || p.vertices
                            .iter()
                            .any(|v| v.start_width != 0.0 || v.end_width != 0.0)
                }
                acadrust::EntityType::Polyline2D(p) => {
                    p.start_width != 0.0
                        || p.end_width != 0.0
                        || p.thickness != 0.0
                        || p.flags.is_spline_fit()
                        || p.vertices
                            .iter()
                            .any(|v| v.start_width != 0.0 || v.end_width != 0.0)
                }
                acadrust::EntityType::Polyline3D(p) => {
                    p.flags.spline_fit || p.default_start_width != 0.0 || p.default_end_width != 0.0
                }
                _ => false,
            };
            if unsupported {
                return Err("带宽度、厚度或拟合标记的图元暂不支持结构整理，未丢弃附加属性".into());
            }
        }
        let mut removed = edit.remove.clone();
        for id in &edit.remove {
            candidate.remove_entity(*handles.get(id).ok_or("源记录失效")?);
        }
        for (id, results) in &edit.replacements {
            let handle = *handles.get(id).ok_or("源记录失效")?;
            let source = self.native.get_entity(handle).ok_or("源记录失效")?;
            if results.is_empty() {
                if !edit.copies {
                    candidate.remove_entity(handle);
                    removed.insert(*id);
                }
                continue;
            }
            for (index, geometry) in results.iter().enumerate() {
                // Sampling validates each result before any authoritative state is changed.
                crate::curves::sample(geometry, crate::curves::Sampling::default())?;
                let mut native = source.clone();
                if crate::properties::update_native(&mut native, geometry, scale).is_err() {
                    native = native_geometry(geometry, scale)?;
                    *native.common_mut() = source.common().clone();
                }
                native.common_mut().raw_record = None;
                if index == 0 && !edit.copies {
                    if self
                        .document
                        .entity(*id)
                        .is_some_and(|e| e.geometry == *geometry)
                    {
                        continue;
                    }
                    *candidate.get_entity_mut(handle).ok_or("源记录失效")? = native;
                } else {
                    let common = native.common_mut();
                    common.handle = Handle::NULL;
                    common.owner_handle = Handle::NULL;
                    if let acadrust::EntityType::Polyline3D(p) = &mut native {
                        for v in &mut p.vertices {
                            v.handle = candidate.allocate_handle();
                        }
                    }
                    let h = candidate.add_entity(native).map_err(|e| e.to_string())?;
                    created.push(Self::fresh_id(&mut ids, h, &mut next)?);
                }
            }
        }
        self.commit(
            candidate,
            ids,
            DrawingChangeKind::Geometry,
            selected.len(),
            removed,
            created,
        )
    }

    pub fn set_geometry(
        &mut self,
        updates: &BTreeMap<EntityId, crate::model::Geometry>,
    ) -> Result<Option<DrawingChange>, String> {
        if updates.is_empty() {
            return Ok(None);
        }
        self.ensure_editable(&updates.keys().copied().collect())?;
        let scale = self.unit_scale()?;
        let handles: BTreeMap<_, _> = self.ids.iter().map(|(h, id)| (*id, *h)).collect();
        let mut candidate = self.native.clone();
        let mut affected = 0;
        for (id, geometry) in updates {
            if self
                .document
                .entity(*id)
                .is_some_and(|e| e.geometry == *geometry)
            {
                continue;
            }
            affected += 1;
            let handle = *handles.get(id).ok_or("缺少源图元记录")?;
            let native = candidate.get_entity_mut(handle).ok_or("源记录失效")?;
            crate::properties::update_native(native, geometry, scale)?;
        }
        if affected == 0 {
            return Ok(None);
        }
        self.commit(
            candidate,
            self.ids.clone(),
            DrawingChangeKind::Geometry,
            affected,
            BTreeSet::new(),
            Vec::new(),
        )
    }

    pub fn edit_layer(&mut self, edit: LayerEdit) -> Result<Option<DrawingChange>, String> {
        use acadrust::tables::Layer;
        let mut candidate = self.native.clone();
        let mut layer_ids = self.layer_ids.clone();
        let layer_name = |id| {
            self.document
                .layer(id)
                .map(|l| l.name.clone())
                .ok_or("图层不存在")
        };
        let color = |c| match c {
            Color::Aci(i) if i > 0 => Ok(NativeColor::Index(i)),
            Color::Rgb { r, g, b } => Ok(NativeColor::Rgb { r, g, b }),
            _ => Err("图层颜色须为 ACI 索引色或 RGB"),
        };
        match edit {
            LayerEdit::Create { name, color: c } => {
                Self::validate_layer_name(&name)?;
                if self
                    .document
                    .layers
                    .iter()
                    .any(|l| l.name.eq_ignore_ascii_case(&name))
                {
                    return Err("图层名称已存在（包括隐式导入层）".into());
                }
                let mut layer = Layer::with_color(name, color(c)?);
                layer.handle = candidate.allocate_handle();
                let next = layer_ids
                    .values()
                    .map(|id| id.0)
                    .max()
                    .map_or(0, |id| id + 1);
                layer_ids.insert(format!("handle:{}", layer.handle.value()), LayerId(next));
                candidate.layers.add(layer)?;
            }
            LayerEdit::Rename { id, name } => {
                Self::validate_layer_name(&name)?;
                let old = layer_name(id)?;
                if old == name {
                    return Ok(None);
                }
                let handle = candidate.rename_layer(&old, &name)?;
                layer_ids.insert(format!("handle:{}", handle.value()), id);
                Self::rename_vertex_layers(&mut candidate, &old, &name);
            }
            LayerEdit::Delete { id, migrate } => {
                let old = layer_name(id)?;
                if old.eq_ignore_ascii_case("0")
                    || old.eq_ignore_ascii_case("Defpoints")
                    || old.contains('|')
                {
                    return Err("不能删除默认、Defpoints 或外部参照图层".into());
                }
                let source_handle = candidate.layers.get(&old).map(|l| l.handle);
                let used=candidate.entities().any(|e|e.common().layer.eq_ignore_ascii_case(&old)
                    || matches!(e,acadrust::EntityType::Viewport(v) if source_handle.is_some_and(|h|v.frozen_layers.contains(&h))));
                if used && migrate.is_none() {
                    return Err("图层被图元或视口引用，必须明确选择迁移目标".into());
                }
                let target = match migrate {
                    Some(target) if target != id => {
                        self.ensure_layer_editable(target)?;
                        Some(layer_name(target)?)
                    }
                    Some(_) => return Err("迁移目标不能是被删除图层".into()),
                    None => None,
                };
                if self.document.layer(id).is_some_and(|l| l.locked) {
                    return Err("请先解锁待删除图层".into());
                }
                // Reuse the native rename routine to migrate name references and XData.
                if let Some(target) = target {
                    let target_layer = candidate
                        .layers
                        .remove(&target)
                        .ok_or("迁移目标缺少原生图层记录")?;
                    let migrated: Vec<_> = candidate
                        .entities()
                        .filter(|e| e.common().layer.eq_ignore_ascii_case(&old))
                        .map(|e| e.common().handle)
                        .collect();
                    candidate.rename_layer(&old, &target)?;
                    Self::rename_vertex_layers(&mut candidate, &old, &target);
                    let source_handle = candidate.layers.remove(&target).unwrap().handle;
                    for handle in migrated {
                        candidate
                            .get_entity_mut(handle)
                            .unwrap()
                            .common_mut()
                            .layer_handle = None;
                    }
                    let viewports: Vec<_> = candidate
                        .entities()
                        .filter_map(|e| match e {
                            acadrust::EntityType::Viewport(v)
                                if v.frozen_layers.contains(&source_handle) =>
                            {
                                Some(e.common().handle)
                            }
                            _ => None,
                        })
                        .collect();
                    for handle in viewports {
                        if let Some(acadrust::EntityType::Viewport(v)) =
                            candidate.get_entity_mut(handle)
                        {
                            for h in &mut v.frozen_layers {
                                if *h == source_handle {
                                    *h = target_layer.handle;
                                }
                            }
                        }
                    }
                    candidate.layers.add(target_layer)?;
                    if self.document.current_layer == Some(id) {
                        let layer = candidate.layers.get(&target).unwrap();
                        candidate.header.current_layer_name = target;
                        candidate.header.current_layer_handle = layer.handle;
                    }
                } else {
                    candidate.layers.remove(&old).ok_or("缺少原生图层记录")?;
                    if self.document.current_layer == Some(id) {
                        let zero = candidate.layers.get("0").ok_or("缺少默认图层")?;
                        candidate.header.current_layer_name = zero.name.clone();
                        candidate.header.current_layer_handle = zero.handle;
                    }
                }
            }
            LayerEdit::Current(id) => {
                self.ensure_layer_editable(id)?;
                let name = layer_name(id)?;
                let layer = candidate.layers.get(&name).ok_or("缺少原生图层记录")?;
                candidate.header.current_layer_name = name;
                candidate.header.current_layer_handle = layer.handle;
            }
            LayerEdit::Color { id, color: c } => {
                let name = layer_name(id)?;
                let layer = candidate.layers.get_mut(&name).ok_or("缺少原生图层记录")?;
                layer.color = color(c)?;
                layer.color_name = None;
                layer.book_name = None;
            }
            LayerEdit::Visible { id, visible } => {
                let name = layer_name(id)?;
                let layer = candidate.layers.get_mut(&name).ok_or("缺少原生图层记录")?;
                layer.flags.off = !visible;
                if visible {
                    layer.flags.frozen = false;
                }
            }
            LayerEdit::Locked { id, locked } => {
                let name = layer_name(id)?;
                candidate
                    .layers
                    .get_mut(&name)
                    .ok_or("缺少原生图层记录")?
                    .flags
                    .locked = locked;
            }
        }
        self.commit_with_layers(
            candidate,
            self.ids.clone(),
            DrawingChangeKind::Layers,
            1,
            BTreeSet::new(),
            Vec::new(),
            layer_ids,
        )
    }

    fn rename_vertex_layers(candidate: &mut CadDocument, old: &str, new: &str) {
        // Touch only affected polylines: a bulk mutable pass would discard unrelated raw records.
        let handles: Vec<_> = candidate
            .entities()
            .filter_map(|e| match e {
                acadrust::EntityType::Polyline3D(p)
                    if p.vertices.iter().any(|v| v.layer.eq_ignore_ascii_case(old)) =>
                {
                    Some(e.common().handle)
                }
                _ => None,
            })
            .collect();
        for handle in handles {
            if let Some(acadrust::EntityType::Polyline3D(p)) = candidate.get_entity_mut(handle) {
                for vertex in &mut p.vertices {
                    if vertex.layer.eq_ignore_ascii_case(old) {
                        vertex.layer = new.to_owned();
                    }
                }
            }
        }
    }

    fn validate_layer_name(name: &str) -> Result<(), String> {
        if name.trim().is_empty()
            || name != name.trim()
            || name
                .chars()
                .any(|c| c.is_control() || "<>/\\\":;?*|=,".contains(c))
        {
            return Err("图层名称不能为空、首尾空格或包含非法符号".into());
        }
        Ok(())
    }

    fn unit_scale(&self) -> Result<f64, String> {
        Ok(self
            .document
            .source_units
            .ok_or("请先明确图纸单位")?
            .millimeters_per_unit())
    }
    fn next_id(ids: &BTreeMap<Handle, EntityId>) -> Result<u32, String> {
        ids.values()
            .map(|id| id.0)
            .max()
            .map_or(Some(0), |id| id.checked_add(1))
            .ok_or_else(|| "图元 ID 已用尽".into())
    }
    fn fresh_id(
        ids: &mut BTreeMap<Handle, EntityId>,
        handle: Handle,
        next: &mut u32,
    ) -> Result<EntityId, String> {
        let id = EntityId(*next);
        *next = next.checked_add(1).ok_or("图元 ID 已用尽")?;
        ids.insert(handle, id);
        Ok(id)
    }
    fn transform_native(entity: &mut acadrust::EntityType, transform: AffineTransform, scale: f64) {
        use acadrust::types::{Matrix4, Transform, Vector3};
        // A transformed clone must not restore the source's verbatim DXF bytes.
        entity.common_mut().raw_record = None;
        let mut matrix = Matrix4::identity();
        for i in 0..3 {
            for j in 0..3 {
                matrix.m[i][j] = transform.linear[i][j];
            }
        }
        let origin = Vector3::new(
            transform.origin[0] / scale,
            transform.origin[1] / scale,
            transform.origin[2] / scale,
        );
        let end = Vector3::new(
            (transform.origin[0] + transform.offset[0]) / scale,
            (transform.origin[1] + transform.offset[1]) / scale,
            (transform.origin[2] + transform.offset[2]) / scale,
        );
        for stage in [
            Transform::from_translation(-origin),
            Transform::from_matrix(matrix),
            Transform::from_translation(end),
        ] {
            if let acadrust::EntityType::Polyline2D(p) = entity {
                let elevation = p
                    .vertices
                    .first()
                    .map(|v| v.location.z)
                    .filter(|z| *z != 0.0)
                    .unwrap_or(p.elevation);
                for v in &mut p.vertices {
                    v.location.z = elevation;
                }
            }
            entity.apply_transform(&stage);
            if let acadrust::EntityType::Polyline2D(p) = entity {
                if let Some(v) = p.vertices.first() {
                    p.elevation = v.location.z;
                }
                for v in &mut p.vertices {
                    v.location.z = 0.0;
                }
            }
        }
    }

    fn commit(
        &mut self,
        candidate: CadDocument,
        ids: BTreeMap<Handle, EntityId>,
        kind: DrawingChangeKind,
        affected: usize,
        removed: BTreeSet<EntityId>,
        created: Vec<EntityId>,
    ) -> Result<Option<DrawingChange>, String> {
        self.commit_with_layers(
            candidate,
            ids,
            kind,
            affected,
            removed,
            created,
            self.layer_ids.clone(),
        )
    }

    fn commit_with_layers(
        &mut self,
        candidate: CadDocument,
        mut ids: BTreeMap<Handle, EntityId>,
        kind: DrawingChangeKind,
        affected: usize,
        removed: BTreeSet<EntityId>,
        created: Vec<EntityId>,
        mut layer_ids: BTreeMap<String, LayerId>,
    ) -> Result<Option<DrawingChange>, String> {
        let document = self.convert_snapshot(&candidate, &mut ids, &mut layer_ids);
        let converted: BTreeSet<_> = document.entities.iter().map(|e| e.id).collect();
        if self
            .document
            .entities
            .iter()
            .any(|e| !removed.contains(&e.id) && !converted.contains(&e.id))
            || created.iter().any(|id| !converted.contains(id))
        {
            return Err("编辑后存在无法转换的图元，已取消整批修改".into());
        }
        if document.entities == self.document.entities
            && document.layers == self.document.layers
            && document.current_layer == self.document.current_layer
        {
            return Ok(None);
        }
        self.serial += 1;
        let change = DrawingChange {
            transaction_id: self.serial,
            affected,
            kind,
            created,
        };
        self.history.push_back(HistoryEntry {
            native: std::mem::replace(&mut self.native, candidate),
            revision: self.revision,
            change: change.clone(),
        });
        if self.history.len() > HISTORY_LIMIT {
            self.history.pop_front();
        }
        self.redo.clear();
        self.revision = self.serial;
        self.ids = ids;
        self.layer_ids = layer_ids;
        self.document = document;
        Ok(Some(change))
    }

    pub fn undo(&mut self) -> Option<DrawingChange> {
        let entry = self.history.pop_back()?;
        let change = entry.change.clone();
        self.redo.push(HistoryEntry {
            native: std::mem::replace(&mut self.native, entry.native),
            revision: std::mem::replace(&mut self.revision, entry.revision),
            change: change.clone(),
        });
        self.refresh_document();
        Some(change)
    }

    pub fn redo(&mut self) -> Option<DrawingChange> {
        let entry = self.redo.pop()?;
        let change = entry.change.clone();
        self.history.push_back(HistoryEntry {
            native: std::mem::replace(&mut self.native, entry.native),
            revision: std::mem::replace(&mut self.revision, entry.revision),
            change: change.clone(),
        });
        self.refresh_document();
        Some(change)
    }

    pub fn export_dxf(&mut self, path: &Path) -> Result<(), String> {
        if !path
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("dxf"))
        {
            return Err("输出文件必须使用 .dxf 扩展名".into());
        }
        self.write_dxf(path, true)?;
        self.saved_revision = self.revision;
        Ok(())
    }

    pub fn save_dxf(&mut self, path: &Path) -> Result<(), String> {
        self.export_dxf(path)?;
        self.document.source.file_path = Some(path.to_path_buf());
        self.document.source.file_name = path.file_name().unwrap().to_string_lossy().into_owned();
        self.document.source.format = FileFormat::AsciiDxf;
        Ok(())
    }

    /// Crash-recovery snapshot; does not change the document's saved version or path.
    pub fn write_recovery(&self, path: &Path) -> Result<(), String> {
        self.write_dxf(path, false)
    }

    fn write_dxf(&self, path: &Path, backup: bool) -> Result<(), String> {
        let bytes = DxfWriter::new(&self.native)
            .write_to_vec()
            .map_err(|error| error.to_string())?;
        crate::file_io::write_dxf(path, &bytes, backup).map_err(|error| error.to_string())
    }

    fn refresh_document(&mut self) {
        let mut ids = std::mem::take(&mut self.ids);
        let mut layer_ids = std::mem::take(&mut self.layer_ids);
        self.document = self.convert_snapshot(&self.native, &mut ids, &mut layer_ids);
        self.ids = ids;
        self.layer_ids = layer_ids;
    }

    fn convert_snapshot(
        &self,
        native: &CadDocument,
        ids: &mut BTreeMap<Handle, EntityId>,
        layer_ids: &mut BTreeMap<String, LayerId>,
    ) -> DrawingDocument {
        let source = self.document.source.clone();
        let warnings = self.document.import_report.warnings.clone();
        let path = source
            .file_path
            .as_deref()
            .unwrap_or_else(|| Path::new(&source.file_name));
        let mut document = convert_document(path, source.format, native, ids, layer_ids);
        document.source = source;
        for warning in warnings {
            if !document.import_report.warnings.contains(&warning) {
                document.import_report.warnings.push(warning);
            }
        }
        document
    }
}
