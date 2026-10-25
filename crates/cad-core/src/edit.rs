//! Batch editing with stable model identities and native DXF record retention.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::path::Path;

use acadrust::{CadDocument, Color as NativeColor, DxfWriter, Handle};

use crate::convert::convert_document;
use crate::model::{Color, DrawingDocument, EntityId, FileFormat, LayerId, LengthUnit};

const HISTORY_LIMIT: usize = 100;

/// One committed batch, shared by apply, undo and redo notifications.
#[derive(Clone, Copy, Debug)]
pub struct DrawingChange {
    /// Unique within this loaded drawing, including branches after undo.
    pub transaction_id: u64,
    /// Number of source entities affected by this batch.
    pub affected: usize,
    /// Original operation, also returned when this transaction is undone/redone.
    pub edit: DrawingEdit,
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
        let layer = match edit {
            DrawingEdit::SetLayer(id) => Some(
                self.document
                    .layer(id)
                    .ok_or("目标图层不存在")?
                    .name
                    .clone(),
            ),
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
        let mut ids = self.ids.clone();
        let mut layer_ids = self.layer_ids.clone();
        let document = self.convert_snapshot(&candidate, &mut ids, &mut layer_ids);
        let converted: BTreeSet<_> = document.entities.iter().map(|entity| entity.id).collect();
        for entity in &self.document.entities {
            if !(converted.contains(&entity.id)
                || matches!(edit, DrawingEdit::Delete) && selected.contains(&entity.id))
            {
                return Err("编辑后存在无法转换的图元，已取消整批修改".to_string());
            }
        }
        if document.entities == self.document.entities {
            return Ok(None);
        }
        self.serial += 1;
        let change = DrawingChange {
            transaction_id: self.serial,
            affected: handles.len(),
            edit,
        };
        self.history.push_back(HistoryEntry {
            native: std::mem::replace(&mut self.native, candidate),
            revision: self.revision,
            change,
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
        let change = entry.change;
        self.redo.push(HistoryEntry {
            native: std::mem::replace(&mut self.native, entry.native),
            revision: std::mem::replace(&mut self.revision, entry.revision),
            change,
        });
        self.refresh_document();
        Some(change)
    }

    pub fn redo(&mut self) -> Option<DrawingChange> {
        let entry = self.redo.pop()?;
        let change = entry.change;
        self.history.push_back(HistoryEntry {
            native: std::mem::replace(&mut self.native, entry.native),
            revision: std::mem::replace(&mut self.revision, entry.revision),
            change,
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
