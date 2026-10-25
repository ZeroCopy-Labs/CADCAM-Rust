//! Interactive curve picking; results remain previews until the final click or keyboard confirmation.
use cad_core::{
    curve_edit::{CurveOptions, CurveTool},
    curve_pick, curves,
    model::{EntityId, Geometry, Point3, Vector3, WorkPlane},
    snap::{self, SnapKind},
    CurveEdit, DrawingChange, DrawingDocument,
};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Boundary,
    Parameter,
    Object,
    FirstLine,
    SecondLine,
    Side,
    Confirm,
    Finished,
}
struct Source {
    id: EntityId,
    geometry: Geometry,
    pick: Point3,
}
struct UndoRecord {
    transaction_id: u64,
    added_boundaries: Vec<u32>,
    removed_boundaries: Vec<u32>,
}
pub struct Session {
    tool: CurveTool,
    options: CurveOptions,
    phase: Phase,
    boundaries: BTreeSet<u32>,
    source: Option<Source>,
    preview: Option<CurveEdit>,
    affected: BTreeSet<EntityId>,
    operations: Vec<UndoRecord>,
}
pub fn interactive(tool: CurveTool) -> bool {
    matches!(
        tool,
        CurveTool::Trim
            | CurveTool::Extend
            | CurveTool::Offset
            | CurveTool::Fillet
            | CurveTool::Chamfer
    )
}
impl Session {
    pub fn new(tool: CurveTool, options: CurveOptions) -> Self {
        Self {
            tool,
            options,
            phase: if matches!(tool, CurveTool::Trim | CurveTool::Extend) {
                Phase::Boundary
            } else {
                Phase::Parameter
            },
            boundaries: BTreeSet::new(),
            source: None,
            preview: None,
            affected: BTreeSet::new(),
            operations: Vec::new(),
        }
    }
    pub fn continuous(&self) -> bool {
        matches!(
            self.tool,
            CurveTool::Trim
                | CurveTool::Extend
                | CurveTool::Offset
                | CurveTool::Fillet
                | CurveTool::Chamfer
        )
    }
    pub fn waiting_target(&self) -> bool {
        self.continuous()
            && matches!(
                self.phase,
                Phase::Object | Phase::FirstLine | Phase::SecondLine
            )
    }
    pub fn finished(&self) -> bool {
        self.phase == Phase::Finished
    }
    pub fn parameter_option(&self) -> Option<(&'static str, &'static str)> {
        match self.tool {
            CurveTool::Offset => Some(("D", "更改偏移距离")),
            CurveTool::Fillet => Some(("R", "更改圆角半径")),
            CurveTool::Chamfer => Some(("D", "更改倒角两边退让距离")),
            _ => None,
        }
    }
    pub fn can_reselect(&self) -> bool {
        matches!(self.phase, Phase::Side | Phase::SecondLine)
    }
    pub fn parameter_input(&self) -> bool {
        self.phase == Phase::Parameter
    }
    pub fn undo_id(&self) -> Option<u64> {
        self.operations.last().map(|op| op.transaction_id)
    }
    pub fn boundaries(&self) -> &BTreeSet<u32> {
        &self.boundaries
    }
    pub fn resume(&mut self) {
        self.source = None;
        self.preview = None;
        self.affected.clear();
        self.phase = if matches!(self.tool, CurveTool::Fillet | CurveTool::Chamfer) {
            Phase::FirstLine
        } else {
            Phase::Object
        };
    }
    pub fn committed(&mut self, change: &DrawingChange, doc: &DrawingDocument) {
        let added: Vec<_> = change
            .created_sources
            .iter()
            .filter(|(_, source)| self.boundaries.contains(&source.0))
            .map(|(id, _)| id.0)
            .filter(|id| !self.boundaries.contains(id))
            .collect();
        let removed: Vec<_> = self
            .boundaries
            .iter()
            .copied()
            .filter(|id| doc.entity(EntityId(*id)).is_none())
            .collect();
        self.boundaries
            .retain(|id| doc.entity(EntityId(*id)).is_some());
        self.boundaries.extend(&added);
        self.operations.push(UndoRecord {
            transaction_id: change.transaction_id,
            added_boundaries: added,
            removed_boundaries: removed,
        });
        self.resume();
    }
    pub fn undone(&mut self, id: u64, doc: &DrawingDocument) {
        if self.undo_id() != Some(id) {
            return;
        }
        let op = self.operations.pop().unwrap();
        for id in op.added_boundaries {
            self.boundaries.remove(&id);
        }
        self.boundaries.extend(
            op.removed_boundaries
                .into_iter()
                .filter(|id| doc.entity(EntityId(*id)).is_some()),
        );
        self.boundaries
            .retain(|id| doc.entity(EntityId(*id)).is_some());
        self.resume();
    }
    pub fn selecting(&self) -> bool {
        self.phase == Phase::Boundary
    }
    pub fn picking(&self) -> bool {
        matches!(
            self.phase,
            Phase::Object | Phase::FirstLine | Phase::SecondLine
        )
    }
    pub fn side(&self) -> bool {
        self.phase == Phase::Side
    }
    pub fn confirmed(&self) -> bool {
        self.phase == Phase::Confirm
    }
    pub fn length(&self) -> f64 {
        self.options.length
    }
    pub fn parameter_spec(&self, plane: WorkPlane) -> Option<super::cad_input::Spec> {
        (self.phase == Phase::Parameter).then_some(super::cad_input::Spec {
            kind: if self.tool == CurveTool::Fillet {
                super::cad_input::Kind::Radius
            } else {
                super::cad_input::Kind::Distance
            },
            reference: None,
            plane,
            point: None,
            value: self.options.length.abs(),
            secondary: 0.,
        })
    }
    pub fn prompt(&self) -> String {
        let tool = match self.tool {
            CurveTool::Trim => "修剪",
            CurveTool::Extend => "延伸",
            CurveTool::Offset => "偏移",
            CurveTool::Fillet => "圆角",
            _ => "倒角",
        };
        let mut prompt = format!(
            "{tool}：{}",
            match self.phase {
                Phase::Boundary => "选择边界对象后确认；Shift 移除",
                Phase::Parameter =>
                    if self.tool == CurveTool::Fillet {
                        "输入半径并确认"
                    } else {
                        "输入距离并确认"
                    },
                Phase::Object => match self.tool {
                    CurveTool::Trim => "悬停预览要去除的区间，点击提交",
                    CurveTool::Extend => "悬停待延伸端预览，点击提交",
                    _ => "点击需要偏移的对象",
                },
                Phase::FirstLine => "点击第一条直线要保留的一侧",
                Phase::SecondLine => "悬停第二条直线的保留侧预览，点击提交",
                Phase::Side => "悬停选侧预览，点击或输入坐标确认提交",
                Phase::Confirm => "Enter/空格提交预览；Esc 取消",
                Phase::Finished => "命令已结束",
            }
        );
        if self.continuous() {
            prompt.push_str(&format!(
                "；已完成 {} 次，U 撤回本命令上一次；Enter 在等待目标时结束，Esc 保留已完成结果",
                self.operations.len()
            ));
        }
        if let Some((key, label)) = self.parameter_option() {
            prompt.push_str(&format!(
                "；{key} {label}（当前 {:.4} mm）",
                self.options.length
            ));
        }
        if self.can_reselect() {
            prompt.push_str("；N 丢弃当前未提交预览并重新选择对象/边对");
        }
        prompt
    }
    pub fn input(
        &mut self,
        text: &str,
        selected: &BTreeSet<u32>,
    ) -> Result<Option<CurveEdit>, String> {
        let upper = text.trim().to_uppercase();
        if self.can_reselect() && matches!(upper.as_str(), "N" | "NEXT") {
            self.resume();
            return Ok(None);
        }
        if self.parameter_option().is_some_and(|(key, _)| {
            upper == key || key == "D" && upper == "DISTANCE" || key == "R" && upper == "RADIUS"
        }) {
            self.source = None;
            self.preview = None;
            self.affected.clear();
            self.phase = Phase::Parameter;
            return Ok(None);
        }
        if self.waiting_target() && text.trim().is_empty() {
            self.preview = None;
            self.affected.clear();
            self.phase = Phase::Finished;
            return Ok(None);
        }
        match self.phase {
            Phase::Boundary => {
                if !text.trim().is_empty() || selected.is_empty() {
                    return Err("请选择边界对象后确认".into());
                }
                self.boundaries = selected.clone();
                self.phase = Phase::Object;
            }
            Phase::Parameter => {
                let value = if text.trim().is_empty() {
                    self.options.length.abs()
                } else {
                    text.trim()
                        .parse::<f64>()
                        .map_err(|_| "请输入有效距离或半径")?
                };
                if !value.is_finite() || value <= curves::DISTANCE_TOLERANCE {
                    return Err("距离/半径须大于几何容差且有限".into());
                }
                self.options.length = value;
                self.phase = if self.tool == CurveTool::Offset {
                    Phase::Object
                } else {
                    Phase::FirstLine
                };
            }
            Phase::Confirm => {
                if !text.trim().is_empty() {
                    return Err("预览固定后请确认提交或 Esc 取消".into());
                }
                return Ok(self.preview.clone());
            }
            _ => return Err("请按提示在画布点选对象或处理侧".into()),
        }
        Ok(None)
    }
    pub fn preview(&self) -> Option<&CurveEdit> {
        self.preview.as_ref()
    }
    pub fn affected(&self) -> &BTreeSet<EntityId> {
        &self.affected
    }
    pub fn first_geometry(&self) -> Option<&Geometry> {
        (self.phase == Phase::SecondLine)
            .then(|| self.source.as_ref().map(|s| &s.geometry))
            .flatten()
    }
    fn show(&mut self, edit: CurveEdit) {
        self.affected = if edit.copies {
            BTreeSet::new()
        } else {
            edit.replacements
                .keys()
                .chain(edit.remove.iter())
                .copied()
                .collect()
        };
        self.preview = Some(edit);
    }
    pub fn side_plane(&self) -> Option<WorkPlane> {
        let source = self.source.as_ref()?;
        let plane = match &source.geometry {
            Geometry::Circle(c) => c.plane,
            Geometry::Arc(a) => a.plane,
            Geometry::Polyline(p) => p.plane.unwrap_or(self.options.plane),
            _ => self.options.plane,
        };
        Some(WorkPlane {
            origin: source.pick,
            x_axis: plane.x_axis,
            y_axis: plane.y_axis(),
        })
    }
    pub fn preview_side(&mut self, point: Point3, freeze: bool) -> Result<(), String> {
        self.preview = None;
        self.affected.clear();
        let source = self.source.as_ref().ok_or("请先点选偏移对象")?;
        let geometry = curve_pick::offset(&source.geometry, point, self.options)?;
        let edit = CurveEdit {
            replacements: BTreeMap::from([(source.id, vec![geometry])]),
            copies: true,
            ..Default::default()
        };
        self.show(edit);
        if freeze {
            self.phase = Phase::Confirm;
        }
        Ok(())
    }
    pub fn viewport(
        &mut self,
        doc: &DrawingDocument,
        id: Option<u32>,
        ray: (Point3, Vector3),
        click: bool,
    ) -> Result<bool, String> {
        self.preview = None;
        self.affected.clear();
        let Some(id) = id else {
            return Ok(false);
        };
        let entity = doc.entity(EntityId(id)).ok_or("点选对象已失效")?;
        if !entity.visible
            || doc
                .layer(entity.layer_id)
                .is_some_and(|l| l.locked || !l.visible)
        {
            return Err("请选择可见且未锁定的目标对象".into());
        }
        let modes = BTreeSet::from([SnapKind::Nearest]);
        let pick = snap::snap_points(&entity.geometry, ray.0, ray.1, None, &modes)
            .into_iter()
            .min_by(|a, b| {
                snap::ray_distance(a.point, ray.0, ray.1)
                    .total_cmp(&snap::ray_distance(b.point, ray.0, ray.1))
            })
            .ok_or("无法在该对象上定位点选位置")?
            .point;
        let source = Source {
            id: entity.id,
            geometry: entity.geometry.clone(),
            pick,
        };
        match self.phase {
            Phase::Object if matches!(self.tool, CurveTool::Trim | CurveTool::Extend) => {
                let boundaries = self
                    .boundaries
                    .iter()
                    .filter(|boundary| **boundary != id)
                    .map(|id| {
                        doc.entity(EntityId(*id))
                            .map(|e| e.geometry.clone())
                            .ok_or("边界对象已失效")
                    })
                    .collect::<Result<Vec<_>, _>>()?;
                if boundaries.is_empty() {
                    return Err("目标之外至少需要一个边界对象".into());
                }
                let output = if self.tool == CurveTool::Trim {
                    curve_pick::trim(&source.geometry, &boundaries, pick)?
                } else {
                    vec![curve_pick::extend(
                        &source.geometry,
                        &boundaries,
                        pick,
                        self.options,
                    )?]
                };
                let mut edit = CurveEdit::default();
                if output.is_empty() {
                    edit.remove.insert(entity.id);
                } else {
                    edit.replacements.insert(entity.id, output);
                }
                self.show(edit);
                if click {
                    self.phase = Phase::Confirm;
                    return Ok(true);
                }
            }
            Phase::Object => {
                curve_pick::parameter(&source.geometry, pick)?;
                if click {
                    self.source = Some(source);
                    self.phase = Phase::Side;
                    return Ok(true);
                }
            }
            Phase::FirstLine => {
                if !matches!(source.geometry, Geometry::Line(_)) {
                    return Err("请选择直线".into());
                }
                if click {
                    self.source = Some(source);
                    self.phase = Phase::SecondLine;
                    return Ok(true);
                }
            }
            Phase::SecondLine => {
                let first = self.source.as_ref().ok_or("请先选择第一条直线")?;
                if first.id == source.id {
                    return Err("请选择另一条直线".into());
                }
                let [a, connector, b] = curve_pick::corner(
                    &first.geometry,
                    &source.geometry,
                    first.pick,
                    pick,
                    self.options,
                    self.tool == CurveTool::Fillet,
                )?;
                let edit = CurveEdit {
                    replacements: BTreeMap::from([
                        (first.id, vec![a, connector]),
                        (source.id, vec![b]),
                    ]),
                    ..Default::default()
                };
                self.show(edit);
                if click {
                    self.phase = Phase::Confirm;
                    return Ok(true);
                }
            }
            _ => {}
        }
        Ok(false)
    }
}
