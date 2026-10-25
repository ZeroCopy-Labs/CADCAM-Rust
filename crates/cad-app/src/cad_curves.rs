//! Explicit parameters and preview recipes for the first contour-editing tools.
use bevy_egui::egui;
use cad_core::{
    curve_edit::{self, CurveOptions, CurveTool},
    model::{DrawingDocument, EntityId, Plane, WorkPlane},
    CurveEdit,
};
use std::collections::BTreeSet;
pub struct CurveControls {
    pub tool: Option<CurveTool>,
    options: CurveOptions,
    pub copy_offset: bool,
}
impl Default for CurveControls {
    fn default() -> Self {
        Self {
            tool: None,
            options: CurveOptions::default(),
            copy_offset: true,
        }
    }
}
pub const TOOLS: [(CurveTool, &str); 9] = [
    (CurveTool::Break, "BREAK"),
    (CurveTool::Trim, "TRIM"),
    (CurveTool::Extend, "EXTEND"),
    (CurveTool::Join, "JOIN"),
    (CurveTool::Close, "CLOSE"),
    (CurveTool::Chamfer, "CHAMFER"),
    (CurveTool::Fillet, "FILLET"),
    (CurveTool::Offset, "OFFSET"),
    (CurveTool::Reverse, "REVERSE"),
];
impl CurveControls {
    pub fn recipe(
        &self,
        doc: &DrawingDocument,
        selected: &BTreeSet<u32>,
        plane: WorkPlane,
    ) -> Result<CurveEdit, String> {
        let tool = self.tool.unwrap_or(CurveTool::Break);
        let entities: Vec<_> = selected
            .iter()
            .map(|id| doc.entity(EntityId(*id)).ok_or("选择已失效"))
            .collect::<Result<_, _>>()?;
        if entities.is_empty() {
            return Err("请先选择需要整理的图元".into());
        }
        if entities.iter().any(|e| {
            doc.layer(e.layer_id)
                .is_some_and(|l| l.locked || !l.visible)
        }) {
            return Err("选择含隐藏或锁定层，已取消整批操作".into());
        }
        let mut options = self.options;
        options.plane = Plane::new(plane.normal(), plane.x_axis);
        let mut edit = CurveEdit::default();
        if matches!(
            tool,
            CurveTool::Join | CurveTool::Chamfer | CurveTool::Fillet
        ) {
            let first = entities[0];
            if entities
                .iter()
                .any(|e| e.layer_id != first.layer_id || e.color != first.color)
            {
                return Err("合并对象须有相同图层和颜色，避免静默覆盖属性".into());
            }
            let output = curve_edit::apply(
                tool,
                &entities
                    .iter()
                    .map(|e| e.geometry.clone())
                    .collect::<Vec<_>>(),
                options,
            )?;
            edit.replacements.insert(first.id, output);
            edit.remove = entities.iter().skip(1).map(|e| e.id).collect();
        } else {
            for e in entities {
                edit.replacements.insert(
                    e.id,
                    curve_edit::apply(tool, &[e.geometry.clone()], options)?,
                );
            }
            edit.copies = tool == CurveTool::Offset && self.copy_offset;
        }
        Ok(edit)
    }
    pub fn ui(&mut self, ui: &mut egui::Ui) -> bool {
        let mut preview = false;
        egui::CollapsingHeader::new("曲线整理").default_open(false).show(ui,|ui|{
            let tool=self.tool.get_or_insert(CurveTool::Break);
            egui::ComboBox::from_id_salt("curve_tool").selected_text(tool.label()).show_ui(ui,|ui|{
                for (value,_) in TOOLS{ui.selectable_value(tool,value,value.label());}
            });
            match tool{
                CurveTool::Break|CurveTool::Trim=>{
                    ui.horizontal(|ui|{ui.label(if *tool==CurveTool::Break{"打断参数"}else{"保留起始参数"});ui.add(egui::DragValue::new(&mut self.options.from).speed(0.01));});
                    if *tool==CurveTool::Trim{ui.horizontal(|ui|{ui.label("保留结束参数");ui.add(egui::DragValue::new(&mut self.options.to).speed(0.01));});}
                    ui.small("参数范围 0～1；多段线按段数等分，非弧长比例。圆的参数从自身 +U 开始。");
                },
                CurveTool::Extend|CurveTool::Chamfer|CurveTool::Fillet|CurveTool::Offset=>{
                    ui.horizontal(|ui|{ui.label(match tool{CurveTool::Fillet=>"半径 mm",CurveTool::Chamfer=>"两边退让 mm",_=>"距离 mm"});ui.add(egui::DragValue::new(&mut self.options.length).speed(0.1));});
                    if *tool==CurveTool::Extend{ui.checkbox(&mut self.options.at_start,"从起点延伸（关闭为终点）");}
                    if *tool==CurveTool::Offset{ui.checkbox(&mut self.copy_offset,"保留原件并创建偏移副本");ui.small("直线/多段线：正值向行进左侧；圆/独立圆弧：正值增大半径。");}
                },_=>{},
            }
            if matches!(tool,CurveTool::Join|CurveTool::Chamfer|CurveTool::Fillet){
                ui.horizontal(|ui|{ui.label("连接容差 mm");ui.add(egui::DragValue::new(&mut self.options.tolerance).speed(0.001));});
                ui.small("倒角/圆角选择两条端点相接直线；合并结果沿用最小 ID 对象的公共属性。");
            }
            if *tool==CurveTool::Close{ui.small("为开放轮廓明确添加直线闭合边；单条直线不支持闭合。");}
            if ui.button("预览曲线整理结果").clicked(){preview=true;}
            ui.small("预览后 Enter 提交，Esc 取消；一条历史恢复整批。首批结构编辑支持零宽/零厚度直线、圆/圆弧和多段线。");
        });
        preview
    }
}
