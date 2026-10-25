//! Diagnostics are snapshots. Selection and repair are disabled when that snapshot is stale.
use bevy_egui::egui;
use cad_core::{
    diagnostics::{self, Kind, Report},
    model::{DrawingDocument, EntityId, Point3},
    CurveEdit,
};
use std::collections::BTreeSet;
pub enum Action {
    Locate {
        ids: BTreeSet<EntityId>,
        point: Point3,
    },
    Preview(CurveEdit),
}
pub struct Controls {
    gap: f64,
    report: Option<(u64, f64, Report)>,
    message: String,
}
impl Default for Controls {
    fn default() -> Self {
        Self {
            gap: 0.01,
            report: None,
            message: String::new(),
        }
    }
}
impl Controls {
    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        doc: &DrawingDocument,
        revision: u64,
    ) -> Option<Action> {
        let mut action = None;
        egui::CollapsingHeader::new("图纸诊断与修复").show(ui,|ui|{
            ui.horizontal(|ui|{ui.label("间隙候选上限 mm");ui.add(egui::DragValue::new(&mut self.gap).range(1e-6..=1.0).speed(0.001));});
            if ui.button("扫描当前图纸").clicked(){self.report=Some((revision,self.gap,diagnostics::analyze(doc,self.gap)));self.message.clear();}
            if !self.message.is_empty(){ui.colored_label(egui::Color32::LIGHT_RED,&self.message);}
            let Some((version,gap,report))=&self.report else{ui.small("诊断不修改图纸；修复需预览并再次确认。");return;};
            let current=*version==revision&&*gap==self.gap;
            if !current{ui.colored_label(egui::Color32::YELLOW,"图纸或参数已变化，请重新扫描后定位/修复。");}
            if report.truncated{ui.colored_label(egui::Color32::YELLOW,format!("诊断达到资源/条目预算，结果不覆盖全图；另省略 {} 项。",report.omitted));}
            ui.label(format!("轮廓 {} · 问题 {} · 已检查组合 {}",report.contours.len(),report.issues.len(),report.pairs));
            if report.issues.is_empty(){ui.label("本次已扫描范围未发现所列问题；不代表完整拓扑验证。");}
            egui::ScrollArea::vertical().id_salt("diagnostic_issues").max_height(260.0).show(ui,|ui|{
                for (index,item) in report.issues.iter().enumerate(){ui.push_id(index,|ui|{ui.group(|ui|{
                    ui.strong(format!("{} · 图元 {}",item.kind.label(),item.entities.iter().map(|id|id.0.to_string()).collect::<Vec<_>>().join(", ")));
                    ui.label(&item.detail);
                    ui.horizontal(|ui|{
                        if ui.add_enabled(current,egui::Button::new("选择并定位")).clicked(){action=Some(Action::Locate{ids:item.entities.iter().copied().collect(),point:item.point});}
                        if matches!(item.kind,Kind::Duplicate|Kind::ZeroLength|Kind::Gap)&&ui.add_enabled(current,egui::Button::new("预览当前项修复")).clicked(){
                            match diagnostics::repair(doc,item,self.gap){Ok(edit)=>{self.message.clear();action=Some(Action::Preview(edit));},Err(error)=>self.message=error,}
                        }
                    });
                });});}
            });
            egui::CollapsingHeader::new("轮廓闭合、长度与方向").show(ui,|ui|{
                for (i,c) in report.contours.iter().take(256).enumerate(){ui.push_id(i,|ui|{
                    let direction=c.direction.map(|(ccw,n)|format!("{}，相对法向 ({:.3},{:.3},{:.3})",if ccw{"逆时针"}else{"顺时针"},n.x,n.y,n.z)).unwrap_or_else(||"方向未定/空间轮廓".into());
                    ui.label(format!("{} 个图元 · {} · {}{:.6} mm · {}",c.entities.len(),if c.closed{"闭合"}else{"开放/分支"},if c.approximate{"约 "}else{""},c.length,direction));
                    if ui.add_enabled(current,egui::Button::new("定位轮廓")).clicked(){action=Some(Action::Locate{ids:c.entities.iter().copied().collect(),point:c.point});}
                });}
                if report.contours.len()>256{ui.small("仅显示前 256 个轮廓。");}
            });
            ui.small("几何容差 1e-6 mm；椭圆/样条交叉为离散候选。只修复选中的重复/零长/间隙项，不自动补齐超限缺口。");
        });
        action
    }
}
