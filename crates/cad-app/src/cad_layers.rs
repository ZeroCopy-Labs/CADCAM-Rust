//! Layer administration; every confirmed edit uses the native drawing transaction.
use bevy_egui::egui;
use cad_core::{
    model::{Color, DrawingDocument, LayerId},
    LayerEdit,
};
#[derive(Default)]
pub struct LayerControls {
    selected: Option<LayerId>,
    name: String,
    rgb: [u8; 3],
    migrate: Option<LayerId>,
}
impl LayerControls {
    pub fn ui(&mut self, ui: &mut egui::Ui, doc: &DrawingDocument) -> Option<LayerEdit> {
        let mut action = None;
        egui::CollapsingHeader::new("图层管理").default_open(true).show(ui,|ui|{
            let previous=self.selected;
            if self.selected.and_then(|id|doc.layer(id)).is_none(){self.selected=doc.current_layer.or_else(||doc.layers.first().map(|l|l.id));}
            egui::ComboBox::from_id_salt("layer_manager_list")
                .selected_text(self.selected.and_then(|id|doc.layer(id)).map(|l|l.name.as_str()).unwrap_or("选择图层"))
                .show_ui(ui,|ui|{
                    for layer in &doc.layers {
                        ui.horizontal(|ui|{
                            let rgb=super::display::color_to_rgb(layer.color);
                            ui.colored_label(egui::Color32::from_rgb((rgb[0]*255.0) as u8,(rgb[1]*255.0) as u8,(rgb[2]*255.0) as u8),"■");
                            ui.selectable_value(&mut self.selected,Some(layer.id),format!("{}{}{}{}",layer.name,if doc.current_layer==Some(layer.id){" [当前]"}else{""},if layer.visible{""}else{" [隐藏]"},if layer.locked{" [锁定]"}else{""}));
                        });
                    }
                });
            if self.selected!=previous || self.name.is_empty(){self.reset(doc);}
            if let Some(layer)=self.selected.and_then(|id|doc.layer(id)) {
                let id=layer.id;
                if ui.add_enabled(layer.visible && !layer.locked && doc.current_layer!=Some(id),egui::Button::new("设为当前图层")).clicked(){action=Some(LayerEdit::Current(id));}
                ui.horizontal(|ui|{
                    let mut visible=layer.visible;
                    if ui.checkbox(&mut visible,"显示").changed(){action=Some(LayerEdit::Visible{id,visible});}
                    let mut locked=layer.locked;
                    if ui.checkbox(&mut locked,"锁定").changed(){action=Some(LayerEdit::Locked{id,locked});}
                });
                ui.horizontal(|ui|{ui.label("颜色");ui.color_edit_button_srgb(&mut self.rgb);});
                if ui.button("应用图层颜色").clicked(){let [r,g,b]=self.rgb;action=Some(LayerEdit::Color{id,color:Color::Rgb{r,g,b}});}
                ui.add(egui::TextEdit::singleline(&mut self.name).hint_text("新建或重命名的图层名"));
                ui.horizontal(|ui|{
                    if ui.button("新建").clicked(){let [r,g,b]=self.rgb;action=Some(LayerEdit::Create{name:self.name.clone(),color:Color::Rgb{r,g,b}});}
                    let protected=layer.name.eq_ignore_ascii_case("0")||layer.name.eq_ignore_ascii_case("Defpoints")||layer.name.contains('|');
                    if ui.add_enabled(!protected,egui::Button::new("重命名")).clicked(){action=Some(LayerEdit::Rename{id,name:self.name.clone()});}
                });
                egui::ComboBox::from_id_salt("layer_delete_target")
                    .selected_text(format!("删除时迁移到：{}",self.migrate.and_then(|id|doc.layer(id)).map(|l|l.name.as_str()).unwrap_or("未选择")))
                    .show_ui(ui,|ui|{
                        ui.selectable_value(&mut self.migrate,None,"未选择（只允许删除空层）");
                        for target in doc.layers.iter().filter(|l|l.id!=id&&l.visible&&!l.locked){ui.selectable_value(&mut self.migrate,Some(target.id),&target.name);}
                    });
                let protected=layer.name.eq_ignore_ascii_case("0")||layer.name.eq_ignore_ascii_case("Defpoints")||layer.name.contains('|');
                if ui.add_enabled(!protected&&!layer.locked,egui::Button::new("删除图层并迁移图元")).clicked(){action=Some(LayerEdit::Delete{id,migrate:self.migrate});}
            }
            ui.small("隐藏不影响 CAM 加工范围；锁定层可查看和捕捉，不能编辑。改名时同步内存中的同名 G 代码规则，冲突时取消。");
        });
        action
    }
    pub fn reset(&mut self, doc: &DrawingDocument) {
        if let Some(layer) = self.selected.and_then(|id| doc.layer(id)) {
            self.name = layer.name.clone();
            let rgb = super::display::color_to_rgb(layer.color);
            self.rgb = rgb.map(|c| (c.clamp(0.0, 1.0) * 255.0).round() as u8);
        }
    }
}
