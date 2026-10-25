//! Selection scope and explicit add/remove/toggle behavior.
use super::display::{DisplayData, DisplayEntity};
use bevy_egui::egui;
use std::collections::BTreeSet;

#[derive(Default, Clone, Copy, PartialEq, Eq)]
pub enum SelectMode {
    Replace,
    #[default]
    Add,
    Remove,
    Toggle,
}
#[derive(Default)]
pub struct SelectionFilter {
    kind: Option<String>,
    layer: Option<String>,
    color: Option<String>,
    pub mode: SelectMode,
}
impl SelectionFilter {
    pub fn matches(&self, e: &DisplayEntity) -> bool {
        e.visible
            && self.kind.as_ref().is_none_or(|kind| kind == e.kind)
            && self.layer.as_ref().is_none_or(|layer| layer == &e.layer)
            && self
                .color
                .as_ref()
                .is_none_or(|color| color == &e.color_key)
    }
    pub fn ui(&mut self, ui: &mut egui::Ui, display: &DisplayData) -> bool {
        let mut invert = false;
        let restricted = self.kind.is_some() || self.layer.is_some() || self.color.is_some();
        egui::CollapsingHeader::new(if restricted {
            "选择方式与过滤 · 已启用"
        } else {
            "选择方式与过滤"
        })
        .show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                for (mode, label) in [
                    (SelectMode::Replace, "替换"),
                    (SelectMode::Add, "追加"),
                    (SelectMode::Remove, "移除"),
                    (SelectMode::Toggle, "切换"),
                ] {
                    ui.selectable_value(&mut self.mode, mode, label);
                }
            });
            for (id, label, current, values) in [
                (
                    "kind",
                    "类型",
                    &mut self.kind,
                    display
                        .entities
                        .iter()
                        .map(|e| e.kind.to_string())
                        .collect::<BTreeSet<_>>(),
                ),
                (
                    "layer",
                    "图层",
                    &mut self.layer,
                    display.entities.iter().map(|e| e.layer.clone()).collect(),
                ),
                (
                    "color",
                    "颜色属性",
                    &mut self.color,
                    display
                        .entities
                        .iter()
                        .map(|e| e.color_key.clone())
                        .collect(),
                ),
            ] {
                egui::ComboBox::from_id_salt(id)
                    .selected_text(format!("{label}：{}", current.as_deref().unwrap_or("全部")))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(current, None, "全部");
                        for value in values {
                            ui.selectable_value(current, Some(value.clone()), value);
                        }
                    });
            }
            if ui.button("反选过滤范围").clicked() {
                invert = true;
            }
            if ui.button("清除过滤").clicked() {
                self.kind = None;
                self.layer = None;
                self.color = None;
            }
            ui.small("默认追加选择；Shift/Alt 移除、Ctrl 切换。过滤只限制新选择。");
            ui.small("Tab 循环重叠候选，Shift+Tab 反向。");
        });
        invert
    }
    pub fn apply(
        &self,
        selection: &mut BTreeSet<u32>,
        hits: BTreeSet<u32>,
        shift: bool,
        alt: bool,
        ctrl: bool,
    ) {
        let mode = if ctrl {
            SelectMode::Toggle
        } else if alt || shift {
            SelectMode::Remove
        } else {
            self.mode
        };
        match mode {
            SelectMode::Replace => *selection = hits,
            SelectMode::Add => selection.extend(hits),
            SelectMode::Remove => selection.retain(|id| !hits.contains(id)),
            SelectMode::Toggle => {
                for id in hits {
                    if !selection.remove(&id) {
                        selection.insert(id);
                    }
                }
            }
        }
    }
}
