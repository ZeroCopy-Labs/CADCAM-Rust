//! CAD ribbon and workspace layout; actions still use the existing edit pipeline.
use std::collections::BTreeSet;

use bevy_egui::egui;
use cad_core::{model::Color, DrawingDocument, DrawingEdit, LayerEdit};

use super::{cad_edit::EditAction, cad_tools::ToolOptions};

#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub enum Pane {
    #[default]
    Properties,
    Layers,
    Tools,
}

pub struct Workspace {
    pub open: bool,
    pub pane: Pane,
    pub history: bool,
    pub context: Option<egui::Pos2>,
}
impl Default for Workspace {
    fn default() -> Self {
        Self {
            open: true,
            pane: Pane::Properties,
            history: false,
            context: None,
        }
    }
}
impl Workspace {
    pub fn reveal(&mut self, pane: Pane) {
        self.open = true;
        self.pane = pane;
    }
    pub fn context_ui(
        &mut self,
        ctx: &egui::Context,
        canvas: egui::Rect,
        command: &super::cad_command::CommandState,
        can_undo: bool,
        can_redo: bool,
    ) -> Option<String> {
        let mut opened = false;
        if let Some(pos) = ctx
            .input(|i| i.pointer.interact_pos())
            .filter(|p| canvas.contains(*p))
        {
            if ctx.input(|i| i.pointer.secondary_clicked()) && !ctx.egui_wants_pointer_input() {
                self.context = Some(pos);
                opened = true;
            }
        }
        let pos = self.context?;
        let mut action = None;
        let response = egui::Area::new(egui::Id::new("cad_context_menu"))
            .order(egui::Order::Foreground)
            .fixed_pos(pos)
            .constrain_to(canvas)
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.set_min_width(175.);
                    if ui
                        .button(
                            if command.curve_waiting_target() || command.copy_waiting_target() {
                                "确认 / 结束连续修改  Enter"
                            } else if command.is_active() {
                                "确认 / 结束  Enter"
                            } else {
                                "重复上一命令  Enter / 空格"
                            },
                        )
                        .clicked()
                    {
                        action = Some(String::new());
                    }
                    if command.continuous_drawing() {
                        if ui.button("撤回上一点  U").clicked() {
                            action = Some("U".into());
                        }
                        if ui.button("闭合并完成  C").clicked() {
                            action = Some("C".into());
                        }
                        if ui.button("放弃整次未提交绘图").clicked() {
                            action = Some("DISCARD".into());
                        }
                    }
                    if command.circle_drawing() {
                        for &(option, label) in command.circle_options() {
                            if ui.button(format!("{label}  {option}")).clicked() {
                                action = Some(option.into());
                            }
                        }
                        if ui.button("撤回上一取点  U").clicked() {
                            action = Some("U".into());
                        }
                    }
                    if command.arc_drawing() {
                        for &(option, label) in command.arc_options() {
                            if ui.button(format!("{label}  {option}")).clicked() {
                                action = Some(option.into());
                            }
                        }
                        if ui.button("撤回圆弧上一取点  U").clicked() {
                            action = Some("U".into());
                        }
                    }
                    if command.rectangle_drawing() {
                        if command.rectangle_corner() && ui.button("设置矩形旋转角度  R").clicked()
                        {
                            action = Some("R".into());
                        }
                        if command.rectangle_corner()
                            && ui.button("输入矩形长度 / 宽度  D").clicked()
                        {
                            action = Some("D".into());
                        }
                        if ui.button("撤回矩形上一步  U").clicked() {
                            action = Some("U".into());
                        }
                    }
                    if command.command_undo_active() {
                        if ui
                            .add_enabled(
                                can_undo && command.command_undo_id().is_some(),
                                egui::Button::new("撤回本命令上次修改  U"),
                            )
                            .clicked()
                        {
                            action = Some("U".into());
                        }
                        if command.curve_change_distance() && ui.button("更改偏移距离  D").clicked()
                        {
                            action = Some("D".into());
                        }
                        if command.continuous_curve()
                            && command.takes_point()
                            && ui.button("丢弃当前预览，重新选对象  N").clicked()
                        {
                            action = Some("N".into());
                        }
                        if ui.button("结束连续修改，保留已完成结果").clicked() {
                            action = Some("CANCEL".into());
                        }
                    }
                    if command.copy_mode().is_some() && !command.selecting() {
                        if ui.button("多次复制  M").clicked() {
                            action = Some("M".into());
                        }
                        if ui.button("单次复制  S").clicked() {
                            action = Some("S".into());
                        }
                    }
                    if command.transform_reference() {
                        if ui.button("参照方向 / 长度  R").clicked() {
                            action = Some("R".into());
                        }
                        if ui.button("切换保留原件复制  C").clicked() {
                            action = Some("C".into());
                        }
                    }
                    if command.mirror_question() {
                        if ui.button("保留源对象  N").clicked() {
                            action = Some("N".into());
                        }
                        if ui.button("替换源对象  Y").clicked() {
                            action = Some("Y".into());
                        }
                    }
                    if command.is_active() {
                        if ui.button("取消当前操作  Esc").clicked() {
                            action = Some("CANCEL".into());
                        }
                    } else {
                        ui.separator();
                        for (enabled, label, name) in [
                            (can_undo, "撤销", "UNDO"),
                            (can_redo, "重做", "REDO"),
                            (true, "移动", "MOVE"),
                            (true, "复制", "COPY"),
                            (true, "删除", "ERASE"),
                        ] {
                            if ui.add_enabled(enabled, egui::Button::new(label)).clicked() {
                                action = Some(name.into());
                            }
                        }
                    }
                });
            });
        if action.is_some()
            || !opened
                && ctx.input(|i| {
                    i.pointer.any_click()
                        && i.pointer
                            .interact_pos()
                            .is_some_and(|p| !response.response.rect.contains(p))
                })
        {
            self.context = None;
        }
        action
    }
}

#[derive(Default)]
pub struct Output {
    pub command: Option<String>,
    pub edit: Option<EditAction>,
}

const DRAW: &[(&str, &str)] = &[
    ("LINE", "直线"),
    ("PLINE", "多段线"),
    ("CIRCLE", "圆"),
    ("ARC", "圆弧"),
    ("RECTANGLE", "矩形"),
    ("ELLIPSE", "椭圆"),
    ("SPLINE", "样条曲线"),
    ("HOLE", "圆孔"),
    ("SLOT", "长圆孔"),
    ("CIRCLED", "圆心 / 直径圆"),
    ("CIRCLE2P", "两点直径圆"),
    ("CIRCLE3P", "三点圆"),
    ("ARCCE", "圆心 / 起点 / 终点方向圆弧"),
    ("ARCCA", "圆心 / 起点 / 扫角圆弧"),
];
const MODIFY: &[(&str, &str)] = &[
    ("MOVE", "移动"),
    ("COPY", "复制"),
    ("ROTATE", "旋转"),
    ("MIRROR", "镜像"),
    ("SCALE", "缩放"),
    ("ERASE", "删除"),
    ("OFFSET", "偏移"),
    ("TRIM", "修剪"),
    ("EXTEND", "延伸"),
    ("FILLET", "圆角"),
    ("CHAMFER", "倒角"),
    ("ARRAYRECT", "矩形阵列"),
    ("ARRAYPOLAR", "环形阵列"),
    ("BREAK", "打断"),
    ("JOIN", "合并"),
    ("CLOSE", "闭合"),
    ("REVERSE", "反向"),
];
const MEASURE: &[(&str, &str)] = &[("DIST", "距离"), ("ANGLE", "角度"), ("ID", "坐标")];

pub fn ribbon(
    ui: &mut egui::Ui,
    workspace: &mut Workspace,
    document: Option<&DrawingDocument>,
    selected: &BTreeSet<u32>,
    tools: &mut ToolOptions,
    active: bool,
) -> Output {
    let mut output = Output::default();
    ui.horizontal(|ui| {
        ui.strong("默认");
        ui.separator();
        ui.weak("CAD 工作区");
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.selectable_label(workspace.open, "特性面板").clicked() {
                workspace.open = !workspace.open;
            }
        });
    });
    let enabled = document.is_some() && !active;
    if ui.available_width() < 1020.0 {
        ui.horizontal_wrapped(|ui| {
            menu(ui, "绘图", DRAW, enabled, selected, &mut output);
            menu(ui, "修改", MODIFY, enabled, selected, &mut output);
            ui.menu_button("图层", |ui| {
                ui.add_enabled_ui(enabled, |ui| layer_ui(ui, document, selected, &mut output));
                if button(ui, "LAYERS", "图层管理").clicked() {
                    workspace.reveal(Pane::Layers);
                    ui.close();
                }
            });
            ui.menu_button("特性", |ui| {
                ui.add_enabled_ui(enabled, |ui| {
                    color_ui(ui, document, selected, tools, &mut output)
                });
                if button(ui, "PROPERTIES", "详细特性").clicked() {
                    workspace.reveal(Pane::Properties);
                    ui.close();
                }
            });
            utility_menu(ui, workspace, enabled, selected, &mut output);
        });
    } else {
        ui.horizontal_top(|ui| {
            group(ui, 164.0, |ui| {
                command_grid(
                    ui,
                    "draw_grid",
                    &DRAW[..6],
                    2,
                    enabled,
                    selected,
                    &mut output,
                );
                menu(ui, "绘图 ▾", DRAW, enabled, selected, &mut output);
            });
            group(ui, 246.0, |ui| {
                command_grid(
                    ui,
                    "modify_grid",
                    &MODIFY[..9],
                    3,
                    enabled,
                    selected,
                    &mut output,
                );
                menu(ui, "修改 ▾", MODIFY, enabled, selected, &mut output);
            });
            group(ui, 192.0, |ui| {
                ui.add_enabled_ui(enabled, |ui| layer_ui(ui, document, selected, &mut output));
                if button(ui, "LAYERS", "图层管理").clicked() {
                    workspace.reveal(Pane::Layers);
                }
                ui.separator();
                ui.weak("图层");
            });
            group(ui, 154.0, |ui| {
                ui.add_enabled_ui(enabled, |ui| {
                    color_ui(ui, document, selected, tools, &mut output)
                });
                if button(ui, "PROPERTIES", "详细特性").clicked() {
                    workspace.reveal(Pane::Properties);
                }
                ui.separator();
                ui.weak("特性");
            });
            group(ui, 122.0, |ui| {
                command_grid(
                    ui,
                    "measure_grid",
                    MEASURE,
                    1,
                    enabled,
                    selected,
                    &mut output,
                );
                utility_menu(ui, workspace, enabled, selected, &mut output);
            });
        });
    }
    output
}

fn group(ui: &mut egui::Ui, width: f32, contents: impl FnOnce(&mut egui::Ui)) {
    ui.group(|ui| {
        ui.vertical(|ui| {
            ui.set_min_width(width);
            ui.set_max_width(width);
            ui.set_min_height(104.0);
            contents(ui);
        });
    });
}

fn available(command: &str, enabled: bool, selected: &BTreeSet<u32>) -> bool {
    enabled && (!matches!(command, "BREAK" | "JOIN" | "CLOSE" | "REVERSE") || !selected.is_empty())
}
fn issue(command: &str, output: &mut Output) {
    output.command = Some(command.into());
}
fn command_grid(
    ui: &mut egui::Ui,
    id: &str,
    commands: &[(&str, &str)],
    columns: usize,
    enabled: bool,
    selected: &BTreeSet<u32>,
    output: &mut Output,
) {
    egui::Grid::new(id).spacing([3.0, 3.0]).show(ui, |ui| {
        for (index, &(command, label)) in commands.iter().enumerate() {
            ui.add_enabled_ui(available(command, enabled, selected), |ui| {
                if button(ui, command, label).clicked() {
                    issue(command, output);
                }
            });
            if (index + 1) % columns == 0 {
                ui.end_row();
            }
        }
    });
}
fn menu(
    ui: &mut egui::Ui,
    label: &str,
    commands: &[(&str, &str)],
    enabled: bool,
    selected: &BTreeSet<u32>,
    output: &mut Output,
) {
    ui.menu_button(label, |ui| {
        for &(command, label) in commands {
            ui.add_enabled_ui(available(command, enabled, selected), |ui| {
                if button(ui, command, label).clicked() {
                    issue(command, output);
                    ui.close();
                }
            });
        }
    });
}
fn utility_menu(
    ui: &mut egui::Ui,
    workspace: &mut Workspace,
    enabled: bool,
    selected: &BTreeSet<u32>,
    output: &mut Output,
) {
    ui.menu_button("实用工具 ▾", |ui| {
        for &(command, label) in MEASURE {
            ui.add_enabled_ui(enabled, |ui| {
                if button(ui, command, label).clicked() {
                    issue(command, output);
                    ui.close();
                }
            });
        }
        if ui
            .add_enabled(
                !selected.is_empty(),
                egui::Button::new("所选曲线长度 / 半径"),
            )
            .clicked()
        {
            workspace.reveal(Pane::Properties);
            ui.close();
        }
        if button(ui, "SETTINGS", "绘图设置与图纸检查").clicked() {
            workspace.reveal(Pane::Tools);
            ui.close();
        }
        menu(
            ui,
            "文字与标注",
            &[("TEXT", "单行文字"), ("DIMALIGNED", "对齐标注")],
            enabled,
            selected,
            output,
        );
    });
}

fn layer_ui(
    ui: &mut egui::Ui,
    document: Option<&DrawingDocument>,
    selected: &BTreeSet<u32>,
    output: &mut Output,
) {
    let Some(doc) = document else {
        ui.weak("请新建或打开图纸");
        return;
    };
    let first = doc.entities.iter().find(|e| selected.contains(&e.id.0));
    let id = first.map(|e| e.layer_id).or(doc.current_layer);
    let mixed = first.is_some_and(|first| {
        doc.entities
            .iter()
            .any(|e| selected.contains(&e.id.0) && e.layer_id != first.layer_id)
    });
    egui::ComboBox::from_id_salt("ribbon_layer")
        .width(172.0)
        .truncate()
        .selected_text(if mixed {
            "*多个图层*"
        } else {
            id.and_then(|id| doc.layer(id))
                .map(|l| l.name.as_str())
                .unwrap_or("当前图层")
        })
        .show_ui(ui, |ui| {
            for layer in &doc.layers {
                ui.horizontal(|ui| {
                    let rgb = super::display::color_to_rgb(layer.color).map(|c| (c * 255.0) as u8);
                    ui.colored_label(egui::Color32::from_rgb(rgb[0], rgb[1], rgb[2]), "■");
                    let label = format!(
                        "{}{}{}",
                        layer.name,
                        if layer.visible { "" } else { " [隐藏]" },
                        if layer.locked { " [锁定]" } else { "" }
                    );
                    if ui
                        .add_enabled(
                            layer.visible && !layer.locked,
                            egui::Button::selectable(!mixed && id == Some(layer.id), label),
                        )
                        .clicked()
                    {
                        output.edit = Some(if selected.is_empty() {
                            EditAction::Layer(LayerEdit::Current(layer.id))
                        } else {
                            EditAction::Apply(DrawingEdit::SetLayer(layer.id))
                        });
                        ui.close();
                    }
                });
            }
        })
        .response
        .on_hover_text(if selected.is_empty() {
            "设置新图元的当前图层"
        } else {
            "修改所选图元的所属图层"
        });
    if let Some(layer) = id.and_then(|id| doc.layer(id)).filter(|_| !mixed) {
        ui.horizontal(|ui| {
            let mut visible = layer.visible;
            let mut locked = layer.locked;
            if ui.checkbox(&mut visible, "显示").changed() {
                output.edit = Some(EditAction::Layer(LayerEdit::Visible {
                    id: layer.id,
                    visible,
                }));
            }
            if ui.checkbox(&mut locked, "锁定").changed() {
                output.edit = Some(EditAction::Layer(LayerEdit::Locked {
                    id: layer.id,
                    locked,
                }));
            }
        });
    } else {
        ui.weak("多个图层，请打开图层管理");
    }
}
fn color_ui(
    ui: &mut egui::Ui,
    document: Option<&DrawingDocument>,
    selected: &BTreeSet<u32>,
    tools: &mut ToolOptions,
    output: &mut Output,
) {
    let Some(doc) = document else {
        return;
    };
    let first = doc.entities.iter().find(|e| selected.contains(&e.id.0));
    let mut color = first.map(|e| e.color).unwrap_or(tools.color);
    let mixed = first.is_some_and(|first| {
        doc.entities
            .iter()
            .any(|e| selected.contains(&e.id.0) && e.color != first.color)
    });
    let label = if mixed {
        "*多个颜色*".into()
    } else {
        match color {
            Color::ByLayer | Color::None => "随层".into(),
            Color::ByBlock => "随块".into(),
            Color::Aci(i) => format!("ACI {i}"),
            Color::Rgb { r, g, b } => format!("#{r:02X}{g:02X}{b:02X}"),
        }
    };
    let editable = !doc
        .entities
        .iter()
        .any(|e| selected.contains(&e.id.0) && doc.layer(e.layer_id).is_some_and(|l| l.locked));
    let mut changed = false;
    ui.add_enabled_ui(editable, |ui| {
        egui::ComboBox::from_id_salt("ribbon_color")
            .width(134.0)
            .truncate()
            .selected_text(label)
            .show_ui(ui, |ui| {
                for (value, label) in [(Color::ByLayer, "随层"), (Color::ByBlock, "随块")] {
                    if ui
                        .selectable_label(!mixed && color == value, label)
                        .clicked()
                    {
                        color = value;
                        changed = true;
                        ui.close();
                    }
                }
                for (rgb, name) in [
                    ([255, 0, 0], "红"),
                    ([255, 255, 0], "黄"),
                    ([0, 255, 0], "绿"),
                    ([0, 255, 255], "青"),
                    ([0, 100, 255], "蓝"),
                    ([255, 0, 255], "洋红"),
                    ([255, 255, 255], "白"),
                ] {
                    let [r, g, b] = rgb;
                    if ui
                        .selectable_label(
                            !mixed && color == Color::Rgb { r, g, b },
                            egui::RichText::new(format!("■ {name}"))
                                .color(egui::Color32::from_rgb(r, g, b)),
                        )
                        .clicked()
                    {
                        color = Color::Rgb { r, g, b };
                        changed = true;
                        ui.close();
                    }
                }
            });
        ui.horizontal(|ui| {
            ui.weak(if selected.is_empty() {
                "新图元"
            } else {
                "所选图元"
            });
            let effective = if matches!(color, Color::ByLayer | Color::None) {
                first
                    .map(|e| e.layer_id)
                    .or(doc.current_layer)
                    .and_then(|id| doc.layer(id))
                    .map(|l| l.color)
                    .unwrap_or(color)
            } else {
                color
            };
            let [r, g, b] = super::display::color_to_rgb(effective).map(|c| (c * 255.0) as u8);
            ui.colored_label(
                if mixed {
                    egui::Color32::GRAY
                } else {
                    egui::Color32::from_rgb(r, g, b)
                },
                "■",
            )
            .on_hover_text("自定义 RGB 和 ACI 索引色可在详细特性中设置");
        });
    });
    if changed {
        if selected.is_empty() {
            tools.set_color(color);
        } else {
            output.edit = Some(EditAction::Apply(DrawingEdit::SetColor(color)));
        }
    }
}

/// Vector icons stay sharp at any display scale and do not depend on glyph fonts.
fn button(ui: &mut egui::Ui, command: &str, label: &str) -> egui::Response {
    let width = (label.chars().count() as f32 * 13.0 + 30.0).max(76.0);
    let (rect, response) = ui.allocate_exact_size(egui::vec2(width, 23.0), egui::Sense::click());
    let visuals = ui.style().interact(&response);
    if response.hovered() || response.is_pointer_button_down_on() {
        ui.painter().rect(
            rect,
            3.0,
            visuals.weak_bg_fill,
            visuals.bg_stroke,
            egui::StrokeKind::Inside,
        );
    }
    let ink = if ui.is_enabled() {
        egui::Color32::from_rgb(105, 194, 237)
    } else {
        visuals.text_color()
    };
    icon(
        ui.painter(),
        egui::Rect::from_min_size(rect.min + egui::vec2(3.0, 3.0), egui::vec2(17.0, 17.0)),
        command,
        ink,
    );
    ui.painter().text(
        rect.min + egui::vec2(25.0, 11.5),
        egui::Align2::LEFT_CENTER,
        label,
        egui::FontId::proportional(13.0),
        visuals.text_color(),
    );
    response.widget_info(|| {
        egui::WidgetInfo::labeled(egui::WidgetType::Button, ui.is_enabled(), label)
    });
    response.on_hover_text(match command {
        "TRIM" => "修剪 · TRIM\n选择边界并确认，再点击要去除的区间；预览后确认。".into(),
        "EXTEND" => "延伸 · EXTEND\n选择边界并确认，再点击待延伸端；只延到有效边界。".into(),
        "OFFSET" => {
            "偏移 · OFFSET\n输入距离，选择对象，再点击偏移侧；预览后确认，保留原件。".into()
        }
        "FILLET" | "CHAMFER" => {
            format!(
                "{label} · {command}\n输入半径/距离，依次点击两条直线要保留的一侧；预览后确认。"
            )
        }
        _ => format!("{label} · {command}"),
    })
}
fn icon(painter: &egui::Painter, rect: egui::Rect, command: &str, color: egui::Color32) {
    let stroke = egui::Stroke::new(1.5, color);
    let p = |x: f32, y: f32| rect.min + egui::vec2(x, y);
    let line = |a, b| {
        painter.line_segment([a, b], stroke);
    };
    let path = |points: &[(f32, f32)]| {
        painter.add(egui::Shape::line(
            points.iter().map(|&(x, y)| p(x, y)).collect(),
            stroke,
        ));
    };
    match command {
        "LINE" => {
            line(p(2., 15.), p(15., 2.));
            for c in [p(2., 15.), p(15., 2.)] {
                painter.rect_filled(
                    egui::Rect::from_center_size(c, egui::vec2(3., 3.)),
                    0.,
                    color,
                );
            }
        }
        "PLINE" | "SPLINE" => path(&[(1., 14.), (5., 3.), (11., 11.), (16., 2.)]),
        "CIRCLE" | "CIRCLED" | "CIRCLE2P" | "CIRCLE3P" | "HOLE" => {
            painter.circle_stroke(rect.center(), 7., stroke);
            painter.circle_filled(rect.center(), 1.3, color);
        }
        "ELLIPSE" => {
            let points = (0..=32)
                .map(|i| {
                    let t = i as f32 * std::f32::consts::TAU / 32.;
                    p(8.5 + 8. * t.cos(), 8.5 + 4.5 * t.sin())
                })
                .collect();
            painter.add(egui::Shape::line(points, stroke));
        }
        "ARC" | "ARCCE" | "ARCCA" | "FILLET" | "ROTATE" | "ANGLE" => {
            let points = (0..=20)
                .map(|i| {
                    let t = i as f32 * std::f32::consts::PI / 20.;
                    p(8.5 + 7. * t.cos(), 12. - 7. * t.sin())
                })
                .collect();
            painter.add(egui::Shape::line(points, stroke));
            if command == "ANGLE" {
                path(&[(16., 12.), (2., 12.), (8., 1.)]);
            }
        }
        "RECTANGLE" | "SCALE" | "SLOT" => {
            painter.rect_stroke(
                rect.shrink(2.),
                if command == "SLOT" { 6. } else { 0. },
                stroke,
                egui::StrokeKind::Inside,
            );
            if command == "SCALE" {
                line(p(2., 15.), p(15., 2.));
            }
        }
        "MOVE" => {
            path(&[(8., 1.), (5., 4.), (8., 1.), (11., 4.)]);
            line(p(8., 1.), p(8., 16.));
            path(&[(1., 8.), (4., 5.), (1., 8.), (4., 11.)]);
            line(p(1., 8.), p(16., 8.));
        }
        "COPY" => {
            for (x, y) in [(1., 1.), (6., 6.)] {
                painter.rect_stroke(
                    egui::Rect::from_min_size(p(x, y), egui::vec2(10., 10.)),
                    0.,
                    stroke,
                    egui::StrokeKind::Inside,
                );
            }
        }
        "MIRROR" => {
            line(p(8.5, 0.), p(8.5, 17.));
            path(&[(5., 3.), (1., 13.), (5., 13.), (5., 3.)]);
            path(&[(12., 3.), (16., 13.), (12., 13.), (12., 3.)]);
        }
        "ERASE" | "TRIM" | "BREAK" => {
            line(p(2., 2.), p(15., 15.));
            line(p(2., 15.), p(15., 2.));
            if command != "ERASE" {
                line(p(1., 12.), p(16., 12.));
            }
        }
        "EXTEND" => {
            line(p(1., 1.), p(1., 16.));
            line(p(1., 9.), p(16., 9.));
            path(&[(5., 5.), (1., 9.), (5., 13.)]);
        }
        "OFFSET" => {
            line(p(1., 11.), p(12., 0.));
            line(p(5., 17.), p(17., 5.));
        }
        "CHAMFER" => path(&[(1., 16.), (1., 6.), (6., 1.), (16., 1.)]),
        "ARRAYRECT" | "ARRAYPOLAR" => {
            for x in [1., 10.] {
                for y in [1., 10.] {
                    painter.rect_stroke(
                        egui::Rect::from_min_size(p(x, y), egui::vec2(6., 6.)),
                        0.,
                        stroke,
                        egui::StrokeKind::Inside,
                    );
                }
            }
        }
        "DIST" => {
            line(p(1., 9.), p(16., 9.));
            line(p(2., 3.), p(2., 15.));
            line(p(15., 3.), p(15., 15.));
            path(&[(5., 6.), (2., 9.), (5., 12.)]);
            path(&[(12., 6.), (15., 9.), (12., 12.)]);
        }
        "ID" => {
            line(p(1., 8.), p(16., 8.));
            line(p(8., 1.), p(8., 16.));
            painter.circle_stroke(rect.center(), 4., stroke);
        }
        "LAYERS" => {
            for y in [2., 7., 12.] {
                path(&[
                    (1., y + 2.),
                    (8., y - 1.),
                    (16., y + 2.),
                    (8., y + 5.),
                    (1., y + 2.),
                ]);
            }
        }
        _ => {
            painter.rect_stroke(rect.shrink(1.), 1., stroke, egui::StrokeKind::Inside);
            for y in [5., 9., 13.] {
                line(p(4., y), p(13., y));
            }
        }
    }
}
