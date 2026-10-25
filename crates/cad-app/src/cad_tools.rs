//! Parameters for concrete drawing and batch-transform commands.
use bevy_egui::egui;
use cad_core::{
    geometry_edit::AffineTransform,
    model::{Color, LayerId, WorkPlane},
    DrawingDocument,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransformKind {
    Rotate,
    Scale,
    Mirror,
    RectArray,
    PolarArray,
}
impl TransformKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Rotate => "旋转",
            Self::Scale => "统一缩放",
            Self::Mirror => "镜像",
            Self::RectArray => "矩形阵列",
            Self::PolarArray => "环形阵列",
        }
    }
    pub fn command(self) -> &'static str {
        match self {
            Self::Rotate => "ROTATE",
            Self::Scale => "SCALE",
            Self::Mirror => "MIRROR",
            Self::RectArray => "ARRAYRECT",
            Self::PolarArray => "ARRAYPOLAR",
        }
    }
}
#[derive(Clone)]
pub struct ToolOptions {
    pub layer: Option<LayerId>,
    pub color: Color,
    pub degree: u32,
    aci: u8,
    rgb: [u8; 3],
    pub angle: f64,
    pub scale: f64,
    pub custom_axis: bool,
    pub axis: [f64; 3],
    pub mirror_plane: bool,
    pub copies: bool,
    pub rows: u32,
    pub columns: u32,
    pub spacing: [f64; 2],
    pub count: u32,
    pub sweep: f64,
}
impl Default for ToolOptions {
    fn default() -> Self {
        Self {
            layer: None,
            color: Color::ByLayer,
            degree: 3,
            aci: 7,
            rgb: [255; 3],
            angle: 90.0,
            scale: 2.0,
            custom_axis: false,
            axis: [0.0, 0.0, 1.0],
            mirror_plane: false,
            copies: false,
            rows: 2,
            columns: 3,
            spacing: [10.0; 2],
            count: 6,
            sweep: 360.0,
        }
    }
}
impl ToolOptions {
    pub fn style_ui(&mut self, ui: &mut egui::Ui, document: &DrawingDocument) {
        self.layer = document.current_layer;
        egui::CollapsingHeader::new("新图元属性").show(ui, |ui| {
            ui.label(format!(
                "当前图层：{}",
                self.layer
                    .and_then(|id| document.layer(id))
                    .map(|l| l.name.as_str())
                    .unwrap_or("无")
            ));
            ui.small("在图层管理中切换当前图层。");
            if self
                .layer
                .and_then(|id| document.layer(id))
                .is_some_and(|l| l.locked || !l.visible)
            {
                ui.colored_label(
                    egui::Color32::LIGHT_RED,
                    "当前层已锁定或隐藏，绘图前请切换或解锁并显示。",
                );
            }
            egui::ComboBox::from_id_salt("drawing_color")
                .selected_text(match self.color {
                    Color::ByLayer => "随层",
                    Color::ByBlock => "随块",
                    Color::Aci(_) => "ACI",
                    _ => "RGB",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.color, Color::ByLayer, "随层");
                    ui.selectable_value(&mut self.color, Color::ByBlock, "随块");
                    ui.selectable_value(&mut self.color, Color::Aci(self.aci), "ACI");
                    let [r, g, b] = self.rgb;
                    ui.selectable_value(&mut self.color, Color::Rgb { r, g, b }, "RGB");
                });
            match self.color {
                Color::Aci(_) => {
                    ui.add(egui::DragValue::new(&mut self.aci).range(1..=255));
                    self.color = Color::Aci(self.aci);
                }
                Color::Rgb { .. } => {
                    ui.color_edit_button_srgb(&mut self.rgb);
                    let [r, g, b] = self.rgb;
                    self.color = Color::Rgb { r, g, b };
                }
                _ => {}
            }
            ui.horizontal(|ui| {
                ui.label("样条次数");
                ui.add(egui::DragValue::new(&mut self.degree).range(1..=5));
            });
        });
    }
    pub fn transform_ui(&mut self, ui: &mut egui::Ui, kind: TransformKind) {
        ui.strong(kind.label());
        match kind {
            TransformKind::Rotate => {
                ui.horizontal(|ui| {
                    ui.label("角度 °");
                    ui.add(egui::DragValue::new(&mut self.angle).speed(1.0));
                });
            }
            TransformKind::Scale => {
                ui.horizontal(|ui| {
                    ui.label("比例");
                    ui.add(egui::DragValue::new(&mut self.scale).speed(0.05));
                });
            }
            TransformKind::Mirror => {
                ui.checkbox(
                    &mut self.mirror_plane,
                    "关于工作平面镜像（关闭时关于平面内 U 轴）",
                );
            }
            TransformKind::RectArray => {
                ui.horizontal(|ui| {
                    ui.label("行 / 列（含原件）");
                    ui.add(egui::DragValue::new(&mut self.rows).range(1..=1000));
                    ui.add(egui::DragValue::new(&mut self.columns).range(1..=1000));
                });
                for (label, value) in ["U 列间距 mm", "V 行间距 mm"]
                    .into_iter()
                    .zip(&mut self.spacing)
                {
                    ui.horizontal(|ui| {
                        ui.label(label);
                        ui.add(egui::DragValue::new(value).speed(0.1));
                    });
                }
            }
            TransformKind::PolarArray => {
                ui.horizontal(|ui| {
                    ui.label("数量（含原件）");
                    ui.add(egui::DragValue::new(&mut self.count).range(1..=1000));
                });
                ui.horizontal(|ui| {
                    ui.label("分布总角度 °");
                    ui.add(egui::DragValue::new(&mut self.sweep).speed(1.0));
                });
                ui.small("整圆不重复终点；部分角度包含首末项，副本随轴旋转。");
            }
        }
        if matches!(
            kind,
            TransformKind::Rotate | TransformKind::Mirror | TransformKind::PolarArray
        ) {
            ui.checkbox(&mut self.custom_axis, "自定义三维轴 / 镜像平面法向");
            if self.custom_axis {
                for (label, value) in ["X", "Y", "Z"].into_iter().zip(&mut self.axis) {
                    ui.horizontal(|ui| {
                        ui.label(label);
                        ui.add(egui::DragValue::new(value).speed(0.1));
                    });
                }
            } else {
                ui.small("旋转/环阵轴为工作平面法向；镜像基点用于确定平面或轴线的位置。");
            }
        }
        if !matches!(kind, TransformKind::RectArray | TransformKind::PolarArray) {
            ui.checkbox(&mut self.copies, "保留原件并创建副本");
        }
    }
    pub fn transforms(
        &self,
        kind: TransformKind,
        base: [f64; 3],
        plane: WorkPlane,
        selected: usize,
    ) -> Result<(Vec<AffineTransform>, bool), String> {
        let n = plane.normal();
        let axis = if self.custom_axis {
            self.axis
        } else {
            [n.x, n.y, n.z]
        };
        match kind {
            TransformKind::Rotate => Ok((
                vec![AffineTransform::rotation(base, axis, self.angle)?],
                self.copies,
            )),
            TransformKind::Scale => Ok((
                vec![AffineTransform::scaling(base, self.scale)?],
                self.copies,
            )),
            TransformKind::Mirror => {
                let v = plane.y_axis;
                let normal = if self.custom_axis {
                    self.axis
                } else if self.mirror_plane {
                    [n.x, n.y, n.z]
                } else {
                    [v.x, v.y, v.z]
                };
                Ok((vec![AffineTransform::mirror(base, normal)?], self.copies))
            }
            TransformKind::RectArray => {
                let count = self.rows as usize * self.columns as usize;
                if (self.rows > 1 && (!self.spacing[1].is_finite() || self.spacing[1].abs() < 1e-9))
                    || (self.columns > 1
                        && (!self.spacing[0].is_finite() || self.spacing[0].abs() < 1e-9))
                {
                    return Err("有多行/多列时，相应间距必须为非零有限数值".into());
                }
                if count.saturating_sub(1).saturating_mul(selected) > 50_000 {
                    return Err("本次数量超过 50000 个新增图元，请减少行列数".into());
                }
                let mut result = Vec::new();
                for row in 0..self.rows {
                    for col in 0..self.columns {
                        if row == 0 && col == 0 {
                            continue;
                        }
                        let d = plane.x_axis * (col as f64 * self.spacing[0])
                            + plane.y_axis * (row as f64 * self.spacing[1]);
                        result.push(AffineTransform::translation([d.x, d.y, d.z])?);
                    }
                }
                Ok((result, true))
            }
            TransformKind::PolarArray => {
                if !self.sweep.is_finite() || self.sweep.abs() < 1e-9 || self.sweep.abs() > 360.0 {
                    return Err("环阵分布角须在 −360° 到 360° 内且非零".into());
                }
                if (self.count as usize)
                    .saturating_sub(1)
                    .saturating_mul(selected)
                    > 50_000
                {
                    return Err("本次数量超过 50000 个新增图元，请减少数量".into());
                }
                let mut result = Vec::new();
                for i in 1..self.count {
                    result.push(AffineTransform::rotation(
                        base,
                        axis,
                        self.sweep * i as f64
                            / if (self.sweep.abs() - 360.0).abs() < 1e-9 {
                                self.count as f64
                            } else {
                                (self.count - 1) as f64
                            },
                    )?);
                }
                Ok((result, true))
            }
        }
    }
}
