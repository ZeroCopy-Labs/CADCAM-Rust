use bevy_egui::egui;
use cad_core::model::{Geometry, Point3, WorkPlane};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Coordinate,
    Distance,
    Angle,
    Text,
    Dimension,
    Hole,
    Slot,
    CopyBase,
}
impl Kind {
    pub const ALL: [Self; 8] = [
        Self::Coordinate,
        Self::Distance,
        Self::Angle,
        Self::Text,
        Self::Dimension,
        Self::Hole,
        Self::Slot,
        Self::CopyBase,
    ];
    pub fn command(self) -> &'static str {
        match self {
            Self::Coordinate => "ID",
            Self::Distance => "DIST",
            Self::Angle => "ANGLE",
            Self::Text => "TEXT",
            Self::Dimension => "DIMALIGNED",
            Self::Hole => "HOLE",
            Self::Slot => "SLOT",
            Self::CopyBase => "COPYBASE",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Coordinate => "坐标",
            Self::Distance => "空间距离",
            Self::Angle => "三点角度（第二点为顶点）",
            Self::Text => "单行文字",
            Self::Dimension => "对齐长度标注",
            Self::Hole => "圆孔",
            Self::Slot => "长圆孔（两端圆心）",
            Self::CopyBase => "指定复制基点",
        }
    }
    pub fn required(self) -> usize {
        match self {
            Self::Distance | Self::Slot => 2,
            Self::Angle | Self::Dimension => 3,
            _ => 1,
        }
    }
    pub fn measuring(self) -> bool {
        matches!(self, Self::Coordinate | Self::Distance | Self::Angle)
    }
}
pub struct Options {
    pub content: String,
    pub height: f64,
    pub rotation: f64,
    pub diameter: f64,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            content: "文字".into(),
            height: 2.5,
            rotation: 0.0,
            diameter: 5.0,
        }
    }
}
impl Options {
    pub fn ui(&mut self, ui: &mut egui::Ui, kind: Kind) {
        if kind == Kind::Text {
            ui.text_edit_singleline(&mut self.content);
            ui.horizontal(|ui| {
                ui.label("文字方向 °");
                ui.add(egui::DragValue::new(&mut self.rotation));
            });
        }
        if matches!(kind, Kind::Text | Kind::Dimension) {
            ui.horizontal(|ui| {
                ui.label("字高 mm");
                ui.add(egui::DragValue::new(&mut self.height).speed(0.1));
            });
        }
        if matches!(kind, Kind::Hole | Kind::Slot) {
            ui.horizontal(|ui| {
                ui.label("孔径 / 孔宽 mm");
                ui.add(egui::DragValue::new(&mut self.diameter).speed(0.1));
            });
        }
        if kind == Kind::Dimension {
            ui.small("取两端点后取标注线位置；非关联线段+文字，使用专用参考层。");
        }
    }
    pub fn geometry(
        &self,
        kind: Kind,
        p: &[Point3],
        plane: WorkPlane,
    ) -> Result<Vec<Geometry>, String> {
        if p.len() != kind.required() {
            return Err("请完成取点".into());
        }
        match kind {
            Kind::Text => Ok(vec![cad_core::drafting::text(
                p[0],
                self.content.clone(),
                self.height,
                self.rotation,
                plane,
            )?]),
            Kind::Dimension => {
                cad_core::drafting::aligned_dimension(p[0], p[1], p[2], self.height, plane)
            }
            Kind::Hole => cad_core::drafting::hole(p[0], self.diameter, plane),
            Kind::Slot => cad_core::drafting::slot(p[0], p[1], self.diameter, plane),
            _ => Ok(Vec::new()),
        }
    }
}
pub fn result(kind: Kind, p: &[Point3], plane: WorkPlane) -> Result<String, String> {
    if p.len() != kind.required() {
        return Err("请完成取点".into());
    }
    match kind {
        Kind::Coordinate => {
            let q = plane.to_local(p[0]);
            Ok(format!(
                "WCS ({:.6}, {:.6}, {:.6}) mm；平面 U/V/N ({:.6}, {:.6}, {:.6}) mm",
                p[0].x, p[0].y, p[0].z, q[0], q[1], q[2]
            ))
        }
        Kind::Distance => Ok(format!(
            "空间距离 {:.6} mm；ΔX {:.6}  ΔY {:.6}  ΔZ {:.6} mm",
            cad_core::curves::distance(p[0], p[1]),
            p[1].x - p[0].x,
            p[1].y - p[0].y,
            p[1].z - p[0].z
        )),
        Kind::Angle => Ok(format!(
            "三点夹角 {:.6}°（0–180°）",
            cad_core::drafting::angle(p[0], p[1], p[2])?
        )),
        _ => Ok(String::new()),
    }
}
#[derive(Default)]
pub struct Measurements {
    key: Option<(u64, std::collections::BTreeSet<u32>)>,
    rows: Vec<String>,
}
impl Measurements {
    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        doc: &cad_core::DrawingDocument,
        revision: u64,
        ids: &std::collections::BTreeSet<u32>,
    ) {
        if ids.is_empty() {
            return;
        }
        if self
            .key
            .as_ref()
            .is_none_or(|(r, keys)| *r != revision || keys != ids)
        {
            self.rows.clear();
            let mut sum = 0.0;
            let mut approx = false;
            let mut failed = 0;
            for id in ids {
                if let Some(e) = doc.entity(cad_core::EntityId(*id)) {
                    if let Some(r) = cad_core::properties::radius(&e.geometry) {
                        self.rows
                            .push(format!("#{} 半径 {:.6} mm；直径 {:.6} mm", id, r, r * 2.0));
                    }
                    if matches!(e.geometry, Geometry::Text(_)) {
                        continue;
                    }
                    match cad_core::diagnostics::length(&e.geometry) {
                        Ok((len, a)) => {
                            sum += len;
                            approx |= a;
                        }
                        Err(_) => failed += 1,
                    }
                }
            }
            self.rows.push(format!(
                "所选曲线总长 {}{sum:.6} mm",
                if approx { "约 " } else { "" }
            ));
            if approx {
                self.rows
                    .push("椭圆/样条按弦差 0.05 mm、转角 5° 估长；不是长度误差保证。".into());
            }
            if failed > 0 {
                self.rows.push(format!("{failed} 个对象未计入长度"));
            }
            self.key = Some((revision, ids.clone()));
        }
        egui::CollapsingHeader::new("所选图元测量").default_open(true).show(ui, |ui| {
            for row in &self.rows {
                ui.label(row);
            }
        });
    }
}
