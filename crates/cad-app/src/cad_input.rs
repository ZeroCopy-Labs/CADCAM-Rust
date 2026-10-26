//! Cursor input edits the same draft as the command line. Locks only constrain previews.
use bevy_egui::egui;
use cad_core::model::{Point3, WorkPlane};

use super::cad_command::CommandState;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Coordinates,
    Polar,
    Radius,
    Diameter,
    Angle,
    Scale,
    Distance,
    ArrayRect,
    Count,
    ReferenceAngle,
    ReferenceLength,
    TargetAngle,
    TargetLength,
    RectangleLength,
    RectangleWidth,
    RectangleAngle,
    ArcSweep,
}
#[derive(Clone, Copy)]
pub struct Spec {
    pub kind: Kind,
    pub reference: Option<[f64; 3]>,
    pub plane: WorkPlane,
    pub point: Option<[f64; 3]>,
    pub value: f64,
    pub secondary: f64,
}
impl Spec {
    fn count(self) -> usize {
        match self.kind {
            Kind::Coordinates => 3,
            Kind::Polar | Kind::ArrayRect => 2,
            _ => 1,
        }
    }
    fn labels(self) -> &'static [&'static str] {
        match self.kind {
            Kind::Coordinates => &["X", "Y", "Z"],
            Kind::Polar => &["长度", "角度 °"],
            Kind::Radius => &["半径"],
            Kind::Diameter => &["直径"],
            Kind::Angle => &["角度 °"],
            Kind::Scale => &["比例"],
            Kind::Distance => &["距离 mm"],
            Kind::ArrayRect => &["列数", "行数"],
            Kind::Count => &["数量"],
            Kind::ReferenceAngle => &["原角度 °"],
            Kind::ReferenceLength => &["原长度 mm"],
            Kind::TargetAngle => &["新方向 °"],
            Kind::TargetLength => &["新长度 mm"],
            Kind::RectangleLength => &["矩形长度 mm"],
            Kind::RectangleWidth => &["矩形宽度 mm"],
            Kind::RectangleAngle => &["矩形旋转 °"],
            Kind::ArcSweep => &["圆弧扫角 °"],
        }
    }
    fn values(self) -> [f64; 3] {
        let point = self.point.unwrap_or(self.reference.unwrap_or([0.; 3]));
        let base = self.reference.unwrap_or([0.; 3]);
        match self.kind {
            Kind::Radius | Kind::Diameter if self.reference.is_none() => [self.value, 0., 0.],
            Kind::ArrayRect => [self.value, self.secondary, 0.],
            Kind::Coordinates => std::array::from_fn(|i| point[i] - base[i]),
            Kind::Polar | Kind::Radius | Kind::Diameter => {
                let a = self.plane.to_local(Point3::new(base[0], base[1], base[2]));
                let b = self
                    .plane
                    .to_local(Point3::new(point[0], point[1], point[2]));
                let u = b[0] - a[0];
                let v = b[1] - a[1];
                [u.hypot(v), v.atan2(u).to_degrees(), 0.]
            }
            _ => [self.value, 0., 0.],
        }
    }
}

pub struct DynamicInput {
    pub enabled: bool,
    pub at_cursor: bool,
    literal: bool,
    coordinate_mode: bool,
    locked: [bool; 3],
    field: usize,
    focus: bool,
    candidate: usize,
    enter: bool,
    rect: Option<egui::Rect>,
    cursor: Option<egui::Pos2>,
}
impl Default for DynamicInput {
    fn default() -> Self {
        Self {
            enabled: true,
            at_cursor: true,
            literal: false,
            coordinate_mode: false,
            locked: [false; 3],
            field: 0,
            focus: false,
            candidate: 0,
            enter: false,
            rect: None,
            cursor: None,
        }
    }
}

pub const COMMANDS: &[(&str, &str, &str)] = &[
    ("LINE", "L", "直线"),
    ("PLINE", "PL", "多段线"),
    ("RECTANGLE", "REC", "矩形"),
    ("CIRCLE", "C", "圆"),
    ("CIRCLED", "", "圆心/直径圆"),
    ("CIRCLE2P", "", "两点直径圆"),
    ("CIRCLE3P", "", "三点圆"),
    ("ARC", "A", "圆弧"),
    ("ARCCE", "", "圆心/起点/终点方向圆弧"),
    ("ARCCA", "", "圆心/起点/扫角圆弧"),
    ("ELLIPSE", "EL", "椭圆"),
    ("SPLINE", "SPL", "样条"),
    ("MOVE", "M", "移动"),
    ("COPY", "CO", "复制"),
    ("ROTATE", "RO", "旋转"),
    ("SCALE", "SC", "缩放"),
    ("MIRROR", "MI", "镜像"),
    ("ERASE", "E", "删除所选"),
    ("OFFSET", "O", "偏移"),
    ("TRIM", "TR", "修剪"),
    ("EXTEND", "EX", "延伸"),
    ("FILLET", "F", "圆角"),
    ("CHAMFER", "CHA", "倒角"),
    ("BREAK", "BR", "打断"),
    ("JOIN", "J", "合并"),
    ("CLOSE", "", "闭合"),
    ("REVERSE", "", "反向"),
    ("ARRAYRECT", "", "矩形阵列"),
    ("ARRAYPOLAR", "", "环形阵列"),
    ("DIST", "DI", "距离"),
    ("ANGLE", "ANG", "角度"),
    ("ID", "", "坐标"),
    ("LAYER", "LA", "图层"),
    ("PROPERTIES", "PR", "特性"),
    ("SETTINGS", "", "绘图设置"),
    ("UNDO", "U", "撤销"),
    ("REDO", "", "重做"),
    ("UCS", "", "工作平面"),
    ("COPYBASE", "", "复制基点"),
    ("PASTE", "", "粘贴"),
    ("HOLE", "", "圆孔"),
    ("SLOT", "", "长圆孔"),
    ("TEXT", "", "单行文字"),
    ("DIMALIGNED", "", "对齐标注"),
];
pub fn canonical(name: &str) -> &str {
    if name.is_empty() {
        return name;
    }
    if name == "CP" {
        return "COPY";
    }
    if matches!(name, "RECT" | "RECTANG") {
        return "RECTANGLE";
    }
    COMMANDS
        .iter()
        .find(|(full, alias, _)| *full == name || !alias.is_empty() && *alias == name)
        .or_else(|| {
            COMMANDS.iter().find(|(full, alias, _)| {
                full.starts_with(name) || !alias.is_empty() && alias.starts_with(name)
            })
        })
        .map(|(full, _, _)| *full)
        .unwrap_or(name)
}
fn candidates(text: &str) -> Vec<(&'static str, &'static str, &'static str)> {
    let text = text.trim().to_uppercase();
    if text.is_empty() {
        return vec![];
    }
    let mut matches: Vec<_> = COMMANDS
        .iter()
        .copied()
        .filter(|(full, alias, _)| {
            full.starts_with(&text) || !alias.is_empty() && alias.starts_with(&text)
        })
        .collect();
    matches.sort_by_key(|(full, alias, _)| !(text == *full || text == *alias));
    matches.truncate(5);
    matches
}
fn raw_id() -> egui::Id {
    egui::Id::new("cad_cursor_input")
}
fn field_id(i: usize) -> egui::Id {
    egui::Id::new(("cad_cursor_field", i))
}
pub fn owns_focus(ctx: &egui::Context) -> bool {
    ctx.memory(|m| m.has_focus(raw_id()) || (0..3).any(|i| m.has_focus(field_id(i))))
}
pub fn confirm_space(ctx: &egui::Context) -> bool {
    if !ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Space)) {
        return false;
    }
    ctx.input_mut(|i| {
        i.events
            .retain(|event| !matches!(event,egui::Event::Text(text) if text==" "))
    });
    true
}
impl DynamicInput {
    pub fn suspend(&mut self) {
        self.enter = false;
        self.focus = false;
        self.rect = None;
    }
    pub fn reset_fields(&mut self) {
        self.locked = [false; 3];
        self.field = 0;
        self.literal = false;
        self.coordinate_mode = false;
        self.candidate = 0;
        self.focus = false;
        self.enter = false;
    }
    pub fn request_focus(&mut self) {
        self.focus = true;
    }
    pub fn is_literal(&self) -> bool {
        self.literal || !self.at_cursor
    }
    pub fn bottom_focus(&mut self) -> bool {
        !self.at_cursor && std::mem::take(&mut self.focus)
    }
    fn spec(&self, command: &CommandState) -> Option<Spec> {
        command.input_spec().map(|mut spec| {
            if self.coordinate_mode && matches!(spec.kind, Kind::Coordinates | Kind::Polar) {
                spec.kind = Kind::Coordinates;
            }
            spec
        })
    }
    pub fn to_bottom(&mut self, command: &CommandState, text: &mut String) {
        if self.at_cursor {
            *text = self.submission(command, text);
        }
        self.at_cursor = false;
        self.reset_fields();
        self.literal = true;
    }
    pub fn to_cursor(&mut self, command: &CommandState, text: &mut String) {
        let value = text.trim();
        if !self.at_cursor
            && command.takes_point()
            && (value.contains([',', '，', '<']) || value.split_whitespace().count() > 1)
            && !value.starts_with(['@', '#'])
            && !value.starts_with("W:")
            && !value.starts_with("w:")
        {
            *text = format!("#{value}");
        }
        self.at_cursor = true;
        self.literal = !text.is_empty();
        self.focus = true;
    }
    pub fn toggle(&mut self, command: &CommandState, text: &mut String) {
        if self.enabled {
            self.to_bottom(command, text);
            self.enabled = false;
        } else {
            self.enabled = true;
            self.to_cursor(command, text);
        }
        self.focus = true;
    }
    /// Runs before global hotkeys, so Tab and text cannot leak into selection or shortcuts.
    pub fn prepare(
        &mut self,
        ctx: &egui::Context,
        canvas: Option<egui::Rect>,
        command: &CommandState,
        text: &mut String,
    ) {
        let pointer = ctx.input(|i| i.pointer.hover_pos());
        let on_canvas = pointer.zip(canvas).is_some_and(|(p, r)| r.contains(p));
        if on_canvas && !ctx.egui_wants_pointer_input() {
            self.cursor = pointer;
        }
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::F12)) {
            self.toggle(command, text);
        }
        if !self.enabled {
            return;
        }
        let owned = owns_focus(ctx);
        let other_text = ctx.egui_wants_keyboard_input() && !owned;
        let bottom = ctx.memory(|m| m.has_focus(egui::Id::new("cad_command_input")));
        if self.at_cursor && (owned || on_canvas && !other_text) && confirm_space(ctx) {
            self.enter = true;
        }
        if on_canvas
            && ctx.input(|i| i.pointer.primary_pressed())
            && !ctx.egui_wants_pointer_input()
        {
            if bottom {
                self.to_cursor(command, text);
            } else {
                self.at_cursor = true;
            }
        }
        let typed = ctx.input(|i| {
            let mut text = String::new();
            let mut pasted = false;
            for event in &i.events {
                match event {
                    egui::Event::Text(t) => text.push_str(t),
                    egui::Event::Paste(t) if owned => {
                        text.push_str(t);
                        pasted = true;
                    }
                    _ => {}
                }
            }
            (!text.is_empty()).then_some((text, pasted))
        });
        if let Some((typed, pasted)) = typed {
            if self.at_cursor && (owned || on_canvas && !other_text) {
                let was_literal = self.literal;
                if !command.is_active() {
                    self.candidate = 0;
                }
                // "2P" / "3P" can arrive after the digit locked a coordinate field.
                if !self.literal
                    && typed.eq_ignore_ascii_case("P")
                    && (!command.circle_options().is_empty() || !command.arc_options().is_empty())
                    && self.locked[0]
                    && !self.locked[1]
                    && !self.locked[2]
                {
                    if let Some(spec) = self.spec(command) {
                        let fields = self.fields(spec, text);
                        if command
                            .circle_options()
                            .iter()
                            .chain(command.arc_options().iter())
                            .any(|(key, _)| *key == format!("{}P", fields[0]))
                        {
                            *text = fields[0].clone();
                            self.locked = [false; 3];
                            self.literal = true;
                        }
                    }
                }
                if !command.is_active()
                    || self.spec(command).is_none()
                    || typed.starts_with(['@', '#'])
                    || (pasted || !owned && typed.chars().count() > 1)
                        && (typed.contains([',', '，', '<'])
                            || typed.split_whitespace().count() > 1)
                    || matches!(typed.to_uppercase().as_str(), "U" | "C")
                    || command
                        .curve_parameter_option()
                        .is_some_and(|(key, _)| typed.eq_ignore_ascii_case(key))
                    || command.curve_reselect() && typed.eq_ignore_ascii_case("N")
                    || command.transform_reference() && typed.eq_ignore_ascii_case("R")
                    || !command.circle_options().is_empty()
                        && matches!(typed.to_uppercase().as_str(), "R" | "D" | "2P" | "3P")
                    || !command.arc_options().is_empty()
                        && matches!(typed.to_uppercase().as_str(), "C" | "A" | "E" | "3P")
                    || command.rectangle_corner()
                        && matches!(typed.to_uppercase().as_str(), "D" | "R")
                    || command.copy_mode().is_some()
                        && !command.selecting()
                        && matches!(typed.to_uppercase().as_str(), "M" | "S")
                    || !self.locked.iter().any(|v| *v) && typed.chars().any(char::is_alphabetic)
                {
                    if !self.literal {
                        text.clear();
                        self.locked = [false; 3];
                    }
                    self.literal = true;
                }
                // Areas perform an invisible sizing pass. Capture the first text before
                // assigning focus, then let TextEdit handle normal caret editing.
                if !owned || self.literal && !was_literal {
                    if self.literal || self.spec(command).is_none() {
                        text.push_str(&typed);
                    } else if let Some(spec) = self.spec(command) {
                        let mut fields = self.fields(spec, text);
                        let i = self.field.min(spec.count() - 1);
                        fields[i] = if self.locked[i] {
                            format!("{}{}", fields[i], typed)
                        } else {
                            typed.clone()
                        };
                        let advance = fields[i].ends_with([',', '，']);
                        fields[i] = fields[i].trim_end_matches([',', '，']).into();
                        self.locked[i] = !fields[i].is_empty();
                        *text = Self::encode(spec, &fields);
                        if advance {
                            self.field = (i + 1) % spec.count();
                        }
                    }
                    ctx.input_mut(|i| {
                        i.events
                            .retain(|e| !matches!(e, egui::Event::Text(_) | egui::Event::Paste(_)))
                    });
                }
                self.focus = true;
            }
        }
        if self.at_cursor
            && (owned || self.focus)
            && ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Enter))
        {
            self.enter = true;
        }
        if self.at_cursor && owned {
            let spec = self.spec(command);
            // Object-picking stages have no fields: leave Tab to candidate cycling.
            let field_or_command = spec.is_some() || !command.is_active();
            let backwards = field_or_command
                && ctx.input_mut(|i| i.consume_key(egui::Modifiers::SHIFT, egui::Key::Tab));
            let tab = backwards
                || field_or_command
                    && ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Tab));
            if tab {
                if let Some(spec) = spec.filter(|_| !self.literal) {
                    self.field =
                        (self.field + if backwards { spec.count() - 1 } else { 1 }) % spec.count();
                } else if !command.is_active() {
                    let matches = candidates(text);
                    if let Some((full, _, _)) = matches.get(self.candidate % matches.len().max(1)) {
                        *text = (*full).into();
                    }
                }
                self.focus = true;
            }
            if !command.is_active() {
                let n = candidates(text).len().max(1);
                ctx.input_mut(|i| {
                    if i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown) {
                        self.candidate = (self.candidate + 1) % n;
                    }
                    if i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp) {
                        self.candidate = (self.candidate + n - 1) % n;
                    }
                });
            }
        }
    }
    pub fn submission(&self, command: &CommandState, text: &str) -> String {
        let trimmed = text.trim();
        if !command.is_active() {
            return candidates(trimmed)
                .get(self.candidate)
                .map(|(full, _, _)| (*full).to_string())
                .unwrap_or_else(|| canonical(&trimmed.to_uppercase()).to_string());
        }
        if self.at_cursor
            && self.literal
            && command.takes_point()
            && (command.input_spec().is_some_and(|s| s.reference.is_some())
                || command.rectangle_placing())
            && !trimmed.starts_with(['@', '#'])
            && !trimmed.starts_with("W:")
            && !trimmed.starts_with("w:")
            && (trimmed.contains([',', '，', '<']) || trimmed.split_whitespace().count() > 1)
        {
            format!("@{trimmed}")
        } else {
            trimmed.into()
        }
    }
    fn fields(&self, spec: Spec, text: &str) -> [String; 3] {
        let values = spec.values();
        let pieces: Vec<_> = text
            .trim_start_matches(['@', '#'])
            .split([',', '<'])
            .collect();
        std::array::from_fn(|i| {
            if self.locked[i] {
                pieces.get(i).copied().unwrap_or("").into()
            } else {
                format!("{:.6}", values[i])
            }
        })
    }
    fn encode(spec: Spec, fields: &[String; 3]) -> String {
        match spec.kind {
            Kind::Coordinates => format!(
                "{}{},{},{}",
                if spec.reference.is_some() { "@" } else { "#" },
                fields[0],
                fields[1],
                fields[2]
            ),
            Kind::Polar => format!("@{}<{}", fields[0], fields[1]),
            Kind::ArrayRect => format!("{},{}", fields[0], fields[1]),
            _ => fields[0].clone(),
        }
    }
    pub fn sync(&self, command: &CommandState, text: &mut String) {
        if self.at_cursor && !self.literal && self.locked.iter().any(|v| *v) {
            if let Some(spec) = self.spec(command) {
                *text = Self::encode(spec, &self.fields(spec, text));
            }
        }
    }
    pub fn constrain(
        &self,
        command: &CommandState,
        text: &str,
        point: [f64; 3],
    ) -> Result<[f64; 3], String> {
        if self.literal || !self.at_cursor || !self.locked.iter().any(|v| *v) {
            return Ok(point);
        }
        let Some(mut spec) = self.spec(command) else {
            return Ok(point);
        };
        spec.point = Some(point);
        let fields = self.fields(spec, text);
        let mut values = spec.values();
        for i in 0..spec.count() {
            if self.locked[i] {
                values[i] = fields[i]
                    .parse::<f64>()
                    .ok()
                    .filter(|v| v.is_finite())
                    .ok_or("动态输入需要有限数值")?;
            }
        }
        let base = spec.reference.unwrap_or([0.; 3]);
        let result = match spec.kind {
            Kind::Coordinates => std::array::from_fn(|i| base[i] + values[i]),
            Kind::Polar | Kind::Radius | Kind::Diameter => {
                if matches!(spec.kind, Kind::Radius | Kind::Diameter) && values[0] <= 0. {
                    return Err("半径必须大于零".into());
                }
                let angle = values[1].to_radians();
                let delta = spec.plane.x_axis * (values[0] * angle.cos())
                    + spec.plane.y_axis * (values[0] * angle.sin());
                [base[0] + delta.x, base[1] + delta.y, base[2] + delta.z]
            }
            Kind::Angle | Kind::TargetAngle => {
                let d = Point3::new(point[0], point[1], point[2]);
                let local = spec.plane.to_local(d);
                let a = spec.plane.to_local(Point3::new(base[0], base[1], base[2]));
                let length = (local[0] - a[0]).hypot(local[1] - a[1]);
                let angle = values[0].to_radians();
                let delta = spec.plane.x_axis * (length * angle.cos())
                    + spec.plane.y_axis * (length * angle.sin());
                [base[0] + delta.x, base[1] + delta.y, base[2] + delta.z]
            }
            Kind::TargetLength => {
                if values[0] <= 0. {
                    return Err("新长度须大于零".into());
                }
                let d = [point[0] - base[0], point[1] - base[1], point[2] - base[2]];
                let length = d.iter().map(|v| v * v).sum::<f64>().sqrt();
                if length <= 1e-9 {
                    point
                } else {
                    std::array::from_fn(|i| base[i] + d[i] * values[0] / length)
                }
            }
            _ => point,
        };
        if result.iter().all(|v| v.is_finite()) {
            Ok(result)
        } else {
            Err("动态输入超出有效数值范围".into())
        }
    }
    pub fn ui(
        &mut self,
        ctx: &egui::Context,
        canvas: egui::Rect,
        command: &CommandState,
        text: &mut String,
    ) -> Option<String> {
        if !self.enabled || !command.is_active() && text.is_empty() && !self.focus && !self.enter {
            self.rect = None;
            return None;
        }
        let enter = std::mem::take(&mut self.enter);
        if canvas.width() < 340. || canvas.height() < 260. {
            let submission = enter.then(|| self.submission(command, text));
            self.to_bottom(command, text);
            self.focus = true;
            return submission;
        }
        let cursor = self.cursor.unwrap_or(canvas.center());
        let size = self
            .rect
            .map(|r| r.size())
            .unwrap_or(egui::vec2(326., 235.));
        let old = self
            .rect
            .filter(|r| ctx.input(|i| i.pointer.hover_pos().is_some_and(|p| r.contains(p))));
        let wanted = old.map(|r| r.min).unwrap_or(egui::pos2(
            if cursor.x + 18. + size.x > canvas.right() {
                cursor.x - size.x - 18.
            } else {
                cursor.x + 18.
            },
            if cursor.y + 20. + size.y > canvas.bottom() {
                cursor.y - size.y - 20.
            } else {
                cursor.y + 20.
            },
        ));
        let pos = egui::pos2(
            wanted.x.min(canvas.right() - size.x).max(canvas.left()),
            wanted.y.min(canvas.bottom() - size.y).max(canvas.top()),
        );
        if old.is_none() && egui::Rect::from_min_size(pos, size).contains(cursor) {
            let submission = enter.then(|| self.submission(command, text));
            self.to_bottom(command, text);
            self.focus = true;
            return submission;
        }
        let mut submitted = None;
        let spec = self.spec(command);
        let response = egui::Area::new(egui::Id::new("cad_dynamic_input"))
            .order(egui::Order::Foreground)
            .fixed_pos(pos)
            .constrain_to(canvas)
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.set_width(306.0_f32.min(canvas.width().max(80.) - 16.));
                    ui.add(egui::Label::new(command.prompt()).wrap());
                    if !self.at_cursor {
                        ui.label(text.as_str());
                        if ui.small_button("切换到光标输入").clicked() {
                            self.to_cursor(command, text);
                        }
                        return;
                    }
                    if let Some(spec) = spec.filter(|_| !self.literal) {
                        if matches!(spec.kind, Kind::Polar | Kind::Coordinates)
                            && spec.reference.is_some()
                        {
                            if ui
                                .small_button(if self.coordinate_mode {
                                    "坐标（切换会清除锁定）"
                                } else {
                                    "长度 / 角度（切换会清除锁定）"
                                })
                                .clicked()
                            {
                                self.coordinate_mode = !self.coordinate_mode;
                                self.locked = [false; 3];
                                text.clear();
                                self.field = 0;
                                self.focus = true;
                            }
                        }
                        ui.weak(match spec.kind {
                            Kind::Coordinates if spec.reference.is_none() => {
                                "WCS 绝对坐标 · Tab 切换字段"
                            }
                            Kind::Coordinates => "相对基点的 WCS 位移 · Tab 切换字段",
                            Kind::Polar => "工作平面长度/角度 · Tab 切换字段",
                            Kind::Radius => "半径 mm · 输入值锁定预览",
                            Kind::Diameter => "直径 mm · 半径为该长度的一半",
                            Kind::Angle => "旋转角度（度） · Enter 确认参数",
                            Kind::Scale => "缩放比例 · Enter 确认参数",
                            Kind::Distance => "输入距离后确认，随后在画布点选",
                            Kind::ArrayRect => "列/行数量 · Tab 切换 · 确认提交",
                            Kind::Count => "阵列数量 · 确认提交；分布角度在工具页设置",
                            Kind::ReferenceAngle | Kind::ReferenceLength => {
                                "输入原值，或点选两个参照点"
                            }
                            Kind::TargetAngle | Kind::TargetLength => {
                                "新值相对于参照计算；确认提交"
                            }
                            Kind::RectangleLength | Kind::RectangleWidth => {
                                "输入正尺寸后确认；U 返回上一步"
                            }
                            Kind::RectangleAngle => "相对工作平面 U 轴的角度；正角绕法向按右手方向",
                            Kind::ArcSweep => "非零且绝对值小于 360°；正角逆时针，负角顺时针",
                        });
                        let mut fields = self.fields(spec, text);
                        let target = self.field;
                        let width = (ui.available_width() / spec.count() as f32 - 8.).max(40.);
                        let mut next_focus = None;
                        ui.horizontal(|ui| {
                            for i in 0..spec.count() {
                                ui.vertical(|ui| {
                                    ui.label(format!(
                                        "{}{}",
                                        spec.labels()[i],
                                        if self.locked[i] { " [锁]" } else { "" }
                                    ));
                                    let mut value = if self.locked[i] {
                                        fields[i].clone()
                                    } else {
                                        String::new()
                                    };
                                    let id = field_id(i);
                                    if self.focus && target == i {
                                        ui.memory_mut(|m| m.request_focus(id));
                                    }
                                    let response = ui.add_sized(
                                        [width, 24.],
                                        egui::TextEdit::singleline(&mut value)
                                            .id(id)
                                            .hint_text(&fields[i]),
                                    );
                                    if response.clicked() || response.has_focus() && !self.focus {
                                        self.field = i;
                                    }
                                    if response.changed() {
                                        let advance = value.contains([',', '，']);
                                        value = value.trim_end_matches([',', '，']).into();
                                        self.locked[i] = !value.is_empty();
                                        fields[i] = if self.locked[i] {
                                            value
                                        } else {
                                            format!("{:.6}", spec.values()[i])
                                        };
                                        *text = if self.locked.iter().any(|v| *v) {
                                            Self::encode(spec, &fields)
                                        } else {
                                            String::new()
                                        };
                                        if advance && i + 1 < spec.count() {
                                            next_focus = Some(i + 1);
                                        }
                                    }
                                });
                            }
                        });
                        if let Some(i) = next_focus {
                            self.field = i;
                            ui.memory_mut(|m| m.request_focus(field_id(i)));
                        }
                        if self.locked.iter().any(|v| *v)
                            && ui.small_button("清除尺寸锁定").clicked()
                        {
                            self.locked = [false; 3];
                            text.clear();
                            self.field = 0;
                            ui.memory_mut(|m| m.request_focus(field_id(0)));
                        }
                    } else {
                        let id = raw_id();
                        if self.focus {
                            ui.memory_mut(|m| m.request_focus(id));
                        }
                        let edited = ui.add(
                            egui::TextEdit::singleline(text)
                                .id(id)
                                .desired_width(f32::INFINITY)
                                .hint_text("输入命令、选项、#绝对 / @相对坐标"),
                        );
                        if edited.changed() && !command.is_active() {
                            self.candidate = 0;
                        }
                        if !command.is_active() {
                            let matches = candidates(text);
                            self.candidate = self.candidate.min(matches.len().saturating_sub(1));
                            for (i, (full, alias, label)) in matches.iter().enumerate() {
                                if ui
                                    .selectable_label(
                                        self.candidate == i,
                                        format!("{full}  {alias}  {label}"),
                                    )
                                    .clicked()
                                {
                                    submitted = Some((*full).into());
                                }
                            }
                            ui.weak("↑↓ 选择 · Tab 补全 · Enter 执行");
                        }
                    }
                    if !command.error_message().is_empty() {
                        ui.colored_label(egui::Color32::LIGHT_RED, command.error_message());
                    }
                    if enter {
                        let matches = if command.is_active() {
                            vec![]
                        } else {
                            candidates(text)
                        };
                        submitted = Some(if let Some((full, _, _)) = matches.get(self.candidate) {
                            (*full).into()
                        } else {
                            self.submission(command, text)
                        });
                    }
                    if !ui.is_sizing_pass() {
                        self.focus = false;
                    }
                });
            });
        self.rect = Some(response.response.rect);
        submitted
    }
}
