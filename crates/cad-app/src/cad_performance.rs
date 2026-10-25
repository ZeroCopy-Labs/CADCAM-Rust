//! Passive timings of the user's own operations; no automatic benchmark workloads.
use bevy_egui::egui;
use std::{collections::BTreeMap, time::Instant};

#[derive(Default)]
pub struct Performance {
    pub open: bool,
    pub last: BTreeMap<String, f64>,
    pub points: usize,
    pub bytes: usize,
    pub candidates: usize,
    pub segments: usize,
    pub discarded: usize,
}
impl Performance {
    pub fn record(&mut self, name: &str, start: Instant) {
        self.last
            .insert(name.into(), start.elapsed().as_secs_f64() * 1000.0);
    }
    pub fn paths(
        &mut self,
        display: &crate::display::DisplayData,
        machining: &crate::display::DisplayData,
    ) {
        self.points = display
            .paths
            .iter()
            .chain(&machining.paths)
            .map(|p| p.points.len())
            .sum();
        self.bytes = display
            .paths
            .iter()
            .chain(&machining.paths)
            .map(|p| p.points.capacity() * std::mem::size_of::<[f64; 3]>())
            .sum();
    }
    pub fn ui(&mut self, ctx: &egui::Context) {
        if !self.open {
            return;
        }
        let mut open = self.open;
        egui::Window::new("性能记录").open(&mut open).default_width(460.0).show(ctx,|ui|{
            ui.label("记录本次实际操作的 CPU 耗时；没有自动运行测试或基准。");
            egui::Grid::new("performance_timings").striped(true).show(ui,|ui|{
                for (name,ms) in &self.last{ui.label(name);ui.monospace(format!("{ms:.3} ms"));ui.end_row();}
            });
            ui.separator();
            ui.label(format!("显示 + 加工路径点：{}；点容器容量约 {:.2} MiB",self.points,self.bytes as f64/(1024.0*1024.0)));
            ui.small("此容量只统计点数组，不是进程内存；未包含原生文档、历史、索引或 GPU。");
            ui.label(format!("本次拾取候选 {} / 索引线段 {}；丢弃过期结果 {} 次",self.candidates,self.segments,self.discarded));
            ui.small("绘制耗时指 CPU 提交 Gizmo，不代表 GPU 时间或帧率；真实大图结果请按相同视图和操作比较。");
            if ui.button("复制当前记录").clicked(){ctx.copy_text(format!("操作耗时(ms)：{:#?}\n路径点={} 点容量={} bytes\n拾取候选={}/{} 过期结果={}",self.last,self.points,self.bytes,self.candidates,self.segments,self.discarded));}
        });
        self.open = open;
    }
}
