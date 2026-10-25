//! Screen projection cache and bounded uniform-grid broad phase. Geometry remains f64 in the document.
use crate::{cad_view::ViewFrame, cam, display::DisplayData};
use bevy::prelude::*;
use bevy_egui::egui;
use std::collections::{BTreeMap, BTreeSet, HashMap};

#[derive(PartialEq)]
struct Key {
    session: u64,
    version: (u64, u64),
    origin: [f64; 3],
    matrix: [f32; 16],
    projection: [f32; 16],
    viewport: Option<[f32; 4]>,
}
struct Segment {
    id: u32,
    a: Vec2,
    b: Vec2,
    world_a: [f64; 3],
    world_b: [f64; 3],
}
struct EntityBox {
    min: Vec2,
    max: Vec2,
    complete: bool,
}
struct TextBox {
    id: u32,
    points: Vec<egui::Pos2>,
    world: [f64; 3],
}
#[derive(Default)]
pub struct PickCache {
    key: Option<Key>,
    segments: Vec<Segment>,
    grid: HashMap<(i32, i32), Vec<usize>>,
    global: Vec<usize>,
    bounds: BTreeMap<u32, EntityBox>,
    texts: Vec<TextBox>,
}
const CELL: f32 = 64.0;
fn cells(min: Vec2, max: Vec2) -> Option<(i32, i32, i32, i32)> {
    if !min.is_finite() || !max.is_finite() {
        return None;
    }
    let a = (min / CELL).floor();
    let b = (max / CELL).floor();
    let x0 = a.x as i32;
    let y0 = a.y as i32;
    let x1 = b.x as i32;
    let y1 = b.y as i32;
    let count =
        (i64::from(x1) - i64::from(x0) + 1).checked_mul(i64::from(y1) - i64::from(y0) + 1)?;
    (count > 0 && count <= 4096).then_some((x0, y0, x1, y1))
}
impl PickCache {
    pub fn update(
        &mut self,
        data: &DisplayData,
        frame: ViewFrame,
        camera: &Camera,
        transform: &GlobalTransform,
        session: u64,
        version: (u64, u64),
    ) -> bool {
        let key = Key {
            session,
            version,
            origin: frame.origin,
            matrix: transform.to_matrix().to_cols_array(),
            projection: camera.clip_from_view().to_cols_array(),
            viewport: camera
                .logical_viewport_rect()
                .map(|r| [r.min.x, r.min.y, r.max.x, r.max.y]),
        };
        if self.key.as_ref() == Some(&key) {
            return false;
        }
        self.key = Some(key);
        self.segments.clear();
        self.grid.clear();
        self.global.clear();
        self.bounds.clear();
        self.texts.clear();
        let text_ids: BTreeSet<_> = data
            .entities
            .iter()
            .filter(|e| e.kind == "TEXT")
            .map(|e| e.id)
            .collect();
        let mut grid_references = 0usize;
        for path in data.paths.iter().filter(|p| p.visible) {
            let points: Vec<_> = path
                .points
                .iter()
                .map(|p| {
                    frame
                        .project(camera, transform, *p)
                        .filter(|p| p.is_finite())
                })
                .collect();
            let bound = self.bounds.entry(path.entity_id).or_insert(EntityBox {
                min: Vec2::splat(f32::INFINITY),
                max: Vec2::splat(f32::NEG_INFINITY),
                complete: true,
            });
            bound.complete &= points.len() >= 2 && points.iter().all(Option::is_some);
            for p in points.iter().flatten() {
                bound.min = bound.min.min(*p);
                bound.max = bound.max.max(*p);
            }
            for (i, pair) in points.windows(2).enumerate() {
                let (Some(a), Some(b)) = (pair[0], pair[1]) else {
                    continue;
                };
                let index = self.segments.len();
                self.segments.push(Segment {
                    id: path.entity_id,
                    a,
                    b,
                    world_a: path.points[i],
                    world_b: path.points[i + 1],
                });
                if let Some((x0, y0, x1, y1)) =
                    cells(a.min(b), a.max(b)).filter(|(x0, y0, x1, y1)| {
                        grid_references
                            + ((i64::from(*x1) - i64::from(*x0) + 1)
                                * (i64::from(*y1) - i64::from(*y0) + 1))
                                as usize
                            <= 1_000_000
                    })
                {
                    grid_references += ((i64::from(x1) - i64::from(x0) + 1)
                        * (i64::from(y1) - i64::from(y0) + 1))
                        as usize;
                    for x in x0..=x1 {
                        for y in y0..=y1 {
                            self.grid.entry((x, y)).or_default().push(index);
                        }
                    }
                } else {
                    self.global.push(index);
                }
            }
            if text_ids.contains(&path.entity_id)
                && points.iter().all(Option::is_some)
                && path.points.len() >= 4
            {
                let p = path.points[0];
                let q = path.points[2];
                self.texts.push(TextBox {
                    id: path.entity_id,
                    points: points
                        .iter()
                        .flatten()
                        .map(|p| egui::pos2(p.x, p.y))
                        .collect(),
                    world: std::array::from_fn(|i| (p[i] + q[i]) * 0.5),
                });
            }
        }
        true
    }
    fn candidates(&self, min: Vec2, max: Vec2) -> Vec<usize> {
        let Some((x0, y0, x1, y1)) = cells(min, max) else {
            return (0..self.segments.len()).collect();
        };
        let mut result = self.global.clone();
        for x in x0..=x1 {
            for y in y0..=y1 {
                if let Some(indices) = self.grid.get(&(x, y)) {
                    result.extend(indices);
                }
            }
        }
        result.sort_unstable();
        result.dedup();
        result
    }
    pub fn len(&self) -> usize {
        self.segments.len()
    }
    pub fn hits(
        &self,
        frame: ViewFrame,
        transform: &GlobalTransform,
        cursor: Vec2,
    ) -> (Vec<u32>, usize) {
        let indices = self.candidates(
            cursor - Vec2::splat(crate::PICK_TOLERANCE),
            cursor + Vec2::splat(crate::PICK_TOLERANCE),
        );
        let mut hits: BTreeMap<u32, (f32, f32)> = BTreeMap::new();
        for i in &indices {
            let s = &self.segments[*i];
            let distance = crate::distance_to_segment(cursor, s.a, s.b);
            if !distance.is_finite() || distance > crate::PICK_TOLERANCE {
                continue;
            }
            let d = s.b - s.a;
            let t = if d.length_squared() > 1e-12 {
                ((cursor - s.a).dot(d) / d.length_squared()).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let p = frame.point(std::array::from_fn(|i| {
                s.world_a[i] + (s.world_b[i] - s.world_a[i]) * t as f64
            }));
            let depth = (p - transform.translation()).length_squared();
            let old = hits.entry(s.id).or_insert((distance, depth));
            if distance < old.0 || distance == old.0 && depth < old.1 {
                *old = (distance, depth);
            }
        }
        for text in &self.texts {
            if crate::point_in_polygon(egui::pos2(cursor.x, cursor.y), &text.points) {
                hits.insert(
                    text.id,
                    (
                        0.0,
                        (frame.point(text.world) - transform.translation()).length_squared(),
                    ),
                );
            }
        }
        let mut hits: Vec<_> = hits.into_iter().collect();
        hits.sort_by(|a, b| {
            a.1 .0
                .total_cmp(&b.1 .0)
                .then(a.1 .1.total_cmp(&b.1 .1))
                .then(a.0.cmp(&b.0))
        });
        (hits.into_iter().map(|(id, _)| id).collect(), indices.len())
    }
    pub fn box_entities(&self, start: Vec2, end: Vec2) -> BTreeSet<u32> {
        let rect = egui::Rect::from_two_pos(egui::pos2(start.x, start.y), egui::pos2(end.x, end.y));
        if end.x >= start.x {
            return self
                .bounds
                .iter()
                .filter(|(_, b)| {
                    b.complete
                        && rect.contains(egui::pos2(b.min.x, b.min.y))
                        && rect.contains(egui::pos2(b.max.x, b.max.y))
                })
                .map(|(id, _)| *id)
                .collect();
        }
        self.candidates(
            Vec2::new(rect.min.x, rect.min.y),
            Vec2::new(rect.max.x, rect.max.y),
        )
        .into_iter()
        .filter_map(|i| {
            let s = &self.segments[i];
            (rect.contains(egui::pos2(s.a.x, s.a.y))
                || rect.contains(egui::pos2(s.b.x, s.b.y))
                || crate::segment_intersects_rect(s.a, s.b, rect))
            .then_some(s.id)
        })
        .collect()
    }
    pub fn box_units(&self, cam: &cam::CamModel, start: Vec2, end: Vec2) -> BTreeSet<cam::UnitId> {
        let chosen = self.box_entities(start, end);
        if end.x < start.x {
            return chosen
                .iter()
                .filter_map(|id| cam.unit_for_entity(*id))
                .collect();
        }
        cam.units
            .iter()
            .filter(|u| {
                let visible: Vec<_> = u
                    .members
                    .iter()
                    .filter(|id| self.bounds.contains_key(id))
                    .collect();
                !visible.is_empty() && visible.iter().all(|id| chosen.contains(id))
            })
            .map(|u| u.id)
            .collect()
    }
}
