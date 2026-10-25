//! Floating display origin; document and CAM coordinates remain unchanged.
use bevy::prelude::*;
use cad_core::model::{Point3, Vector3};

#[derive(Clone, Copy, Debug, Default)]
pub struct ViewFrame {
    pub origin: [f64; 3],
    pub pixel_size: f64,
}

impl ViewFrame {
    pub fn point(self, p: [f64; 3]) -> Vec3 {
        Vec3::new(
            (p[0] - self.origin[0]) as f32,
            (p[2] - self.origin[2]) as f32,
            -((p[1] - self.origin[1]) as f32),
        )
    }
    pub fn world(self, p: Vec3) -> Point3 {
        Point3::new(
            self.origin[0] + p.x as f64,
            self.origin[1] - p.z as f64,
            self.origin[2] + p.y as f64,
        )
    }
    pub fn ray(
        self,
        camera: &Camera,
        transform: &GlobalTransform,
        cursor: Vec2,
    ) -> Option<(Point3, Vector3)> {
        let ray = camera.viewport_to_world(transform, cursor).ok()?;
        Some((
            self.world(ray.origin),
            Vector3::new(
                ray.direction.x as f64,
                -ray.direction.z as f64,
                ray.direction.y as f64,
            )
            .normalize(),
        ))
    }
    pub fn project(
        self,
        camera: &Camera,
        transform: &GlobalTransform,
        p: [f64; 3],
    ) -> Option<Vec2> {
        camera.world_to_viewport(transform, self.point(p)).ok()
    }
}
