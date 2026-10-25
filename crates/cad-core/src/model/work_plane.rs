//! A fixed coordinate frame for precise planar drafting in world space.

use super::{Point3, Vector3, GEOMETRY_EPSILON};

#[derive(Clone, Copy, Debug)]
pub struct WorkPlane {
    pub origin: Point3,
    pub x_axis: Vector3,
    pub y_axis: Vector3,
}

impl Default for WorkPlane {
    fn default() -> Self {
        Self::xy(Point3::new(0.0, 0.0, 0.0))
    }
}

impl WorkPlane {
    pub fn xy(origin: Point3) -> Self {
        Self {
            origin,
            x_axis: Vector3::new(1.0, 0.0, 0.0),
            y_axis: Vector3::new(0.0, 1.0, 0.0),
        }
    }

    pub fn yz(origin: Point3) -> Self {
        Self {
            origin,
            x_axis: Vector3::new(0.0, 1.0, 0.0),
            y_axis: Vector3::new(0.0, 0.0, 1.0),
        }
    }

    pub fn zx(origin: Point3) -> Self {
        Self {
            origin,
            x_axis: Vector3::new(0.0, 0.0, 1.0),
            y_axis: Vector3::new(1.0, 0.0, 0.0),
        }
    }

    /// Origin, point on +U, then a point on the +V side (not necessarily orthogonal).
    pub fn from_points(origin: Point3, x: Point3, xy: Point3) -> Result<Self, String> {
        let u = Vector3::new(x.x - origin.x, x.y - origin.y, x.z - origin.z);
        let v = Vector3::new(xy.x - origin.x, xy.y - origin.y, xy.z - origin.z);
        let ul = u.length();
        let vl = v.length();
        if ![origin.x, origin.y, origin.z, ul, vl]
            .iter()
            .all(|value| value.is_finite())
            || ul <= GEOMETRY_EPSILON
            || vl <= GEOMETRY_EPSILON
        {
            return Err("定义平面的点必须有限且互不重合".into());
        }
        let x_axis = u * (1.0 / ul);
        let normal = x_axis.cross(v * (1.0 / vl));
        if normal.length() < 1e-8 {
            return Err("三点共线或过于接近共线，请重新输入第三点".into());
        }
        Ok(Self {
            origin,
            x_axis,
            y_axis: normal.normalize().cross(x_axis),
        })
    }

    pub fn normal(self) -> Vector3 {
        self.x_axis.cross(self.y_axis)
    }

    pub fn to_world(self, local: [f64; 3]) -> Point3 {
        let delta = self.x_axis * local[0] + self.y_axis * local[1] + self.normal() * local[2];
        Point3::new(
            self.origin.x + delta.x,
            self.origin.y + delta.y,
            self.origin.z + delta.z,
        )
    }

    pub fn to_local(self, world: Point3) -> [f64; 3] {
        let delta = Vector3::new(
            world.x - self.origin.x,
            world.y - self.origin.y,
            world.z - self.origin.z,
        );
        [
            delta.dot(self.x_axis),
            delta.dot(self.y_axis),
            delta.dot(self.normal()),
        ]
    }

    pub fn intersect_ray(self, origin: Point3, direction: Vector3) -> Result<Point3, String> {
        let normal = self.normal();
        let denominator = normal.dot(direction);
        if !denominator.is_finite() || denominator.abs() < 1e-6 {
            return Err("视线与工作平面近乎平行，请调整视角或输入精确坐标".into());
        }
        let delta = Vector3::new(
            self.origin.x - origin.x,
            self.origin.y - origin.y,
            self.origin.z - origin.z,
        );
        let distance = delta.dot(normal) / denominator;
        if distance < 0.0 {
            return Err("工作平面位于视线后方，请调整视角或输入坐标".into());
        }
        let p = Point3::new(
            origin.x + direction.x * distance,
            origin.y + direction.y * distance,
            origin.z + direction.z * distance,
        );
        if ![p.x, p.y, p.z].iter().all(|value| value.is_finite()) {
            return Err("交点坐标超出有效数值范围".into());
        }
        Ok(p)
    }
}
