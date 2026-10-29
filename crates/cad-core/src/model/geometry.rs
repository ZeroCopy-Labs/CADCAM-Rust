//! Geometry types of the internal CAD model.
//!
//! All coordinates are represented as plain `f64` values in the drawing's
//! world coordinate system (WCS). Planar entities keep the plane they belong to
//! so that later CAM code can still reason about normals and arc directions.
//!
//! These values are intentionally free of IDs, colors, parser handles and
//! editor/selection state. Geometry can therefore be copied into CAM snapshots
//! or display adapters without carrying UI lifetimes or file-format objects.

use std::ops::{Add, Mul, Sub};

/// Small geometric tolerance used for zero-length / degenerate checks.
/// It is a length epsilon in drawing units, not a display-pixel tolerance.
pub const GEOMETRY_EPSILON: f64 = 1e-9;

/// A point in drawing world coordinates.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Point3 {
    /// X coordinate in WCS.
    pub x: f64,
    /// Y coordinate in WCS.
    pub y: f64,
    /// Z coordinate in WCS.
    pub z: f64,
}

impl Point3 {
    /// Construct a point from WCS coordinates.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// A vector in drawing world coordinates.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Vector3 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
}

impl Vector3 {
    /// Construct a vector from components.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Euclidean length of the vector.
    pub fn length(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Normalize the vector. Returns a zero vector for (near-)zero input.
    pub fn normalize(self) -> Self {
        let length = self.length();
        if length <= GEOMETRY_EPSILON {
            Self::new(0.0, 0.0, 0.0)
        } else {
            self * (1.0 / length)
        }
    }

    /// Dot product with another vector.
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product `self × other`.
    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }
}

impl Mul<f64> for Vector3 {
    type Output = Self;

    fn mul(self, scale: f64) -> Self {
        Self::new(self.x * scale, self.y * scale, self.z * scale)
    }
}

impl Add for Vector3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

/// Plane of a planar entity.
///
/// `x_axis` defines the zero-angle direction for arcs and circles and is always
/// perpendicular to `normal`. `y_axis` is `normal × x_axis`.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Plane {
    /// Unit normal of the plane.
    pub normal: Vector3,
    /// Unit reference axis used as angle zero.
    pub x_axis: Vector3,
}

impl Plane {
    /// Create a plane, normalizing both axes.
    pub fn new(normal: Vector3, x_axis: Vector3) -> Self {
        Self {
            normal: normal.normalize(),
            x_axis: x_axis.normalize(),
        }
    }

    /// Second in-plane axis, computed as `normal × x_axis`.
    pub fn y_axis(self) -> Vector3 {
        self.normal.cross(self.x_axis)
    }
}

/// A line segment defined by two points.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Line {
    /// Start point in WCS.
    pub start: Point3,
    /// End point in WCS.
    pub end: Point3,
}

/// One segment of a polyline.
///
/// Coordinates are WCS 3D. Arc angles are interpreted in the owning
/// [`Polyline::plane`], not in an implicit XY plane and not in a per-segment
/// plane.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PolylineSegment {
    /// Straight segment from `start` to `end`.
    Line {
        /// Segment start in WCS.
        start: Point3,
        /// Segment end in WCS.
        end: Point3,
    },
    /// Circular arc segment.
    Arc {
        /// Arc start in WCS.
        start: Point3,
        /// Arc end in WCS.
        end: Point3,
        /// Arc center in WCS.
        center: Point3,
        /// Arc radius.
        radius: f64,
        /// Start angle measured from `Polyline::plane.x_axis`.
        start_angle: f64,
        /// Signed sweep angle; positive is counter-clockwise around the plane normal.
        sweep_angle: f64,
    },
}

/// An ordered sequence of line and arc segments.
///
/// `closed == true` means the closing segment is already part of `segments`;
/// consumers can walk the whole boundary without a special final step.
///
/// `plane` is shared by all arc segments of a planar (2D-source) polyline.
/// A 3D-source polyline has `plane == None` and line segments only.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Polyline {
    /// Ordered segments, including the closing segment for closed polylines.
    pub segments: Vec<PolylineSegment>,
    /// Whether the source polyline was closed.
    pub closed: bool,
    /// Plane used to interpret arc angles; `None` for spatial line-only paths.
    pub plane: Option<Plane>,
}

/// A full circle.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Circle {
    /// Circle center in WCS.
    pub center: Point3,
    /// Circle radius in drawing units.
    pub radius: f64,
    /// Plane containing the circle and defining its angle basis.
    pub plane: Plane,
}

/// A circular arc.
///
/// `sweep_angle` is signed: positive means counter-clockwise around
/// `plane.normal`. The full angle is measured from `plane.x_axis`.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Arc {
    /// Arc center in WCS.
    pub center: Point3,
    /// Arc radius in drawing units.
    pub radius: f64,
    /// Start angle measured from `plane.x_axis`.
    pub start_angle: f64,
    /// Signed sweep angle; positive is CCW around `plane.normal`.
    pub sweep_angle: f64,
    /// Plane containing the arc and defining its angle basis.
    pub plane: Plane,
}

/// A full or partial ellipse.
///
/// `major_axis` points from the center to the major-axis endpoint (its length
/// is the semi-major axis). `minor_axis` is the corresponding semi-minor axis
/// vector; `major_axis × minor_axis` gives the plane normal. The parametric form
/// is `center + major_axis·cos(t) + minor_axis·sin(t)`.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Ellipse {
    /// Ellipse center in WCS.
    pub center: Point3,
    /// Semi-major axis vector; its length is the major radius.
    pub major_axis: Vector3,
    /// Semi-minor axis vector, perpendicular to `major_axis`.
    pub minor_axis: Vector3,
    /// Parameter at which the ellipse starts.
    pub start_parameter: f64,
    /// Parameter range. A full ellipse uses `2π`; partial ellipses are always
    /// traversed in the positive (counter-clockwise) parameter direction.
    pub sweep_parameter: f64,
}

impl Ellipse {
    pub fn is_full(&self) -> bool {
        self.sweep_parameter >= std::f64::consts::TAU - 1e-10
    }

    pub fn point_at(&self, parameter: f64) -> Point3 {
        let d = self.major_axis * parameter.cos() + self.minor_axis * parameter.sin();
        Point3::new(self.center.x + d.x, self.center.y + d.y, self.center.z + d.z)
    }

    /// Parameter of the radial intersection with the supporting ellipse.
    pub fn parameter_at(&self, point: Point3) -> Result<f64, String> {
        let a = self.major_axis.length();
        let b = self.minor_axis.length();
        if a <= GEOMETRY_EPSILON || b <= GEOMETRY_EPSILON {
            return Err("椭圆半轴必须大于零".into());
        }
        let d = Vector3::new(
            point.x - self.center.x,
            point.y - self.center.y,
            point.z - self.center.z,
        );
        let x = d.dot(self.major_axis) / (a * a);
        let y = d.dot(self.minor_axis) / (b * b);
        if !x.is_finite() || !y.is_finite() || x.hypot(y) <= 1e-12 {
            return Err("请指定椭圆中心以外的有效方向点".into());
        }
        Ok(y.atan2(x).rem_euclid(std::f64::consts::TAU))
    }
}

/// A NURBS spline.
///
/// The knot vector, control points, weights and flags are preserved as-is from
/// the source file; evaluation is intentionally left to a later phase.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Spline {
    /// NURBS degree.
    pub degree: u32,
    /// Control points in WCS.
    pub control_points: Vec<Point3>,
    /// Knot vector.
    pub knots: Vec<f64>,
    /// Optional rational weights; empty means all weights are 1.
    pub weights: Vec<f64>,
    /// Optional fit points preserved from the source.
    pub fit_points: Vec<Point3>,
    /// Closed spline flag.
    pub closed: bool,
    /// Periodic spline flag.
    pub periodic: bool,
    /// Rational (weighted) spline flag.
    pub rational: bool,
    /// Planar spline flag from the source.
    pub planar: bool,
}

/// Geometry of one entity in the internal model.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Geometry {
    /// Basic single-line text; never a cutting contour.
    Text(Text),
    /// Standalone line.
    Line(Line),
    /// Ordered line/arc path.
    Polyline(Polyline),
    /// Full circle.
    Circle(Circle),
    /// Circular arc.
    Arc(Arc),
    /// Full or partial ellipse.
    Ellipse(Ellipse),
    /// NURBS spline.
    Spline(Spline),
}

impl Geometry {
    /// Stable name of the geometry kind, used in reports and summaries.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Text(_) => "TEXT",
            Self::Line(_) => "LINE",
            Self::Polyline(_) => "POLYLINE",
            Self::Circle(_) => "CIRCLE",
            Self::Arc(_) => "ARC",
            Self::Ellipse(_) => "ELLIPSE",
            Self::Spline(_) => "SPLINE",
        }
    }
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Text {
    pub position: Point3,
    pub content: String,
    pub height: f64,
    pub plane: Plane,
}
