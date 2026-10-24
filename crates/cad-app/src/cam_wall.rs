//! One-sided end-cut allowances using the local direction and inner-shell projection.

use crate::cam_analysis::{EndCutSide, TubeSection};

/// Return the extra X movement from the already Z-corrected pose.
pub fn end_cut_allowance(
    section: &TubeSection,
    thickness: f64,
    source: [f64; 3],
    pose: [f64; 4],
    slopes: [f64; 2],
    side: EndCutSide,
) -> Result<f64, String> {
    let (sin, cos) = pose[3].to_radians().sin_cos();
    let y = pose[1] - section.center_y;
    let z = pose[2] - section.center_z;
    // Move one wall thickness along the tool direction, then project onto the inner shell.
    let candidate = [
        y * cos + z * sin - thickness * sin,
        -y * sin + z * cos - thickness * cos,
    ];
    let inner = project_inner(
        candidate,
        section.half_y() - thickness,
        section.half_z() - thickness,
        (section.corner_radius() - thickness).max(0.0),
    );
    let inner_x = source[0]
        + slopes[0] * (section.center_y + inner[0] - source[1])
        + slopes[1] * (section.center_z + inner[1] - source[2]);
    let extra = match side {
        EndCutSide::Left => (source[0].max(inner_x) - pose[0]).max(0.0),
        EndCutSide::Right => (source[0].min(inner_x) - pose[0]).min(0.0),
        EndCutSide::Section => 0.0,
    };
    if extra.is_finite() {
        Ok(extra)
    } else {
        Err("壁厚补偿产生非法 X 偏移".to_string())
    }
}

fn project_inner(point: [f64; 2], hy: f64, hz: f64, radius: f64) -> [f64; 2] {
    let r = radius.clamp(0.0, hy.min(hz));
    let (sy, sz) = (
        if point[0] < 0.0 { -1.0 } else { 1.0 },
        if point[1] < 0.0 { -1.0 } else { 1.0 },
    );
    let (y, z) = (point[0].abs(), point[1].abs());
    let (cy, cz) = (hy - r, hz - r);
    if y > cy && z > cz {
        let (dy, dz) = (y - cy, z - cz);
        let length = dy.hypot(dz);
        [sy * (cy + r * dy / length), sz * (cz + r * dz / length)]
    } else if y - hy > z - hz {
        [sy * hy, point[1].clamp(-cz, cz)]
    } else {
        [point[0].clamp(-cy, cy), sz * hz]
    }
}
