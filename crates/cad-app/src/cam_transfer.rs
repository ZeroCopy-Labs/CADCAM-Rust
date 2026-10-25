//! Clearance-constrained quintic transfers, emitted as rapid line segments.

use crate::cam_trajectory::{push_rapid, MachineMove, MachinePose, TransferConfig};

pub fn smooth_transfer(
    start: MachinePose,
    end: MachinePose,
    safe_z: f64,
    local: bool,
    config: TransferConfig,
    decimals: usize,
) -> Option<Vec<MachineMove>> {
    let distance = (end.x - start.x).hypot(end.y - start.y);
    if distance <= 1.0e-9 && (end.a - start.a).abs() <= 1.0e-9 {
        return None;
    }
    let step = 10.0_f64.powi(-(decimals as i32));
    let tolerance = config.smooth_tolerance - 3.0_f64.sqrt() * step * 0.5;
    if tolerance <= 0.0 {
        return None;
    }
    // Extend the protected vertical legs enough to cover output rounding at both ends.
    let above = |height: f64| ((height + step) / step).ceil() * step;
    let (retract_z, approach_z, peak, rise, fall) = if local {
        if !config.enabled || distance <= 1.0e-9 {
            return None;
        }
        (
            above(start.z + config.retract_height),
            above(end.z + config.approach_height),
            above(start.z.max(end.z) + config.total_height()),
            config.rise_ratio,
            config.fall_ratio,
        )
    } else {
        // A rotation is allowed only after the complete lift to this plane.
        // All Bezier controls stay on or above it; rounding cannot lower it below safe_z.
        let clearance = above(
            safe_z
                .max(start.z + config.retract_height)
                .max(end.z + config.approach_height),
        );
        (
            clearance,
            clearance,
            above(clearance + config.total_height()),
            0.5,
            0.5,
        )
    };
    if rise <= 1.0e-9 || fall <= 1.0e-9 || peak <= retract_z.max(approach_z) + step {
        return None;
    }
    let at = |t: f64, z: f64| MachinePose {
        x: start.x + (end.x - start.x) * t,
        y: start.y + (end.y - start.y) * t,
        z,
        a: start.a,
    };
    let up = peak - retract_z;
    let down = peak - approach_z;
    let ascent = [
        at(0.0, retract_z),
        at(0.0, retract_z + up / 5.0),
        at(0.0, retract_z + 2.0 * up / 5.0),
        at(3.0 * rise / 5.0, peak),
        at(4.0 * rise / 5.0, peak),
        at(rise, peak),
    ];
    let descent = [
        at(1.0 - fall, peak),
        at(1.0 - 4.0 * fall / 5.0, peak),
        at(1.0 - 3.0 * fall / 5.0, peak),
        at(1.0, approach_z + 2.0 * down / 5.0),
        at(1.0, approach_z + down / 5.0),
        at(1.0, approach_z),
    ];
    let plateau = std::array::from_fn(|i| lerp(ascent[5], descent[0], i as f64 / 5.0));
    // The shared parameter gives C2 joins even with unequal rise/fall proportions.
    // A uses one quintic ease across the entire upper transfer, with zero first/second
    // derivatives where it meets the protected constant-A vertical legs.
    let angular = [start, start, start, end, end, end];
    let mut moves = Vec::new();
    push_rapid(&mut moves, ascent[0]);
    for (mut controls, from, to) in [
        (ascent, 0.0, rise),
        (plateau, rise, 1.0 - fall),
        (descent, 1.0 - fall, 1.0),
    ] {
        if to - from <= 1.0e-9 {
            continue;
        }
        let (prefix, _) = split(angular, to);
        let (_, angles) = split(prefix, from / to);
        for (control, angle) in controls.iter_mut().zip(angles) {
            control.a = angle.a;
        }
        if !controls
            .iter()
            .all(|p| [p.x, p.y, p.z, p.a].iter().all(|v| v.is_finite()))
            || !flatten(controls, tolerance, step, 0, &mut moves)
        {
            return None;
        }
    }
    push_rapid(&mut moves, end);
    Some(moves)
}

fn lerp(a: MachinePose, b: MachinePose, t: f64) -> MachinePose {
    MachinePose {
        x: a.x + (b.x - a.x) * t,
        y: a.y + (b.y - a.y) * t,
        z: a.z + (b.z - a.z) * t,
        a: a.a + (b.a - a.a) * t,
    }
}

fn split(mut row: [MachinePose; 6], t: f64) -> ([MachinePose; 6], [MachinePose; 6]) {
    let mut left = [row[0]; 6];
    let mut right = [row[5]; 6];
    for level in 1..6 {
        for i in 0..6 - level {
            row[i] = lerp(row[i], row[i + 1], t);
        }
        left[level] = row[0];
        right[5 - level] = row[5 - level];
    }
    (left, right)
}

fn flatten(
    controls: [MachinePose; 6],
    tolerance: f64,
    step: f64,
    depth: usize,
    moves: &mut Vec<MachineMove>,
) -> bool {
    let start = controls[0];
    let end = controls[5];
    // Bounding every control's difference from the degree-elevated straight line
    // bounds the entire curve, rather than testing a few sample positions only.
    let flat = (end.a - start.a).abs() <= 2.0_f64.max(step * 2.0)
        && controls.iter().enumerate().all(|(i, p)| {
            let line = lerp(start, end, i as f64 / 5.0);
            (p.x - line.x).hypot(p.y - line.y).hypot(p.z - line.z) <= tolerance
                && (p.a - line.a).abs() <= 0.05_f64.max(step * 0.5)
        });
    if moves.len() >= 4096 {
        return false;
    }
    if flat {
        push_rapid(moves, end);
        return true;
    }
    if depth >= 12 {
        return false;
    }
    let (left, right) = split(controls, 0.5);
    flatten(left, tolerance, step, depth + 1, moves)
        && flatten(right, tolerance, step, depth + 1, moves)
}
