//! Where water goes over an edge, and how it leaves it: the rules a falling
//! stream and a drawn falling sheet share, so what is drawn is what pushes.

use nalgebra::Vector2;

use crate::water::geometry::Step;

/// Gravity, m/s².
pub const GRAVITY: f32 = 9.81;

/// Least step, m, at which water standing on a floor above the water below
/// it falls rather than slopes: steeper than 45° across a column.
const FALL: f64 = 0.5;

/// Least height, m, the floor the water leaves must stand over the water or
/// the ground below for there to be an edge to go over. Over less, a deep
/// front running out over ground a hand lower would be taken for a fall on
/// one column and a slope on the next.
const LEDGE: f64 = 0.2;

/// How far water falling into the void, off the world's edge, is followed,
/// m: until it has broken up in the air, as `SHEET_BREAKUP` in
/// `shader/water.frag` draws it. Nothing catches it, so there is nowhere
/// else for it to end.
pub const VOID_DROP: f64 = 30.0;

/// How fast water is thrown out over a lip when it barely moves, m/s, so
/// even a trickle clears the face it falls down.
const LIP_SPEED: f32 = 0.8;

/// The fastest water is taken to leave a lip, m/s.
const MAX_LAUNCH: f32 = 14.0;

/// Whether water at `upper` on a bed at `bed` goes over an edge to water at
/// `lower` (the ground itself, where that is dry), falling between them
/// rather than sloping: a step over `FALL` where the upper water's own floor
/// stands a `LEDGE` above the lower water, which a slope between them would
/// run through. However high the step, water whose floor lies below the
/// water beside it has no edge to go over: it is a front running out over a
/// floor, as where a breach floods a tunnel, and slopes down to the water
/// ahead of it.
pub fn falls(upper: f64, bed: f64, lower: f64) -> bool {
    upper - lower > FALL && bed - lower > LEDGE
}

/// The water a surface at `level` falls to off the world's edge: the sea's
/// level `outside`, or `VOID_DROP` below where there is none.
pub fn off_map(level: f64, outside: Option<f64>) -> f64 {
    outside.unwrap_or(level - VOID_DROP)
}

/// The level velocity water leaves a lip with, going out over an edge
/// facing `face` at `out` m/s (its discharge over its depth there) from
/// water moving at `flow`: out over the edge at that speed, and along the
/// lip as the water behind it drifts, no faster than it goes out. A trickle
/// is pushed out towards `LIP_SPEED`, so that it leans clear of rock that
/// leans out below the lip.
pub fn launch(face: Vector2<f32>, out: f32, flow: Vector2<f32>) -> Vector2<f32> {
    let out = out.max(0.0);
    let out = out + (LIP_SPEED - out).max(0.0) * 0.5;
    let edge = Vector2::new(-face.y, face.x);
    let velocity = face * out + edge * flow.dot(&edge).clamp(-out, out);
    let speed = velocity.norm();
    if speed > MAX_LAUNCH {
        velocity * (MAX_LAUNCH / speed)
    } else {
        velocity
    }
}

/// How long water takes to fall `fall` metres from rest, s.
pub fn fall_time(fall: f32) -> f32 {
    (2.0 * fall.max(0.0) / GRAVITY).sqrt()
}

/// The two ends of a column's edge across `step`, as the sides of the
/// neighbouring column they lie on.
pub fn edge_ends(step: Step) -> [(i32, i32); 2] {
    match (step.di, step.dk) {
        (1, _) => [(-1, -1), (-1, 1)],
        (-1, _) => [(1, -1), (1, 1)],
        (_, 1) => [(-1, -1), (1, -1)],
        _ => [(-1, 1), (1, 1)],
    }
}

/// The step along a column's edge across `step` towards its end on the
/// neighbouring column's `sides` (`edge_ends`).
pub fn along_edge(step: Step, (sx, sz): (i32, i32)) -> Step {
    if step.di != 0 {
        Step { di: 0, dk: sz }
    } else {
        Step { di: sx, dk: 0 }
    }
}

/// How far past the edge into the lower column across `step` its floor
/// stands above `ledge`, in columns, along the row of its `quarters` from
/// the end of the edge on the lower column's `sides`: where the ledge the
/// water falls from really ends.
pub fn ledge_reach(step: Step, (sx, sz): (i32, i32), quarters: &[f32; 4], ledge: f32) -> f32 {
    let near = (sx > 0) as usize + 2 * (sz > 0) as usize;
    let far = near ^ if step.di != 0 { 1 } else { 2 };
    match (quarters[near] > ledge, quarters[far] > ledge) {
        (false, _) => 0.0,
        (true, false) => 0.5,
        (true, true) => 1.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Water falling into the void is followed as far as the shader draws
    /// it before it has broken up in the air.
    #[test]
    fn the_void_drop_is_where_the_shader_has_broken_falling_water_up() {
        let glsl =
            std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/shader/water.frag"))
                .expect("shader/water.frag");
        let breakup = glsl
            .split_once("SHEET_BREAKUP = vec2(")
            .and_then(|(_, rest)| rest.split_once(')'))
            .map(|(pair, _)| pair)
            .expect("water.frag declares SHEET_BREAKUP");
        let gone: f64 = breakup
            .split(',')
            .nth(1)
            .and_then(|end| end.trim().parse().ok())
            .expect("SHEET_BREAKUP's end");
        assert_eq!(gone, VOID_DROP);
    }

    #[test]
    fn water_on_a_floor_above_the_water_below_falls_to_it() {
        assert!(falls(3.1, 3.0, 2.0));
        assert!(!falls(2.4, 2.3, 2.0), "a step under FALL slopes");
    }

    #[test]
    fn a_front_on_a_floor_below_the_water_ahead_slopes_however_high() {
        // A breach's water 1.4 m deep beside a film on the same floor.
        assert!(!falls(0.65, -0.75, -0.74));
    }

    #[test]
    fn deep_water_over_ground_a_hand_lower_slopes() {
        assert!(!falls(3.6, 3.0, 2.95));
    }

    #[test]
    fn a_trickle_is_pushed_out_and_a_torrent_leaves_as_it_runs() {
        let face = Vector2::x();
        let trickle = launch(face, 0.0, Vector2::zeros());
        assert!((trickle - face * 0.5 * LIP_SPEED).norm() < 1.0e-6);
        let torrent = launch(face, 4.0, Vector2::new(3.0, 1.5));
        assert!((torrent - Vector2::new(4.0, 1.5)).norm() < 1.0e-6);
        let spilling = launch(face, 1.0, Vector2::new(0.0, 3.0));
        assert!((spilling - Vector2::new(1.0, 1.0)).norm() < 1.0e-6);
    }
}
