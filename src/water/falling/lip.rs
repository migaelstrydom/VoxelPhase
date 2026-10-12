//! Where water goes over an edge, and how it leaves it: the rules a falling
//! stream and a drawn falling sheet share, so what is drawn is what pushes.

use nalgebra::Vector2;

use crate::water::geometry::Step;

/// Gravity, m/s².
pub const GRAVITY: f32 = 9.81;

/// Surfaces further apart than this do not share a corner, m: a step this
/// high between two wet cells is water going over an edge. Lower steps are
/// water running down a slope, up to 200 %, unless the water `falls`.
pub const JOIN: f64 = 1.0;

/// Least step, m, at which water standing on a floor above the water below
/// it falls rather than slopes: steeper than 45° across a column.
const FALL: f64 = 0.5;

/// How far water falling into the void past the map's edge is taken to
/// fall, m.
const VOID_DROP: f64 = 6.0;

/// How fast water is thrown out over a lip when it barely moves, m/s, so
/// even a trickle clears the face it falls down.
const LIP_SPEED: f32 = 0.8;

/// The furthest water is thrown out from its lip by the time it lands, m.
const MAX_THROW: f32 = 1.6;

/// Whether water at `upper` on a bed at `bed` goes over an edge to water at
/// `lower`, falling between them rather than sloping: a step over `JOIN`, or
/// over `FALL` where the upper water's own floor stands above the lower
/// water, which a slope between them would run through.
pub fn falls(upper: f64, bed: f64, lower: f64) -> bool {
    let step = upper - lower;
    step > JOIN || (step > FALL && bed > lower)
}

/// The water a surface at `level` falls to off the map's edge: the sea's
/// level `outside`, or `VOID_DROP` below where there is none.
pub fn off_map(level: f64, outside: Option<f64>) -> f64 {
    outside.unwrap_or(level - VOID_DROP)
}

/// How far out from its lip water going over an edge facing `face` lands,
/// leaving it with the lip's `flow` and falling `fall` metres in
/// sqrt(2 fall / g) seconds: it leans out clear of rock that leans out below
/// the lip.
///
/// It is thrown the way its water moves, which a lip's corner shares with
/// the edges beside it: a lip running across the lattice is a staircase of
/// edges facing two ways, and thrown each its own way their water would
/// cross. The face only keeps it going out.
pub fn thrown(face: Vector2<f32>, flow: Vector2<f32>, fall: f32) -> Vector2<f32> {
    if fall <= 0.0 {
        return Vector2::zeros();
    }
    let velocity = flow + face * (LIP_SPEED - flow.dot(&face)).max(0.0) * 0.5;
    let speed = velocity.norm();
    let direction = if speed > 1.0e-4 {
        velocity / speed
    } else {
        face
    };
    direction * (speed * (2.0 * fall / GRAVITY).sqrt()).min(MAX_THROW)
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
