//! What a foot's probe says of the floor.
//!
//! Each foot has one probe, cast at where the foot stands or is about to
//! land (`PlacerFoot::probe_anchor`), which reports a contact and its normal
//! (`PlacerCtx::left_ground`, `left_ground_normal` and their right-hand
//! twins). Everything the placer knows of the ground's height comes through
//! here:
//!
//! ```text
//!   probe contact ──► is it a floor? ──► floor_sample
//!                     (is_floor)             │
//!                                            ▼
//!            its plane, carried no further than PROBE_REACH
//!                     (floor_height_near)
//! ```
//!
//! A probe that struck a wall, or nothing, tells no height; which of the
//! two it was is `probe_found_geometry`.

use nalgebra::{Point3, Vector2, Vector3};

use super::placer::{FootSide, PlacerCtx};

/// How far from where a foot's probe struck its floor plane still tells the
/// floor's height, in metres.
const PROBE_REACH: f32 = 0.6;

/// Least `normal.y` of a face a foot may stand on: about 66° from level.
///
/// Physics sets no slope limit on support, so a body can be held up by
/// faces far steeper than a comfortable walk: wedged in the crease of a V
/// channel, each face it stands on is near 57°. A cut-off below what the
/// body can stand on leaves its feet with no floor anywhere, and they hang
/// in a swing that never lands. Only faces past this are walls: crate
/// sides, cliff faces, a creature's flank.
const FLOOR_MIN_NORMAL_Y: f32 = 0.4;

/// Terrain contact + normal for the given foot, when the probe has hit
/// floor-like geometry (upward-facing normal). Wall hits are not
/// landing surfaces and must not steer foot heights.
pub(super) fn floor_sample(
    ctx: &PlacerCtx<'_>,
    side: FootSide,
) -> Option<(Point3<f32>, Vector3<f32>)> {
    let (contact, normal) = match side {
        FootSide::Left => (ctx.left_ground, ctx.left_ground_normal),
        FootSide::Right => (ctx.right_ground, ctx.right_ground_normal),
    };
    contact.filter(|_| is_floor(normal)).map(|c| (c, normal))
}

/// The floor's height at `(x, z)`, carried along the plane this foot's
/// probe found, where that is near enough to where it struck to tell. A
/// plane carried further is a guess: off a slope's crest it puts the shelf
/// half a metre up, and a probe still aimed where the foot was a moment ago
/// puts the floor under it anywhere.
pub(super) fn floor_height_near(
    ctx: &PlacerCtx<'_>,
    side: FootSide,
    x: f32,
    z: f32,
) -> Option<f32> {
    plane_height_near(floor_sample(ctx, side)?, x, z)
}

/// The height at `(x, z)` of the floor plane a probe struck, where that is
/// near enough to where it struck to tell. See [`floor_height_near`].
pub(super) fn plane_height_near(
    (contact, normal): (Point3<f32>, Vector3<f32>),
    x: f32,
    z: f32,
) -> Option<f32> {
    let across = Vector2::new(x - contact.x, z - contact.z).magnitude();
    (across <= PROBE_REACH).then(|| floor_height_at(contact, normal, x, z))
}

/// Whether this foot's probe found any geometry at all, floor-like or not.
///
/// [`floor_sample`] answers a different question — *is there a surface here
/// I could stand on* — and collapses "nothing out there" and "a wall face"
/// into the same `None`. Only the first is evidence that the landing target
/// has no ground under it. The second is a probe that struck something
/// upright on its way down: a crate, a creature, the player. Probes are cast
/// against the physics world as well as the terrain, so for a creature that
/// walks up to the player and steps around it, that happens constantly.
pub(super) fn probe_found_geometry(ctx: &PlacerCtx<'_>, side: FootSide) -> bool {
    match side {
        FootSide::Left => ctx.left_ground.is_some(),
        FootSide::Right => ctx.right_ground.is_some(),
    }
}

/// Whether a face with this normal is one a foot may stand on.
pub(crate) fn is_floor(normal: Vector3<f32>) -> bool {
    normal.y > FLOOR_MIN_NORMAL_Y
}

/// Height of the floor plane through `contact` with the given normal,
/// evaluated at `(x, z)`. Exact for uniform slopes; for flat treads
/// (normal = +Y) it degenerates to `contact.y`.
pub(crate) fn floor_height_at(contact: Point3<f32>, normal: Vector3<f32>, x: f32, z: f32) -> f32 {
    contact.y
        - (normal.x * (x - contact.x) + normal.z * (z - contact.z))
            / normal.y.max(FLOOR_MIN_NORMAL_Y)
}
