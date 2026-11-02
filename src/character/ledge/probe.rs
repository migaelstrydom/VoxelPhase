use nalgebra::{Point3, Unit, Vector3};
use traversal_planner::section::Footing;

use crate::physics::RigidBodyHandle;
use crate::traversal::{WorldCaster, WorldHit};

use super::config::LedgeConfig;
use super::edge::{Edge, WorldEdge};

/// Radius of the spheres the hands feel with, in metres.
const FINGER: f32 = 0.02;
/// How far under the lip the face is felt for, in metres: below the rounding
/// a voxel mesh gives a lip.
const FACE_DROP: f32 = 0.06;
/// Spacing of the casts down that look for a top ahead, in metres.
const TOP_STEP: f32 = 0.05;
/// How far a hand's top may lie from the lip first found, in metres: it is
/// felt for a few centimetres further in, and a top may rise or fall there.
const TOP_MATCH: f32 = 0.08;
/// How far the top under the body may lie from the lip, in metres.
const STAND_MATCH: f32 = 0.1;

/// Where a body looks for an edge: its capsule, and the way it faces.
#[derive(Debug, Clone, Copy)]
pub struct Reach {
    /// The capsule's centre.
    pub centre: Point3<f32>,
    /// Horizontal and unit length: the way the hands reach.
    pub heading: Vector3<f32>,
    /// The capsule's radius.
    pub radius: f32,
    /// Half the capsule's height, caps included.
    pub half_height: f32,
}

/// The edge at the hands of a body reaching as `reach` does, if both hands
/// find a lip on one body: grippable, and climbable if the top takes the
/// body standing. `docs/LEDGE_GRABS.md`, *The edge*.
pub fn edge_at_hands(caster: &WorldCaster, reach: &Reach, config: &LedgeConfig) -> Option<Edge> {
    let side = reach.heading.cross(&Vector3::y());
    let left = hand_hold(caster, reach, side * config.hand_offset, config)?;
    let right = hand_hold(caster, reach, -side * config.hand_offset, config)?;
    if left.body != right.body || (left.at.y - right.at.y).abs() > config.hand_step {
        return None;
    }
    let outward = Unit::try_new(left.outward + right.outward, 1.0e-3)?;
    let mut found = WorldEdge {
        hands: [left.at, right.at],
        outward,
        stand: None,
    };
    found.stand = stand_point(caster, reach, &found, config);
    Edge::from_world(caster.world, left.body, &found, left.footing)
}

/// Where one hand holds a lip.
struct HandHold {
    /// On the top, `grip_depth` behind the lip.
    at: Point3<f32>,
    /// Horizontal, out of the face.
    outward: Vector3<f32>,
    body: Option<RigidBodyHandle>,
    footing: Footing,
}

/// The lip one hand holds, felt for from the body's axis moved `offset`
/// across it: a top ahead within the catch band, a face under it, the top
/// again just behind the face, and open air over the hand.
fn hand_hold(
    caster: &WorldCaster,
    reach: &Reach,
    offset: Vector3<f32>,
    config: &LedgeConfig,
) -> Option<HandHold> {
    let origin = reach.centre + offset;
    let lowest = origin.y + config.catch_low;
    let highest = origin.y + config.catch_high;
    let grips =
        |hit: &WorldHit| hit.contact.normal.y >= config.grip_slope_degrees.to_radians().cos();

    let lip = first_top_ahead(caster, reach, origin, (lowest, highest), config, &grips)?;

    let face_from = Point3::new(origin.x, lip - FACE_DROP, origin.z);
    let face_length = reach.radius + config.reach_ahead + FINGER;
    let face = caster.cast(face_from, reach.heading, face_length, FINGER)?;
    let outward = horizontal(face.contact.normal)?;
    let steep = face.contact.normal.y < config.face_slope_degrees.to_radians().cos();
    if !steep || outward.dot(&reach.heading) >= 0.0 {
        return None;
    }

    let behind = face.contact.point - outward * config.grip_depth;
    let top_from = Point3::new(behind.x, lip + config.hand_clearance + FINGER, behind.z);
    let top = caster
        .cast(
            top_from,
            -Vector3::y(),
            config.hand_clearance + 2.0 * TOP_MATCH,
            FINGER,
        )
        .filter(|hit| grips(hit) && (hit.contact.point.y - lip).abs() <= TOP_MATCH)?;

    let above = Point3::new(
        origin.x,
        top.contact.point.y + config.hand_clearance,
        origin.z,
    );
    let to_hand = (face.contact.point - origin).dot(&reach.heading) + config.grip_depth - FINGER;
    if caster.cast(above, reach.heading, to_hand, FINGER).is_some() {
        return None;
    }

    Some(HandHold {
        at: top.contact.point,
        outward,
        body: top.contact.body,
        footing: top.footing,
    })
}

/// The height of the nearest top ahead of `origin`, within `reach_ahead` of
/// the capsule, that the fingers hold, between the `band`'s heights.
fn first_top_ahead(
    caster: &WorldCaster,
    reach: &Reach,
    origin: Point3<f32>,
    (lowest, highest): (f32, f32),
    config: &LedgeConfig,
    grips: &dyn Fn(&WorldHit) -> bool,
) -> Option<f32> {
    let steps = (config.reach_ahead / TOP_STEP).ceil() as usize;
    (0..=steps).find_map(|step| {
        let ahead = reach.radius + FINGER + step as f32 * TOP_STEP;
        let above = origin + reach.heading * ahead;
        let from = Point3::new(above.x, highest + FINGER, above.z);
        caster
            .cast(from, -Vector3::y(), highest - lowest, FINGER)
            .filter(|hit| grips(hit))
            .map(|hit| hit.contact.point.y)
    })
}

/// Where the body's centre stands on the top once pulled up: `stand_depth`
/// behind the lip, on a top it can stand on, with room overhead for it.
fn stand_point(
    caster: &WorldCaster,
    reach: &Reach,
    edge: &WorldEdge,
    config: &LedgeConfig,
) -> Option<Point3<f32>> {
    let outward = edge.outward.into_inner();
    let lip = edge.grip() + outward * config.grip_depth;
    let over = lip - outward * config.stand_depth;
    let drop = reach.radius + 2.0 * STAND_MATCH;
    let from = Point3::new(over.x, lip.y + reach.radius + STAND_MATCH, over.z);
    let floor = caster
        .cast(from, -Vector3::y(), drop, reach.radius)
        .filter(|hit| hit.contact.normal.y >= config.stand_slope_degrees.to_radians().cos())
        .filter(|hit| (hit.contact.point.y - lip.y).abs() <= STAND_MATCH)?;
    let column = 2.0 * (reach.half_height - reach.radius);
    if caster
        .cast(floor.centre, Vector3::y(), column, reach.radius)
        .is_some()
    {
        return None;
    }
    Some(floor.centre + Vector3::y() * (reach.half_height - reach.radius))
}

/// The horizontal part of `v`, unit length; `None` if `v` is upright.
fn horizontal(v: Vector3<f32>) -> Option<Vector3<f32>> {
    Vector3::new(v.x, 0.0, v.z).try_normalize(1.0e-3)
}
