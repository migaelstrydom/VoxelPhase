use nalgebra::{Point3, Unit, Vector3};
use traversal_planner::section::Footing;

use crate::physics::RigidBodyHandle;
use crate::traversal::{WorldCaster, WorldHit};

use super::config::LedgeConfig;
use super::edge::{Edge, WorldEdge};

/// Radius of the spheres the hands feel with, in metres.
const FINGER: f32 = 0.02;
/// How far under the lip the face is felt for, in metres, nearest first:
/// just under a sharp lip (a free slab's), and further down below the bevel
/// a voxel mesh cuts on a lip, about a voxel high.
const FACE_DROPS: [f32; 3] = [0.06, 0.15, 0.3];
/// How far beyond the face the top may begin, in metres: a voxel mesh's
/// bevel on a lip, about a voxel wide.
const BEVEL: f32 = 0.3;
/// Spacing of the casts down that look for a top ahead, in metres.
const TOP_STEP: f32 = 0.05;
/// How far a hand's top may lie from the lip first found, in metres: it is
/// felt for a few centimetres further in, and a top may rise or fall there.
const TOP_MATCH: f32 = 0.08;
/// How far the top under the body may lie below the lip, in metres.
const STAND_BELOW: f32 = 0.1;
/// How far the top under the body may rise above the lip, in metres: a voxel
/// mesh rounds a lip over about a voxel, so the hands find it lower than the
/// top behind it.
const STAND_ABOVE: f32 = 0.3;

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
    /// The body's mass, in kg: only a body that outweighs it
    /// `bearing_mass_ratio` times over, or does not move, bears a hold.
    pub mass: f32,
}

/// The edge at the hands of a body reaching as `reach` does, if both hands
/// find a lip on one body, from a mantle's lowest lip to the highest a hand
/// catches: grippable, and climbable if the top takes the body standing.
/// `docs/LEDGE_GRABS.md`, *The edge*.
pub fn edge_at_hands(caster: &WorldCaster, reach: &Reach, config: &LedgeConfig) -> Option<Edge> {
    let side = reach.heading.cross(&Vector3::y());
    let left = hand_hold(caster, reach, side * config.hand_offset, config)?;
    let right = hand_hold(caster, reach, -side * config.hand_offset, config)?;
    if left.body != right.body || (left.at.y - right.at.y).abs() > config.hand_step {
        return None;
    }
    if !bears(caster, left.body, reach.mass, config) {
        return None;
    }
    let outward = Unit::try_new(left.outward + right.outward, 1.0e-3)?;
    let mut found = WorldEdge {
        hands: [left.at, right.at],
        outward,
        face_depth: 0.5 * (left.face_depth + right.face_depth),
        stand: None,
    };
    found.stand = stand_point(caster, reach, &found, config);
    Edge::from_world(caster.world, left.body, &found, left.footing)
}

/// Whether `body` bears a hanger of `mass`: static, or heavy enough that the
/// hanger's weight and its pulls barely move it. A crate lighter than that
/// would only be dragged about by the grip, as a cardboard box would crumple.
fn bears(
    caster: &WorldCaster,
    body: Option<RigidBodyHandle>,
    mass: f32,
    config: &LedgeConfig,
) -> bool {
    body.and_then(|handle| caster.world.body(handle))
        .is_none_or(|body| !body.is_dynamic() || body.mass() >= mass * config.bearing_mass_ratio)
}

/// Where one hand holds a lip.
struct HandHold {
    /// On the top, at least `grip_depth` behind the face.
    at: Point3<f32>,
    /// Horizontal, out of the face.
    outward: Vector3<f32>,
    /// How far behind the face the hand holds, in metres: more than
    /// `grip_depth` where the lip is bevelled.
    face_depth: f32,
    body: Option<RigidBodyHandle>,
    footing: Footing,
}

/// The lip one hand holds, felt for from the body's axis moved `offset`
/// across it: a top ahead within the catch band, a face under it (below a
/// bevel, if the lip has one), a hold on the top `grip_depth` behind the face
/// or where the top begins, whichever is further, and open air over the
/// hand.
fn hand_hold(
    caster: &WorldCaster,
    reach: &Reach,
    offset: Vector3<f32>,
    config: &LedgeConfig,
) -> Option<HandHold> {
    let origin = reach.centre + offset;
    let lowest = origin.y + config.catch_low.min(config.mantle_low);
    let highest = origin.y + config.catch_high;
    let grips =
        |hit: &WorldHit| hit.contact.normal.y >= config.grip_slope_degrees.to_radians().cos();
    let along = |p: Point3<f32>| (p - origin).dot(&reach.heading);

    let top = first_top_ahead(caster, reach, origin, (lowest, highest), config, &grips)?;
    let lip = top.contact.point.y;
    let top_at = along(top.contact.point);

    let face_length = reach.radius + config.reach_ahead + FINGER;
    let steep = config.face_slope_degrees.to_radians().cos();
    let face = FACE_DROPS.iter().find_map(|drop| {
        let from = Point3::new(origin.x, lip - drop, origin.z);
        caster
            .cast(from, reach.heading, face_length, FINGER)
            .filter(|hit| hit.contact.normal.y < steep)
            .filter(|hit| along(hit.contact.point) <= top_at + FINGER)
    })?;
    let outward = horizontal(face.contact.normal)?;
    if outward.dot(&reach.heading) >= 0.0 {
        return None;
    }
    let face_at = along(face.contact.point);

    let hold_at = (face_at + config.grip_depth).max(top_at);
    let behind = origin + reach.heading * hold_at;
    let top_from = Point3::new(behind.x, lip + config.hand_clearance + FINGER, behind.z);
    let hold = caster
        .cast(
            top_from,
            -Vector3::y(),
            config.hand_clearance + 2.0 * TOP_MATCH,
            FINGER,
        )
        .filter(|hit| grips(hit) && (hit.contact.point.y - lip).abs() <= TOP_MATCH)?;

    let above = Point3::new(
        origin.x,
        hold.contact.point.y + config.hand_clearance,
        origin.z,
    );
    if caster
        .cast(above, reach.heading, hold_at - FINGER, FINGER)
        .is_some()
    {
        return None;
    }

    Some(HandHold {
        at: hold.contact.point,
        outward,
        face_depth: hold_at - face_at,
        body: hold.contact.body,
        footing: hold.footing,
    })
}

/// The nearest top ahead of `origin` that the fingers hold, between the
/// `band`'s heights, followed up to its crest: from the capsule out to
/// `reach_ahead`, and a bevel's width beyond. A bevelled lip's shallow slope
/// is held where it levels off, not wherever it first enters the band, which
/// would move with the body.
fn first_top_ahead(
    caster: &WorldCaster,
    reach: &Reach,
    origin: Point3<f32>,
    (lowest, highest): (f32, f32),
    config: &LedgeConfig,
    grips: &dyn Fn(&WorldHit) -> bool,
) -> Option<WorldHit> {
    let steps = ((config.reach_ahead + BEVEL) / TOP_STEP).ceil() as usize;
    let top_at = |step: usize| {
        let ahead = reach.radius + FINGER + step as f32 * TOP_STEP;
        let above = origin + reach.heading * ahead;
        let from = Point3::new(above.x, highest + FINGER, above.z);
        caster
            .cast(from, -Vector3::y(), highest - lowest, FINGER)
            .filter(|hit| grips(hit))
    };
    let (first, mut top) = (0..=steps).find_map(|step| top_at(step).map(|hit| (step, hit)))?;
    for step in first + 1..=steps {
        match top_at(step) {
            Some(next) if next.contact.point.y > top.contact.point.y + CREST_RISE => top = next,
            _ => break,
        }
    }
    Some(top)
}

/// How much a top must rise from one look to the next to be still climbing
/// to its crest, in metres.
const CREST_RISE: f32 = 0.01;

/// Where the body's centre stands on the top once pulled up: `stand_depth`
/// behind the lip, on a top it can stand on, with room overhead for it.
fn stand_point(
    caster: &WorldCaster,
    reach: &Reach,
    edge: &WorldEdge,
    config: &LedgeConfig,
) -> Option<Point3<f32>> {
    let outward = edge.outward.into_inner();
    let face = edge.grip() + outward * edge.face_depth;
    let over = face - outward * config.stand_depth;
    let from = Point3::new(over.x, face.y + STAND_ABOVE + reach.radius, over.z);
    let floor = caster
        .cast(from, -Vector3::y(), STAND_ABOVE + STAND_BELOW, reach.radius)
        .filter(|hit| hit.contact.distance > 0.0)
        .filter(|hit| hit.contact.normal.y >= config.stand_slope_degrees.to_radians().cos())?;
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
