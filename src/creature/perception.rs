use nalgebra::{Point3, Vector3};
use specs::{Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, WriteStorage};

use crate::components::{Position, Rotation};
use crate::damage::Dead;
use crate::player::Player;
use crate::terrain::TerrainWorld;
use crate::time::Time;

/// Spacing of the line-of-sight march, in metres.
///
/// The terrain exposes point solidity but no ray query, so sight is sampled
/// along the ray. Half a metre is coarse enough to be cheap at typical sight
/// ranges and fine enough that a creature cannot see through a wall — the
/// thinnest terrain feature is a voxel, and voxels are larger than this.
const LOS_STEP: f32 = 0.5;

/// What a creature can currently sense.
///
/// Split from [`Brain`](super::Brain) deliberately: perception answers "what is
/// out there", the brain answers "what to do about it". Keeping them apart
/// means a new behaviour reuses sensing unchanged, and perception can be
/// debug-drawn without instantiating a brain.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Perception {
    /// How far the creature can see, in metres.
    pub sight_range: f32,
    /// Cosine of the half-angle of the vision cone. 1.0 is a needle straight
    /// ahead, 0.0 is everything in front, -1.0 is all-round awareness.
    pub sight_cone_cos: f32,
    /// Radius within which the creature notices a target regardless of facing
    /// or cover — something right behind you is heard, not seen.
    pub hearing_range: f32,
    /// Seconds a lost target stays remembered before the creature gives up.
    /// Without this, a target stepping behind a rock erases itself instantly
    /// and creatures forget mid-chase.
    pub memory_duration: f32,

    /// The target being tracked, if any.
    pub target: Option<PerceivedTarget>,
}

/// A target the creature is aware of, whether or not it can see it right now.
#[derive(Debug, Clone, Copy)]
pub struct PerceivedTarget {
    pub entity: specs::Entity,
    /// Where the target was last perceived. While visible this is its live
    /// position; once lost it is the last known one, which is what makes a
    /// creature search where you were rather than where you are.
    pub position: Point3<f32>,
    /// Planar distance at the moment of the last perception.
    pub distance: f32,
    /// True when the target is perceivable this frame.
    pub visible: bool,
    /// Seconds since the target was last perceivable. Zero while visible.
    pub since_seen: f32,
}

impl Perception {
    /// A ground creature: a wide forward cone plus close all-round hearing.
    pub fn ground_creature(sight_range: f32) -> Self {
        Self {
            sight_range,
            sight_cone_cos: (70.0f32.to_radians()).cos(),
            hearing_range: 6.0,
            memory_duration: 5.0,
            target: None,
        }
    }

    /// True when a target is currently perceivable.
    pub fn sees_target(&self) -> bool {
        self.target.is_some_and(|t| t.visible)
    }
}

/// Fills [`Perception`] from the world each frame.
///
/// Targets are currently "the player", which is the only hostile thing in the
/// world. When factions arrive this is the one system that has to change —
/// brains ask perception who the target is and never search for one themselves.
pub struct PerceptionSystem;

impl<'a> System<'a> for PerceptionSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        Option<Read<'a, TerrainWorld>>,
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Rotation>,
        ReadStorage<'a, Dead>,
        WriteStorage<'a, Perception>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time, terrain, players, positions, rotations, deads, mut perceptions) = data;
        let dt = time.delta_seconds();

        // A dead player stops being a threat, so creatures lose interest
        // rather than crowding a corpse.
        let candidates: Vec<(specs::Entity, Point3<f32>)> =
            (&entities, &players, &positions, !&deads)
                .join()
                .map(|(entity, _, pos, _)| (entity, Point3::from(pos.0)))
                .collect();

        for (pos, rot, perception, _) in (&positions, &rotations, &mut perceptions, !&deads).join()
        {
            let eye = Point3::from(pos.0);
            let facing = Vector3::new(rot.0.sin(), 0.0, rot.0.cos());

            let perceived = candidates
                .iter()
                .filter_map(|&(entity, target_pos)| {
                    let offset = Vector3::new(target_pos.x - eye.x, 0.0, target_pos.z - eye.z);
                    let distance = offset.magnitude();
                    perceivable(
                        perception,
                        eye,
                        facing,
                        target_pos,
                        offset,
                        distance,
                        terrain.as_deref(),
                    )
                    .then_some((entity, target_pos, distance))
                })
                .min_by(|a, b| a.2.total_cmp(&b.2));

            perception.target = match (perceived, perception.target) {
                // Perceivable now: track it live.
                (Some((entity, position, distance)), _) => Some(PerceivedTarget {
                    entity,
                    position,
                    distance,
                    visible: true,
                    since_seen: 0.0,
                }),
                // Lost, but still within memory: hold the last known position
                // so the creature searches where the target was.
                (None, Some(mut remembered)) => {
                    remembered.visible = false;
                    remembered.since_seen += dt;
                    (remembered.since_seen < perception.memory_duration).then_some(remembered)
                }
                (None, None) => None,
            };
        }
    }
}

/// Whether a target at `target_pos` can be sensed from `eye`.
///
/// Hearing is checked first and ignores both facing and cover: a target close
/// enough is noticed however it approaches.
fn perceivable(
    perception: &Perception,
    eye: Point3<f32>,
    facing: Vector3<f32>,
    target_pos: Point3<f32>,
    offset: Vector3<f32>,
    distance: f32,
    terrain: Option<&TerrainWorld>,
) -> bool {
    if distance <= perception.hearing_range {
        return true;
    }
    if distance > perception.sight_range {
        return false;
    }
    let Some(direction) = offset.try_normalize(1e-4) else {
        return true;
    };
    if direction.dot(&facing) < perception.sight_cone_cos {
        return false;
    }
    has_line_of_sight(eye, target_pos, terrain)
}

/// March the segment between two points looking for solid terrain.
///
/// Endpoints are skipped: the eye is inside its own body and the target's
/// centre may sit inside terrain it is standing on, and neither should count
/// as an obstruction.
fn has_line_of_sight(from: Point3<f32>, to: Point3<f32>, terrain: Option<&TerrainWorld>) -> bool {
    let Some(terrain) = terrain else {
        return true;
    };
    let segment = to - from;
    let length = segment.magnitude();
    let Some(direction) = segment.try_normalize(1e-4) else {
        return true;
    };

    let mut travelled = LOS_STEP;
    while travelled < length - LOS_STEP {
        let sample = from + direction * travelled;
        if terrain.is_solid_at(sample.x, sample.y, sample.z) {
            return false;
        }
        travelled += LOS_STEP;
    }
    true
}
