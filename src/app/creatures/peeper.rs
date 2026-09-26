//! Peeper — a one-eyed stalker that walks on stilts.
//!
//! The counterweight to the roller. A roller is heavy, blind-ish, and
//! unkillable; you deal with one by using the terrain. A peeper is the
//! opposite on every axis: it is fragile, it sees a long way, and it can
//! be killed outright — but it is *tall*, which means it spots you across
//! ground a roller would never see you over, and its neck reaches further
//! than its body suggests.
//!
//! Its one trick is the eye. A peeper with nothing to look at stands
//! dozing with its lid shut, which makes it a landmark rather than a
//! threat: the player can creep past one, and the snap of the eye opening
//! is the whole alert beat. That is why it carries no
//! [`AlertTelegraph`](crate::creature::AlertTelegraph) — the hop-and-
//! shiver exists for creatures with no face to pull, and this one is
//! mostly face.
//!
//! Everything below the eye is shared: it walks on the same
//! [`LeggedLocomotion`](crate::animation::LeggedLocomotion) as the player
//! and the heart critter, through the same character control chain, driven
//! by the same [`Brain`] as the roller.

use nalgebra::{Point3, UnitVector3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use crate::animation::peeper::{PeeperAnimator, PeeperRigConfig};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::character::{CharacterIntent, CharacterState, Grounding, LocomotionConfig};
use crate::components::{
    Orientation, Position, Renderable, RigidBodyComponent, Rotation, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::{Brain, MeleeAttack, Perception};
use crate::damage::{Health, Ragdoll};
use crate::drive::{Actuator, Allowance, BodyMotion, DriveIntent};
use crate::physics::{
    BulkShape, ColliderDesc, ColliderShape, ConstraintKind, FrictionModel, RigidBodyDesc, Volume,
};
use crate::rendering::material::MaterialId;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Spindly. A peeper is legs and air, and a blast should send it
/// cartwheeling — being thrown about is most of what makes it fun to
/// fight.
const DENSITY: f32 = 300.0;

/// Yaw authority, in rad/s². Below the heart critter's: a peeper pivots
/// deliberately, so circling it is a real option.
const TURN_AUTHORITY: f32 = 420.0;

/// Half-width of the thing it is pecking at. The attack has to cover the
/// target's own girth or a peeper standing against the player misses.
const TARGET_HALF_WIDTH: f32 = 0.45;

/// Where a peeper holds station while attacking, between the distance at
/// which the two bodies touch and the distance its peck reaches.
///
/// Halfway, and both bounds matter. Any closer and the solver is what ends
/// up deciding the standoff — the creature walks into the player, gets
/// shoved out, and walks back in. Any further and the peck cannot land
/// from where the brain parks it.
const STANDOFF_BETWEEN_CONTACT_AND_REACH: f32 = 0.5;

/// Seconds of wind-up before a peck lands, and of recovery after. The
/// wind-up is long on purpose: it is the warning, and the head rearing
/// back over the haunch is unmistakable.
const PECK_WINDUP: f32 = 0.5;
const PECK_RECOVERY: f32 = 0.85;

/// A stalking creature that pecks.
#[derive(Deserialize)]
pub struct PeeperDef {
    /// Position (x, z). Y comes from the terrain surface, since a creature
    /// authored at a hand-picked height would either hang in the air or
    /// start embedded once the terrain around it is reshaped.
    pub pos: (f32, f32),

    /// Flat-ground top speed in m/s. The player walks at 5.0 and sprints
    /// at 8.0, so a peeper is outrunnable — it is a thing you leave
    /// behind, not a thing you escape.
    #[serde(default = "PeeperDef::default_speed")]
    pub speed: f32,
    /// How far it can see. Long: being tall is the creature's advantage.
    #[serde(default = "PeeperDef::default_sight_range")]
    pub sight_range: f32,
    /// Hit points a peck takes off.
    #[serde(default = "PeeperDef::default_peck_damage")]
    pub peck_damage: f32,
    /// Hit points it can take before it dies.
    #[serde(default = "PeeperDef::default_health")]
    pub health: f32,
}

impl PeeperDef {
    pub fn default_speed() -> f32 {
        3.4
    }
    pub fn default_sight_range() -> f32 {
        28.0
    }
    pub fn default_peck_damage() -> f32 {
        14.0
    }
    pub fn default_health() -> f32 {
        55.0
    }
}

/// What water and wind see of a peeper: two stilts, a haunch, a neck and an
/// eye, and not the air between them that the capsule round it also holds.
///
/// It weighs what the capsule says, which is far more than this displaces,
/// so it wades on its stilts until the water reaches its eye, and past that
/// walks the bottom. The legs are straight and the neck upright: a rest pose
/// is all a rigid body can carry.
fn stilts_and_eye(rig: &PeeperRigConfig, clearance: f32) -> BulkShape {
    // Heights above the foot centres, which sit `clearance` below the
    // capsule's centre.
    let hip = rig.standing_height();
    let shoulder = hip + rig.haunch_height;
    let eye = shoulder + rig.neck_length;
    let at = |height: f32, across: f32| Vector3::new(across, height - clearance, 0.0);
    let capsule = |length: f32, radius: f32| ColliderShape::Capsule {
        half_height: 0.5 * length + radius,
        radius,
    };
    let stilt = capsule(hip, rig.leg_radius);
    BulkShape::displacing(vec![
        Volume::at(stilt.clone(), at(0.5 * hip, -0.5 * rig.hip_width)),
        Volume::at(stilt, at(0.5 * hip, 0.5 * rig.hip_width)),
        Volume::at(
            capsule(rig.haunch_height, rig.haunch_radius),
            at(hip + 0.5 * rig.haunch_height, 0.0),
        ),
        Volume::at(
            capsule(rig.neck_length, rig.neck_radius),
            at(shoulder + 0.5 * rig.neck_length, 0.0),
        ),
        Volume::at(
            ColliderShape::Sphere {
                radius: rig.eye_radius,
            },
            at(eye, 0.0),
        ),
    ])
}

impl Spawnable for PeeperDef {
    fn material_count(&self) -> usize {
        0
    }

    fn create_materials(&self, _ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // The rig is vertex-coloured — the hide, the eye and the feelers
        // are all in the mesh the animator regenerates each frame, which no
        // material can be attached to anyway.
        Ok(Vec::new())
    }

    fn spawn(&self, world: &mut World, _materials: &[MaterialId]) -> Vec<Entity> {
        let rig = PeeperRigConfig::default();
        // The capsule covers the whole creature, so its centre — and with
        // it the character's ground clearance — sits at half its height.
        let clearance = rig.body_height() * 0.5;
        let radius = rig.eye_radius;
        let locomotion = LocomotionConfig::creature(self.speed, radius, clearance);
        let (air_steer_speed, jump_speed) = (locomotion.air_steer_speed, locomotion.jump_speed);

        let surface_y = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };
        // No terrain beneath the authored position means the level moved
        // and this creature's spot went with it. Dropping it silently beats
        // spawning something that falls forever.
        let Some(surface_y) = surface_y else {
            return Vec::new();
        };

        let initial_pos = Point3::new(self.pos.0, surface_y + clearance, self.pos.1);
        let animator = PeeperAnimator::new(rig, initial_pos, clearance, 0.0);

        // Reach is the rig's own geometry plus the target's girth, so
        // lengthening the neck lengthens the attack by itself. The standoff
        // the brain holds is derived from the same two numbers, which is
        // what keeps the creature out of the player's collider while still
        // close enough to connect.
        let contact = radius + TARGET_HALF_WIDTH;
        let reach = rig.strike_extent() + TARGET_HALF_WIDTH;
        let standoff = contact + (reach - contact) * STANDOFF_BETWEEN_CONTACT_AND_REACH;
        let attack =
            MeleeAttack::new(self.peck_damage, reach).with_timing(PECK_WINDUP, PECK_RECOVERY);

        let (body_handle, keep_upright) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .bulk(stilts_and_eye(&rig, clearance))
                    .position(initial_pos)
                    .gravity_scale(1.0)
                    .linear_damping(0.0)
                    .angular_damping(0.95),
            );
            physics.world.attach_collider(
                body_handle,
                ColliderDesc::capsule(clearance, radius)
                    .density(DENSITY)
                    .restitution(0.0)
                    .friction_model(FrictionModel::Isotropic(0.8)),
            );
            // A capsule has nothing to resist a torque about its long axis,
            // so without this it topples the first time it clips a rock.
            // Released on death, which is what lets the legs fold.
            let keep_upright = physics
                .world
                .create_constraint(ConstraintKind::KeepUpright {
                    body: body_handle,
                    target_up: UnitVector3::new_normalize(Vector3::y()),
                    compliance: 0.0,
                    max_impulse: f32::INFINITY,
                });
            (body_handle, keep_upright)
        };

        vec![world
            .create_entity()
            // Intent is the seam: `BrainSystem` writes it, the shared
            // character control chain reads it. Neither knows about the
            // other, and neither knows this one has an eye for a head.
            .with(CharacterIntent::default())
            .with(CharacterState::default())
            .with(locomotion)
            .with(Grounding::default())
            .with(animator)
            .with(Position(initial_pos.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Rotation(0.0))
            .with(Orientation::default())
            .with(Renderable)
            .with(SensorSet::default())
            .with(ContactCandidates::default())
            .with(RigidBodyComponent(body_handle))
            .with(
                Actuator::character()
                    .with_non_support_grip(0.0)
                    .with_drive_gain(5.0)
                    .with_allowance(Allowance::character(
                        air_steer_speed,
                        TURN_AUTHORITY,
                        jump_speed,
                    )),
            )
            .with(DriveIntent::default())
            .with(BodyMotion::default())
            .with(Perception::ground_creature(self.sight_range))
            .with(Brain::hunter(standoff))
            .with(attack)
            .with(Health::new(self.health, 6.0))
            .with(Ragdoll::new(vec![keep_upright]))
            .build()]
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::physics::{PhysicsWorld, RigidBodyHandle};
    use crate::water::buoyancy::{lift, StillWater};

    /// A peeper's body standing with its feet on a floor at zero.
    fn peeper(world: &mut PhysicsWorld) -> RigidBodyHandle {
        let rig = PeeperRigConfig::default();
        let clearance = rig.body_height() * 0.5;
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .bulk(stilts_and_eye(&rig, clearance))
                .position(Point3::new(0.0, clearance, 0.0)),
        );
        world.attach_collider(
            body,
            ColliderDesc::capsule(clearance, rig.eye_radius).density(DENSITY),
        );
        body
    }

    #[test]
    fn a_peeper_wades_on_its_stilts_and_is_never_floated_off_them() {
        let mut world = PhysicsWorld::default();
        let body = peeper(&mut world);
        let weight = world.body(body).unwrap().mass() * 9.81;
        let rig = PeeperRigConfig::default();

        // Knee deep, the capsule round it would already have floated it.
        let knee_deep = StillWater {
            surface: 0.5 * rig.standing_height(),
            floor: 0.0,
        };
        assert!(lift(&world, body, &knee_deep) < 0.05 * weight);
        // Over its eye, it still keeps its feet.
        let drowned = StillWater {
            surface: 2.0 * rig.body_height(),
            floor: 0.0,
        };
        let fully = lift(&world, body, &drowned);
        assert!(fully < weight, "lift {fully} N against weight {weight} N");
    }
}
