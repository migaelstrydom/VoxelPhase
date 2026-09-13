//! Heart critter — a small creature you catch rather than fight.
//!
//! It has no attack, no health and no weight worth speaking of. What it
//! has is legs: it walks on the same [`LeggedLocomotion`] the player does,
//! through the same character control chain, driven by a brain instead of
//! a keyboard. Catching one restores hit points.
//!
//! [`LeggedLocomotion`]: crate::animation::LeggedLocomotion
//!
//! The chase is the point, so the numbers below are tuned for one: a
//! critter is a little slower than the player flat out but turns in a
//! fraction of the distance, which makes cornering it the skill rather
//! than outrunning it.

use nalgebra::{Point3, UnitVector3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use crate::animation::critter::{CritterAnimator, CritterRigConfig};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::character::{CharacterIntent, CharacterState, Grounding, LocomotionConfig};
use crate::components::{
    Orientation, Position, Renderable, RigidBodyComponent, Rotation, Velocity,
};
use crate::core::error::EngineResult;
use crate::creature::{Brain, Collectable, Perception};
use crate::drive::{Actuator, Allowance, BodyMotion, DriveIntent};
use crate::physics::{ColliderDesc, ConstraintKind, FrictionModel, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Light: the critter is mostly fluff, and a heavy one shoulders the
/// player around when it bolts past.
const DENSITY: f32 = 120.0;

/// Yaw authority, in rad/s². Higher than the player's, and it is the whole
/// reason the chase works: a critter pivots on the spot where the player
/// has to arc.
const TURN_AUTHORITY: f32 = 900.0;

/// A collectable creature that runs from the player.
#[derive(Deserialize)]
pub struct HeartCritterDef {
    /// Position (x, z). Y comes from the terrain surface, since a creature
    /// authored at a hand-picked height would either hang in the air or
    /// start embedded once the terrain around it is reshaped.
    pub pos: (f32, f32),

    /// Flat-ground top speed in m/s. The player walks at 5.0 and sprints
    /// at 8.0, so the default is catchable in a straight line — the
    /// critter's advantage is its turning circle, not its pace.
    #[serde(default = "HeartCritterDef::default_speed")]
    pub speed: f32,
    /// How close the player has to get before it bolts.
    #[serde(default = "HeartCritterDef::default_flee_range")]
    pub flee_range: f32,
    /// Hit points restored to whoever catches it.
    #[serde(default = "HeartCritterDef::default_reward")]
    pub reward: f32,
}

impl HeartCritterDef {
    pub fn default_speed() -> f32 {
        4.2
    }
    pub fn default_flee_range() -> f32 {
        7.0
    }
    pub fn default_reward() -> f32 {
        25.0
    }
}

impl Spawnable for HeartCritterDef {
    fn material_count(&self) -> usize {
        0
    }

    fn create_materials(&self, _ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // The rig is vertex-coloured — the fur, the feet and the heart on
        // its chest are all in the mesh the animator regenerates each
        // frame, which no material can be attached to anyway.
        Ok(Vec::new())
    }

    fn spawn(&self, world: &mut World, _materials: &[MaterialId]) -> Vec<Entity> {
        let rig = CritterRigConfig::default();
        // The capsule covers the whole critter, so its centre — and with it
        // the character's ground clearance — sits at half its height.
        let clearance = rig.body_height() * 0.5;
        let radius = rig.torso_width * 0.5;
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
        let animator = CritterAnimator::new(rig, initial_pos, clearance, 0.0);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
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
            physics
                .world
                .create_constraint(ConstraintKind::KeepUpright {
                    body: body_handle,
                    target_up: UnitVector3::new_normalize(Vector3::y()),
                    compliance: 0.0,
                    max_impulse: f32::INFINITY,
                });
            body_handle
        };

        vec![world
            .create_entity()
            // Intent is the seam: `BrainSystem` writes it, the shared
            // character control chain reads it. Neither knows about the
            // other, and neither knows this one has a heart for a torso.
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
            .with(crate::sensing::SensorSet::default())
            .with(crate::sensing::ContactCandidates::default())
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
            .with(Perception::ground_creature(self.flee_range * 1.5))
            .with(Brain::skittish(self.flee_range))
            // Catchable from a body-width away: demanding an exact overlap
            // on something this size reads as broken rather than as hard.
            .with(Collectable::heart(radius + 0.5, self.reward))
            .build()]
    }
}
