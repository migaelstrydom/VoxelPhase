//! A powered platform that patrols a route of world points under its own
//! motor.
//!
//! Nothing anchors it to the world. It is a heavy dynamic box with two pieces
//! of machinery bolted on:
//!
//! ```text
//!   MovingPlatformSystem ──► DriveIntent ──► Actuator ──► solver
//!    (thrust at the next     (intent)       (motor, capped
//!     waypoint, plus drag)                   acceleration)
//!   DeckSuspension ──► KeepUpright ─────► solver
//!    (tilt, damping)     (attitude, soft or rigid)
//! ```
//!
//! The motor commands speed and never position, but it commands it *toward* the
//! waypoint it is currently running to, recomputed from wherever the platform
//! actually is, so displacement is transient: shove it with a grenade and it
//! flies back at the same waypoint it was already heading for. Gravity stays
//! switched on, so a platform that loses its `Actuator` component — the same
//! trick `DeathSystem` uses on the player — stops being a platform and becomes
//! a falling box.

use nalgebra::{Point3, UnitVector3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::box_object::create_box_material_for_style;
use super::shared::finish::ColliderSurface;
use super::shared::models::cuboid_model;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::drive::{Actuator, BodyMotion, DriveIntent};
use crate::level::BoxStyle;
use crate::physics::constraint::ConstraintKind;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::platform::{DeckSuspension, MovingPlatform, RouteLoop, SeekMotion};
use crate::rendering::material::MaterialId;
use crate::rendering::physical_finish::PhysicalSurface;
use crate::systems::PhysicsResource;

/// Heavy enough that a player walking on to it barely registers, light enough
/// that a grenade still means something.
const PLATFORM_SURFACE: PhysicalSurface = PhysicalSurface {
    restitution: 0.1,
    friction: 0.9,
    density: 300.0,
};

/// Acceleration budget of the motor, in m/s². Must clear gravity with room to
/// spare or the platform sags on every upward leg; the lower this is, the more
/// a heavy load drags the patrol off its cruise speed.
const MOTOR_MAX_ACCEL: f32 = 40.0;

#[derive(Deserialize)]
pub struct MovingPlatformDef {
    /// The patrol, in world space. The platform spawns on the first waypoint
    /// and heads for the second. Two waypoints is the shuttle it used to be;
    /// more than two corner at each one.
    pub waypoints: Vec<(f32, f32, f32)>,
    /// What happens at the end of the list: turn around, or wrap and go again.
    #[serde(default = "MovingPlatformDef::default_looping")]
    pub looping: RouteLoop,
    #[serde(default = "MovingPlatformDef::default_half_extents")]
    pub half_extents: (f32, f32, f32),
    /// Cruise speed, in m/s.
    #[serde(default = "MovingPlatformDef::default_speed")]
    pub speed: f32,
    /// Spin-up time in seconds, and with it how tightly the platform corners:
    /// a corner is rounded over about `speed · spin_up`, and a turnaround
    /// swings about a third of that past its waypoint. Author clearance for it.
    #[serde(default = "MovingPlatformDef::default_spin_up")]
    pub spin_up: f32,
    /// How far the deck tips under a player on its edge, and how fast the ring
    /// that follows dies. Omit for a deck that does not move at all.
    #[serde(default = "MovingPlatformDef::default_suspension")]
    pub suspension: DeckSuspension,
}

impl MovingPlatformDef {
    pub fn default_half_extents() -> (f32, f32, f32) {
        (2.0, 0.3, 2.0)
    }
    /// Both motion defaults are `SeekMotion`'s own, deliberately rather than
    /// literals here. A second copy of a default is a trap: it looks
    /// authoritative, it is the one a reader reaches for first, and changing
    /// the other has no effect on anything the game spawns.
    pub fn default_speed() -> f32 {
        SeekMotion::default().speed
    }
    pub fn default_looping() -> RouteLoop {
        RouteLoop::Shuttle
    }
    pub fn default_spin_up() -> f32 {
        SeekMotion::default().tau
    }
    pub fn default_suspension() -> DeckSuspension {
        DeckSuspension::PLATFORM_DECK
    }
}

impl Spawnable for MovingPlatformDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![create_box_material_for_style(
            BoxStyle::Warning,
            PLATFORM_SURFACE,
            ctx.textures,
            ctx.materials,
        )?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let waypoints: Vec<Vector3<f32>> = self
            .waypoints
            .iter()
            .map(|&(x, y, z)| Vector3::new(x, y, z))
            .collect();
        let from = waypoints.first().copied().unwrap_or_else(Vector3::zeros);
        let pos = Point3::from(from);
        let half_extents = Vector3::new(
            self.half_extents.0,
            self.half_extents.1,
            self.half_extents.2,
        );
        let model = cuboid_model(half_extents, materials[0]);
        let tuning = self.suspension.tune(&half_extents);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            // Linear damping is left at zero: the motor already sets the
            // speed, and damping would only fight it. Angular damping is the
            // suspension's, and damps only the wobble — the motor drives
            // through the centre of mass and has no angular authority to lose.
            // Gravity stays on so an unpowered platform falls.
            let body_desc = RigidBodyDesc::dynamic()
                .position(pos)
                .gravity_scale(1.0)
                .linear_damping(0.0)
                .angular_damping(tuning.angular_damping);

            let body_handle = physics.world.create_body(body_desc);
            physics.world.attach_collider(
                body_handle,
                ColliderDesc::box_shape(half_extents).with_physical_surface(PLATFORM_SURFACE),
            );

            // Attitude with unlimited authority, soft by as much as the
            // suspension asks for. The deck always ends up level; the
            // compliance is how long it argues about it first.
            let _ = physics
                .world
                .create_constraint(ConstraintKind::KeepUpright {
                    body: body_handle,
                    target_up: UnitVector3::new_normalize(Vector3::y()),
                    compliance: tuning.compliance,
                    max_impulse: f32::INFINITY,
                });

            body_handle
        };

        let platform = MovingPlatform::new(
            waypoints,
            self.looping,
            SeekMotion {
                speed: self.speed,
                tau: self.spin_up,
            },
        );

        vec![world
            .create_entity()
            .with(Position(from))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(nalgebra::UnitQuaternion::identity()))
            .with(RigidBodyComponent(body_handle))
            .with(Actuator::medium(MOTOR_MAX_ACCEL, 0.0))
            .with(DriveIntent::default())
            .with(BodyMotion::default())
            .with(platform)
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build()]
    }
}
