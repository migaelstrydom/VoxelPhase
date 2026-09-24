use nalgebra::{UnitVector3, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use crate::animation::{CharacterAnimator, CharacterRigConfig};
use crate::character::{CharacterIntent, CharacterState, Grounding, LocomotionConfig};
use crate::components::{
    Orientation, Position, Renderable, RigidBodyComponent, Rotation, Velocity,
};
use crate::damage::{Health, Ragdoll};
use crate::drive::{Actuator, Allowance, BodyMotion, DriveIntent};
use crate::physics::{ColliderDesc, ConstraintKind, FrictionModel, RigidBodyDesc};
use crate::player::Player;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;

/// Spawns the player entity with all required components.
pub fn spawn_player(world: &mut World, initial_pos: nalgebra::Point3<f32>) -> Entity {
    let locomotion = LocomotionConfig::player();
    let (collider_half_height, collider_radius) =
        (locomotion.collider_half_height, locomotion.collider_radius);
    let (air_steer_speed, jump_speed) = (locomotion.air_steer_speed, locomotion.jump_speed);
    // The yaw allowance's ceiling, in rad/s². A capsule's supports are a point
    // and a torsional row bounded by `μ·N·r` therefore has nothing to bear on
    // (§6.2), so this is the whole of the player's angular authority. It is the
    // acceleration the old reactionless yaw drive was bounded by, so a turn
    // costs what it always did.
    const TURN_AUTHORITY: f32 = 500.0;
    // Effective density of the player capsule, kg/m³, giving a mass of 57kg.
    //
    // The capsule is a *bounding* volume — 1m tall and half a metre across —
    // and a spindly humanoid fills a little over a third of it. So the figure
    // that belongs here is flesh's own density times that fill fraction,
    // roughly 985 x 0.36, and not flesh's density itself: at the 800 declared
    // before, the capsule was solid meat and the player weighed 131kg, enough
    // to craze a block of ice by walking into it.
    //
    // Everything about how the player *moves* is authored as a velocity or an
    // acceleration and multiplied by the mass where it is applied — see
    // `TractionPlanner::plan` and `Allowance` — so this number decides what the
    // player weighs against the world and not how they handle. What it does
    // move is float depth, which is this density over the water's.
    const CAPSULE_DENSITY: f32 = 350.0;
    /// Facing the player spawns with, matching the `Rotation` below.
    const INITIAL_YAW: f32 = 0.0;

    let rig_config = CharacterRigConfig::default();
    // let body_radius = rig_config.body_radius;
    // The capsule's centre rides half its height above the floor; the rig's
    // pelvis belongs lower than that, on bent knees. The animator owns the
    // difference.
    let animator =
        CharacterAnimator::new(rig_config, initial_pos, collider_half_height, INITIAL_YAW);

    let (body_handle, upright_handle) = {
        let mut physics = world.write_resource::<PhysicsResource>();
        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.0)
            .angular_damping(0.95);
        let body_handle = physics.world.create_body(body_desc);
        // A real material coefficient: it keeps the player planted on slopes
        // and lets moving surfaces (play-wheels, platforms) drag them
        // tangentially, and it means the same thing to whatever the player
        // leans on. Which contacts the player is allowed to draw it at is the
        // actuator's business — see `non_support_grip` below.
        let collider_desc = ColliderDesc::capsule(collider_half_height, collider_radius)
            .density(CAPSULE_DENSITY)
            .restitution(0.0)
            .friction_model(FrictionModel::Isotropic(0.8));
        physics.world.attach_collider(body_handle, collider_desc);
        physics
            .world
            .body_mut(body_handle)
            .unwrap()
            .scale_local_inertia(Vector3::new(1.0, 50.0, 1.0));
        // Held so `DeathSystem` can release it. Without that the corpse stays
        // rigidly upright, which reads as a bug rather than a death.
        let upright_handle = physics
            .world
            .create_constraint(ConstraintKind::KeepUpright {
                body: body_handle,
                target_up: UnitVector3::new_normalize(Vector3::new(0.0, 1.0, 0.0)),
                compliance: 0.0,
                max_impulse: f32::INFINITY,
            });
        (body_handle, upright_handle)
    };

    world
        .create_entity()
        .with(Player)
        .with(CharacterIntent::default())
        .with(CharacterState::default())
        .with(locomotion)
        .with(Grounding::default())
        // The player's corpse is never despawned — death should be a state to
        // recover from, not an entity disappearing out from under the camera.
        .with(Health::persistent(100.0))
        .with(Ragdoll::new(vec![upright_handle]))
        .with(animator)
        .with(Position(Vector3::new(
            initial_pos.x,
            initial_pos.y,
            initial_pos.z,
        )))
        .with(Velocity(Vector3::zeros()))
        .with(Rotation(0.0))
        .with(Orientation::default())
        .with(Renderable)
        .with(SensorSet::default())
        .with(ContactCandidates::default())
        .with(RigidBodyComponent(body_handle))
        // Grip nothing that is not holding them up, so jumps along vertical
        // surfaces are not grabbed; and push five times harder through the
        // contacts that do. The gain is the game's decision that the player is
        // a cartoon: an honest 0.8 against gravity caps acceleration at
        // 7.85 m/s², which was measured in the old engine and is far too slow
        // to play. See `Actuator::drive_gain`.
        // The airborne half of the same character. Nothing is holding the
        // player up mid-jump, so every scrap of air steering, jump shaping and
        // turning is momentum the world did not have — granted here, by name,
        // with a ceiling on each. See `Actuator::allowance`.
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
        .build()
}
