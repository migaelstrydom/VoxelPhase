use nalgebra::{UnitVector3, Vector3};
use specs::{Builder, Entity, World, WorldExt};

use crate::animation::{CharacterAnimator, CharacterRigConfig};
use crate::character::{CharacterIntent, CharacterState, Grounding, LocomotionConfig};
use crate::components::{
    Orientation, Position, Renderable, RigidBodyComponent, Rotation, Velocity,
};
use crate::damage::{Health, Ragdoll};
use crate::drive::{Actuator, BodyMotion, DriveIntent};
use crate::physics::{ColliderDesc, ConstraintKind, FrictionModel, RigidBodyDesc};
use crate::player::Player;
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;

/// Spawns the player entity with all required components.
pub fn spawn_player(world: &mut World, initial_pos: nalgebra::Point3<f32>) -> Entity {
    let locomotion = LocomotionConfig::player();
    let (collider_half_height, collider_radius) =
        (locomotion.collider_half_height, locomotion.collider_radius);

    let rig_config = CharacterRigConfig::default();
    // let body_radius = rig_config.body_radius;
    let animator = CharacterAnimator::new(rig_config, initial_pos);

    let (body_handle, upright_handle) = {
        let mut physics = world.write_resource::<PhysicsResource>();
        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.0)
            .angular_damping(0.95);
        let body_handle = physics.world.create_body(body_desc);
        // Floor friction keeps the player planted on slopes and lets moving
        // surfaces (play-wheels, platforms) drag them tangentially. Wall
        // friction stays near zero so jumps along vertical surfaces don't get
        // grabbed. The local up axis is body-Y — the KeepUpright constraint
        // holds that aligned with world up, so contact normals pointing up
        // resolve to "floor" and horizontal normals resolve to "wall".
        let collider_desc = ColliderDesc::capsule(collider_half_height, collider_radius)
            .density(800.0)
            .restitution(0.0)
            .friction_model(FrictionModel::AxisBiased {
                floor: 0.8,
                wall: 0.0,
                local_up: UnitVector3::new_normalize(Vector3::y()),
                cos_floor: (45.0f32.to_radians()).cos(),
                cos_wall: (75.0f32.to_radians()).cos(),
            });
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
                target_up: nalgebra::UnitVector3::new_normalize(Vector3::new(0.0, 1.0, 0.0)),
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
        .with(Actuator::character())
        .with(DriveIntent::default())
        .with(BodyMotion::default())
        .build()
}
