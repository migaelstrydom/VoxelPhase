use nalgebra::Vector3;
use specs::{Builder, Entity, World, WorldExt};

use crate::animation::{CharacterAnimator, CharacterRigConfig};
use crate::components::{
    Orientation, Position, Renderable, RigidBodyComponent, Rotation, Velocity, VelocityDriven,
};
use crate::physics::{ColliderDesc, ConstraintKind, RigidBodyDesc};
use crate::player::{Player, PlayerConfig, PlayerState, PlayerTargetState};
use crate::sensing::{ContactCandidates, SensorSet};
use crate::systems::PhysicsResource;

/// Spawns the player entity with all required components.
pub fn spawn_player(world: &mut World, initial_pos: nalgebra::Point3<f32>) -> Entity {
    let player_config = world.read_resource::<PlayerConfig>();
    drop(player_config);

    let rig_config = CharacterRigConfig::default();
    // let body_radius = rig_config.body_radius;
    let animator = CharacterAnimator::new(rig_config, initial_pos);

    let body_handle = {
        let mut physics = world.write_resource::<PhysicsResource>();
        let body_desc = RigidBodyDesc::dynamic()
            .position(initial_pos)
            .gravity_scale(1.0)
            .linear_damping(0.0)
            .angular_damping(0.95);
        let body_handle = physics.world.create_body(body_desc);
        let collider_desc = ColliderDesc::capsule(0.5, 0.25)
            .density(800.0)
            .restitution(0.0)
            .friction(0.0);
        physics.world.attach_collider(body_handle, collider_desc);
        physics
            .world
            .body_mut(body_handle)
            .unwrap()
            .scale_local_inertia(Vector3::new(1.0, 50.0, 1.0));
        physics
            .world
            .create_constraint(ConstraintKind::KeepUpright {
                body: body_handle,
                target_up: nalgebra::UnitVector3::new_normalize(Vector3::new(0.0, 1.0, 0.0)),
                compliance: 0.0,
            });
        body_handle
    };

    world
        .create_entity()
        .with(Player)
        .with(PlayerTargetState::default())
        .with(PlayerState::default())
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
        .with(VelocityDriven::default())
        .build()
}
