use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector3};
use specs::{Component, DenseVecStorage, VecStorage};

use crate::model::Model;
use crate::physics::RigidBodyHandle;
use crate::rendering::camera::Camera;

// Physics Components
#[derive(Component, Debug, Clone, Copy)]
#[storage(VecStorage)]
pub struct Position(pub Vector3<f32>);

#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Velocity(pub Vector3<f32>);

#[derive(Component, Debug)]
pub struct Rotation(pub f32); // Radians

/// 3D orientation as a unit quaternion.
/// Synced from physics for entities with RigidBodyComponent.
#[derive(Component, Debug, Clone, Copy)]
#[storage(VecStorage)]
pub struct Orientation(pub UnitQuaternion<f32>);

impl Default for Orientation {
    fn default() -> Self {
        Self(UnitQuaternion::identity())
    }
}

/// Links an entity to a rigid body in the physics world.
#[derive(Component, Debug, Clone, Copy)]
#[storage(VecStorage)]
pub struct RigidBodyComponent(pub RigidBodyHandle);

/// Velocity-driven dynamic body with per-substep drive.
///
/// Entities with this component have their ECS velocity set as a drive target
/// in the physics engine. Each substep, the body accelerates toward the target,
/// and the solver can oppose the drive via contact impulses. This allows smooth
/// pushing of heavy objects at a speed determined by mass ratio, without the
/// jitter caused by direct velocity overrides.
///
/// Use this for any gameplay object that is controlled by game code but
/// should interact physically: player characters, moving platforms, doors.
#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct VelocityDriven {
    /// Maximum linear acceleration toward the target velocity (units/s²).
    /// High values (500+) give snappy free movement; the solver limits speed
    /// at contacts regardless.
    pub max_accel: f32,
    /// Target angular velocity, set by gameplay systems (e.g. player turning).
    /// Synced to the physics body each frame alongside the linear velocity drive.
    pub angular_velocity: Vector3<f32>,
    /// Maximum angular acceleration toward the target angular velocity (rad/s²).
    /// Controls how aggressively the body can change its rotation rate.
    pub angular_max_accel: f32,
}

impl Default for VelocityDriven {
    fn default() -> Self {
        Self {
            max_accel: 500.0,
            angular_velocity: Vector3::zeros(),
            angular_max_accel: 500.0,
        }
    }
}

/// A model instance referencing a shared Model definition.
#[derive(Component)]
#[storage(VecStorage)]
pub struct ModelInstance {
    /// The model definition (shared across instances).
    pub model: Arc<Model>,
}

impl ModelInstance {
    pub fn new(model: Arc<Model>) -> Self {
        Self { model }
    }
}

#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Renderable;

#[derive(Component, Debug)]
#[storage(DenseVecStorage)]
pub struct CameraComponent(pub Camera);
