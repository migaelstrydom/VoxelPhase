use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
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
#[storage(VecStorage)]
pub struct Acceleration(pub Vector3<f32>);

/// Gravity strength for an entity. Applied as downward acceleration.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Gravity(pub f32);

/// Motion state for CCD collision detection.
///
/// Stores previous and predicted positions for swept collision queries.
/// Updated by MotionPredictionSystem, consumed by collision systems.
#[derive(Component, Debug, Clone)]
#[storage(VecStorage)]
pub struct MotionState {
    /// Position at the start of the frame (before movement).
    pub prev: Point3<f32>,
    /// Predicted position at the end of the frame (before collision resolution).
    pub predicted: Point3<f32>,
}

impl MotionState {
    pub fn new(position: Point3<f32>) -> Self {
        Self {
            prev: position,
            predicted: position,
        }
    }
}

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

/// Marker for velocity-driven dynamic bodies.
///
/// Entities with this component have their ECS velocity synced into the
/// physics engine before each step. The solver may then modify the velocity
/// via contact impulses, and the result is synced back to ECS.
///
/// Use this for any gameplay object that is controlled by game code but
/// should interact physically: player characters, moving platforms, doors.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct VelocityDriven;

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
