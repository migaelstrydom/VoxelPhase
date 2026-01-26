use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{Component, DenseVecStorage, VecStorage};

use crate::collision::Sphere;
use crate::model::Model;
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

// Collision Components

/// Collision shape for an entity. Currently supports sphere only.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Collider {
    pub shape: Sphere,
}

impl Collider {
    pub fn sphere(radius: f32) -> Self {
        Self {
            shape: Sphere::new(radius),
        }
    }
}

/// Physical properties for collision response.
#[derive(Component, Debug, Clone, Copy)]
#[storage(VecStorage)]
pub struct PhysicsBody {
    /// Bounciness (0 = no bounce, 1 = perfect bounce).
    pub restitution: f32,
    /// Friction coefficient.
    pub friction: f32,
    /// Mass in kilograms.
    pub mass: f32,
}

impl Default for PhysicsBody {
    fn default() -> Self {
        Self {
            restitution: 0.2,
            friction: 0.8,
            mass: 1.0,
        }
    }
}

#[derive(Component, Debug)]
pub struct Rotation(pub f32); // Radians

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
