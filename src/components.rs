use std::sync::Arc;

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, VecStorage};

use crate::model::Model;
use crate::rendering::camera::Camera;

// Physics Components
#[derive(Component, Debug)]
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
