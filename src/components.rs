use crate::{rendering::vertex::Vertex, resources::textures::TextureHandle};
use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, VecStorage};

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

#[derive(Component, Debug)]
pub struct SpinSpeed(pub f32); // Radians per logic update

#[derive(Component)]
#[storage(VecStorage)]
pub struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub texture_handles: Vec<TextureHandle>,
}

#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)] // Changed to DenseVecStorage for consistency, can be VecStorage too
pub struct Renderable;

// New Camera Component
use crate::rendering::camera::Camera;

#[derive(Component, Debug)]
#[storage(DenseVecStorage)] // Can use other storage types if preferred, e.g., VecStorage
pub struct CameraComponent(pub Camera);
