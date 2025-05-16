use ash::vk;
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

#[derive(Component, Debug)]
pub struct Rotation(pub f32); // Radians

#[derive(Component, Debug)]
pub struct SpinSpeed(pub f32); // Radians per logic update

// Render Components
#[derive(Component)]
#[storage(VecStorage)]
struct Mesh {
    vertex_buffer: vk::Buffer,
    index_buffer: vk::Buffer,
    index_count: u32,
}

#[derive(Component, Debug, Default)]
pub struct Renderable;

// New Camera Component
use crate::rendering::camera::Camera;

#[derive(Component, Debug)]
#[storage(DenseVecStorage)] // Can use other storage types if preferred, e.g., VecStorage
pub struct CameraComponent(pub Camera);
