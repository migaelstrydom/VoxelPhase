use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

#[derive(Component, Debug)]
pub struct Position(pub Vector3<f32>);

#[derive(Component, Debug)]
pub struct Rotation(pub f32); // Radians

#[derive(Component, Debug)]
pub struct SpinSpeed(pub f32); // Radians per logic update

#[derive(Component, Debug, Default)]
pub struct Renderable;
