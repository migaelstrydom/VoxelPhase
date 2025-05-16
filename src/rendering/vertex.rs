use nalgebra::{Vector2, Vector4};

#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct Vertex {
    pub pos: Vector4<f32>,
    pub color: Vector4<f32>,
    pub tex_coords: Vector2<f32>,
}
