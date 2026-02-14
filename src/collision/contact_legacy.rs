//! Contact manifold structures for collision response.

use nalgebra::{Point3, Vector3};

/// A single contact point between two objects.
#[derive(Debug, Clone, Copy)]
pub struct ContactPoint {
    /// Contact normal pointing from the terrain/obstacle toward the entity.
    pub normal: Vector3<f32>,
    /// Penetration depth (positive means overlapping).
    pub depth: f32,
}

impl ContactPoint {
    pub fn new(_point: Point3<f32>, normal: Vector3<f32>, depth: f32) -> Self {
        Self { normal, depth }
    }
}
