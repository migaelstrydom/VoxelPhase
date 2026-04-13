//! Triangle primitive for collision detection and spatial queries.

use nalgebra::{Point3, Vector3};

use super::AABB;

/// A triangle represented by three vertices.
#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    pub v0: Point3<f32>,
    pub v1: Point3<f32>,
    pub v2: Point3<f32>,
}

impl Triangle {
    pub fn new(v0: Point3<f32>, v1: Point3<f32>, v2: Point3<f32>) -> Self {
        Self { v0, v1, v2 }
    }

    /// Compute the face normal (not normalized).
    pub fn normal_unnormalized(&self) -> Vector3<f32> {
        let e1 = self.v1 - self.v0;
        let e2 = self.v2 - self.v0;
        e1.cross(&e2)
    }

    /// Compute the unit face normal.
    pub fn normal(&self) -> Vector3<f32> {
        let n = self.normal_unnormalized();
        let len_sq = n.magnitude_squared();
        if len_sq > 1e-10 {
            n / len_sq.sqrt()
        } else {
            Vector3::new(0.0, 1.0, 0.0) // Degenerate triangle fallback
        }
    }

    /// Get a vertex by index (0, 1, or 2).
    pub fn vertex(&self, index: usize) -> Point3<f32> {
        match index {
            0 => self.v0,
            1 => self.v1,
            _ => self.v2,
        }
    }

    /// Compute the axis-aligned bounding box of this triangle.
    pub fn aabb(&self) -> AABB {
        AABB::new(
            Point3::new(
                self.v0.x.min(self.v1.x).min(self.v2.x),
                self.v0.y.min(self.v1.y).min(self.v2.y),
                self.v0.z.min(self.v1.z).min(self.v2.z),
            ),
            Point3::new(
                self.v0.x.max(self.v1.x).max(self.v2.x),
                self.v0.y.max(self.v1.y).max(self.v2.y),
                self.v0.z.max(self.v1.z).max(self.v2.z),
            ),
        )
    }

    /// Get an edge by index (0, 1, or 2).
    /// Returns (start, end) of the edge.
    pub fn edge(&self, index: usize) -> (Point3<f32>, Point3<f32>) {
        match index {
            0 => (self.v0, self.v1),
            1 => (self.v1, self.v2),
            _ => (self.v2, self.v0),
        }
    }
}
