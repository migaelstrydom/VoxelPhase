//! Trait for static geometry that physics bodies can collide against.
//!
//! This allows the physics engine to query external collision geometry (like terrain)
//! without owning it directly.

use nalgebra::{Point3, Vector3};

/// Contact information from a static geometry query.
#[derive(Debug, Clone)]
pub struct StaticContact {
    /// Contact point on the static geometry surface.
    pub point: Point3<f32>,
    /// Contact normal pointing away from the static geometry (toward the query shape).
    pub normal: Vector3<f32>,
    /// Penetration depth (positive means overlapping).
    pub depth: f32,
}

impl StaticContact {
    pub fn new(point: Point3<f32>, normal: Vector3<f32>, depth: f32) -> Self {
        Self {
            point,
            normal,
            depth,
        }
    }
}

/// Result of a swept collision query against static geometry.
#[derive(Debug, Clone)]
pub struct SweptStaticContact {
    /// Time of contact in range [0, 1] where 0 = start, 1 = end.
    pub t: f32,
    /// Contact point on the static geometry surface.
    pub point: Point3<f32>,
    /// Surface normal at contact point.
    pub normal: Vector3<f32>,
}

impl SweptStaticContact {
    pub fn new(t: f32, point: Point3<f32>, normal: Vector3<f32>) -> Self {
        Self { t, point, normal }
    }
}

/// Trait for querying collision against static geometry.
///
/// Implementors provide collision detection against their geometry.
/// The physics engine calls these methods during collision detection.
pub trait StaticGeometry {
    /// Query sphere collision against static geometry.
    ///
    /// Returns all contacts where the sphere overlaps geometry.
    fn query_sphere(&self, center: Point3<f32>, radius: f32) -> Vec<StaticContact>;

    /// Sweep a sphere from start to end against static geometry.
    ///
    /// Returns the first contact along the sweep path, if any.
    fn sweep_sphere(
        &self,
        start: Point3<f32>,
        end: Point3<f32>,
        radius: f32,
    ) -> Option<SweptStaticContact>;
}
