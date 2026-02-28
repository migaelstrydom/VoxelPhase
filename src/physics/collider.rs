//! Collider shapes and collision properties.

use nalgebra::{Isometry3, Matrix3, Point3, UnitQuaternion, Vector3};

use super::math::{box_inertia_tensor, sphere_inertia_tensor};

/// Shape of a collider.
#[derive(Debug, Clone)]
pub enum ColliderShape {
    /// A sphere centered at the collider's local origin.
    Sphere { radius: f32 },
    /// An oriented box (rectangular prism) centered at the collider's local origin.
    Box { half_extents: Vector3<f32> },
}

impl ColliderShape {
    /// Compute the mass from shape and density.
    pub fn compute_mass(&self, density: f32) -> f32 {
        match self {
            ColliderShape::Sphere { radius } => {
                let volume = (4.0 / 3.0) * std::f32::consts::PI * radius.powi(3);
                volume * density
            }
            ColliderShape::Box { half_extents } => {
                let volume = 8.0 * half_extents.x * half_extents.y * half_extents.z;
                volume * density
            }
        }
    }

    /// Compute the local inertia tensor given mass.
    pub fn compute_inertia(&self, mass: f32) -> Matrix3<f32> {
        match self {
            ColliderShape::Sphere { radius } => sphere_inertia_tensor(mass, *radius),
            ColliderShape::Box { half_extents } => box_inertia_tensor(mass, *half_extents),
        }
    }

    /// Get the bounding radius of the shape.
    pub fn bounding_radius(&self) -> f32 {
        match self {
            ColliderShape::Sphere { radius } => *radius,
            ColliderShape::Box { half_extents } => half_extents.norm(),
        }
    }
}

/// Material properties for collision response.
#[derive(Debug, Clone, Copy)]
pub struct ColliderMaterial {
    /// Coefficient of restitution (bounciness). 0 = no bounce, 1 = perfect bounce.
    pub restitution: f32,
    /// Friction coefficient.
    pub friction: f32,
}

impl ColliderMaterial {
    /// Combine two materials: average restitution, geometric mean friction.
    pub fn combine(a: &Self, b: &Self) -> (f32, f32) {
        let restitution = (a.restitution + b.restitution) * 0.5;
        let friction = (a.friction * b.friction).sqrt();
        (restitution, friction)
    }
}

impl Default for ColliderMaterial {
    fn default() -> Self {
        Self {
            restitution: 0.3,
            friction: 0.5,
        }
    }
}

/// Descriptor for creating a collider.
#[derive(Debug, Clone)]
pub struct ColliderDesc {
    pub shape: ColliderShape,
    /// Local offset from the body's center of mass.
    pub offset: Isometry3<f32>,
    /// Density for computing mass (kg/m³).
    pub density: f32,
    /// Material properties.
    pub material: ColliderMaterial,
}

impl ColliderDesc {
    pub fn sphere(radius: f32) -> Self {
        Self {
            shape: ColliderShape::Sphere { radius },
            offset: Isometry3::identity(),
            density: 1000.0,
            material: ColliderMaterial::default(),
        }
    }

    pub fn box_shape(half_extents: Vector3<f32>) -> Self {
        Self {
            shape: ColliderShape::Box { half_extents },
            offset: Isometry3::identity(),
            density: 1000.0,
            material: ColliderMaterial::default(),
        }
    }

    pub fn density(mut self, density: f32) -> Self {
        self.density = density;
        self
    }

    pub fn restitution(mut self, restitution: f32) -> Self {
        self.material.restitution = restitution;
        self
    }

    pub fn friction(mut self, friction: f32) -> Self {
        self.material.friction = friction;
        self
    }
}

/// A collider attached to a rigid body.
#[derive(Debug)]
pub struct Collider {
    /// The collision shape.
    shape: ColliderShape,
    /// Local transform relative to the body.
    offset: Isometry3<f32>,
    /// Material properties.
    material: ColliderMaterial,
    /// Mass computed from shape and density.
    mass: f32,
    /// Local inertia tensor.
    local_inertia: Matrix3<f32>,
}

impl Collider {
    pub(crate) fn new(desc: ColliderDesc) -> Self {
        let mass = desc.shape.compute_mass(desc.density);
        let local_inertia = desc.shape.compute_inertia(mass);

        Self {
            shape: desc.shape,
            offset: desc.offset,
            material: desc.material,
            mass,
            local_inertia,
        }
    }

    pub fn shape(&self) -> &ColliderShape {
        &self.shape
    }

    pub fn material(&self) -> &ColliderMaterial {
        &self.material
    }

    pub fn mass(&self) -> f32 {
        self.mass
    }

    pub fn local_inertia(&self) -> &Matrix3<f32> {
        &self.local_inertia
    }

    /// Get the world-space center of the collider given the body's position.
    pub fn world_center(
        &self,
        body_position: Point3<f32>,
        body_rotation: UnitQuaternion<f32>,
    ) -> Point3<f32> {
        let body_isometry = Isometry3::from_parts(body_position.coords.into(), body_rotation);
        let world_offset = body_isometry * self.offset;
        Point3::from(world_offset.translation.vector)
    }
}
