//! Collider shapes and collision properties.

use std::sync::Arc;

use nalgebra::{Isometry3, Matrix3, Point3, UnitQuaternion, UnitVector3, Vector3};

use super::math::{box_inertia_tensor, capsule_inertia_tensor, sphere_inertia_tensor};
use crate::collision::ConvexHull;

/// Shape of a collider.
#[derive(Debug, Clone)]
pub enum ColliderShape {
    /// A sphere centered at the collider's local origin.
    Sphere { radius: f32 },
    /// An oriented box (rectangular prism) centered at the collider's local origin.
    Box { half_extents: Vector3<f32> },
    /// A capsule (cylinder + hemisphere caps) along the local Y axis.
    /// `half_height` includes the caps; total height = `2 * half_height`.
    Capsule { half_height: f32, radius: f32 },
    /// A convex hull defined by pre-computed vertices and faces.
    /// `Arc` because hull data is potentially large and shared across cloned colliders.
    ConvexHull { hull: Arc<ConvexHull> },
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
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                let r = *radius;
                let cyl_h = 2.0 * (half_height - r);
                let pi = std::f32::consts::PI;
                let volume = pi * r * r * (cyl_h + (4.0 / 3.0) * r);
                volume * density
            }
            ColliderShape::ConvexHull { hull } => hull.compute_volume() * density,
        }
    }

    /// Compute the local inertia tensor given mass.
    pub fn compute_inertia(&self, mass: f32) -> Matrix3<f32> {
        match self {
            ColliderShape::Sphere { radius } => sphere_inertia_tensor(mass, *radius),
            ColliderShape::Box { half_extents } => box_inertia_tensor(mass, *half_extents),
            ColliderShape::Capsule {
                half_height,
                radius,
            } => capsule_inertia_tensor(mass, *half_height, *radius),
            ColliderShape::ConvexHull { hull } => hull.compute_inertia(mass),
        }
    }

    /// Get the bounding radius of the shape.
    pub fn bounding_radius(&self) -> f32 {
        match self {
            ColliderShape::Sphere { radius } => *radius,
            ColliderShape::Box { half_extents } => half_extents.norm(),
            ColliderShape::Capsule { half_height, .. } => *half_height,
            ColliderShape::ConvexHull { hull } => hull.bounding_radius,
        }
    }

    /// Get the XZ footprint radius of the shape on a horizontal surface.
    pub fn footprint_radius(&self) -> f32 {
        match self {
            ColliderShape::Sphere { radius } => *radius,
            ColliderShape::Box { half_extents } => {
                (half_extents.x * half_extents.x + half_extents.z * half_extents.z).sqrt()
            }
            ColliderShape::Capsule { radius, .. } => *radius,
            ColliderShape::ConvexHull { hull } => hull.bounding_radius,
        }
    }
}

/// Friction law used when resolving a contact.
///
/// Friction is evaluated per manifold, parameterized by the world-space contact
/// normal and the collider's world-space orientation. The default `Isotropic`
/// variant ignores both — it is a single scalar and costs a field load.
///
/// `AxisBiased` lets a collider advertise different friction for contacts along
/// its local "up" axis versus contacts perpendicular to it. Typical use: a
/// player capsule with grippy floor contacts but near-zero friction on walls so
/// it can slide up and along vertical surfaces without being dragged down.
#[derive(Debug, Clone, Copy)]
pub enum FrictionModel {
    /// Constant friction coefficient regardless of contact normal.
    Isotropic(f32),
    /// Friction depends on the angle between the contact normal and a local up axis.
    ///
    /// Let `c = |n_world · (rot * local_up)|`. We lerp smoothly between the
    /// `wall` and `floor` values: below `cos_wall` we return `wall`; above
    /// `cos_floor` we return `floor`; in between a smoothstep blend. Choosing
    /// `cos_wall < cos_floor` defines the transition band (e.g. 45°..60°).
    AxisBiased {
        /// Friction for contacts whose normal aligns with local_up (floors).
        floor: f32,
        /// Friction for contacts whose normal is perpendicular to local_up (walls).
        wall: f32,
        /// Collider-local up axis (e.g. `Y` for an upright capsule).
        local_up: UnitVector3<f32>,
        /// Cosine of the most-tilted angle still treated as "floor" (upper band edge).
        cos_floor: f32,
        /// Cosine of the least-tilted angle still treated as "wall" (lower band edge).
        cos_wall: f32,
    },
}

impl FrictionModel {
    /// Evaluate friction at a specific contact.
    ///
    /// `normal_world` is the contact normal in world space. Its sign does not
    /// matter — we use the absolute alignment with the local up axis.
    /// `rot_world` is the collider's world-space orientation (used only by
    /// orientation-dependent variants).
    pub fn evaluate(&self, normal_world: &Vector3<f32>, rot_world: &UnitQuaternion<f32>) -> f32 {
        match self {
            FrictionModel::Isotropic(mu) => *mu,
            FrictionModel::AxisBiased {
                floor,
                wall,
                local_up,
                cos_floor,
                cos_wall,
            } => {
                let up_world = rot_world * local_up.into_inner();
                let c = normal_world.dot(&up_world).abs();
                if c >= *cos_floor {
                    *floor
                } else if c <= *cos_wall {
                    *wall
                } else {
                    let t = (c - cos_wall) / (cos_floor - cos_wall);
                    let s = t * t * (3.0 - 2.0 * t);
                    wall + (floor - wall) * s
                }
            }
        }
    }

    /// A simple scalar friction coefficient.
    pub fn isotropic(mu: f32) -> Self {
        FrictionModel::Isotropic(mu)
    }
}

/// Material properties for collision response.
#[derive(Debug, Clone, Copy)]
pub struct ColliderMaterial {
    /// Coefficient of restitution (bounciness). 0 = no bounce, 1 = perfect bounce.
    pub restitution: f32,
    /// Friction law for this material.
    pub friction: FrictionModel,
}

impl ColliderMaterial {
    /// Evaluate this material's friction at a specific contact normal.
    pub fn friction_at(&self, normal_world: &Vector3<f32>, rot_world: &UnitQuaternion<f32>) -> f32 {
        self.friction.evaluate(normal_world, rot_world)
    }

    /// Combine two materials at a specific contact: average restitution,
    /// geometric mean friction. Each side sees the contact normal from its own
    /// perspective; AxisBiased uses the absolute alignment with local up so
    /// both sides evaluate against the same world-space normal without flipping.
    pub fn combine_at(
        a: &Self,
        b: &Self,
        normal_world: &Vector3<f32>,
        rot_a: &UnitQuaternion<f32>,
        rot_b: &UnitQuaternion<f32>,
    ) -> (f32, f32) {
        let restitution = (a.restitution + b.restitution) * 0.5;
        let fa = a.friction_at(normal_world, rot_a);
        let fb = b.friction_at(normal_world, rot_b);
        (restitution, (fa * fb).sqrt())
    }
}

impl Default for ColliderMaterial {
    fn default() -> Self {
        Self {
            restitution: 0.3,
            friction: FrictionModel::Isotropic(0.5),
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

    pub fn convex_hull(hull: Arc<ConvexHull>) -> Self {
        Self {
            shape: ColliderShape::ConvexHull { hull },
            offset: Isometry3::identity(),
            density: 1000.0,
            material: ColliderMaterial::default(),
        }
    }

    pub fn capsule(half_height: f32, radius: f32) -> Self {
        Self {
            shape: ColliderShape::Capsule {
                half_height,
                radius,
            },
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

    /// Set isotropic friction (the common case). For direction-dependent
    /// friction use [`Self::friction_model`].
    pub fn friction(mut self, friction: f32) -> Self {
        self.material.friction = FrictionModel::Isotropic(friction);
        self
    }

    /// Set a custom friction law (e.g. `FrictionModel::AxisBiased`).
    pub fn friction_model(mut self, model: FrictionModel) -> Self {
        self.material.friction = model;
        self
    }

    /// Set the offset translation (position relative to body center of mass).
    pub fn offset_translation(mut self, translation: Vector3<f32>) -> Self {
        self.offset.translation.vector = translation;
        self
    }

    /// Set the offset rotation (orientation relative to body frame).
    pub fn offset_rotation(mut self, rotation: UnitQuaternion<f32>) -> Self {
        self.offset.rotation = rotation;
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

    /// Local transform relative to the body.
    pub fn offset(&self) -> &Isometry3<f32> {
        &self.offset
    }

    /// Compose the body transform with this collider's local offset.
    pub fn world_transform(
        &self,
        body_position: Point3<f32>,
        body_rotation: UnitQuaternion<f32>,
    ) -> Isometry3<f32> {
        let body_iso = Isometry3::from_parts(body_position.coords.into(), body_rotation);
        body_iso * self.offset
    }

    /// Get the world-space center of the collider given the body's position.
    pub fn world_center(
        &self,
        body_position: Point3<f32>,
        body_rotation: UnitQuaternion<f32>,
    ) -> Point3<f32> {
        Point3::from(
            self.world_transform(body_position, body_rotation)
                .translation
                .vector,
        )
    }
}
