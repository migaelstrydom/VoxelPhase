//! Collider shapes and collision properties.

use std::sync::Arc;

use nalgebra::{Isometry3, Matrix3, Point3, UnitQuaternion, Vector3};

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

    /// Area of the shape's shadow on a plane perpendicular to `direction`, a
    /// unit vector in the shape's own frame: the area it presents to a flow
    /// along it.
    pub fn projected_area(&self, direction: Vector3<f32>) -> f32 {
        match self {
            ColliderShape::Sphere { radius } => std::f32::consts::PI * radius * radius,
            ColliderShape::Box { half_extents: h } => {
                4.0 * (h.y * h.z * direction.x.abs()
                    + h.x * h.z * direction.y.abs()
                    + h.x * h.y * direction.z.abs())
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                // The caps' disc, and the cylinder's rectangle foreshortened
                // by how far the flow is off its axis.
                let cylinder = 2.0 * (half_height - radius).max(0.0);
                let across_axis = direction.cross(&Vector3::y()).norm();
                std::f32::consts::PI * radius * radius + 2.0 * radius * cylinder * across_axis
            }
            ColliderShape::ConvexHull { hull } => hull.projected_area(direction),
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
/// One variant today: a single scalar per material. It stays an enum because a
/// material may yet want a law of its own — a genuinely anisotropic surface, a
/// speed-dependent coefficient — but a character's grip is not one of those.
/// What a *body* is allowed to draw at a contact belongs to its actuator's
/// non-support grip (`physics/drive/grip.rs`), not to the surface it touches.
#[derive(Debug, Clone, Copy)]
pub enum FrictionModel {
    /// Constant friction coefficient regardless of contact normal.
    Isotropic(f32),
}

impl FrictionModel {
    /// The coefficient this law yields at a contact.
    pub fn coefficient(&self) -> f32 {
        match self {
            FrictionModel::Isotropic(mu) => *mu,
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
    /// This material's friction coefficient.
    pub fn friction(&self) -> f32 {
        self.friction.coefficient()
    }

    /// Combine two materials at a contact: average restitution, geometric mean
    /// friction.
    pub fn combine(a: &Self, b: &Self) -> (f32, f32) {
        let restitution = (a.restitution + b.restitution) * 0.5;
        (restitution, (a.friction() * b.friction()).sqrt())
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

    /// Shift this collider's local offset by `delta`, expressed in the body
    /// frame. Used when a body re-centres its origin on its colliders' centre
    /// of mass: the body moves, every collider moves the opposite way, and the
    /// collider stays where it is in the world.
    pub(crate) fn shift_offset(&mut self, delta: Vector3<f32>) {
        self.offset.translation.vector += delta;
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

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use super::*;
    use crate::collision::convex_hull::cube_hull;

    #[test]
    fn a_box_presents_one_face_head_on_and_more_obliquely() {
        let shape = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 0.5, 0.25),
        };
        assert!((shape.projected_area(Vector3::x()) - 0.5).abs() < 1e-6);
        assert!((shape.projected_area(Vector3::z()) - 2.0).abs() < 1e-6);
        let diagonal = Vector3::new(1.0, 0.0, 1.0).normalize();
        assert!((shape.projected_area(diagonal) - 2.5 / 2f32.sqrt()).abs() < 1e-5);
    }

    #[test]
    fn a_hull_presents_the_same_area_as_the_box_it_is() {
        let half = Vector3::new(1.0, 0.5, 0.25);
        let hull = ColliderShape::ConvexHull {
            hull: Arc::new(cube_hull(half)),
        };
        let solid = ColliderShape::Box { half_extents: half };
        for direction in [
            Vector3::x(),
            Vector3::y(),
            Vector3::new(1.0, 2.0, -3.0).normalize(),
        ] {
            let (a, b) = (
                hull.projected_area(direction),
                solid.projected_area(direction),
            );
            assert!((a - b).abs() < 1e-5, "{direction:?}: hull {a}, box {b}");
        }
    }

    #[test]
    fn a_capsule_end_on_presents_its_disc_and_side_on_its_outline() {
        let shape = ColliderShape::Capsule {
            half_height: 0.5,
            radius: 0.2,
        };
        assert!((shape.projected_area(Vector3::y()) - PI * 0.04).abs() < 1e-6);
        let side = PI * 0.04 + 0.4 * 0.6;
        assert!((shape.projected_area(Vector3::x()) - side).abs() < 1e-6);
    }
}
