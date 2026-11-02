//! The firm heart of a loose clump: what physics sees of it.
//!
//! A ball of leaves is mostly air. What stops when it lands is the dense
//! middle of each lobe, so the body is one sphere inside each lobe, a share
//! of its radius; the leaves round it are drawn (`skin`) and drag what passes
//! through them (`drag`), but nothing collides with them. A cluster of
//! spheres, not one ball: a single sphere has nothing to stop it rolling, and
//! the engine's friction has no rolling resistance, while a lumpy cluster
//! tumbles like a sack and stops.

use nalgebra::{Isometry3, Point3};

use super::shape::FoliageShape;
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc, RigidBodyHandle};

/// How a clump's core is built.
#[derive(Debug, Clone, Copy)]
pub struct LeafCore {
    /// Radius of each lobe's core sphere as a share of the lobe's own.
    pub core_fraction: f32,
    /// Mass per cubic metre of leaves, kg/m³: the clump's whole mass over the
    /// space its lobes fill. Light, so the player shoves it rather than
    /// stands on it.
    pub bulk_density: f32,
    /// Grip of the core on whatever it lands on. High: leaves snag.
    pub friction: f32,
    /// Share of its speed the clump loses per second to the air through its
    /// leaves.
    pub linear_damping: f32,
    /// Share of its spin the clump loses per second: leaves brushing the
    /// ground and the air as it tumbles, which is what brings a rolling clump
    /// to rest on a gentle slope.
    pub angular_damping: f32,
}

impl Default for LeafCore {
    fn default() -> Self {
        Self {
            core_fraction: 0.6,
            bulk_density: 14.0,
            friction: 0.9,
            linear_damping: 0.15,
            angular_damping: 0.6,
        }
    }
}

impl LeafCore {
    /// The clump's mass, kg.
    pub fn mass(&self, shape: &FoliageShape) -> f32 {
        self.bulk_density * shape.leaf_volume()
    }

    /// One sphere per lobe, at its centre. Every sphere has the same density,
    /// so the core's centre of mass is the lobes' cube-weighted centre, which
    /// is where [`FoliageShape::centred`] put the origin.
    pub fn colliders(&self, shape: &FoliageShape) -> Vec<ColliderDesc> {
        let radii: Vec<f32> = shape
            .lobes
            .iter()
            .map(|l| l.radius * self.core_fraction)
            .collect();
        let core_volume: f32 = radii
            .iter()
            .map(|r| 4.0 / 3.0 * std::f32::consts::PI * r.powi(3))
            .sum();
        let density = self.mass(shape) / core_volume;
        shape
            .lobes
            .iter()
            .zip(radii)
            .map(|(lobe, radius)| {
                ColliderDesc::sphere(radius)
                    .density(density)
                    .restitution(0.0)
                    .friction(self.friction)
                    .offset_translation(lobe.centre.coords)
            })
            .collect()
    }

    /// The clump as a dynamic body at `pose`, its origin at the shape's.
    pub fn spawn(
        &self,
        world: &mut PhysicsWorld,
        shape: &FoliageShape,
        pose: Isometry3<f32>,
    ) -> RigidBodyHandle {
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::from(pose.translation.vector))
                .rotation(pose.rotation)
                .linear_damping(self.linear_damping)
                .angular_damping(self.angular_damping),
        );
        for collider in self.colliders(shape) {
            world.attach_collider(body, collider);
        }
        body
    }
}
