//! A boulder: a fragment made a rigid body that tumbles, lands and sleeps.
//!
//! ```text
//!   Boulder { bricks, mesh, mass, spin, material }
//!     │ place(PhysicsWorld)
//!     │   body at the mesh's origin, one hull collider per brick, density
//!     │   set so the body weighs what the fragment's samples do
//!     │   recentred on its colliders: the origin moves to the centre of mass
//!     ▼
//!   PlacedBoulder { body, model (mesh around the centre of mass), centre }
//! ```
//!
//! The body starts still but spinning, so it is awake. The blast's shove,
//! queued for the next physics step, throws it with every other body near the
//! blast; by then the terrain has been remeshed without it.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};

use super::brick_shaper::Bricks;
use super::dust::CrumbleSize;
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc, RigidBodyHandle};
use crate::terrain::{FragmentMesh, VoxelMaterial};

/// Damping a boulder tumbles with, per second. Rock does not roll for long:
/// its corners catch, which bricks only roughly have.
const LINEAR_DAMPING: f32 = 0.05;
const ANGULAR_DAMPING: f32 = 0.3;

/// A fragment to be made a body.
pub struct Boulder {
    /// Its collision shape.
    pub bricks: Bricks,
    /// Its render mesh, around [`FragmentMesh::origin`].
    pub mesh: FragmentMesh,
    /// What its samples weigh, in kg.
    pub mass: f32,
    /// Its spin as it breaks away, rad/s.
    pub spin: Vector3<f32>,
    /// What most of it is made of: its grip and bounce.
    pub material: VoxelMaterial,
    /// Its volume, the measure the debris budget ranks by, in m³.
    pub volume: f32,
    /// The crumble it would make.
    pub size: CrumbleSize,
}

/// A boulder in the physics world.
pub struct PlacedBoulder {
    pub body: RigidBodyHandle,
    /// Its render mesh around the body's origin.
    pub mesh: FragmentMesh,
    /// The body's origin, its centre of mass, at the blast.
    pub centre: Point3<f32>,
}

impl Boulder {
    /// Add the boulder to `world`.
    pub fn place(self, world: &mut PhysicsWorld) -> PlacedBoulder {
        let origin = self.mesh.origin;
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(origin)
                .angular_velocity(self.spin)
                .linear_damping(LINEAR_DAMPING)
                .angular_damping(ANGULAR_DAMPING),
        );

        let density = self.mass / self.bricks.volume().max(f32::EPSILON);
        let surface = self.material.surface();
        for brick in self.bricks.bricks {
            let mut desc = ColliderDesc::convex_hull(Arc::new(brick.hull))
                .density(density)
                .offset_translation(brick.centre - origin)
                .offset_rotation(self.bricks.rotation);
            if let Some(surface) = surface {
                desc = desc
                    .friction(surface.friction)
                    .restitution(surface.restitution);
            }
            world.attach_collider(body, desc);
        }

        let moved = world.recenter_on_colliders(body);
        let mut mesh = self.mesh;
        for vertex in &mut mesh.vertices {
            vertex.pos -= moved;
        }
        mesh.origin += moved;
        PlacedBoulder {
            body,
            centre: mesh.origin,
            mesh,
        }
    }
}
