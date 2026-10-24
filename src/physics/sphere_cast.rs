//! The world's scene query: a sphere swept along a ray, against the static
//! geometry and every body but those excluded.
//!
//! ```text
//!   SphereCast (origin, direction, length, radius, exclude)
//!        │ PhysicsWorld::cast_sphere
//!        ▼
//!   BodySweep::cast_shape ── bodies near the ray (collider index)
//!        │                 └ static triangles in the ray's bounds
//!        ▼
//!   SphereCastHit (point, normal, face normal, distance, body, surface)
//! ```
//!
//! What asks is none of the physics' business: the follow camera casts for
//! what stands between it and its target.

use nalgebra::{Point3, Vector3};

use super::handle::RigidBodyHandle;
use super::sweep::SweptSurface;

/// A sphere to sweep along a ray.
#[derive(Debug, Clone, Copy)]
pub struct SphereCast<'a> {
    /// The sphere's centre where the sweep starts, in world space.
    pub origin: Point3<f32>,
    /// The way it travels; unit length.
    pub direction: Vector3<f32>,
    /// How far it travels, in metres.
    pub length: f32,
    /// The sphere's radius, in metres.
    pub radius: f32,
    /// Bodies the sweep passes through.
    pub exclude: &'a [RigidBodyHandle],
    /// Whether a sphere that starts inside a box or a hull body touches it
    /// there, rather than passing out through its far side unseen: true for
    /// a cast asking what stands at a point, false for one that only wants
    /// what lies below the start.
    pub solid_bodies: bool,
    /// What the sphere can touch.
    pub obstacles: CastObstacles,
}

/// What a [`SphereCast`] can touch besides the static geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CastObstacles {
    /// Every body not excluded.
    Everything,
    /// Static bodies only: the level's fixed pieces, not what moves through
    /// it.
    Scenery,
    /// Every body not excluded but dynamic ones whose surface moves faster
    /// than `fastest` m/s: what stands, not what flies.
    Settled { fastest: f32 },
}

impl SphereCast<'_> {
    /// The sphere's centre once it has travelled `distance`.
    pub fn centre_at(&self, distance: f32) -> Point3<f32> {
        self.origin + self.direction * distance
    }
}

/// Where a [`SphereCast`] first touched something.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphereCastHit {
    /// Where the sphere touched, in world space.
    pub point: Point3<f32>,
    /// Normal at the touch, out of the surface toward the sphere's centre.
    /// On an edge or a corner it is rounded, leaning between the faces there.
    pub normal: Vector3<f32>,
    /// Normal of the face the sphere arrived on: `normal` on a face, and on
    /// an edge or a corner the face beside it most squarely in the way.
    pub face_normal: Vector3<f32>,
    /// How far the sphere travelled before the touch, in metres. Zero when
    /// it started touching.
    pub distance: f32,
    /// The body touched, static bodies included; `None` for the static
    /// geometry.
    pub body: Option<RigidBodyHandle>,
    /// The surface touched, for what it is made of.
    pub surface: SweptSurface,
}
