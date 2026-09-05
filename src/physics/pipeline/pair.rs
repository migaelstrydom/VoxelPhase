//! Contact types that flow through the physics pipeline.
//!
//! `PairManifold` is the output of the narrowphase. `SolverManifold` is produced
//! by the manifold cache and consumed by the solver and post-stabilizer.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::ContactManifold;
use crate::collision::contact::FeatureId;
use crate::physics::drive::plan::TractionRow;
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};

/// Metadata shared by all contacts in a collider pair manifold.
///
/// Pair-level fields are stored here once rather than duplicated on every contact point.
#[derive(Debug, Clone)]
pub struct PairHeader {
    /// First body (None for static geometry contacts).
    pub body_a: Option<RigidBodyHandle>,
    /// Second body.
    pub body_b: RigidBodyHandle,
    /// First collider (None for static geometry contacts).
    pub collider_a: Option<ColliderHandle>,
    /// Second collider.
    pub collider_b: Option<ColliderHandle>,
    /// Combined restitution coefficient for this pair.
    pub restitution: f32,
    /// Combined friction coefficient for this pair.
    pub friction: f32,
}

/// A contact manifold tagged with its pair metadata.
///
/// This is the output of the narrowphase and the input to the manifold cache.
#[derive(Debug, Clone)]
pub struct PairManifold {
    /// Body and collider metadata for this pair.
    pub header: PairHeader,
    /// Geometric contacts from the collision library (carries `FeatureId` per point).
    pub manifold: ContactManifold,
}

/// Per-contact working data for the solver.
///
/// The manifold cache produces these by merging a `PairManifold` with cached
/// impulses. The solver reads warm-start impulses and writes accumulated impulses
/// back in-place, which the cache then reads via `write_back`.
#[derive(Debug, Clone)]
pub struct SolverContact {
    /// Contact point in world space.
    pub point: Point3<f32>,
    /// Solver-facing contact normal (may be smoothed).
    pub normal: Vector3<f32>,
    /// Raw geometric normal before any smoothing.
    pub raw_normal: Vector3<f32>,
    /// Solver-facing penetration depth (>= 0).
    pub depth: f32,
    /// Raw penetration depth (may be negative for margin-only contacts).
    pub raw_depth: f32,
    /// Feature pair that produced this contact, used for impulse cache matching.
    pub feature_id: FeatureId,
    /// Normal impulse inherited from the previous frame's cache (warm-start).
    pub warm_normal_impulse: f32,
    /// Friction impulse from the previous frame, stored in world space.
    ///
    /// Perpendicular to the cached normal at the time of storage. On warm-start
    /// this is re-projected onto the current tangent plane so that small normal
    /// drift does not rotate the cached friction direction.
    pub warm_friction_impulse_ws: Vector3<f32>,
    /// Normal impulse accumulated by the solver this step.
    pub accumulated_normal_impulse: f32,
    /// Friction impulse accumulated by the solver this step, in world space.
    pub accumulated_friction_impulse_ws: Vector3<f32>,
    /// Scale on this contact's tangential budget, `1.0` for an ordinary
    /// contact.
    ///
    /// Carries the non-support grip of the bodies this contact touches: a body
    /// may scale the budget it draws at a contact that is not holding it up.
    /// The contact's own `μ` is untouched — this says what the participants
    /// are allowed to draw against it.
    pub tangential_scale: f32,
    /// What the drive asks of this contact's tangential row: the relative
    /// velocity to aim for, and the multiplier on the budget spent getting
    /// there.
    ///
    /// Default — every contact of every undriven body, and every contact CCD
    /// builds mid-substep — is ordinary friction: hold still, at the honest
    /// `μ·N`.
    pub traction: TractionRow,
}

/// A solver-ready manifold: pair metadata plus per-contact working data.
///
/// Produced by `ManifoldCache::merge` and consumed by `solve` and
/// `post_stabilize`. The solver writes accumulated impulses into each
/// `SolverContact` in-place; `ManifoldCache::write_back` reads them back.
#[derive(Debug, Clone)]
pub struct SolverManifold {
    /// Body and collider metadata for this pair.
    pub header: PairHeader,
    /// Per-contact working data, up to 4 points.
    pub contacts: SmallVec<[SolverContact; 4]>,
}
