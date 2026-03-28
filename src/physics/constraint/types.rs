//! Constraint type definitions: persistent constraints, solver-ready rows, and the kind enum.

use nalgebra::{UnitQuaternion, UnitVector3, Vector3};
use smallvec::SmallVec;

use crate::physics::handle::RigidBodyHandle;

/// Handle to a constraint in the physics world.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct ConstraintHandle(pub(crate) generational_arena::Index);

/// User-facing constraint definition. Each variant carries the parameters
/// needed to expand into one or more solver-ready `ConstraintRow`s.
pub enum ConstraintKind {
    /// Keep a body's world-space up-axis aligned with a target direction.
    /// Produces 2 constraint rows (tilt around two axes perpendicular to target).
    KeepUpright {
        /// The body to keep upright.
        body: RigidBodyHandle,
        /// Target up direction (world-space, typically +Y).
        target_up: UnitVector3<f32>,
        /// Angular compliance (0 = perfectly rigid, >0 = soft).
        /// Folded into effective mass as `1 / (J·M⁻¹·Jᵀ + compliance/dt²)`.
        compliance: f32,
    },

    /// Weld two bodies together at fixed relative position and orientation.
    /// Produces 3 positional rows (X, Y, Z) + 3 angular rows = 6 total.
    /// Unbounded impulse limits for maximum stiffness within PGS.
    Weld {
        body_a: RigidBodyHandle,
        body_b: RigidBodyHandle,
        /// Anchor point in body_a's local space.
        local_anchor_a: Vector3<f32>,
        /// Anchor point in body_b's local space.
        local_anchor_b: Vector3<f32>,
        /// Relative orientation of body_b in body_a's frame at creation time.
        relative_orientation: UnitQuaternion<f32>,
        /// Positional compliance (0 = rigid).
        compliance: f32,
        /// Angular compliance (0 = rigid).
        angular_compliance: f32,
    },

    /// Drive a point on body_b toward a point on body_a, with optional
    /// orientation locking.
    /// Produces 3 positional rows (X, Y, Z) + 3 angular rows = 6 total.
    /// Newton's third law is automatic — the solver applies equal and opposite
    /// forces, so heavy held objects resist the player's movement and turning.
    FollowPoint {
        /// The anchor body (e.g. the player). Its local_anchor_a defines the
        /// hold point relative to its center of mass.
        body_a: RigidBodyHandle,
        /// Body-local offset on body_a where the hold point is.
        local_anchor_a: Vector3<f32>,
        /// The driven body (e.g. the held object).
        body_b: RigidBodyHandle,
        /// Body-local offset on body_b (the grab surface point).
        local_anchor_b: Vector3<f32>,
        /// Softness for the positional constraint.
        compliance: f32,
        /// Maximum impulse per axis per substep for the positional constraint.
        max_impulse: f32,
        /// Relative orientation of body_b in body_a's frame at the time of
        /// grab. The angular rows drive the current relative orientation back
        /// toward this snapshot.
        relative_orientation: UnitQuaternion<f32>,
        /// Softness for the angular (orientation-lock) constraint.
        angular_compliance: f32,
        /// Maximum angular impulse per axis per substep. Controls how much
        /// torque the grab can exert — light objects lock orientation, heavy
        /// objects droop under gravity.
        angular_max_impulse: f32,
    },
}

impl ConstraintKind {
    /// Number of solver rows this constraint expands to.
    pub fn row_count(&self) -> usize {
        match self {
            ConstraintKind::KeepUpright { .. } => 2,
            ConstraintKind::Weld { .. } => 6,
            ConstraintKind::FollowPoint { .. } => 6,
        }
    }

    /// Whether this constraint references the given body.
    pub fn references_body(&self, handle: RigidBodyHandle) -> bool {
        match self {
            ConstraintKind::KeepUpright { body, .. } => *body == handle,
            ConstraintKind::Weld {
                body_a, body_b, ..
            }
            | ConstraintKind::FollowPoint {
                body_a, body_b, ..
            } => *body_a == handle || *body_b == handle,
        }
    }

    /// All body handles referenced by this constraint.
    pub fn referenced_bodies(&self) -> SmallVec<[RigidBodyHandle; 2]> {
        match self {
            ConstraintKind::KeepUpright { body, .. } => smallvec::smallvec![*body],
            ConstraintKind::Weld {
                body_a, body_b, ..
            }
            | ConstraintKind::FollowPoint {
                body_a, body_b, ..
            } => smallvec::smallvec![*body_a, *body_b],
        }
    }
}

/// Persistent constraint stored in the physics world arena.
pub struct Constraint {
    /// The constraint definition.
    pub kind: ConstraintKind,
    /// Cached impulses for warm-starting (one per row the kind expands to).
    pub warm_impulses: SmallVec<[f32; 6]>,
    /// Whether this constraint is active.
    pub active: bool,
}

impl Constraint {
    /// Create a new active constraint with zeroed warm-start impulses.
    pub fn new(kind: ConstraintKind) -> Self {
        let row_count = kind.row_count();
        Self {
            kind,
            warm_impulses: smallvec::smallvec![0.0; row_count],
            active: true,
        }
    }
}

/// Flat, uniform struct the solver iterates during the PGS loop.
///
/// Each user-defined `Constraint` is expanded into one or more of these
/// before the solve loop begins. The solver treats all rows identically
/// regardless of which constraint type produced them.
pub struct ConstraintRow {
    /// First constrained body (None for world-anchored).
    pub body_a: Option<RigidBodyHandle>,
    /// Second constrained body (None for world-anchored).
    pub body_b: Option<RigidBodyHandle>,
    /// Linear Jacobian for body A (world-space).
    pub lin_jac_a: Vector3<f32>,
    /// Angular Jacobian for body A (world-space).
    pub ang_jac_a: Vector3<f32>,
    /// Linear Jacobian for body B (world-space).
    pub lin_jac_b: Vector3<f32>,
    /// Angular Jacobian for body B (world-space).
    pub ang_jac_b: Vector3<f32>,
    /// Precomputed 1 / (J·M⁻¹·Jᵀ + compliance/dt²).
    pub effective_mass_inv: f32,
    /// Velocity bias (position correction or motor target).
    pub bias: f32,
    /// Accumulated impulse for warm-start and clamping.
    pub accumulated_impulse: f32,
    /// Impulse bounds (min, max).
    pub bounds: (f32, f32),
    /// Index of the parent Constraint in the arena (raw arena index).
    /// Used for warm-start write-back after solving.
    pub constraint_index: generational_arena::Index,
    /// Which row within the parent constraint (0, 1, ...).
    /// Used to write accumulated_impulse back to the correct warm_impulses slot.
    pub row_index: usize,
}
