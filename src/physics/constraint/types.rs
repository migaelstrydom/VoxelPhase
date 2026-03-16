//! Constraint type definitions: persistent constraints, solver-ready rows, and the kind enum.

use nalgebra::{UnitVector3, Vector3};
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
}

impl ConstraintKind {
    /// Number of solver rows this constraint expands to.
    pub fn row_count(&self) -> usize {
        match self {
            ConstraintKind::KeepUpright { .. } => 2,
        }
    }

    /// Whether this constraint references the given body.
    pub fn references_body(&self, handle: RigidBodyHandle) -> bool {
        match self {
            ConstraintKind::KeepUpright { body, .. } => *body == handle,
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
