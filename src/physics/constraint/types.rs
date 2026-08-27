//! Constraint type definitions: persistent constraints, solver-ready rows, and the kind enum.

use nalgebra::{Point3, UnitQuaternion, UnitVector3, Vector3};
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
        /// Bound on the angular impulse each row may accumulate, matching the
        /// `max_impulse` convention of the other constraint kinds.
        /// `f32::INFINITY` means unlimited authority: the body is held upright
        /// no matter what load is placed on it.
        ///
        /// A finite bound models an attitude control system with a real
        /// actuator behind it — a thruster-stabilised platform tips under an
        /// off-centre load and rights itself only if it has authority to
        /// spare. Righting rate is linear in this value, so it works well as a
        /// tuning dial, but it is *not* calibrated in N·m: the solver clamps
        /// accumulated impulse once per frame while warm-starting re-applies it
        /// each substep, so realised torque also depends on substep count and
        /// `warm_start_scale`. Tune by feel against a reference load.
        ///
        /// A finite bound also forces velocity-level solving: hard projection
        /// and NGS position correction both bypass impulse bounds, so they are
        /// disabled when authority is limited. See `keep_upright::expand`.
        ///
        /// UNSTABLE — prefer `f32::INFINITY` for now. A bound low enough to
        /// actually saturate injects energy rather than bleeding it: the row
        /// re-applies a frame's accumulated impulse once per substep via
        /// warm-starting, and unlike an unbounded row it cannot undo the
        /// over-application on the next iteration. Measured divergence to
        /// ±47 rad/s on a 52 kg·m² body at bounds of 50–200. Fixing it means
        /// decrementing `accumulated_impulse` on warm start, which is shared
        /// with contacts and every other joint.
        ///
        /// Note that `RigidBodyDesc`'s default `angular_damping` of 0.05 will
        /// dominate small bounds — check it before concluding a dial is dead.
        max_impulse: f32,
    },

    /// Locks all 6 DOF between two bodies (or body to world). Equivalent
    /// to a weld joint. Produces 3 positional rows (X, Y, Z) + 3 angular
    /// rows = 6 total. For world-anchored Fixed with compliance=0, the tilt
    /// angular rows get `Enforcement::HardProjection`.
    Fixed {
        /// The first body (None for world-anchored).
        body_a: Option<RigidBodyHandle>,
        /// The second body.
        body_b: RigidBodyHandle,
        /// Body-local anchor on body_a (or world position if body_a is None).
        local_anchor_a: Vector3<f32>,
        /// Body-local anchor on body_b.
        local_anchor_b: Vector3<f32>,
        /// Positional compliance (0 = perfectly rigid).
        compliance: f32,
        /// Maximum impulse per axis per substep. Finite values make the
        /// joint breakable — if any row saturates, the joint deactivates.
        max_impulse: f32,
    },

    /// Constrains two anchor points to coincide. 3 DOF locked (translation),
    /// 3 DOF free (rotation). Produces 3 positional rows (X, Y, Z).
    BallJoint {
        /// The first body (None for world-anchored).
        body_a: Option<RigidBodyHandle>,
        /// The second body.
        body_b: RigidBodyHandle,
        /// Body-local anchor on body_a (or world position if body_a is None).
        local_anchor_a: Vector3<f32>,
        /// Body-local anchor on body_b.
        local_anchor_b: Vector3<f32>,
        /// Positional compliance (0 = perfectly rigid).
        compliance: f32,
        /// Maximum impulse per axis per substep. Finite values make the
        /// joint breakable — if any row saturates, the joint deactivates.
        max_impulse: f32,
    },

    /// Constrains two anchor points to coincide and restricts rotation to
    /// one axis (the hinge axis). 5 DOF locked, 1 DOF free.
    /// Produces 3 positional rows (X, Y, Z) + 2 angular rows = 5 total.
    Hinge {
        /// The first body (None for world-anchored hinges).
        body_a: Option<RigidBodyHandle>,
        /// The second body.
        body_b: RigidBodyHandle,
        /// Body-local anchor on body_a (or world position if body_a is None).
        local_anchor_a: Vector3<f32>,
        /// Body-local anchor on body_b.
        local_anchor_b: Vector3<f32>,
        /// Hinge axis in body_b's local frame.
        local_axis_b: UnitVector3<f32>,
        /// Hinge axis in body_a's local frame (or world-space if body_a is None).
        local_axis_a: UnitVector3<f32>,
        /// Reference axis perpendicular to the hinge axis in body_b's local
        /// frame. Used to construct a stable perpendicular basis for the
        /// angular lock rows and to measure hinge angle for limits/motors.
        local_ref_b: UnitVector3<f32>,
        /// Positional compliance (0 = perfectly rigid).
        compliance: f32,
        /// Maximum impulse per axis per substep.
        max_impulse: f32,
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
    /// Convenience constructor for a world-anchored ball joint.
    ///
    /// Pins the body's local anchor to a fixed world position with 3 DOF free
    /// (all rotation).
    pub fn world_ball_joint(
        body: RigidBodyHandle,
        world_anchor: Point3<f32>,
        local_anchor: Vector3<f32>,
        compliance: f32,
        max_impulse: f32,
    ) -> Self {
        ConstraintKind::BallJoint {
            body_a: None,
            body_b: body,
            local_anchor_a: world_anchor.coords,
            local_anchor_b: local_anchor,
            compliance,
            max_impulse,
        }
    }

    /// Convenience constructor for a world-anchored fixed joint (weld to world).
    ///
    /// Locks all 6 DOF, pinning the body at its current position and orientation.
    pub fn world_fixed(
        body: RigidBodyHandle,
        world_anchor: Point3<f32>,
        local_anchor: Vector3<f32>,
        compliance: f32,
        max_impulse: f32,
    ) -> Self {
        ConstraintKind::Fixed {
            body_a: None,
            body_b: body,
            local_anchor_a: world_anchor.coords,
            local_anchor_b: local_anchor,
            compliance,
            max_impulse,
        }
    }

    /// Convenience constructor for a world-anchored hinge.
    ///
    /// `hinge_axis` is in world space; it is converted to body-local for both
    /// the axis and reference vectors.
    pub fn world_hinge(
        body: RigidBodyHandle,
        world_anchor: Point3<f32>,
        local_anchor: Vector3<f32>,
        hinge_axis: UnitVector3<f32>,
        body_rotation: &UnitQuaternion<f32>,
        compliance: f32,
        max_impulse: f32,
    ) -> Self {
        let inv_rot = body_rotation.inverse();
        let local_axis_b = UnitVector3::new_normalize(inv_rot * hinge_axis.into_inner());

        // Pick the cardinal axis least aligned with local_axis_b for a stable
        // perpendicular reference.
        let abs_a = local_axis_b.into_inner().map(|c| c.abs());
        let seed = if abs_a.x <= abs_a.y && abs_a.x <= abs_a.z {
            Vector3::x()
        } else if abs_a.y <= abs_a.z {
            Vector3::y()
        } else {
            Vector3::z()
        };
        let local_ref_b =
            UnitVector3::new_normalize(local_axis_b.into_inner().cross(&seed).normalize());

        ConstraintKind::Hinge {
            body_a: None,
            body_b: body,
            local_anchor_a: world_anchor.coords,
            local_anchor_b: local_anchor,
            local_axis_b,
            local_axis_a: hinge_axis,
            local_ref_b,
            compliance,
            max_impulse,
        }
    }

    /// Number of solver rows this constraint expands to.
    pub fn row_count(&self) -> usize {
        match self {
            ConstraintKind::KeepUpright { .. } => 2,
            ConstraintKind::BallJoint { .. } => 3,
            ConstraintKind::Fixed { .. } => 6,
            ConstraintKind::Hinge { .. } => 5,
            ConstraintKind::FollowPoint { .. } => 6,
        }
    }

    /// Whether this constraint references the given body.
    pub fn references_body(&self, handle: RigidBodyHandle) -> bool {
        match self {
            ConstraintKind::KeepUpright { body, .. } => *body == handle,
            ConstraintKind::BallJoint { body_a, body_b, .. }
            | ConstraintKind::Fixed { body_a, body_b, .. }
            | ConstraintKind::Hinge { body_a, body_b, .. } => {
                body_a.map_or(false, |a| a == handle) || *body_b == handle
            }
            ConstraintKind::FollowPoint { body_a, body_b, .. } => {
                *body_a == handle || *body_b == handle
            }
        }
    }

    /// All body handles referenced by this constraint.
    pub fn referenced_bodies(&self) -> SmallVec<[RigidBodyHandle; 2]> {
        match self {
            ConstraintKind::KeepUpright { body, .. } => {
                smallvec::smallvec![*body]
            }
            ConstraintKind::BallJoint { body_a, body_b, .. }
            | ConstraintKind::Fixed { body_a, body_b, .. }
            | ConstraintKind::Hinge { body_a, body_b, .. } => {
                let mut bodies = SmallVec::new();
                if let Some(a) = body_a {
                    bodies.push(*a);
                }
                bodies.push(*body_b);
                bodies
            }
            ConstraintKind::FollowPoint { body_a, body_b, .. } => {
                smallvec::smallvec![*body_a, *body_b]
            }
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

/// Controls whether the NGS position correction pass processes this row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CorrectionMode {
    /// PGS velocity solving only. No position correction.
    /// Use for: friction rows, motor rows, angular rows without positional drift.
    VelocityOnly,
    /// PGS velocity solving + NGS position correction.
    /// Use for: joint position locks, contact normals.
    PositionAndVelocity,
}

/// Identifies what kind of correction the row applies if its `CorrectionMode`
/// permits position correction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RowKind {
    /// Position correction moves bodies along a linear axis.
    Linear,
    /// Position correction rotates bodies around an angular axis.
    Angular,
}

/// Controls whether the post-solve projection pass enforces this row exactly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Enforcement {
    /// Normal iterative solving (PGS + optional NGS).
    Iterative,
    /// Post-solve direct velocity projection. The solver rewrites the
    /// velocity to satisfy the constraint exactly, bypassing iterative
    /// convergence. Used when PGS convergence is too slow for visual
    /// correctness (e.g. upright constraints, where even zero compliance
    /// allows visible drift due to limited solver iterations and
    /// Jacobian degeneracy at large angles).
    HardProjection,
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
    /// Whether NGS position correction processes this row.
    pub correction_mode: CorrectionMode,
    /// What kind of correction (linear position or angular rotation).
    pub row_kind: RowKind,
    /// Whether the post-solve projection pass enforces this row exactly.
    pub enforcement: Enforcement,
}
