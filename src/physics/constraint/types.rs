//! Constraint type definitions: persistent constraints, solver-ready rows, and the kind enum.

use nalgebra::{Isometry3, Point3, UnitQuaternion, UnitVector3, Vector3};
use smallvec::SmallVec;

use crate::physics::handle::RigidBodyHandle;

use super::spring_slider::{self, SpringSlider};
use super::suspension::{self, Suspension};

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
        /// Angular compliance, in rad per N·m (0 = perfectly rigid).
        ///
        /// A real spring: the body settles where the torque on it balances the
        /// row, leaning `compliance · torque / constraint_position_beta` out of
        /// true and ringing back when the load comes off. Compliance also rules
        /// out `Enforcement::HardProjection` and the NGS position pass, both of
        /// which would stand the body up regardless and leave the spring with
        /// nothing to do.
        ///
        /// `platform::DeckSuspension` authors this end of it in degrees under a
        /// stated load, which is the form the number is meaningful in.
        compliance: f32,
        /// Bound on the angular impulse each row may accumulate, matching the
        /// `max_impulse` convention of the other constraint kinds.
        /// `f32::INFINITY` means unlimited authority: the body is held upright
        /// no matter what load is placed on it.
        ///
        /// A finite bound models an attitude control system with a real
        /// actuator behind it — a thruster-stabilised platform tips under an
        /// off-centre load and rights itself only if it has authority to
        /// spare. It is the impulse each row may deliver per substep, so
        /// realised torque is `max_impulse / substep_dt` at saturation.
        /// Righting rate is linear in this value; tune by feel against a
        /// reference load.
        ///
        /// A finite bound also forces velocity-level solving: hard projection
        /// and NGS position correction both bypass impulse bounds, so they are
        /// disabled when authority is limited. See `keep_upright::expand`.
        ///
        /// Note that `RigidBodyDesc`'s default `angular_damping` of 0.05 will
        /// dominate small bounds — check it before concluding a dial is dead.
        max_impulse: f32,
    },

    /// Hold a body's pitch and roll about its own heading, leaving rotation
    /// about world up free. Produces 2 constraint rows. At `pitch` 0 it holds
    /// a body upright exactly as `KeepUpright` with a +Y target does; pitched,
    /// it lays the body's +Y over toward its heading and still lets it turn.
    /// See `keep_attitude`.
    KeepAttitude {
        /// The body to hold.
        body: RigidBodyHandle,
        /// How far the body's +Y is tipped from vertical toward its heading,
        /// in radians. Meant to be changed while the constraint lives: the
        /// rows chase it at `beta`.
        pitch: f32,
        /// Angular compliance, in rad per N·m (0 = perfectly rigid). As
        /// `KeepUpright`'s.
        compliance: f32,
        /// Bound on the angular impulse each row may accumulate. As
        /// `KeepUpright`'s.
        max_impulse: f32,
    },

    /// Locks all 6 DOF between two bodies (or body to world). Equivalent
    /// to a weld joint. Produces 3 positional rows (X, Y, Z) + 3 angular
    /// rows = 6 total.
    Fixed {
        /// The first body (None for world-anchored).
        body_a: Option<RigidBodyHandle>,
        /// The second body.
        body_b: RigidBodyHandle,
        /// Body-local anchor on body_a (or world position if body_a is None).
        local_anchor_a: Vector3<f32>,
        /// Body-local anchor on body_b.
        local_anchor_b: Vector3<f32>,
        /// The rotation the joint holds body_b at, relative to body_a's
        /// (or to the world when world-anchored): `rot_a⁻¹ · rot_b`.
        reference: UnitQuaternion<f32>,
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
        /// How fast anchor a is being moved over body_a (or the world), in
        /// world space, m/s: zero for a fixed anchor. An owner that moves
        /// the anchor each frame states its speed here, so the rows carry
        /// body_b along at it; otherwise only the position correction moves
        /// body_b, while the rows hold its velocity to the anchor standing
        /// still.
        anchor_velocity: Vector3<f32>,
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

    /// Holds body_b on a line through body_a, sprung along it: a prismatic
    /// joint whose free axis pushes back toward rest. See `spring_slider`.
    SpringSlider(SpringSlider),

    /// A medium-anchored drive: world-anchored motor rows that push the body
    /// toward a target velocity against nothing at all.
    ///
    /// The honest form of a platform's motor, a thruster or a rotor — the
    /// reaction goes into the world, and the entity says so through
    /// `ReactionAnchor::Medium`. Produces 3 linear rows (X, Y, Z) + 3 angular
    /// rows = 6 total. Owned by `PhysicsWorld::set_body_drive`, which is the
    /// only thing that may create, update or retire one: a body carrying both
    /// this and a support drive would get twice the authority its actuator
    /// declares.
    MediumDrive {
        /// The driven body.
        body: RigidBodyHandle,
        /// Target linear velocity, in world space.
        linear_target: Vector3<f32>,
        /// Target angular velocity, in world space.
        angular_target: Vector3<f32>,
        /// Linear acceleration budget, in m/s². Bounds each linear row at the
        /// impulse that changes the body's velocity by `max_accel · dt`.
        max_accel: f32,
        /// Angular acceleration budget, in rad/s².
        angular_max_accel: f32,
    },

    /// A body held up on a spring above whatever a cast from it hits, with
    /// friction where it stands. Produces 4 rows: spring, friction pair,
    /// torsion. See `suspension`.
    Suspension(Box<Suspension>),
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
            anchor_velocity: Vector3::zeros(),
            local_anchor_b: local_anchor,
            compliance,
            max_impulse,
        }
    }

    /// Convenience constructor for a world-anchored fixed joint (weld to world).
    ///
    /// Locks all 6 DOF, pinning the body with `local_anchor` at `world_anchor`
    /// and turned to `body_rotation`.
    pub fn world_fixed(
        body: RigidBodyHandle,
        world_anchor: Point3<f32>,
        local_anchor: Vector3<f32>,
        body_rotation: &UnitQuaternion<f32>,
        compliance: f32,
        max_impulse: f32,
    ) -> Self {
        ConstraintKind::Fixed {
            body_a: None,
            body_b: body,
            local_anchor_a: world_anchor.coords,
            local_anchor_b: local_anchor,
            reference: *body_rotation,
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
            ConstraintKind::KeepUpright { .. } | ConstraintKind::KeepAttitude { .. } => 2,
            ConstraintKind::BallJoint { .. } => 3,
            ConstraintKind::Fixed { .. } => 6,
            ConstraintKind::Hinge { .. } => 5,
            ConstraintKind::FollowPoint { .. } => 6,
            ConstraintKind::SpringSlider(_) => spring_slider::ROW_COUNT,
            ConstraintKind::MediumDrive { .. } => crate::physics::drive::medium::ROW_COUNT,
            ConstraintKind::Suspension(_) => suspension::ROW_COUNT,
        }
    }

    /// Whether the body this holds may sleep while it is active.
    ///
    /// A joint that ties a body to the world and nothing else answers only to
    /// that body's own motion: once the body is still, nothing the joint does
    /// will move it. Every other kind either drives its body or ties it to
    /// another body that may still be moving.
    pub fn permits_sleep(&self) -> bool {
        match self {
            ConstraintKind::Fixed { body_a, .. }
            | ConstraintKind::BallJoint { body_a, .. }
            | ConstraintKind::Hinge { body_a, .. } => body_a.is_none(),
            // A spring's body slows to nothing at the end of every swing,
            // where the sleep test's speed threshold would freeze it short
            // of rest.
            ConstraintKind::SpringSlider(_)
            | ConstraintKind::KeepUpright { .. }
            | ConstraintKind::KeepAttitude { .. }
            | ConstraintKind::FollowPoint { .. }
            | ConstraintKind::MediumDrive { .. } => false,
            // Standing on the world answers only to the body's own motion;
            // standing on a body that moves ties the two together, and
            // islands are built from contacts alone, so both stay awake.
            ConstraintKind::Suspension(suspension) => suspension.partner().is_none(),
        }
    }

    /// Whether this holds its body's position against the world, so that the
    /// body bears a load the way the ground does.
    ///
    /// A joint to the world does, and so does a medium drive, which pushes
    /// against the world for whatever rides on it. Holding an attitude does
    /// not: a capsule kept upright still falls, and what rests on it is
    /// resting on whatever it stands on.
    pub fn anchors_position(&self) -> bool {
        match self {
            ConstraintKind::Fixed { body_a, .. }
            | ConstraintKind::BallJoint { body_a, .. }
            | ConstraintKind::Hinge { body_a, .. } => body_a.is_none(),
            ConstraintKind::SpringSlider(slider) => slider.body_a.is_none(),
            ConstraintKind::MediumDrive { .. } => true,
            ConstraintKind::KeepUpright { .. }
            | ConstraintKind::KeepAttitude { .. }
            | ConstraintKind::FollowPoint { .. } => false,
            // A suspension holds its body up the way a contact does, and a
            // contact is not an anchor either.
            ConstraintKind::Suspension(_) => false,
        }
    }

    /// Whether the bodies this ties together stack as one: whether shock
    /// propagation may order them, making the lower appear heavier to what
    /// rests on the upper.
    ///
    /// A spring does not. Its whole response is the give between its
    /// bodies, and a sprung body made to look heavier to what lands on it
    /// stops that load dead instead of taking it into the spring.
    pub fn stacks(&self) -> bool {
        !matches!(
            self,
            ConstraintKind::SpringSlider(_) | ConstraintKind::Suspension(_)
        )
    }

    /// Follows `handle`'s origin moving by `delta`, in its own frame (a
    /// re-centre after it lost colliders): every anchor this constraint holds
    /// on it shifts by `-delta`, so it stays on the same point of the body.
    ///
    /// The joints are the kinds that hold anchors on a body that can lose
    /// colliders; a drive, a slider or a suspension is authored on a body
    /// that keeps its own, and is left as it is.
    pub fn shift_anchor(&mut self, handle: RigidBodyHandle, delta: Vector3<f32>) {
        match self {
            ConstraintKind::BallJoint {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                ..
            }
            | ConstraintKind::Fixed {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                ..
            }
            | ConstraintKind::Hinge {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                ..
            } => {
                if *body_a == Some(handle) {
                    *local_anchor_a -= delta;
                }
                if *body_b == handle {
                    *local_anchor_b -= delta;
                }
            }
            ConstraintKind::FollowPoint {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                ..
            } => {
                if *body_a == handle {
                    *local_anchor_a -= delta;
                }
                if *body_b == handle {
                    *local_anchor_b -= delta;
                }
            }
            ConstraintKind::KeepUpright { .. }
            | ConstraintKind::KeepAttitude { .. }
            | ConstraintKind::SpringSlider(_)
            | ConstraintKind::MediumDrive { .. }
            | ConstraintKind::Suspension(_) => {}
        }
    }

    /// Whether this constraint references the given body.
    pub fn references_body(&self, handle: RigidBodyHandle) -> bool {
        match self {
            ConstraintKind::KeepUpright { body, .. }
            | ConstraintKind::KeepAttitude { body, .. } => *body == handle,
            ConstraintKind::BallJoint { body_a, body_b, .. }
            | ConstraintKind::Fixed { body_a, body_b, .. }
            | ConstraintKind::Hinge { body_a, body_b, .. } => {
                body_a.map_or(false, |a| a == handle) || *body_b == handle
            }
            ConstraintKind::SpringSlider(slider) => {
                slider.body_a == Some(handle) || slider.body_b == handle
            }
            ConstraintKind::FollowPoint { body_a, body_b, .. } => {
                *body_a == handle || *body_b == handle
            }
            ConstraintKind::MediumDrive { body, .. } => *body == handle,
            ConstraintKind::Suspension(suspension) => {
                suspension.body() == handle || suspension.partner() == Some(handle)
            }
        }
    }

    /// All body handles referenced by this constraint.
    pub fn referenced_bodies(&self) -> SmallVec<[RigidBodyHandle; 2]> {
        match self {
            ConstraintKind::KeepUpright { body, .. }
            | ConstraintKind::KeepAttitude { body, .. } => {
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
            ConstraintKind::SpringSlider(slider) => {
                let mut bodies = SmallVec::new();
                if let Some(a) = slider.body_a {
                    bodies.push(a);
                }
                bodies.push(slider.body_b);
                bodies
            }
            ConstraintKind::FollowPoint { body_a, body_b, .. } => {
                smallvec::smallvec![*body_a, *body_b]
            }
            ConstraintKind::MediumDrive { body, .. } => smallvec::smallvec![*body],
            ConstraintKind::Suspension(suspension) => {
                let mut bodies = SmallVec::new();
                bodies.push(suspension.body());
                if let Some(partner) = suspension.partner() {
                    bodies.push(partner);
                }
                bodies
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
    pub effective_mass: f32,
    /// Velocity bias (position correction or motor target).
    pub bias: f32,
    /// Softness: `compliance / dt²`, the same term that softens
    /// `effective_mass`. Zero for a rigid row.
    ///
    /// It appears a second time in the solve, multiplying the accumulated
    /// impulse, and that is what makes a compliant row a spring: the row
    /// settles where its impulse balances the softness term rather than
    /// driving the error to zero. Softening the effective mass alone only
    /// slows convergence down — the row still ends up rigid, just later.
    pub softness: f32,
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
    /// How the row recomputes its bias before each substep, for a row whose
    /// force must follow its error (a spring). `None`: the bias set at
    /// expansion holds for the whole frame.
    pub refresh: Option<RowRefresh>,
    /// A bound that follows a sibling row's live impulse rather than the
    /// fixed `bounds`. `None`: `bounds` applies.
    pub coupling: Option<CoupledBound>,
}

/// What a refreshed row needs to recompute its bias from the bodies' motion
/// since the row was expanded.
///
/// The row's error moves, to first order, by the Jacobian times the
/// displacement of its bodies, so the bias before each substep is
/// `bias_rate · (error + J · Δq)`.
#[derive(Debug, Clone, Copy)]
pub struct RowRefresh {
    /// The row's position error when it was expanded, in the units its
    /// Jacobian measures (metres along a linear row).
    pub error: f32,
    /// Bias per unit of error, in 1/s: the bias is `bias_rate · error`.
    pub bias_rate: f32,
    /// Body A's pose at expansion; `None` for a world-anchored side.
    pub pose_a: Option<Isometry3<f32>>,
    /// Body B's pose at expansion; `None` for a world-anchored side.
    pub pose_b: Option<Isometry3<f32>>,
}

/// A row's impulse bound given as a multiple of a sibling row's live
/// accumulated impulse: the way friction is bounded by load.
///
/// The sibling is named by its `row_index` within the same constraint, which
/// survives any reordering of the row slice; the solver resolves it to a slot
/// when the rows are prepared, as ODE's `findex` does.
///
/// The bound covers the whole reaction along the row: the row's own impulse
/// plus the load row's share along it, `|λ + λ_load · load_share|`. Two rows
/// naming each other through `cone_with` are bounded together, as a disc,
/// rather than each on its own (a square, with √2 more grip on the
/// diagonals).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CoupledBound {
    /// `row_index` of the load row this bound follows.
    pub coupled_to: u32,
    /// `row_index` of the row sharing this one's cone, if any.
    pub cone_with: Option<u32>,
    /// Bound per unit of load impulse: `μ` times whatever turns the load
    /// row's impulse into a normal load (and a patch radius for a torsional
    /// row).
    pub coefficient: f32,
    /// The load row's impulse along this row's direction, per unit of its
    /// impulse.
    pub load_share: f32,
}

impl CoupledBound {
    /// The bound on the whole reaction for a load row's impulse `load`. A
    /// load row pulling rather than pushing bounds it at zero.
    pub fn radius(&self, load: f32) -> f32 {
        self.coefficient * load.max(0.0)
    }
}
