//! What a sweep found, expressed in the same terms as a discrete contact.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::collision::contact::FeatureId;
use crate::physics::drive::plan::TractionRow;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::{PairHeader, SolverContact};

/// An impact found by sweeping, ready to be turned into solver contacts.
///
/// Carries a [`PairHeader`] rather than naming what was hit, so an impact
/// against terrain and an impact against another body are the same type. That
/// is the same trick the discrete pipeline plays: the `Option` in
/// `PairHeader::body_a` absorbs the static case, and everything downstream —
/// the solver, the contact events, the impact ledger — stays single-path.
pub(super) struct SweptImpact {
    /// Pair metadata with combined material, in the solver's A→B convention.
    pub header: PairHeader,
    /// Fraction of the substep's motion at which contact occurs, in `[0, 1]`.
    pub toi: f32,
    pub point: Point3<f32>,
    pub normal: Vector3<f32>,
    /// The body to clamp back to the time of impact, and its orientation there.
    ///
    /// Only one side is ever clamped: the candidate that was swept. Its partner
    /// keeps its integrated position, and the solver takes over from there.
    pub clamped: RigidBodyHandle,
    pub clamped_rotation: UnitQuaternion<f32>,
}

/// Build a cold (no warm-start) `SolverContact` for transient CCD contacts.
///
/// CCD contacts live for exactly one solve. They are never entered into the
/// manifold cache — the headers that carry them leave `collider_b` empty, which
/// is what the cache keys on — so there is no previous frame's impulse to warm
/// start from and none of theirs to hand on.
pub(super) fn cold_solver_contact(
    point: Point3<f32>,
    normal: Vector3<f32>,
    raw_normal: Vector3<f32>,
    depth: f32,
    raw_depth: f32,
    feature_id: FeatureId,
) -> SolverContact {
    SolverContact {
        point,
        normal,
        raw_normal,
        depth,
        raw_depth,
        feature_id,
        warm_normal_impulse: 0.0,
        warm_friction_impulse_ws: Vector3::zeros(),
        accumulated_normal_impulse: 0.0,
        accumulated_friction_impulse_ws: Vector3::zeros(),
        tangential_scale: 1.0,
        traction: TractionRow::default(),
    }
}
