use rustc_hash::{FxHashMap, FxHashSet};

use generational_arena::Arena;
use nalgebra::{Point3, UnitQuaternion};

use crate::physics::body::RigidBody;
use crate::physics::collider::Collider;
use crate::physics::contact_event::ContactEvent;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::static_geometry::StaticGeometry;

use super::ownership::NarrowphaseOwnership;

/// Mutable state passed to the CCD strategy each substep.
pub struct CcdContext<'a> {
    pub bodies: &'a mut Arena<RigidBody>,
    pub colliders: &'a Arena<Collider>,
    pub contact_events: &'a mut Vec<ContactEvent>,
    /// Bodies the narrowphase currently owns (excluded from CCD while its
    /// frame-start manifold still describes them).
    pub narrowphase_ownership: &'a NarrowphaseOwnership,
    /// Currently sleeping bodies (excluded from CCD).
    pub sleeping: Option<&'a FxHashSet<RigidBodyHandle>>,
    /// Body positions/rotations captured before position integration.
    pub pre_states: &'a FxHashMap<generational_arena::Index, (Point3<f32>, UnitQuaternion<f32>)>,
    /// Shared contact margin from PhysicsConfig.
    pub contact_margin: f32,
    /// Shared restitution velocity threshold from PhysicsConfig.
    pub restitution_velocity_threshold: f32,
    /// Per-substep CCD activation threshold from PhysicsConfig.
    pub ccd_threshold: f32,
    /// Frame-level CCD activation threshold from PhysicsConfig.
    pub ccd_frame_coverage: f32,
    /// Substeps this frame, as declared to `update_contacts()`.
    pub substeps_per_frame: u32,
}

/// Continuous collision detection strategy.
///
/// Different CCD approaches can be plugged in:
/// - **Sweep-and-clamp**: sweep bounding sphere, clamp body to hit, solve contacts.
/// - **Speculative**: add speculative contacts without clamping.
/// - **None**: disable CCD entirely.
pub trait CcdStrategy: Send + Sync {
    /// Called once per frame, before any substep. Strategies that cache
    /// per-frame state (such as static-geometry queries) reset it here.
    ///
    /// `substeps` is how many `run()` calls this frame will make, letting a
    /// strategy size lookahead to the frame rather than assume a fixed count.
    fn begin_frame(&mut self, substeps: u32) {
        let _ = substeps;
    }

    /// Run CCD for one substep. Returns the number of corrections applied.
    fn run(
        &mut self,
        ctx: &mut CcdContext<'_>,
        dt: f32,
        static_geometry: &dyn StaticGeometry,
    ) -> u32;
}
