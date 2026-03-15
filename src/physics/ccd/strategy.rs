use std::collections::{HashMap, HashSet};

use generational_arena::Arena;
use nalgebra::{Point3, UnitQuaternion};

use crate::physics::body::RigidBody;
use crate::physics::collider::Collider;
use crate::physics::contact_event::ContactEvent;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::static_geometry::StaticGeometry;

/// Mutable state passed to the CCD strategy each substep.
pub struct CcdContext<'a> {
    pub bodies: &'a mut Arena<RigidBody>,
    pub colliders: &'a Arena<Collider>,
    pub contact_events: &'a mut Vec<ContactEvent>,
    /// Bodies already handled by the narrowphase (excluded from CCD).
    pub narrowphase_handled: &'a HashSet<RigidBodyHandle>,
    /// Currently sleeping bodies (excluded from CCD).
    pub sleeping: Option<&'a HashSet<RigidBodyHandle>>,
    /// Body positions/rotations captured before position integration.
    pub pre_states: &'a HashMap<generational_arena::Index, (Point3<f32>, UnitQuaternion<f32>)>,
    /// Shared contact margin from PhysicsConfig.
    pub contact_margin: f32,
    /// Shared restitution velocity threshold from PhysicsConfig.
    pub restitution_velocity_threshold: f32,
    /// CCD activation threshold from PhysicsConfig.
    pub ccd_threshold: f32,
}

/// Continuous collision detection strategy.
///
/// Different CCD approaches can be plugged in:
/// - **Sweep-and-clamp**: sweep bounding sphere, clamp body to hit, solve contacts.
/// - **Speculative**: add speculative contacts without clamping.
/// - **None**: disable CCD entirely.
pub trait CcdStrategy: Send + Sync {
    /// Run CCD for one substep. Returns the number of corrections applied.
    fn run(
        &mut self,
        ctx: &mut CcdContext<'_>,
        dt: f32,
        static_geometry: &dyn StaticGeometry,
    ) -> u32;
}
