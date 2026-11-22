use nalgebra::{Point3, Vector3};

use super::patch::Patch;
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};

/// A contact that closed faster than [`HitConfig::speed`](super::HitConfig)
/// and took impulse: one per colliding pair per frame, from the substep it
/// closed fastest in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hit {
    /// The first body; `None` for static geometry.
    pub body_a: Option<RigidBodyHandle>,
    /// The first collider; `None` for static geometry.
    pub collider_a: Option<ColliderHandle>,
    /// The second body.
    pub body_b: RigidBodyHandle,
    /// The second collider; `None` where the contact came from a sweep that
    /// keeps no collider.
    pub collider_b: Option<ColliderHandle>,
    /// Centre of the patch, world space.
    pub point: Point3<f32>,
    /// Contact normal, unit length, pointing towards `body_b`.
    pub normal: Vector3<f32>,
    /// How fast the pair closed along the normal, m/s: the fastest of the
    /// patch's contacts that took impulse, before the solve that stopped it.
    pub approach: f32,
    /// The pair's restitution, as solved.
    pub restitution: f32,
    /// How far the pair yields before it stops, m: both sides' gives in
    /// series.
    pub give: f32,
    /// Radius of the sharper side's feature, m: what a point or line contact
    /// presses with.
    pub bluntness: f32,
    /// The pair's mass along the normal at `point`, kg: what the contact had
    /// to stop. Infinite only if neither side can move.
    pub effective_mass: f32,
    /// What the pair met over.
    pub patch: Patch,
    /// The pair was hit the frame before too and is still closing: the rest
    /// of a stop the solver split across two frames, not a new blow. Its
    /// approach is what the first left.
    pub continues: bool,
}

impl Hit {
    /// Whether `body` is either side of the hit.
    pub fn involves(&self, body: RigidBodyHandle) -> bool {
        self.body_b == body || self.body_a == Some(body)
    }

    /// Which way the hit pushed `body`, unit length: along the normal for
    /// `body_b`, against it for `body_a`. `None` if `body` was not in it.
    pub fn push_on(&self, body: RigidBodyHandle) -> Option<Vector3<f32>> {
        if self.body_b == body {
            Some(self.normal)
        } else if self.body_a == Some(body) {
            Some(-self.normal)
        } else {
            None
        }
    }
}

/// What a blast gave a body, as the step applied it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Shove {
    /// The body shoved.
    pub body: RigidBodyHandle,
    /// The impulse it was given, N·s, world space: its change of velocity
    /// is this over its mass.
    pub impulse: Vector3<f32>,
    /// Where the blast struck it: its centre of mass, until a blast loads
    /// each part of a body's bulk at its own point.
    pub point: Point3<f32>,
}
