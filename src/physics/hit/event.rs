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
    /// Each side's own mass along the normal at `point`, kg, `[a, b]`
    /// ([`Side::index`](super::side::Side::index)): what the contact would have
    /// had to stop were the other side a wall. Infinite for a side that
    /// cannot move.
    pub side_masses: [f32; 2],
    /// Whether each side, `[a, b]` ([`Side::index`](super::side::Side::index)),
    /// was backed against the blow: held by a constraint that anchors it to
    /// the world
    /// ([`anchors_position`](crate::physics::ConstraintKind::anchors_position)),
    /// or pressed the other way by another of its contacts in the same
    /// solve, as a block lying on the ground is when it is struck from
    /// above. A backed side passes the blow on rather than moving with it.
    pub backed: [bool; 2],
    /// What the pair met over.
    pub patch: Patch,
    /// The same pair was hit the frame before too. Usually that is the rest
    /// of a stop the solver split across two frames, whose approach is what
    /// the first left; but a body tumbling against the same partner (a
    /// column landing on one end and slapping down on the other) strikes it
    /// anew, so whether to skip it is the reader's call.
    pub continues: bool,
}

/// What a blast gave a body, as the step applied it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Shove {
    /// The body shoved.
    pub body: RigidBodyHandle,
    /// The impulse it was given, N·s, world space: its change of velocity
    /// is this over its mass.
    pub impulse: Vector3<f32>,
    /// Where the blast struck it: the centre of the pressure on its bulk,
    /// which spins a body loaded harder at one end than the other.
    pub point: Point3<f32>,
}
