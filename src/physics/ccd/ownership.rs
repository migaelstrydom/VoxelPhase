/// Tracks which contact pairs the narrowphase currently owns, so CCD can stand
/// down for those pairs without standing down for the bodies involved.
///
/// The narrowphase runs once per frame, but a frame is integrated over several
/// substeps. A pair that had contacts at frame start is genuinely the solver's
/// to handle — sweeping it as well would resolve the same contact twice and
/// inject phantom impulses. But that ownership expires: once the pair has moved
/// far enough that the frame-start manifold no longer describes it, the solver
/// is holding stale contacts and CCD must take over again, or the bodies can
/// sweep past each other unchecked for the rest of the frame.
///
/// Ownership is keyed by *pair*, not by body. A grenade skimming the ground has
/// a floor contact, but that says nothing about the wall it is about to hit —
/// keying by body would exclude it from CCD entirely and let it through.
///
/// Ownership is recorded with the pair's separation at the moment its manifold
/// was generated, and released once the pair has drifted a release distance
/// away from it.
use generational_arena::Arena;
use nalgebra::Vector3;
use rustc_hash::FxHashMap;

use crate::physics::body::RigidBody;
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::pipeline::pair::PairHeader;

/// Unordered identifier for a contact pair, with static geometry as `None`.
///
/// Canonicalised so that a pair keys the same however its sides were ordered —
/// `shape_type_rank` is free to swap A and B between the narrowphase and CCD.
/// Static geometry sorts first, mirroring `PairHeader`'s convention of putting
/// it in the `Option` slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ContactPairKey {
    first: Option<ColliderHandle>,
    second: ColliderHandle,
}

impl ContactPairKey {
    /// Key for a pair of colliders, in either order.
    pub fn new(a: ColliderHandle, b: ColliderHandle) -> Self {
        let (idx_a, _) = a.raw_parts();
        let (idx_b, _) = b.raw_parts();
        if idx_a <= idx_b {
            Self {
                first: Some(a),
                second: b,
            }
        } else {
            Self {
                first: Some(b),
                second: a,
            }
        }
    }

    /// Key for a collider against the static geometry of the world.
    pub fn against_static(collider: ColliderHandle) -> Self {
        Self {
            first: None,
            second: collider,
        }
    }

    /// Key for the pair a manifold header describes.
    ///
    /// `None` when the header carries no concrete collider for its `b` side,
    /// which is how CCD tags its own synthesised contacts. Those are not
    /// narrowphase output and have no ownership to record.
    pub fn from_header(header: &PairHeader) -> Option<Self> {
        let collider_b = header.collider_b?;
        Some(match header.collider_a {
            Some(collider_a) => Self::new(collider_a, collider_b),
            None => Self::against_static(collider_b),
        })
    }
}

#[derive(Default)]
pub struct NarrowphaseOwnership {
    /// Pair separation at the moment its owning manifold was generated.
    anchors: FxHashMap<ContactPairKey, Vector3<f32>>,
}

impl NarrowphaseOwnership {
    pub fn new() -> Self {
        Self::default()
    }

    /// Drop all ownership records. Called before each narrowphase pass.
    pub fn clear(&mut self) {
        self.anchors.clear();
    }

    /// Record that the narrowphase owns `key`, anchored at `separation`.
    pub fn insert(&mut self, key: ContactPairKey, separation: Vector3<f32>) {
        self.anchors.insert(key, separation);
    }

    /// Whether the narrowphase still owns `key` at its current `separation`.
    ///
    /// `release_distance` is how far the pair may drift from its anchor before
    /// the frame-start manifold is considered stale. Sized well above resting
    /// jitter so settled pairs never oscillate back into the CCD path.
    pub fn owns(
        &self,
        key: ContactPairKey,
        separation: Vector3<f32>,
        release_distance: f32,
    ) -> bool {
        let Some(anchor) = self.anchors.get(&key) else {
            return false;
        };
        (separation - anchor).magnitude_squared() <= release_distance * release_distance
    }
}

/// How far apart a pair's two sides currently sit.
///
/// Static geometry never moves, so the world origin stands in for it and the
/// separation reduces to the body's own position. Ownership then expires on the
/// body's absolute travel, which is what a body-vs-terrain manifold goes stale
/// on. For a body pair it is the relative motion that matters: two bodies
/// crossing the world side by side never invalidate their shared manifold.
pub fn pair_separation(
    bodies: &Arena<RigidBody>,
    body_a: Option<RigidBodyHandle>,
    body_b: RigidBodyHandle,
) -> Option<Vector3<f32>> {
    let pos_b = bodies.get(body_b.0)?.position();
    let pos_a = match body_a {
        Some(handle) => bodies.get(handle.0)?.position(),
        None => nalgebra::Point3::origin(),
    };
    Some(pos_b - pos_a)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collider(idx: usize) -> ColliderHandle {
        ColliderHandle(generational_arena::Index::from_raw_parts(idx, 0))
    }

    #[test]
    fn pair_key_is_order_independent() {
        assert_eq!(
            ContactPairKey::new(collider(1), collider(7)),
            ContactPairKey::new(collider(7), collider(1))
        );
    }

    #[test]
    fn static_and_body_pairs_are_distinct_keys() {
        assert_ne!(
            ContactPairKey::against_static(collider(3)),
            ContactPairKey::new(collider(3), collider(3))
        );
    }

    /// The bug this keying exists to fix: a floor contact must not suppress
    /// sweeping against a wall.
    #[test]
    fn ownership_of_one_pair_does_not_cover_another() {
        let mut ownership = NarrowphaseOwnership::new();
        let floor = ContactPairKey::against_static(collider(1));
        let wall = ContactPairKey::new(collider(1), collider(2));
        ownership.insert(floor, Vector3::zeros());

        assert!(ownership.owns(floor, Vector3::zeros(), 0.5));
        assert!(!ownership.owns(wall, Vector3::zeros(), 0.5));
    }

    #[test]
    fn ownership_expires_once_the_pair_drifts() {
        let mut ownership = NarrowphaseOwnership::new();
        let key = ContactPairKey::against_static(collider(1));
        ownership.insert(key, Vector3::zeros());

        assert!(ownership.owns(key, Vector3::new(0.4, 0.0, 0.0), 0.5));
        assert!(!ownership.owns(key, Vector3::new(0.6, 0.0, 0.0), 0.5));
    }
}
