//! Pairs of bodies that pass through each other: no contacts between them,
//! in the narrowphase or in CCD. For a pair something else holds in
//! relation, such as a climber and the light crate it climbs, whose contacts
//! would only shove one against the other.

use rustc_hash::FxHashSet;

use crate::physics::RigidBodyHandle;

/// The pairs of bodies whose contacts are switched off.
#[derive(Debug, Default, Clone)]
pub struct ContactExemptions {
    pairs: FxHashSet<(RigidBodyHandle, RigidBodyHandle)>,
}

impl ContactExemptions {
    /// Switch off contacts between `a` and `b`.
    pub fn insert(&mut self, a: RigidBodyHandle, b: RigidBodyHandle) {
        self.pairs.insert(key(a, b));
    }

    /// Switch them back on.
    pub fn remove(&mut self, a: RigidBodyHandle, b: RigidBodyHandle) {
        self.pairs.remove(&key(a, b));
    }

    /// Whether contacts between `a` and `b` are off.
    pub fn exempts(&self, a: RigidBodyHandle, b: RigidBodyHandle) -> bool {
        !self.pairs.is_empty() && self.pairs.contains(&key(a, b))
    }

    /// Forget every pair `body` is in: it is gone.
    pub fn forget(&mut self, body: RigidBodyHandle) {
        self.pairs.retain(|&(a, b)| a != body && b != body);
    }
}

/// The pair in a fixed order, so either way round finds it.
fn key(a: RigidBodyHandle, b: RigidBodyHandle) -> (RigidBodyHandle, RigidBodyHandle) {
    if a.raw_parts() <= b.raw_parts() {
        (a, b)
    } else {
        (b, a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::{PhysicsConfig, PhysicsWorld, RigidBodyDesc};

    #[test]
    fn a_pair_is_found_either_way_round_and_forgotten_with_its_body() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let (a, b, c) = (
            world.create_body(RigidBodyDesc::dynamic()),
            world.create_body(RigidBodyDesc::dynamic()),
            world.create_body(RigidBodyDesc::dynamic()),
        );
        let mut exemptions = ContactExemptions::default();
        exemptions.insert(b, a);
        assert!(exemptions.exempts(a, b) && exemptions.exempts(b, a));
        assert!(!exemptions.exempts(a, c));
        exemptions.forget(a);
        assert!(!exemptions.exempts(a, b));
    }
}
