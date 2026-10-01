//! Which bodies a narrowphase pass generates contacts for.

use rustc_hash::FxHashSet;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

/// Which bodies a narrowphase pass generates contacts for.
///
/// A pair is generated when at least one side starts it. A sleeping body
/// starts none, since it does not move; it is only touched. When a pass's
/// contacts wake bodies, a further pass generates theirs, started by them
/// alone: the bodies awake before it were covered by an earlier pass and take
/// no part, so no pair is generated twice.
#[derive(Clone, Copy, Default)]
pub struct ContactScope<'a> {
    /// Bodies asleep: touched, but they start no pair.
    sleeping: Option<&'a FxHashSet<RigidBodyHandle>>,
    /// When given, the only bodies that start pairs.
    woken: Option<&'a FxHashSet<RigidBodyHandle>>,
}

/// The part one body plays in a pass.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum ContactRole {
    /// Starts pairs: with static geometry and with any body in reach.
    Starts,
    /// Takes part only in pairs another body starts.
    Touched,
    /// Covered by an earlier pass.
    Skipped,
}

impl<'a> ContactScope<'a> {
    /// Every awake body starts pairs; `sleeping` bodies are only touched.
    pub fn awake(sleeping: Option<&'a FxHashSet<RigidBodyHandle>>) -> Self {
        Self {
            sleeping,
            woken: None,
        }
    }

    /// Only the `woken` bodies start pairs; those still `sleeping` and static
    /// bodies are touched, and every other body is skipped.
    pub fn woken(
        woken: &'a FxHashSet<RigidBodyHandle>,
        sleeping: &'a FxHashSet<RigidBodyHandle>,
    ) -> Self {
        Self {
            sleeping: Some(sleeping),
            woken: Some(woken),
        }
    }

    pub(super) fn role(&self, handle: RigidBodyHandle, body: &RigidBody) -> ContactRole {
        if body.is_static() || self.sleeping.is_some_and(|s| s.contains(&handle)) {
            ContactRole::Touched
        } else if self.woken.is_some_and(|w| !w.contains(&handle)) {
            ContactRole::Skipped
        } else {
            ContactRole::Starts
        }
    }
}
