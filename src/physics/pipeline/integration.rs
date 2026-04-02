//! Velocity and position integration for rigid bodies.

use rustc_hash::FxHashSet;

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

/// Integrate forces into velocities for all bodies.
pub fn integrate_forces(
    bodies: &mut Arena<RigidBody>,
    dt: f32,
    gravity: Vector3<f32>,
    sleeping: Option<&FxHashSet<RigidBodyHandle>>,
) {
    for (idx, body) in bodies.iter_mut() {
        if let Some(sleeping) = sleeping {
            let handle = RigidBodyHandle(idx);
            if sleeping.contains(&handle) {
                continue;
            }
        }
        body.integrate_forces(dt, gravity);
    }
}

/// Integrate velocities into positions for all bodies.
pub fn integrate_bodies(
    bodies: &mut Arena<RigidBody>,
    dt: f32,
    sleeping: Option<&FxHashSet<RigidBodyHandle>>,
) {
    for (idx, body) in bodies.iter_mut() {
        if let Some(sleeping) = sleeping {
            let handle = RigidBodyHandle(idx);
            if sleeping.contains(&handle) {
                continue;
            }
        }
        body.integrate_velocities(dt);
    }
}
