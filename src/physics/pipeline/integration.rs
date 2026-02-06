//! Velocity and position integration for rigid bodies.

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;

/// Integrate forces into velocities for all bodies.
pub fn integrate_forces(bodies: &mut Arena<RigidBody>, dt: f32, gravity: Vector3<f32>) {
    for (_, body) in bodies.iter_mut() {
        body.integrate_forces(dt, gravity);
    }
}

/// Integrate velocities into positions for all bodies.
pub fn integrate_bodies(bodies: &mut Arena<RigidBody>, dt: f32) {
    for (_, body) in bodies.iter_mut() {
        body.integrate_velocities(dt);
    }
}
