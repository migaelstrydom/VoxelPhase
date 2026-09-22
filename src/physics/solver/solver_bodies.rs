//! The velocities the solver works on, packed densely for the velocity phase.
//!
//! A `RigidBody` is a large struct in a generational arena; reading or writing
//! a velocity through it costs a generation check and drags the whole body
//! through the cache. The velocity phase touches only velocities, thousands of
//! times per substep, so it works on a dense copy instead:
//!
//! ```text
//!   Arena<RigidBody> ──gather──▶ SolverBodies ──▶ warm start, joint rows,
//!                                  (slot per        contact rows × iterations
//!                                   body)        ──scatter──▶ Arena<RigidBody>
//! ```
//!
//! Gathering copies each body's velocities, inverse mass and world inverse
//! inertia exactly; the rows do the same arithmetic on the copies that they did
//! on the bodies; scattering writes the velocities back. The result is the
//! same, bit for bit.

use generational_arena::{Arena, Index};
use nalgebra::{Matrix3, Vector3};

use crate::physics::body::RigidBody;

/// Marks an arena slot with no solver body.
const NO_SLOT: u32 = u32::MAX;

/// One body as the velocity phase sees it.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SolverBody {
    pub linear_velocity: Vector3<f32>,
    pub angular_velocity: Vector3<f32>,
    /// The body's real inverse mass; zero for anything that cannot be pushed.
    pub inv_mass: f32,
    /// `RigidBody::world_inv_inertia()` at the time of gathering — zero for a
    /// massless body, as there.
    pub world_inv_inertia: Matrix3<f32>,
}

impl SolverBody {
    fn gather(body: &RigidBody) -> Self {
        Self {
            linear_velocity: body.linear_velocity(),
            angular_velocity: body.angular_velocity(),
            inv_mass: body.inv_mass(),
            world_inv_inertia: body.world_inv_inertia(),
        }
    }

    /// `RigidBody::apply_impulse_at_point`, with the lever already known.
    pub fn apply_impulse_at(&mut self, impulse: Vector3<f32>, lever: Vector3<f32>) {
        if self.inv_mass > 0.0 {
            self.linear_velocity += impulse * self.inv_mass;
            self.angular_velocity += self.world_inv_inertia * lever.cross(&impulse);
        }
    }

    /// `RigidBody::apply_impulse`: a linear impulse at the centre of mass.
    pub fn apply_linear_impulse(&mut self, impulse: Vector3<f32>) {
        if self.inv_mass > 0.0 {
            self.linear_velocity += impulse * self.inv_mass;
        }
    }

    /// `RigidBody::apply_angular_impulse`.
    pub fn apply_angular_impulse(&mut self, angular_impulse: Vector3<f32>) {
        if self.inv_mass > 0.0 {
            self.angular_velocity += self.world_inv_inertia * angular_impulse;
        }
    }
}

/// Dense solver bodies for one velocity phase, and which arena body each is.
#[derive(Debug, Default)]
pub(crate) struct SolverBodies {
    bodies: Vec<SolverBody>,
    /// Arena index of each solver body, in slot order, for the scatter.
    handles: Vec<Index>,
    /// Solver slot per arena slot number, `NO_SLOT` where there is none.
    slot_of: Vec<u32>,
}

impl SolverBodies {
    /// Forget every body. Keeps the buffers.
    pub fn clear(&mut self) {
        for handle in &self.handles {
            self.slot_of[handle.into_raw_parts().0] = NO_SLOT;
        }
        self.bodies.clear();
        self.handles.clear();
    }

    /// The solver slot for `index`, gathering the body on first use.
    ///
    /// `None` when the arena has no such body.
    pub fn gather(&mut self, arena: &Arena<RigidBody>, index: Index) -> Option<usize> {
        let (arena_slot, _) = index.into_raw_parts();
        if let Some(&slot) = self.slot_of.get(arena_slot) {
            if slot != NO_SLOT && self.handles[slot as usize] == index {
                return Some(slot as usize);
            }
        }

        let body = arena.get(index)?;
        if self.slot_of.len() <= arena_slot {
            self.slot_of.resize(arena_slot + 1, NO_SLOT);
        }
        let slot = self.bodies.len();
        self.slot_of[arena_slot] = slot as u32;
        self.bodies.push(SolverBody::gather(body));
        self.handles.push(index);
        Some(slot)
    }

    pub fn get(&self, slot: usize) -> &SolverBody {
        &self.bodies[slot]
    }

    pub fn get_mut(&mut self, slot: usize) -> &mut SolverBody {
        &mut self.bodies[slot]
    }

    /// Write every gathered body's velocities back to the arena.
    pub fn scatter(&self, arena: &mut Arena<RigidBody>) {
        for (solver_body, &index) in self.bodies.iter().zip(&self.handles) {
            if let Some(body) = arena.get_mut(index) {
                body.set_linear_velocity(solver_body.linear_velocity);
                body.set_angular_velocity(solver_body.angular_velocity);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::physics::body::RigidBodyDesc;

    #[test]
    fn a_body_is_gathered_once_and_scattered_back() {
        let mut arena = Arena::new();
        let index = arena.insert(RigidBody::new(
            RigidBodyDesc::dynamic().linear_velocity(Vector3::new(1.0, 0.0, 0.0)),
        ));
        let mut solver_bodies = SolverBodies::default();

        let slot = solver_bodies.gather(&arena, index).unwrap();
        assert_eq!(solver_bodies.gather(&arena, index), Some(slot));

        solver_bodies.get_mut(slot).linear_velocity.y = 2.0;
        solver_bodies.scatter(&mut arena);
        assert_eq!(arena[index].linear_velocity(), Vector3::new(1.0, 2.0, 0.0));
    }

    #[test]
    fn a_reused_arena_slot_is_not_mistaken_for_its_old_body() {
        let mut arena = Arena::new();
        let old = arena.insert(RigidBody::new(RigidBodyDesc::dynamic()));
        let mut solver_bodies = SolverBodies::default();
        solver_bodies.gather(&arena, old).unwrap();

        arena.remove(old);
        let new = arena.insert(RigidBody::new(RigidBodyDesc::dynamic()));
        assert_eq!(old.into_raw_parts().0, new.into_raw_parts().0);

        assert_eq!(solver_bodies.gather(&arena, old), Some(0));
        assert_eq!(solver_bodies.gather(&arena, new), Some(1));
    }

    #[test]
    fn clearing_forgets_every_body() {
        let mut arena = Arena::new();
        let index = arena.insert(RigidBody::new(RigidBodyDesc::dynamic()));
        let mut solver_bodies = SolverBodies::default();
        solver_bodies.gather(&arena, index).unwrap();

        solver_bodies.clear();
        arena.remove(index);
        assert_eq!(solver_bodies.gather(&arena, index), None);
    }
}
