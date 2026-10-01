//! Per-frame record of the work contacts did on each body, and who did it.
//!
//! [`ImpactLedger`](super::ImpactLedger) says how hard a body was hit. This
//! says how much energy each contact gave it, and from which body — which is
//! what an energy audit needs to tell energy a body was *given* from energy
//! the solver made up.
//!
//! ```text
//!   substep:  snapshot (v₀, ω₀) ──► solve ──► record_solved
//!                                               │  per contact, per side:
//!                                               │  J · (v̄ + ω̄ × r)
//!                                               ▼
//!                               work[(body, by)] += …   summed over the frame
//! ```
//!
//! An impulse `J` that takes a body from `v₀` to `v₁` does `J · (v₀ + v₁)/2`
//! of work on it, and over every contact on a body those terms add up to
//! exactly the kinetic energy the solve gave it, spin included. Each
//! contact's share is attributed to the body on its other side. `J` is the
//! impulse the body actually received: shock propagation scales a contact's
//! impulse differently on its two sides, which is momentum the contact made
//! up, and the ledger must see it rather than hide it.
//!
//! Off by default — it costs a lookup per contact per substep, and nothing in
//! the game reads it. Audits switch it on.

use nalgebra::Vector3;
use rustc_hash::FxHashMap;

use generational_arena::Arena;

use super::body::RigidBody;
use super::handle::RigidBodyHandle;
use super::pipeline::pair::SolverManifold;
use super::solver::ManifoldConditions;

/// Work done on bodies by contacts over one frame, keyed by the body worked
/// on and the body on the other side of the contact (`None` for static
/// geometry).
#[derive(Debug, Default)]
pub struct ContactWorkLedger {
    /// Whether the world records anything into this ledger.
    enabled: bool,
    /// Each body's velocities before the current substep's solve.
    before: FxHashMap<RigidBodyHandle, (Vector3<f32>, Vector3<f32>)>,
    /// Work done this frame, in joules.
    work: FxHashMap<(RigidBodyHandle, Option<RigidBodyHandle>), f32>,
}

impl ContactWorkLedger {
    /// Whether contacts' work is being recorded.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub(crate) fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        self.clear();
    }

    /// Drop the last frame's record. Called once per frame, before any solve.
    pub(crate) fn clear(&mut self) {
        self.before.clear();
        self.work.clear();
    }

    /// Note the velocities of every body in `manifolds`, before the solve.
    pub(crate) fn snapshot(&mut self, bodies: &Arena<RigidBody>, manifolds: &[SolverManifold]) {
        if !self.enabled {
            return;
        }
        self.before.clear();
        for manifold in manifolds {
            let header = &manifold.header;
            for handle in header.body_a.into_iter().chain([header.body_b]) {
                if let Some(body) = bodies.get(handle.0) {
                    self.before
                        .insert(handle, (body.linear_velocity(), body.angular_velocity()));
                }
            }
        }
    }

    /// Add the work each solved contact did on the bodies either side of it.
    pub(crate) fn record_solved(
        &mut self,
        bodies: &Arena<RigidBody>,
        manifolds: &[SolverManifold],
        conditions: &ManifoldConditions,
    ) {
        if !self.enabled {
            return;
        }
        for (index, manifold) in manifolds.iter().enumerate() {
            let header = &manifold.header;
            let (scale_a, scale_b) = conditions.shock_scales_for(index);
            for contact in &manifold.contacts {
                let linear = contact.normal * contact.accumulated_normal_impulse
                    + contact.accumulated_friction_impulse_ws;
                let angular = contact.normal * contact.accumulated_torsional_impulse;
                let mut credit = |body: RigidBodyHandle, by, sign: f32, scale: f32| {
                    let Some(work) =
                        self.work_on(bodies, body, contact.point, sign * scale, &linear, &angular)
                    else {
                        return;
                    };
                    *self.work.entry((body, by)).or_default() += work;
                };
                credit(header.body_b, header.body_a, 1.0, scale_b);
                if let Some(body_a) = header.body_a {
                    credit(body_a, Some(header.body_b), -1.0, scale_a);
                }
            }
        }
    }

    /// Work an impulse `factor · (linear, angular)` at `point` did on `body`
    /// over the substep: the impulse against the body's mean velocity there.
    fn work_on(
        &self,
        bodies: &Arena<RigidBody>,
        handle: RigidBodyHandle,
        point: nalgebra::Point3<f32>,
        factor: f32,
        linear: &Vector3<f32>,
        angular: &Vector3<f32>,
    ) -> Option<f32> {
        let body = bodies.get(handle.0)?;
        if !body.is_dynamic() {
            return None;
        }
        let (v0, w0) = self.before.get(&handle)?;
        let mean_linear = (v0 + body.linear_velocity()) * 0.5;
        let mean_angular = (w0 + body.angular_velocity()) * 0.5;
        let arm = point - body.position();
        let impulse = linear * factor;
        let angular_impulse = arm.cross(&impulse) + angular * factor;
        Some(impulse.dot(&mean_linear) + angular_impulse.dot(&mean_angular))
    }

    /// Work done on `body` this frame by contacts with each other body, or
    /// with static geometry (`None`), in joules.
    pub fn on(
        &self,
        body: RigidBodyHandle,
    ) -> impl Iterator<Item = (Option<RigidBodyHandle>, f32)> + '_ {
        self.work
            .iter()
            .filter(move |((on, _), _)| *on == body)
            .map(|((_, by), work)| (*by, *work))
    }

    /// Every record this frame: the body worked on, the body that did it
    /// (`None` for static geometry), and the work, in joules.
    pub fn iter(
        &self,
    ) -> impl Iterator<Item = (RigidBodyHandle, Option<RigidBodyHandle>, f32)> + '_ {
        self.work.iter().map(|((on, by), work)| (*on, *by, *work))
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{
        ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
        SequentialStepper, Stepper,
    };

    fn kinetic(world: &PhysicsWorld, handle: RigidBodyHandle) -> f32 {
        let body = world.body(handle).unwrap();
        0.5 * body.mass() * body.linear_velocity().norm_squared()
    }

    /// A box slides into another at rest, in empty space. Everything the
    /// struck box ends up with came through the contact, so the work the
    /// ledger credits to the striker is its kinetic energy.
    #[test]
    fn the_work_a_contact_does_is_the_energy_it_passes_on() {
        let mut config = PhysicsConfig::default();
        config.gravity = Vector3::zeros();
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);
        world.record_contact_work(true);
        let boxed = |world: &mut PhysicsWorld, x: f32, speed: f32| {
            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(Point3::new(x, 0.0, 0.0))
                    .linear_velocity(Vector3::new(speed, 0.0, 0.0)),
            );
            world.attach_collider(
                body,
                ColliderDesc::box_shape(Vector3::new(0.25, 0.25, 0.25))
                    .restitution(0.5)
                    .friction(0.5),
            );
            body
        };
        let striker = boxed(&mut world, -1.0, 2.0);
        let struck = boxed(&mut world, 0.0, 0.0);

        let mut stepper = SequentialStepper::new(1.0 / 240.0, 4);
        let mut debug = DebugLines::default();
        let mut given = 0.0;
        for _ in 0..60 {
            stepper.step(&mut world, 1.0 / 60.0, &EmptyGeometry, &[], &[], &mut debug);
            given += world
                .contact_work()
                .on(struck)
                .filter(|(by, _)| *by == Some(striker))
                .map(|(_, work)| work)
                .sum::<f32>();
        }

        let gained = kinetic(&world, struck);
        assert!(gained > 0.1, "the boxes never met: {gained} J");
        assert!(
            (given - gained).abs() < 0.01 * gained,
            "the ledger says {given} J was passed on; the struck box has {gained} J"
        );
    }
}
