//! Per-frame record of how hard each body was hit.
//!
//! [`ContactEvent`](super::ContactEvent) reports that a contact *exists*, but is
//! emitted before the solver runs, so it cannot say how much momentum the
//! contact actually transferred. The ledger fills that gap: after each solve it
//! folds the accumulated normal impulses into a per-body total, giving callers a
//! single scalar for "how hard did this body just get hit".
//!
//! The total is a frame quantity, summed across every substep, so it stays
//! comparable regardless of the substep count. A body resting under gravity
//! accumulates roughly `mass * gravity * frame_dt` per frame, which is small
//! next to any real collision.

use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;

use super::handle::RigidBodyHandle;
use super::pipeline::pair::SolverManifold;

/// Momentum delivered to a single body over one frame.
#[derive(Debug, Clone, Copy)]
pub struct BodyImpact {
    /// Sum of normal impulses over every contact and substep this frame (N·s).
    pub total_impulse: f32,
    /// Largest single normal impulse contributing to `total_impulse` (N·s).
    pub peak_impulse: f32,
    /// World-space location of the contact that produced `peak_impulse`.
    pub point: Point3<f32>,
    /// Solver-facing contact normal at `point`, oriented towards this body.
    pub normal: Vector3<f32>,
}

/// Accumulates [`BodyImpact`] records for one frame.
///
/// Cleared at the start of each frame by `PhysicsWorld::update_contacts`, then
/// fed by every solve pass — the main substep solver and the CCD solver alike.
#[derive(Debug, Default)]
pub struct ImpactLedger {
    impacts: FxHashMap<RigidBodyHandle, BodyImpact>,
}

impl ImpactLedger {
    /// Drop all records. Called once per frame, before any solving.
    pub fn clear(&mut self) {
        self.impacts.clear();
    }

    /// Fold the impulses of already-solved manifolds into the ledger.
    ///
    /// Must be called *after* the solver has written
    /// `accumulated_normal_impulse`; calling it on unsolved manifolds records
    /// only the warm-start seed.
    pub fn record_solved(&mut self, manifolds: &[SolverManifold]) {
        for manifold in manifolds {
            let header = &manifold.header;
            for contact in &manifold.contacts {
                let impulse = contact.accumulated_normal_impulse;
                if impulse <= 0.0 {
                    continue;
                }
                self.add(header.body_b, impulse, contact.point, contact.normal);
                if let Some(body_a) = header.body_a {
                    self.add(body_a, impulse, contact.point, -contact.normal);
                }
            }
        }
    }

    /// Impulse record for a body, or `None` if it took no impulse this frame.
    pub fn get(&self, handle: RigidBodyHandle) -> Option<&BodyImpact> {
        self.impacts.get(&handle)
    }

    /// Iterate every body that took an impulse this frame.
    pub fn iter(&self) -> impl Iterator<Item = (RigidBodyHandle, &BodyImpact)> {
        self.impacts
            .iter()
            .map(|(handle, impact)| (*handle, impact))
    }

    fn add(
        &mut self,
        handle: RigidBodyHandle,
        impulse: f32,
        point: Point3<f32>,
        normal: Vector3<f32>,
    ) {
        match self.impacts.get_mut(&handle) {
            Some(existing) => {
                existing.total_impulse += impulse;
                if impulse > existing.peak_impulse {
                    existing.peak_impulse = impulse;
                    existing.point = point;
                    existing.normal = normal;
                }
            }
            None => {
                self.impacts.insert(
                    handle,
                    BodyImpact {
                        total_impulse: impulse,
                        peak_impulse: impulse,
                        point,
                        normal,
                    },
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::contact::FeatureId;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::pipeline::pair::{PairHeader, SolverContact};
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc};
    use generational_arena::Arena;

    fn handle(arena: &mut Arena<u8>) -> RigidBodyHandle {
        RigidBodyHandle(arena.insert(0))
    }

    fn contact(impulse: f32) -> SolverContact {
        SolverContact {
            point: Point3::new(1.0, 2.0, 3.0),
            normal: Vector3::y(),
            raw_normal: Vector3::y(),
            depth: 0.0,
            raw_depth: 0.0,
            feature_id: FeatureId::SINGLE,
            warm_normal_impulse: 0.0,
            warm_friction_impulse_ws: Vector3::zeros(),
            accumulated_normal_impulse: impulse,
            accumulated_friction_impulse_ws: Vector3::zeros(),
        }
    }

    fn manifold(
        body_a: Option<RigidBodyHandle>,
        body_b: RigidBodyHandle,
        impulses: &[f32],
    ) -> SolverManifold {
        SolverManifold {
            header: PairHeader {
                body_a,
                body_b,
                collider_a: None,
                collider_b: None,
                restitution: 0.0,
                friction: 0.0,
            },
            contacts: impulses.iter().map(|i| contact(*i)).collect(),
        }
    }

    #[test]
    fn impulses_sum_across_contacts_and_substeps() {
        let mut arena = Arena::new();
        let body = handle(&mut arena);
        let mut ledger = ImpactLedger::default();

        ledger.record_solved(&[manifold(None, body, &[2.0, 3.0])]);
        ledger.record_solved(&[manifold(None, body, &[4.0])]);

        let impact = ledger.get(body).expect("body took an impulse");
        assert_eq!(impact.total_impulse, 9.0);
        assert_eq!(impact.peak_impulse, 4.0);
    }

    #[test]
    fn a_dynamic_pair_records_against_both_bodies_with_opposed_normals() {
        let mut arena = Arena::new();
        let a = handle(&mut arena);
        let b = handle(&mut arena);
        let mut ledger = ImpactLedger::default();

        ledger.record_solved(&[manifold(Some(a), b, &[5.0])]);

        assert_eq!(ledger.get(a).unwrap().total_impulse, 5.0);
        assert_eq!(ledger.get(b).unwrap().total_impulse, 5.0);
        assert_eq!(ledger.get(a).unwrap().normal, -Vector3::y());
        assert_eq!(ledger.get(b).unwrap().normal, Vector3::y());
    }

    #[test]
    fn zero_impulse_contacts_are_not_recorded() {
        let mut arena = Arena::new();
        let body = handle(&mut arena);
        let mut ledger = ImpactLedger::default();

        ledger.record_solved(&[manifold(None, body, &[0.0])]);

        assert!(ledger.get(body).is_none());
    }

    /// The grenade detonation rule reads `total_impulse` against a threshold of
    /// `mass * speed`. That only works if a real impact and a body at rest sit
    /// on opposite sides of it by a wide margin, so measure both.
    #[test]
    fn a_hard_landing_and_a_resting_body_are_orders_of_magnitude_apart() {
        let geometry = FlatQuadGeometry::new(50.0);
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let mut stepper = SequentialStepper::new(1.0 / 60.0, 4);
        let mut debug = DebugLines::default();

        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, 3.0, 0.0))
                .linear_velocity(Vector3::new(0.0, -20.0, 0.0)),
        );
        world.attach_collider(
            body,
            ColliderDesc::sphere(0.2).density(2000.0).restitution(0.0),
        );
        let mass = world.body(body).unwrap().mass();

        let mut peak_frame_impulse = 0.0f32;
        // Once settled the body sleeps and stops producing impulses entirely,
        // so track the largest non-zero frame after the bounce has died down —
        // the settled-but-awake case is the one that could false-trigger.
        let mut resting_impulse = 0.0f32;
        for frame in 0..180 {
            stepper.step(&mut world, 1.0 / 60.0, &geometry, &[], &[], &mut debug);
            let total = world
                .impacts()
                .get(body)
                .map(|i| i.total_impulse)
                .unwrap_or(0.0);
            peak_frame_impulse = peak_frame_impulse.max(total);
            if frame >= 20 && total > 0.0 {
                resting_impulse = resting_impulse.max(total);
            }
        }

        // The impact must arrest a 20 m/s fall, so it clears the impulse needed
        // to arrest 6 m/s several times over (measured: ~1439 vs 402 N·s).
        let threshold = mass * 6.0;
        assert!(
            peak_frame_impulse > threshold * 2.0,
            "impact {peak_frame_impulse} did not clear threshold {threshold}"
        );
        // A body at rest only takes its own weight over one frame:
        // mass * gravity * frame_dt (measured: ~11 N·s).
        assert!(
            resting_impulse < threshold * 0.25,
            "resting {resting_impulse} too close to threshold {threshold}"
        );
    }
}
