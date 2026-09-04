//! Grounding: the boolean projection of the Support Set.
//!
//! A body is grounded when something holds it up. That question is answered
//! once, by `SupportResolver`, from the contacts the step already produced;
//! this module adds only the carry-over that keeps a sleeping body standing on
//! the floor it fell asleep on.

use rustc_hash::FxHashSet;

use crate::physics::drive::SupportSets;
use crate::physics::RigidBodyHandle;

/// Projects Support Sets to a grounded set, across steps.
#[derive(Default)]
pub struct GroundingDetector {
    /// The grounded set from the previous step.
    ///
    /// Sleeping bodies generate no contacts, but they haven't moved either —
    /// without the carry-over, a body falling asleep while resting on the
    /// floor would read as airborne.
    last_grounded: FxHashSet<RigidBodyHandle>,
}

impl GroundingDetector {
    pub fn new() -> Self {
        Self::default()
    }

    /// Bodies that are grounded this step.
    pub fn grounded_bodies(
        &mut self,
        supports: &SupportSets,
        sleeping: &FxHashSet<RigidBodyHandle>,
    ) -> FxHashSet<RigidBodyHandle> {
        let mut grounded: FxHashSet<RigidBodyHandle> = supports.supported_bodies().collect();
        for handle in sleeping {
            if self.last_grounded.contains(handle) {
                grounded.insert(*handle);
            }
        }
        self.last_grounded = grounded.clone();
        grounded
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::{ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc};

    const DT: f32 = 1.0 / 60.0;

    /// A world with one sphere above the quad at y=0, awake and falling.
    fn sphere_over_a_floor(config: PhysicsConfig, height: f32) -> (PhysicsWorld, RigidBodyHandle) {
        let mut world = PhysicsWorld::new(config);
        let body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, height, 0.0)));
        let _ = world.attach_collider(body, ColliderDesc::sphere(0.5));
        world.wake_body(body);
        (world, body)
    }

    /// One frame: contacts once, then a single substep, as the game loop does,
    /// ending with the grounding read the game also performs every frame.
    fn step(world: &mut PhysicsWorld, floor: &FlatQuadGeometry) -> FxHashSet<RigidBodyHandle> {
        let mut debug = DebugLines::default();
        world.update_contacts(DT, 1, floor, &[], &mut debug);
        world.substep(DT, floor, &[]);
        world.grounded_handles()
    }

    fn run(world: &mut PhysicsWorld, frames: u32) -> FxHashSet<RigidBodyHandle> {
        let floor = FlatQuadGeometry::new(20.0);
        let mut grounded = FxHashSet::default();
        for _ in 0..frames {
            grounded = step(world, &floor);
        }
        grounded
    }

    #[test]
    fn a_body_resting_on_the_floor_is_grounded() {
        let (mut world, body) = sphere_over_a_floor(PhysicsConfig::default(), 1.0);
        assert!(run(&mut world, 120).contains(&body));
    }

    #[test]
    fn a_body_in_free_air_is_not_grounded() {
        let (mut world, body) = sphere_over_a_floor(PhysicsConfig::default(), 8.0);
        assert!(!run(&mut world, 10).contains(&body));
    }

    #[test]
    fn a_body_that_falls_asleep_on_the_floor_stays_grounded() {
        let (mut world, body) = sphere_over_a_floor(PhysicsConfig::default(), 1.0);
        assert!(run(&mut world, 120).contains(&body));
        assert!(world.is_sleeping(body), "the sphere should have settled");

        let floor = FlatQuadGeometry::new(20.0);
        for _ in 0..120 {
            assert!(
                step(&mut world, &floor).contains(&body),
                "the floor did not move, so neither should the answer"
            );
        }
    }

    #[test]
    fn the_floor_is_the_surface_gravity_pulls_toward() {
        // The same quad, with gravity turned on its side: what was a floor is
        // now a wall, and nothing holds the sphere up.
        let mut config = PhysicsConfig::default();
        config.gravity = Vector3::new(-9.81, 0.0, 0.0);
        let (mut world, body) = sphere_over_a_floor(config, 0.5);
        assert!(!run(&mut world, 60).contains(&body));
    }
}
