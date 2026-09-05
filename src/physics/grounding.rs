//! Grounding: the projection of the Support Set a character reads.
//!
//! A body is grounded when something holds it up, and the axis it is held
//! along is the mean of the normals doing the holding. Both answers are
//! produced once, by `SupportResolver`, from the contacts the step already
//! produced; this module adds only the carry-over that keeps a sleeping body
//! standing on the floor it fell asleep on.

use nalgebra::{Point3, Vector3};
use rustc_hash::{FxHashMap, FxHashSet};

use crate::physics::drive::SupportSets;
use crate::physics::RigidBodyHandle;

/// What is holding one body up.
#[derive(Clone, Copy, Debug)]
pub struct Support {
    /// Mean of the normals doing the holding: the body's local up.
    pub normal: Vector3<f32>,
    /// Velocity of the surface at those contacts. Zero for static geometry,
    /// the platform's own motion for a body riding one.
    ///
    /// It travels with the normal because everything standing on something
    /// needs both: the axis to stand along, and the frame to stand still in.
    pub surface_velocity: Vector3<f32>,
}

impl Support {
    pub fn new(normal: Vector3<f32>, surface_velocity: Vector3<f32>) -> Self {
        Self {
            normal,
            surface_velocity,
        }
    }
}

/// Every grounded body of one step, with what holds each up.
pub type GroundedBodies = FxHashMap<RigidBodyHandle, Support>;

/// Projects Support Sets to the grounded bodies and their support normals,
/// across steps.
#[derive(Default)]
pub struct GroundingDetector {
    /// The grounded bodies from the previous step, with what held them up.
    ///
    /// Sleeping bodies generate no contacts, but they haven't moved either —
    /// without the carry-over, a body falling asleep while resting on the
    /// floor would read as airborne. The support is carried with it: the floor
    /// it fell asleep on has not tilted, and nothing that carries a sleeping
    /// body is going anywhere either.
    last_grounded: GroundedBodies,
}

impl GroundingDetector {
    pub fn new() -> Self {
        Self::default()
    }

    /// Bodies that are grounded this step, each with what holds it up.
    ///
    /// `velocity_at` answers how fast a supporting body's material is moving
    /// at a contact point; the detector never reaches into the body arena
    /// itself, so a caller with a different notion of "how fast is that
    /// surface" can supply it.
    pub fn grounded_bodies(
        &mut self,
        supports: &SupportSets,
        sleeping: &FxHashSet<RigidBodyHandle>,
        velocity_at: impl Fn(RigidBodyHandle, Point3<f32>) -> Vector3<f32>,
    ) -> GroundedBodies {
        let mut grounded: GroundedBodies = supports
            .supported_bodies()
            .filter_map(|handle| {
                supports.get(handle).map(|set| {
                    (
                        handle,
                        Support::new(set.mean_normal(), set.surface_velocity(&velocity_at)),
                    )
                })
            })
            .collect();
        for handle in sleeping {
            if let Some(support) = self.last_grounded.get(handle) {
                grounded.entry(*handle).or_insert(*support);
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
    fn step(world: &mut PhysicsWorld, floor: &FlatQuadGeometry) -> GroundedBodies {
        let mut debug = DebugLines::default();
        world.update_contacts(DT, 1, floor, &[], &mut debug);
        world.substep(DT, floor, &[]);
        world.grounded_bodies()
    }

    fn run(world: &mut PhysicsWorld, frames: u32) -> GroundedBodies {
        let floor = FlatQuadGeometry::new(20.0);
        let mut grounded = GroundedBodies::default();
        for _ in 0..frames {
            grounded = step(world, &floor);
        }
        grounded
    }

    #[test]
    fn a_body_resting_on_the_floor_is_grounded() {
        let (mut world, body) = sphere_over_a_floor(PhysicsConfig::default(), 1.0);
        assert!(run(&mut world, 120).contains_key(&body));
    }

    #[test]
    fn a_body_in_free_air_is_not_grounded() {
        let (mut world, body) = sphere_over_a_floor(PhysicsConfig::default(), 8.0);
        assert!(!run(&mut world, 10).contains_key(&body));
    }

    #[test]
    fn a_body_that_falls_asleep_on_the_floor_stays_grounded() {
        let (mut world, body) = sphere_over_a_floor(PhysicsConfig::default(), 1.0);
        assert!(run(&mut world, 120).contains_key(&body));
        assert!(world.is_sleeping(body), "the sphere should have settled");

        let floor = FlatQuadGeometry::new(20.0);
        for _ in 0..120 {
            assert!(
                step(&mut world, &floor).contains_key(&body),
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
        assert!(!run(&mut world, 60).contains_key(&body));
    }
}
