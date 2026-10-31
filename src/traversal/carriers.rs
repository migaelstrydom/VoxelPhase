//! Carriers: bodies that move under their own drive and carry what stands on
//! them (moving platforms).
//!
//! The planner's ground holds still, so a carrier is something it cannot
//! plan on honestly. Until it can, the strip reads one as fixed ground where
//! it stands this frame, however fast it moves (not as loose footing in
//! motion, which no footfall may use, nor as a piece in flight, which the
//! casts pass through), and the plan stands aside while the player rides one:
//! the drive and the legs, which step in the support's frame, carry on
//! untracked.

use specs::{Join, ReadStorage};

use crate::components::RigidBodyComponent;
use crate::physics::{PhysicsWorld, RigidBodyHandle};
use crate::platform::MovingPlatform;

/// The bodies of every carrier in the world.
pub fn carriers(
    platforms: &ReadStorage<MovingPlatform>,
    bodies: &ReadStorage<RigidBodyComponent>,
) -> Vec<RigidBodyHandle> {
    (platforms, bodies).join().map(|(_, body)| body.0).collect()
}

/// What `body` stands on this step, as far as the plan is concerned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StandsOn {
    /// Held up by one of the carriers, alone or with other ground.
    Carrier,
    /// Held up by other ground only.
    Ground,
    /// Held up by nothing.
    Air,
}

impl StandsOn {
    /// What holds `body` up in `world`'s last step, among `carriers`.
    pub fn of(world: &PhysicsWorld, body: RigidBodyHandle, carriers: &[RigidBodyHandle]) -> Self {
        let supports = world.support_sets();
        let Some(set) = supports.get(body).filter(|set| !set.is_empty()) else {
            return StandsOn::Air;
        };
        let on_carrier = set.contacts().iter().any(|contact| {
            contact
                .partner
                .is_some_and(|partner| carriers.contains(&partner))
        });
        if on_carrier {
            StandsOn::Carrier
        } else {
            StandsOn::Ground
        }
    }
}

/// Whether the plan stands aside: from when the body stands on a carrier
/// until it stands on other ground, the air between included, as a hop on
/// a deck or a jump off one is made in the deck's frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Riding(bool);

impl Riding {
    /// Takes what the body stands on this step; whether the plan stands aside after it.
    pub fn update(&mut self, stands_on: StandsOn) -> bool {
        match stands_on {
            StandsOn::Carrier => self.0 = true,
            StandsOn::Ground => self.0 = false,
            StandsOn::Air => {}
        }
        self.0
    }

    /// Whether the plan stands aside.
    pub fn is_riding(&self) -> bool {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    use super::*;

    const FRAME: f32 = 1.0 / 60.0;
    const SUBSTEPS: u32 = 4;

    /// A ball resting on a deck, the deck on a static slab; the deck moving
    /// along +x at `deck_speed`, held up by the slab.
    fn ball_on_deck(deck_speed: f32) -> (PhysicsWorld, RigidBodyHandle, RigidBodyHandle) {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let slab =
            world.create_body(RigidBodyDesc::static_body().position(Point3::new(0.0, -0.5, 0.0)));
        world.attach_collider(slab, ColliderDesc::box_shape(Vector3::new(20.0, 0.5, 20.0)));
        let deck = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, 0.25, 0.0))
                .linear_velocity(Vector3::x() * deck_speed),
        );
        world.attach_collider(deck, ColliderDesc::box_shape(Vector3::new(2.0, 0.25, 2.0)));
        let ball = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, 0.8, 0.0))
                .linear_velocity(Vector3::x() * deck_speed),
        );
        world.attach_collider(ball, ColliderDesc::sphere(0.3));
        world.wake_body(deck);
        world.wake_body(ball);
        let dt = FRAME / SUBSTEPS as f32;
        for _ in 0..10 {
            world.update_contacts(
                dt,
                SUBSTEPS,
                &EmptyGeometry,
                &[],
                &mut DebugLines::default(),
            );
            for _ in 0..SUBSTEPS {
                world.substep(dt, &EmptyGeometry, &[]);
            }
        }
        (world, deck, ball)
    }

    #[test]
    fn a_body_on_a_carrier_stands_on_it_and_one_on_a_slab_on_ground() {
        let (world, deck, ball) = ball_on_deck(2.5);
        assert_eq!(StandsOn::of(&world, ball, &[deck]), StandsOn::Carrier);
        assert_eq!(
            StandsOn::of(&world, ball, &[]),
            StandsOn::Ground,
            "a deck no carrier"
        );
        assert_eq!(
            StandsOn::of(&world, deck, &[deck]),
            StandsOn::Ground,
            "on the slab"
        );
    }

    #[test]
    fn riding_lasts_through_the_air_until_other_ground() {
        let mut riding = Riding::default();
        assert!(!riding.update(StandsOn::Air));
        assert!(riding.update(StandsOn::Carrier));
        assert!(riding.update(StandsOn::Air), "a hop off the deck");
        assert!(!riding.update(StandsOn::Ground), "landed beside it");
    }
}
