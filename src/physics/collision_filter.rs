//! Which colliders meet: layers a collider belongs to, and layers it meets.
//!
//! ```text
//!   ColliderDesc::filter(CollisionFilter { membership, meets })
//!        │
//!        ├─▶ narrowphase pairs, CCD pairs, BodySweep::cast movers
//!        │      a pair is made only if each side meets the other's membership
//!        │
//!        └─▶ queries (raycasts, sphere casts) see a collider only if
//!               its membership is among the layers they look for
//! ```
//!
//! The engine names no layers but [`Layers::DEFAULT`]: what each bit means is
//! the game's business. Static geometry (terrain) belongs to no layer and is
//! met by every collider and seen by every query: the ground is always solid.

/// A set of collision layers, one per bit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Layers(pub u32);

impl Layers {
    /// No layer at all.
    pub const NONE: Self = Self(0);
    /// Every layer.
    pub const ALL: Self = Self(u32::MAX);
    /// The layer a collider belongs to unless it says otherwise.
    pub const DEFAULT: Self = Self(1);

    /// The single layer numbered `index`, 0 to 31. Layer 0 is
    /// [`Self::DEFAULT`].
    pub const fn layer(index: u32) -> Self {
        assert!(index < 32, "there are 32 collision layers");
        Self(1 << index)
    }

    /// Every layer in either set.
    pub const fn with(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Every layer in this set but those in `other`.
    pub const fn without(self, other: Self) -> Self {
        Self(self.0 & !other.0)
    }

    /// Whether the two sets share a layer.
    pub const fn intersects(self, other: Self) -> bool {
        self.0 & other.0 != 0
    }
}

/// What a collider is to other colliders and to queries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CollisionFilter {
    /// The layers this collider belongs to.
    pub membership: Layers,
    /// The layers whose colliders this one makes contact with.
    pub meets: Layers,
}

impl CollisionFilter {
    /// In the default layer, meeting everything: what a collider is unless
    /// told otherwise, so a world that never filters behaves as if there were
    /// no filtering.
    pub const DEFAULT: Self = Self {
        membership: Layers::DEFAULT,
        meets: Layers::ALL,
    };

    /// Whether a contact between colliders filtered `self` and `other` is
    /// made: only if each meets the other. One-sided interest is not enough,
    /// since a contact pushes both.
    pub const fn meets(&self, other: &Self) -> bool {
        self.meets.intersects(other.membership) && other.meets.intersects(self.membership)
    }

    /// Whether a query looking for `layers` sees this collider.
    pub const fn seen_by(&self, layers: Layers) -> bool {
        self.membership.intersects(layers)
    }
}

impl Default for CollisionFilter {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEBRIS: Layers = Layers::layer(3);
    const CHARACTER: Layers = Layers::layer(4);

    #[test]
    fn unfiltered_colliders_meet() {
        assert!(CollisionFilter::DEFAULT.meets(&CollisionFilter::DEFAULT));
    }

    /// Debris that ignores characters makes no contact with one, even though
    /// the character would meet it.
    #[test]
    fn a_contact_needs_both_sides_to_meet() {
        let debris = CollisionFilter {
            membership: DEBRIS,
            meets: Layers::ALL.without(CHARACTER),
        };
        let character = CollisionFilter {
            membership: CHARACTER,
            meets: Layers::ALL,
        };
        assert!(!debris.meets(&character));
        assert!(!character.meets(&debris));
        assert!(debris.meets(&CollisionFilter::DEFAULT));
    }

    #[test]
    fn a_query_sees_only_the_layers_it_looks_for() {
        let debris = CollisionFilter {
            membership: DEBRIS,
            meets: Layers::ALL,
        };
        assert!(debris.seen_by(Layers::ALL));
        assert!(!debris.seen_by(Layers::ALL.without(DEBRIS)));
        assert!(CollisionFilter::DEFAULT.seen_by(Layers::DEFAULT.with(DEBRIS)));
    }

    /// A world's filtering seen end to end: contacts, CCD, mover sweeps and
    /// queries all skip a collider their filter does not meet.
    mod in_a_world {
        use nalgebra::{Point3, Vector3};

        use super::*;
        use crate::debug::DebugLines;
        use crate::physics::bench_harness::geometry::EmptyGeometry;
        use crate::physics::{
            BodySweep, CastObstacles, ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc,
            RigidBodyHandle, SequentialStepper, SphereCast, Stepper,
        };

        /// Colliders in the debris layer that meet nothing in it.
        const LOOSE_DEBRIS: CollisionFilter = CollisionFilter {
            membership: DEBRIS,
            meets: Layers::ALL.without(DEBRIS),
        };

        fn slab(world: &mut PhysicsWorld, filter: CollisionFilter) -> RigidBodyHandle {
            let body = world.create_body(RigidBodyDesc::static_body().position(Point3::origin()));
            world.attach_collider(
                body,
                ColliderDesc::box_shape(Vector3::new(2.0, 0.05, 2.0)).filter(filter),
            );
            body
        }

        fn ball(
            world: &mut PhysicsWorld,
            at: Point3<f32>,
            velocity: Vector3<f32>,
            filter: CollisionFilter,
        ) -> RigidBodyHandle {
            let body = world.create_body(
                RigidBodyDesc::dynamic()
                    .position(at)
                    .linear_velocity(velocity),
            );
            world.attach_collider(body, ColliderDesc::sphere(0.1).filter(filter));
            world.wake_body(body);
            body
        }

        /// Where `body` is after `seconds` of 60 Hz frames.
        fn height_after(world: &mut PhysicsWorld, body: RigidBodyHandle, seconds: f32) -> f32 {
            let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
            for _ in 0..(seconds * 60.0) as usize {
                stepper.step(
                    world,
                    1.0 / 60.0,
                    &EmptyGeometry,
                    &[],
                    &[],
                    &mut DebugLines::default(),
                );
            }
            world.body(body).unwrap().position().y
        }

        /// A ball lands on a slab it meets and falls through one it does not,
        /// slowly (the narrowphase) and fast (CCD).
        #[test]
        fn a_ball_passes_through_what_it_does_not_meet() {
            for speed in [0.0, 60.0] {
                let mut world = PhysicsWorld::new(PhysicsConfig::default());
                slab(&mut world, CollisionFilter::DEFAULT);
                let start = Point3::new(0.0, 1.0, 0.0);
                let fall = Vector3::new(0.0, -speed, 0.0);
                let lands = ball(&mut world, start, fall, CollisionFilter::DEFAULT);
                assert!(height_after(&mut world, lands, 1.0) > 0.0, "at {speed} m/s");

                let mut world = PhysicsWorld::new(PhysicsConfig::default());
                slab(&mut world, LOOSE_DEBRIS);
                let through = ball(&mut world, start, fall, LOOSE_DEBRIS);
                assert!(
                    height_after(&mut world, through, 1.0) < -1.0,
                    "at {speed} m/s"
                );
            }
        }

        /// A ray and a sphere cast see a collider only if they look for its
        /// layer.
        #[test]
        fn queries_see_only_the_layers_they_look_for() {
            let mut world = PhysicsWorld::new(PhysicsConfig::default());
            slab(&mut world, LOOSE_DEBRIS);
            let above = Point3::new(0.0, 1.0, 0.0);
            let down = -Vector3::y();
            let ray = |layers| world.raycast_looking_for(above, down, 2.0, &[], layers);
            assert!(ray(Layers::ALL).is_some());
            assert!(ray(Layers::ALL.without(DEBRIS)).is_none());

            let cast = |layers| {
                world.cast_sphere(
                    &EmptyGeometry,
                    &SphereCast {
                        origin: above,
                        direction: down,
                        length: 2.0,
                        radius: 0.1,
                        exclude: &[],
                        solid_bodies: false,
                        obstacles: CastObstacles::Everything,
                        layers,
                    },
                )
            };
            assert!(cast(DEBRIS).is_some());
            assert!(cast(Layers::DEFAULT).is_none());
        }

        /// A body swept as a mover passes through a collider it does not
        /// meet, even when the sweep looks for every layer.
        #[test]
        fn a_mover_sweeps_through_what_it_does_not_meet() {
            let mut world = PhysicsWorld::new(PhysicsConfig::default());
            slab(&mut world, LOOSE_DEBRIS);
            let above = Point3::new(0.0, 1.0, 0.0);
            let meets = ball(
                &mut world,
                above,
                Vector3::zeros(),
                CollisionFilter::DEFAULT,
            );
            let passes = ball(
                &mut world,
                above + Vector3::x(),
                Vector3::zeros(),
                LOOSE_DEBRIS,
            );
            let drop = Vector3::new(0.0, -2.0, 0.0);
            assert!(BodySweep::new(&world).cast(&[meets], drop).is_some());
            assert!(BodySweep::new(&world).cast(&[passes], drop).is_none());
        }
    }
}
