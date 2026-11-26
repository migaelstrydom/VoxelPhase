use nalgebra::{Point3, Vector3};

use crate::physics::{CastObstacles, PhysicsWorld, RigidBodyHandle, SphereCast, StaticGeometry};

/// How far above the ground under it a rush is looked along, m, at the
/// least: the striker's body does not meet what it stands on.
const OVER_GROUND: f32 = 0.1;

/// The least gap a striker rushes across, as a share of its run-up: across
/// less, it meets what it strikes at under seven tenths of its speed, and
/// it backs off instead.
const NEAREST_SHARE_OF_RUN_UP: f32 = 0.5;

/// Seconds a striker drives on once it has crossed the gap: into what it
/// meets.
const FOLLOW_THROUGH: f32 = 0.1;

/// A blow struck with the whole body. Its wind-up over, the striker drives
/// itself along the way it faces toward `speed`, as hard as its legs or
/// wings drive it, across the gap to what it strikes: what it gives that
/// is the two bodies colliding, judged as any other hit. Its weight and its
/// speed decide how hard, and it is pushed back by what it gives.
///
/// ```text
///   side view, the striker facing right
///      ┌─┐
///      │ │
///      │✚│ ◯ ────── carry ──────▶ ◯   its body, `radius` wide, swept at
///      │ │                             the height of what it strikes
///      └─┘
///          ├─ run-up ─┤                up to speed; it rushes nothing
///                                      nearer than half of it
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rush {
    /// The speed it drives its body toward, m/s.
    pub speed: f32,
    /// How hard it drives toward it, m/s²: about what its legs or its wings
    /// give it. It sets how far back it rushes from, and for how long.
    pub grip: f32,
    /// How far ahead of where it stands the rush carries its body, m: what
    /// it can meet.
    pub carry: f32,
    /// Its body's half-width, m.
    pub radius: f32,
    /// How far its body reaches above and below its centre, m.
    pub half_height: f32,
}

impl Rush {
    /// The gap its body needs to get up to speed from a standstill, m.
    pub fn run_up(&self) -> f32 {
        self.speed.powi(2) / (2.0 * self.grip)
    }

    /// The least gap it rushes across, m.
    pub fn nearest(&self) -> f32 {
        self.run_up() * NEAREST_SHARE_OF_RUN_UP
    }

    /// Seconds it drives to cross `gap` from a standstill, and on into
    /// what it meets there.
    pub fn seconds_across(&self, gap: f32) -> f32 {
        let run_up = self.run_up();
        let getting_up = (2.0 * gap.clamp(0.0, run_up) / self.grip).sqrt();
        let at_speed = (gap - run_up).max(0.0) / self.speed;
        getting_up + at_speed + FOLLOW_THROUGH
    }

    /// The first body a rush thrown now would meet, and the gap it would
    /// cross to it: the striker's body, centred at `centre` and facing
    /// `facing` (level, unit length), swept `carry` along it at the height
    /// of its own nearest `height` (what it strikes at), though not so low
    /// that it meets its own ground. `None` if it would meet nothing, or
    /// the scenery or a static body first.
    pub fn meets(
        &self,
        world: &PhysicsWorld,
        scenery: &dyn StaticGeometry,
        striker: RigidBodyHandle,
        centre: Point3<f32>,
        facing: Vector3<f32>,
        height: f32,
    ) -> Option<(RigidBodyHandle, f32)> {
        let reach = (self.half_height - self.radius).max(0.0);
        let lowest = (centre.y - reach + OVER_GROUND).min(centre.y);
        let at = height.clamp(lowest, (centre.y + reach).max(lowest));
        let touch = world.cast_sphere(
            scenery,
            &SphereCast {
                origin: Point3::new(centre.x, at, centre.z),
                direction: facing,
                length: self.carry,
                radius: self.radius,
                exclude: &[striker],
                solid_bodies: true,
                obstacles: CastObstacles::Everything,
                filter: world.filter_of(striker),
            },
        )?;
        let body = touch
            .body
            .filter(|&body| world.body(body).is_some_and(|body| body.is_dynamic()))?;
        Some((body, touch.distance))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    /// A body 1.2 m tall and 0.6 m wide, rushing 0.8 m to 4 m/s, which it
    /// is up to after 0.4 m.
    const RUSH: Rush = Rush {
        speed: 4.0,
        grip: 20.0,
        carry: 0.8,
        radius: 0.3,
        half_height: 0.6,
    };

    /// A rush across its run-up gets up to speed on the way, across more
    /// goes on at speed, and either drives on a little into what it meets.
    #[test]
    fn a_rush_lasts_as_long_as_its_gap_takes_to_cross() {
        assert!((RUSH.run_up() - 0.4).abs() < 1e-6);
        assert!((RUSH.nearest() - 0.2).abs() < 1e-6);
        assert!((RUSH.seconds_across(0.4) - (0.2 + FOLLOW_THROUGH)).abs() < 1e-6);
        assert!((RUSH.seconds_across(1.2) - (0.4 + FOLLOW_THROUGH)).abs() < 1e-6);
        assert!((RUSH.seconds_across(0.1) - (0.1 + FOLLOW_THROUGH)).abs() < 1e-6);
    }

    fn ball(world: &mut PhysicsWorld, at: Point3<f32>, radius: f32) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic().position(at));
        world.attach_collider(body, ColliderDesc::sphere(radius).density(1000.0));
        body
    }

    /// A world with a striker centred 0.6 m up at the origin and a ball
    /// where `at` says.
    fn yard(at: Point3<f32>) -> (PhysicsWorld, RigidBodyHandle, RigidBodyHandle) {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let striker = ball(&mut world, Point3::new(0.0, 0.6, 0.0), 0.3);
        let target = ball(&mut world, at, 0.3);
        (world, striker, target)
    }

    /// The body a rush from the striker along +x meets, at `height`.
    fn meets(
        world: &PhysicsWorld,
        striker: RigidBodyHandle,
        height: f32,
    ) -> Option<RigidBodyHandle> {
        let centre = Point3::new(0.0, 0.6, 0.0);
        RUSH.meets(world, &EmptyGeometry, striker, centre, Vector3::x(), height)
            .map(|(body, _)| body)
    }

    /// A body in front of the striker, within its carry, is met across the
    /// gap between them; one behind it, beside it or past its carry is not.
    #[test]
    fn a_rush_meets_what_is_in_front_within_its_carry() {
        let (world, striker, target) = yard(Point3::new(1.2, 0.6, 0.0));
        let centre = Point3::new(0.0, 0.6, 0.0);
        let met = RUSH.meets(&world, &EmptyGeometry, striker, centre, Vector3::x(), 0.6);
        let (body, gap) = met.expect("it meets what is in front");
        assert_eq!(body, target);
        assert!((gap - 0.6).abs() < 1e-3, "{gap}");
        for at in [
            Point3::new(-1.2, 0.5, 0.0),
            Point3::new(0.0, 0.5, 1.2),
            Point3::new(1.8, 0.5, 0.0),
        ] {
            let (world, striker, _) = yard(at);
            assert_eq!(meets(&world, striker, 0.5), None, "{at:?}");
        }
    }

    /// A rush is swept at the height of what it strikes, as far as its
    /// body reaches: a small thing low in front is met, though the
    /// striker's middle would pass over it.
    #[test]
    fn a_rush_is_swept_at_the_height_of_what_it_strikes() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let striker = ball(&mut world, Point3::new(0.0, 0.6, 0.0), 0.3);
        let low = ball(&mut world, Point3::new(0.9, 0.15, 0.0), 0.12);
        assert_eq!(meets(&world, striker, 0.6), None, "level with its middle");
        assert_eq!(meets(&world, striker, 0.15), Some(low));
    }

    /// The first body in the way is the one met.
    #[test]
    fn a_body_in_the_way_is_met_first() {
        let (mut world, striker, _) = yard(Point3::new(1.2, 0.6, 0.0));
        let between = ball(&mut world, Point3::new(0.65, 0.6, 0.0), 0.1);
        assert_eq!(meets(&world, striker, 0.6), Some(between));
    }
}
