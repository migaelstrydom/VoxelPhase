//! What sets a grenade off on impact.

use crate::physics::{Hit, PhysicsWorld, RigidBodyHandle};

/// Of this frame's hits on the grenade `body`, the one that changed its speed
/// most, if by at least `speed`, m/s.
///
/// It reads only what the blow did to the grenade, never what it struck. A
/// glancing blow counts only its speed into the surface; something light
/// gives way, and changes the grenade's speed by less than it arrived at.
pub(crate) fn detonating_hit(
    world: &PhysicsWorld,
    body: RigidBodyHandle,
    speed: f32,
) -> Option<&Hit> {
    let mass = world.body(body)?.mass();
    world
        .hits_on(body)
        .map(|hit| (hit, hit.delta_v(mass)))
        .filter(|&(_, delta_v)| delta_v >= speed)
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(hit, _)| hit)
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::{
        Atmosphere, ColliderDesc, PhysicsConfig, RigidBodyDesc, SequentialStepper, Stepper,
    };
    use crate::projectile::{create_grenade_body, GrenadeConfig};

    const FRAME_DT: f32 = 1.0 / 60.0;

    /// Flat ground in vacuum, stepped as the game steps it.
    struct Range {
        world: PhysicsWorld,
        stepper: SequentialStepper,
        ground: FlatQuadGeometry,
        debug: DebugLines,
        config: GrenadeConfig,
    }

    impl Range {
        fn new(gravity: bool) -> Self {
            let mut physics = PhysicsConfig::default();
            physics.atmosphere = Atmosphere::VACUUM;
            if !gravity {
                physics.gravity = Vector3::zeros();
            }
            Self {
                world: PhysicsWorld::new(physics),
                stepper: SequentialStepper::new(1.0 / 240.0, 12),
                ground: FlatQuadGeometry::new(50.0),
                debug: DebugLines::default(),
                config: GrenadeConfig::default(),
            }
        }

        fn grenade(&mut self, origin: Point3<f32>, velocity: Vector3<f32>) -> RigidBodyHandle {
            let body = create_grenade_body(&mut self.world, &self.config, origin, velocity);
            self.world.wake_body(body);
            body
        }

        /// Step until a hit would set `grenade` off, at most `seconds`.
        fn detonation(&mut self, grenade: RigidBodyHandle, seconds: f32) -> Option<Hit> {
            for _ in 0..(seconds / FRAME_DT) as usize {
                self.stepper.step(
                    &mut self.world,
                    FRAME_DT,
                    &self.ground,
                    &[],
                    &[],
                    &mut self.debug,
                );
                let speed = self.config.detonation_impact_speed;
                if let Some(hit) = detonating_hit(&self.world, grenade, speed) {
                    return Some(*hit);
                }
            }
            None
        }
    }

    /// A grenade dropped so it arrives at `arrival` m/s, `phase` of a frame's
    /// travel further, clear of the ground quad's diagonal (x = z).
    fn dropped(arrival: f32, phase: f32) -> Option<Hit> {
        let mut range = Range::new(true);
        let gravity = range.config.gravity;
        let fall = arrival * arrival / (2.0 * gravity) + phase * arrival * FRAME_DT;
        let origin = Point3::new(3.0, range.config.radius + fall, 1.0);
        let grenade = range.grenade(origin, Vector3::zeros());
        range.detonation(grenade, 2.0)
    }

    #[test]
    fn a_throw_into_the_ground_sets_it_off_where_it_struck() {
        let mut range = Range::new(true);
        let speed = range.config.throw_speed;
        let down = 30f32.to_radians();
        let grenade = range.grenade(
            Point3::new(3.0, 2.0, 1.0),
            Vector3::new(down.cos(), -down.sin(), 0.0) * speed,
        );
        let hit = range
            .detonation(grenade, 1.0)
            .expect("a throw at the ground went off");
        let centre = range.world.body(grenade).unwrap().position();
        assert!(hit.point.y.abs() < 0.02, "struck at {:?}", hit.point);
        assert!(
            (hit.point.xz() - centre.xz()).norm() < range.config.radius,
            "struck at {:?}, under a grenade at {centre:?}",
            hit.point
        );
    }

    /// A landing is judged by the speed it arrived at, which does not depend
    /// on where in a frame it touched down: a fifth over the threshold goes
    /// off at every phase, a fifth under it at none.
    #[test]
    fn a_landing_goes_off_by_its_speed_whatever_the_frame_phase() {
        let threshold = GrenadeConfig::default().detonation_impact_speed;
        for phase in [0.0, 0.2, 0.4, 0.6, 0.8] {
            assert!(
                dropped(1.2 * threshold, phase).is_some(),
                "a landing over the threshold, phase {phase}, did not go off"
            );
            assert!(
                dropped(0.8 * threshold, phase).is_none(),
                "a landing under the threshold, phase {phase}, went off"
            );
        }
    }

    /// Skimming the ground at full throwing speed, it meets it at little more
    /// than the speed it was sinking at.
    #[test]
    fn a_glancing_skim_does_not_set_it_off() {
        let mut range = Range::new(true);
        let grenade = range.grenade(
            Point3::new(3.0, range.config.radius + 0.05, 1.0),
            Vector3::new(range.config.throw_speed, -1.0, 0.0),
        );
        assert_eq!(range.detonation(grenade, 1.0), None);
        let velocity = range.world.body(grenade).unwrap().linear_velocity();
        assert!(velocity.x > 10.0, "it skimmed on: {velocity:?}");
    }

    /// Thrown head on into a ball at rest, it goes off against one as heavy
    /// as itself, which takes half its speed, but not against one a tenth of
    /// its weight, which barely slows it.
    #[test]
    fn a_light_body_gives_way_and_does_not_set_it_off() {
        for (density, goes_off) in [(2000.0, true), (200.0, false)] {
            let mut range = Range::new(false);
            let speed = range.config.throw_speed;
            let grenade = range.grenade(Point3::new(-1.0, 5.0, 0.0), Vector3::x() * speed);
            let ball = range
                .world
                .create_body(RigidBodyDesc::dynamic().position(Point3::new(1.0, 5.0, 0.0)));
            range.world.attach_collider(
                ball,
                ColliderDesc::sphere(range.config.radius)
                    .density(density)
                    .restitution(0.0),
            );
            range.world.wake_body(ball);
            let hit = range.detonation(grenade, 0.5);
            assert_eq!(hit.is_some(), goes_off, "density {density}: {hit:?}");
            if let Some(hit) = hit {
                assert!(hit.involves(ball));
            }
        }
    }
}
