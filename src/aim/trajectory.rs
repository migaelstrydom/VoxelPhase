//! Flying a [`Launch`] until it hits something.

use nalgebra::{Point3, Vector3};

use crate::sensing::ProbeTarget;

use super::launch::Launch;

/// Where a predicted flight ends.
#[derive(Debug, Clone, Copy)]
pub struct Impact {
    /// World-space point on the surface that was hit.
    pub point: Point3<f32>,
    /// Surface normal there, pointing away from the surface.
    pub normal: Vector3<f32>,
    /// Seconds of flight before the hit.
    pub time: f32,
}

/// Walks a ballistic arc in straight segments, casting each one against the
/// world and reporting the first thing hit.
///
/// The arc is sampled rather than solved because the world is arbitrary
/// geometry, not a plane: the only general way to ask "what does this curve
/// hit first" is to ask the ray caster repeatedly. Cost is one cast per step,
/// so `step` trades accuracy for query count and `max_time` bounds the worst
/// case — a throw into open sky costs `max_time / step` casts and no more.
///
/// A chord misses the arc it spans by `gravity * step^2 / 8`, which is under a
/// centimetre at the default step and falls off quadratically. The step is
/// therefore chosen for cast count, not for accuracy: each cast walks every
/// collider in the world, and this runs every frame the player is aiming.
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryPredictor {
    /// Seconds of flight per straight segment. A chord of the true arc, so a
    /// shorter step tracks the curve more closely and costs another cast.
    pub step: f32,
    /// How far ahead to look before giving up, in seconds of flight.
    pub max_time: f32,
}

impl Default for TrajectoryPredictor {
    fn default() -> Self {
        Self {
            step: 1.0 / 12.0,
            max_time: 3.0,
        }
    }
}

impl TrajectoryPredictor {
    /// Fly `launch` through `world` and report the first impact, if the flight
    /// reaches one inside [`Self::max_time`].
    ///
    /// The projectile is treated as a point. A real grenade would touch a
    /// surface one radius early, which moves the landing by centimetres —
    /// below what the cursor can show, and the point on the surface is the
    /// more useful thing to draw anyway.
    pub fn predict(&self, launch: &Launch, world: &dyn ProbeTarget) -> Option<Impact> {
        if self.step <= 0.0 {
            return None;
        }

        let mut elapsed = 0.0;
        let mut from = launch.origin;

        while elapsed < self.max_time {
            let next_elapsed = (elapsed + self.step).min(self.max_time);
            let to = launch.position_at(next_elapsed);
            let segment = to - from;
            let length = segment.magnitude();

            if length > 1e-5 {
                let direction = segment / length;
                if let Some(hit) = world.raycast(from, direction, length) {
                    let fraction = hit.t.clamp(0.0, 1.0);
                    return Some(Impact {
                        point: hit.point,
                        normal: hit.normal,
                        time: elapsed + (next_elapsed - elapsed) * fraction,
                    });
                }
            }

            from = to;
            elapsed = next_elapsed;
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensing::ProbeHit;

    /// A ground plane at a fixed height, for tests that only care about when
    /// the arc comes down.
    struct Ground {
        height: f32,
    }

    impl ProbeTarget for Ground {
        fn raycast(
            &self,
            origin: Point3<f32>,
            direction: Vector3<f32>,
            length: f32,
        ) -> Option<ProbeHit> {
            let end = origin + direction * length;
            if origin.y < self.height || end.y > self.height {
                return None;
            }
            let span = origin.y - end.y;
            let t = if span.abs() < 1e-6 {
                0.0
            } else {
                (origin.y - self.height) / span
            };
            Some(ProbeHit {
                t,
                point: origin + direction * (length * t),
                normal: Vector3::y(),
            })
        }
    }

    /// Nothing anywhere: every cast misses.
    struct Void;

    impl ProbeTarget for Void {
        fn raycast(&self, _: Point3<f32>, _: Vector3<f32>, _: f32) -> Option<ProbeHit> {
            None
        }
    }

    fn flat_throw() -> Launch {
        Launch {
            origin: Point3::new(0.0, 5.0, 0.0),
            velocity: Vector3::new(10.0, 0.0, 0.0),
            gravity: Vector3::new(0.0, -10.0, 0.0),
        }
    }

    #[test]
    fn a_throw_lands_where_the_arc_meets_the_ground() {
        let impact = TrajectoryPredictor::default()
            .predict(&flat_throw(), &Ground { height: 0.0 })
            .expect("a throw over ground must land");

        // Falls 5m at 10m/s^2 => 1s of flight, 10m downrange.
        assert!((impact.time - 1.0).abs() < 0.05, "time was {}", impact.time);
        assert!(
            (impact.point.x - 10.0).abs() < 0.2,
            "landed at x = {}",
            impact.point.x
        );
        assert!(
            impact.point.y.abs() < 0.2,
            "landed at y = {}",
            impact.point.y
        );
    }

    #[test]
    fn a_throw_into_nothing_predicts_no_landing() {
        assert!(TrajectoryPredictor::default()
            .predict(&flat_throw(), &Void)
            .is_none());
    }

    #[test]
    fn the_search_stops_at_the_horizon_it_was_given() {
        // Ground far below: reachable, but not within half a second of flight.
        let predictor = TrajectoryPredictor {
            step: 1.0 / 60.0,
            max_time: 0.5,
        };

        assert!(predictor
            .predict(&flat_throw(), &Ground { height: -100.0 })
            .is_none());
    }
}
