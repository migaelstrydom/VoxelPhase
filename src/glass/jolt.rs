//! The load a struck body puts on the pieces it carries.
//!
//! A pane held in a frame is moved by its joints whenever the frame is.
//! What the frame does steadily (rests on the ground, falls, is carried
//! round a corner) loads nothing. What cracks glass is the frame being
//! struck: stopped dead, or kicked. The change of velocity the step's hits
//! gave the body, felt by a piece riding on it, is what the piece's bonds
//! had to give it.
//!
//! ```text
//!   hits on the body ──▶ jolt_of ──▶ Motion { linear, angular }
//!                                      .at(r) for the piece at r
//!                                        └─▶ mass × |Δv| = load
//! ```

use nalgebra::Vector3;

use crate::physics::{PhysicsWorld, RigidBodyHandle};

/// A rigid motion increment: a linear part and an angular part, evaluated at
/// a point by the usual `linear + angular × r`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Motion {
    pub linear: Vector3<f32>,
    pub angular: Vector3<f32>,
}

impl Motion {
    /// The increment felt at body-relative world offset `r`.
    pub fn at(&self, r: Vector3<f32>) -> Vector3<f32> {
        self.linear + self.angular.cross(&r)
    }
}

/// The change of velocity the last step's hits gave `body`: each hit's
/// impulse pushing it where it struck. A side backed against a hit passes
/// it on and is not moved by it.
pub fn jolt_of(world: &PhysicsWorld, body: RigidBodyHandle) -> Motion {
    let Some(rigid) = world.body(body).filter(|b| b.mass() > 0.0) else {
        return Motion::default();
    };
    let mut push = Vector3::zeros();
    let mut twist = Vector3::zeros();
    for hit in world.hits_on(body) {
        let Some(side) = hit.side(body).filter(|side| !side.backed()) else {
            continue;
        };
        let impulse = side.push() * hit.impulse();
        if !impulse.iter().all(|x| x.is_finite()) {
            continue;
        }
        push += impulse;
        twist += (hit.point - rigid.position()).cross(&impulse);
    }
    Motion {
        linear: push / rigid.mass(),
        angular: rigid.world_inv_inertia() * twist,
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    const DT: f32 = 1.0 / 60.0;

    /// The largest jolt a box dropped at `speed` onto the ground took, m/s,
    /// over the frames it fell and landed.
    fn landing_jolt(speed: f32) -> f32 {
        let geometry = FlatQuadGeometry::new(50.0);
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, 0.3, 0.0))
                .linear_velocity(Vector3::new(0.0, -speed, 0.0)),
        );
        world.attach_collider(
            body,
            ColliderDesc::box_shape(Vector3::repeat(0.25)).restitution(0.0),
        );
        let mut stepper = SequentialStepper::new(DT, 4);
        let mut debug = DebugLines::default();
        (0..30)
            .map(|_| {
                stepper.step(&mut world, DT, &geometry, &[], &[], &mut debug);
                jolt_of(&world, body).linear.magnitude()
            })
            .fold(0.0, f32::max)
    }

    /// Falling is no jolt and resting is none; landing stops the body by
    /// its speed.
    #[test]
    fn a_landing_jolts_by_the_speed_it_stops() {
        let jolt = landing_jolt(6.0);
        assert!((jolt - 6.0).abs() < 0.3, "jolted {jolt} m/s");
        assert!(landing_jolt(0.0) < 1e-6);
    }

    #[test]
    fn a_spin_loads_the_rim_more_than_the_hub() {
        let spin = Motion {
            linear: Vector3::zeros(),
            angular: Vector3::new(0.0, 0.0, 2.0),
        };
        assert_eq!(spin.at(Vector3::zeros()).magnitude(), 0.0);
        assert!((spin.at(Vector3::new(1.0, 0.0, 0.0)).magnitude() - 2.0).abs() < 1e-6);
    }
}
