use rustc_hash::FxHashMap;

use generational_arena::{Arena, Index};
use nalgebra::{Point3, UnitQuaternion};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

/// Decides which bodies have been still long enough to sleep.
///
/// Still means slow, not moving, and not speeding up. Slow and moving differ:
/// position correction moves a body out of whatever it is sunk into without
/// giving it any velocity, so a body pushed out of a pillar it is skewered on
/// reads as motionless from its velocity alone, and would sleep there, still
/// skewered. And slow is not at rest: a body starting to fall from near a
/// balance (a tall box just past its edge, which turns at 0.025 rad/s² with
/// its centre 1 cm over) stays under every speed threshold for seconds. Its
/// speed rises every substep, where a resting body's is noise about nothing,
/// so a body whose speed has risen through a whole window is not still.
pub struct SleepTracker {
    /// Squared linear speed threshold for sleep candidacy.
    linear_threshold_sq: f32,
    /// Squared angular speed threshold for sleep candidacy.
    angular_threshold_sq: f32,
    /// Substeps below threshold required to qualify for sleep.
    delay_substeps: u32,
    /// Per-body record of how long it has been still, and where it was.
    bodies: FxHashMap<Index, Stillness>,
}

/// One body's record.
struct Stillness {
    /// Consecutive substeps the body has been still for.
    substeps_still: u32,
    /// Consecutive substeps its speed has risen for.
    substeps_rising: u32,
    /// Its speed and spin at the last update.
    speed: f32,
    spin: f32,
    /// Where the body was at the last update.
    position: Point3<f32>,
    /// How the body was turned at the last update.
    rotation: UnitQuaternion<f32>,
}

/// The least rise in speed (m/s) or spin (rad/s) over a substep that counts
/// as speeding up: far under a fall from balance's (2.5e-4 rad/s a substep
/// at 1 cm over), far over a resting body's rounding.
const RISE: f32 = 1.0e-6;

impl SleepTracker {
    pub fn new(linear_threshold: f32, angular_threshold: f32, delay_substeps: u32) -> Self {
        Self {
            linear_threshold_sq: linear_threshold * linear_threshold,
            angular_threshold_sq: angular_threshold * angular_threshold,
            delay_substeps,
            bodies: FxHashMap::default(),
        }
    }

    /// Record the body after a substep `dt` long, and whether it has now
    /// been still for long enough to sleep.
    pub fn update_body(&mut self, handle: RigidBodyHandle, body: &RigidBody, dt: f32) -> bool {
        let (position, rotation) = (body.position(), body.rotation());
        let slow = self.is_below(
            body.linear_velocity().norm_squared(),
            body.angular_velocity().norm_squared(),
        );
        // A body seen for the first time has not been watched moving yet:
        // its speed is all there is to go on.
        let unmoved = self.bodies.get(&handle.0).map_or(true, |record| {
            let travelled = (position - record.position).norm_squared();
            let turned = record.rotation.angle_to(&rotation).powi(2);
            self.is_below(travelled / (dt * dt), turned / (dt * dt))
        });
        let (speed, spin) = (
            body.linear_velocity().norm(),
            body.angular_velocity().norm(),
        );
        let record = self.bodies.entry(handle.0).or_insert(Stillness {
            substeps_still: 0,
            substeps_rising: 0,
            speed,
            spin,
            position,
            rotation,
        });
        let rising = speed > record.speed + RISE || spin > record.spin + RISE;
        record.substeps_rising = if rising {
            record.substeps_rising.saturating_add(1)
        } else {
            0
        };
        let falling = record.substeps_rising >= self.delay_substeps;
        record.substeps_still = if slow && unmoved && !falling {
            record.substeps_still.saturating_add(1)
        } else {
            0
        };
        record.speed = speed;
        record.spin = spin;
        record.position = position;
        record.rotation = rotation;
        record.substeps_still >= self.delay_substeps
    }

    pub fn clear_body(&mut self, handle: RigidBodyHandle) {
        self.bodies.remove(&handle.0);
    }

    pub fn retain_live(&mut self, bodies: &Arena<RigidBody>) {
        self.bodies.retain(|idx, _| bodies.contains(*idx));
    }

    /// Whether linear and angular rates, squared, are both under threshold.
    fn is_below(&self, linear_sq: f32, angular_sq: f32) -> bool {
        linear_sq < self.linear_threshold_sq && angular_sq < self.angular_threshold_sq
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};

    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::SolidsGeometry;
    use crate::physics::{
        ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc, SequentialStepper, Stepper,
    };

    use super::*;

    /// A box 1.5 m square and 3.1 m tall, 70 kg (the Ledge Garden's light
    /// tall box), set down on one bottom edge tilted `past` radians beyond
    /// its balance over that edge, and left for three seconds: how far it
    /// is tilted then, from upright.
    fn tilt_after(past: f32) -> f32 {
        let floor = SolidsGeometry::new(20.0, &[]);
        let (hx, hy) = (0.75f32, 1.55f32);
        let tilt = (hx / hy).atan() + past;
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), tilt);
        let corner = rotation * Vector3::new(hx, -hy, 0.0);
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, -corner.y + 0.001, 0.0))
                .rotation(rotation),
        );
        world.attach_collider(
            body,
            ColliderDesc::box_shape(Vector3::new(hx, hy, 0.75)).density(10.0),
        );
        world.wake_body(body);
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        for _ in 0..180 {
            stepper.step(
                &mut world,
                1.0 / 60.0,
                &floor,
                &[],
                &[],
                &mut DebugLines::default(),
            );
        }
        world.body(body).unwrap().rotation().angle()
    }

    /// Near its balance a box starts to fall too slowly for any speed
    /// threshold to tell from resting; it slept there, balanced on its edge.
    #[test]
    fn a_box_just_past_its_balance_falls_rather_than_sleeping_on_its_edge() {
        assert!(tilt_after(0.01) > 1.5, "it topples onto its side");
        assert!(tilt_after(0.05) > 1.5, "it topples onto its side");
        assert!(tilt_after(-0.05) < 0.1, "short of balance it falls back");
    }
}
