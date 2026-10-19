//! The spring slider end to end: a sprung body sags under its weight by
//! `g/ω²`, bobs at the spring's frequency, keeps to its line and its
//! attitude, and throws back what lands on it.

use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::super::geometry::{EmptyGeometry, FlatQuadGeometry};
use crate::debug::DebugLines;
use crate::physics::constraint::spring::Spring;
use crate::physics::constraint::spring_slider::SpringSlider;
use crate::physics::constraint::ConstraintKind;
use crate::physics::{
    ColliderDesc, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, SequentialStepper, StaticGeometry,
    Stepper,
};

const FIXED_DT: f32 = 1.0 / 240.0;
const FRAME_DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const REST: Point3<f32> = Point3::new(0.0, 2.0, 0.0);

/// A 1 m square bed of `density` hung from the world at `REST` on a spring
/// stated against its own mass.
struct Bed {
    world: PhysicsWorld,
    stepper: SequentialStepper,
    bed: RigidBodyHandle,
}

impl Bed {
    fn new(spring: Spring, density: f32) -> Self {
        let mut world = PhysicsWorld::default();
        let bed = world.create_body(RigidBodyDesc::dynamic().position(REST));
        let _ = world.attach_collider(
            bed,
            ColliderDesc::box_shape(Vector3::new(0.5, 0.05, 0.5)).density(density),
        );
        let mass = world.body(bed).unwrap().mass();
        world.create_constraint(ConstraintKind::SpringSlider(SpringSlider {
            body_a: None,
            body_b: bed,
            local_anchor_a: REST.coords,
            local_anchor_b: Vector3::zeros(),
            local_axis_a: Vector3::y_axis(),
            reference: UnitQuaternion::identity(),
            spring,
            reference_mass: mass,
        }));
        world.wake_body(bed);
        Self {
            world,
            stepper: SequentialStepper::new(FIXED_DT, 12),
            bed,
        }
    }

    fn step(&mut self, ground: &dyn StaticGeometry) {
        let mut debug_lines = DebugLines::default();
        self.stepper.step(
            &mut self.world,
            FRAME_DT,
            ground,
            &[],
            &[],
            &mut debug_lines,
        );
    }

    fn position(&self) -> Point3<f32> {
        self.world.body(self.bed).unwrap().position()
    }
}

#[test]
fn a_sprung_body_sags_by_its_weight_over_the_stiffness() {
    let spring = Spring::from_hz(2.0, 0.5);
    let mut bed = Bed::new(spring, 200.0);
    for _ in 0..240 {
        bed.step(&EmptyGeometry);
    }
    let sag = REST.y - bed.position().y;
    let expected = GRAVITY / (spring.angular_frequency * spring.angular_frequency);
    assert!(
        (sag - expected).abs() < 0.05 * expected,
        "sagged {sag:.4} m, expected {expected:.4} m"
    );
}

#[test]
fn a_sprung_body_bobs_at_the_springs_frequency() {
    let spring = Spring::from_hz(2.0, 0.02);
    let mut bed = Bed::new(spring, 200.0);
    let settled = REST.y - GRAVITY / (spring.angular_frequency * spring.angular_frequency);

    // Released from rest at the unloaded length, it swings about the
    // loaded one; count the upward crossings over three seconds.
    let mut crossings = Vec::new();
    let mut below = false;
    for frame in 0..180 {
        bed.step(&EmptyGeometry);
        let y = bed.position().y;
        if below && y > settled {
            crossings.push(frame as f32 * FRAME_DT);
        }
        below = y < settled;
    }
    assert!(crossings.len() >= 3, "crossings at {crossings:?}");
    let period = (crossings[crossings.len() - 1] - crossings[0]) / (crossings.len() - 1) as f32;
    assert!(
        (period - 0.5).abs() < 0.05,
        "period {period:.3} s, expected 0.5 s (crossings at {crossings:?})"
    );
}

#[test]
fn a_sprung_body_keeps_to_its_line_and_attitude() {
    let mut bed = Bed::new(Spring::from_hz(2.0, 0.5), 200.0);
    let mass = bed.world.body(bed.bed).unwrap().mass();
    bed.world
        .apply_impulse(bed.bed, Vector3::new(3.0 * mass, 0.0, -2.0 * mass));
    bed.world
        .apply_angular_impulse(bed.bed, Vector3::new(0.5, 2.0, -0.5) * mass);
    let mut widest = 0.0f32;
    for _ in 0..60 {
        bed.step(&EmptyGeometry);
        let p = bed.position();
        widest = widest.max(Vector3::new(p.x, 0.0, p.z).norm());
    }
    let turned = bed.world.body(bed.bed).unwrap().rotation().angle();
    assert!(widest < 0.02, "strayed {widest:.4} m off its line");
    assert!(turned < 0.01, "turned {turned:.4} rad off its attitude");
}

#[test]
fn a_ball_dropped_on_a_sprung_bed_is_thrown_back_up() {
    // A bed light beside what lands on it, as a trampoline's is: 4 kg under
    // a 17 kg ball. A perfectly plastic landing keeps 17/21 of the ball's
    // energy, and parting where the two move together keeps 17/21 again,
    // two thirds in all; the spring's damping and its implicit step take
    // some of the rest, and the drop comes back half way.
    let mut bed = Bed::new(Spring::from_hz(4.0, 0.01), 40.0);
    let radius = 0.2;
    let drop_from = REST.y + 2.0;
    let touches_at = REST.y + 0.05 + radius;
    let ball = bed
        .world
        .create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, drop_from, 0.0)));
    let _ = bed.world.attach_collider(
        ball,
        ColliderDesc::sphere(radius).density(500.0).restitution(0.0),
    );
    bed.world.wake_body(ball);
    let ground = FlatQuadGeometry::new(10.0);

    // Wait for it to land and leave again, then take its highest point.
    let mut landed = false;
    let mut peak = f32::MIN;
    for _ in 0..180 {
        bed.step(&ground);
        let body = bed.world.body(ball).unwrap();
        if !landed && body.linear_velocity().y > 0.0 {
            landed = true;
        }
        if landed {
            peak = peak.max(body.position().y);
        }
    }
    let kept = (peak - touches_at) / (drop_from - touches_at);
    assert!(landed, "the ball never came back up");
    assert!(
        kept > 0.45,
        "a ball with no restitution of its own came back {:.0} % of its drop",
        kept * 100.0
    );
    assert!(
        kept < 1.0,
        "the bed added energy: {:.0} % of the drop",
        kept * 100.0
    );
}
