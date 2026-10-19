use std::f32::consts::{PI, TAU};

use nalgebra::{Point3, Vector3};
use specs::{Component, DenseVecStorage, Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::character::CharacterIntent;
use crate::components::RigidBodyComponent;
use crate::damage::Dead;
use crate::systems::PhysicsResource;
use crate::time::Time;
use crate::water::{WaterQuery, WaterWorld};

/// A creature that moves by swimming: fish, turtles.
///
/// Like [`crate::creature::Roller`], it reads the [`CharacterIntent`] a brain
/// writes and turns it into motion its own way, here as thrust through the
/// water. The water does the rest: a swimmer is as dense as the water, so
/// buoyancy holds it up and the water's drag slows it, and nothing here
/// fights either. Out of the water it has no authority at all and flops where
/// it lands.
///
/// ```text
///   intent ─► shoal check ─► thrust toward the heading (cruise, or sprint)
///   water column ──────────► hold a depth in it
///   velocity ──────────────► yaw toward travel, with a tail beat
/// ```
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Swimmer {
    /// The body's mass, kg, which the thrust and depth hold are sized by.
    pub mass: f32,
    /// The body's moment of inertia about world up, kg·m², which the turn is
    /// sized by.
    pub yaw_inertia: f32,
    /// Top speed, m/s: what a sprint asks for.
    pub sprint_speed: f32,
    /// Speed a wander or a calm approach asks for, m/s.
    pub cruise_speed: f32,
    /// Seconds to reach `sprint_speed` from rest.
    pub accel_time: f32,
    /// Where in the water column it swims: 0 just under the surface, 1 just
    /// over the floor.
    pub depth_share: f32,
    /// Closest it comes to the surface or the floor, m: its own half-height
    /// and a little more.
    pub clearance: f32,
    /// Tail beats per second at top speed.
    pub beat_rate: f32,
    /// How far each beat swings the body's heading either way, radians.
    pub beat_swing: f32,
    /// Where in its beat the tail is, radians. Advanced by this system.
    pub beat_phase: f32,
}

impl Swimmer {
    /// The water depth below which it will not swim on, m.
    fn shallowest(&self) -> f32 {
        self.clearance * 2.5
    }
}

/// How far ahead, in seconds of travel at top speed, a swimmer looks for
/// water too shallow to swim on. At least `MIN_LOOKAHEAD` metres.
const LOOKAHEAD_TIME: f32 = 0.6;

/// The shortest look ahead, m.
const MIN_LOOKAHEAD: f32 = 1.0;

/// Turns tried, either way, when the water ahead runs shallow: the first with
/// water deep enough wins, so a swimmer slides along a shore rather than
/// stopping dead at it.
const SHOAL_TURNS: [f32; 3] = [PI / 4.0, PI / 2.0, 3.0 * PI / 4.0];

/// Vertical speed per metre off its depth that a swimmer asks for, 1/s.
const DEPTH_GAIN: f32 = 1.2;

/// Fastest it climbs or dives to its depth, m/s.
const DEPTH_SPEED: f32 = 0.8;

/// Yaw rate per radian off its heading that a swimmer asks for, 1/s.
const TURN_GAIN: f32 = 4.0;

/// Fastest it turns, rad/s.
const TURN_RATE: f32 = 4.0;

/// Share of a full beat a swimmer keeps up at rest, so a hovering fish still
/// fins rather than freezing.
const IDLE_BEAT: f32 = 0.3;

/// Applies swimming from intent.
pub struct SwimmerLocomotionSystem;

impl<'a> System<'a> for SwimmerLocomotionSystem {
    type SystemData = (
        Read<'a, Time>,
        Write<'a, PhysicsResource>,
        Option<Read<'a, WaterWorld>>,
        WriteStorage<'a, Swimmer>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Dead>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, mut physics_res, water, mut swimmers, intents, rigid_bodies, deads) = data;
        let Some(water) = water else {
            return;
        };
        let water = water.query();
        let dt = time.delta_seconds();
        if dt <= 0.0 {
            return;
        }

        for (swimmer, intent, rb, _) in (&mut swimmers, &intents, &rigid_bodies, !&deads).join() {
            let Some(body) = physics_res.world.body(rb.0) else {
                continue;
            };
            let (position, velocity) = (body.position(), body.linear_velocity());
            let (rotation, spin) = (body.rotation(), body.angular_velocity());

            // Out of the water a swimmer has nothing to push against.
            let Some(here) = water.sample(position) else {
                continue;
            };
            if position.y > here.surface {
                continue;
            }

            let flat = Vector3::new(velocity.x, 0.0, velocity.z);
            let speed_share = (flat.norm() / swimmer.sprint_speed).min(1.0);

            // Thrust: toward the heading, at cruise or sprint, out of shoals.
            let wanted = Vector3::new(intent.direction.x, 0.0, intent.direction.z);
            let target_flat = match wanted.try_normalize(1e-4) {
                Some(direction) => {
                    let pace = if intent.sprint {
                        swimmer.sprint_speed
                    } else {
                        swimmer.cruise_speed
                    };
                    let lookahead = (swimmer.sprint_speed * LOOKAHEAD_TIME).max(MIN_LOOKAHEAD);
                    open_water(&water, swimmer, position, direction, lookahead)
                        .map_or(Vector3::zeros(), |d| d * pace)
                }
                None => Vector3::zeros(),
            };
            let accel = swimmer.sprint_speed / swimmer.accel_time.max(1e-3);
            let flat_change = (target_flat - flat).cap_magnitude(accel * dt);

            // Depth: a share of the way down the column, kept off both ends.
            let top = here.surface - swimmer.clearance;
            let bottom = (here.floor + swimmer.clearance).min(top);
            let target_y = top + (bottom - top) * swimmer.depth_share;
            let wanted_rise =
                ((target_y - position.y) * DEPTH_GAIN).clamp(-DEPTH_SPEED, DEPTH_SPEED);
            let rise_change = (wanted_rise - velocity.y).clamp(-accel * dt, accel * dt);

            let impulse = (flat_change + Vector3::y() * rise_change) * swimmer.mass;
            physics_res.world.apply_impulse(rb.0, impulse);

            // Heading: along its travel, or toward where it means to go when
            // it has barely started; the tail beat swings it either side.
            swimmer.beat_phase = (swimmer.beat_phase
                + TAU * swimmer.beat_rate * speed_share.max(IDLE_BEAT) * dt)
                % TAU;
            let travel = if flat.norm() > 0.1 { flat } else { target_flat };
            let Some(travel) = travel.try_normalize(1e-4) else {
                continue;
            };
            let forward = rotation * Vector3::z();
            let facing = forward.z.atan2(forward.x);
            let heading = travel.z.atan2(travel.x)
                + swimmer.beat_swing * swimmer.beat_phase.sin() * speed_share.max(IDLE_BEAT);
            let off = wrap_angle(heading - facing);
            // atan2(z, x) grows clockwise seen from above; a yaw about +Y
            // turns +Z toward +X, so the rate that closes `off` is its negative.
            let wanted_spin = (-off * TURN_GAIN).clamp(-TURN_RATE, TURN_RATE);
            let spin_change = wanted_spin - spin.y;
            physics_res
                .world
                .apply_angular_impulse(rb.0, Vector3::y() * spin_change * swimmer.yaw_inertia);
        }
    }
}

/// The direction to swim on from `position`: `direction` if the water a
/// `lookahead` along it is deep enough, else the smallest turn either way that
/// finds such water, else none.
fn open_water(
    water: &WaterQuery<'_>,
    swimmer: &Swimmer,
    position: Point3<f32>,
    direction: Vector3<f32>,
    lookahead: f32,
) -> Option<Vector3<f32>> {
    let deep_enough = |d: Vector3<f32>| {
        water
            .sample(position + d * lookahead)
            .is_some_and(|ahead| ahead.surface - ahead.floor >= swimmer.shallowest())
    };
    if deep_enough(direction) {
        return Some(direction);
    }
    SHOAL_TURNS
        .iter()
        .flat_map(|&turn| [turn, -turn])
        .map(|turn| rotate_flat(direction, turn))
        .find(|&d| deep_enough(d))
}

/// `v` turned `angle` radians about world up.
fn rotate_flat(v: Vector3<f32>, angle: f32) -> Vector3<f32> {
    let (sin, cos) = angle.sin_cos();
    Vector3::new(v.x * cos + v.z * sin, 0.0, -v.x * sin + v.z * cos)
}

/// `angle` brought into (-π, π].
fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(TAU) - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_quarter_turn_is_a_right_angle_and_keeps_length() {
        let v = Vector3::new(1.0, 0.0, 0.0);
        let turned = rotate_flat(v, PI / 2.0);
        assert!(turned.dot(&v).abs() < 1e-6);
        assert!((turned.norm() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn wrapped_angles_take_the_short_way_round() {
        assert!((wrap_angle(1.5 * PI) + 0.5 * PI).abs() < 1e-5);
        assert!((wrap_angle(-1.5 * PI) - 0.5 * PI).abs() < 1e-5);
        assert!(wrap_angle(0.25).abs() - 0.25 < 1e-6);
    }

    /// The yaw sign: a positive yaw about +Y turns +Z toward +X, so the
    /// `atan2(z, x)` of a body's forward falls as it yaws positively. The
    /// turn in `SwimmerLocomotionSystem` relies on exactly this.
    #[test]
    fn a_positive_yaw_lowers_the_facing_angle() {
        let yaw = nalgebra::UnitQuaternion::from_axis_angle(&Vector3::<f32>::y_axis(), 0.1);
        let forward = yaw * Vector3::z();
        let facing = forward.z.atan2(forward.x);
        assert!(facing < PI / 2.0, "facing {facing} should fall below π/2");
    }
}
