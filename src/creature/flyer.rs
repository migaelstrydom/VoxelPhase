//! Flying: a creature that holds a height over the ground and stoops on
//! what it hunts.
//!
//! ```text
//!   CharacterIntent ──▶ level flight toward where the brain steers, or,
//!                       below level, a stoop down that line
//!   TerrainWorld ─────▶ cruise height over the ground beneath
//!   how hard it works ─▶ WingBeat ──poses──▶ PosedModel (its two wings)
//! ```
//!
//! The same seam as walking and swimming: the brain says which way, and
//! this turns that into impulses on the body. Gravity pulls a flyer like
//! anything else; what holds it up is the lift given it here every frame,
//! so one that dies, and is no longer flown, falls with its wings as they
//! were. Stooping, it is given none: it folds its wings and falls, and they
//! only keep it on its line.

use std::f32::consts::{PI, TAU};

use nalgebra::{UnitQuaternion, Vector3};
use specs::{
    Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, Write, WriteStorage,
};

use crate::character::CharacterIntent;
use crate::components::{PosedModel, RigidBodyComponent};
use crate::damage::Dead;
use crate::model::Transform;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::time::Time;

/// Climb or dive speed wanted per metre off the height it is making for,
/// 1/s, and the fastest it climbs or dives, m/s.
const HEIGHT_GAIN: f32 = 2.5;
const CLIMB_SPEED: f32 = 6.0;

/// Turn rate wanted per radian off its heading, 1/s, and the fastest it
/// turns, rad/s.
const TURN_GAIN: f32 = 5.0;
const TURN_RATE: f32 = 4.0;

/// One wing of a flyer: a part of its model, and the hinge it beats on.
#[derive(Debug, Clone, Copy)]
pub struct Wing {
    /// Which part of the model it is.
    pub part: usize,
    /// Where it joins the body, in the entity's frame.
    pub root: Vector3<f32>,
    /// The axis it swings about there, in the entity's frame, pointing
    /// the way its leading edge faces.
    pub hinge: Vector3<f32>,
    /// The way it reaches out from its root at rest, in the entity's
    /// frame: it feathers about this.
    pub span: Vector3<f32>,
}

impl Wing {
    /// +1 if a right-handed turn about its hinge raises it, −1 if that
    /// lowers it: a left wing is a right wing mirrored.
    fn handed(&self) -> f32 {
        if self.hinge.cross(&self.span).y >= 0.0 {
            1.0
        } else {
            -1.0
        }
    }
}

/// How a flyer's wings beat. A wing is modelled where it is at rest, so a
/// model nothing poses is drawn with its wings level.
#[derive(Debug, Clone, Copy)]
pub struct WingBeat {
    /// The right wing and the left.
    pub wings: [Wing; 2],
    /// Radians a wing swings above its rest at the top of a beat.
    pub raise: f32,
    /// Radians it swings below its rest at the bottom.
    pub lower: f32,
    /// Radians it feathers: its leading edge tipped down as it comes
    /// down, and up as it goes up.
    pub feather: f32,
    /// Beats a second gliding.
    pub glide_rate: f32,
    /// Beats a second climbing its hardest or at full speed.
    pub hard_rate: f32,
}

impl WingBeat {
    /// Beats a second for a flyer working `effort` hard, 0 to 1.
    pub fn rate(&self, effort: f32) -> f32 {
        self.glide_rate + (self.hard_rate - self.glide_rate) * effort.clamp(0.0, 1.0)
    }

    /// Where `wing` is `phase` of the way through a beat, as a pose in the
    /// entity's frame: at the top of its swing at 0, at the bottom at ½.
    pub fn pose(&self, wing: &Wing, phase: f32) -> Transform {
        let (falling, swing) = (phase * TAU).sin_cos();
        let lift = if swing >= 0.0 {
            self.raise * swing
        } else {
            self.lower * swing
        };
        // Fastest through the middle of the stroke, and the feathering
        // with it.
        let twist = self.feather * falling;
        let axis = |v: Vector3<f32>| v.try_normalize(1e-6).unwrap_or_else(Vector3::z);
        let handed = wing.handed();
        let turn = UnitQuaternion::from_scaled_axis(axis(wing.hinge) * (handed * lift))
            * UnitQuaternion::from_scaled_axis(axis(wing.span) * (handed * twist));
        // Turned about its root, which stays where it is.
        Transform {
            translation: wing.root - turn * wing.root,
            rotation: turn,
            scale: Vector3::repeat(1.0),
        }
    }

    /// Every part's pose `phase` of the way through a beat, for a model of
    /// `parts` parts: the wings where the beat has them, the rest at rest.
    pub fn poses(&self, phase: f32, parts: usize) -> Vec<Transform> {
        let mut poses = vec![Transform::default(); parts];
        self.pose_onto(phase, &mut poses);
        poses
    }

    /// Write the wings' poses `phase` of the way through a beat into
    /// `poses`, a model's poses in part order.
    fn pose_onto(&self, phase: f32, poses: &mut [Transform]) {
        for wing in &self.wings {
            if let Some(pose) = poses.get_mut(wing.part) {
                *pose = self.pose(wing, phase);
            }
        }
    }
}

/// How a creature flies.
#[derive(Component, Debug, Clone, Copy)]
#[storage(DenseVecStorage)]
pub struct Flyer {
    /// Its body's mass, kg.
    pub mass: f32,
    /// Its body's inertia about the vertical, kg·m².
    pub yaw_inertia: f32,
    /// Level speed, m/s.
    pub speed: f32,
    /// Seconds it takes to reach that speed from rest.
    pub accel_time: f32,
    /// Height it holds over the ground, m.
    pub cruise_height: f32,
    /// How its wings beat.
    pub wings: WingBeat,
    /// How far through a beat its wings are, 0 to 1.
    pub beat_phase: f32,
}

impl Flyer {
    /// The most its wings change its velocity by, m/s².
    fn accel(&self) -> f32 {
        self.speed / self.accel_time.max(1e-3)
    }

    /// How hard it is working at `velocity`, 0 to 1: climbing, or making
    /// speed. Gliding level and slow, or coming down, it hardly works.
    fn effort(&self, velocity: Vector3<f32>) -> f32 {
        let climbing = velocity.y / CLIMB_SPEED;
        let speeding = Vector3::new(velocity.x, 0.0, velocity.z).norm() / self.speed.max(1e-3);
        climbing.max(speeding).clamp(0.0, 1.0)
    }

    /// Carry its wings `dt` seconds on through their beat at `velocity`.
    fn beat(&mut self, velocity: Vector3<f32>, dt: f32) {
        let rate = self.wings.rate(self.effort(velocity));
        self.beat_phase = (self.beat_phase + rate * dt).rem_euclid(1.0);
    }
}

/// Applies flying from intent.
pub struct FlyerLocomotionSystem;

impl<'a> System<'a> for FlyerLocomotionSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        Write<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
        WriteStorage<'a, Flyer>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, PosedModel>,
        ReadStorage<'a, Dead>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time, mut physics, terrain, mut flyers, intents, bodies, mut posed, deads) =
            data;
        let dt = time.delta_seconds();
        if dt <= 0.0 {
            return;
        }
        let gravity = physics.world.config().gravity;
        for (entity, flyer, intent, rb, _) in
            (&entities, &mut flyers, &intents, &bodies, !&deads).join()
        {
            let Some(body) = physics.world.body(rb.0) else {
                continue;
            };
            let (position, velocity) = (body.position(), body.linear_velocity());
            let (rotation, spin) = (body.rotation(), body.angular_velocity());

            // A flyer drawn as one rigid model has no wings to pose.
            flyer.beat(velocity, dt);
            if let Some(posed) = posed.get_mut(entity) {
                flyer.wings.pose_onto(flyer.beat_phase, &mut posed.poses);
            }

            // A heading below level is a stoop.
            let line = intent
                .direction
                .try_normalize(1e-4)
                .filter(|heading| heading.y < 0.0);
            let change = match line {
                Some(line) => stoop(flyer, line, velocity, dt),
                None => {
                    let ground = terrain
                        .as_ref()
                        .and_then(|terrain| terrain.mesh_surface_height_at(position.x, position.z))
                        .unwrap_or(position.y - flyer.cruise_height);
                    let level = steer(
                        flyer,
                        Vector3::new(intent.direction.x, 0.0, intent.direction.z),
                        velocity,
                        ground + flyer.cruise_height - position.y,
                        dt,
                    );
                    // Lift: what gravity will take from it this frame,
                    // given back.
                    level - gravity * body.gravity_scale() * dt
                }
            };
            physics.world.apply_impulse(rb.0, change * flyer.mass);

            // It faces the way it flies, unless its brain would have it face
            // elsewhere.
            let flat = Vector3::new(velocity.x, 0.0, velocity.z);
            let Some(travel) = intent
                .face
                .and_then(|face| Vector3::new(face.x, 0.0, face.z).try_normalize(1e-3))
                .or_else(|| flat.try_normalize(0.1))
            else {
                continue;
            };
            let forward = rotation * Vector3::z();
            let off = wrap_angle(travel.z.atan2(travel.x) - forward.z.atan2(forward.x));
            // atan2(z, x) grows clockwise seen from above; a yaw about +Y
            // turns +Z toward +X, so the rate that closes `off` is its
            // negative.
            let wanted_spin = (-off * TURN_GAIN).clamp(-TURN_RATE, TURN_RATE);
            physics.world.apply_angular_impulse(
                rb.0,
                Vector3::y() * (wanted_spin - spin.y) * flyer.yaw_inertia,
            );
        }
    }
}

/// The change of velocity a flyer makes this step: toward level flight
/// along `heading` (zero to hover) and toward the height `rise` above it.
fn steer(
    flyer: &Flyer,
    heading: Vector3<f32>,
    velocity: Vector3<f32>,
    rise: f32,
    dt: f32,
) -> Vector3<f32> {
    let accel = flyer.accel();
    let flat = Vector3::new(velocity.x, 0.0, velocity.z);
    let wanted_flat = heading
        .try_normalize(1e-4)
        .map_or_else(Vector3::zeros, |direction| direction * flyer.speed);
    let flat_change = (wanted_flat - flat).cap_magnitude(accel * dt);
    let wanted_climb = (rise * HEIGHT_GAIN).clamp(-CLIMB_SPEED, CLIMB_SPEED);
    let climb_change = (wanted_climb - velocity.y).clamp(-accel * dt, accel * dt);
    flat_change + Vector3::y() * climb_change
}

/// The change of velocity a stooping flyer's wings make this step: folded,
/// they only keep it on `line` (unit length, below level), taking off what
/// of its velocity is across it. Gravity drives it down the line.
fn stoop(flyer: &Flyer, line: Vector3<f32>, velocity: Vector3<f32>, dt: f32) -> Vector3<f32> {
    let across = velocity - line * velocity.dot(&line);
    (-across).cap_magnitude(flyer.accel() * dt)
}

fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(TAU) - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Wings a metre long from shoulders a tenth of a metre out, hinged
    /// fore and aft with their leading edges ahead: parts 1 and 2 of a
    /// model whose body is part 0.
    const WINGS: WingBeat = WingBeat {
        wings: [
            Wing {
                part: 1,
                root: Vector3::new(0.1, 0.05, 0.0),
                hinge: Vector3::new(0.0, 0.0, 1.0),
                span: Vector3::new(1.0, 0.0, 0.0),
            },
            Wing {
                part: 2,
                root: Vector3::new(-0.1, 0.05, 0.0),
                hinge: Vector3::new(0.0, 0.0, 1.0),
                span: Vector3::new(-1.0, 0.0, 0.0),
            },
        ],
        raise: 0.8,
        lower: 0.5,
        feather: 0.2,
        glide_rate: 1.5,
        hard_rate: 4.0,
    };

    const FLYER: Flyer = Flyer {
        mass: 10.0,
        yaw_inertia: 1.0,
        speed: 6.0,
        accel_time: 1.0,
        cruise_height: 5.0,
        wings: WINGS,
        beat_phase: 0.0,
    };

    /// Fly from rest for `seconds`, always `rise_to` above where it started.
    fn fly(heading: Vector3<f32>, rise_to: f32, seconds: f32) -> (Vector3<f32>, f32) {
        let dt = 1.0 / 60.0;
        let (mut velocity, mut height) = (Vector3::zeros(), 0.0);
        for _ in 0..(seconds / dt) as usize {
            velocity += steer(&FLYER, heading, velocity, rise_to - height, dt);
            height += velocity.y * dt;
        }
        (velocity, height)
    }

    /// Where the point `along` metres out a wing from its root is, `phase`
    /// through a beat.
    fn on_wing(wing: &Wing, along: f32, phase: f32) -> Vector3<f32> {
        let at_rest = wing.root + wing.span * along;
        let pose = WINGS.pose(wing, phase);
        pose.rotation * at_rest + pose.translation
    }

    #[test]
    fn it_comes_up_to_speed_along_its_heading_and_no_faster() {
        let (velocity, _) = fly(Vector3::z(), 0.0, 3.0);
        assert!((velocity.z - FLYER.speed).abs() < 0.05, "{velocity:?}");
        assert!(velocity.x.abs() < 1e-3);
    }

    #[test]
    fn it_climbs_to_the_height_it_is_given_and_holds_it() {
        let (velocity, height) = fly(Vector3::zeros(), 5.0, 6.0);
        assert!((height - 5.0).abs() < 0.1, "at {height} m");
        assert!(velocity.norm() < 0.2, "still moving at {velocity:?}");
    }

    #[test]
    fn it_comes_down_as_readily_as_it_climbs() {
        let (_, height) = fly(Vector3::zeros(), -4.0, 6.0);
        assert!((height + 4.0).abs() < 0.1, "at {height} m");
    }

    /// Stooping for a second down a line 20° off the vertical, from a hover
    /// that left it drifting across the line, a flyer is on the line and
    /// falling down it as fast as gravity takes it there, faster than it
    /// flies.
    #[test]
    fn a_stoop_falls_down_its_line_and_its_wings_keep_it_there() {
        let gravity = Vector3::new(0.0, -9.81, 0.0);
        let line = Vector3::new(20f32.to_radians().sin(), -20f32.to_radians().cos(), 0.0);
        let (dt, mut velocity) = (1.0 / 60.0, Vector3::new(0.0, 0.0, 1.0));
        for _ in 0..60 {
            velocity += stoop(&FLYER, line, velocity, dt) + gravity * dt;
        }
        let down = velocity.dot(&line);
        let across = (velocity - line * down).norm();
        assert!(across < 0.1, "{across} m/s across its line");
        assert!(
            down > 0.97 * gravity.dot(&line) && down > FLYER.speed,
            "{down} m/s"
        );
    }

    #[test]
    fn its_wings_turn_on_their_roots_up_at_the_top_of_a_beat_and_down_at_the_bottom() {
        for wing in &WINGS.wings {
            for step in 0..16 {
                let phase = step as f32 / 16.0;
                // The root never leaves the body.
                assert!((on_wing(wing, 0.0, phase) - wing.root).norm() < 1e-5);
                // A rigid wing: its tip stays a wing's length from its root.
                let reach = (on_wing(wing, 1.0, phase) - wing.root).norm();
                assert!((reach - 1.0).abs() < 1e-4, "a wing {reach} m long");
            }
            let (top, bottom) = (on_wing(wing, 1.0, 0.0), on_wing(wing, 1.0, 0.5));
            assert!((top.y - wing.root.y - WINGS.raise.sin()).abs() < 1e-4);
            assert!((bottom.y - wing.root.y + WINGS.lower.sin()).abs() < 1e-4);
            // Coming down, its leading edge is tipped down.
            let leading = wing.root + wing.span * 0.5 + Vector3::z() * 0.1;
            let pose = WINGS.pose(wing, 0.25);
            let edge = pose.rotation * leading + pose.translation;
            assert!(edge.y < on_wing(wing, 0.5, 0.25).y - 0.01);
        }
        // The two wings beat as one, each the other's mirror.
        for step in 0..8 {
            let phase = step as f32 / 8.0;
            let (right, left) = (
                on_wing(&WINGS.wings[0], 1.0, phase),
                on_wing(&WINGS.wings[1], 1.0, phase),
            );
            assert!((right.y - left.y).abs() < 1e-5 && (right.x + left.x).abs() < 1e-5);
        }
    }

    #[test]
    fn only_the_wings_are_posed_and_a_model_without_them_is_left_alone() {
        let poses = WINGS.poses(0.0, 3);
        assert!(poses[0].rotation.angle() < 1e-6 && poses[0].translation.norm() < 1e-6);
        assert!((poses[1].rotation.angle() - WINGS.raise).abs() < 1e-5);
        assert!((poses[2].rotation.angle() - WINGS.raise).abs() < 1e-5);
        // A model of one part has no wings to pose.
        assert_eq!(WINGS.poses(0.3, 1).len(), 1);
    }

    #[test]
    fn it_beats_slowly_gliding_and_faster_climbing_or_at_speed() {
        let beats_in_a_second = |velocity: Vector3<f32>| {
            let mut flyer = FLYER;
            let (dt, mut beats, mut last) = (1.0 / 240.0, 0.0, 0.0);
            for _ in 0..240 {
                flyer.beat(velocity, dt);
                beats += (flyer.beat_phase - last).rem_euclid(1.0);
                last = flyer.beat_phase;
            }
            beats
        };
        let gliding = beats_in_a_second(Vector3::zeros());
        assert!((gliding - WINGS.glide_rate).abs() < 0.02, "{gliding} beats");
        let stooping = beats_in_a_second(Vector3::new(0.0, -CLIMB_SPEED, 0.0));
        assert!((stooping - WINGS.glide_rate).abs() < 0.02);
        let climbing = beats_in_a_second(Vector3::new(0.0, CLIMB_SPEED, 0.0));
        assert!(
            (climbing - WINGS.hard_rate).abs() < 0.02,
            "{climbing} beats"
        );
        let cruising = beats_in_a_second(Vector3::new(0.0, 0.0, FLYER.speed));
        assert!((cruising - WINGS.hard_rate).abs() < 0.02);
        let ambling = beats_in_a_second(Vector3::new(0.5 * FLYER.speed, 0.0, 0.0));
        assert!(gliding < ambling && ambling < cruising);
    }
}
