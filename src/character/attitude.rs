use specs::{Component, DenseVecStorage};

use crate::physics::{ConstraintHandle, ConstraintKind, PhysicsWorld};

/// The constraint holding a character's body at its attitude, for a character
/// whose body can lie down — to swim.
///
/// A character without one is held upright by whatever its spawner gave it,
/// and swims standing up. `CharacterState::body_pitch` is the pitch asked
/// for; this is where it is handed to the engine.
#[derive(Component, Debug, Clone, Copy)]
#[storage(DenseVecStorage)]
pub struct AttitudeControl {
    /// A `ConstraintKind::KeepAttitude` on the character's body.
    pub constraint: ConstraintHandle,
}

/// How firmly the attitude is held.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AttitudeHold {
    /// Exactly, at any cost: a character in control of itself.
    Rigid,
    /// Sprung toward it, `compliance` rad per N·m: a body righting itself,
    /// which takes time and can be stopped.
    Sprung { compliance: f32 },
    /// Not at all: the body turns however the world turns it.
    Released,
}

impl AttitudeControl {
    /// Hold the body at `pitch` from now on, as firmly as `hold` says. A
    /// constraint that has been removed — a corpse's — is left alone.
    pub fn hold(&self, physics: &mut PhysicsWorld, pitch: f32, hold: AttitudeHold) {
        let Some(constraint) = physics.constraint_mut(self.constraint) else {
            return;
        };
        if let ConstraintKind::KeepAttitude {
            pitch: held,
            compliance,
            max_impulse,
            ..
        } = &mut constraint.kind
        {
            *held = pitch;
            (*compliance, *max_impulse) = match hold {
                AttitudeHold::Rigid => (0.0, f32::INFINITY),
                AttitudeHold::Sprung { compliance } => (compliance, f32::INFINITY),
                AttitudeHold::Released => (0.0, 0.0),
            };
        }
    }
}

/// Move `current` toward `target` at no more than `rate` per second.
pub fn approach(current: f32, target: f32, rate: f32, dt: f32) -> f32 {
    let step = rate * dt;
    current + (target - current).clamp(-step, step)
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use nalgebra::{Point3, UnitQuaternion, Vector3};

    use super::*;
    use crate::app::spawners::create_player_body;
    use crate::character::LocomotionConfig;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::{PhysicsConfig, SequentialStepper, Stepper};

    const FRAME_DT: f32 = 1.0 / 60.0;

    /// The player's body laid on flat ground turned by `lying`, held at
    /// `hold` and the pitch `pitch` gives each frame: its rotation and
    /// centre after each of `frames` frames.
    fn held(
        lying: UnitQuaternion<f32>,
        hold: AttitudeHold,
        pitch: impl Fn(usize) -> f32,
        frames: usize,
    ) -> Vec<(UnitQuaternion<f32>, Point3<f32>)> {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let locomotion = LocomotionConfig::player();
        let at = Point3::new(3.0, locomotion.collider_radius + 0.01, 1.0);
        let (body, constraint) = create_player_body(&mut world, at, &locomotion);
        world.body_mut(body).unwrap().set_rotation(lying);
        world.wake_body(body);
        let control = AttitudeControl { constraint };
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        let ground = FlatQuadGeometry::new(50.0);
        let mut debug = DebugLines::default();
        (0..frames)
            .map(|frame| {
                control.hold(&mut world, pitch(frame), hold);
                stepper.step(&mut world, FRAME_DT, &ground, &[], &[], &mut debug);
                let body = world.body(body).unwrap();
                (body.rotation(), body.position())
            })
            .collect()
    }

    /// How far a body turned by `rotation` leans from upright, rad.
    fn lean(rotation: &UnitQuaternion<f32>) -> f32 {
        (rotation * Vector3::y()).y.clamp(-1.0, 1.0).acos()
    }

    fn on_its_side() -> UnitQuaternion<f32> {
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.5)
    }

    /// Let go, a body lying down stays down.
    #[test]
    fn a_released_body_lies_where_it_fell() {
        let held = held(on_its_side(), AttitudeHold::Released, |_| 0.0, 60);
        let lean = lean(&held[59].0);
        assert!(lean > 1.3, "it rose to {lean}");
    }

    /// The player's body lying along +z on its front (`face_up` false) or
    /// its back, rolled a further `roll` about its length.
    fn lying(face_up: bool, roll: f32) -> UnitQuaternion<f32> {
        let pitch = if face_up { -FRAC_PI_2 } else { FRAC_PI_2 };
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pitch)
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), roll)
    }

    /// Sprung lying along its heading, as a knocked-down body gathering
    /// itself is, a body lying nearly on its side turns onto its front or
    /// its back within `gather_time`, still lying, rocks back no more than a
    /// quarter of the way, and rolls no further than about its width.
    #[test]
    fn a_body_held_lying_turns_from_its_side_onto_its_front_or_back() {
        let config = LocomotionConfig::player();
        let compliance = config.down.gather_compliance;
        let frames = (config.down.gather_time / FRAME_DT).round() as usize;
        for face_up in [false, true] {
            let pitch = if face_up { -FRAC_PI_2 } else { FRAC_PI_2 };
            let start = lying(face_up, 80f32.to_radians());
            let held = held(
                start,
                AttitudeHold::Sprung { compliance },
                |_| pitch,
                2 * frames,
            );
            let off_face = |rotation: &UnitQuaternion<f32>| {
                let belly = (rotation * Vector3::z()).y;
                let facing = if face_up { belly } else { -belly };
                facing.clamp(-1.0, 1.0).acos()
            };
            let (rotation, centre) = held[frames - 1];
            assert!(
                off_face(&rotation) < 0.25,
                "face up {face_up}: {:.2} rad off its face",
                off_face(&rotation)
            );
            let rocked = held[frames..]
                .iter()
                .map(|(rotation, _)| off_face(rotation))
                .fold(0.0, f32::max);
            assert!(rocked < 0.45, "face up {face_up}: rocked {rocked:.2} rad");
            assert!(lean(&rotation) > 1.4, "face up {face_up}: rose");
            let rolled = (centre.x - 3.0).abs();
            assert!(rolled < 0.5, "face up {face_up}: rolled {rolled:.2} m");
        }
    }

    /// Sprung as a knocked-down body getting up is, pitched toward standing
    /// at `rise_rate` from lying on its front or its back, it stands within
    /// `rise_limit`, and does not pop up off its feet on the way.
    #[test]
    fn a_body_pitched_up_from_lying_stands_within_its_rise_limit() {
        let config = LocomotionConfig::player();
        let down = config.down;
        let frames = (down.rise_limit / FRAME_DT).round() as usize;
        for face_up in [false, true] {
            let lying_pitch = if face_up { -FRAC_PI_2 } else { FRAC_PI_2 };
            let step = down.rise_rate * FRAME_DT;
            let pitch = |frame: usize| {
                let rose = step * frame as f32;
                lying_pitch.signum() * (FRAC_PI_2 - rose).max(0.0)
            };
            let held = held(
                lying(face_up, 0.0),
                AttitudeHold::Sprung {
                    compliance: down.rise_compliance,
                },
                pitch,
                frames,
            );
            let stood = held
                .iter()
                .position(|(rotation, _)| lean(rotation) <= down.upright_tilt)
                .unwrap_or_else(|| {
                    panic!(
                        "face up {face_up}: still leaning {:.2}",
                        lean(&held[frames - 1].0)
                    )
                });
            let highest = held[..=stood]
                .iter()
                .map(|(_, centre)| centre.y)
                .fold(0.0, f32::max);
            assert!(
                highest <= config.collider_half_height + 0.03,
                "face up {face_up}: popped up to {highest}"
            );
        }
    }
}
