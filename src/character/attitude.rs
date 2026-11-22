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
    use nalgebra::{Point3, UnitQuaternion, Vector3};

    use super::*;
    use crate::app::spawners::create_player_body;
    use crate::character::LocomotionConfig;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::{PhysicsConfig, SequentialStepper, Stepper};

    const FRAME_DT: f32 = 1.0 / 60.0;

    /// Lean, rad, and the height of the centre, m, of the player's body laid
    /// on its side on flat ground and held at `hold`, for each of `frames`
    /// frames.
    fn lean_held(hold: AttitudeHold, frames: usize) -> Vec<(f32, f32)> {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let locomotion = LocomotionConfig::player();
        let lying = Point3::new(3.0, locomotion.collider_radius + 0.01, 1.0);
        let (body, constraint) = create_player_body(&mut world, lying, &locomotion);
        let on_its_side = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.5);
        world.body_mut(body).unwrap().set_rotation(on_its_side);
        world.wake_body(body);
        let control = AttitudeControl { constraint };
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        let ground = FlatQuadGeometry::new(50.0);
        let mut debug = DebugLines::default();
        (0..frames)
            .map(|_| {
                control.hold(&mut world, 0.0, hold);
                stepper.step(&mut world, FRAME_DT, &ground, &[], &[], &mut debug);
                let body = world.body(body).unwrap();
                let up = body.rotation() * Vector3::y();
                (up.y.clamp(-1.0, 1.0).acos(), body.position().y)
            })
            .collect()
    }

    /// Let go, a body lying down stays down.
    #[test]
    fn a_released_body_lies_where_it_fell() {
        let (lean, _) = lean_held(AttitudeHold::Released, 60)[59];
        assert!(lean > 1.3, "it rose to {lean}");
    }

    /// Sprung as a knocked-down body getting up is, it stands within a
    /// second, and is not flung off the ground on the way.
    #[test]
    fn a_sprung_body_gets_up_within_a_second() {
        let config = LocomotionConfig::player();
        let compliance = config.down.rise_compliance;
        let held = lean_held(AttitudeHold::Sprung { compliance }, 60);
        let stood = held
            .iter()
            .position(|&(lean, _)| lean <= config.down.upright_tilt)
            .unwrap_or_else(|| panic!("still leaning {:.2} after a second", held[59].0));
        let highest = held[..=stood].iter().map(|&(_, y)| y).fold(0.0, f32::max);
        assert!(
            highest <= config.collider_half_height + 0.1,
            "flung to {highest}"
        );
    }
}
