//! The push a pump's body takes from the water it throws.
//!
//! ```text
//!   spout: Q m³/s leaving at v  ──▶  the body is pushed -ρ·Q·v, at the spout
//! ```
//!
//! A pump standing welded only presses on its weld; one lying loose is
//! shoved along the bed, or spun, by its own jets. The push is a force on
//! the physics step, as buoyancy is, so it turns with the body within a
//! frame and leaves a body at rest under it asleep.

use nalgebra::{Point3, Vector3};
use specs::Entity;

use super::component::WaterPump;
use super::system::pump_id;
use crate::physics::{ForceContext, ForceOutput, RigidBodyHandle, SubstepForceProvider};
use crate::water::buoyancy::FLUID_DENSITY;
use crate::water::WaterWorld;

/// One spout's push on its body, in the body's frame.
#[derive(Debug, Clone, Copy)]
struct Push {
    /// Where it acts, from the body's origin, m.
    at: Point3<f32>,
    /// N.
    force: Vector3<f32>,
}

/// Every running pump's recoil on its body, for one frame's physics.
#[derive(Debug, Default)]
pub struct PumpRecoil {
    bodies: Vec<RigidBodyHandle>,
    /// Each body's pushes, in the order of `bodies`.
    pushes: Vec<Vec<Push>>,
}

impl PumpRecoil {
    /// The recoil of `pumps`, each on its entity's body, from what each
    /// spout moved over the water's last step.
    pub fn of<'a>(
        pumps: impl IntoIterator<Item = (Entity, &'a WaterPump, RigidBodyHandle)>,
        water: &WaterWorld,
    ) -> Self {
        let mut recoil = Self::default();
        for (entity, pump, body) in pumps {
            let pushes: Vec<Push> = pump
                .spouts
                .iter()
                .enumerate()
                .filter_map(|(index, spout)| {
                    let discharge = water.field().pump_discharge(pump_id(entity, index))?;
                    (discharge > 0.0).then(|| Push {
                        at: spout.outlet,
                        force: -spout.velocity * (FLUID_DENSITY * discharge),
                    })
                })
                .collect();
            if !pushes.is_empty() {
                recoil.bodies.push(body);
                recoil.pushes.push(pushes);
            }
        }
        recoil
    }

    /// Whether no pump is running.
    pub fn is_idle(&self) -> bool {
        self.bodies.is_empty()
    }
}

impl SubstepForceProvider for PumpRecoil {
    fn affected_bodies(&self) -> &[RigidBodyHandle] {
        &self.bodies
    }

    fn compute_force(&self, handle: RigidBodyHandle, ctx: &ForceContext) -> ForceOutput {
        let mut output = ForceOutput::zero();
        let (Some(index), Some(body)) = (
            self.bodies.iter().position(|body| *body == handle),
            ctx.bodies.get(handle.0),
        ) else {
            return output;
        };
        let turn = body.rotation();
        for push in &self.pushes[index] {
            let force = turn * push.force;
            output.force += force;
            output.torque += (turn * push.at.coords).cross(&force);
        }
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::EmptyGeometry;
    use crate::physics::{
        Atmosphere, ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc, SequentialStepper,
        Stepper,
    };

    /// A ball in empty, weightless space with one spout pushing it, after
    /// a second: its mass, velocity and spin.
    fn pushed(push: Push) -> (f32, Vector3<f32>, Vector3<f32>) {
        let mut config = PhysicsConfig {
            gravity: Vector3::zeros(),
            atmosphere: Atmosphere::VACUUM,
            ..PhysicsConfig::default()
        };
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);
        let ball = world.create_body(RigidBodyDesc::dynamic().position(Point3::origin()));
        world.attach_collider(ball, ColliderDesc::sphere(0.5).density(1000.0));
        world.wake_body(ball);
        let recoil = PumpRecoil {
            bodies: vec![ball],
            pushes: vec![vec![push]],
        };
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        for _ in 0..60 {
            stepper.step(
                &mut world,
                1.0 / 60.0,
                &EmptyGeometry,
                &[],
                &[&recoil],
                &mut DebugLines::default(),
            );
        }
        let body = world.body(ball).unwrap();
        (body.mass(), body.linear_velocity(), body.angular_velocity())
    }

    #[test]
    fn a_spout_through_the_centre_pushes_its_body_straight_back() {
        // 0.1 m³/s thrown up at 4 m/s: 400 N down.
        let (mass, velocity, spin) = pushed(Push {
            at: Point3::new(0.0, 0.5, 0.0),
            force: Vector3::new(0.0, -400.0, 0.0),
        });
        assert!((velocity.y + 400.0 / mass).abs() < 0.01 * 400.0 / mass);
        assert!(spin.norm() < 1.0e-4, "it spun: {spin:?}");
    }

    #[test]
    fn a_spout_off_to_one_side_spins_its_body() {
        let (_, _, spin) = pushed(Push {
            at: Point3::new(0.5, 0.0, 0.0),
            force: Vector3::new(0.0, -400.0, 0.0),
        });
        assert!(spin.z < -0.1, "spin {spin:?}");
    }

    #[test]
    fn the_push_turns_with_the_body() {
        // Pushed off-centre the ball turns, and the push with it, so it
        // does not gather all its speed along the way it was first pushed.
        let (mass, velocity, _) = pushed(Push {
            at: Point3::new(0.5, 0.0, 0.0),
            force: Vector3::new(0.0, -400.0, 0.0),
        });
        assert!(
            velocity.x.abs() > 0.02 * 400.0 / mass,
            "velocity {velocity:?}"
        );
    }
}
