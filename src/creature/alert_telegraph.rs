use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, Join, Read, ReadStorage, System, Write, WriteStorage};

use super::brain::{Behaviour, Brain};
use crate::components::RigidBodyComponent;
use crate::damage::Dead;
use crate::systems::PhysicsResource;
use crate::time::Time;

/// Makes the moment a creature notices you legible.
///
/// [`Brain::alert_duration`] exists to give the player a beat of warning before
/// a charge, but a beat nothing expresses is just latency. A rigged creature
/// would telegraph through its animator; a featureless sphere has no face to
/// pull, so it says the same thing through the only channel it has — its body.
///
/// The tell is a hop followed by a shudder: one impulse to break the stillness,
/// then a brief shiver that reads as winding up. Both go through the physics
/// world, so the creature is genuinely disturbed rather than playing a
/// canned motion over its real position, and a roller alerted on a slope will
/// hop and drift downhill exactly as a boulder should.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct AlertTelegraph {
    /// Upward velocity of the hop, in m/s. Scaled by mass into an impulse, so
    /// this reads the same on a pebble and a boulder.
    pub hop_speed: f32,
    /// Peak angular acceleration of the shudder, in rad/s². Also mass-scaled.
    pub shudder_strength: f32,
    /// Shudder oscillations per second. Fast enough to read as a shiver rather
    /// than as the creature trying to leave.
    pub shudder_hz: f32,

    /// Seconds since the alert began, or `None` when not alerted. Also the
    /// latch: the hop fires on the transition into `Alerted` and must not
    /// re-fire while the creature stays alerted.
    elapsed: Option<f32>,
}

impl AlertTelegraph {
    /// A tell sized for a ground creature: a visible but not comic hop, and a
    /// shudder that lasts as long as the alert does.
    pub fn ground_creature() -> Self {
        Self {
            hop_speed: 2.2,
            shudder_strength: 9.0,
            shudder_hz: 11.0,
            elapsed: None,
        }
    }

    /// True while the creature is mid-tell.
    pub fn is_telegraphing(&self) -> bool {
        self.elapsed.is_some()
    }
}

/// Drives [`AlertTelegraph`] from the brain's behaviour.
///
/// Runs after `BrainSystem` so it sees the transition on the frame it happens,
/// and before the physics step so the impulse lands in the same frame.
pub struct AlertTelegraphSystem;

impl<'a> System<'a> for AlertTelegraphSystem {
    type SystemData = (
        Read<'a, Time>,
        Write<'a, PhysicsResource>,
        ReadStorage<'a, Brain>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Dead>,
        WriteStorage<'a, AlertTelegraph>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, mut physics_res, brains, rigid_bodies, deads, mut telegraphs) = data;
        let dt = time.delta_seconds();

        for (brain, rb, telegraph, _) in (&brains, &rigid_bodies, &mut telegraphs, !&deads).join() {
            if !matches!(brain.behaviour, Behaviour::Alerted { .. }) {
                telegraph.elapsed = None;
                continue;
            }

            let Some(mass) = physics_res.world.body(rb.0).map(|body| body.mass()) else {
                continue;
            };

            match telegraph.elapsed {
                // First frame of the alert: hop.
                None => {
                    telegraph.elapsed = Some(0.0);
                    physics_res
                        .world
                        .apply_impulse(rb.0, Vector3::y() * telegraph.hop_speed * mass);
                }
                // Already alerted: shiver for the rest of the beat.
                Some(elapsed) => {
                    let elapsed = elapsed + dt;
                    telegraph.elapsed = Some(elapsed);

                    // Alternating torque about a fixed horizontal axis. A
                    // rolling body has no rest orientation to return to, so the
                    // shudder has to cancel itself over a cycle or it would
                    // just be a slow drive in one direction.
                    let phase = elapsed * telegraph.shudder_hz * std::f32::consts::TAU;
                    let inertia = mass * telegraph.shudder_strength;
                    physics_res
                        .world
                        .apply_angular_impulse(rb.0, Vector3::x() * phase.sin() * inertia * dt);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn alerted() -> Brain {
        let mut brain = Brain::hunter(1.5);
        brain.behaviour = Behaviour::Alerted { remaining: 0.6 };
        brain
    }

    #[test]
    fn the_tell_is_armed_by_the_alert_and_disarmed_by_leaving_it() {
        let mut telegraph = AlertTelegraph::ground_creature();
        assert!(!telegraph.is_telegraphing());

        telegraph.elapsed = Some(0.0);
        assert!(telegraph.is_telegraphing());
    }

    #[test]
    fn the_shudder_cancels_itself_over_a_whole_cycle() {
        // Summing the drive over one full period must come back to ~zero, or
        // the shudder would be a slow push in one direction.
        let telegraph = AlertTelegraph::ground_creature();
        let period = 1.0 / telegraph.shudder_hz;
        let steps = 2000;
        let dt = period / steps as f32;

        let total: f32 = (0..steps)
            .map(|i| {
                let elapsed = i as f32 * dt;
                (elapsed * telegraph.shudder_hz * std::f32::consts::TAU).sin() * dt
            })
            .sum();

        assert!(
            total.abs() < 1e-4,
            "shudder integrated to {total}, which would drift the creature"
        );
    }

    #[test]
    fn a_hunter_starts_out_not_alerted() {
        assert!(!matches!(
            Brain::hunter(1.5).behaviour,
            Behaviour::Alerted { .. }
        ));
        assert!(matches!(alerted().behaviour, Behaviour::Alerted { .. }));
    }
}
