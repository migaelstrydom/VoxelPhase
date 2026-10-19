//! Thrusts each hover sled the way its riders lean, holds its heading under
//! them, and coasts it when none ride.

use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::character::{CharacterIntent, Grounding};
use crate::components::RigidBodyComponent;
use crate::systems::PhysicsResource;
use crate::time::Time;

use super::components::HoverSled;

/// Least change of speed worth an impulse, m/s: below it a sled at rest is
/// left to sleep.
const LEAST_CHANGE: f32 = 1e-3;

/// Runs before `PhysicsSyncSystem` steps the world, so a frame's thrust is
/// in the step it was asked for. It pushes the sled as its thrusters do,
/// level and through its centre of mass, and leaves its riders to be carried
/// by what they stand on.
pub struct HoverSledSystem;

impl<'a> System<'a> for HoverSledSystem {
    type SystemData = (
        Read<'a, Time>,
        Write<'a, PhysicsResource>,
        WriteStorage<'a, HoverSled>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, Grounding>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, mut physics, mut sleds, bodies, intents, groundings) = data;
        let dt = time.delta_seconds();
        let world = &mut physics.world;

        let riders: Vec<_> = (&intents, &groundings, &bodies)
            .join()
            .filter(|(_, grounding, _)| grounding.is_grounded)
            .filter_map(|(_, _, body)| world.body(body.0).map(|b| (b.position(), b.mass())))
            .collect();

        for (sled, body) in (&mut sleds, &bodies).join() {
            let Some(hull) = world.body(body.0) else {
                continue;
            };
            let rotation = hull.rotation();
            let to_local = rotation.inverse();
            let (forward, across) = (rotation * Vector3::x(), rotation * Vector3::z());

            let mut lean = Vector3::zeros();
            let mut mass = hull.mass();
            let mut carried = 0;
            for (position, rider_mass) in &riders {
                let local = Point3::from(to_local * (position - hull.position()));
                if let Some(stance) = sled.stance(&local) {
                    lean += sled.lean(stance, &forward, &across);
                    mass += rider_mass;
                    carried += 1;
                }
            }
            let lean = (carried > 0).then(|| lean / carried as f32);

            let heading = HoverSled::heading(&forward);
            sled.held_heading = (carried > 0).then(|| sled.held_heading.unwrap_or(heading));
            let yaw_hold = sled
                .held_heading
                .zip(hull.world_inv_inertia().try_inverse())
                .map(|(held, inertia)| {
                    let error = wrap_angle(heading - held);
                    let yaw = sled
                        .glide
                        .yaw_rate_change(hull.angular_velocity().y, error, dt);
                    inertia * (Vector3::y() * yaw)
                });
            let velocity = hull.linear_velocity();
            let level = Vector3::new(velocity.x, 0.0, velocity.z);
            let change = sled.glide.velocity_change(level, lean, dt);
            if change.norm() > LEAST_CHANGE {
                world.apply_impulse(body.0, change * mass);
            }
            if let Some(angular) = yaw_hold {
                world.apply_angular_impulse(body.0, angular);
            }
        }
    }
}

/// `angle` brought into −π..π.
fn wrap_angle(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}
