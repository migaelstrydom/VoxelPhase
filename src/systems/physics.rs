use crate::components::{Acceleration, Collider, Gravity, Position, Velocity};
use crate::player::PlayerConfig;
use crate::terrain::TerrainManager;
use crate::time::Time;
use nalgebra::Point3;
use specs::{Join, LendJoin, Read, ReadExpect, ReadStorage, System, WriteStorage};

/// Applies gravity to entities that have the Gravity component.
/// Sets the Y component of acceleration based on the gravity value.
pub struct GravitySystem;

impl<'a> System<'a> for GravitySystem {
    type SystemData = (ReadStorage<'a, Gravity>, WriteStorage<'a, Acceleration>);

    fn run(&mut self, (gravities, mut accelerations): Self::SystemData) {
        for (gravity, accel) in (&gravities, &mut accelerations).join() {
            // Gravity is a downward force (negative Y in our coordinate system)
            accel.0.y = -gravity.0;
        }
    }
}

/// Integrates physics: applies acceleration to velocity, and velocity to position.
/// Uses semi-implicit Euler integration with swept collision detection (CCD)
/// to prevent fast-moving objects from tunneling through terrain.
pub struct PhysicsSystem;

impl<'a> System<'a> for PhysicsSystem {
    type SystemData = (
        ReadExpect<'a, Time>,
        ReadExpect<'a, PlayerConfig>,
        Option<Read<'a, TerrainManager>>,
        ReadStorage<'a, Acceleration>,
        ReadStorage<'a, Collider>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, Position>,
    );

    fn run(
        &mut self,
        (
            time,
            config,
            terrain_manager_opt,
            accelerations,
            colliders,
            mut velocities,
            mut positions,
        ): Self::SystemData,
    ) {
        let dt = time.delta_seconds();
        let terminal_vel = config.terminal_velocity;

        // Update velocity from acceleration
        for (accel, vel) in (&accelerations, &mut velocities).join() {
            vel.0 += accel.0 * dt;

            // Clamp vertical velocity to terminal velocity
            if vel.0.y < -terminal_vel {
                vel.0.y = -terminal_vel;
            }
        }

        // Update positions with swept collision detection and surface sliding
        for (vel, pos, collider) in (&mut velocities, &mut positions, (&colliders).maybe()).join() {
            // Only do swept collision if we have terrain and a collider
            if let (Some(ref terrain_manager), Some(coll)) = (&terrain_manager_opt, collider) {
                let radius = coll.shape.radius;

                // Move with collision detection and sliding
                move_with_sliding(
                    &mut pos.0,
                    &mut vel.0,
                    dt,
                    radius,
                    terrain_manager,
                    3, // Max slide iterations
                );
            } else {
                // No terrain or no collider, just move
                pos.0 += vel.0 * dt;
            }
        }
    }
}

/// Move an entity with swept collision detection and surface sliding.
///
/// When a collision is detected, the entity slides along the surface
/// rather than stopping completely. This allows smooth movement across terrain.
fn move_with_sliding(
    position: &mut nalgebra::Vector3<f32>,
    velocity: &mut nalgebra::Vector3<f32>,
    dt: f32,
    radius: f32,
    terrain_manager: &TerrainManager,
    max_iterations: u32,
) {
    let mut remaining_time = dt;
    let epsilon = 0.001; // Small offset to prevent getting stuck

    for _ in 0..max_iterations {
        if remaining_time <= 0.0 {
            break;
        }

        let start = Point3::new(position.x, position.y, position.z);
        let delta = *velocity * remaining_time;

        // Skip if not moving significantly
        if delta.magnitude_squared() < epsilon * epsilon {
            break;
        }

        let end = Point3::new(
            position.x + delta.x,
            position.y + delta.y,
            position.z + delta.z,
        );

        // Check for collision along movement path
        if let Some(contact) = terrain_manager.query_swept_sphere(start, end, radius) {
            // Move to just before the contact point
            let safe_t = (contact.t - epsilon / delta.magnitude()).max(0.0);
            *position += delta * safe_t;

            // Calculate time consumed
            let time_used = remaining_time * contact.t;
            remaining_time -= time_used;

            // Remove velocity component going into the surface
            let vel_into_surface = velocity.dot(&contact.normal);
            if vel_into_surface < 0.0 {
                *velocity -= contact.normal * vel_into_surface;
            }

            // If velocity is now negligible, stop
            if velocity.magnitude_squared() < epsilon * epsilon {
                break;
            }
        } else {
            // No collision, move freely and we're done
            *position += delta;
            break;
        }
    }
}
