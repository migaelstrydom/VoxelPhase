//! Collision detection and response systems.

use nalgebra::{Point3, Vector3};
use specs::{Entities, Join, Read, ReadStorage, System, WriteStorage};

use crate::collision::ContactPoint;
use crate::components::{Collider, OnGround, PhysicsBody, Position, Velocity};
use crate::terrain::TerrainManager;

/// Small distance for ground probing (how far below the player we check for ground)
const GROUND_PROBE_DISTANCE: f32 = 0.1;

/// System that detects collisions between entities and terrain,
/// applies velocity response, and updates ground state.
///
/// Uses a "ground probe" - a short downward sweep to detect if ground is nearby,
/// since swept collision keeps entities slightly above surfaces.
pub struct TerrainCollisionSystem;

impl<'a> System<'a> for TerrainCollisionSystem {
    type SystemData = (
        Option<Read<'a, TerrainManager>>,
        Entities<'a>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Collider>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, OnGround>,
        ReadStorage<'a, PhysicsBody>,
    );

    fn run(
        &mut self,
        (
            terrain_manager_opt,
            entities,
            positions,
            colliders,
            mut velocities,
            mut on_grounds,
            physics_bodies,
        ): Self::SystemData,
    ) {
        // Skip if no terrain loaded
        let Some(ref terrain_manager) = terrain_manager_opt else {
            return;
        };

        for (entity, pos, collider, vel) in
            (&entities, &positions, &colliders, &mut velocities).join()
        {
            let center = Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let radius = collider.shape.radius;

            // Ground detection using a downward probe
            // This detects ground even when swept collision keeps us slightly above
            let probe_start = center;
            let probe_end = Point3::new(center.x, center.y - GROUND_PROBE_DISTANCE, center.z);

            let ground_contact = if let Some(contact) =
                terrain_manager.query_swept_sphere(probe_start, probe_end, radius)
            {
                // Check if the contact normal points mostly upward (it's ground, not a wall)
                contact.normal.y > 0.5
            } else {
                false
            };

            let ground_normal = if ground_contact {
                // Do a regular collision check to get the actual ground normal
                let contacts = terrain_manager.query_sphere_collision(
                    Point3::new(center.x, center.y - GROUND_PROBE_DISTANCE * 0.5, center.z),
                    radius,
                );
                contacts
                    .iter()
                    .filter(|c| c.normal.y > 0.5)
                    .fold(Vector3::zeros(), |acc, c| acc + c.normal)
            } else {
                Vector3::zeros()
            };

            // Update ground state
            if let Some(og) = on_grounds.get_mut(entity) {
                og.grounded = ground_contact;
                og.ground_normal = if ground_contact && ground_normal.magnitude_squared() > 1e-6 {
                    Some(ground_normal.normalize())
                } else if ground_contact {
                    Some(Vector3::new(0.0, 1.0, 0.0)) // Default up normal
                } else {
                    None
                };
            }

            // Also check for any current penetration contacts and respond
            let contacts = terrain_manager.query_sphere_collision(center, radius);
            if !contacts.is_empty() {
                let (restitution, friction) = physics_bodies
                    .get(entity)
                    .map(|pb| (pb.restitution, pb.friction))
                    .unwrap_or((0.2, 0.5));

                apply_collision_response(&mut vel.0, &contacts, restitution, friction);
            }
        }
    }
}

/// Apply collision response to velocity based on contacts.
fn apply_collision_response(
    velocity: &mut Vector3<f32>,
    contacts: &[ContactPoint],
    restitution: f32,
    friction: f32,
) {
    if contacts.is_empty() {
        return;
    }

    // Find the deepest contact for primary response
    let deepest = contacts
        .iter()
        .max_by(|a, b| a.depth.partial_cmp(&b.depth).unwrap())
        .unwrap();

    let normal = deepest.normal;

    // Velocity component into the surface
    let vel_into_surface = velocity.dot(&normal);

    // Only respond if moving into the surface
    if vel_into_surface < 0.0 {
        // Remove velocity into surface and apply restitution for bounce
        let impulse = -(1.0 + restitution) * vel_into_surface;
        *velocity += normal * impulse;

        // Apply friction to tangential velocity
        let tangent_vel = *velocity - normal * velocity.dot(&normal);
        if tangent_vel.magnitude_squared() > 1e-6 {
            let friction_impulse = tangent_vel.magnitude() * friction;
            let tangent_dir = tangent_vel.normalize();
            *velocity -= tangent_dir * friction_impulse.min(tangent_vel.magnitude());
        }
    }
}

/// System that corrects entity positions to resolve penetration.
pub struct PenetrationResolutionSystem;

impl<'a> System<'a> for PenetrationResolutionSystem {
    type SystemData = (
        Option<Read<'a, TerrainManager>>,
        WriteStorage<'a, Position>,
        ReadStorage<'a, Collider>,
    );

    fn run(&mut self, (terrain_manager_opt, mut positions, colliders): Self::SystemData) {
        let Some(ref terrain_manager) = terrain_manager_opt else {
            return;
        };

        // Multiple iterations to resolve deep penetrations
        const MAX_ITERATIONS: usize = 4;

        for (pos, collider) in (&mut positions, &colliders).join() {
            for _ in 0..MAX_ITERATIONS {
                let center = Point3::new(pos.0.x, pos.0.y, pos.0.z);
                let radius = collider.shape.radius;

                let contacts = terrain_manager.query_sphere_collision(center, radius);

                if contacts.is_empty() {
                    break;
                }

                // Find deepest penetration
                let deepest = contacts
                    .iter()
                    .max_by(|a, b| a.depth.partial_cmp(&b.depth).unwrap())
                    .unwrap();

                // Push out of collision
                if deepest.depth > 0.001 {
                    pos.0 += deepest.normal * (deepest.depth + 0.001);
                } else {
                    break;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_collision_response_into_ground() {
        let mut velocity = Vector3::new(5.0, -10.0, 0.0);
        let contacts = vec![ContactPoint::new(
            Point3::origin(),
            Vector3::new(0.0, 1.0, 0.0), // Ground normal pointing up
            0.1,
        )];

        apply_collision_response(&mut velocity, &contacts, 0.0, 0.0);

        // Y velocity should be zeroed (no bounce with restitution 0)
        assert!(velocity.y >= 0.0);
        // X velocity preserved (no friction)
        assert!((velocity.x - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_collision_response_bounce() {
        let mut velocity = Vector3::new(0.0, -10.0, 0.0);
        let contacts = vec![ContactPoint::new(
            Point3::origin(),
            Vector3::new(0.0, 1.0, 0.0),
            0.1,
        )];

        apply_collision_response(&mut velocity, &contacts, 1.0, 0.0);

        // Should bounce back with full velocity
        assert!((velocity.y - 10.0).abs() < 0.01);
    }
}
