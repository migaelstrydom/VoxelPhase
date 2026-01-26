//! Collision detection and response systems.

use nalgebra::{Point3, Vector3};
use specs::{Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::collision::ContactPoint;
use crate::components::{Collider, PhysicsBody, Position, Velocity};
use crate::terrain::TerrainManager;
use crate::{biped::BipedController, debug::DebugLines};

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
        ReadStorage<'a, PhysicsBody>,
        ReadStorage<'a, BipedController>,
    );

    fn run(
        &mut self,
        (
            terrain_manager_opt,
            entities,
            positions,
            colliders,
            mut velocities,
            physics_bodies,
            biped_controllers,
        ): Self::SystemData,
    ) {
        // Skip if no terrain loaded
        let Some(ref terrain_manager) = terrain_manager_opt else {
            return;
        };

        for (entity, pos, collider, vel) in
            (&entities, &positions, &colliders, &mut velocities).join()
        {
            if biped_controllers.get(entity).is_some() {
                continue;
            }
            let center = Point3::new(pos.0.x, pos.0.y, pos.0.z);
            let radius = collider.shape.radius;
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
        Entities<'a>,
        WriteStorage<'a, Position>,
        ReadStorage<'a, Collider>,
        ReadStorage<'a, BipedController>,
        Write<'a, DebugLines>,
    );

    fn run(
        &mut self,
        (
            terrain_manager_opt,
            entities,
            mut positions,
            colliders,
            biped_controllers,
            mut _debug_lines,
        ): Self::SystemData,
    ) {
        let Some(ref terrain_manager) = terrain_manager_opt else {
            return;
        };

        // Multiple iterations to resolve deep penetrations
        const MAX_ITERATIONS: usize = 4;

        for (entity, pos, collider) in (&entities, &mut positions, &colliders).join() {
            // The biped should be fully kinematic, so nothing should be pushing it into the terrain.
            if biped_controllers.get(entity).is_some() {
                continue;
            }
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
