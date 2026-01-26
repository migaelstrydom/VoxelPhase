//! Terrain collision for dynamic (non-biped) entities.
//!
//! This system handles CCD collision resolution for entities like grenades,
//! boxes, and other physics objects that have Collider + MotionState but are
//! not biped characters.

use nalgebra::Vector3;
use specs::{Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::collision::resolve_terrain_sweep;
use crate::components::{Collider, MotionState, PhysicsBody, Position, Velocity};
use crate::terrain::TerrainManager;
use crate::{biped::BipedController, debug::DebugLines};

/// Resolves dynamic entity collisions against terrain using CCD.
///
/// Uses MotionState.prev → MotionState.predicted sweep for continuous collision detection.
/// Applies physics response (bounce, friction) based on PhysicsBody properties.
/// This system is authoritative for Position updates of non-biped dynamic entities.
pub struct DynamicTerrainCollisionSystem;

impl<'a> System<'a> for DynamicTerrainCollisionSystem {
    type SystemData = (
        Option<Read<'a, TerrainManager>>,
        Entities<'a>,
        ReadStorage<'a, MotionState>,
        ReadStorage<'a, Collider>,
        ReadStorage<'a, PhysicsBody>,
        ReadStorage<'a, BipedController>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        Write<'a, DebugLines>,
    );

    fn run(
        &mut self,
        (
            terrain_manager_opt,
            entities,
            motion_states,
            colliders,
            physics_bodies,
            biped_controllers,
            mut positions,
            mut velocities,
            mut _debug_lines,
        ): Self::SystemData,
    ) {
        let Some(ref terrain_manager) = terrain_manager_opt else {
            return;
        };

        for (entity, motion, collider, pos, vel) in (
            &entities,
            &motion_states,
            &colliders,
            &mut positions,
            &mut velocities,
        )
            .join()
        {
            // Skip biped entities - they're handled by SpringBipedCollisionSystem
            if biped_controllers.get(entity).is_some() {
                continue;
            }

            let radius = collider.shape.radius;

            // CCD sweep from prev to predicted
            let sweep_result = resolve_terrain_sweep(
                motion.prev,
                motion.predicted,
                vel.0,
                radius,
                terrain_manager,
                3,
            );

            let mut resolved_pos = sweep_result.position;
            let mut resolved_vel = sweep_result.velocity;

            // Apply physics response if we hit something
            if let Some(ref contact) = sweep_result.contact {
                let (restitution, friction) = physics_bodies
                    .get(entity)
                    .map(|pb| (pb.restitution, pb.friction))
                    .unwrap_or((0.2, 0.5));

                // Apply bounce using original velocity for magnitude calculation.
                // The sweep already removed the into-surface component from resolved_vel,
                // so we only need to add the bounce impulse.
                let vel_into_surface = vel.0.dot(&contact.normal);
                if vel_into_surface < 0.0 {
                    resolved_vel += contact.normal * (-vel_into_surface * restitution);
                }

                // Apply friction to tangential velocity
                let tangent_vel = resolved_vel - contact.normal * resolved_vel.dot(&contact.normal);
                if tangent_vel.magnitude_squared() > 1e-6 {
                    let friction_impulse = tangent_vel.magnitude() * friction;
                    let tangent_dir = tangent_vel.normalize();
                    resolved_vel -= tangent_dir * friction_impulse.min(tangent_vel.magnitude());
                }
            }

            // Penetration resolution: if overlapping, push out
            let contacts = terrain_manager.query_sphere_collision(resolved_pos, radius);
            if let Some(deepest) = contacts
                .iter()
                .max_by(|a, b| a.depth.partial_cmp(&b.depth).unwrap())
            {
                if deepest.depth > 0.001 {
                    resolved_pos += deepest.normal * (deepest.depth + 0.001);
                }
            }

            // Commit resolved position and velocity
            pos.0 = Vector3::new(resolved_pos.x, resolved_pos.y, resolved_pos.z);
            vel.0 = resolved_vel;
        }
    }
}
