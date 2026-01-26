//! Terrain collision resolution for biped characters.
//!
//! This system is authoritative for biped Position updates. It uses MotionState
//! to sweep from prev to predicted position and resolves collisions against terrain.
//! Grounding is determined by foot contact with terrain, not pelvis contact.

use nalgebra::Vector3;
use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::biped::BipedController;
use crate::collision::resolve_terrain_sweep;
use crate::components::{MotionState, Position, Velocity};
use crate::debug::DebugLines;
use crate::terrain::TerrainManager;

/// Resolves biped collisions against terrain using CCD.
///
/// Uses MotionState.prev → MotionState.predicted sweep for continuous collision detection.
/// This system is authoritative for Position updates of biped entities.
/// Grounding state is determined by whether either foot contacts the terrain.
pub struct SpringBipedCollisionSystem;

impl<'a> System<'a> for SpringBipedCollisionSystem {
    type SystemData = (
        Option<Read<'a, TerrainManager>>,
        ReadStorage<'a, MotionState>,
        WriteStorage<'a, BipedController>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        Write<'a, DebugLines>,
    );

    fn run(
        &mut self,
        (
            terrain_manager_opt,
            motion_states,
            mut controllers,
            mut positions,
            mut velocities,
            mut debug_lines,
        ): Self::SystemData,
    ) {
        let Some(ref terrain_manager) = terrain_manager_opt else {
            return;
        };

        for (motion, controller, pos, vel) in (
            &motion_states,
            &mut controllers,
            &mut positions,
            &mut velocities,
        )
            .join()
        {
            let body_radius = controller.config.body_radius;

            // Primary CCD: sweep body from prev position to predicted position
            let sweep_result = resolve_terrain_sweep(
                motion.prev,
                motion.predicted,
                vel.0,
                body_radius,
                terrain_manager,
                3,
            );

            let resolved_pos = sweep_result.position;
            let mut resolved_vel = sweep_result.velocity;
            let body_contact = sweep_result.contact;

            // Handle body collision response
            if let Some(ref contact) = body_contact {
                let vel_into_surface = resolved_vel.dot(&contact.normal);
                if vel_into_surface < 0.0 {
                    resolved_vel -= contact.normal * vel_into_surface;
                }
            }

            let is_grounded = body_contact.is_some();
            let ground_normal = body_contact.map(|c| c.normal).unwrap_or(Vector3::y());

            // Cancel velocity into ground when grounded
            if is_grounded {
                let vel_into_surface = resolved_vel.dot(&ground_normal);
                if vel_into_surface < 0.0 {
                    resolved_vel -= ground_normal * vel_into_surface;
                }
            }

            // Update controller state
            controller.state.is_grounded = is_grounded;

            debug_lines.add("Grounded", if is_grounded { "true" } else { "false" });

            // Commit resolved position and velocity
            pos.0 = Vector3::new(resolved_pos.x, resolved_pos.y, resolved_pos.z);
            vel.0 = resolved_vel;

            // Sync skeleton pelvis from resolved position
            controller.set_pelvis_position(resolved_pos);
        }
    }
}
