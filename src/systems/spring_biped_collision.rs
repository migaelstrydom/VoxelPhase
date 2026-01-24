//! Terrain collision resolution for spring biped skeletons.
//!
//! This system is authoritative for biped Position updates. It uses MotionState
//! to sweep from prev to predicted position and resolves collisions against terrain.

use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::collision::{resolve_terrain_sweep, SweptContact};
use crate::components::{MotionState, PelvisTarget, Position, Velocity};
use crate::debug::DebugLines;
use crate::skeleton::SpringBipedCharacter;
use crate::terrain::TerrainManager;

/// Resolves spring biped collisions against terrain using CCD.
///
/// Uses MotionState.prev → MotionState.predicted sweep for continuous collision detection.
/// This system is authoritative for Position updates of biped entities.
pub struct SpringBipedCollisionSystem;

impl<'a> System<'a> for SpringBipedCollisionSystem {
    type SystemData = (
        Option<Read<'a, TerrainManager>>,
        ReadStorage<'a, MotionState>,
        ReadStorage<'a, PelvisTarget>,
        WriteStorage<'a, SpringBipedCharacter>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        Write<'a, DebugLines>,
    );

    fn run(
        &mut self,
        (
            terrain_manager_opt,
            motion_states,
            pelvis_targets,
            mut characters,
            mut positions,
            mut velocities,
            mut debug_lines,
        ): Self::SystemData,
    ) {
        let Some(ref terrain_manager) = terrain_manager_opt else {
            return;
        };

        for (motion, pelvis_target, character, pos, vel) in (
            &motion_states,
            &pelvis_targets,
            &mut characters,
            &mut positions,
            &mut velocities,
        )
            .join()
        {
            let radius = character.skeleton.config.pelvis_radius;

            // Primary CCD: sweep from prev position to predicted position
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
            let mut contact_opt = sweep_result.contact;

            // Resting contact probe: short downward sweep to maintain ground contact
            // when velocity is low (prevents floating when stationary).
            if contact_opt.is_none() {
                let probe_end = Point3::new(resolved_pos.x, resolved_pos.y - 0.25, resolved_pos.z);
                if let Some(probe_contact) =
                    terrain_manager.query_swept_sphere(resolved_pos, probe_end, radius)
                {
                    resolved_pos = probe_contact.point + probe_contact.normal * radius;
                    contact_opt = Some(probe_contact);
                }
            }

            // Penetration resolution: if already overlapping terrain, push out
            if contact_opt.is_none() {
                let contacts = terrain_manager.query_sphere_collision(resolved_pos, radius);
                if let Some(deepest) = contacts
                    .iter()
                    .max_by(|a, b| a.depth.partial_cmp(&b.depth).unwrap())
                {
                    resolved_pos += deepest.normal * deepest.depth;
                    contact_opt = Some(SweptContact::new(0.0, resolved_pos, deepest.normal));
                }
            }

            // Update grounded state from contact (pelvis collision).
            let mut grounded_from_contact = false;
            if let Some(ref contact) = contact_opt {
                // Cancel velocity into surface
                let vel_into_surface = resolved_vel.dot(&contact.normal);
                if vel_into_surface < 0.0 {
                    resolved_vel -= contact.normal * vel_into_surface;
                }

                // Ground detection: normal.y > 0.5 means surface is walkable (~60° max slope)
                grounded_from_contact = contact.normal.y > 0.5;
                character.ground_normal = if grounded_from_contact {
                    Some(contact.normal)
                } else {
                    None
                };

                debug_lines.add("Collision", "true");
            }

            // Support from IK targets: drive pelvis toward target height when we have contact.
            let supported = pelvis_target.has_contact && resolved_vel.y <= 0.1;

            if supported {
                resolved_pos.y = pelvis_target.target_y;
                if resolved_vel.y < 0.0 {
                    resolved_vel.y = 0.0;
                }
            }

            character.grounded = grounded_from_contact || supported;
            if !character.grounded {
                character.ground_normal = None;
            }

            // Commit resolved position and velocity
            pos.0 = Vector3::new(resolved_pos.x, resolved_pos.y, resolved_pos.z);
            vel.0 = resolved_vel;

            // Sync skeleton pelvis from resolved position
            character.set_pelvis_position(resolved_pos);

            // Resolve joint collisions (feet, knees) against terrain
            let query = character.get_collision_aabb();
            let triangles = terrain_manager.query_triangles(&query);
            let _ = character.resolve_collisions(&triangles);

            character.mark_dirty();
        }
    }
}
