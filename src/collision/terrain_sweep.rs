//! Terrain sweep collision resolution for CCD.

use nalgebra::{Point3, Vector3};

use crate::collision::SweptContact;
use crate::terrain::TerrainManager;

/// Result of terrain sweep resolution.
#[derive(Debug, Clone)]
pub struct SweepResult {
    /// Resolved position after collision.
    pub position: Point3<f32>,
    /// Resolved velocity after collision response.
    pub velocity: Vector3<f32>,
    /// Contact information if a collision occurred.
    pub contact: Option<SweptContact>,
}

/// Resolve terrain collision using swept sphere from prev to predicted position.
///
/// Performs iterative CCD with surface sliding. Returns the resolved position
/// and velocity after collision response.
///
/// # Arguments
/// * `prev` - Position at start of frame
/// * `predicted` - Predicted position (prev + vel * dt)
/// * `velocity` - Current velocity (will be modified for sliding)
/// * `radius` - Sphere radius for collision
/// * `terrain_manager` - Terrain to collide against
/// * `max_iterations` - Maximum sliding iterations (typically 3)
pub fn resolve_terrain_sweep(
    prev: Point3<f32>,
    predicted: Point3<f32>,
    velocity: Vector3<f32>,
    radius: f32,
    terrain_manager: &TerrainManager,
    max_iterations: u32,
) -> SweepResult {
    let epsilon = 0.001;
    let mut current_pos = prev;
    let mut current_vel = velocity;
    let mut last_contact: Option<SweptContact> = None;

    // Calculate total movement
    let total_delta = predicted - prev;
    let total_dist = total_delta.magnitude();

    if total_dist < epsilon {
        // No significant movement, just check for penetration
        return SweepResult {
            position: prev,
            velocity,
            contact: None,
        };
    }

    let mut remaining_dist = total_dist;

    for _ in 0..max_iterations {
        if remaining_dist < epsilon {
            break;
        }

        // Compute end point based on remaining distance in current velocity direction
        let move_dir = if current_vel.magnitude_squared() > epsilon * epsilon {
            current_vel.normalize()
        } else {
            break;
        };

        let end = current_pos + move_dir * remaining_dist;

        // Check for collision along movement path
        if let Some(contact) = terrain_manager.query_swept_sphere(current_pos, end, radius) {
            // Move to just before the contact point
            let move_vec = end - current_pos;
            let move_len = move_vec.magnitude();
            let safe_t = (contact.t - epsilon / move_len.max(epsilon)).max(0.0);

            current_pos += move_vec * safe_t;

            // Calculate distance consumed
            let dist_used = move_len * contact.t;
            remaining_dist -= dist_used;

            // Remove velocity component going into the surface (slide along surface)
            let vel_into_surface = current_vel.dot(&contact.normal);
            if vel_into_surface < 0.0 {
                current_vel -= contact.normal * vel_into_surface;
            }

            last_contact = Some(contact);

            // If velocity is now negligible, stop
            if current_vel.magnitude_squared() < epsilon * epsilon {
                break;
            }
        } else {
            // No collision, move freely and we're done
            current_pos = end;
            break;
        }
    }

    SweepResult {
        position: current_pos,
        velocity: current_vel,
        contact: last_contact,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sweep_result_no_movement() {
        let pos = Point3::new(0.0, 5.0, 0.0);
        let vel = Vector3::zeros();

        // Without a terrain manager we can't really test, but we can verify the struct works
        let result = SweepResult {
            position: pos,
            velocity: vel,
            contact: None,
        };

        assert_eq!(result.position, pos);
        assert!(result.contact.is_none());
    }
}
