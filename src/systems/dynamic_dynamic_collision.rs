//! Dynamic-to-dynamic collision detection and response.
//!
//! Handles collisions between all dynamic entities (beach balls, player, etc.)
//! using CCD and mass-based impulse response.

use nalgebra::{Point3, Vector3};
use specs::{Entities, Entity, Join, ReadStorage, System, Write, WriteStorage};

use crate::biped::BipedController;
use crate::components::{Collider, MotionState, PhysicsBody, Position, Velocity};
use crate::debug::DebugLines;

/// Resolves collisions between dynamic entities using swept sphere-sphere CCD.
pub struct DynamicDynamicCollisionSystem;

/// Data for a single dynamic entity participating in collision.
struct DynamicEntity {
    entity: Entity,
    prev_pos: Point3<f32>,
    predicted_pos: Point3<f32>,
    radius: f32,
    physics: PhysicsBody,
    is_kinematic: bool,
}

/// Result of a sphere-sphere collision check.
struct CollisionResult {
    /// Collision normal (points from entity 1 to entity 2).
    normal: Vector3<f32>,
    /// Penetration depth.
    depth: f32,
}

impl<'a> System<'a> for DynamicDynamicCollisionSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Collider>,
        ReadStorage<'a, MotionState>,
        ReadStorage<'a, PhysicsBody>,
        ReadStorage<'a, BipedController>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        Write<'a, DebugLines>,
    );

    fn run(
        &mut self,
        (
            entities,
            colliders,
            motion_states,
            physics_bodies,
            biped_controllers,
            mut positions,
            mut velocities,
            mut _debug_lines,
        ): Self::SystemData,
    ) {
        // Collect all dynamic entities into a Vec for pairwise comparison
        let mut dynamics: Vec<DynamicEntity> = Vec::new();

        for (entity, collider, motion, physics) in
            (&entities, &colliders, &motion_states, &physics_bodies).join()
        {
            dynamics.push(DynamicEntity {
                entity,
                prev_pos: motion.prev,
                predicted_pos: motion.predicted,
                radius: collider.shape.radius,
                physics: *physics,
                is_kinematic: biped_controllers.get(entity).is_some(),
            });
        }

        // O(n^2) pairwise collision check
        let n = dynamics.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let d1 = &dynamics[i];
                let d2 = &dynamics[j];
                _debug_lines.add("Collision", "true");

                if let Some(collision) = check_sphere_sphere_collision(d1, d2) {
                    apply_collision_response(
                        d1.entity,
                        d2.entity,
                        &collision,
                        &d1.physics,
                        &d2.physics,
                        d1.is_kinematic,
                        d2.is_kinematic,
                        &mut positions,
                        &mut velocities,
                    );
                }
            }
        }
    }
}

/// Check for collision between two moving spheres using CCD.
///
/// Uses displacement (predicted - prev) rather than velocity so that
/// t in [0, 1] represents the actual frame interval.
fn check_sphere_sphere_collision(
    d1: &DynamicEntity,
    d2: &DynamicEntity,
) -> Option<CollisionResult> {
    let combined_radius = d1.radius + d2.radius;
    let combined_radius_sq = combined_radius * combined_radius;

    // Use actual frame displacement, not velocity
    let displacement1 = d1.predicted_pos - d1.prev_pos;
    let displacement2 = d2.predicted_pos - d2.prev_pos;
    let rel_displacement = displacement1 - displacement2;

    let initial_delta = d1.prev_pos - d2.prev_pos;
    let initial_dist_sq = initial_delta.magnitude_squared();

    // First check: already overlapping at start of frame?
    if initial_dist_sq < combined_radius_sq {
        let dist = initial_dist_sq.sqrt();
        let normal = if dist < 0.0001 {
            Vector3::x()
        } else {
            initial_delta / dist
        };
        let depth = combined_radius - dist;
        return Some(CollisionResult { normal, depth });
    }

    // Swept sphere-sphere: solve quadratic for time of first contact
    // Position at time t: prev + displacement * t, where t in [0, 1]
    // Solve: |initial_delta + rel_displacement * t|^2 = combined_radius^2
    let a = rel_displacement.magnitude_squared();
    let b = 2.0 * initial_delta.dot(&rel_displacement);
    let c = initial_dist_sq - combined_radius_sq;

    // If not moving relative to each other, only check end overlap
    if a < 1e-10 {
        return check_end_overlap(d1, d2, combined_radius);
    }

    let discriminant = b * b - 4.0 * a * c;

    // No real roots means trajectories never intersect
    if discriminant < 0.0 {
        return check_end_overlap(d1, d2, combined_radius);
    }

    // Find earliest intersection time
    let sqrt_disc = discriminant.sqrt();
    let t = (-b - sqrt_disc) / (2.0 * a);

    // Collision must happen within this frame (t in [0, 1])
    if t < 0.0 || t > 1.0 {
        return check_end_overlap(d1, d2, combined_radius);
    }

    // Collision at time t - calculate normal at contact point
    let contact_delta = initial_delta + rel_displacement * t;
    let dist = contact_delta.magnitude();

    let normal = if dist < 0.0001 {
        Vector3::x()
    } else {
        contact_delta / dist
    };

    // Depth is how much they'd overlap at end of frame without response
    let final_delta = d1.predicted_pos - d2.predicted_pos;
    let final_dist = final_delta.magnitude();
    let depth = (combined_radius - final_dist).max(0.001);

    Some(CollisionResult { normal, depth })
}

/// Check if spheres overlap at their predicted end positions.
fn check_end_overlap(
    d1: &DynamicEntity,
    d2: &DynamicEntity,
    combined_radius: f32,
) -> Option<CollisionResult> {
    let final_delta = d1.predicted_pos - d2.predicted_pos;
    let final_dist_sq = final_delta.magnitude_squared();
    let combined_radius_sq = combined_radius * combined_radius;

    if final_dist_sq < combined_radius_sq {
        let dist = final_dist_sq.sqrt();
        let normal = if dist < 0.0001 {
            Vector3::x()
        } else {
            final_delta / dist
        };
        let depth = combined_radius - dist;

        return Some(CollisionResult { normal, depth });
    }

    None
}

/// Apply impulse-based collision response with mass.
fn apply_collision_response(
    entity1: Entity,
    entity2: Entity,
    collision: &CollisionResult,
    physics1: &PhysicsBody,
    physics2: &PhysicsBody,
    is_kinematic1: bool,
    is_kinematic2: bool,
    positions: &mut WriteStorage<Position>,
    velocities: &mut WriteStorage<Velocity>,
) {
    let vel1 = match velocities.get(entity1) {
        Some(v) => v.0,
        None => return,
    };
    let vel2 = match velocities.get(entity2) {
        Some(v) => v.0,
        None => return,
    };

    // Relative velocity
    let rel_vel = vel1 - vel2;
    let vel_along_normal = rel_vel.dot(&collision.normal);

    // Only resolve if moving toward each other
    if vel_along_normal > 0.0 {
        return;
    }

    // Calculate impulse magnitude with mass
    let restitution = (physics1.restitution + physics2.restitution) * 0.5;
    let inv_mass1 = if is_kinematic1 {
        0.0
    } else {
        1.0 / physics1.mass
    };
    let inv_mass2 = if is_kinematic2 {
        0.0
    } else {
        1.0 / physics2.mass
    };
    let inv_mass_sum = inv_mass1 + inv_mass2;

    if inv_mass_sum <= 0.0 {
        return;
    }

    let impulse_mag = -(1.0 + restitution) * vel_along_normal / inv_mass_sum;

    // Apply velocity impulses
    let impulse = collision.normal * impulse_mag;

    if !is_kinematic1 {
        if let Some(v) = velocities.get_mut(entity1) {
            v.0 += impulse * inv_mass1;
        }
    }
    if !is_kinematic2 {
        if let Some(v) = velocities.get_mut(entity2) {
            v.0 -= impulse * inv_mass2;
        }
    }

    // Separate positions to resolve penetration (proportional to inverse mass)
    if collision.depth > 0.001 {
        let separation1 = collision.depth * inv_mass1 / inv_mass_sum + 0.001;
        let separation2 = collision.depth * inv_mass2 / inv_mass_sum + 0.001;

        if !is_kinematic1 {
            if let Some(p) = positions.get_mut(entity1) {
                p.0 += collision.normal * separation1;
            }
        }
        if !is_kinematic2 {
            if let Some(p) = positions.get_mut(entity2) {
                p.0 -= collision.normal * separation2;
            }
        }
    }
}
