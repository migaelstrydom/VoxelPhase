//! Constraint solver for contact resolution.

use generational_arena::Arena;
use nalgebra::{Point3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::collider::ColliderMaterial;
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::world::PhysicsConfig;

/// A contact constraint to be solved.
#[derive(Debug, Clone)]
pub struct ContactConstraint {
    /// First body (or None for static geometry).
    pub body_a: Option<RigidBodyHandle>,
    /// Second body.
    pub body_b: RigidBodyHandle,
    /// First collider (None for static geometry).
    pub collider_a: Option<ColliderHandle>,
    /// Second collider (None for transient CCD contacts).
    pub collider_b: Option<ColliderHandle>,
    /// Contact point in world space.
    pub point: Point3<f32>,
    /// Contact normal pointing from A to B.
    pub normal: Vector3<f32>,
    /// Penetration depth.
    pub depth: f32,
    /// Combined material properties.
    pub restitution: f32,
    pub friction: f32,
    /// Cached normal impulse from manifold (for warm-starting).
    pub warm_normal_impulse: f32,
    /// Cached tangent impulses from manifold (for warm-starting).
    pub warm_tangent_impulse: [f32; 2],
}

/// Accumulated impulses from the solver, for writing back to the manifold cache.
#[derive(Debug, Clone, Default)]
pub struct SolvedImpulses {
    pub normal: f32,
    pub tangent: [f32; 2],
}

/// Solve contact constraints with warm-starting and multiple iterations.
///
/// Pipeline:
/// 1. Warm-start: apply cached impulses from the manifold cache
/// 2. Iterative solve: run `config.solver_iterations` passes of sequential impulses
/// 3. Return accumulated impulses for manifold writeback
pub fn solve(
    bodies: &mut Arena<RigidBody>,
    contacts: &[ContactConstraint],
    config: &PhysicsConfig,
) -> Vec<SolvedImpulses> {
    if contacts.is_empty() {
        return Vec::new();
    }

    // Phase 1: Warm-start — apply cached impulses from previous frame
    warm_start(bodies, contacts);

    // Phase 2: Iterative sequential-impulse solving
    // Initialize with warm-start values so writeback captures the total impulse
    let mut accumulated: Vec<SolvedImpulses> = contacts
        .iter()
        .map(|c| SolvedImpulses {
            normal: c.warm_normal_impulse,
            tangent: c.warm_tangent_impulse,
        })
        .collect();

    for _ in 0..config.solver_iterations {
        for (i, contact) in contacts.iter().enumerate() {
            let impulse = solve_single_contact(bodies, contact, config.restitution_velocity_threshold);
            accumulated[i].normal += impulse.normal;
            accumulated[i].tangent[0] += impulse.tangent[0];
            accumulated[i].tangent[1] += impulse.tangent[1];
        }
    }

    // Phase 3: Position correction — run once after velocity solving, not per iteration
    for contact in contacts {
        if contact.depth > 0.001 {
            let inv_mass_a = contact
                .body_a
                .and_then(|h| bodies.get(h.0))
                .map(|b| b.inv_mass())
                .unwrap_or(0.0);
            let inv_mass_b = bodies
                .get(contact.body_b.0)
                .map(|b| b.inv_mass())
                .unwrap_or(0.0);
            apply_position_correction(bodies, contact, inv_mass_a, inv_mass_b);
        }
    }

    accumulated
}

/// Apply cached impulses from the manifold to give the solver a head start.
fn warm_start(bodies: &mut Arena<RigidBody>, contacts: &[ContactConstraint]) {
    for contact in contacts {
        if contact.warm_normal_impulse.abs() < 1e-8
            && contact.warm_tangent_impulse[0].abs() < 1e-8
            && contact.warm_tangent_impulse[1].abs() < 1e-8
        {
            continue;
        }

        let impulse = contact.normal * contact.warm_normal_impulse;

        // Compute tangent directions for warm-starting friction
        let tangent_impulse = compute_tangent_impulse(contact);

        let total = impulse + tangent_impulse;

        if let Some(handle_a) = contact.body_a {
            if let Some(body_a) = bodies.get_mut(handle_a.0) {
                if body_a.is_dynamic() {
                    body_a.apply_impulse_at_point(-total, contact.point);
                }
            }
        }

        if let Some(body_b) = bodies.get_mut(contact.body_b.0) {
            if body_b.is_dynamic() {
                body_b.apply_impulse_at_point(total, contact.point);
            }
        }
    }
}

/// Compute a tangent impulse vector from cached tangent impulse magnitudes.
fn compute_tangent_impulse(contact: &ContactConstraint) -> Vector3<f32> {
    let (t1, t2) = compute_tangent_basis(&contact.normal);
    t1 * contact.warm_tangent_impulse[0] + t2 * contact.warm_tangent_impulse[1]
}

/// Compute a stable orthonormal tangent basis from a normal vector.
fn compute_tangent_basis(normal: &Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    // Choose the axis least aligned with the normal to avoid degeneracy
    let reference = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let t1 = normal.cross(&reference).normalize();
    let t2 = normal.cross(&t1);
    (t1, t2)
}

/// Solve contacts without warm-starting (for transient contacts like CCD).
pub fn solve_contacts(
    bodies: &mut Arena<RigidBody>,
    contacts: &[ContactConstraint],
    restitution_velocity_threshold: f32,
) {
    for contact in contacts {
        solve_single_contact(bodies, contact, restitution_velocity_threshold);
    }
}

fn solve_single_contact(
    bodies: &mut Arena<RigidBody>,
    contact: &ContactConstraint,
    restitution_velocity_threshold: f32,
) -> SolvedImpulses {
    let zero = SolvedImpulses::default();

    // Get body B's state
    let (pos_b, vel_b, angular_vel_b, inv_mass_b, inv_inertia_b) = {
        let Some(body_b) = bodies.get(contact.body_b.0) else {
            return zero;
        };
        (
            body_b.position(),
            body_b.linear_velocity(),
            body_b.angular_velocity(),
            body_b.inv_mass(),
            body_b.world_inv_inertia(),
        )
    };

    // Get body A's state (or defaults for static geometry)
    let (pos_a, vel_a, angular_vel_a, inv_mass_a, inv_inertia_a) = match contact.body_a {
        Some(handle) => {
            let Some(body_a) = bodies.get(handle.0) else {
                return zero;
            };
            (
                body_a.position(),
                body_a.linear_velocity(),
                body_a.angular_velocity(),
                body_a.inv_mass(),
                body_a.world_inv_inertia(),
            )
        }
        None => (
            contact.point,
            Vector3::zeros(),
            Vector3::zeros(),
            0.0,
            nalgebra::Matrix3::zeros(),
        ),
    };

    // Compute relative velocity at contact point
    let r_a = contact.point - pos_a;
    let r_b = contact.point - pos_b;

    let vel_at_contact_a = vel_a + angular_vel_a.cross(&r_a);
    let vel_at_contact_b = vel_b + angular_vel_b.cross(&r_b);
    let rel_vel = vel_at_contact_b - vel_at_contact_a;

    let vel_along_normal = rel_vel.dot(&contact.normal);

    // Only resolve if objects are approaching
    if vel_along_normal > 0.0 {
        return zero;
    }

    // Compute effective mass for the contact
    let r_a_cross_n = r_a.cross(&contact.normal);
    let r_b_cross_n = r_b.cross(&contact.normal);

    let angular_effect_a = (inv_inertia_a * r_a_cross_n).cross(&r_a);
    let angular_effect_b = (inv_inertia_b * r_b_cross_n).cross(&r_b);

    let effective_mass =
        inv_mass_a + inv_mass_b + (angular_effect_a + angular_effect_b).dot(&contact.normal);

    if effective_mass <= 0.0 {
        return zero;
    }

    // Zero out restitution for slow approaches to prevent micro-bouncing at rest
    let restitution = if vel_along_normal.abs() < restitution_velocity_threshold {
        0.0
    } else {
        contact.restitution
    };
    let j = -(1.0 + restitution) * vel_along_normal / effective_mass;

    // Apply normal impulse
    let impulse = contact.normal * j;

    if let Some(handle_a) = contact.body_a {
        if let Some(body_a) = bodies.get_mut(handle_a.0) {
            if body_a.is_dynamic() {
                body_a.apply_impulse_at_point(-impulse, contact.point);
            }
        }
    }

    if let Some(body_b) = bodies.get_mut(contact.body_b.0) {
        if body_b.is_dynamic() {
            body_b.apply_impulse_at_point(impulse, contact.point);
        }
    }

    // Apply friction impulse and capture the tangent impulse magnitudes
    let tangent_solved = apply_friction(bodies, contact, &rel_vel, j);

    SolvedImpulses {
        normal: j.max(0.0),
        tangent: tangent_solved,
    }
}

fn apply_friction(
    bodies: &mut Arena<RigidBody>,
    contact: &ContactConstraint,
    rel_vel: &Vector3<f32>,
    normal_impulse: f32,
) -> [f32; 2] {
    let (t1, t2) = compute_tangent_basis(&contact.normal);

    // Project relative velocity onto each tangent direction
    let v_t1 = rel_vel.dot(&t1);
    let v_t2 = rel_vel.dot(&t2);

    let max_friction = contact.friction * normal_impulse.abs();

    // Clamp each tangent impulse independently
    let j_t1 = (-v_t1).clamp(-max_friction, max_friction);
    let j_t2 = (-v_t2).clamp(-max_friction, max_friction);

    let friction = t1 * j_t1 + t2 * j_t2;

    if friction.magnitude_squared() < 1e-12 {
        return [0.0, 0.0];
    }

    if let Some(handle_a) = contact.body_a {
        if let Some(body_a) = bodies.get_mut(handle_a.0) {
            if body_a.is_dynamic() {
                body_a.apply_impulse_at_point(-friction, contact.point);
            }
        }
    }

    if let Some(body_b) = bodies.get_mut(contact.body_b.0) {
        if body_b.is_dynamic() {
            body_b.apply_impulse_at_point(friction, contact.point);
        }
    }

    [j_t1, j_t2]
}

fn apply_position_correction(
    bodies: &mut Arena<RigidBody>,
    contact: &ContactConstraint,
    inv_mass_a: f32,
    inv_mass_b: f32,
) {
    let total_inv_mass = inv_mass_a + inv_mass_b;
    if total_inv_mass <= 0.0 {
        return;
    }

    // Baumgarte stabilization: push objects apart based on penetration
    let correction_factor = 0.2;
    let slop = 0.001;
    let correction = (contact.depth - slop).max(0.0) * correction_factor / total_inv_mass;

    if let Some(handle_a) = contact.body_a {
        if let Some(body_a) = bodies.get_mut(handle_a.0) {
            if body_a.is_dynamic() {
                let pos = body_a.position();
                body_a.set_position(pos - contact.normal * correction * inv_mass_a);
            }
        }
    }

    if let Some(body_b) = bodies.get_mut(contact.body_b.0) {
        if body_b.is_dynamic() {
            let pos = body_b.position();
            body_b.set_position(pos + contact.normal * correction * inv_mass_b);
        }
    }
}

/// Combine material properties from two colliders.
pub fn combine_materials(mat_a: &ColliderMaterial, mat_b: &ColliderMaterial) -> (f32, f32) {
    let restitution = (mat_a.restitution + mat_b.restitution) * 0.5;
    let friction = (mat_a.friction * mat_b.friction).sqrt();
    (restitution, friction)
}
