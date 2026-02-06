//! Constraint solver for contact resolution.

use generational_arena::Arena;
use nalgebra::{Point3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::collider::ColliderMaterial;
use crate::physics::handle::RigidBodyHandle;

/// A contact constraint to be solved.
#[derive(Debug, Clone)]
pub struct ContactConstraint {
    /// First body (or None for static geometry).
    pub body_a: Option<RigidBodyHandle>,
    /// Second body.
    pub body_b: RigidBodyHandle,
    /// Contact point in world space.
    pub point: Point3<f32>,
    /// Contact normal pointing from A to B.
    pub normal: Vector3<f32>,
    /// Penetration depth.
    pub depth: f32,
    /// Combined material properties.
    pub restitution: f32,
    pub friction: f32,
}

/// Solve contact constraints using sequential impulses.
///
/// This is a simple single-iteration solver. Phase 3 will add multiple iterations
/// and warm starting for stable stacking.
pub fn solve_contacts(
    bodies: &mut Arena<RigidBody>,
    contacts: &[ContactConstraint],
    restitution_velocity_threshold: f32,
) -> Vec<f32> {
    let mut impulses = Vec::with_capacity(contacts.len());
    for contact in contacts {
        impulses.push(solve_single_contact(bodies, contact, restitution_velocity_threshold));
    }
    impulses
}

fn solve_single_contact(
    bodies: &mut Arena<RigidBody>,
    contact: &ContactConstraint,
    restitution_velocity_threshold: f32,
) -> f32 {
    // Get body B's state
    let (pos_b, vel_b, angular_vel_b, inv_mass_b, inv_inertia_b) = {
        let Some(body_b) = bodies.get(contact.body_b.0) else {
            return 0.0;
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
                return 0.0;
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
            contact.point, // Static geometry at contact point
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
        // Separating, but still need position correction if penetrating
        if contact.depth > 0.001 {
            apply_position_correction(bodies, contact, inv_mass_a, inv_mass_b);
        }
        return 0.0;
    }

    // Compute effective mass for the contact
    let r_a_cross_n = r_a.cross(&contact.normal);
    let r_b_cross_n = r_b.cross(&contact.normal);

    let angular_effect_a = (inv_inertia_a * r_a_cross_n).cross(&r_a);
    let angular_effect_b = (inv_inertia_b * r_b_cross_n).cross(&r_b);

    let effective_mass =
        inv_mass_a + inv_mass_b + (angular_effect_a + angular_effect_b).dot(&contact.normal);

    if effective_mass <= 0.0 {
        return 0.0;
    }

    // Zero out restitution for slow approaches to prevent micro-bouncing at rest
    let restitution = if vel_along_normal.abs() < restitution_velocity_threshold {
        0.0
    } else {
        contact.restitution
    };
    let j = -(1.0 + restitution) * vel_along_normal / effective_mass;

    // Apply impulse
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

    // Apply friction impulse
    apply_friction(bodies, contact, &rel_vel, j);

    // Position correction for penetration
    if contact.depth > 0.001 {
        apply_position_correction(bodies, contact, inv_mass_a, inv_mass_b);
    }

    j.max(0.0)
}

fn apply_friction(
    bodies: &mut Arena<RigidBody>,
    contact: &ContactConstraint,
    rel_vel: &Vector3<f32>,
    normal_impulse: f32,
) {
    // Compute tangent velocity
    let vel_along_normal = rel_vel.dot(&contact.normal);
    let tangent_vel = rel_vel - contact.normal * vel_along_normal;
    let tangent_speed = tangent_vel.magnitude();

    if tangent_speed < 1e-6 {
        return;
    }

    let tangent_dir = tangent_vel / tangent_speed;

    // Coulomb friction: friction impulse <= mu * normal impulse
    let max_friction = contact.friction * normal_impulse.abs();
    let friction_impulse = tangent_speed.min(max_friction);
    let friction = -tangent_dir * friction_impulse;

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
    let correction_factor = 0.2; // How much penetration to correct per frame
    let slop = 0.001; // Allow small penetration to avoid jitter
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
    // Average restitution, geometric mean friction (common approach)
    let restitution = (mat_a.restitution + mat_b.restitution) * 0.5;
    let friction = (mat_a.friction * mat_b.friction).sqrt();
    (restitution, friction)
}
