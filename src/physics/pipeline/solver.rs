//! Constraint solver for contact resolution.

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::collider::ColliderMaterial;
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::pipeline::post_stabilizer::{is_kinematic_static_contact, post_stabilize};
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
    pub point: nalgebra::Point3<f32>,
    /// Contact normal pointing from A to B.
    pub normal: Vector3<f32>,
    /// Raw contact normal before any smoothing or clustering.
    pub raw_normal: Vector3<f32>,
    /// Penetration depth.
    pub depth: f32,
    /// Unclamped penetration depth (can be negative for margin contacts).
    pub raw_depth: f32,
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
/// 3. Post-stabilization: penetration correction + contact damping
/// 4. Return accumulated impulses for manifold writeback
pub fn solve(
    bodies: &mut Arena<RigidBody>,
    contacts: &[ContactConstraint],
    config: &PhysicsConfig,
    dt: f32,
) -> Vec<SolvedImpulses> {
    if contacts.is_empty() {
        return Vec::new();
    }

    // Phase 1: Capture pre-solve normal velocities (before warm-start).
    let pre_solve_vn: Vec<f32> = contacts
        .iter()
        .map(|contact| relative_normal_velocity(bodies, contact))
        .collect();

    // Phase 2: Warm-start — apply cached impulses from previous frame
    let warm_scales: Vec<f32> = pre_solve_vn
        .iter()
        .map(|vn| {
            if vn.abs() > config.restitution_velocity_threshold {
                0.0
            } else {
                config.warm_start_scale
            }
        })
        .collect();
    warm_start(bodies, contacts, &warm_scales);

    // Phase 3: Iterative sequential-impulse solving
    let mut accumulated: Vec<SolvedImpulses> = contacts
        .iter()
        .zip(warm_scales.iter())
        .map(|(c, scale)| SolvedImpulses {
            normal: c.warm_normal_impulse * *scale,
            tangent: [
                c.warm_tangent_impulse[0] * *scale,
                c.warm_tangent_impulse[1] * *scale,
            ],
        })
        .collect();

    for _ in 0..config.solver_iterations {
        for (i, contact) in contacts.iter().enumerate() {
            solve_single_contact_pgs(
                bodies,
                contact,
                config.restitution_velocity_threshold,
                pre_solve_vn[i],
                config.restitution_depth_slop,
                &mut accumulated[i],
            );
        }
    }

    // Phase 4: Post-stabilization correction after velocity solving
    post_stabilize(bodies, contacts, &config.post_stabilise, dt);

    accumulated
}

/// Apply cached impulses from the manifold to give the solver a head start.
fn warm_start(bodies: &mut Arena<RigidBody>, contacts: &[ContactConstraint], scales: &[f32]) {
    for (contact, scale) in contacts.iter().zip(scales.iter()) {
        if *scale <= 0.0 {
            continue;
        }
        if contact.warm_normal_impulse.abs() < 1e-8
            && contact.warm_tangent_impulse[0].abs() < 1e-8
            && contact.warm_tangent_impulse[1].abs() < 1e-8
        {
            continue;
        }

        let impulse = contact.normal * (contact.warm_normal_impulse * *scale);

        let tangent_impulse = compute_tangent_impulse(contact) * *scale;

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
    restitution_depth_slop: f32,
) {
    for contact in contacts {
        let mut accumulated = SolvedImpulses::default();
        let pre_solve_vn = relative_normal_velocity(bodies, contact);
        solve_single_contact_pgs(
            bodies,
            contact,
            restitution_velocity_threshold,
            pre_solve_vn,
            restitution_depth_slop,
            &mut accumulated,
        );
    }
}

fn relative_normal_velocity(bodies: &Arena<RigidBody>, contact: &ContactConstraint) -> f32 {
    let (pos_b, vel_b, angular_vel_b) = {
        let Some(body_b) = bodies.get(contact.body_b.0) else {
            return 0.0;
        };
        (
            body_b.position(),
            body_b.linear_velocity(),
            body_b.angular_velocity(),
        )
    };

    let (pos_a, vel_a, angular_vel_a) = match contact.body_a {
        Some(handle) => {
            let Some(body_a) = bodies.get(handle.0) else {
                return 0.0;
            };
            (
                body_a.position(),
                body_a.linear_velocity(),
                body_a.angular_velocity(),
            )
        }
        None => (contact.point, Vector3::zeros(), Vector3::zeros()),
    };

    let r_a = contact.point - pos_a;
    let r_b = contact.point - pos_b;
    let vel_at_contact_a = vel_a + angular_vel_a.cross(&r_a);
    let vel_at_contact_b = vel_b + angular_vel_b.cross(&r_b);
    let rel_vel = vel_at_contact_b - vel_at_contact_a;
    rel_vel.dot(&contact.normal)
}

fn solve_single_contact_pgs(
    bodies: &mut Arena<RigidBody>,
    contact: &ContactConstraint,
    restitution_velocity_threshold: f32,
    pre_solve_vn: f32,
    restitution_depth_slop: f32,
    accumulated: &mut SolvedImpulses,
) {
    let (pos_b, vel_b, angular_vel_b, inv_mass_b, inv_inertia_b) = {
        let Some(body_b) = bodies.get(contact.body_b.0) else {
            return;
        };
        let kinematic_static = is_kinematic_static_contact(body_b, contact);
        (
            body_b.position(),
            body_b.linear_velocity(),
            body_b.angular_velocity(),
            if kinematic_static {
                1.0
            } else {
                body_b.inv_mass()
            },
            if kinematic_static {
                nalgebra::Matrix3::zeros()
            } else {
                body_b.world_inv_inertia()
            },
        )
    };

    let (pos_a, vel_a, angular_vel_a, inv_mass_a, inv_inertia_a) = match contact.body_a {
        Some(handle) => {
            let Some(body_a) = bodies.get(handle.0) else {
                return;
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

    let r_a = contact.point - pos_a;
    let r_b = contact.point - pos_b;

    let vel_at_contact_a = vel_a + angular_vel_a.cross(&r_a);
    let vel_at_contact_b = vel_b + angular_vel_b.cross(&r_b);
    let rel_vel = vel_at_contact_b - vel_at_contact_a;

    let vel_along_normal = rel_vel.dot(&contact.normal);

    if vel_along_normal > 0.0 {
        return;
    }

    let r_a_cross_n = r_a.cross(&contact.normal);
    let r_b_cross_n = r_b.cross(&contact.normal);

    let angular_effect_a = (inv_inertia_a * r_a_cross_n).cross(&r_a);
    let angular_effect_b = (inv_inertia_b * r_b_cross_n).cross(&r_b);

    let effective_mass =
        inv_mass_a + inv_mass_b + (angular_effect_a + angular_effect_b).dot(&contact.normal);

    if effective_mass <= 0.0 {
        return;
    }

    let restitution = if pre_solve_vn.abs() < restitution_velocity_threshold {
        0.0
    } else if contact.raw_depth >= -restitution_depth_slop {
        contact.restitution
    } else {
        0.0
    };
    let restitution_velocity = if pre_solve_vn < 0.0 {
        restitution * pre_solve_vn
    } else {
        0.0
    };
    let delta_normal = -(vel_along_normal + restitution_velocity) / effective_mass;
    let old_normal = accumulated.normal;
    let new_normal = (old_normal + delta_normal).max(0.0);
    let applied_normal = new_normal - old_normal;
    accumulated.normal = new_normal;

    if applied_normal.abs() > 1e-10 {
        let impulse = contact.normal * applied_normal;
        if let Some(handle_a) = contact.body_a {
            if let Some(body_a) = bodies.get_mut(handle_a.0) {
                if body_a.is_dynamic() {
                    body_a.apply_impulse_at_point(-impulse, contact.point);
                }
            }
        }

        apply_body_b_impulse(bodies, contact, impulse);
    }

    if accumulated.normal <= 0.0 || contact.friction <= 0.0 {
        accumulated.tangent = [0.0, 0.0];
        return;
    }

    let (pos_b, vel_b, angular_vel_b, inv_mass_b, inv_inertia_b) = {
        let Some(body_b) = bodies.get(contact.body_b.0) else {
            return;
        };
        (
            body_b.position(),
            body_b.linear_velocity(),
            body_b.angular_velocity(),
            body_b.inv_mass(),
            body_b.world_inv_inertia(),
        )
    };

    let (pos_a, vel_a, angular_vel_a, inv_mass_a, inv_inertia_a) = match contact.body_a {
        Some(handle) => {
            let Some(body_a) = bodies.get(handle.0) else {
                return;
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

    let r_a = contact.point - pos_a;
    let r_b = contact.point - pos_b;
    let vel_at_contact_a = vel_a + angular_vel_a.cross(&r_a);
    let vel_at_contact_b = vel_b + angular_vel_b.cross(&r_b);
    let rel_vel = vel_at_contact_b - vel_at_contact_a;

    let (t1, t2) = compute_tangent_basis(&contact.normal);
    let v_t1 = rel_vel.dot(&t1);
    let v_t2 = rel_vel.dot(&t2);

    let r_a_cross_t1 = r_a.cross(&t1);
    let r_b_cross_t1 = r_b.cross(&t1);
    let angular_effect_a_t1 = (inv_inertia_a * r_a_cross_t1).cross(&r_a);
    let angular_effect_b_t1 = (inv_inertia_b * r_b_cross_t1).cross(&r_b);
    let effective_mass_t1 =
        inv_mass_a + inv_mass_b + (angular_effect_a_t1 + angular_effect_b_t1).dot(&t1);

    let r_a_cross_t2 = r_a.cross(&t2);
    let r_b_cross_t2 = r_b.cross(&t2);
    let angular_effect_a_t2 = (inv_inertia_a * r_a_cross_t2).cross(&r_a);
    let angular_effect_b_t2 = (inv_inertia_b * r_b_cross_t2).cross(&r_b);
    let effective_mass_t2 =
        inv_mass_a + inv_mass_b + (angular_effect_a_t2 + angular_effect_b_t2).dot(&t2);

    if effective_mass_t1 <= 0.0 || effective_mass_t2 <= 0.0 {
        return;
    }

    let delta_t1 = -v_t1 / effective_mass_t1;
    let delta_t2 = -v_t2 / effective_mass_t2;

    let old_t1 = accumulated.tangent[0];
    let old_t2 = accumulated.tangent[1];
    let mut new_t1 = old_t1 + delta_t1;
    let mut new_t2 = old_t2 + delta_t2;

    let max_friction = contact.friction * accumulated.normal;
    let mag = (new_t1 * new_t1 + new_t2 * new_t2).sqrt();
    if mag > max_friction {
        let scale = max_friction / mag;
        new_t1 *= scale;
        new_t2 *= scale;
    }

    let applied_t1 = new_t1 - old_t1;
    let applied_t2 = new_t2 - old_t2;
    accumulated.tangent = [new_t1, new_t2];

    if applied_t1.abs() > 1e-10 || applied_t2.abs() > 1e-10 {
        let friction = t1 * applied_t1 + t2 * applied_t2;
        if let Some(handle_a) = contact.body_a {
            if let Some(body_a) = bodies.get_mut(handle_a.0) {
                if body_a.is_dynamic() {
                    body_a.apply_impulse_at_point(-friction, contact.point);
                }
            }
        }

        apply_body_b_impulse(bodies, contact, friction);
    }
}

fn apply_body_b_impulse(
    bodies: &mut Arena<RigidBody>,
    contact: &ContactConstraint,
    impulse: Vector3<f32>,
) {
    if let Some(body_b) = bodies.get_mut(contact.body_b.0) {
        if body_b.is_dynamic() {
            body_b.apply_impulse_at_point(impulse, contact.point);
        } else if is_kinematic_static_contact(body_b, contact) {
            body_b.set_linear_velocity(body_b.linear_velocity() + impulse);
        }
    }
}

/// Combine material properties from two colliders.
pub fn combine_materials(mat_a: &ColliderMaterial, mat_b: &ColliderMaterial) -> (f32, f32) {
    let restitution = (mat_a.restitution + mat_b.restitution) * 0.5;
    let friction = (mat_a.friction * mat_b.friction).sqrt();
    (restitution, friction)
}
