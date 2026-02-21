//! Constraint solver for contact resolution.

use generational_arena::Arena;
use nalgebra::{Matrix3, Point3, Vector3};
use std::collections::HashMap;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::{PairHeader, SolverContact, SolverManifold};
use crate::physics::pipeline::post_stabilizer::post_stabilize;
use crate::physics::world::PhysicsConfig;

/// Minimum effective inverse mass allowed in the normal solver.
///
/// Prevents near-zero denominators (degenerate geometry or deeply wedged bodies)
/// from producing runaway impulses.
const MIN_EFFECTIVE_INV_MASS: f32 = 1.0e-8;

/// Snapshot of both bodies' physics state for a contact pair.
///
/// Stores real physics values — no kinematic overrides. Callers that need the
/// kinematic-static treatment apply the override via `effective_mass_with_overrides`.
struct BodyPairState {
    pos_a: Point3<f32>,
    vel_a: Vector3<f32>,
    angular_vel_a: Vector3<f32>,
    inv_mass_a: f32,
    inv_inertia_a: Matrix3<f32>,

    pos_b: Point3<f32>,
    vel_b: Vector3<f32>,
    angular_vel_b: Vector3<f32>,
    inv_mass_b: f32,
    inv_inertia_b: Matrix3<f32>,
}

impl BodyPairState {
    /// Extract physics state for both sides of a contact pair.
    ///
    /// `contact_point` is used as the fallback position for static body_a (where
    /// body_a is None). The actual value doesn't affect physics since static bodies
    /// have zero mass and inertia.
    fn extract(
        bodies: &Arena<RigidBody>,
        header: &PairHeader,
        contact_point: Point3<f32>,
    ) -> Option<Self> {
        let body_b = bodies.get(header.body_b.0)?;

        let (pos_a, vel_a, angular_vel_a, inv_mass_a, inv_inertia_a) = match header.body_a {
            Some(handle) => {
                let body_a = bodies.get(handle.0)?;
                (
                    body_a.position(),
                    body_a.linear_velocity(),
                    body_a.angular_velocity(),
                    body_a.inv_mass(),
                    body_a.world_inv_inertia(),
                )
            }
            None => (
                contact_point,
                Vector3::zeros(),
                Vector3::zeros(),
                0.0,
                Matrix3::zeros(),
            ),
        };

        Some(Self {
            pos_a,
            vel_a,
            angular_vel_a,
            inv_mass_a,
            inv_inertia_a,
            pos_b: body_b.position(),
            vel_b: body_b.linear_velocity(),
            angular_vel_b: body_b.angular_velocity(),
            inv_mass_b: body_b.inv_mass(),
            inv_inertia_b: body_b.world_inv_inertia(),
        })
    }

    /// Relative velocity at the contact point, projected onto the normal.
    fn relative_normal_velocity(&self, point: Point3<f32>, normal: &Vector3<f32>) -> f32 {
        let rel_vel = self.relative_velocity_at(point);
        rel_vel.dot(normal)
    }

    /// Relative velocity at a point (B minus A).
    fn relative_velocity_at(&self, point: Point3<f32>) -> Vector3<f32> {
        let r_a = point - self.pos_a;
        let r_b = point - self.pos_b;
        let vel_at_a = self.vel_a + self.angular_vel_a.cross(&r_a);
        let vel_at_b = self.vel_b + self.angular_vel_b.cross(&r_b);
        vel_at_b - vel_at_a
    }

    /// Compute the effective mass for an impulse along the given direction.
    fn effective_inv_mass(&self, point: Point3<f32>, direction: &Vector3<f32>) -> f32 {
        self.effective_inv_mass_with_overrides(
            point,
            direction,
            self.inv_mass_b,
            self.inv_inertia_b,
        )
    }

    /// Compute effective (inverse) mass with explicit body_b overrides.
    ///
    /// Used by the normal solver to treat kinematic-vs-static contacts
    /// as having unit (inverse) mass, while friction uses the real (zero) mass so
    /// kinematic bodies aren't slowed by friction against static geometry.
    fn effective_inv_mass_with_overrides(
        &self,
        point: Point3<f32>,
        direction: &Vector3<f32>,
        inv_mass_b: f32,
        inv_inertia_b: Matrix3<f32>,
    ) -> f32 {
        let r_a = point - self.pos_a;
        let r_b = point - self.pos_b;

        let r_a_cross = r_a.cross(direction);
        let r_b_cross = r_b.cross(direction);

        let angular_a = (self.inv_inertia_a * r_a_cross).cross(&r_a);
        let angular_b = (inv_inertia_b * r_b_cross).cross(&r_b);

        self.inv_mass_a + inv_mass_b + (angular_a + angular_b).dot(direction)
    }
}

/// Returns true if body_b is kinematic and body_a is static geometry.
#[inline]
fn is_kinematic_static(body: &RigidBody, header: &PairHeader) -> bool {
    body.is_kinematic() && header.body_a.is_none()
}

/// Solve contact constraints with warm-starting and multiple iterations.
///
/// Pipeline:
/// 1. Warm-start: apply cached impulses from the manifold cache
/// 2. Iterative solve: run `config.solver_iterations` passes of sequential impulses
/// 3. Post-stabilization: penetration correction + contact damping
///
/// Accumulated impulses are written in-place on each `SolverContact`.
pub fn solve(
    bodies: &mut Arena<RigidBody>,
    manifolds: &mut [SolverManifold],
    config: &PhysicsConfig,
    dt: f32,
) {
    if manifolds.is_empty() {
        return;
    }

    // Phase 1: Capture pre-solve normal velocities and warm-start scales.
    // Indexed as [manifold_idx][contact_idx] = (pre_solve_vn, warm_scale).
    let pre_solve: Vec<Vec<(f32, f32)>> = manifolds
        .iter()
        .map(|m| {
            m.contacts
                .iter()
                .map(|c| {
                    let vn = BodyPairState::extract(bodies, &m.header, c.point)
                        .map(|s| s.relative_normal_velocity(c.point, &c.normal))
                        .unwrap_or(0.0);
                    let warm_scale = if vn.abs() > config.restitution_velocity_threshold {
                        0.0
                    } else {
                        config.warm_start_scale
                    };
                    (vn, warm_scale)
                })
                .collect()
        })
        .collect();

    // Phase 2: Warm-start — apply cached impulses from previous frame.
    for (mi, manifold) in manifolds.iter_mut().enumerate() {
        let header = &manifold.header;
        for (ci, contact) in manifold.contacts.iter_mut().enumerate() {
            warm_start_contact(bodies, header, contact, pre_solve[mi][ci].1);
        }
    }

    // Phase 3: Iterative sequential-impulse solving.
    let iterations = effective_solver_iterations(manifolds, config.solver_iterations);
    for _ in 0..iterations {
        for (mi, manifold) in manifolds.iter_mut().enumerate() {
            let header = &manifold.header;
            for (ci, contact) in manifold.contacts.iter_mut().enumerate() {
                solve_normal_impulse(
                    bodies,
                    header,
                    contact,
                    config.restitution_velocity_threshold,
                    pre_solve[mi][ci].0,
                );
                solve_friction_impulse(bodies, header, contact);
            }
        }
    }

    // Phase 4: Post-stabilization correction after velocity solving.
    post_stabilize(bodies, manifolds, &config.post_stabilise, dt);
}

fn effective_solver_iterations(manifolds: &[SolverManifold], base_iterations: u32) -> u32 {
    let mut per_body_counts: HashMap<RigidBodyHandle, usize> = HashMap::new();
    let mut per_body_normals: HashMap<RigidBodyHandle, Vec<Vector3<f32>>> = HashMap::new();
    for manifold in manifolds {
        for contact in &manifold.contacts {
            let entry = per_body_counts.entry(manifold.header.body_b).or_insert(0);
            *entry += 1;
            per_body_normals
                .entry(manifold.header.body_b)
                .or_default()
                .push(contact.normal);
        }
    }

    let mut extra = 0u32;
    if let Some(max_contacts) = per_body_counts.values().copied().max() {
        if max_contacts > 2 {
            extra += ((max_contacts - 2).min(4)) as u32;
        }
    }

    for normals in per_body_normals.values() {
        if normals.len() < 2 {
            continue;
        }
        let mut sum = Vector3::zeros();
        for n in normals {
            sum += *n;
        }
        if sum.magnitude_squared() < 1e-6 {
            continue;
        }
        let avg = sum.normalize();
        let mut min_dot = 1.0f32;
        for n in normals {
            min_dot = min_dot.min(n.dot(&avg));
        }
        if min_dot < 0.85 {
            extra += 2;
            break;
        }
    }

    base_iterations + extra
}

/// Apply cached impulse for a single contact and initialize its accumulated impulses.
fn warm_start_contact(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    contact: &mut SolverContact,
    scale: f32,
) {
    contact.accumulated_normal_impulse = contact.warm_normal_impulse * scale;
    contact.accumulated_tangent_impulse = [
        contact.warm_tangent_impulse[0] * scale,
        contact.warm_tangent_impulse[1] * scale,
    ];

    if scale <= 0.0 {
        return;
    }
    if contact.warm_normal_impulse.abs() < 1e-8
        && contact.warm_tangent_impulse[0].abs() < 1e-8
        && contact.warm_tangent_impulse[1].abs() < 1e-8
    {
        return;
    }

    let (t1, t2) = compute_tangent_basis(&contact.normal);
    let normal_impulse = contact.normal * (contact.warm_normal_impulse * scale);
    let tangent_impulse =
        (t1 * contact.warm_tangent_impulse[0] + t2 * contact.warm_tangent_impulse[1]) * scale;
    let total = normal_impulse + tangent_impulse;

    apply_impulse_pair(bodies, header, contact.point, total);
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
    manifolds: &mut [SolverManifold],
    restitution_velocity_threshold: f32,
) {
    for manifold in manifolds.iter_mut() {
        let header = &manifold.header;
        for contact in manifold.contacts.iter_mut() {
            let pre_solve_vn = BodyPairState::extract(bodies, header, contact.point)
                .map(|state| state.relative_normal_velocity(contact.point, &contact.normal))
                .unwrap_or(0.0);
            solve_normal_impulse(
                bodies,
                header,
                contact,
                restitution_velocity_threshold,
                pre_solve_vn,
            );
            solve_friction_impulse(bodies, header, contact);
        }
    }
}

// ---------------------------------------------------------------------------
// Normal impulse resolution
// ---------------------------------------------------------------------------

fn solve_normal_impulse(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    contact: &mut SolverContact,
    restitution_velocity_threshold: f32,
    pre_solve_vn: f32,
) {
    let Some(state) = BodyPairState::extract(bodies, header, contact.point) else {
        return;
    };

    let vel_along_normal = state.relative_normal_velocity(contact.point, &contact.normal);
    if vel_along_normal > 0.0 && contact.accumulated_normal_impulse <= 1e-8 {
        return;
    }

    // Kinematic-vs-static contacts use fake unit mass so the normal solver
    // can push the kinematic body out of static geometry. Friction does NOT
    // use this override — kinematic bodies should move freely along surfaces.
    let kinematic_static = bodies
        .get(header.body_b.0)
        .is_some_and(|b| is_kinematic_static(b, header));

    let (eff_inv_mass_b, eff_inv_inertia_b) = if kinematic_static {
        (1.0, Matrix3::zeros())
    } else {
        (state.inv_mass_b, state.inv_inertia_b)
    };

    let effective_inv_mass = state.effective_inv_mass_with_overrides(
        contact.point,
        &contact.normal,
        eff_inv_mass_b,
        eff_inv_inertia_b,
    );
    if effective_inv_mass <= MIN_EFFECTIVE_INV_MASS || !effective_inv_mass.is_finite() {
        return;
    }

    let speed = pre_solve_vn.abs();
    let restitution_scale =
        ((speed - restitution_velocity_threshold) / restitution_velocity_threshold).clamp(0.0, 1.0);
    let restitution = header.restitution * restitution_scale;
    let restitution_velocity = if pre_solve_vn < 0.0 {
        restitution * pre_solve_vn
    } else {
        0.0
    };

    let delta = -(vel_along_normal + restitution_velocity) / effective_inv_mass;
    let old = contact.accumulated_normal_impulse;
    let new = (old + delta).max(0.0);
    let applied = new - old;
    contact.accumulated_normal_impulse = new;

    if applied.abs() > 1e-10 {
        let impulse = contact.normal * applied;
        apply_impulse_pair(bodies, header, contact.point, impulse);
    }
}

// ---------------------------------------------------------------------------
// Friction impulse resolution
// ---------------------------------------------------------------------------

fn solve_friction_impulse(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    contact: &mut SolverContact,
) {
    if contact.accumulated_normal_impulse <= 0.0 || header.friction <= 0.0 {
        contact.accumulated_tangent_impulse = [0.0, 0.0];
        return;
    }

    let Some(state) = BodyPairState::extract(bodies, header, contact.point) else {
        return;
    };

    let rel_vel = state.relative_velocity_at(contact.point);
    let (t1, t2) = compute_tangent_basis(&contact.normal);

    let effective_mass_t1 = state.effective_inv_mass(contact.point, &t1);
    let effective_mass_t2 = state.effective_inv_mass(contact.point, &t2);
    if effective_mass_t1 <= MIN_EFFECTIVE_INV_MASS || !effective_mass_t1.is_finite()
        || effective_mass_t2 <= MIN_EFFECTIVE_INV_MASS || !effective_mass_t2.is_finite()
    {
        return;
    }

    let delta_t1 = -rel_vel.dot(&t1) / effective_mass_t1;
    let delta_t2 = -rel_vel.dot(&t2) / effective_mass_t2;

    let mut new_t1 = contact.accumulated_tangent_impulse[0] + delta_t1;
    let mut new_t2 = contact.accumulated_tangent_impulse[1] + delta_t2;

    let max_friction = header.friction * contact.accumulated_normal_impulse;
    let mag = (new_t1 * new_t1 + new_t2 * new_t2).sqrt();
    if mag > max_friction {
        let scale = max_friction / mag;
        new_t1 *= scale;
        new_t2 *= scale;
    }

    let applied_t1 = new_t1 - contact.accumulated_tangent_impulse[0];
    let applied_t2 = new_t2 - contact.accumulated_tangent_impulse[1];
    contact.accumulated_tangent_impulse = [new_t1, new_t2];

    if applied_t1.abs() > 1e-10 || applied_t2.abs() > 1e-10 {
        let impulse = t1 * applied_t1 + t2 * applied_t2;
        apply_impulse_pair(bodies, header, contact.point, impulse);
    }
}

// ---------------------------------------------------------------------------
// Impulse application
// ---------------------------------------------------------------------------

/// Apply equal-and-opposite impulses to both bodies in a contact pair.
fn apply_impulse_pair(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    point: Point3<f32>,
    impulse: Vector3<f32>,
) {
    if let Some(handle_a) = header.body_a {
        if let Some(body_a) = bodies.get_mut(handle_a.0) {
            if body_a.is_dynamic() {
                body_a.apply_impulse_at_point(-impulse, point);
            }
        }
    }

    if let Some(body_b) = bodies.get_mut(header.body_b.0) {
        if body_b.is_dynamic() {
            body_b.apply_impulse_at_point(impulse, point);
        } else if is_kinematic_static(body_b, header) {
            body_b.set_linear_velocity(body_b.linear_velocity() + impulse);
        }
    }
}
