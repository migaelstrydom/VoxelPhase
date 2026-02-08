//! Constraint solver for contact resolution.

use generational_arena::Arena;
use nalgebra::{Matrix3, Point3, Vector3};

use crate::physics::body::RigidBody;
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
    pub point: Point3<f32>,
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

/// Snapshot of both bodies' physics state for a single contact.
///
/// Stores the real physics values — no kinematic overrides. Callers that
/// need the kinematic-static treatment (normal impulse) apply the override
/// via `effective_mass_with_overrides`.
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
    /// Extract physics state for both sides of a contact.
    ///
    /// Returns `None` if either body handle is stale (removed from the arena).
    fn extract(bodies: &Arena<RigidBody>, contact: &ContactConstraint) -> Option<Self> {
        let body_b = bodies.get(contact.body_b.0)?;

        let (pos_a, vel_a, angular_vel_a, inv_mass_a, inv_inertia_a) = match contact.body_a {
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
                contact.point,
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
    fn relative_normal_velocity(&self, contact: &ContactConstraint) -> f32 {
        let rel_vel = self.relative_velocity_at_contact(contact);
        rel_vel.dot(&contact.normal)
    }

    /// Relative velocity at the contact point (B minus A).
    fn relative_velocity_at_contact(&self, contact: &ContactConstraint) -> Vector3<f32> {
        let r_a = contact.point - self.pos_a;
        let r_b = contact.point - self.pos_b;
        let vel_at_a = self.vel_a + self.angular_vel_a.cross(&r_a);
        let vel_at_b = self.vel_b + self.angular_vel_b.cross(&r_b);
        vel_at_b - vel_at_a
    }

    /// Compute the effective mass for an impulse along the given direction.
    fn effective_mass(&self, contact: &ContactConstraint, direction: &Vector3<f32>) -> f32 {
        self.effective_mass_with_overrides(
            contact,
            direction,
            self.inv_mass_b,
            self.inv_inertia_b,
        )
    }

    /// Compute effective mass with explicit body_b overrides.
    ///
    /// Used by the normal solver to treat kinematic-vs-static contacts
    /// as having unit mass, while friction uses the real (zero) mass so
    /// kinematic bodies aren't slowed by friction against static geometry.
    fn effective_mass_with_overrides(
        &self,
        contact: &ContactConstraint,
        direction: &Vector3<f32>,
        inv_mass_b: f32,
        inv_inertia_b: Matrix3<f32>,
    ) -> f32 {
        let r_a = contact.point - self.pos_a;
        let r_b = contact.point - self.pos_b;

        let r_a_cross = r_a.cross(direction);
        let r_b_cross = r_b.cross(direction);

        let angular_a = (self.inv_inertia_a * r_a_cross).cross(&r_a);
        let angular_b = (inv_inertia_b * r_b_cross).cross(&r_b);

        self.inv_mass_a + inv_mass_b + (angular_a + angular_b).dot(direction)
    }
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
        .map(|contact| {
            BodyPairState::extract(bodies, contact)
                .map(|state| state.relative_normal_velocity(contact))
                .unwrap_or(0.0)
        })
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
            solve_normal_impulse(
                bodies,
                contact,
                config.restitution_velocity_threshold,
                pre_solve_vn[i],
                config.restitution_depth_slop,
                &mut accumulated[i],
            );
            solve_friction_impulse(bodies, contact, &mut accumulated[i]);
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

        let normal_impulse = contact.normal * (contact.warm_normal_impulse * *scale);
        let tangent_impulse = compute_tangent_impulse(contact) * *scale;
        let total = normal_impulse + tangent_impulse;

        apply_impulse_pair(bodies, contact, total);
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
        let pre_solve_vn = BodyPairState::extract(bodies, contact)
            .map(|state| state.relative_normal_velocity(contact))
            .unwrap_or(0.0);
        solve_normal_impulse(
            bodies,
            contact,
            restitution_velocity_threshold,
            pre_solve_vn,
            restitution_depth_slop,
            &mut accumulated,
        );
        solve_friction_impulse(bodies, contact, &mut accumulated);
    }
}

// ---------------------------------------------------------------------------
// Normal impulse resolution
// ---------------------------------------------------------------------------

fn solve_normal_impulse(
    bodies: &mut Arena<RigidBody>,
    contact: &ContactConstraint,
    restitution_velocity_threshold: f32,
    pre_solve_vn: f32,
    restitution_depth_slop: f32,
    accumulated: &mut SolvedImpulses,
) {
    let Some(state) = BodyPairState::extract(bodies, contact) else {
        return;
    };

    let vel_along_normal = state.relative_normal_velocity(contact);
    if vel_along_normal > 0.0 {
        return;
    }

    // Kinematic-vs-static contacts use fake unit mass so the normal solver
    // can push the kinematic body out of static geometry. Friction does NOT
    // use this override — kinematic bodies should move freely along surfaces.
    let kinematic_static = bodies
        .get(contact.body_b.0)
        .is_some_and(|b| is_kinematic_static_contact(b, contact));
    let (eff_inv_mass_b, eff_inv_inertia_b) = if kinematic_static {
        (1.0, Matrix3::zeros())
    } else {
        (state.inv_mass_b, state.inv_inertia_b)
    };

    let effective_mass = state.effective_mass_with_overrides(
        contact,
        &contact.normal,
        eff_inv_mass_b,
        eff_inv_inertia_b,
    );
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

    let delta = -(vel_along_normal + restitution_velocity) / effective_mass;
    let old = accumulated.normal;
    let new = (old + delta).max(0.0);
    let applied = new - old;
    accumulated.normal = new;

    if applied.abs() > 1e-10 {
        let impulse = contact.normal * applied;
        apply_impulse_pair(bodies, contact, impulse);
    }
}

// ---------------------------------------------------------------------------
// Friction impulse resolution
// ---------------------------------------------------------------------------

fn solve_friction_impulse(
    bodies: &mut Arena<RigidBody>,
    contact: &ContactConstraint,
    accumulated: &mut SolvedImpulses,
) {
    if accumulated.normal <= 0.0 || contact.friction <= 0.0 {
        accumulated.tangent = [0.0, 0.0];
        return;
    }

    let Some(state) = BodyPairState::extract(bodies, contact) else {
        return;
    };

    let rel_vel = state.relative_velocity_at_contact(contact);
    let (t1, t2) = compute_tangent_basis(&contact.normal);

    let effective_mass_t1 = state.effective_mass(contact, &t1);
    let effective_mass_t2 = state.effective_mass(contact, &t2);
    if effective_mass_t1 <= 0.0 || effective_mass_t2 <= 0.0 {
        return;
    }

    let delta_t1 = -rel_vel.dot(&t1) / effective_mass_t1;
    let delta_t2 = -rel_vel.dot(&t2) / effective_mass_t2;

    let mut new_t1 = accumulated.tangent[0] + delta_t1;
    let mut new_t2 = accumulated.tangent[1] + delta_t2;

    let max_friction = contact.friction * accumulated.normal;
    let mag = (new_t1 * new_t1 + new_t2 * new_t2).sqrt();
    if mag > max_friction {
        let scale = max_friction / mag;
        new_t1 *= scale;
        new_t2 *= scale;
    }

    let applied_t1 = new_t1 - accumulated.tangent[0];
    let applied_t2 = new_t2 - accumulated.tangent[1];
    accumulated.tangent = [new_t1, new_t2];

    if applied_t1.abs() > 1e-10 || applied_t2.abs() > 1e-10 {
        let impulse = t1 * applied_t1 + t2 * applied_t2;
        apply_impulse_pair(bodies, contact, impulse);
    }
}

// ---------------------------------------------------------------------------
// Impulse application
// ---------------------------------------------------------------------------

/// Apply equal-and-opposite impulses to both bodies in a contact pair.
fn apply_impulse_pair(
    bodies: &mut Arena<RigidBody>,
    contact: &ContactConstraint,
    impulse: Vector3<f32>,
) {
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
        } else if is_kinematic_static_contact(body_b, contact) {
            body_b.set_linear_velocity(body_b.linear_velocity() + impulse);
        }
    }
}
