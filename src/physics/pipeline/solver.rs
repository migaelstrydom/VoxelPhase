//! Constraint solver for contact resolution.

use generational_arena::Arena;
use nalgebra::{Matrix3, Point3, Vector3};
use smallvec::SmallVec;
use std::collections::HashMap;
use std::sync::OnceLock;

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

/// Number of PGS micro-iterations for block normal solve on multi-contact
/// manifolds. Extra local iterations capture cross-contact coupling within
/// a single outer solver pass, reducing rocking in stacks and eccentric loads.
const BLOCK_NORMAL_MICRO_ITERATIONS: u32 = 4;

fn solver_diag_enabled() -> bool {
    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| {
        std::env::var("RUST_DUDE_SOLVER_DIAG")
            .map(|v| matches!(v.as_str(), "1" | "true" | "TRUE" | "yes" | "YES"))
            .unwrap_or(false)
    })
}

fn solver_diag_body_filter() -> Option<usize> {
    static BODY_FILTER: OnceLock<Option<usize>> = OnceLock::new();
    *BODY_FILTER.get_or_init(|| {
        std::env::var("RUST_DUDE_SOLVER_DIAG_BODY_INDEX")
            .ok()
            .and_then(|v| v.parse::<usize>().ok())
    })
}

fn solver_diag_pair_includes_filtered_body(header: &PairHeader) -> bool {
    let Some(filter_idx) = solver_diag_body_filter() else {
        return true;
    };
    let body_b_idx = header.body_b.raw_parts().0;
    if body_b_idx == filter_idx {
        return true;
    }
    header
        .body_a
        .map(|h| h.raw_parts().0 == filter_idx)
        .unwrap_or(false)
}

fn log_impulse_torque_diag(
    kind: &str,
    header: &PairHeader,
    contact: &SolverContact,
    state: &BodyPairState,
    impulse_to_b: &Vector3<f32>,
) {
    if !solver_diag_enabled() || !solver_diag_pair_includes_filtered_body(header) {
        return;
    }
    let j_mag = impulse_to_b.magnitude();
    if j_mag <= 1.0e-6 {
        return;
    }

    let pair_kind = if header.body_a.is_none() {
        "static-dynamic"
    } else {
        "dynamic-dynamic"
    };
    let r_b = contact.point - state.pos_b;
    let tau_b = r_b.cross(impulse_to_b);
    let tangent_mag = contact.accumulated_friction_impulse_ws.magnitude();

    if let Some(handle_a) = header.body_a {
        let r_a = contact.point - state.pos_a;
        let tau_a = r_a.cross(&(-*impulse_to_b));
        eprintln!(
            "solver_diag impulse kind={kind} pair={pair_kind} a={:?} b={:?} feature={:?} \
             depth={:.5} j={:.6} jn_acc={:.6} jt_acc={:.6} \
             tau_a=[{:.6},{:.6},{:.6}] tau_b=[{:.6},{:.6},{:.6}]",
            handle_a,
            header.body_b,
            contact.feature_id,
            contact.depth,
            j_mag,
            contact.accumulated_normal_impulse,
            tangent_mag,
            tau_a.x,
            tau_a.y,
            tau_a.z,
            tau_b.x,
            tau_b.y,
            tau_b.z,
        );
    } else {
        eprintln!(
            "solver_diag impulse kind={kind} pair={pair_kind} a=static b={:?} feature={:?} \
             depth={:.5} j={:.6} jn_acc={:.6} jt_acc={:.6} \
             tau_b=[{:.6},{:.6},{:.6}]",
            header.body_b,
            contact.feature_id,
            contact.depth,
            j_mag,
            contact.accumulated_normal_impulse,
            tangent_mag,
            tau_b.x,
            tau_b.y,
            tau_b.z,
        );
    }
}

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
    //
    // Persisted contacts (warm_normal_impulse > 0) always get warm-started
    // regardless of approach velocity. This prevents velocity-driven bodies
    // (e.g. player characters) from having warm-start disabled every frame
    // due to their externally-set velocity exceeding the threshold.
    let pre_solve: Vec<Vec<(f32, f32)>> = manifolds
        .iter()
        .map(|m| {
            m.contacts
                .iter()
                .map(|c| {
                    let vn = BodyPairState::extract(bodies, &m.header, c.point)
                        .map(|s| s.relative_normal_velocity(c.point, &c.normal))
                        .unwrap_or(0.0);
                    let is_persisted = c.warm_normal_impulse > 0.0;
                    let warm_scale = if is_persisted {
                        config.warm_start_scale
                    } else if vn.abs() > config.restitution_velocity_threshold {
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
            // Block normal solve: multi-contact manifolds get extra local
            // iterations to capture cross-contact coupling.
            let normal_passes = if manifold.contacts.len() > 1 {
                BLOCK_NORMAL_MICRO_ITERATIONS
            } else {
                1
            };
            for _ in 0..normal_passes {
                for ci in 0..manifold.contacts.len() {
                    let is_persisted = manifold.contacts[ci].warm_normal_impulse > 0.0;
                    solve_normal_impulse(
                        bodies,
                        &manifold.header,
                        &mut manifold.contacts[ci],
                        config.restitution_velocity_threshold,
                        pre_solve[mi][ci].0,
                        is_persisted,
                    );
                }
            }

            // Per-contact friction solve
            for ci in 0..manifold.contacts.len() {
                solve_friction_impulse(bodies, &manifold.header, &mut manifold.contacts[ci]);
            }

            // Manifold-level friction budget projection: prevent individual
            // contacts from fighting each other with maxed-out friction cones.
            if manifold.contacts.len() > 1 {
                manifold_friction_projection(bodies, &manifold.header, &mut manifold.contacts);
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

    // Project cached world-space friction onto the current tangent plane
    // so that small normal drift does not rotate the friction direction.
    let warm_friction_scaled = contact.warm_friction_impulse_ws * scale;
    let projected =
        warm_friction_scaled - contact.normal * warm_friction_scaled.dot(&contact.normal);

    // Clamp to Coulomb limit with the warm normal impulse
    let mag = projected.magnitude();
    let max_friction = header.friction * contact.accumulated_normal_impulse;
    let friction_ws = if mag > max_friction && mag > 1e-8 {
        projected * (max_friction / mag)
    } else {
        projected
    };

    contact.accumulated_friction_impulse_ws = friction_ws;

    if scale <= 0.0 {
        return;
    }
    if contact.warm_normal_impulse.abs() < 1e-8
        && contact.warm_friction_impulse_ws.magnitude_squared() < 1e-16
    {
        return;
    }

    let normal_impulse = contact.normal * contact.accumulated_normal_impulse;
    let total = normal_impulse + friction_ws;

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
                false,
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
    is_persisted: bool,
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

    // Persisted contacts suppress restitution: the high approach velocity
    // is from an external velocity drive, not a new impact.
    let restitution = if is_persisted {
        0.0
    } else {
        let speed = pre_solve_vn.abs();
        let restitution_scale = ((speed - restitution_velocity_threshold)
            / restitution_velocity_threshold)
            .clamp(0.0, 1.0);
        header.restitution * restitution_scale
    };
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
        log_impulse_torque_diag("normal", header, contact, &state, &impulse);
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
        contact.accumulated_friction_impulse_ws = Vector3::zeros();
        return;
    }

    let Some(state) = BodyPairState::extract(bodies, header, contact.point) else {
        return;
    };

    let rel_vel = state.relative_velocity_at(contact.point);
    let (t1, t2) = compute_tangent_basis(&contact.normal);

    let effective_mass_t1 = state.effective_inv_mass(contact.point, &t1);
    let effective_mass_t2 = state.effective_inv_mass(contact.point, &t2);
    if effective_mass_t1 <= MIN_EFFECTIVE_INV_MASS
        || !effective_mass_t1.is_finite()
        || effective_mass_t2 <= MIN_EFFECTIVE_INV_MASS
        || !effective_mass_t2.is_finite()
    {
        return;
    }

    // Decompose world-space accumulator into current tangent basis
    let curr_t1 = contact.accumulated_friction_impulse_ws.dot(&t1);
    let curr_t2 = contact.accumulated_friction_impulse_ws.dot(&t2);

    let delta_t1 = -rel_vel.dot(&t1) / effective_mass_t1;
    let delta_t2 = -rel_vel.dot(&t2) / effective_mass_t2;

    let mut new_t1 = curr_t1 + delta_t1;
    let mut new_t2 = curr_t2 + delta_t2;

    let max_friction = header.friction * contact.accumulated_normal_impulse;
    let mag = (new_t1 * new_t1 + new_t2 * new_t2).sqrt();
    if mag > max_friction {
        let scale = max_friction / mag;
        new_t1 *= scale;
        new_t2 *= scale;
    }

    let applied_t1 = new_t1 - curr_t1;
    let applied_t2 = new_t2 - curr_t2;

    // Reconstruct world-space accumulator from current basis
    contact.accumulated_friction_impulse_ws = t1 * new_t1 + t2 * new_t2;

    if applied_t1.abs() > 1e-10 || applied_t2.abs() > 1e-10 {
        let impulse = t1 * applied_t1 + t2 * applied_t2;
        log_impulse_torque_diag("friction", header, contact, &state, &impulse);
        apply_impulse_pair(bodies, header, contact.point, impulse);
    }
}

// ---------------------------------------------------------------------------
// Manifold-level friction budget projection (Stage 2)
// ---------------------------------------------------------------------------

/// Enforce a shared friction budget across all contacts in a manifold.
///
/// After per-contact friction solve, the total friction effort (sum of individual
/// friction impulse magnitudes) must not exceed `mu * sum(lambda_n)`. If it does,
/// all contacts' friction impulses are scaled down proportionally. This prevents
/// individual contacts from each maxing out their friction cones and producing
/// oscillating net torque.
fn manifold_friction_projection(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    contacts: &mut SmallVec<[SolverContact; 4]>,
) {
    let total_normal: f32 = contacts.iter().map(|c| c.accumulated_normal_impulse).sum();
    if total_normal <= 0.0 || header.friction <= 0.0 {
        return;
    }

    let budget = header.friction * total_normal;
    let total_friction_mag: f32 = contacts
        .iter()
        .map(|c| c.accumulated_friction_impulse_ws.magnitude())
        .sum();

    if total_friction_mag <= budget || total_friction_mag < 1e-8 {
        return;
    }

    let scale = budget / total_friction_mag;
    for contact in contacts.iter_mut() {
        let old = contact.accumulated_friction_impulse_ws;
        contact.accumulated_friction_impulse_ws = old * scale;
        let delta = contact.accumulated_friction_impulse_ws - old;
        if delta.magnitude_squared() > 1e-20 {
            apply_impulse_pair(bodies, header, contact.point, delta);
        }
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
