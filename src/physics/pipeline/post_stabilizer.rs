//! Post-stabilization corrections applied after velocity solving.
//!
//! Includes penetration correction (Baumgarte or split-impulse),
//! contact rolling resistance, and contact linear damping.

use std::collections::{HashMap, HashSet};

use generational_arena::{Arena, Index};
use nalgebra::{Matrix3, Point3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::math::integrate_orientation;
use crate::physics::pipeline::pair::{PairHeader, SolverManifold};

#[derive(Debug, Clone, Copy)]
pub struct PostStabiliseConfig {
    /// Baumgarte position correction factor.
    pub baumgarte_factor: f32,
    /// Baumgarte slop for penetration correction.
    pub baumgarte_slop: f32,
    /// Enable split-impulse post-stabilization instead of Baumgarte.
    pub split_impulse_enabled: bool,
    /// Split-impulse correction factor for penetration bias.
    pub correction_factor: f32,
    /// Split-impulse slop for penetration bias.
    pub slop: f32,
    /// Iterations for split-impulse post-stabilization.
    pub iterations: u32,
    /// Max correction speed (m/s) for shallow contacts. Applied when
    /// penetration depth is at or below `slop`, preventing micro-jitter.
    pub max_correction_speed: f32,
    /// Max correction speed (m/s) for deep penetrations. The effective
    /// cap scales linearly from `max_correction_speed` at `slop` to
    /// this value at `deep_threshold`.
    pub deep_correction_speed: f32,
    /// Penetration depth at which `deep_correction_speed` fully applies.
    /// Between `slop` and this value, the cap interpolates linearly.
    pub deep_threshold: f32,
    /// Rolling resistance factor applied to bodies with contacts.
    pub contact_rolling_resistance: f32,
    /// Linear damping factor applied to bodies with contacts.
    pub contact_linear_damping: f32,
}

impl Default for PostStabiliseConfig {
    fn default() -> Self {
        Self {
            baumgarte_factor: 0.05,
            baumgarte_slop: 0.005,
            split_impulse_enabled: true,
            correction_factor: 0.2,
            slop: 0.005,
            iterations: 4,
            max_correction_speed: 0.1,
            deep_correction_speed: 1.0,
            deep_threshold: 0.1,
            contact_rolling_resistance: 0.1,
            contact_linear_damping: 0.1,
        }
    }
}

/// Run all post-stabilization passes after velocity solving.
///
/// 1. Penetration correction (Baumgarte or split-impulse)
/// 2. Contact rolling resistance
/// 3. Contact linear damping
pub(crate) fn post_stabilize(
    bodies: &mut Arena<RigidBody>,
    manifolds: &[SolverManifold],
    config: &PostStabiliseConfig,
    dt: f32,
) {
    if config.split_impulse_enabled {
        apply_split_impulse_correction(
            bodies,
            manifolds,
            config.correction_factor,
            config.slop,
            config.iterations,
            config.max_correction_speed,
            config.deep_correction_speed,
            config.deep_threshold,
            dt,
        );
    } else {
        apply_baumgarte_correction(bodies, manifolds, config, dt);
    }

    apply_contact_rolling_resistance(bodies, manifolds, config.contact_rolling_resistance, dt);
    apply_contact_linear_damping(bodies, manifolds, config.contact_linear_damping, dt);
}

/// Returns true if body_b is kinematic and body_a is static geometry (None).
#[inline]
fn is_kinematic_static(body: &RigidBody, header: &PairHeader) -> bool {
    body.is_kinematic() && header.body_a.is_none()
}

// ---------------------------------------------------------------------------
// Baumgarte position correction
// ---------------------------------------------------------------------------

fn apply_baumgarte_correction(
    bodies: &mut Arena<RigidBody>,
    manifolds: &[SolverManifold],
    config: &PostStabiliseConfig,
    dt: f32,
) {
    for manifold in manifolds {
        let header = &manifold.header;
        for contact in &manifold.contacts {
            if contact.depth > 0.001 {
                let inv_mass_a = header
                    .body_a
                    .and_then(|h| bodies.get(h.0))
                    .map(|b| b.inv_mass())
                    .unwrap_or(0.0);
                let inv_mass_b = bodies
                    .get(header.body_b.0)
                    .map(|b| {
                        if is_kinematic_static(b, header) {
                            1.0
                        } else {
                            b.inv_mass()
                        }
                    })
                    .unwrap_or(0.0);
                let effective_cap = if config.deep_threshold > config.baumgarte_slop
                    && contact.depth > config.baumgarte_slop
                {
                    let t = ((contact.depth - config.baumgarte_slop)
                        / (config.deep_threshold - config.baumgarte_slop))
                        .min(1.0);
                    config.max_correction_speed
                        + t * (config.deep_correction_speed - config.max_correction_speed)
                } else {
                    config.max_correction_speed
                };
                apply_position_correction(
                    bodies,
                    header,
                    contact.point,
                    contact.normal,
                    contact.depth,
                    inv_mass_a,
                    inv_mass_b,
                    config.baumgarte_factor,
                    config.baumgarte_slop,
                    effective_cap,
                    dt,
                );
            }
        }
    }
}

fn apply_position_correction(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    point: Point3<f32>,
    normal: Vector3<f32>,
    depth: f32,
    inv_mass_a: f32,
    inv_mass_b: f32,
    correction_factor: f32,
    slop: f32,
    max_correction_speed: f32,
    dt: f32,
) {
    let total_inv_mass = inv_mass_a + inv_mass_b;
    if total_inv_mass <= 0.0 {
        return;
    }

    let _ = point; // position correction uses normal and depth only
    let mut correction = (depth - slop).max(0.0) * correction_factor / total_inv_mass;
    if max_correction_speed > 0.0 && dt > 0.0 {
        correction = correction.min(max_correction_speed * dt);
    }

    if let Some(handle_a) = header.body_a {
        if let Some(body_a) = bodies.get_mut(handle_a.0) {
            if body_a.is_dynamic() || body_a.is_kinematic() {
                let pos = body_a.position();
                body_a.set_position(pos - normal * correction * inv_mass_a);
            }
        }
    }

    if let Some(body_b) = bodies.get_mut(header.body_b.0) {
        if body_b.is_dynamic() || body_b.is_kinematic() {
            let pos = body_b.position();
            body_b.set_position(pos + normal * correction * inv_mass_b);
        }
    }
}

// ---------------------------------------------------------------------------
// Split-impulse position correction
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct PseudoState {
    /// Body position at the start of the split-impulse pass.
    base_position: Point3<f32>,
    /// Pseudo-updated position during split-impulse iterations.
    position: Point3<f32>,
    /// Pseudo-updated rotation during split-impulse iterations.
    rotation: nalgebra::UnitQuaternion<f32>,
    /// Pseudo linear velocity accumulator for the current iteration.
    linear: Vector3<f32>,
    /// Pseudo angular velocity accumulator for the current iteration.
    angular: Vector3<f32>,
}

impl PseudoState {
    fn new(position: Point3<f32>, rotation: nalgebra::UnitQuaternion<f32>) -> Self {
        Self {
            base_position: position,
            position,
            rotation,
            linear: Vector3::zeros(),
            angular: Vector3::zeros(),
        }
    }
}

fn apply_split_impulse_correction(
    bodies: &mut Arena<RigidBody>,
    manifolds: &[SolverManifold],
    correction_factor: f32,
    slop: f32,
    iterations: u32,
    max_correction_speed: f32,
    deep_correction_speed: f32,
    deep_threshold: f32,
    dt: f32,
) {
    let total_contacts: usize = manifolds.iter().map(|m| m.contacts.len()).sum();
    if total_contacts == 0 || iterations == 0 || dt <= 0.0 {
        return;
    }

    let inv_dt = 1.0 / dt;
    let mut pseudo_states: HashMap<Index, PseudoState> = HashMap::new();
    let mut accumulated: Vec<f32> = vec![0.0; total_contacts];

    for _ in 0..iterations {
        let mut flat_idx = 0;
        for manifold in manifolds {
            let header = &manifold.header;
            for contact in &manifold.contacts {
                let (pos_b, base_pos_b, inv_mass_b, inv_inertia_b) = {
                    let Some(body_b) = bodies.get(header.body_b.0) else {
                        flat_idx += 1;
                        continue;
                    };
                    let state = pseudo_states
                        .entry(header.body_b.0)
                        .or_insert_with(|| PseudoState::new(body_b.position(), body_b.rotation()));
                    let inv_mass = if is_kinematic_static(body_b, header) {
                        1.0
                    } else {
                        body_b.inv_mass()
                    };
                    (
                        state.position,
                        state.base_position,
                        inv_mass,
                        body_b.world_inv_inertia(),
                    )
                };

                let (pos_a, base_pos_a, inv_mass_a, inv_inertia_a) = match header.body_a {
                    Some(handle) => {
                        let Some(body_a) = bodies.get(handle.0) else {
                            flat_idx += 1;
                            continue;
                        };
                        let state = pseudo_states.entry(handle.0).or_insert_with(|| {
                            PseudoState::new(body_a.position(), body_a.rotation())
                        });
                        (
                            state.position,
                            state.base_position,
                            body_a.inv_mass(),
                            body_a.world_inv_inertia(),
                        )
                    }
                    None => (contact.point, contact.point, 0.0, Matrix3::zeros()),
                };

                let delta_a = pos_a - base_pos_a;
                let delta_b = pos_b - base_pos_b;
                let depth = contact.depth - (delta_b - delta_a).dot(&contact.normal) - slop;
                if depth <= 0.0 {
                    flat_idx += 1;
                    continue;
                }

                let r_a = contact.point - pos_a;
                let r_b = contact.point - pos_b;

                let (linear_a, angular_a) = header
                    .body_a
                    .and_then(|handle| pseudo_states.get(&handle.0))
                    .map(|state| (state.linear, state.angular))
                    .unwrap_or((Vector3::zeros(), Vector3::zeros()));
                let (linear_b, angular_b) = pseudo_states
                    .get(&header.body_b.0)
                    .map(|state| (state.linear, state.angular))
                    .unwrap_or((Vector3::zeros(), Vector3::zeros()));

                let vel_at_contact_a = linear_a + angular_a.cross(&r_a);
                let vel_at_contact_b = linear_b + angular_b.cross(&r_b);
                let rel_vel = vel_at_contact_b - vel_at_contact_a;
                let vel_along_normal = rel_vel.dot(&contact.normal);

                let r_a_cross_n = r_a.cross(&contact.normal);
                let r_b_cross_n = r_b.cross(&contact.normal);

                let angular_effect_a = (inv_inertia_a * r_a_cross_n).cross(&r_a);
                let angular_effect_b = (inv_inertia_b * r_b_cross_n).cross(&r_b);

                let effective_mass = inv_mass_a
                    + inv_mass_b
                    + (angular_effect_a + angular_effect_b).dot(&contact.normal);
                if effective_mass <= 0.0 {
                    flat_idx += 1;
                    continue;
                }

                let mut bias = -depth * correction_factor * inv_dt;
                if max_correction_speed > 0.0 {
                    let effective_cap = if deep_threshold > slop && depth > slop {
                        let t = ((depth - slop) / (deep_threshold - slop)).min(1.0);
                        max_correction_speed + t * (deep_correction_speed - max_correction_speed)
                    } else {
                        max_correction_speed
                    };
                    bias = bias.max(-effective_cap);
                }
                let delta_impulse = -(vel_along_normal + bias) / effective_mass;
                let old_impulse = accumulated[flat_idx];
                let new_impulse = (old_impulse + delta_impulse).max(0.0);
                let applied = new_impulse - old_impulse;
                accumulated[flat_idx] = new_impulse;

                if applied.abs() <= 1e-10 {
                    flat_idx += 1;
                    continue;
                }

                let impulse = contact.normal * applied;

                if let Some(handle_a) = header.body_a {
                    if inv_mass_a > 0.0 {
                        let entry = pseudo_states.entry(handle_a.0).or_insert_with(|| {
                            let body_a = bodies
                                .get(handle_a.0)
                                .map(|body| (body.position(), body.rotation()));
                            let (pos, rot) = body_a.unwrap_or((
                                contact.point,
                                nalgebra::UnitQuaternion::identity(),
                            ));
                            PseudoState::new(pos, rot)
                        });
                        entry.linear -= impulse * inv_mass_a;
                        let angular_impulse = r_a.cross(&-impulse);
                        entry.angular += inv_inertia_a * angular_impulse;
                    }
                }

                if inv_mass_b > 0.0 {
                    let entry = pseudo_states.entry(header.body_b.0).or_insert_with(|| {
                        let body_b = bodies
                            .get(header.body_b.0)
                            .map(|body| (body.position(), body.rotation()));
                        let (pos, rot) = body_b.unwrap_or((
                            contact.point,
                            nalgebra::UnitQuaternion::identity(),
                        ));
                        PseudoState::new(pos, rot)
                    });
                    entry.linear += impulse * inv_mass_b;
                    let angular_impulse = r_b.cross(&impulse);
                    entry.angular += inv_inertia_b * angular_impulse;
                }

                flat_idx += 1;
            }
        }

        for state in pseudo_states.values_mut() {
            if state.linear.magnitude_squared() < 1e-12
                && state.angular.magnitude_squared() < 1e-12
            {
                continue;
            }
            state.position += state.linear * dt;
            state.rotation = integrate_orientation(state.rotation, state.angular, dt);
            state.linear = Vector3::zeros();
            state.angular = Vector3::zeros();
        }
    }

    for (handle, state) in pseudo_states {
        if let Some(body) = bodies.get_mut(handle) {
            if body.is_static() {
                continue;
            }
            body.set_position(state.position);
            body.set_rotation(state.rotation);
        }
    }
}

// ---------------------------------------------------------------------------
// Contact damping
// ---------------------------------------------------------------------------

fn apply_contact_rolling_resistance(
    bodies: &mut Arena<RigidBody>,
    manifolds: &[SolverManifold],
    rolling_resistance: f32,
    dt: f32,
) {
    let total_contacts: usize = manifolds.iter().map(|m| m.contacts.len()).sum();
    if total_contacts == 0 || rolling_resistance <= 0.0 || dt <= 0.0 {
        return;
    }

    let factor = (1.0 - rolling_resistance * dt).clamp(0.0, 1.0);
    if factor >= 1.0 {
        return;
    }

    let mut impacted: HashSet<RigidBodyHandle> = HashSet::new();
    for manifold in manifolds {
        if let Some(handle_a) = manifold.header.body_a {
            impacted.insert(handle_a);
        }
        impacted.insert(manifold.header.body_b);
    }

    for handle in impacted {
        if let Some(body) = bodies.get_mut(handle.0) {
            if body.is_dynamic() {
                body.set_angular_velocity(body.angular_velocity() * factor);
            }
        }
    }
}

fn apply_contact_linear_damping(
    bodies: &mut Arena<RigidBody>,
    manifolds: &[SolverManifold],
    linear_damping: f32,
    dt: f32,
) {
    let total_contacts: usize = manifolds.iter().map(|m| m.contacts.len()).sum();
    if total_contacts == 0 || linear_damping <= 0.0 || dt <= 0.0 {
        return;
    }

    let factor = (1.0 - linear_damping * dt).clamp(0.0, 1.0);
    if factor >= 1.0 {
        return;
    }

    let mut impacted: HashSet<RigidBodyHandle> = HashSet::new();
    for manifold in manifolds {
        if let Some(handle_a) = manifold.header.body_a {
            impacted.insert(handle_a);
        }
        impacted.insert(manifold.header.body_b);
    }

    for handle in impacted {
        if let Some(body) = bodies.get_mut(handle.0) {
            if body.is_dynamic() {
                body.set_linear_velocity(body.linear_velocity() * factor);
            }
        }
    }
}
