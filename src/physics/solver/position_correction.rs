//! Position correction strategies: NGS (nonlinear Gauss-Seidel) and Baumgarte.
//!
//! Also includes contact damping (rolling resistance + linear damping).

use std::collections::{HashMap, HashSet};

use generational_arena::{Arena, Index};
use nalgebra::{Matrix3, Point3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::math::integrate_orientation;
use crate::physics::pipeline::pair::{PairHeader, SolverManifold};

use super::body_pair::is_kinematic_static;

/// Configuration for position correction and contact damping.
#[derive(Debug, Clone, Copy)]
pub struct PositionCorrectionConfig {
    /// Baumgarte position correction factor.
    pub baumgarte_factor: f32,
    /// Baumgarte slop for penetration correction.
    pub baumgarte_slop: f32,
    /// Enable NGS position correction instead of Baumgarte.
    pub ngs_enabled: bool,
    /// NGS correction factor for penetration bias.
    pub correction_factor: f32,
    /// NGS slop for penetration bias.
    pub slop: f32,
    /// Iterations for NGS position correction.
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
    /// Enable angular correction during NGS position solving.
    /// When false, only linear position corrections are applied.
    pub ngs_angular_correction: bool,
}

impl Default for PositionCorrectionConfig {
    fn default() -> Self {
        Self {
            baumgarte_factor: 0.3,
            baumgarte_slop: 0.005,
            ngs_enabled: true,
            correction_factor: 0.2,
            slop: 0.005,
            iterations: 3,
            max_correction_speed: 0.1,
            deep_correction_speed: 1.0,
            deep_threshold: 0.1,
            contact_rolling_resistance: 0.1,
            contact_linear_damping: 0.1,
            ngs_angular_correction: false,
        }
    }
}

/// Run all position correction passes after velocity solving.
///
/// 1. Penetration correction (Baumgarte or NGS)
/// 2. Contact rolling resistance
/// 3. Contact linear damping
pub(crate) fn apply_position_correction(
    bodies: &mut Arena<RigidBody>,
    manifolds: &[SolverManifold],
    config: &PositionCorrectionConfig,
    dt: f32,
    contact_generation_positions: &HashMap<Index, Point3<f32>>,
) {
    if config.ngs_enabled {
        apply_ngs_correction(
            bodies,
            manifolds,
            config.correction_factor,
            config.slop,
            config.iterations,
            config.max_correction_speed,
            config.deep_correction_speed,
            config.deep_threshold,
            config.ngs_angular_correction,
            dt,
            contact_generation_positions,
        );
    } else {
        apply_baumgarte_correction(bodies, manifolds, config, dt);
    }

    apply_contact_rolling_resistance(bodies, manifolds, config.contact_rolling_resistance, dt);
    apply_contact_linear_damping(bodies, manifolds, config.contact_linear_damping, dt);
}

// ---------------------------------------------------------------------------
// Baumgarte position correction
// ---------------------------------------------------------------------------

fn apply_baumgarte_correction(
    bodies: &mut Arena<RigidBody>,
    manifolds: &[SolverManifold],
    config: &PositionCorrectionConfig,
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
                apply_baumgarte_single(
                    bodies,
                    header,
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

fn apply_baumgarte_single(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
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
// NGS (nonlinear Gauss-Seidel) position correction
// ---------------------------------------------------------------------------

/// Corrected body transform tracked during NGS position solving.
///
/// Positions update immediately after each contact (true Gauss-Seidel),
/// so subsequent contacts within the same iteration see the corrected geometry.
#[derive(Debug, Clone)]
struct CorrectedTransform {
    /// Body position at the time contacts were generated.
    base_position: Point3<f32>,
    /// Current corrected position (updated after each contact).
    position: Point3<f32>,
    /// Current corrected rotation (updated after each contact).
    rotation: nalgebra::UnitQuaternion<f32>,
}

impl CorrectedTransform {
    fn new(
        base_position: Point3<f32>,
        position: Point3<f32>,
        rotation: nalgebra::UnitQuaternion<f32>,
    ) -> Self {
        Self {
            base_position,
            position,
            rotation,
        }
    }
}

fn apply_ngs_correction(
    bodies: &mut Arena<RigidBody>,
    manifolds: &[SolverManifold],
    correction_factor: f32,
    slop: f32,
    iterations: u32,
    max_correction_speed: f32,
    deep_correction_speed: f32,
    deep_threshold: f32,
    angular_correction: bool,
    dt: f32,
    contact_generation_positions: &HashMap<Index, Point3<f32>>,
) {
    let total_contacts: usize = manifolds.iter().map(|m| m.contacts.len()).sum();
    if total_contacts == 0 || iterations == 0 || dt <= 0.0 {
        return;
    }

    let max_correction = max_correction_speed * dt;
    let deep_correction = deep_correction_speed * dt;

    let mut transforms: HashMap<Index, CorrectedTransform> = HashMap::new();
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
                    let base_pos = contact_generation_positions
                        .get(&header.body_b.0)
                        .copied()
                        .unwrap_or_else(|| body_b.position());
                    let transform = transforms.entry(header.body_b.0).or_insert_with(|| {
                        CorrectedTransform::new(base_pos, body_b.position(), body_b.rotation())
                    });
                    let inv_mass = if is_kinematic_static(body_b, header) {
                        1.0
                    } else {
                        body_b.inv_mass()
                    };
                    (
                        transform.position,
                        transform.base_position,
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
                        let base_pos = contact_generation_positions
                            .get(&handle.0)
                            .copied()
                            .unwrap_or_else(|| body_a.position());
                        let transform = transforms.entry(handle.0).or_insert_with(|| {
                            CorrectedTransform::new(base_pos, body_a.position(), body_a.rotation())
                        });
                        (
                            transform.position,
                            transform.base_position,
                            body_a.inv_mass(),
                            body_a.world_inv_inertia(),
                        )
                    }
                    None => (contact.point, contact.point, 0.0, Matrix3::zeros()),
                };

                // Recompute separation from corrected positions
                let delta_a = pos_a - base_pos_a;
                let delta_b = pos_b - base_pos_b;
                let separation = contact.depth - (delta_b - delta_a).dot(&contact.normal) - slop;
                if separation <= 0.0 {
                    flat_idx += 1;
                    continue;
                }

                let r_a = contact.point - base_pos_a;
                let r_b = contact.point - base_pos_b;

                let r_a_cross_n = r_a.cross(&contact.normal);
                let r_b_cross_n = r_b.cross(&contact.normal);

                let effective_mass = if angular_correction {
                    let angular_effect_a = (inv_inertia_a * r_a_cross_n).cross(&r_a);
                    let angular_effect_b = (inv_inertia_b * r_b_cross_n).cross(&r_b);
                    inv_mass_a
                        + inv_mass_b
                        + (angular_effect_a + angular_effect_b).dot(&contact.normal)
                } else {
                    inv_mass_a + inv_mass_b
                };
                if effective_mass <= 0.0 {
                    flat_idx += 1;
                    continue;
                }

                // Position-level correction (no velocity, no inv_dt scaling)
                let mut correction = separation * correction_factor;
                if max_correction > 0.0 {
                    let effective_cap = if deep_threshold > slop && separation > slop {
                        let t = ((separation - slop) / (deep_threshold - slop)).min(1.0);
                        max_correction + t * (deep_correction - max_correction)
                    } else {
                        max_correction
                    };
                    correction = correction.min(effective_cap);
                }

                // Accumulated clamping (position-level impulse)
                let delta_impulse = correction / effective_mass;
                let old_impulse = accumulated[flat_idx];
                let new_impulse = (old_impulse + delta_impulse).max(0.0);
                let applied = new_impulse - old_impulse;
                accumulated[flat_idx] = new_impulse;

                if applied.abs() <= 1e-10 {
                    flat_idx += 1;
                    continue;
                }

                // Apply position and rotation corrections immediately (Gauss-Seidel)
                let impulse = contact.normal * applied;

                if let Some(handle_a) = header.body_a {
                    if inv_mass_a > 0.0 {
                        if let Some(entry) = transforms.get_mut(&handle_a.0) {
                            entry.position -= impulse * inv_mass_a;
                            if angular_correction {
                                let delta_angle = inv_inertia_a * r_a.cross(&-impulse);
                                entry.rotation =
                                    integrate_orientation(entry.rotation, delta_angle, 1.0);
                            }
                        }
                    }
                }

                if inv_mass_b > 0.0 {
                    if let Some(entry) = transforms.get_mut(&header.body_b.0) {
                        entry.position += impulse * inv_mass_b;
                        if angular_correction {
                            let delta_angle = inv_inertia_b * r_b.cross(&impulse);
                            entry.rotation =
                                integrate_orientation(entry.rotation, delta_angle, 1.0);
                        }
                    }
                }

                flat_idx += 1;
            }
        }
    }

    // Write corrected transforms back to bodies
    for (handle, transform) in transforms {
        if let Some(body) = bodies.get_mut(handle) {
            if body.is_static() {
                continue;
            }
            body.set_position(transform.position);
            body.set_rotation(transform.rotation);
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
