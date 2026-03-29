//! Position correction: NGS (nonlinear Gauss-Seidel) for contacts and constraints.
//!
//! Also includes contact damping (rolling resistance + linear damping).

use std::collections::{HashMap, HashSet};

use generational_arena::{Arena, Index};
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::constraint::types::{Constraint, ConstraintKind};
use crate::physics::handle::RigidBodyHandle;
use crate::physics::math::integrate_orientation;
use crate::physics::pipeline::pair::SolverManifold;

use super::body_pair::is_kinematic_static;

/// Configuration for NGS position correction and contact damping.
#[derive(Debug, Clone, Copy)]
pub struct PositionCorrectionConfig {
    /// NGS correction factor for contact penetration.
    pub correction_factor: f32,
    /// Penetration slop — contacts shallower than this are not corrected.
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
    /// Enable angular correction during NGS contact position solving.
    /// When false, only linear position corrections are applied.
    ///
    /// NOTE: Angular NGS correction is unstable in practice — for both contacts
    /// and constraints. Constraint NGS intentionally applies linear-only
    /// corrections for the same reason. Angular drift in weld/follow-point
    /// constraints is handled by the PGS velocity rows (which resist relative
    /// angular velocity) and by hard velocity projection (KeepUpright). If
    /// you're tempted to add angular NGS for constraints, test thoroughly with
    /// the bench harness — previous attempts caused oscillation and energy
    /// growth.
    pub ngs_angular_correction: bool,
    /// NGS correction factor for constraint drift (weld, follow-point).
    /// Higher than `correction_factor` because rigid constraints need
    /// aggressive correction — unlike contacts, there is no overshoot risk.
    pub constraint_correction_factor: f32,
}

impl Default for PositionCorrectionConfig {
    fn default() -> Self {
        Self {
            correction_factor: 0.2,
            slop: 0.005,
            iterations: 3,
            max_correction_speed: 0.1,
            deep_correction_speed: 1.0,
            deep_threshold: 0.1,
            contact_rolling_resistance: 0.1,
            contact_linear_damping: 0.1,
            ngs_angular_correction: false,
            constraint_correction_factor: 0.2,
        }
    }
}

/// Run all position correction passes after velocity solving.
///
/// 1. NGS penetration + constraint drift correction
/// 2. Contact rolling resistance
/// 3. Contact linear damping
pub(crate) fn apply_position_correction(
    bodies: &mut Arena<RigidBody>,
    manifolds: &[SolverManifold],
    constraints: &Arena<Constraint>,
    config: &PositionCorrectionConfig,
    dt: f32,
    contact_generation_positions: &HashMap<Index, Point3<f32>>,
) {
    apply_ngs_correction(
        bodies,
        manifolds,
        constraints,
        config.correction_factor,
        config.constraint_correction_factor,
        config.slop,
        config.iterations,
        config.max_correction_speed,
        config.deep_correction_speed,
        config.deep_threshold,
        config.ngs_angular_correction,
        dt,
        contact_generation_positions,
    );

    apply_contact_rolling_resistance(bodies, manifolds, config.contact_rolling_resistance, dt);
    apply_contact_linear_damping(bodies, manifolds, config.contact_linear_damping, dt);
}

// ---------------------------------------------------------------------------
// Baumgarte position correction
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
    constraints: &Arena<Constraint>,
    correction_factor: f32,
    constraint_correction_factor: f32,
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
    let has_constraints = constraints.iter().any(|(_, c)| c.active);
    if (total_contacts == 0 && !has_constraints) || iterations == 0 || dt <= 0.0 {
        return;
    }

    let max_correction = max_correction_speed * dt;
    let deep_correction = deep_correction_speed * dt;

    let mut transforms: HashMap<Index, CorrectedTransform> = HashMap::new();
    let mut accumulated: Vec<f32> = vec![0.0; total_contacts];

    for _ in 0..iterations {
        // --- Contact position corrections ---
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

        // --- Constraint position corrections ---
        correct_constraint_drift(
            bodies,
            constraints,
            constraint_correction_factor,
            &mut transforms,
        );
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

/// Apply direct position/rotation corrections for weld and follow-point constraints.
///
/// Reads live body transforms (from the `transforms` map, falling back to the
/// body arena) and applies mass-weighted corrections that reduce anchor drift
/// and angular error without injecting velocity.
fn correct_constraint_drift(
    bodies: &Arena<RigidBody>,
    constraints: &Arena<Constraint>,
    correction_factor: f32,
    transforms: &mut HashMap<Index, CorrectedTransform>,
) {
    for (_index, constraint) in constraints.iter() {
        if !constraint.active {
            continue;
        }

        match &constraint.kind {
            ConstraintKind::FollowPoint {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                compliance,
                ..
            } => {
                let linear_factor = correction_factor / (1.0 + compliance);
                correct_follow_point_drift(
                    bodies,
                    body_a.0,
                    body_b.0,
                    local_anchor_a,
                    local_anchor_b,
                    linear_factor,
                    transforms,
                );
            }

            ConstraintKind::KeepUpright { .. } => {
                // No NGS — handled by hard velocity projection.
            }
        }
    }
}

/// Get the current (corrected) position and rotation for a body.
fn get_corrected_transform(
    bodies: &Arena<RigidBody>,
    handle: Index,
    transforms: &mut HashMap<Index, CorrectedTransform>,
) -> Option<(Point3<f32>, UnitQuaternion<f32>, f32, Matrix3<f32>)> {
    let body = bodies.get(handle)?;
    let transform = transforms.entry(handle).or_insert_with(|| {
        CorrectedTransform::new(body.position(), body.position(), body.rotation())
    });
    Some((
        transform.position,
        transform.rotation,
        body.inv_mass(),
        body.world_inv_inertia(),
    ))
}

fn correct_follow_point_drift(
    bodies: &Arena<RigidBody>,
    handle_a: Index,
    handle_b: Index,
    local_anchor_a: &Vector3<f32>,
    local_anchor_b: &Vector3<f32>,
    linear_factor: f32,
    transforms: &mut HashMap<Index, CorrectedTransform>,
) {
    let Some((pos_a, rot_a, inv_mass_a, _)) =
        get_corrected_transform(bodies, handle_a, transforms)
    else {
        return;
    };
    let Some((pos_b, rot_b, inv_mass_b, _)) =
        get_corrected_transform(bodies, handle_b, transforms)
    else {
        return;
    };

    let anchor_a = pos_a + rot_a * local_anchor_a;
    let anchor_b = pos_b + rot_b * local_anchor_b;
    let error = anchor_b - anchor_a;

    if error.norm_squared() > 1e-14 {
        let inv_mass_sum = inv_mass_a + inv_mass_b;
        if inv_mass_sum > 0.0 {
            let correction = error * linear_factor;
            if let Some(t) = transforms.get_mut(&handle_a) {
                t.position += correction * (inv_mass_a / inv_mass_sum);
            }
            if let Some(t) = transforms.get_mut(&handle_b) {
                t.position -= correction * (inv_mass_b / inv_mass_sum);
            }
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
