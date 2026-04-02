//! Warm-starting and adaptive iteration count for PGS solvers.

use rustc_hash::FxHashMap;

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::{PairHeader, SolverContact, SolverManifold};

use super::impulse::apply_impulse_pair;

/// Apply cached impulse for a single contact and initialize its accumulated impulses.
pub(crate) fn warm_start_contact(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    contact: &mut SolverContact,
    scale: f32,
    shock_scales: (f32, f32),
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

    apply_impulse_pair(bodies, header, contact.point, total, shock_scales);
}

/// Compute effective solver iteration count based on contact complexity.
///
/// Bodies with many contacts or divergent normals get extra iterations to
/// improve convergence.
pub(crate) fn effective_solver_iterations(
    manifolds: &[SolverManifold],
    base_iterations: u32,
) -> u32 {
    let mut per_body_counts: FxHashMap<RigidBodyHandle, usize> = FxHashMap::default();
    let mut per_body_normals: FxHashMap<RigidBodyHandle, Vec<Vector3<f32>>> = FxHashMap::default();
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
