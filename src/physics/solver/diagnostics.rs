//! Solver diagnostic logging, gated by environment variables.

use std::sync::OnceLock;

use nalgebra::Vector3;

use crate::physics::pipeline::pair::{PairHeader, SolverContact};

use super::body_pair::BodyPairState;

pub(crate) fn solver_diag_enabled() -> bool {
    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| {
        std::env::var("VOXEL_PHASE_SOLVER_DIAG")
            .map(|v| matches!(v.as_str(), "1" | "true" | "TRUE" | "yes" | "YES"))
            .unwrap_or(false)
    })
}

fn solver_diag_body_filter() -> Option<usize> {
    static BODY_FILTER: OnceLock<Option<usize>> = OnceLock::new();
    *BODY_FILTER.get_or_init(|| {
        std::env::var("VOXEL_PHASE_SOLVER_DIAG_BODY_INDEX")
            .ok()
            .and_then(|v| v.parse::<usize>().ok())
    })
}

pub(crate) fn solver_diag_pair_includes_filtered_body(header: &PairHeader) -> bool {
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

pub(crate) fn log_impulse_torque_diag(
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
