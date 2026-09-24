//! Backward Euler for one group of stores, by Newton's method.
//!
//! Each store solves `V' = V + dt · (Σ Q_in(V') − Σ Q_out(V') − Q_loss(V'))`.
//! Every outflow and loss law rises with its own store's volume, so the step
//! is unconditionally stable: a large gain costs accuracy, as lag, never
//! stability. Stores upstream are solved first, so their outflow at `V'` is
//! already known; the only cycles are reversible links between basins, and a
//! cycle's stores are solved together.

use crate::water::ids::StoreId;
use crate::water::network::{LossLaw, Network, Port, StoreView};

/// Newton iterations per group per tick.
pub const NEWTON_STEPS: usize = 3;

/// Solve one group for its new volumes. `volumes` holds every store's
/// volume: already-solved stores at their new value, this group's at its
/// start-of-tick value on entry and its solved value on return.
pub fn solve_group(
    network: &Network,
    loss: &LossLaw,
    group: &[StoreId],
    start: &[f64],
    volumes: &mut [f64],
    dt: f64,
) {
    let n = group.len();
    let position = |id: StoreId| group.iter().position(|g| *g == id);
    for _ in 0..NEWTON_STEPS {
        let mut residual = vec![0.0f64; n];
        let mut jacobian = vec![0.0f64; n * n];
        for (i, &id) in group.iter().enumerate() {
            let store = network.store(id).expect("group stores are live");
            let v = volumes[id.0 as usize];
            let (lost, d_lost) = loss.loss(store, v);
            residual[i] = v - start[id.0 as usize] + dt * lost;
            jacobian[i * n + i] += 1.0 + dt * d_lost;
        }
        for (_, link) in network.flowing_links() {
            let (up_i, down_i) = (position(link.up), position(link.down));
            if up_i.is_none() && down_i.is_none() {
                continue;
            }
            let up = view(network, link.up, link.up_port, volumes);
            let down = view(network, link.down, link.down_port, volumes);
            let q = link.law.discharge(up, down);
            let (dq_up, dq_down) = link.law.jacobian(up, down);
            if let Some(u) = up_i {
                residual[u] += dt * q;
                jacobian[u * n + u] += dt * dq_up;
                if let Some(d) = down_i {
                    jacobian[u * n + d] += dt * dq_down;
                }
            }
            if let Some(d) = down_i {
                residual[d] -= dt * q;
                jacobian[d * n + d] -= dt * dq_down;
                if let Some(u) = up_i {
                    jacobian[d * n + u] -= dt * dq_up;
                }
            }
        }
        let Some(step) = solve_dense(&mut jacobian, &mut residual, n) else {
            return;
        };
        for (i, &id) in group.iter().enumerate() {
            let v = &mut volumes[id.0 as usize];
            *v = (*v - step[i]).max(0.0);
        }
    }
}

/// A store seen at the volume the solve currently holds for it.
pub fn view<'a>(network: &'a Network, id: StoreId, port: Port, volumes: &[f64]) -> StoreView<'a> {
    let store = network.store(id).expect("links join live stores");
    let volume = if store.is_finite() {
        volumes[id.0 as usize]
    } else {
        store.volume()
    };
    StoreView {
        store,
        volume,
        port,
    }
}

/// Solve `a · x = b` for a small dense system by Gaussian elimination with
/// partial pivoting. `None` if singular.
fn solve_dense(a: &mut [f64], b: &mut [f64], n: usize) -> Option<Vec<f64>> {
    for col in 0..n {
        let pivot =
            (col..n).max_by(|&i, &j| a[i * n + col].abs().total_cmp(&a[j * n + col].abs()))?;
        if a[pivot * n + col].abs() < 1e-300 {
            return None;
        }
        if pivot != col {
            for k in 0..n {
                a.swap(col * n + k, pivot * n + k);
            }
            b.swap(col, pivot);
        }
        for row in col + 1..n {
            let factor = a[row * n + col] / a[col * n + col];
            for k in col..n {
                a[row * n + k] -= factor * a[col * n + k];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let mut sum = b[row];
        for k in row + 1..n {
            sum -= a[row * n + k] * x[k];
        }
        x[row] = sum / a[row * n + row];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dense_solve_matches_a_known_system() {
        let mut a = vec![2.0, 1.0, 1.0, 3.0];
        let mut b = vec![3.0, 5.0];
        let x = solve_dense(&mut a, &mut b, 2).unwrap();
        assert!((x[0] - 0.8).abs() < 1e-12 && (x[1] - 1.4).abs() < 1e-12);
    }
}
