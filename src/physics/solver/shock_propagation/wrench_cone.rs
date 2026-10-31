//! The wrenches a body's frictional contacts can push it with.

use nalgebra::Vector3;
use smallvec::SmallVec;

/// Lever arm below which torques are weighed as if at this reach, in metres,
/// so a body held at a single point still weighs force against torque.
const MIN_REACH: f32 = 0.1;

/// Columns a least-squares solve over the passive set may hold. A solution
/// in six dimensions needs at most six (Carathéodory); two more absorb
/// degenerate ties.
const MAX_PASSIVE: usize = 8;

/// Residual, as a share of the wrench, at which a solve has balanced it.
/// Columns past that point only trade rounding, and fill the passive set.
const SOLVED: f64 = 1e-9;

/// The pyramid edges a solution leans on, by their order of adding.
pub(super) type Support = SmallVec<[usize; MAX_PASSIVE]>;

/// The cone of wrenches (force and torque about a body's centre of mass) a
/// set of contacts can give the body with impulses that push (normal ≥ 0)
/// and stay within their friction (a four-sided pyramid inside each cone,
/// spanned by its four edges: the normal is their mean).
#[derive(Default)]
pub(super) struct ContactWrenchCone {
    /// Each pyramid edge with the lever arm it pushes at.
    edges: SmallVec<[(Vector3<f32>, Vector3<f32>); 64]>,
    /// The longest lever arm of any contact.
    reach: f32,
}

impl ContactWrenchCone {
    pub(super) fn clear(&mut self) {
        self.edges.clear();
        self.reach = 0.0;
    }

    /// Adds a contact at `lever` from the centre of mass, pushing along
    /// `normal` with friction coefficient `friction`.
    pub(super) fn add_contact(&mut self, normal: Vector3<f32>, lever: Vector3<f32>, friction: f32) {
        self.reach = self.reach.max(lever.norm());
        if friction <= 0.0 {
            self.edges.push((lever, normal));
            return;
        }
        let seed = if normal.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        let t1 = (seed - normal * seed.dot(&normal)).normalize();
        let t2 = normal.cross(&t1);
        for edge in [
            normal + t1 * friction,
            normal - t1 * friction,
            normal + t2 * friction,
            normal - t2 * friction,
        ] {
            self.edges.push((lever, edge));
        }
    }

    /// The longest lever arm, at least `MIN_REACH`: the length torques are
    /// divided by to weigh them against forces.
    pub(super) fn reach(&self) -> f32 {
        self.reach.max(MIN_REACH)
    }

    /// How far `force` and `torque` lie outside the cone, as a share of their
    /// size: 0 for a wrench inside it, 1 when there are no contacts. The
    /// contacts can balance a load of the opposite wrench to within it.
    ///
    /// `support` holds the edges a solution for a nearby wrench leaned on,
    /// which the solve starts from, and is left holding this one's. A load
    /// that has shifted a little is mostly held by the same edges, so the
    /// solve then takes one step instead of one per edge.
    pub(super) fn shortfall(
        &self,
        force: Vector3<f32>,
        torque: Vector3<f32>,
        support: &mut Support,
    ) -> f64 {
        let reach = self.reach() as f64;
        let wrench = |f: Vector3<f32>, t: Vector3<f32>| -> [f64; 6] {
            [
                f.x as f64,
                f.y as f64,
                f.z as f64,
                t.x as f64 / reach,
                t.y as f64 / reach,
                t.z as f64 / reach,
            ]
        };
        let target = wrench(force, torque);
        let size = dot(&target, &target).sqrt();
        if size < 1e-9 {
            return 0.0;
        }
        if self.edges.is_empty() {
            return 1.0;
        }
        let columns: SmallVec<[[f64; 6]; 64]> = self
            .edges
            .iter()
            .map(|&(lever, edge)| wrench(edge, lever.cross(&edge)))
            .collect();
        let x = nnls(&columns, &target, support);
        let mut residual = target.map(|v| -v);
        for (edge, &xj) in columns.iter().zip(&x) {
            for r in 0..6 {
                residual[r] += edge[r] * xj;
            }
        }
        dot(&residual, &residual).sqrt() / size
    }
}

fn dot(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    (0..6).map(|r| a[r] * b[r]).sum()
}

/// Least squares over the columns in `set`, by Cholesky on the normal
/// equations, with a ridge far below any real column for degenerate sets.
fn least_squares(a: &[[f64; 6]], b: &[f64; 6], set: &[usize]) -> SmallVec<[f64; MAX_PASSIVE]> {
    let p = set.len();
    let mut l = [[0.0f64; MAX_PASSIVE]; MAX_PASSIVE];
    let mut z: SmallVec<[f64; MAX_PASSIVE]> = set.iter().map(|&j| dot(&a[j], b)).collect();
    let trace: f64 = set.iter().map(|&j| dot(&a[j], &a[j])).sum();
    let ridge = 1e-12 * trace.max(1e-30);
    for i in 0..p {
        for k in 0..=i {
            let mut sum = dot(&a[set[i]], &a[set[k]]) + if i == k { ridge } else { 0.0 };
            for m in 0..k {
                sum -= l[i][m] * l[k][m];
            }
            l[i][k] = if i == k {
                sum.max(ridge).sqrt()
            } else {
                sum / l[k][k]
            };
        }
    }
    for i in 0..p {
        let mut sum = z[i];
        for m in 0..i {
            sum -= l[i][m] * z[m];
        }
        z[i] = sum / l[i][i];
    }
    for i in (0..p).rev() {
        let mut sum = z[i];
        for m in i + 1..p {
            sum -= l[m][i] * z[m];
        }
        z[i] = sum / l[i][i];
    }
    z
}

/// Non-negative least squares (Lawson–Hanson): `x ≥ 0` minimising `|a·x − b|`.
///
/// Starts from the columns in `support`, less those least squares over them
/// would push with a pull, as the passive set the method would hold there;
/// leaves the final passive set in it. A start that fills the passive set
/// before the method converges is abandoned for one from no columns.
fn nnls(a: &[[f64; 6]], b: &[f64; 6], support: &mut Support) -> SmallVec<[f64; 64]> {
    let n = a.len();
    let mut x: SmallVec<[f64; 64]> = SmallVec::from_elem(0.0, n);
    let mut set: Support = support.iter().copied().filter(|&j| j < n).collect();
    while !set.is_empty() {
        let z = least_squares(a, b, &set);
        if z.iter().all(|&v| v > 0.0) {
            for (c, &j) in set.iter().enumerate() {
                x[j] = z[c];
            }
            break;
        }
        let mut c = 0;
        set.retain(|_| {
            c += 1;
            z[c - 1] > 0.0
        });
    }
    let started_warm = !set.is_empty();
    if !lawson_hanson(a, b, &mut x, &mut set) && started_warm {
        x.fill(0.0);
        set.clear();
        lawson_hanson(a, b, &mut x, &mut set);
    }
    *support = set;
    x
}

/// Lawson–Hanson's iterations from the passive set `set`, over which `x` is
/// the positive least-squares solution and zero elsewhere. Whether they
/// converged, rather than stopped with the passive set full.
fn lawson_hanson(a: &[[f64; 6]], b: &[f64; 6], x: &mut [f64], set: &mut Support) -> bool {
    let n = a.len();
    for _ in 0..3 * n + 6 {
        let mut residual = *b;
        for &j in set.iter() {
            for r in 0..6 {
                residual[r] -= a[j][r] * x[j];
            }
        }
        if dot(&residual, &residual) <= SOLVED * SOLVED * dot(b, b) {
            return true;
        }
        let entering = (0..n)
            .filter(|j| !set.contains(j))
            .map(|j| (j, dot(&a[j], &residual)))
            .filter(|&(_, gain)| gain > 1e-12)
            .max_by(|p, q| p.1.total_cmp(&q.1));
        let Some((j, _)) = entering else {
            return true;
        };
        if set.len() == MAX_PASSIVE {
            return false;
        }
        set.push(j);
        for _ in 0..3 * n + 6 {
            let z = least_squares(a, b, set);
            if z.iter().all(|&v| v > 0.0) {
                for (c, &j) in set.iter().enumerate() {
                    x[j] = z[c];
                }
                break;
            }
            // Step towards z until the first coefficient reaches zero, and
            // let it leave.
            let mut step = f64::INFINITY;
            for (c, &j) in set.iter().enumerate() {
                if z[c] <= 0.0 {
                    step = step.min(x[j] / (x[j] - z[c]));
                }
            }
            for (c, &j) in set.iter().enumerate() {
                x[j] += step * (z[c] - x[j]);
            }
            set.retain(|j| {
                if x[*j] <= 1e-12 {
                    x[*j] = 0.0;
                    false
                } else {
                    true
                }
            });
            if set.is_empty() {
                break;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::{ContactWrenchCone, Support};

    /// A unit box resting on its four bottom corners.
    fn box_on_corners(friction: f32) -> ContactWrenchCone {
        let mut cone = ContactWrenchCone::default();
        for (x, z) in [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)] {
            cone.add_contact(Vector3::y(), Vector3::new(x, -0.5, z), friction);
        }
        cone
    }

    /// Whether the corners hold a downward `weight` set on the top face at
    /// `(x, z)` with a push `side` along x there.
    fn holds(cone: &ContactWrenchCone, weight: f32, side: f32, x: f32, z: f32) -> bool {
        let load = Vector3::new(side, -weight, 0.0);
        let torque = Vector3::new(x, 0.5, z).cross(&load);
        cone.shortfall(-load, -torque, &mut Support::new()) <= 0.05
    }

    #[test]
    fn a_box_holds_a_load_over_its_base() {
        let cone = box_on_corners(0.5);
        assert!(holds(&cone, 1.0, 0.0, 0.0, 0.0));
        assert!(holds(&cone, 1.0, 0.0, 0.45, -0.4));
    }

    #[test]
    fn a_box_cannot_hold_a_load_past_its_edge() {
        let cone = box_on_corners(0.5);
        assert!(!holds(&cone, 1.0, 0.0, 0.9, 0.0));
    }

    #[test]
    fn a_box_holds_a_push_within_friction_only() {
        let cone = box_on_corners(0.5);
        assert!(holds(&cone, 1.0, 0.3, 0.0, 0.0));
        assert!(!holds(&cone, 1.0, 0.8, 0.0, 0.0));
    }

    /// A push within friction still tips the box when its moment about the
    /// far edge outweighs the load's.
    #[test]
    fn a_push_within_friction_can_tip_the_box() {
        let cone = box_on_corners(0.5);
        assert!(!holds(&cone, 1.0, 0.45, 0.4, 0.0));
    }

    #[test]
    fn nothing_holds_a_load_without_contacts() {
        let cone = ContactWrenchCone::default();
        assert_eq!(
            cone.shortfall(Vector3::y(), Vector3::zeros(), &mut Support::new()),
            1.0
        );
        assert_eq!(
            cone.shortfall(Vector3::zeros(), Vector3::zeros(), &mut Support::new()),
            0.0
        );
    }

    /// Started from the edges that held one load, the solve finds the same
    /// shortfall for the next as it does from none, whether that load is
    /// held or not.
    #[test]
    fn a_solve_from_a_support_finds_what_one_from_none_does() {
        let cone = box_on_corners(0.5);
        let mut support = Support::new();
        let first = Vector3::new(0.1, -1.0, 0.0);
        cone.shortfall(
            -first,
            -Vector3::new(0.2, 0.5, 0.0).cross(&first),
            &mut support,
        );
        assert!(!support.is_empty());
        for (side, x) in [(0.12, 0.25), (0.45, 0.4), (0.8, 0.0), (0.0, 0.9)] {
            let load = Vector3::new(side, -1.0, 0.0);
            let torque = Vector3::new(x, 0.5, 0.0).cross(&load);
            let warm = cone.shortfall(-load, -torque, &mut support.clone());
            let cold = cone.shortfall(-load, -torque, &mut Support::new());
            assert!(
                (warm - cold).abs() < 1e-9,
                "{side}, {x}: {warm} from the support, {cold} from none"
            );
        }
    }

    /// Edges along one row of contacts cannot balance a twist across it.
    /// Started from eight of them, the four at each end of the row, which
    /// fill the passive set, the solve must still reach the contact off it.
    #[test]
    fn a_start_from_a_full_support_finds_what_one_from_none_does() {
        let mut cone = ContactWrenchCone::default();
        for x in [-0.5, 0.0, 0.5] {
            cone.add_contact(Vector3::y(), Vector3::new(x, -0.5, -0.5), 0.5);
        }
        cone.add_contact(Vector3::y(), Vector3::new(0.0, -0.5, 0.5), 0.5);
        let load = Vector3::new(0.0, -1.0, 0.0);
        let torque = Vector3::new(0.0, 0.5, 0.0).cross(&load);
        let mut ends: Support = [0, 1, 2, 3, 8, 9, 10, 11].into_iter().collect();
        let warm = cone.shortfall(-load, -torque, &mut ends);
        let cold = cone.shortfall(-load, -torque, &mut Support::new());
        assert!(
            (warm - cold).abs() < 1e-9,
            "{warm} from the row's ends, {cold} from none"
        );
    }
}
