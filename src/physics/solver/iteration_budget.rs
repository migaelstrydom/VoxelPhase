//! How many velocity iterations an island gets.
//!
//! An island starts from the configured base and earns extra passes for the
//! hardest body in it, judged by every contact pressing on it, from either
//! side of its manifolds. Which body of a pair is the first is the
//! broadphase's choice and says nothing about the pair, so a body squeezed
//! from above and below is judged the same whichever it is:
//!
//! - more than two contacts: one extra per contact beyond two, at most four;
//! - contact normals that disagree (any more than ~32° off their mean): two.
//!
//! Both are measured per body in one pass over the island's manifolds, into
//! arrays indexed by solver slot.

use nalgebra::Vector3;

use crate::physics::pipeline::pair::SolverManifold;

use super::contact_row::ContactRows;

/// Contacts beyond this many on one body earn extra iterations.
const FREE_CONTACTS: u32 = 2;
/// The most iterations contact count alone can add.
const MAX_CONTACT_EXTRA: u32 = 4;
/// Normals whose cosine to their mean falls below this disagree.
const DIVERGENT_NORMAL_COS: f32 = 0.85;
/// Iterations added to an island with a body whose normals disagree.
const DIVERGENCE_EXTRA: u32 = 2;

/// Per-body contact measurements of one island, for one substep.
#[derive(Debug, Default)]
pub(crate) struct IterationBudget {
    /// Solver slots of each manifold's bodies, the first only where it moves;
    /// `None` when the manifold has no rows.
    bodies_of_manifold: Vec<Option<(Option<usize>, usize)>>,
    /// Contacts pressing on each solver slot.
    contacts: Vec<u32>,
    /// Sum of those contacts' normals.
    normal_sum: Vec<Vector3<f32>>,
    /// Lowest cosine between one of those normals and their mean.
    min_cos: Vec<f32>,
}

impl IterationBudget {
    /// Measure every body's contacts. `contact_rows` holds the rows of
    /// `manifolds`, in order; `body_count` is the number of solver slots.
    pub fn measure<'a>(
        &mut self,
        manifolds: impl Iterator<Item = &'a SolverManifold> + Clone,
        contact_rows: &ContactRows,
        body_count: usize,
    ) {
        self.bodies_of_manifold.clear();
        self.bodies_of_manifold
            .extend((0..manifolds.clone().count()).map(|mi| contact_rows.manifold_slots(mi)));
        self.contacts.clear();
        self.contacts.resize(body_count, 0);
        self.normal_sum.clear();
        self.normal_sum.resize(body_count, Vector3::zeros());
        self.min_cos.clear();
        self.min_cos.resize(body_count, 1.0);

        for (manifold, bodies) in manifolds.clone().zip(&self.bodies_of_manifold) {
            for (body, side) in pressed(*bodies) {
                for contact in &manifold.contacts {
                    self.contacts[body] += 1;
                    self.normal_sum[body] += contact.normal * side;
                }
            }
        }
        for (manifold, bodies) in manifolds.clone().zip(&self.bodies_of_manifold) {
            for (body, side) in pressed(*bodies) {
                let sum = self.normal_sum[body];
                if self.contacts[body] < 2 || sum.magnitude_squared() < 1e-6 {
                    continue;
                }
                let mean = sum.normalize();
                for contact in &manifold.contacts {
                    self.min_cos[body] = self.min_cos[body].min((contact.normal * side).dot(&mean));
                }
            }
        }
    }

    /// Iterations for the measured island, starting from `base`.
    pub fn iterations(&self, base: u32) -> u32 {
        let bodies = self
            .bodies_of_manifold
            .iter()
            .flat_map(|bodies| pressed(*bodies).map(|(body, _)| body));
        let most_contacts = bodies.clone().map(|b| self.contacts[b]).max().unwrap_or(0);
        let diverges = bodies
            .clone()
            .any(|b| self.min_cos[b] < DIVERGENT_NORMAL_COS);

        let contact_extra = most_contacts
            .saturating_sub(FREE_CONTACTS)
            .min(MAX_CONTACT_EXTRA);
        let divergence_extra = if diverges { DIVERGENCE_EXTRA } else { 0 };
        base + contact_extra + divergence_extra
    }
}

/// Each moving body of a manifold with the sign that turns the manifold's
/// normal into the direction its contacts press that body: along the normal
/// for the second body, against it for the first.
fn pressed(bodies: Option<(Option<usize>, usize)>) -> impl Iterator<Item = (usize, f32)> + Clone {
    bodies
        .into_iter()
        .flat_map(|(a, b)| a.map(|a| (a, -1.0)).into_iter().chain([(b, 1.0)]))
}

#[cfg(test)]
mod tests {
    use generational_arena::Arena;
    use nalgebra::{Point3, Vector3};
    use smallvec::smallvec;

    use super::*;
    use crate::collision::contact::FeatureId;
    use crate::physics::body::{RigidBody, RigidBodyDesc};
    use crate::physics::drive::plan::TractionRow;
    use crate::physics::handle::RigidBodyHandle;
    use crate::physics::pipeline::pair::{PairHeader, SolverContact};
    use crate::physics::solver::body_pair::LeverOrigins;
    use crate::physics::solver::solver_bodies::SolverBodies;

    const BASE: u32 = 3;

    fn contact(x: f32, y: f32, normal: Vector3<f32>) -> SolverContact {
        SolverContact {
            point: Point3::new(x, y, 0.0),
            normal,
            raw_normal: normal,
            depth: 0.0,
            raw_depth: 0.0,
            gap: 0.0,
            closing_allowance: 0.0,
            feature_id: FeatureId::SINGLE,
            warm_normal_impulse: 0.0,
            warm_friction_impulse_ws: Vector3::zeros(),
            accumulated_normal_impulse: 0.0,
            accumulated_friction_impulse_ws: Vector3::zeros(),
            accumulated_torsional_impulse: 0.0,
            tangential_scale: 1.0,
            traction: TractionRow::default(),
        }
    }

    /// Two contacts across a face at height `y`, pressing along `normal`.
    fn manifold(
        body_a: Option<RigidBodyHandle>,
        body_b: RigidBodyHandle,
        y: f32,
        normal: Vector3<f32>,
    ) -> SolverManifold {
        SolverManifold {
            header: PairHeader {
                body_a,
                body_b,
                collider_a: None,
                collider_b: None,
                restitution: 0.0,
                friction: 0.5,
            },
            contacts: smallvec![contact(-0.4, y, normal), contact(0.4, y, normal)],
        }
    }

    /// The iterations a box on the ground earns with another box on it, the
    /// pair between them listing the upper box first or the lower.
    fn squeezed_box_iterations(upper_first: bool) -> u32 {
        let mut arena = Arena::new();
        let mut body = |y: f32| {
            RigidBodyHandle(arena.insert(RigidBody::new(
                RigidBodyDesc::dynamic().position(Point3::new(0.0, y, 0.0)),
            )))
        };
        let lower = body(0.5);
        let upper = body(1.5);
        let up = Vector3::y();
        let between = if upper_first {
            manifold(Some(upper), lower, 1.0, -up)
        } else {
            manifold(Some(lower), upper, 1.0, up)
        };
        let manifolds = [manifold(None, lower, 0.0, up), between];

        let mut solver_bodies = SolverBodies::default();
        let mut rows = ContactRows::default();
        rows.prepare(
            &arena,
            &mut solver_bodies,
            manifolds
                .iter()
                .map(|m| (&m.header, m.contacts.as_slice(), (1.0, 1.0))),
            LeverOrigins::Current,
        );
        let mut budget = IterationBudget::default();
        budget.measure(manifolds.iter(), &rows, solver_bodies.len());
        budget.iterations(BASE)
    }

    /// The lower box has four contacts pressing on it, two from the ground and
    /// two from the box above, whichever body the pair between them lists
    /// first: two beyond the free ones.
    #[test]
    fn a_squeezed_body_earns_its_iterations_from_either_side_of_a_pair() {
        let expected = BASE + 2;
        assert_eq!(squeezed_box_iterations(true), expected);
        assert_eq!(squeezed_box_iterations(false), expected);
    }
}
