//! How many velocity iterations each island gets.
//!
//! An island starts from the configured base and earns extra passes for the
//! hardest body in it, judged by the contacts pressing on it as the second
//! body of a manifold:
//!
//! - more than two contacts: one extra per contact beyond two, at most four;
//! - contact normals that disagree (any more than ~32° off their mean): two.
//!
//! Both are measured per body in one pass over every manifold, into arrays
//! indexed by solver slot, so asking for an island's count only reads them.

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

/// Per-body contact measurements for one substep.
#[derive(Debug, Default)]
pub(crate) struct IterationBudget {
    /// Solver slot of each manifold's second body; `None` when it has no rows.
    body_of_manifold: Vec<Option<usize>>,
    /// Contacts pressing on each solver slot.
    contacts: Vec<u32>,
    /// Sum of those contacts' normals.
    normal_sum: Vec<Vector3<f32>>,
    /// Lowest cosine between one of those normals and their mean.
    min_cos: Vec<f32>,
}

impl IterationBudget {
    /// Measure every body's contacts. `body_count` is the number of solver
    /// slots.
    pub fn measure(
        &mut self,
        manifolds: &[SolverManifold],
        contact_rows: &ContactRows,
        body_count: usize,
    ) {
        self.body_of_manifold.clear();
        self.body_of_manifold
            .extend((0..manifolds.len()).map(|mi| contact_rows.manifold_slots(mi).map(|(_, b)| b)));
        self.contacts.clear();
        self.contacts.resize(body_count, 0);
        self.normal_sum.clear();
        self.normal_sum.resize(body_count, Vector3::zeros());
        self.min_cos.clear();
        self.min_cos.resize(body_count, 1.0);

        for (manifold, body) in manifolds.iter().zip(&self.body_of_manifold) {
            let Some(body) = *body else { continue };
            for contact in &manifold.contacts {
                self.contacts[body] += 1;
                self.normal_sum[body] += contact.normal;
            }
        }
        for (manifold, body) in manifolds.iter().zip(&self.body_of_manifold) {
            let Some(body) = *body else { continue };
            let sum = self.normal_sum[body];
            if self.contacts[body] < 2 || sum.magnitude_squared() < 1e-6 {
                continue;
            }
            let mean = sum.normalize();
            for contact in &manifold.contacts {
                self.min_cos[body] = self.min_cos[body].min(contact.normal.dot(&mean));
            }
        }
    }

    /// Iterations for the island made of `manifolds`, starting from `base`.
    pub fn iterations(&self, manifolds: &[usize], base: u32) -> u32 {
        let bodies = manifolds.iter().filter_map(|&mi| self.body_of_manifold[mi]);
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
