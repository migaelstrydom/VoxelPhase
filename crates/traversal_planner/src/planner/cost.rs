use crate::plan::{Contact, FootfallId};
use crate::section::Footing;

use super::settings::CostWeights;

/// The costs every footfall carries whatever model proposed it.
pub struct FootfallCosts<'a> {
    pub weights: &'a CostWeights,
    /// The footfall a foot is already swinging to, if one is.
    pub swinging: Option<FootfallId>,
}

impl FootfallCosts<'_> {
    /// Whether a foot may land on `footing` at all: loose footing that is
    /// already moving is no place to land.
    pub fn admits(&self, footing: Footing) -> bool {
        !matches!(footing, Footing::Loose { resting: false, .. })
    }

    /// What landing on `contact`'s footing costs.
    pub fn of(&self, contact: &Contact) -> f32 {
        let weights = self.weights;
        match contact.footing {
            Footing::Fixed => 0.0,
            Footing::Loose { mass, .. } => {
                // `loose` at the reference mass, toward twice that for a
                // weightless body and toward nothing for a very heavy one.
                let reference = weights.loose_reference_mass;
                weights.loose * 2.0 * reference / (mass + reference)
            }
        }
    }

    /// What moving a tentative footfall from where the last plan had it
    /// costs: more for one a foot is already swinging to.
    pub fn churn(&self, contact: &Contact, before: &Contact) -> f32 {
        let swinging = before.footfall.is_some() && before.footfall == self.swinging;
        let weight = if swinging {
            self.weights.swinging_churn
        } else {
            self.weights.churn
        };
        if contact.limb != before.limb {
            return weight;
        }
        let moved = contact.at - before.at;
        weight * (moved.s * moved.s + moved.d * moved.d).sqrt()
    }
}
