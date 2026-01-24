//! System that converts contact candidates into IK targets.

use std::collections::HashMap;

use specs::{Entities, Join, ReadStorage, System, WriteStorage};

use crate::components::{ContactCandidate, ContactCandidates, IKTarget, IKTargets, ProbePurpose};

/// Selects IK targets from contact candidates.
pub struct IKTargetSystem;

impl<'a> System<'a> for IKTargetSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, ContactCandidates>,
        WriteStorage<'a, IKTargets>,
    );

    fn run(&mut self, (entities, candidates, mut targets): Self::SystemData) {
        for (entity, candidate_set) in (&entities, &candidates).join() {
            let mut best: HashMap<ProbePurpose, &ContactCandidate> = HashMap::new();

            for candidate in &candidate_set.candidates {
                match best.get(&candidate.purpose) {
                    Some(existing) => {
                        if candidate.distance < existing.distance {
                            best.insert(candidate.purpose, candidate);
                        }
                    }
                    None => {
                        best.insert(candidate.purpose, candidate);
                    }
                }
            }

            let mut result = IKTargets::default();
            for (_purpose, candidate) in best {
                result.targets.push(IKTarget {
                    purpose: candidate.purpose,
                    target: candidate.point,
                    normal: candidate.normal,
                    weight: 1.0,
                });
            }

            let _ = targets.insert(entity, result);
        }
    }
}
