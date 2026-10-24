use crate::plan::Plan;
use crate::planner::{PlanRequest, Planner};
use crate::section::SectionQuery;

use super::violation::{Rule, Violation};

/// How far the next tentative footfall may move when nothing changed, in
/// metres: a cell and a half of the lattice at its coarsest, so a footfall
/// settling into a neighbouring cell as the body moves on is not churn.
const TENTATIVE_TOLERANCE: f32 = 0.15;
/// How far the committed footfall may move, in metres.
const COMMITTED_TOLERANCE: f32 = 1.0e-3;

/// Replans `elapsed` seconds into `plan`, made for `request`, on the same
/// ground, the body having tracked the plan exactly: the commitment must
/// hold and the next tentative footfall stay, unless it is the last. Nothing is checked while the
/// body is in flight, where a plan starts from no stance.
pub fn check_replan(
    planner: &Planner,
    query: &dyn SectionQuery,
    request: &PlanRequest,
    plan: &Plan,
    elapsed: f32,
) -> Vec<Violation> {
    let (Some(body), Some(stance)) = (plan.body_at(elapsed, query), plan.stance_at(elapsed)) else {
        return Vec::new();
    };
    let previous = plan.after(elapsed);
    let later = PlanRequest {
        body,
        lead: stance.limb.other(),
        planted: Some(*stance),
        previous: Some(&previous),
        ..*request
    };
    let mut violations = Vec::new();
    let mut broke = |rule, detail: String| {
        violations.push(Violation {
            rule,
            time: elapsed,
            detail,
        })
    };
    let replanned = match planner.plan(query, &later) {
        Ok(planned) => {
            if planned.diverged {
                broke(
                    Rule::KeepsCommitment,
                    "the replan dropped its commitment".into(),
                );
            }
            planned.plan
        }
        // A refusal as the goal slides onto an obstacle is right; one that
        // dropped the commitment to get there is not.
        Err(refusal) => {
            if refusal.diverged {
                broke(
                    Rule::KeepsCommitment,
                    "the replan refused, dropping its commitment".into(),
                );
            }
            refusal.partial
        }
    };

    let before: Vec<_> = previous.footfalls().map(|(_, contact)| *contact).collect();
    let after: Vec<_> = replanned.footfalls().map(|(_, contact)| *contact).collect();
    let moved = |index: usize| {
        before
            .get(index)
            .zip(after.get(index))
            .map(|(before, after)| (after.at - before.at).length())
    };
    if previous.committed > 0 {
        if let Some(distance) = moved(0).filter(|&distance| distance > COMMITTED_TOLERANCE) {
            broke(
                Rule::KeepsCommitment,
                format!("the committed footfall moved {distance:.3} m"),
            );
        }
    }
    // The last footfall moves as the goal slides ahead with the body, so the
    // next tentative one is held still only when another follows it.
    let followed = before.len() > 2;
    if let Some(distance) = moved(1).filter(|&distance| followed && distance > TENTATIVE_TOLERANCE)
    {
        broke(
            Rule::TentativeStays,
            format!("the next tentative footfall moved {distance:.3} m"),
        );
    }
    violations
}
