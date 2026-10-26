use crate::params::GaitParams;
use crate::plan::{Contact, Motion, Phase, Plan};
use crate::section::{Footing, SectionQuery};

use super::violation::{Rule, Violation};

/// Slack on times and positions that must match, in seconds or metres.
const TOLERANCE: f32 = 1.0e-3;
/// How near a footfall must be to a foothold of the section, in metres.
const ON_FOOTHOLD: f32 = 0.02;

/// Every property `plan` breaks on the ground `query` reads.
pub fn check_plan(plan: &Plan, query: &dyn SectionQuery, params: &GaitParams) -> Vec<Violation> {
    let mut violations = Vec::new();
    let mut broke = |rule, time, detail: String| violations.push(Violation { rule, time, detail });

    for (index, phase) in plan.phases.iter().enumerate() {
        if let Some(before) = index.checked_sub(1).map(|before| &plan.phases[before]) {
            let gap = phase.start - (before.start + before.duration);
            let jump = start_s(phase) - before.end_s();
            if gap.abs() > TOLERANCE || jump.abs() > TOLERANCE {
                broke(
                    Rule::PhasesFollowOn,
                    phase.start,
                    format!("phase {index} starts {gap:.3} s and {jump:.3} m off the one before"),
                );
            }
        }
        if !query.clears(phase) {
            broke(
                Rule::PhaseClears,
                phase.start,
                format!("phase {index}: {:?}", phase.motion),
            );
        }
        if let Motion::Flight {
            takeoff, rise_held, ..
        } = phase.motion
        {
            // No floor on the climb: a body rolling off ground that falls
            // away leaves it moving down. Nor a ceiling on one leaving a rise
            // (`rise_held`): it carries the climb it rode up with, no push.
            if rise_held.is_none() && takeoff.climb > params.flight.max_takeoff_climb + TOLERANCE {
                broke(
                    Rule::ClimbWithinLimits,
                    phase.start,
                    format!("takeoff climb {:.2} m/s", takeoff.climb),
                );
            }
        }
    }

    let mut last: Option<Contact> = plan
        .phases
        .first()
        .and_then(|phase| phase.contacts.first())
        .copied();
    for (time, contact) in plan.footfalls() {
        if let Some(last) = last {
            let jump = lands_a_flight(plan, time);
            check_step(&last, contact, jump, params, &mut |rule, detail| {
                broke(rule, time, detail)
            });
        }
        if matches!(contact.footing, Footing::Loose { resting: false, .. }) {
            broke(
                Rule::FootingHolds,
                time,
                format!("{:?} on moving footing", contact.at),
            );
        }
        if !on_foothold(contact, query) {
            broke(
                Rule::OnFoothold,
                time,
                format!("{:?} is on no foothold", contact.at),
            );
        }
        match plan.body_at(time, query) {
            Some(body) if params.foot_reaches(body.s, body.z, &contact.at) => {}
            body => broke(
                Rule::FootReachesBody,
                time,
                format!(
                    "foot at z {:.2}, body {:?}",
                    contact.at.z,
                    body.map(|body| body.z)
                ),
            ),
        }
        last = Some(*contact);
    }
    violations
}

/// Where along the path a phase starts.
fn start_s(phase: &Phase) -> f32 {
    match phase.motion {
        Motion::Grounded { from, .. } => from,
        Motion::Flight { takeoff, .. } => takeoff.s,
    }
}

/// Whether the footfall at `time` lands a flight, a hop or a roll-off,
/// which no step limit holds.
fn lands_a_flight(plan: &Plan, time: f32) -> bool {
    plan.phases.iter().any(|phase| match phase.motion {
        Motion::Flight { .. } => (phase.start + phase.duration - time).abs() < TOLERANCE,
        Motion::Grounded { .. } => false,
    })
}

/// The step from `last` to `next` against the gait's limits; its length
/// not, if it lands a `jump`.
fn check_step(
    last: &Contact,
    next: &Contact,
    jump: bool,
    params: &GaitParams,
    broke: &mut impl FnMut(Rule, String),
) {
    if next.limb == last.limb {
        broke(Rule::FeetAlternate, format!("{:?} twice", next.limb));
    }
    let (length, rise) = (next.at.s - last.at.s, next.at.z - last.at.z);
    let stride = &params.stride;
    let reach = params.foot.reach(next.limb);
    let out_of_limits = [
        (length < stride.min_length - TOLERANCE, "step too short"),
        (
            !jump && length > stride.max_length + TOLERANCE,
            "step too long",
        ),
        (rise > stride.max_rise + TOLERANCE, "rise too high"),
        (-rise > stride.max_drop + TOLERANCE, "drop too deep"),
        (
            next.at.d < reach.start - ON_FOOTHOLD || next.at.d > reach.end + ON_FOOTHOLD,
            "foot out of its sideways reach",
        ),
    ];
    for (_, what) in out_of_limits.iter().filter(|(broken, _)| *broken) {
        broke(
            Rule::WithinStride,
            format!(
                "{what}: length {length:.2}, rise {rise:.2}, d {:.2}",
                next.at.d
            ),
        );
    }
}

/// Whether a foothold of the section is where `contact` is.
fn on_foothold(contact: &Contact, query: &dyn SectionQuery) -> bool {
    query.foothold_at(&contact.at, ON_FOOTHOLD).is_some()
}
