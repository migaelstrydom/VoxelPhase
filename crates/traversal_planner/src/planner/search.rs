use crate::params::GaitParams;
use crate::path::PathVec;
use crate::plan::{BodyState, Contact, Limb, Motion, Phase, Plan};
use crate::section::{Footing, SectionQuery};

use super::body_path::BodySegment;
use super::body_search::{BodySearch, FlightSpan, PreviousPath, RideSpan};
use super::footfall_search::{Footfall, FootfallSearch};
use super::plan_builder::build_plan;
use super::settings::PlannerSettings;

/// How far the body may be from where the last plan has it now, in metres,
/// before its commitment is dropped.
const STRAY_DISTANCE: f32 = 0.05;
/// Slack on the committed footfall's time, in seconds.
const STRAY_TIME: f32 = 1.0e-3;
/// How near a foothold the committed footfall must still be, in metres.
const FOOTHOLD_TOLERANCE: f32 = 0.02;

/// What one plan is asked for.
#[derive(Clone, Copy, Debug)]
pub struct PlanRequest<'a> {
    pub body: BodyState,
    /// The foot that lands next.
    pub lead: Limb,
    /// Where the other foot is planted; `None` takes it to be under the body,
    /// on fixed ground at its natural offset.
    pub planted: Option<Contact>,
    /// The speed along the path the player asks for, in m/s.
    pub target_speed: f32,
    /// The last frame's plan in this frame's path coordinates, its landed
    /// footfalls dropped: its committed footfall is kept if it still can be,
    /// and moving its tentative ones costs churn.
    pub previous: Option<&'a Plan>,
}

/// A plan that carries the body to the goal: as far as the asked-for speed
/// goes in `min_duration`, and at least `min_footfalls` natural steps.
#[derive(Clone, Debug, PartialEq)]
pub struct Planned {
    pub plan: Plan,
    pub cost: f32,
    /// Whether the last plan's committed footfall could no longer be kept,
    /// so the plan started over from the body.
    pub diverged: bool,
}

/// No plan on this heading carries the body to the goal.
#[derive(Clone, Debug, PartialEq)]
pub struct Refusal {
    /// The cheapest plan to the furthest point any plan reached: where the
    /// way runs out.
    pub partial: Plan,
    pub diverged: bool,
}

/// Plans in two stages: the body's path first (where it rides, where it
/// flies, how fast), then the footfalls along it.
///
/// ```text
///   section ──► BodySearch ──► BodyPath ──► FootfallSearch ──► footfalls
///                                  └───────────────┬───────────────┘
///                                              build_plan ──► Plan
/// ```
///
/// Each frame only what follows the last plan's committed footfall is
/// planned again: the phases up to it are kept while the body is still on
/// them and they still hold.
///
/// Where the feet cannot land a flight of the body's path, that flight is
/// vetoed and the body's path searched again, up to `max_vetoes` times. When
/// the body cannot reach the goal, or its feet still cannot follow, the
/// planner refuses, with the plan as far as it got.
pub struct Planner {
    pub params: GaitParams,
    pub settings: PlannerSettings,
}

impl Planner {
    pub fn new(params: GaitParams, settings: PlannerSettings) -> Self {
        Self { params, settings }
    }

    pub fn plan(
        &self,
        query: &dyn SectionQuery,
        request: &PlanRequest,
    ) -> Result<Planned, Refusal> {
        let previous: Vec<Contact> = request
            .previous
            .map(|plan| plan.footfalls().map(|(_, contact)| *contact).collect())
            .unwrap_or_default();
        let wanted_commitment = request.previous.is_some_and(|plan| plan.committed > 0);

        // Only what follows the committed footfall is planned again.
        let mut kept_refusal = None;
        if let Some(kept) = self.committed_prefix(query, request) {
            match self.plan_from(query, request, &kept.body, kept.planted, &previous[1..]) {
                Ok((tail, cost)) => {
                    return Ok(Planned {
                        plan: kept.followed_by(tail),
                        cost,
                        diverged: false,
                    })
                }
                Err(partial) => kept_refusal = Some(kept.followed_by(partial)),
            }
        }

        // No commitment to keep, or none the way on from it: plan afresh. A
        // fresh plan where the commitment's had none means dropping it; where
        // neither has one, the refusal is about the way ahead, and the
        // commitment stands.
        let fresh = self.plan_from(
            query,
            request,
            &request.body,
            self.planted(request),
            &previous,
        );
        match (fresh, kept_refusal) {
            (Ok((plan, cost)), _) => Ok(Planned {
                plan,
                cost,
                diverged: wanted_commitment,
            }),
            (Err(_), Some(partial)) => Err(Refusal {
                partial,
                diverged: false,
            }),
            (Err(partial), None) => Err(Refusal {
                partial,
                diverged: wanted_commitment,
            }),
        }
    }

    /// The last plan's phases up to its committed footfall, if the body is
    /// still on them and they still hold: they clear the ground and the
    /// footfall is on a foothold that will hold it.
    fn committed_prefix(&self, query: &dyn SectionQuery, request: &PlanRequest) -> Option<Kept> {
        let previous = request.previous.filter(|plan| plan.committed > 0)?;
        let first = previous.phases.get(previous.committed)?;
        let landing = *first.contacts.first()?;
        let (time, footfall) = previous.footfalls().next()?;
        if *footfall != landing || (time - first.start).abs() > STRAY_TIME {
            return None;
        }
        let now = previous.body_at(0.0, query)?;
        let strayed = (now.s - request.body.s).abs() > STRAY_DISTANCE
            || (now.z - request.body.z).abs() > STRAY_DISTANCE;
        let phases = &previous.phases[..previous.committed];
        let holds = phases.iter().all(|phase| query.clears(phase))
            && query
                .foothold_at(&landing.at, FOOTHOLD_TOLERANCE)
                .is_some_and(|foothold| {
                    !matches!(foothold.footing, Footing::Loose { resting: false, .. })
                });
        if strayed || !holds {
            return None;
        }
        Some(Kept {
            phases: phases.to_vec(),
            time,
            body: previous.body_at(time, query)?,
            planted: landing,
        })
    }

    /// The cheapest plan from the body at `body` with `planted` holding it,
    /// as asked for by `request`; or, refused, the plan as far as it got.
    /// The goal is measured from the request's body, so a plan from its
    /// committed footfall reaches no further than a fresh one. Each footfall
    /// moved from `previous`, and each flight from the last plan's, costs
    /// churn.
    fn plan_from(
        &self,
        query: &dyn SectionQuery,
        request: &PlanRequest,
        body: &BodyState,
        planted: Contact,
        previous: &[Contact],
    ) -> Result<(Plan, f32), Plan> {
        let previous_path = request.previous.map(previous_path);
        let mut vetoed = Vec::new();
        loop {
            let found = BodySearch {
                query,
                params: &self.params,
                settings: &self.settings,
                target_speed: request.target_speed,
                vetoed: &vetoed,
                goal: request.body.s + self.settings.body_goal(request.target_speed, &self.params),
                planted: planted.at.s,
                previous: previous_path.as_ref(),
            }
            .search(*body);
            let (path, body_cost) = match &found {
                Ok(plan) => (&plan.path, plan.cost),
                Err(partial) => (partial, 0.0),
            };
            let feet = FootfallSearch {
                query,
                params: &self.params,
                settings: &self.settings,
                target_speed: request.target_speed,
                spacing: self
                    .settings
                    .footfall_spacing(request.target_speed, &self.params),
                body: path,
                previous,
            }
            .search(Footfall {
                contact: planted,
                body_s: body.s,
            });

            match (found.is_ok(), feet) {
                (true, Ok(feet)) => {
                    return Ok((
                        build_plan(path, &planted, &feet.footfalls),
                        body_cost + feet.cost,
                    ))
                }
                (false, Ok(feet)) => return Err(build_plan(path, &planted, &feet.footfalls)),
                (_, Err(refused)) => {
                    // The feet could not land a flight: try the body's path
                    // again without it, or refuse with what they managed.
                    let reached = refused
                        .footfalls
                        .last()
                        .map_or(body.s, |footfall| footfall.body_s);
                    let unfollowed = path.flights().find_map(|flight| match *flight {
                        BodySegment::Flight {
                            takeoff, landing, ..
                        } if landing > reached => Some(FlightSpan {
                            takeoff: takeoff.s,
                            landing,
                        }),
                        _ => None,
                    });
                    match unfollowed {
                        Some(span) if vetoed.len() < self.settings.max_vetoes => vetoed.push(span),
                        _ => {
                            return Err(build_plan(
                                &path.until_unlanded(reached),
                                &planted,
                                &refused.footfalls,
                            ))
                        }
                    }
                }
            }
        }
    }

    /// The foot planted when the plan starts: the request's, or the other
    /// one from the lead, under the body at its natural offset, on fixed
    /// ground.
    fn planted(&self, request: &PlanRequest) -> Contact {
        if let Some(planted) = request.planted {
            return planted;
        }
        let limb = request.lead.other();
        Contact {
            limb,
            at: PathVec::new(
                request.body.s,
                self.params.foot.natural_offset(limb),
                request.body.z - self.params.body.radius,
            ),
            normal: PathVec::UP,
            footing: Footing::Fixed,
        }
    }
}

/// The last plan's committed phases, kept, and where they leave the body.
struct Kept {
    /// From now to the committed footfall.
    phases: Vec<Phase>,
    /// Seconds from now until the committed footfall lands.
    time: f32,
    /// The body as it lands.
    body: BodyState,
    /// The committed footfall, which holds the body from then.
    planted: Contact,
}

impl Kept {
    /// The kept phases, then `tail` from the committed footfall on.
    fn followed_by(self, tail: Plan) -> Plan {
        let committed = self.phases.len();
        let mut phases = self.phases;
        phases.extend(tail.phases.into_iter().map(|phase| Phase {
            start: phase.start + self.time,
            ..phase
        }));
        Plan { phases, committed }
    }
}

/// The body path of the last plan, as the next search reads it.
fn previous_path(plan: &Plan) -> PreviousPath {
    let mut path = PreviousPath::default();
    for phase in &plan.phases {
        match phase.motion {
            Motion::Flight { takeoff, .. } => path.flights.push(FlightSpan {
                takeoff: takeoff.s,
                landing: phase.end_s(),
            }),
            Motion::Grounded {
                from,
                to,
                end_speed,
                ..
            } if to > from => path.rides.push(RideSpan {
                from,
                to,
                end_speed,
            }),
            Motion::Grounded { .. } => {}
        }
    }
    path
}
