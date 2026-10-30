use smallvec::SmallVec;

use crate::params::GaitParams;
use crate::path::PathVec;
use crate::plan::{BodyState, Contact, FootfallId, Limb, Motion, Phase, Plan};
use crate::section::{Footing, SectionQuery};

use super::body_path::{BodyPath, BodySegment};
use super::body_search::{BodySearch, FlightSpan, PreviousPath, RideSpan};
use super::footfall_search::{Footfall, FootfallPlan, FootfallSearch, FootfallsRefused};
use super::plan_builder::build_plan;
use super::settings::PlannerSettings;

/// How far the body may be from where the last plan has it now, in metres,
/// before its commitment is dropped.
const STRAY_DISTANCE: f32 = 0.05;
/// How many of the last plan's footfalls moving costs churn at most: the
/// committed one and the next, which the feet are already setting up for.
/// Past them the last plan guessed with less of the way in sight, and each
/// frame chooses again.
const CHURNED_FOOTFALLS: usize = 2;
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
    /// Where the lead foot is swinging to, if onto one of the last plan's
    /// footfalls.
    pub aim: Option<Aim>,
    /// The last frame's plan in this frame's path coordinates, its landed
    /// footfalls dropped: its committed footfall is kept if it still can be,
    /// and moving its tentative ones costs churn.
    pub previous: Option<&'a Plan>,
    /// How far along the path a plan need carry the body at most, short of
    /// its usual goal: the strip is not yet sampled further. `None` for the
    /// usual goal.
    pub reach_limit: Option<f32>,
}

/// The lead foot's swing onto a plan's footfall: which one, and the seconds
/// until it lands there.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aim {
    pub footfall: FootfallId,
    pub lands_in: f32,
}

/// A plan that carries the body to the goal: `min_footfalls` natural steps
/// and the longest hop, and a step past its last landing.
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
        let mut outcome = self.plan_unstamped(query, request);
        let plan = match &mut outcome {
            Ok(planned) => &mut planned.plan,
            Err(refusal) => &mut refusal.partial,
        };
        plan.target_speed = request.target_speed;
        plan.number_footfalls(first_free_id(request));
        outcome
    }

    fn plan_unstamped(
        &self,
        query: &dyn SectionQuery,
        request: &PlanRequest,
    ) -> Result<Planned, Refusal> {
        let previous: Vec<Contact> = request
            .previous
            .map(|plan| {
                plan.footfalls()
                    .take(settled_footfalls(plan))
                    .map(|(_, contact)| *contact)
                    .collect()
            })
            .unwrap_or_default();
        let wanted_commitment = request.previous.is_some_and(|plan| plan.committed > 0);

        // The last plan through its next tentative footfall, which then
        // stays where it was: what follows is planned again.
        let next_settled = request
            .previous
            .is_some_and(|plan| settled_footfalls(plan) > 1);
        if let Some(kept) = next_settled
            .then(|| self.kept_through(query, request, 1))
            .flatten()
        {
            if let Ok((tail, cost)) = self.plan_from(query, request, &kept.body, kept.planted, &[])
            {
                return Ok(Planned {
                    plan: kept.followed_by(tail),
                    cost,
                    diverged: false,
                });
            }
        }

        // Failing that, only what follows the committed footfall.
        let kept_previous = previous.get(1..).unwrap_or_default();
        let mut kept_refusal = None;
        if let Some(kept) = self.kept_through(query, request, 0) {
            match self.plan_from(query, request, &kept.body, kept.planted, kept_previous) {
                Ok((tail, cost)) => {
                    return Ok(Planned {
                        plan: kept.followed_by(tail),
                        cost,
                        diverged: false,
                    })
                }
                Err(unplanned) => kept_refusal = Some((kept, unplanned)),
            }
        }

        // No commitment to keep, or none the way on from it: plan afresh. A
        // fresh plan where the commitment's had none means dropping it; where
        // neither has one, the refusal is about the way ahead, and the
        // commitment stands. Its first footfall is the lead foot's, which may
        // be swinging onto the last plan's stance footfall.
        let landing_late = request
            .previous
            .and_then(|plan| plan.stance_at(0.0))
            .filter(|stance| {
                stance.footfall.is_some() && stance.footfall == request.aim.map(|aim| aim.footfall)
            });
        let fresh_previous: Vec<Contact> = landing_late
            .into_iter()
            .copied()
            .chain(previous.iter().copied())
            .collect();
        let planted = self.planted(request);
        let fresh = self.plan_from(query, request, &request.body, planted, &fresh_previous);
        match (fresh, kept_refusal) {
            (Ok((plan, cost)), _) => Ok(Planned {
                plan,
                cost,
                diverged: wanted_commitment,
            }),
            (Err(_), Some((kept, unplanned))) => {
                let tail = self.partial(
                    query,
                    request,
                    unplanned,
                    &kept.body,
                    kept.planted,
                    kept_previous,
                );
                Err(Refusal {
                    partial: kept.followed_by(tail),
                    diverged: false,
                })
            }
            (Err(unplanned), None) => Err(Refusal {
                partial: self.partial(
                    query,
                    request,
                    unplanned,
                    &request.body,
                    planted,
                    &fresh_previous,
                ),
                diverged: wanted_commitment,
            }),
        }
    }

    /// The last plan's phases up to its `footfall`th footfall (the first is
    /// the committed one), if the body is still on them and they still hold:
    /// they stand on the feet the request has down, they clear the ground,
    /// and that footfall and every one before it are on footholds that will
    /// hold them.
    fn kept_through(
        &self,
        query: &dyn SectionQuery,
        request: &PlanRequest,
        footfall: usize,
    ) -> Option<Kept> {
        let previous = request
            .previous
            .filter(|plan| plan.committed > 0 && stands_as_asked(plan, request))?;
        let (time, landing) = previous.footfalls().nth(footfall)?;
        let landing = *landing;
        let phase = previous.phases.iter().position(|phase| {
            (phase.start - time).abs() <= STRAY_TIME && phase.contacts.contains(&landing)
        })?;
        let now = previous.body_at(0.0, query)?;
        let strayed = (now.s - request.body.s).abs() > STRAY_DISTANCE
            || (now.z - request.body.z).abs() > STRAY_DISTANCE;
        let phases = &previous.phases[..phase];
        let holds = phases.iter().all(|phase| query.clears(phase))
            && previous.footfalls().take(footfall + 1).all(|(_, contact)| {
                query
                    .foothold_at(&contact.at, FOOTHOLD_TOLERANCE)
                    .is_some_and(|foothold| {
                        !matches!(foothold.footing, Footing::Loose { resting: false, .. })
                    })
            });
        if strayed || !holds {
            return None;
        }
        Some(Kept {
            phases: phases.to_vec(),
            committed: previous.committed,
            time,
            body: previous.body_at(time, query)?,
            planted: landing,
        })
    }

    /// The cheapest plan from the body at `body` with `planted` holding it,
    /// as asked for by `request`; or, refused, why. The goal is measured
    /// from the request's body, so a plan from its committed footfall
    /// reaches no further than a fresh one. Each footfall moved from
    /// `previous`, and leaving each of the last plan's near flights, costs
    /// churn.
    ///
    /// A flight the feet cannot land is vetoed and the body's path searched
    /// again; a body path that does not reach the goal is not, since the
    /// plan is refused whatever its feet do.
    fn plan_from(
        &self,
        query: &dyn SectionQuery,
        request: &PlanRequest,
        body: &BodyState,
        planted: Contact,
        previous: &[Contact],
    ) -> Result<(Plan, f32), Unplanned> {
        let previous_path = request.previous.map(previous_path);
        let mut vetoed = Vec::new();
        loop {
            let found = BodySearch {
                query,
                params: &self.params,
                settings: &self.settings,
                target_speed: request.target_speed,
                vetoed: &vetoed,
                goal: self.goal(request),
                planted: planted.at.s,
                previous: previous_path.as_ref(),
            }
            .search(*body)
            .map_err(Unplanned::Body)?;
            let refused = match self.footfalls(query, request, &found.path, body, planted, previous)
            {
                Ok(feet) => {
                    return Ok((
                        build_plan(&found.path, &planted, &feet.footfalls),
                        found.cost + feet.cost,
                    ))
                }
                Err(refused) => refused,
            };
            let reached = reached(&refused, body);
            let unfollowed = found
                .path
                .flights()
                .chain(found.path.leap())
                .find_map(|flight| match *flight {
                    BodySegment::Flight {
                        takeoff, landing, ..
                    }
                    | BodySegment::Leap {
                        takeoff,
                        until: landing,
                        ..
                    } if landing > reached => Some(FlightSpan {
                        takeoff: takeoff.s,
                        landing,
                    }),
                    _ => None,
                });
            match unfollowed {
                Some(span) if vetoed.len() < self.settings.max_vetoes => vetoed.push(span),
                _ => {
                    return Err(Unplanned::Feet(build_plan(
                        &found.path.until_unlanded(reached),
                        &planted,
                        &refused.footfalls,
                    )))
                }
            }
        }
    }

    /// The plan as far as it got, for a refusal `plan_from` gave with the
    /// same arguments: a body path short of the goal has its feet placed
    /// along it only now, since only the refusal returned needs them.
    fn partial(
        &self,
        query: &dyn SectionQuery,
        request: &PlanRequest,
        unplanned: Unplanned,
        body: &BodyState,
        planted: Contact,
        previous: &[Contact],
    ) -> Plan {
        let path = match unplanned {
            Unplanned::Feet(plan) => return plan,
            Unplanned::Body(path) => path,
        };
        match self.footfalls(query, request, &path, body, planted, previous) {
            Ok(feet) => build_plan(&path, &planted, &feet.footfalls),
            Err(refused) => build_plan(
                &path.until_unlanded(reached(&refused, body)),
                &planted,
                &refused.footfalls,
            ),
        }
    }

    /// The cheapest footfalls carrying the body along `path` from `body`,
    /// `planted` holding it.
    fn footfalls(
        &self,
        query: &dyn SectionQuery,
        request: &PlanRequest,
        path: &BodyPath,
        body: &BodyState,
        planted: Contact,
        previous: &[Contact],
    ) -> Result<FootfallPlan, FootfallsRefused> {
        FootfallSearch {
            query,
            params: &self.params,
            settings: &self.settings,
            target_speed: request.target_speed,
            spacing: self
                .settings
                .footfall_spacing(request.target_speed, &self.params),
            body: path,
            previous,
            swinging: request.aim.map(|aim| aim.footfall),
        }
        .search(Footfall {
            contact: planted,
            body_s: body.s,
        })
    }

    /// Where along the path a plan for `request` must carry the body.
    fn goal(&self, request: &PlanRequest) -> f32 {
        let goal = request.body.s + self.settings.body_goal(request.target_speed, &self.params);
        request.reach_limit.map_or(goal, |limit| goal.min(limit))
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
            footfall: None,
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

/// Why `Planner::plan_from` found no plan.
enum Unplanned {
    /// The body's path does not reach the goal: as far as it got, its feet
    /// not yet placed.
    Body(BodyPath),
    /// The feet cannot follow the body's path: the plan as far as they got.
    Feet(Plan),
}

/// Where along the path refused footfalls carried the body, from `body`.
fn reached(refused: &FootfallsRefused, body: &BodyState) -> f32 {
    refused
        .footfalls
        .last()
        .map_or(body.s, |footfall| footfall.body_s)
}

/// The last plan's phases up to one of its footfalls, kept, and where they
/// leave the body.
struct Kept {
    /// From now to the footfall.
    phases: Vec<Phase>,
    /// How many of them are committed.
    committed: usize,
    /// Seconds from now until the footfall lands.
    time: f32,
    /// The body as it lands.
    body: BodyState,
    /// The footfall, which holds the body from then.
    planted: Contact,
}

impl Kept {
    /// The kept phases, then `tail` from the footfall on. A tail with
    /// nothing in it, the footfall landing past the goal, still shows the
    /// footfall: a stance there, for the next plan to keep it by.
    fn followed_by(self, tail: Plan) -> Plan {
        let committed = self.committed;
        let mut phases = self.phases;
        if tail.phases.is_empty() {
            phases.push(Phase {
                start: self.time,
                duration: 0.0,
                contacts: SmallVec::from_slice(&[self.planted]),
                motion: Motion::Grounded {
                    from: self.body.s,
                    to: self.body.s,
                    start_speed: self.body.speed,
                    end_speed: self.body.speed,
                },
            });
        }
        phases.extend(tail.phases.into_iter().map(|phase| Phase {
            start: phase.start + self.time,
            ..phase
        }));
        Plan {
            phases,
            committed,
            target_speed: 0.0,
        }
    }
}

/// Whether `plan` stands on the feet `request` has, so it can be kept. The
/// legs land footfalls on their clock, the plan on its own, and the two
/// agree on footfalls by name:
///
/// - the foot holding the body is the plan's stance footfall, and the foot
///   the plan lands next is the one the legs do;
/// - or the legs are ahead: they have landed the plan's next footfall, and
///   the foot the plan lands after it is the one they land next;
/// - or the legs are behind: the foot they land next swings onto the plan's
///   stance footfall, and is down before the plan's next hop pushes off it;
/// - or no foot is down, between strides at a run, or the plan is just into
///   a flight, and the foot the plan lands next is the one the legs do.
///
/// A plan whose feet the legs did not follow cannot be kept, however well
/// its body path holds.
fn stands_as_asked(plan: &Plan, request: &PlanRequest) -> bool {
    let mut footfalls = plan.footfalls().map(|(_, contact)| contact);
    let (next, after) = (footfalls.next(), footfalls.next());
    let lands_next = |contact: Option<&Contact>| contact.is_none_or(|c| c.limb == request.lead);
    let on = |contact: Option<&Contact>| {
        let footfall = contact.and_then(|contact| contact.footfall);
        footfall.is_some() && request.planted.and_then(|p| p.footfall) == footfall
    };
    let Some(stance) = plan.stance_at(0.0) else {
        return lands_next(next);
    };
    if on(Some(stance)) {
        return lands_next(next);
    }
    if on(next) {
        return lands_next(after);
    }
    let pushed_off = plan.next_hop().unwrap_or(f32::INFINITY);
    let behind = request
        .aim
        .is_some_and(|aim| stance.footfall == Some(aim.footfall) && aim.lands_in <= pushed_off);
    behind || (request.planted.is_none() && lands_next(next))
}

/// The first footfall id a plan for `request` may give a new footfall: past
/// every one the last plan and the legs hold.
fn first_free_id(request: &PlanRequest) -> FootfallId {
    let held = [
        request.previous.and_then(Plan::last_footfall_id),
        request.planted.and_then(|planted| planted.footfall),
        request.aim.map(|aim| aim.footfall),
    ];
    held.into_iter()
        .flatten()
        .max()
        .map_or(FootfallId(1), |last| FootfallId(last.0 + 1))
}

/// How many of the last plan's footfalls it chose knowing what lies past
/// them, up to [`CHURNED_FOOTFALLS`]: its last footfall, and a flight landing
/// on it, were where its search stopped, not what it settled on.
fn settled_footfalls(plan: &Plan) -> usize {
    plan.footfalls()
        .count()
        .saturating_sub(1)
        .min(CHURNED_FOOTFALLS)
}

/// The body path of the last plan, as the next search reads it: its rides,
/// and the flights landing on its settled footfalls, which it settled up to
/// its last settled footfall.
fn previous_path(plan: &Plan) -> PreviousPath {
    let last_settled = settled_footfalls(plan)
        .checked_sub(1)
        .and_then(|last| plan.footfalls().nth(last));
    let (settled_time, settled_until) = last_settled
        .map_or((f32::NEG_INFINITY, f32::NEG_INFINITY), |(time, contact)| {
            (time + STRAY_TIME, contact.at.s)
        });
    let mut path = PreviousPath {
        flights: Vec::new(),
        rides: Vec::new(),
        settled_until,
    };
    for phase in &plan.phases {
        match phase.motion {
            Motion::Flight { takeoff, .. } if phase.start + phase.duration <= settled_time => {
                path.flights.push(FlightSpan {
                    takeoff: takeoff.s,
                    landing: phase.end_s(),
                })
            }
            Motion::Flight { .. } => {}
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
