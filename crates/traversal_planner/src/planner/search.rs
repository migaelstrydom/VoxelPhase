use crate::params::GaitParams;
use crate::path::PathVec;
use crate::plan::{BodyState, Contact, Limb, Plan};
use crate::section::{Footing, SectionQuery};

use super::body_path::BodySegment;
use super::body_search::{BodySearch, FlightSpan};
use super::footfall_search::{Footfall, FootfallSearch};
use super::plan_builder::build_plan;
use super::settings::PlannerSettings;

/// What one plan is asked for.
#[derive(Clone, Copy, Debug)]
pub struct PlanRequest<'a> {
    pub body: BodyState,
    /// The foot that lands next. The other is taken to be planted under the
    /// body, on fixed ground at its natural offset.
    pub lead: Limb,
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
        let committed = request
            .previous
            .filter(|plan| plan.committed > 0)
            .and_then(|_| previous.first().copied());
        let planted = self.planted(request);
        let mut vetoed = Vec::new();

        loop {
            let body = BodySearch {
                query,
                params: &self.params,
                settings: &self.settings,
                target_speed: request.target_speed,
                vetoed: &vetoed,
            }
            .search(request.body);
            let (path, body_cost) = match &body {
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
                previous: &previous,
                committed,
            }
            .search(Footfall {
                contact: planted,
                body_s: request.body.s,
            });

            match (body.is_ok(), feet) {
                (true, Ok(feet)) => {
                    return Ok(Planned {
                        plan: build_plan(path, &planted, &feet.footfalls),
                        cost: body_cost + feet.cost,
                        diverged: feet.diverged,
                    })
                }
                (false, Ok(feet)) => {
                    return Err(Refusal {
                        partial: build_plan(path, &planted, &feet.footfalls),
                        diverged: feet.diverged,
                    })
                }
                (_, Err(refused)) => {
                    // The feet could not land a flight: try the body's path
                    // again without it, or refuse with what they managed.
                    let reached = refused
                        .footfalls
                        .last()
                        .map_or(request.body.s, |footfall| footfall.body_s);
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
                            return Err(Refusal {
                                partial: build_plan(path, &planted, &refused.footfalls),
                                diverged: refused.diverged,
                            })
                        }
                    }
                }
            }
        }
    }

    /// The foot planted under the body when the plan starts: the other one
    /// from the lead, at its natural offset, on fixed ground.
    fn planted(&self, request: &PlanRequest) -> Contact {
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
