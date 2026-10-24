use smallvec::{smallvec, SmallVec};

use crate::path::PathVec;
use crate::plan::{BodyState, Contact, Limb, Motion, Phase};
use crate::section::{Foothold, FootholdWindow, SectionQuery};

use super::state::PlanState;
use super::transition::{Candidate, ExpandContext, TransitionModel};

/// Spacing at which the ground along a foot's line is followed, in metres.
const GROUND_SPACING: f32 = 0.04;
/// Half the width of a foot's line across the path, in metres: less than half
/// the spacing of the lanes, so only the foot's own lane is followed.
const GROUND_LINE_HALF_WIDTH: f32 = 0.05;
/// Largest rise between neighbouring samples that is still one piece of
/// ground, in metres.
const GROUND_STEP: f32 = 0.05;
/// Slowest speed a stride is planned at, in m/s: below it a stride would take
/// unboundedly long.
const MIN_PLANNING_SPEED: f32 = 0.3;

/// One step to the other foot: the body rides the floor, or rides to a
/// takeoff and flies, and the other foot lands on a foothold ahead.
///
/// The body lands with the foot, behind it by a fraction of the step
/// (`landing_leads`: halfway when walking, nearly over it when hopping),
/// resting on a body floor it can stand on: the capsule has to be held up by
/// the ground under it, so a gap wider than the capsule is flown, not
/// straddled.
///
/// ```text
///      stance            landing
///        ▼  b ──ride──► t ⌒flight⌒ b'  ▼
///        f                             f'      b' = f' - lead (f' - f)
/// ```
///
/// The foot pushes off with the body over or past it (`t >= f`), so the body
/// is held over every footfall: across stepping stones it rides each stone.
#[derive(Clone, Copy, Debug, Default)]
pub struct Stride;

impl TransitionModel for Stride {
    fn expand(&self, from: &PlanState, context: &ExpandContext, out: &mut Vec<Candidate>) {
        let params = context.params;
        let limb = from.stance.limb.other();
        let stance = from.stance.at;
        let window = FootholdWindow {
            s: stance.s + params.stride.min_length..stance.s + params.stride.max_length,
            d: params.foot.reach(limb),
            spacing: context.settings.footfall_spacing,
        };
        let mut footholds = Vec::new();
        context.query.footholds(&window, &mut footholds);
        for foothold in &footholds {
            self.step_to(from, limb, foothold, context, out);
        }
    }
}

impl Stride {
    fn step_to(
        &self,
        from: &PlanState,
        limb: Limb,
        foothold: &Foothold,
        context: &ExpandContext,
        out: &mut Vec<Candidate>,
    ) {
        let params = context.params;
        let stance = from.stance.at;
        let rise = foothold.at.z - stance.z;
        if foothold.edge_margin < params.foot.min_edge_margin
            || rise > params.stride.max_rise
            || -rise > params.stride.max_drop
        {
            return;
        }
        let contact = Contact {
            limb,
            at: foothold.at,
            normal: foothold.normal,
            footing: foothold.footing,
        };
        let step_cost = self.step_cost(from, limb, foothold, context);
        let step = foothold.at.s - stance.s;
        let mut last_landing = None;
        for lead in &context.settings.landing_leads {
            let landing_s = foothold.at.s - lead * step;
            if last_landing != Some(landing_s) && landing_s > from.body.s {
                last_landing = Some(landing_s);
                self.land_at(from, contact, landing_s, step_cost, context, out);
            }
        }
    }

    /// The ways to carry the body to land at `landing_s` as `contact` lands:
    /// riding all the way, or riding to a takeoff and flying.
    fn land_at(
        &self,
        from: &PlanState,
        contact: Contact,
        landing_s: f32,
        step_cost: f32,
        context: &ExpandContext,
        out: &mut Vec<Candidate>,
    ) {
        let (params, query) = (context.params, context.query);
        let stance = from.stance.at;
        let Some(landing_floor) = query.floor(landing_s) else {
            return;
        };
        let steepest = params.body.max_ground_slope_degrees.to_radians();
        if landing_floor.normal.z < steepest.cos() {
            return;
        }
        if !reaches(landing_floor.z, contact.at.z, context) {
            return;
        }
        // Where the capsule rests as it lands, along the path.
        let support = landing_s - params.body.radius * landing_floor.normal.s;
        let held = [contact.at, stance]
            .iter()
            .any(|foot| on_ground_reaching(query, foot, support));
        let step_cost = if held {
            step_cost
        } else {
            step_cost + context.settings.costs.unheld_body
        };

        let start = BodyState {
            speed: from.body.speed.max(MIN_PLANNING_SPEED),
            ..from.body
        };
        let ride = query.ride(start.s, landing_s, params);

        // Riding all the way, the stance foot holds the body until it lands.
        if ride.end >= landing_s && reaches(landing_floor.z, stance.z, context) {
            let (phase, end_speed) = self.grounded(from, start, landing_s, context);
            out.push(Candidate {
                landing: self.landing(from, contact, landing_s, landing_floor.z, end_speed, &phase),
                cost: step_cost + context.settings.costs.ride_travel * ride.vertical_travel,
                phases: smallvec![phase],
            });
        }

        // The foot pushes off with the body over or past it.
        let earliest_takeoff = start.s.max(stance.s);
        let latest_takeoff = ride.end.min(landing_s - context.settings.min_flight);
        let mut last_takeoff = None;
        for setback in &context.settings.takeoff_setbacks {
            let takeoff = (latest_takeoff - setback).max(earliest_takeoff);
            if takeoff > latest_takeoff || last_takeoff == Some(takeoff) {
                continue;
            }
            last_takeoff = Some(takeoff);
            if let Some(candidate) = self.flight(
                from,
                start,
                takeoff,
                (contact, landing_s, landing_floor.z),
                step_cost,
                context,
            ) {
                out.push(candidate);
            }
        }
    }

    /// Riding the floor from `start` to `to`, the speed moving toward the
    /// target as fast as the drive allows. Returns the phase and its end speed.
    fn grounded(
        &self,
        from: &PlanState,
        start: BodyState,
        to: f32,
        context: &ExpandContext,
    ) -> (Phase, f32) {
        let distance = to - start.s;
        let change = context.params.body.max_acceleration * distance / start.speed;
        let end_speed = (start.speed + (context.target_speed - start.speed).clamp(-change, change))
            .max(MIN_PLANNING_SPEED);
        let phase = Phase {
            start: from.time,
            duration: 2.0 * distance / (start.speed + end_speed),
            contacts: smallvec![from.stance],
            motion: Motion::Grounded {
                from: start.s,
                to,
                start_speed: start.speed,
                end_speed,
            },
        };
        (phase, end_speed)
    }

    /// Riding to `takeoff`, then flying to land at `landing_s`.
    fn flight(
        &self,
        from: &PlanState,
        start: BodyState,
        takeoff: f32,
        (contact, landing_s, landing_z): (Contact, f32, f32),
        step_cost: f32,
        context: &ExpandContext,
    ) -> Option<Candidate> {
        let (params, query) = (context.params, context.query);
        // The stance, however short, is where the stance foot holds the body.
        let (stance, speed) = self.grounded(from, start, takeoff, context);
        let takeoff_z = if takeoff > start.s {
            query.floor(takeoff)?.z
        } else {
            start.z
        };
        // Flying, it lets go at takeoff.
        if !reaches(takeoff_z, from.stance.at.z, context) {
            return None;
        }
        let ride = query.ride(start.s, takeoff, params);
        let cost = step_cost
            + context.settings.costs.flight
            + context.settings.costs.ride_travel * ride.vertical_travel;
        let time = stance.start + stance.duration;

        let duration = (landing_s - takeoff) / speed;
        let gravity = params.gravity;
        let climb = (landing_z - takeoff_z + 0.5 * gravity * duration * duration) / duration;
        if !(0.0..=params.flight.max_takeoff_climb).contains(&climb) {
            return None;
        }
        let cost = cost + context.settings.costs.takeoff_climb * climb * climb;
        let flight = Phase {
            start: time,
            duration,
            contacts: SmallVec::new(),
            motion: Motion::Flight {
                takeoff: BodyState {
                    s: takeoff,
                    z: takeoff_z,
                    speed,
                    climb,
                },
                gravity,
            },
        };
        Some(Candidate {
            landing: self.landing(from, contact, landing_s, landing_z, speed, &flight),
            phases: smallvec![stance, flight],
            cost,
        })
    }

    fn landing(
        &self,
        from: &PlanState,
        stance: Contact,
        s: f32,
        z: f32,
        speed: f32,
        last: &Phase,
    ) -> PlanState {
        PlanState {
            time: last.start + last.duration,
            body: BodyState {
                s,
                z,
                speed,
                climb: 0.0,
            },
            stance,
            footfalls: from.footfalls + 1,
        }
    }

    /// The step's length off the natural one for the speed asked for, the
    /// foot off its natural offset, and the foot near an edge.
    fn step_cost(
        &self,
        from: &PlanState,
        limb: Limb,
        foothold: &Foothold,
        context: &ExpandContext,
    ) -> f32 {
        let (params, costs) = (context.params, &context.settings.costs);
        let speed = context.target_speed.max(MIN_PLANNING_SPEED);
        let natural_length = (speed / context.settings.step_rate)
            .clamp(params.stride.min_length, params.stride.max_length);
        let length = foothold.at.s - from.stance.at.s;
        let length_off = (length - natural_length) / natural_length;
        let reach = params.foot.reach_in.max(params.foot.reach_out);
        let sideways_off = (foothold.at.d - params.foot.natural_offset(limb)) / reach;
        let near_edge = (costs.comfortable_edge_margin - foothold.edge_margin).max(0.0)
            / costs.comfortable_edge_margin;
        costs.step_length * length_off * length_off
            + costs.sideways * sideways_off * sideways_off
            + costs.edge * near_edge * near_edge
    }
}

/// Whether a foot at height `foot_z` is within reach of the body with its
/// bottom-sphere centre at `body_z`.
fn reaches(body_z: f32, foot_z: f32, context: &ExpandContext) -> bool {
    let foot = &context.params.foot;
    let bottom = body_z - context.params.body.radius;
    foot_z >= bottom - foot.below_body && foot_z <= bottom + foot.above_body
}

/// Whether the ground under `foot` runs unbroken along its line to `support`:
/// the capsule resting there is on what that foot stands on, not on a stone
/// between the feet that neither touches.
fn on_ground_reaching(query: &dyn SectionQuery, foot: &PathVec, support: f32) -> bool {
    let (from, to) = (foot.s.min(support), foot.s.max(support));
    let window = FootholdWindow {
        s: from - GROUND_SPACING..to + GROUND_SPACING,
        d: foot.d - GROUND_LINE_HALF_WIDTH..foot.d + GROUND_LINE_HALF_WIDTH,
        spacing: GROUND_SPACING,
    };
    let mut ground = Vec::new();
    query.footholds(&window, &mut ground);
    ground.sort_by(|a, b| a.at.s.total_cmp(&b.at.s));
    let (Some(first), Some(last)) = (ground.first(), ground.last()) else {
        return false;
    };
    let reach = 2.0 * GROUND_SPACING;
    first.at.s <= from + reach
        && last.at.s >= to - reach
        && ground.windows(2).all(|pair| {
            pair[1].at.s - pair[0].at.s <= reach
                && (pair[1].at.z - pair[0].at.z).abs() <= GROUND_STEP
        })
}
