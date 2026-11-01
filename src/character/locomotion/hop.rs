use nalgebra::Vector3;

use super::effects::EntryEffects;
use super::fall::Fall;
use super::input::{Facts, LocomotionInput, Requests};
use super::on_foot::OnFoot;
use super::requests::{PlannedLanding, PlannedTakeoff};
use super::rule::{MovementRule, RuleSpeeds};
use super::state::LocomotionState;
use super::takeoff::TakeoffPhase;
use crate::character::config::LocomotionConfig;

/// A flight the plan took off on: a running leap, a hop up a step, or a
/// roll-off at a lip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hop {
    pub phase: TakeoffPhase,
    /// Where it comes down, from the takeoff, moved by the plan's landing
    /// requests. A plan quiet for a frame leaves it where it was.
    pub landing: Option<PlannedLanding>,
    /// A roll-off's rise left to keep in flight, in seconds: it left on the
    /// climb the body had, which it keeps for
    /// `LocomotionConfig::roll_off_rise_held` from losing contact, and then
    /// has cancelled. `None` for a hop, which left on a climb of its own.
    pub rise_held: Option<f32>,
}

impl Hop {
    /// Taking off as `takeoff` asks, the body `facts` says. A roll-off asked
    /// for once the body is already off the brow (a plan made in the air,
    /// its ground still forgiven) has spent that much of its held rise.
    pub fn takeoff(takeoff: PlannedTakeoff, facts: &Facts, config: &LocomotionConfig) -> Self {
        Self {
            phase: TakeoffPhase::launch(config.launch_window),
            landing: takeoff.landing,
            rise_held: match takeoff.climb {
                Some(_) => None,
                None => Some(config.roll_off_rise_held - facts.since_contact),
            },
        }
    }

    /// Down: on its feet, the landing foot on its footfall. Its landing
    /// missed: a Fall.
    pub fn tick(self, input: &LocomotionInput) -> LocomotionState {
        let grounded = input.facts.grounded;
        if self.phase.down(grounded) {
            return LocomotionState::OnFoot(OnFoot::default());
        }
        let flying = self.phase == TakeoffPhase::Flight;
        let hop = Self {
            phase: self.phase.advanced(grounded, input.dt),
            landing: input
                .requests
                .landing
                .or(self.landing.map(|landing| landing.later(input.dt))),
            rise_held: self
                .rise_held
                .map(|held| if flying { held - input.dt } else { held }),
        };
        if flying && hop.missed(&input.facts, input.config) {
            return LocomotionState::Fall(Fall);
        }
        LocomotionState::Hop(hop)
    }

    /// Whether the body in the air has missed its landing: the landing's time
    /// has passed by more than the slack, or the body, carried on as it
    /// moves now, comes down out of a leg's reach of it (the stick steered
    /// away).
    fn missed(&self, facts: &Facts, config: &LocomotionConfig) -> bool {
        let Some(landing) = self.landing else {
            return false;
        };
        if landing.in_seconds < -config.hop_landing_slack {
            return true;
        }
        let comes_down = facts.position + facts.velocity * landing.in_seconds.max(0.0);
        let off = (comes_down.x - landing.at.x).hypot(comes_down.z - landing.at.z);
        off > landing.reach
    }

    /// The takeoff's climb, if it asks for one, and its planar speed as the
    /// air cap. Every takeoff cancels ground forgiveness, so grounding drops
    /// as soon as the contacts do.
    pub fn entry(requests: &Requests) -> EntryEffects {
        EntryEffects {
            vertical_speed: requests.takeoff.and_then(|takeoff| takeoff.climb),
            air_speed: requests.takeoff.map(|takeoff| takeoff.planar.norm()),
            cancel_forgiveness: true,
            ..EntryEffects::default()
        }
    }

    /// The ground speed while still in contact, the air speed after. A
    /// roll-off's rise is cancelled once it has been held.
    pub fn movement_rule(&self, move_dir: Vector3<f32>, speeds: &RuleSpeeds) -> MovementRule {
        let speed = match self.phase {
            TakeoffPhase::Launch { .. } => speeds.ground,
            TakeoffPhase::Flight => speeds.air,
        };
        MovementRule {
            clamp_up: self.rise_held.is_some_and(|held| held <= 0.0),
            ..MovementRule::steered(move_dir, speed, speeds.air_accel)
        }
    }

    /// The water takes a Hop that comes down in it.
    pub fn water_takes(&self) -> bool {
        true
    }
}
