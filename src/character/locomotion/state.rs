use std::mem::discriminant;

use nalgebra::Vector3;

use super::down::{Down, DownPhase};
use super::effects::EntryEffects;
use super::fall::Fall;
use super::hop::Hop;
use super::input::{Facts, LocomotionInput};
use super::jump::{Jump, JumpKind};
use super::ledge::{Ledge, LedgePhase};
use super::on_foot::OnFoot;
use super::rule::{MovementRule, RuleSpeeds};
use super::swim::Swim;
use super::takeoff::TakeoffPhase;
use crate::character::attitude::AttitudeHold;
use crate::character::config::LocomotionConfig;

/// What the character is doing. Only [`tick`](Self::tick) changes it, and
/// only [`enter`](Self::enter) says what changing it does.
///
/// ```text
///   OnFoot ──(plan's takeoff)──► Hop ──(down)──► OnFoot
///      │                          ├──(jump pressed early)──► Jump
///      │                          └──(landing missed)──► Fall
///      ├──(jump pressed)──► Jump ──(down)──► OnFoot
///      └──(grace runs out)──► Fall ──(down)──► OnFoot
///   OnFoot, Hop, Fall, a falling Jump ──(deep water)──► Swim
///   Swim ──(feet down, wadeable)──► OnFoot    Swim ──(out over a drop)──► Fall
///   Swim ──(jump pressed at the surface)──► Jump
///   Jump in flight, Fall, Swim ──(an edge at the hands)──► Ledge
///   OnFoot ──(stick into a climbable edge at the chest)──► Ledge (a mantle)
///   Ledge ──(drop, edge lost)──► Fall    Ledge ──(jump)──► Jump (back-jump)
///   Ledge ──(pull-up done)──► OnFoot
///   any ──(knocked down)──► Down ──(limp, then righted)──► OnFoot
///   Down, its time up ──(deep water)──► Swim
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LocomotionState {
    OnFoot(OnFoot),
    Hop(Hop),
    Jump(Jump),
    Fall(Fall),
    Swim(Swim),
    Ledge(Ledge),
    Down(Down),
}

impl Default for LocomotionState {
    fn default() -> Self {
        Self::OnFoot(OnFoot::default())
    }
}

impl LocomotionState {
    /// The next state. A knockdown comes first, whatever the state; then an
    /// edge the free hands catch, the water out of; then water too deep to
    /// stand in takes whichever states [`water_takes`](Self::water_takes);
    /// otherwise each state's own exits decide.
    pub fn tick(self, input: &LocomotionInput) -> Self {
        if let Some(seconds) = input.facts.knocked {
            return Self::Down(Down::knocked(&self, seconds));
        }
        if let Some(ledge) = Ledge::caught(&self, input) {
            return Self::Ledge(ledge);
        }
        let (facts, config) = (&input.facts, input.config);
        let taken = self.water_takes(facts)
            && config.swim.takes_to_water(
                &facts.immersion,
                facts.position.y,
                config.collider_half_height,
                facts.grounded,
            );
        if taken {
            return Self::Swim(Swim);
        }
        match self {
            Self::OnFoot(state) => state.tick(input),
            Self::Hop(state) => state.tick(input),
            Self::Jump(state) => state.tick(input),
            Self::Fall(state) => state.tick(input),
            Self::Swim(state) => state.tick(input),
            Self::Ledge(state) => state.tick(input),
            Self::Down(state) => state.tick(input),
        }
    }

    /// What changing from `from` to `to` does. Nothing, if `to` is the same
    /// state moved on (grace counting down, a takeoff's phase). The only
    /// place that looks at the state left.
    pub fn enter(from: &Self, to: &Self, input: &LocomotionInput) -> EntryEffects {
        if discriminant(from) == discriminant(to) {
            return EntryEffects::default();
        }
        let (facts, requests, config) = (&input.facts, &input.requests, input.config);
        match to {
            Self::OnFoot(_) => EntryEffects {
                match_edge: matches!(from, Self::Ledge(_))
                    .then_some(requests.edge)
                    .flatten(),
                arm_crouch_lockout: matches!(
                    from,
                    Self::Jump(Jump {
                        kind: JumpKind::LongJump,
                        ..
                    })
                ),
                ..EntryEffects::default()
            },
            Self::Hop(_) => Hop::entry(requests),
            Self::Jump(jump) => jump.entry(facts, requests, config),
            // A missed Hop keeps the air speed it took off with.
            Self::Fall(_) => EntryEffects {
                air_speed: (!matches!(from, Self::Hop(_)))
                    .then(|| facts.horizontal_speed().max(config.walk_speed)),
                ..EntryEffects::default()
            },
            Self::Swim(_) => EntryEffects::default(),
            Self::Ledge(_) => Ledge::entry(requests),
            // A jump pressed before the blow is not taken on getting up.
            Self::Down(_) => EntryEffects {
                consume_jump: true,
                buckle: true,
                ..EntryEffects::default()
            },
        }
    }

    /// [`tick`](Self::tick), then [`enter`](Self::enter) the result.
    pub fn advance(self, input: &LocomotionInput) -> (Self, EntryEffects) {
        let next = self.tick(input);
        (next, Self::enter(&self, &next, input))
    }

    /// This tick's movement rule: a planar target of `move_dir` at the speed
    /// `speeds` gives the state, or a committed velocity with no authority to
    /// steer it elsewhere. A swimmer's target is relative to the water, which
    /// the caller adds.
    pub fn movement_rule(&self, move_dir: Vector3<f32>, speeds: &RuleSpeeds) -> MovementRule {
        match self {
            Self::OnFoot(state) => state.movement_rule(move_dir, speeds),
            Self::Hop(state) => state.movement_rule(move_dir, speeds),
            Self::Jump(state) => state.movement_rule(move_dir, speeds),
            Self::Fall(state) => state.movement_rule(move_dir, speeds),
            Self::Swim(state) => state.movement_rule(move_dir, speeds),
            Self::Ledge(state) => state.movement_rule(),
            Self::Down(state) => state.movement_rule(),
        }
    }

    /// Whether water too deep to stand in takes the body from this state.
    pub fn water_takes(&self, facts: &Facts) -> bool {
        match self {
            Self::OnFoot(state) => state.water_takes(),
            Self::Hop(state) => state.water_takes(),
            Self::Jump(state) => state.water_takes(facts),
            Self::Fall(state) => state.water_takes(),
            Self::Swim(state) => state.water_takes(),
            Self::Ledge(state) => state.water_takes(),
            Self::Down(state) => state.water_takes(),
        }
    }

    /// Whether the body is knocked down: what no creature strikes, so that it
    /// gets up rather than being kept down.
    pub fn is_down(&self) -> bool {
        matches!(self, Self::Down(_))
    }

    /// Whether the hands are free to take an edge: on foot (to mantle), in
    /// a jump's flight, a fall, or the water. Not in a Hop, whose landing
    /// is planned, nor while a jump is still launching or already holding.
    pub fn hands_free(&self) -> bool {
        match self {
            Self::OnFoot(_) | Self::Fall(_) | Self::Swim(_) => true,
            Self::Jump(jump) => matches!(jump.phase, TakeoffPhase::Flight),
            Self::Hop(_) | Self::Ledge(_) | Self::Down(_) => false,
        }
    }

    /// Whether the character acts at all: grabs, throws, steers. Not while
    /// knocked down.
    pub fn in_control(&self) -> bool {
        !matches!(self, Self::Down(_))
    }

    /// How firmly the body's attitude is held: rigidly, unless knocked down.
    pub fn attitude_hold(&self, config: &LocomotionConfig) -> AttitudeHold {
        match self {
            Self::Down(down) => down.attitude_hold(&config.down),
            _ => AttitudeHold::Rigid,
        }
    }

    /// Whether releasing the jump key cuts the rise (variable-height jump).
    pub fn allows_jump_cutoff(&self) -> bool {
        match self {
            Self::Jump(jump) => jump.allows_cutoff(),
            Self::OnFoot(_)
            | Self::Hop(_)
            | Self::Fall(_)
            | Self::Swim(_)
            | Self::Ledge(_)
            | Self::Down(_) => false,
        }
    }

    /// Short comma-free name for the state, for recorders and reports.
    pub fn tag(&self) -> &'static str {
        match self {
            Self::OnFoot(OnFoot { grace: None }) => "on_foot",
            Self::OnFoot(OnFoot { grace: Some(_) }) => "grace",
            Self::Hop(_) => "hop",
            Self::Jump(_) => "jump",
            Self::Fall(_) => "fall",
            Self::Swim(_) => "swim",
            Self::Ledge(Ledge { phase, .. }) => match phase {
                LedgePhase::Grab { .. } => "ledge_grab",
                LedgePhase::Hang { .. } => "ledge_hang",
                LedgePhase::PullUp { .. } => "ledge_pull_up",
            },
            Self::Down(Down { phase }) => match phase {
                DownPhase::Limp { .. } => "down",
                DownPhase::Gathered { .. } => "gathering",
                DownPhase::Rising { .. } => "rising",
            },
        }
    }
}
