use nalgebra::Vector3;

use super::effects::EntryEffects;
use super::fall::Fall;
use super::input::{LocomotionInput, Requests};
use super::jump::Jump;
use super::on_foot::OnFoot;
use super::rule::MovementRule;
use super::state::LocomotionState;
use crate::character::ledge::{Edge, LedgeConfig, SensedEdge};

/// Hands on an edge: caught from a jump, a fall or the water, or a mantle
/// from the ground. Leaves by dropping (a Fall), jumping backwards (a Jump),
/// pulling up (OnFoot on the top), or losing the edge (a Fall).
/// `docs/LEDGE_GRABS.md`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ledge {
    /// The edge held, in its body's frame.
    pub edge: Edge,
    pub phase: LedgePhase,
}

/// Where in a ledge the body is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LedgePhase {
    /// The hands landing on the edge: the stick is ignored for `settle`.
    Grab { elapsed: f32 },
    /// Hanging, shimmying with the stick along the edge. `lean` is a drop or
    /// a pull-up being held towards commitment.
    Hang { lean: Option<Lean> },
    /// Climbing onto the top, `duration` seconds in all.
    PullUp { elapsed: f32, duration: f32 },
}

/// A drop or a pull-up the stick is holding towards.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lean {
    pub towards: LeanTowards,
    /// Seconds held so far.
    pub held: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeanTowards {
    Drop,
    PullUp,
}

/// The stick, read against an edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StickOnEdge {
    Neutral,
    /// Into the face.
    Towards,
    /// Out from the face.
    Away,
    /// Along the edge.
    Along,
}

impl StickOnEdge {
    /// How `move_dir` reads against a face whose outward normal is
    /// `outward`.
    pub fn of(move_dir: Vector3<f32>, outward: Vector3<f32>, config: &LedgeConfig) -> Self {
        let push = Vector3::new(move_dir.x, 0.0, move_dir.z);
        if push.norm() < config.stick_dead_zone {
            return Self::Neutral;
        }
        let out = push.normalize().dot(&outward);
        let cone = config.lean_cone_degrees.to_radians().cos();
        if out >= cone {
            Self::Away
        } else if out <= -cone {
            Self::Towards
        } else {
            Self::Along
        }
    }
}

impl Ledge {
    /// The ledge `from` takes, if its hands are free and the hands found an
    /// edge: climbed straight away if it is under the shoulders, climbable,
    /// and the stick pushes into it (a mantle); otherwise, in the air or the
    /// water, grabbed if it is at the chest or above. On foot only the
    /// mantle.
    pub fn caught(from: &LocomotionState, input: &LocomotionInput) -> Option<Self> {
        let (facts, requests) = (&input.facts, &input.requests);
        let config = &input.config.ledge;
        let sensed = requests.edge.filter(|_| from.hands_free())?;
        let lip = sensed.at.grip().y - facts.position.y;
        let into = StickOnEdge::of(requests.move_dir, sensed.at.outward.into_inner(), config)
            == StickOnEdge::Towards;
        let mantle = into && lip <= config.shoulder && sensed.edge.climbable();
        let grab = lip >= config.catch_low && !matches!(from, LocomotionState::OnFoot(_));
        let phase = match (mantle, grab) {
            (true, _) => LedgePhase::PullUp {
                elapsed: 0.0,
                duration: config.mantle_time,
            },
            (false, true) => LedgePhase::Grab { elapsed: 0.0 },
            (false, false) => return None,
        };
        Some(Self {
            edge: sensed.edge,
            phase,
        })
    }

    /// The edge lost: a Fall. A pull-up runs its course to OnFoot. Otherwise
    /// jump is a back-jump and crouch a drop, at once; after the settle the
    /// stick held away drops and held in pulls up.
    pub fn tick(self, input: &LocomotionInput) -> LocomotionState {
        let (requests, dt) = (&input.requests, input.dt);
        let config = &input.config.ledge;
        let Some(sensed) = requests
            .edge
            .filter(|sensed| sensed.edge.body == self.edge.body)
        else {
            return LocomotionState::Fall(Fall);
        };
        let held = |phase| {
            LocomotionState::Ledge(Self {
                edge: sensed.edge,
                phase,
            })
        };
        let let_go = || {
            if requests.jump {
                Some(LocomotionState::Jump(Jump::back_jump(
                    requests, &sensed, config,
                )))
            } else if requests.drop {
                Some(LocomotionState::Fall(Fall))
            } else {
                None
            }
        };
        match self.phase {
            LedgePhase::PullUp { elapsed, duration } if elapsed + dt >= duration => {
                LocomotionState::OnFoot(OnFoot::default())
            }
            LedgePhase::PullUp { elapsed, duration } => held(LedgePhase::PullUp {
                elapsed: elapsed + dt,
                duration,
            }),
            LedgePhase::Grab { elapsed } => let_go().unwrap_or_else(|| {
                held(if elapsed + dt < config.settle {
                    LedgePhase::Grab {
                        elapsed: elapsed + dt,
                    }
                } else {
                    LedgePhase::Hang { lean: None }
                })
            }),
            LedgePhase::Hang { lean } => {
                let_go().unwrap_or_else(|| match leaned(lean, requests, &sensed, config, dt) {
                    Some(Lean {
                        towards: LeanTowards::Drop,
                        held: t,
                    }) if t >= config.commit_hold => LocomotionState::Fall(Fall),
                    Some(Lean {
                        towards: LeanTowards::PullUp,
                        held: t,
                    }) if t >= config.commit_hold && sensed.edge.climbable() => {
                        held(LedgePhase::PullUp {
                            elapsed: 0.0,
                            duration: config.pull_up_time,
                        })
                    }
                    lean => held(LedgePhase::Hang {
                        lean: lean.map(|lean| Lean {
                            held: lean.held.min(config.commit_hold),
                            ..lean
                        }),
                    }),
                })
            }
        }
    }

    /// Entering: the body takes the edge's velocity, so the grip starts with
    /// nothing to stop, and a jump pressed on the way up is spent.
    pub fn entry(requests: &Requests) -> EntryEffects {
        EntryEffects {
            match_edge: requests.edge,
            consume_jump: true,
            cancel_forgiveness: true,
            ..EntryEffects::default()
        }
    }

    /// No authority anywhere: the grip moves the body.
    pub fn movement_rule(&self) -> MovementRule {
        MovementRule::slack()
    }

    /// The water does not take a body hanging from an edge.
    pub fn water_takes(&self) -> bool {
        false
    }
}

/// The lean the stick holds this frame: kept and grown while it holds the
/// same way, started afresh when it turns, gone when it lets go or shimmies.
fn leaned(
    lean: Option<Lean>,
    requests: &Requests,
    sensed: &SensedEdge,
    config: &LedgeConfig,
    dt: f32,
) -> Option<Lean> {
    let towards = match StickOnEdge::of(requests.move_dir, sensed.at.outward.into_inner(), config) {
        StickOnEdge::Away => LeanTowards::Drop,
        StickOnEdge::Towards => LeanTowards::PullUp,
        StickOnEdge::Neutral | StickOnEdge::Along => return None,
    };
    let held = match lean {
        Some(lean) if lean.towards == towards => lean.held + dt,
        _ => dt,
    };
    Some(Lean { towards, held })
}
