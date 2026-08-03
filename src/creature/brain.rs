use nalgebra::{Point3, Vector3};
use specs::{
    Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, Write, WriteStorage,
};

use super::perception::Perception;
use super::steering;
use crate::character::CharacterIntent;
use crate::components::Position;
use crate::damage::{Dead, Health};
use crate::debug::DebugLog;
use crate::time::Time;

/// What a creature is doing right now.
///
/// A flat FSM rather than a behaviour tree. With one creature a tree is all
/// ceremony and no benefit; the shared structure worth factoring out only
/// becomes visible once several creatures disagree about it. Each variant
/// carries only the state that variant needs, so an illegal combination —
/// searching with no last known position, say — cannot be represented.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Behaviour {
    /// Nothing sensed. Holds still.
    Idle,
    /// Something was sensed but the creature has not committed yet. Turns
    /// toward it and waits out `remaining` before charging, which is what
    /// gives the player the beat of warning that makes an ambush readable.
    Alerted { remaining: f32 },
    /// Closing on a visible target.
    Chasing,
    /// Within attack range. Holds station and circles.
    Attacking {
        strafe_clockwise: bool,
        remaining: f32,
    },
    /// Target lost. Heads to where it was last perceived, then gives up.
    Searching { last_known: Point3<f32> },
    /// Badly hurt. Runs away and does not come back.
    Fleeing,
}

impl Default for Behaviour {
    fn default() -> Self {
        Self::Idle
    }
}

/// Drives one creature's decisions.
///
/// The brain's entire output is a [`CharacterIntent`] — the same struct the
/// keyboard fills for the player. It never touches physics, velocity or
/// animation, so a creature cannot acquire movement abilities the player
/// doesn't have, and any locomotion fix benefits both.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Brain {
    pub behaviour: Behaviour,

    /// Distance at which the creature stops closing and starts attacking.
    pub attack_range: f32,
    /// Seconds spent alerted before committing to a charge.
    pub alert_duration: f32,
    /// Health fraction below which the creature breaks off and flees. Zero
    /// disables it — something mindless fights to the death.
    pub flee_health_fraction: f32,
    /// Seconds before an attacking creature reverses its circling direction.
    /// Holding a direction for a while is what reads as circling rather than
    /// as jitter.
    pub strafe_interval: f32,
    /// How close to a last known position counts as having searched it.
    pub search_arrival_radius: f32,
}

impl Brain {
    /// A melee creature that charges, circles at contact range, and flees when
    /// badly hurt.
    pub fn hunter(attack_range: f32) -> Self {
        Self {
            behaviour: Behaviour::default(),
            attack_range,
            alert_duration: 0.6,
            flee_health_fraction: 0.2,
            strafe_interval: 1.5,
            search_arrival_radius: 1.5,
        }
    }

    /// A creature that never breaks off, however hurt it gets.
    pub fn relentless(mut self) -> Self {
        self.flee_health_fraction = 0.0;
        self
    }
}

/// Turns perception into intent.
///
/// Runs before `CharacterControlSystem` so a decision reaches the body on the
/// frame it is made.
pub struct BrainSystem;

impl<'a> System<'a> for BrainSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        ReadStorage<'a, Perception>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Health>,
        ReadStorage<'a, Dead>,
        WriteStorage<'a, Brain>,
        WriteStorage<'a, CharacterIntent>,
        Write<'a, DebugLog>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            time,
            perceptions,
            positions,
            healths,
            deads,
            mut brains,
            mut intents,
            mut debug_log,
        ) = data;
        let dt = time.delta_seconds();

        let mut awake = 0;
        let mut total = 0;

        for (entity, perception, pos, brain, intent, _) in (
            &entities,
            &perceptions,
            &positions,
            &mut brains,
            &mut intents,
            !&deads,
        )
            .join()
        {
            let me = Point3::from(pos.0);
            // Something with no `Health` cannot be hurt, so it never flees.
            let health_fraction = healths.get(entity).map_or(1.0, |h| h.fraction());

            brain.behaviour = next_behaviour(brain, perception, me, health_fraction, dt);
            *intent = intent_for(brain, perception, me);

            total += 1;
            if !matches!(brain.behaviour, Behaviour::Idle) {
                awake += 1;
            }
            debug_log.add(
                format!("Creature/{}/Behaviour", entity.id()),
                format!(
                    "{:?} target={} intent={:.2}",
                    brain.behaviour,
                    match perception.target {
                        Some(t) if t.visible => "visible".to_string(),
                        Some(t) => format!("remembered {:.1}s", t.since_seen),
                        None => "none".to_string(),
                    },
                    intent.direction.magnitude()
                ),
            );
        }

        debug_log.add("Creature/Count", format!("{} ({} active)", total, awake));
    }
}

/// Decide this frame's behaviour. Pure, so the transition table is testable
/// without a world.
fn next_behaviour(
    brain: &Brain,
    perception: &Perception,
    me: Point3<f32>,
    health_fraction: f32,
    dt: f32,
) -> Behaviour {
    // Fleeing is terminal: a creature that has broken off does not rejoin.
    if matches!(brain.behaviour, Behaviour::Fleeing) {
        return Behaviour::Fleeing;
    }
    if brain.flee_health_fraction > 0.0 && health_fraction <= brain.flee_health_fraction {
        return Behaviour::Fleeing;
    }

    match (brain.behaviour, perception.target) {
        // Nothing sensed and nothing remembered.
        (_, None) => Behaviour::Idle,

        // First contact: hesitate before committing.
        (Behaviour::Idle, Some(target)) if target.visible => Behaviour::Alerted {
            remaining: brain.alert_duration,
        },

        (Behaviour::Alerted { remaining }, Some(target)) => {
            if !target.visible {
                Behaviour::Searching {
                    last_known: target.position,
                }
            } else if remaining - dt <= 0.0 {
                Behaviour::Chasing
            } else {
                Behaviour::Alerted {
                    remaining: remaining - dt,
                }
            }
        }

        (Behaviour::Chasing, Some(target)) => {
            if !target.visible {
                Behaviour::Searching {
                    last_known: target.position,
                }
            } else if target.distance <= brain.attack_range {
                Behaviour::Attacking {
                    strafe_clockwise: me.x >= me.z,
                    remaining: brain.strafe_interval,
                }
            } else {
                Behaviour::Chasing
            }
        }

        (
            Behaviour::Attacking {
                strafe_clockwise,
                remaining,
            },
            Some(target),
        ) => {
            if !target.visible {
                Behaviour::Searching {
                    last_known: target.position,
                }
            } else if target.distance > brain.attack_range {
                Behaviour::Chasing
            } else if remaining - dt <= 0.0 {
                Behaviour::Attacking {
                    strafe_clockwise: !strafe_clockwise,
                    remaining: brain.strafe_interval,
                }
            } else {
                Behaviour::Attacking {
                    strafe_clockwise,
                    remaining: remaining - dt,
                }
            }
        }

        // Head for the last known position. On arrival there is nothing more
        // to do but wait: perception's memory expires on its own, which drops
        // the target and returns the creature to Idle via the `None` arm.
        (Behaviour::Searching { last_known }, Some(target)) => {
            if target.visible {
                Behaviour::Chasing
            } else {
                Behaviour::Searching { last_known }
            }
        }

        (Behaviour::Idle, Some(_)) => Behaviour::Idle,
        (Behaviour::Fleeing, _) => Behaviour::Fleeing,
    }
}

/// Translate a behaviour into the intent that expresses it.
fn intent_for(brain: &Brain, perception: &Perception, me: Point3<f32>) -> CharacterIntent {
    let direction = match brain.behaviour {
        Behaviour::Idle => Vector3::zeros(),

        // Creep toward the target. `CharacterIntent` steers and faces with the
        // same vector, so standing still would also mean not turning to look —
        // the crouch flag below is what makes this read as a wary approach
        // rather than a charge.
        Behaviour::Alerted { .. } => perception
            .target
            .map(|t| steering::seek(me, t.position))
            .unwrap_or_else(Vector3::zeros),

        Behaviour::Chasing => perception
            .target
            .map(|t| steering::seek(me, t.position))
            .unwrap_or_else(Vector3::zeros),

        Behaviour::Attacking {
            strafe_clockwise, ..
        } => perception
            .target
            .map(|t| {
                // Hold contact range while circling, so the creature orbits
                // instead of shouldering into the player and pushing them.
                steering::blend(&[
                    (
                        steering::keep_distance(me, t.position, brain.attack_range, 0.3),
                        1.0,
                    ),
                    (steering::strafe(me, t.position, strafe_clockwise), 1.0),
                ])
            })
            .unwrap_or_else(Vector3::zeros),

        Behaviour::Searching { last_known } => steering::seek(me, last_known),

        Behaviour::Fleeing => perception
            .target
            .map(|t| steering::flee(me, t.position))
            .unwrap_or_else(Vector3::zeros),
    };

    CharacterIntent {
        direction,
        // Reuse the player's own gait modifiers rather than inventing a
        // creature-only speed channel: a fleeing creature sprints, a wary one
        // moves at crouch speed.
        sprint: matches!(brain.behaviour, Behaviour::Fleeing),
        crouch: matches!(brain.behaviour, Behaviour::Alerted { .. }),
        ..CharacterIntent::default()
    }
}

#[cfg(test)]
mod tests {
    use specs::{Builder, World, WorldExt};

    use super::super::perception::PerceivedTarget;
    use super::*;

    fn an_entity() -> specs::Entity {
        let mut world = World::new();
        world.create_entity().build()
    }

    fn perception_of(position: Point3<f32>, distance: f32, visible: bool) -> Perception {
        Perception {
            target: Some(PerceivedTarget {
                entity: an_entity(),
                position,
                distance,
                visible,
                since_seen: if visible { 0.0 } else { 1.0 },
            }),
            ..Perception::ground_creature(20.0)
        }
    }

    fn origin() -> Point3<f32> {
        Point3::origin()
    }

    fn brain_in(behaviour: Behaviour) -> Brain {
        Brain {
            behaviour,
            ..Brain::hunter(2.0)
        }
    }

    #[test]
    fn spotting_a_target_alerts_before_charging() {
        let brain = brain_in(Behaviour::Idle);
        let seen = perception_of(Point3::new(0.0, 0.0, 10.0), 10.0, true);

        let next = next_behaviour(&brain, &seen, origin(), 1.0, 0.016);
        assert!(
            matches!(next, Behaviour::Alerted { .. }),
            "first contact must telegraph, not charge instantly"
        );
    }

    #[test]
    fn the_alert_timer_expires_into_a_chase() {
        let brain = brain_in(Behaviour::Alerted { remaining: 0.01 });
        let seen = perception_of(Point3::new(0.0, 0.0, 10.0), 10.0, true);

        assert_eq!(
            next_behaviour(&brain, &seen, origin(), 1.0, 0.016),
            Behaviour::Chasing
        );
    }

    #[test]
    fn closing_to_attack_range_switches_from_chase_to_attack() {
        let brain = brain_in(Behaviour::Chasing);
        let close = perception_of(Point3::new(0.0, 0.0, 1.0), 1.0, true);

        assert!(matches!(
            next_behaviour(&brain, &close, origin(), 1.0, 0.016),
            Behaviour::Attacking { .. }
        ));
    }

    #[test]
    fn a_target_backing_out_of_range_resumes_the_chase() {
        let brain = brain_in(Behaviour::Attacking {
            strafe_clockwise: true,
            remaining: 1.0,
        });
        let far = perception_of(Point3::new(0.0, 0.0, 9.0), 9.0, true);

        assert_eq!(
            next_behaviour(&brain, &far, origin(), 1.0, 0.016),
            Behaviour::Chasing
        );
    }

    #[test]
    fn losing_sight_mid_chase_searches_the_last_known_position() {
        let brain = brain_in(Behaviour::Chasing);
        let last_seen_at = Point3::new(3.0, 0.0, 4.0);
        let lost = perception_of(last_seen_at, 5.0, false);

        assert_eq!(
            next_behaviour(&brain, &lost, origin(), 1.0, 0.016),
            Behaviour::Searching {
                last_known: last_seen_at
            },
            "a target stepping behind cover must be searched for, not forgotten"
        );
    }

    #[test]
    fn forgetting_the_target_entirely_returns_to_idle() {
        let brain = brain_in(Behaviour::Searching {
            last_known: Point3::new(3.0, 0.0, 4.0),
        });
        let forgotten = Perception::ground_creature(20.0);

        assert_eq!(
            next_behaviour(&brain, &forgotten, origin(), 1.0, 0.016),
            Behaviour::Idle
        );
    }

    #[test]
    fn strafe_direction_reverses_when_its_interval_expires() {
        let brain = brain_in(Behaviour::Attacking {
            strafe_clockwise: true,
            remaining: 0.01,
        });
        let close = perception_of(Point3::new(0.0, 0.0, 1.0), 1.0, true);

        assert_eq!(
            next_behaviour(&brain, &close, origin(), 1.0, 0.016),
            Behaviour::Attacking {
                strafe_clockwise: false,
                remaining: brain.strafe_interval
            },
            "circling must reverse so it reads as orbiting, not drifting off"
        );
    }

    #[test]
    fn heavy_damage_breaks_off_the_attack() {
        let brain = brain_in(Behaviour::Chasing);
        let close = perception_of(Point3::new(0.0, 0.0, 1.0), 1.0, true);

        assert_eq!(
            next_behaviour(&brain, &close, origin(), 0.1, 0.016),
            Behaviour::Fleeing
        );
    }

    #[test]
    fn fleeing_is_terminal_even_after_healing() {
        let brain = brain_in(Behaviour::Fleeing);
        let close = perception_of(Point3::new(0.0, 0.0, 1.0), 1.0, true);

        assert_eq!(
            next_behaviour(&brain, &close, origin(), 1.0, 0.016),
            Behaviour::Fleeing,
            "a creature that has broken off must not rejoin the fight"
        );
    }

    #[test]
    fn a_relentless_brain_never_flees() {
        let brain = Brain {
            behaviour: Behaviour::Chasing,
            ..Brain::hunter(2.0).relentless()
        };
        let close = perception_of(Point3::new(0.0, 0.0, 1.0), 1.0, true);

        assert!(matches!(
            next_behaviour(&brain, &close, origin(), 0.01, 0.016),
            Behaviour::Attacking { .. }
        ));
    }

    #[test]
    fn a_chasing_creature_walks_at_its_target() {
        let brain = brain_in(Behaviour::Chasing);
        let seen = perception_of(Point3::new(0.0, 0.0, 10.0), 10.0, true);

        let intent = intent_for(&brain, &seen, origin());
        assert!(intent.direction.z > 0.9, "heads toward the target");
        assert!(!intent.sprint && !intent.crouch, "at plain walking speed");
    }

    #[test]
    fn a_fleeing_creature_sprints_away() {
        let brain = brain_in(Behaviour::Fleeing);
        let seen = perception_of(Point3::new(0.0, 0.0, 10.0), 10.0, true);

        let intent = intent_for(&brain, &seen, origin());
        assert!(intent.direction.z < -0.9, "heads away from the target");
        assert!(intent.sprint, "flight uses the player's own sprint channel");
    }

    #[test]
    fn an_idle_creature_asks_for_nothing() {
        let intent = intent_for(
            &brain_in(Behaviour::Idle),
            &Perception::ground_creature(20.0),
            origin(),
        );
        assert_eq!(intent.direction, Vector3::zeros());
    }
}
