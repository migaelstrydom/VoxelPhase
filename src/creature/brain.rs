use nalgebra::{Point3, Vector3};
use rand::Rng;
use specs::{
    Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, Write, WriteStorage,
};

use super::perception::{PerceivedTarget, Perception};
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
    /// Badly hurt, or frightened. Runs away. `calm` counts the seconds since
    /// it last perceived whatever routed it; a rout that has nothing left to
    /// run from ends.
    Fleeing { calm: f32 },
    /// Nothing to react to, so it mills about. `heading` is held for
    /// `remaining` seconds and then nudged, which reads as an animal
    /// pottering rather than as one teleporting its mind every frame.
    Wandering {
        heading: Vector3<f32>,
        remaining: f32,
    },
}

impl Default for Behaviour {
    fn default() -> Self {
        Self::Idle
    }
}

/// What a creature does about what it senses.
///
/// The two dispositions want opposite things from the same perception, and
/// separating them here keeps each transition table short enough to read.
/// A hunter's is unchanged from when it was the only one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Temperament {
    /// Closes on what it senses, circles at contact range, and breaks off
    /// only when badly hurt.
    Hunter,
    /// Runs from what it senses and mills about when it senses nothing.
    /// Never attacks, and rejoins the world as soon as the fright is out of
    /// range — where a hunter's rout has to be outrun first.
    Skittish {
        /// Distance within which a sensed target is frightening. Beyond
        /// it the creature carries on wandering, which is what lets a
        /// player creep up rather than being spotted across a valley.
        flee_range: f32,
    },
}

/// Seconds of lost sight a committed creature tolerates before it gives up
/// chasing and starts searching.
///
/// Sight is not a steady signal: a target crossing behind a rock, a rise in
/// the ground, or the edge of the vision cone drops out for a frame at a time
/// and comes straight back. Reacting to every one of those is what makes a
/// creature lurch between charging and casting about.
const SIGHT_LOSS_GRACE: f32 = 0.75;

/// Seconds without perceiving anything before a routed creature calms down
/// and rejoins the world.
const ROUT_RECOVERY: f32 = 4.0;

/// How far past its attack range a target must get before an attacking
/// creature resumes the chase, as a multiple of that range.
///
/// Without this the transition is a bare threshold sitting in the middle of
/// the standoff band, and a target drifting across it flips the creature
/// between closing in and backing off every frame — from outside, a creature
/// that cannot make its mind up.
const ATTACK_RELEASE: f32 = 1.25;

/// Half-width of the standoff band, as a fraction of attack range.
///
/// Scaled rather than absolute because a fixed band means something very
/// different to a boulder that attacks from a metre and a creature that
/// attacks from five. It must stay inside `ATTACK_RELEASE`, or a creature can
/// settle at a distance its own brain calls out of range.
const STANDOFF_BAND_FRACTION: f32 = 0.15;

/// What a creature knows about its own injuries.
///
/// Two readings of the same `Health`, because breaking off needs both: how
/// badly hurt it is, and whether that hurt is fresh.
#[derive(Debug, Clone, Copy)]
struct Wounds {
    /// Remaining hit points as a 0..1 fraction.
    fraction: f32,
    /// Cumulative damage absorbed.
    taken: f32,
}

impl Wounds {
    /// What something that cannot be hurt reports.
    fn unhurt() -> Self {
        Self {
            fraction: 1.0,
            taken: 0.0,
        }
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
    pub temperament: Temperament,

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
    /// Seconds a wandering creature holds a heading before nudging it.
    pub wander_interval: f32,
    /// Largest nudge to a wandering heading, in radians. A quarter turn
    /// wanders; a half turn dithers on the spot.
    pub wander_turn: f32,

    /// `Health::taken` seen last frame.
    ///
    /// Breaking off is triggered by *being hurt while badly injured*, not by
    /// *being badly injured*. The difference is the whole reason a rout can
    /// end: a creature that ran away is still under the threshold when it
    /// calms down, so a level trigger would send it straight back into
    /// flight the moment it laid eyes on anyone. Cumulative damage is what
    /// the edge is measured against rather than remaining hit points,
    /// because hit points stop falling at the death floor while the blows
    /// keep coming. Written by `BrainSystem`.
    pub previous_damage_taken: f32,
}

impl Brain {
    /// A melee creature that charges, circles at contact range, and flees when
    /// badly hurt.
    pub fn hunter(attack_range: f32) -> Self {
        Self {
            behaviour: Behaviour::default(),
            temperament: Temperament::Hunter,
            attack_range,
            alert_duration: 0.6,
            flee_health_fraction: 0.2,
            strafe_interval: 1.5,
            search_arrival_radius: 1.5,
            wander_interval: 1.2,
            wander_turn: std::f32::consts::FRAC_PI_2,
            previous_damage_taken: 0.0,
        }
    }

    /// Whether a fresh injury is enough to break this creature off.
    fn breaks_off_at(&self, health: Wounds) -> bool {
        self.flee_health_fraction > 0.0
            && health.fraction <= self.flee_health_fraction
            && health.taken > self.previous_damage_taken
    }

    /// A creature that never breaks off, however hurt it gets.
    pub fn relentless(mut self) -> Self {
        self.flee_health_fraction = 0.0;
        self
    }

    /// A creature that wanders until something comes within `flee_range`,
    /// then runs. It has no attack, so `attack_range` is meaningless to it.
    pub fn skittish(flee_range: f32) -> Self {
        Self {
            behaviour: Behaviour::Wandering {
                heading: Vector3::zeros(),
                remaining: 0.0,
            },
            temperament: Temperament::Skittish { flee_range },
            attack_range: 0.0,
            ..Self::hunter(0.0)
        }
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
            let wounds = healths.get(entity).map_or(Wounds::unhurt(), |h| Wounds {
                fraction: h.fraction(),
                taken: h.taken,
            });

            brain.behaviour = next_behaviour(brain, perception, me, wounds, dt);
            brain.previous_damage_taken = wounds.taken;
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
    wounds: Wounds,
    dt: f32,
) -> Behaviour {
    match brain.temperament {
        Temperament::Hunter => hunter_behaviour(brain, perception, me, wounds, dt),
        Temperament::Skittish { flee_range } => {
            skittish_behaviour(brain, perception, flee_range, dt)
        }
    }
}

/// A creature that wanders, and runs from whatever it notices nearby.
///
/// Flight ends here as soon as the fright is out of range or out of memory,
/// with no clock to run down. A hunter's rout is the slower thing: it has to
/// stay shaken for `ROUT_RECOVERY` before it will face anyone again.
fn skittish_behaviour(
    brain: &Brain,
    perception: &Perception,
    flee_range: f32,
    dt: f32,
) -> Behaviour {
    let frightened = perception
        .target
        .is_some_and(|target| target.distance <= flee_range);
    if frightened {
        return Behaviour::Fleeing { calm: 0.0 };
    }

    match brain.behaviour {
        Behaviour::Wandering { heading, remaining } if remaining - dt > 0.0 => {
            Behaviour::Wandering {
                heading,
                remaining: remaining - dt,
            }
        }
        // Either the heading has run its course or the creature has just
        // stopped running. Either way, pick a new one.
        Behaviour::Wandering { heading, .. } => Behaviour::Wandering {
            heading: nudge(heading, brain.wander_turn),
            remaining: brain.wander_interval,
        },
        _ => Behaviour::Wandering {
            heading: nudge(Vector3::zeros(), std::f32::consts::PI),
            remaining: brain.wander_interval,
        },
    }
}

/// Turn a heading by up to `max_turn` radians either way. A zero heading
/// is replaced outright, which is how a creature picks its first one.
fn nudge(heading: Vector3<f32>, max_turn: f32) -> Vector3<f32> {
    let mut rng = rand::thread_rng();
    let angle = match heading.try_normalize(1e-4) {
        Some(current) => current.z.atan2(current.x) + rng.gen_range(-max_turn..max_turn),
        None => rng.gen_range(-std::f32::consts::PI..std::f32::consts::PI),
    };
    Vector3::new(angle.cos(), 0.0, angle.sin())
}

fn hunter_behaviour(
    brain: &Brain,
    perception: &Perception,
    me: Point3<f32>,
    wounds: Wounds,
    dt: f32,
) -> Behaviour {
    // A rout runs until the creature has shaken the thing that caused it.
    // Then it stands down — hurt, wary, and able to be come upon again.
    // The clock is carried rather than read off `since_seen` because a
    // creature can be routed by something it never saw, and a target it has
    // no memory of would otherwise count as shaken on the first frame.
    if let Behaviour::Fleeing { calm } = brain.behaviour {
        let calm = if perception.sees_target() {
            0.0
        } else {
            calm + dt
        };
        return if calm >= ROUT_RECOVERY {
            Behaviour::Idle
        } else {
            Behaviour::Fleeing { calm }
        };
    }
    if brain.breaks_off_at(wounds) {
        return Behaviour::Fleeing { calm: 0.0 };
    }

    match (brain.behaviour, perception.target) {
        // Nothing sensed and nothing remembered.
        (_, None) => Behaviour::Idle,

        // First contact: hesitate before committing.
        (Behaviour::Idle, Some(target)) if target.visible => Behaviour::Alerted {
            remaining: brain.alert_duration,
        },

        (Behaviour::Alerted { remaining }, Some(target)) => {
            if !still_engaged(target) {
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
            if !still_engaged(target) {
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
            if !still_engaged(target) {
                Behaviour::Searching {
                    last_known: target.position,
                }
            } else if target.distance > brain.attack_range * ATTACK_RELEASE {
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
        (Behaviour::Fleeing { calm }, _) => Behaviour::Fleeing { calm },
        // A hunter never wanders; if it somehow got there, it stops.
        (Behaviour::Wandering { .. }, _) => Behaviour::Idle,
    }
}

/// Whether a creature that has already committed still counts the target as
/// in front of it.
///
/// Deliberately more generous than `visible`: acquiring a target needs a clear
/// sighting, but holding one only needs to have had a recent one.
fn still_engaged(target: PerceivedTarget) -> bool {
    target.visible || target.since_seen < SIGHT_LOSS_GRACE
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
                        steering::keep_distance(
                            me,
                            t.position,
                            brain.attack_range,
                            brain.attack_range * STANDOFF_BAND_FRACTION,
                        ),
                        1.0,
                    ),
                    (steering::strafe(me, t.position, strafe_clockwise), 1.0),
                ])
            })
            .unwrap_or_else(Vector3::zeros),

        Behaviour::Searching { last_known } => {
            // Having arrived, stand and look rather than walk on. Seeking a
            // point it is already standing on sends the creature over it and
            // back again, which from outside reads as one that has lost
            // interest and is pacing away from the player it was chasing.
            let offset = Vector3::new(last_known.x - me.x, 0.0, last_known.z - me.z);
            if offset.magnitude() <= brain.search_arrival_radius {
                Vector3::zeros()
            } else {
                steering::seek(me, last_known)
            }
        }

        Behaviour::Fleeing { .. } => perception
            .target
            .map(|t| steering::flee(me, t.position))
            .unwrap_or_else(Vector3::zeros),

        Behaviour::Wandering { heading, .. } => heading,
    };

    CharacterIntent {
        direction,
        // Reuse the player's own gait modifiers rather than inventing a
        // creature-only speed channel: a fleeing creature sprints, a wary one
        // moves at crouch speed.
        sprint: matches!(brain.behaviour, Behaviour::Fleeing { .. }),
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
        lost_perception_of(position, distance, if visible { 0.0 } else { 1.0 }, visible)
    }

    /// A perception whose target was last seen `since_seen` seconds ago.
    fn lost_perception_of(
        position: Point3<f32>,
        distance: f32,
        since_seen: f32,
        visible: bool,
    ) -> Perception {
        Perception {
            target: Some(PerceivedTarget {
                entity: an_entity(),
                position,
                distance,
                visible,
                since_seen,
            }),
            ..Perception::ground_creature(20.0)
        }
    }

    fn origin() -> Point3<f32> {
        Point3::origin()
    }

    /// A creature nothing has touched.
    fn unhurt() -> Wounds {
        Wounds::unhurt()
    }

    /// A creature just hit down to `fraction` of its hit points.
    fn wounded_to(fraction: f32) -> Wounds {
        Wounds {
            fraction,
            taken: 10.0,
        }
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

        let next = next_behaviour(&brain, &seen, origin(), unhurt(), 0.016);
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
            next_behaviour(&brain, &seen, origin(), unhurt(), 0.016),
            Behaviour::Chasing
        );
    }

    #[test]
    fn closing_to_attack_range_switches_from_chase_to_attack() {
        let brain = brain_in(Behaviour::Chasing);
        let close = perception_of(Point3::new(0.0, 0.0, 1.0), 1.0, true);

        assert!(matches!(
            next_behaviour(&brain, &close, origin(), unhurt(), 0.016),
            Behaviour::Attacking { .. }
        ));
    }

    #[test]
    fn a_target_barely_out_of_range_does_not_resume_the_chase() {
        let brain = brain_in(Behaviour::Attacking {
            strafe_clockwise: true,
            remaining: 1.0,
        });
        // Just outside attack range, but well inside the release margin —
        // the distance a circling creature crosses constantly.
        let drifting = perception_of(Point3::new(0.0, 0.0, 2.2), 2.2, true);

        assert!(
            matches!(
                next_behaviour(&brain, &drifting, origin(), unhurt(), 0.016),
                Behaviour::Attacking { .. }
            ),
            "a target drifting across the attack threshold must not flip the behaviour"
        );
    }

    #[test]
    fn a_target_backing_out_of_range_resumes_the_chase() {
        let brain = brain_in(Behaviour::Attacking {
            strafe_clockwise: true,
            remaining: 1.0,
        });
        let far = perception_of(Point3::new(0.0, 0.0, 9.0), 9.0, true);

        assert_eq!(
            next_behaviour(&brain, &far, origin(), unhurt(), 0.016),
            Behaviour::Chasing
        );
    }

    #[test]
    fn losing_sight_mid_chase_searches_the_last_known_position() {
        let brain = brain_in(Behaviour::Chasing);
        let last_seen_at = Point3::new(3.0, 0.0, 4.0);
        let lost = perception_of(last_seen_at, 5.0, false);

        assert_eq!(
            next_behaviour(&brain, &lost, origin(), unhurt(), 0.016),
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
            next_behaviour(&brain, &forgotten, origin(), unhurt(), 0.016),
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
            next_behaviour(&brain, &close, origin(), unhurt(), 0.016),
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
            next_behaviour(&brain, &close, origin(), wounded_to(0.1), 0.016),
            Behaviour::Fleeing { calm: 0.0 }
        );
    }

    /// Hit points stop falling at the death floor while the blows keep
    /// landing, so a rout triggered by remaining health could only ever fire
    /// once per creature.
    #[test]
    fn a_creature_parked_at_the_death_floor_can_still_be_routed_again() {
        let brain = Brain {
            behaviour: Behaviour::Chasing,
            previous_damage_taken: 400.0,
            ..Brain::hunter(2.0)
        };
        let close = perception_of(Point3::new(0.0, 0.0, 1.0), 1.0, true);
        let floored = Wounds {
            fraction: 1.0 / 55.0,
            taken: 900.0,
        };

        assert_eq!(
            next_behaviour(&brain, &close, origin(), floored, 0.016),
            Behaviour::Fleeing { calm: 0.0 },
            "another grenade on a creature already at the floor must still rout it"
        );
    }

    #[test]
    fn a_rout_holds_while_the_thing_that_caused_it_is_still_there() {
        let brain = brain_in(Behaviour::Fleeing { calm: 3.0 });
        let close = perception_of(Point3::new(0.0, 0.0, 1.0), 1.0, true);

        assert_eq!(
            next_behaviour(&brain, &close, origin(), unhurt(), 0.016),
            Behaviour::Fleeing { calm: 0.0 },
            "a creature must not turn and fight while it can still see what routed it"
        );
    }

    /// A grenade can arrive from somewhere the creature never looked. The
    /// recovery clock has to be carried rather than read off how long the
    /// target has been out of sight, or a rout with nothing to run from is
    /// over on the frame it began.
    #[test]
    fn a_rout_holds_even_when_nothing_was_ever_perceived() {
        let brain = brain_in(Behaviour::Fleeing { calm: 0.0 });
        let blind = Perception::ground_creature(20.0);

        assert_eq!(
            next_behaviour(&brain, &blind, origin(), wounded_to(0.1), 0.016),
            Behaviour::Fleeing { calm: 0.016 },
            "being hurt by something unseen must still send a creature running"
        );
    }

    #[test]
    fn a_rout_ends_once_the_creature_has_shaken_its_pursuer() {
        let brain = brain_in(Behaviour::Fleeing {
            calm: ROUT_RECOVERY - 0.01,
        });
        let long_lost = lost_perception_of(Point3::new(0.0, 0.0, 30.0), 30.0, 2.0, false);

        assert_eq!(
            next_behaviour(&brain, &long_lost, origin(), wounded_to(0.1), 0.016),
            Behaviour::Idle,
            "a creature that got away must calm down, hurt or not"
        );
    }

    #[test]
    fn calming_down_does_not_send_a_hurt_creature_straight_back_into_flight() {
        // Still under the flee threshold, but nothing has hurt it since.
        let brain = Brain {
            behaviour: Behaviour::Idle,
            previous_damage_taken: 50.0,
            ..Brain::hunter(2.0)
        };
        let seen = perception_of(Point3::new(0.0, 0.0, 10.0), 10.0, true);
        let standing_wound = Wounds {
            fraction: 0.1,
            taken: 50.0,
        };

        assert!(
            matches!(
                next_behaviour(&brain, &seen, origin(), standing_wound, 0.016),
                Behaviour::Alerted { .. }
            ),
            "being hurt is what breaks a creature off, not still being hurt"
        );
    }

    #[test]
    fn a_fresh_wound_routs_a_creature_that_had_already_calmed_down() {
        let brain = Brain {
            behaviour: Behaviour::Chasing,
            previous_damage_taken: 50.0,
            ..Brain::hunter(2.0)
        };
        let seen = perception_of(Point3::new(0.0, 0.0, 10.0), 10.0, true);
        let fresh_wound = Wounds {
            fraction: 0.05,
            taken: 55.0,
        };

        assert_eq!(
            next_behaviour(&brain, &seen, origin(), fresh_wound, 0.016),
            Behaviour::Fleeing { calm: 0.0 },
            "a second grenade must rout it again"
        );
    }

    #[test]
    fn a_blink_of_lost_sight_does_not_break_a_chase() {
        let brain = brain_in(Behaviour::Chasing);
        let blinked = lost_perception_of(Point3::new(0.0, 0.0, 10.0), 10.0, 0.1, false);

        assert_eq!(
            next_behaviour(&brain, &blinked, origin(), unhurt(), 0.016),
            Behaviour::Chasing,
            "a target crossing behind cover for a frame must not stop the chase"
        );
    }

    #[test]
    fn losing_sight_for_good_still_ends_the_chase() {
        let brain = brain_in(Behaviour::Chasing);
        let gone = lost_perception_of(Point3::new(0.0, 0.0, 10.0), 10.0, SIGHT_LOSS_GRACE, false);

        assert!(
            matches!(
                next_behaviour(&brain, &gone, origin(), unhurt(), 0.016),
                Behaviour::Searching { .. }
            ),
            "the grace period must expire, or a creature chases a target it has lost"
        );
    }

    #[test]
    fn a_creature_that_has_reached_the_last_known_position_holds_still() {
        let brain = brain_in(Behaviour::Searching {
            last_known: Point3::new(0.0, 0.0, 0.5),
        });
        let gone = lost_perception_of(Point3::new(0.0, 0.0, 0.5), 0.5, 2.0, false);

        assert_eq!(
            intent_for(&brain, &gone, origin()).direction,
            Vector3::zeros(),
            "seeking a point it is standing on walks the creature over it and back"
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
            next_behaviour(&brain, &close, origin(), wounded_to(0.01), 0.016),
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
        let brain = brain_in(Behaviour::Fleeing { calm: 0.0 });
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
