//! A creature chasing something that dodges, and what its feet do about
//! the corner.
//!
//! ```text
//!            quarry breaks ──►──►──►──►
//!                          ▲
//!                          │
//!   creature ──►──►──►──►──┘
//! ```
//!
//! The one thing this scenario has that none of the straight-line ones do
//! is a *steered* corner taken at close range: the body keeps its speed,
//! the heading swings through ninety degrees inside a single gait cycle,
//! and the direction a stride is planned along is not the direction it was
//! planned along when the step began. Everything else here — flat ground,
//! one speed — is deliberately plain, so a defect found in a chase is a
//! defect of turning.
//!
//! The pursuit is not invented. It runs the game's own
//! [`steering`](crate::creature::steering) primitives under the game's own
//! behaviour thresholds, and turns the body the way
//! `CharacterControlSystem` does — toward the *intent*, not toward the
//! travel. That distinction is most of the scenario: the intent swings the
//! full ninety degrees the instant the quarry breaks, while the velocity
//! is dragged round behind it, so for several frames the creature is
//! facing one way and moving another.
//!
//! Test-only module (`#[cfg(test)]` in `mod.rs`).

use nalgebra::{Point3, Vector2, Vector3};

use crate::creature::steering;

use super::sim::Input;

/// Proportional gain on yaw, in 1/s. `LocomotionConfig::creature`'s value:
/// a creature turns more deliberately than the player.
const TURN_AGGRESSION: f32 = 6.0;

/// How far past its attack range the quarry must get before the creature
/// resumes the chase. `brain::ATTACK_RELEASE`.
const ATTACK_RELEASE: f32 = 1.25;

/// Half-width of the standoff band, as a fraction of attack range.
/// `brain::STANDOFF_BAND_FRACTION`.
const STANDOFF_BAND_FRACTION: f32 = 0.15;

/// A pursuer, a quarry, and the moment the quarry breaks sideways.
#[derive(Clone, Copy, Debug)]
pub struct Chase {
    /// Top speed of the pursuing creature. A peeper's default.
    pub speed: f32,
    /// Top speed of the thing it is chasing. The player walks at 5.
    pub target_speed: f32,
    /// How far ahead the quarry waits while the pursuer closes.
    pub lead: f32,
    /// The range the pursuer holds station at once it arrives, and
    /// circles. `Brain::attack_range` — a peeper's standoff is 0.88 m.
    pub attack_range: f32,
    /// Seconds of closing before the quarry breaks.
    pub break_time: f32,
    /// Which way the quarry breaks: +1 turns the pursuer one way, −1 the
    /// other. Both are worth running — a gait is not left/right
    /// symmetric, because the phase clock is not.
    pub break_sign: f32,
    /// Which way the creature circles once it is holding the standoff.
    /// It interacts with `break_sign`: a quarry that breaks the way the
    /// creature is already circling asks for a much smaller course change
    /// than one that breaks against it.
    pub clockwise: bool,
    /// How hard the pursuer may change its velocity, in m/s². The game's
    /// character drive has 500, which is a snap: the body is on the new
    /// line within a frame while the facing is still slewing round behind
    /// it. Lower values are what a gentler drive would look like, and are
    /// worth running for contrast.
    pub acceleration: f32,
}

impl Default for Chase {
    fn default() -> Self {
        Self {
            speed: 3.4,
            target_speed: 5.0,
            lead: 5.0,
            attack_range: 0.88,
            break_time: 1.6,
            break_sign: 1.0,
            clockwise: true,
            acceleration: 500.0,
        }
    }
}

impl Chase {
    pub fn breaking(self, sign: f32) -> Self {
        Self {
            break_sign: sign,
            ..self
        }
    }

    /// Run the pursuit forward and return one [`Input`] per frame.
    ///
    /// Precomputed rather than evaluated lazily because the pursuit is
    /// stateful: each frame's steering depends on where the chase has got
    /// to, and `simulate_rig` wants a plain function of frame index.
    pub fn track(&self, frames: usize, fps: f32) -> Vec<Input> {
        let dt = 1.0 / fps;
        let mut position = Vector2::zeros();
        let mut velocity = Vector2::zeros();
        // Facing the quarry it has already spotted and is running at.
        let mut yaw = std::f32::consts::FRAC_PI_2;
        // Circling direction, held rather than rerolled — the brain holds
        // one for `strafe_interval` seconds, and this run is shorter.
        let clockwise = self.clockwise;
        let mut attacking = false;
        let mut out = Vec::with_capacity(frames);

        for f in 0..frames {
            let time = f as f32 * dt;
            let me = Point3::new(position.x, 0.0, position.y);
            let target = self.target_at(time);
            let range = (target - position).magnitude();

            // The brain's own hysteresis: commit to the standoff on
            // arrival, and let go only once the quarry is well clear.
            attacking = if attacking {
                range <= self.attack_range * ATTACK_RELEASE
            } else {
                range <= self.attack_range
            };

            let heading = self.heading(me, target, attacking, clockwise);

            // Bounded acceleration toward the steering demand.
            let change = heading * self.speed - velocity;
            let limit = self.acceleration * dt;
            velocity += if change.magnitude() > limit {
                change.normalize() * limit
            } else {
                change
            };

            // The body turns toward where it *wants* to go, not where it
            // is going: `CharacterControlSystem` aims the yaw drive at the
            // intent direction.
            if heading.magnitude() > 1e-3 {
                let target_yaw = heading.x.atan2(heading.y);
                yaw = wrap_angle(
                    yaw + wrap_angle(target_yaw - yaw) * (TURN_AGGRESSION * dt).min(1.0),
                );
            }

            position += velocity * dt;

            out.push(Input {
                velocity: Vector3::new(velocity.x, 0.0, velocity.y),
                yaw,
                intent: Vector3::new(heading.x, 0.0, heading.y),
            });
        }
        out
    }

    /// The steering demand for one frame, in the game's own terms:
    /// close on the quarry until in range, then hold the standoff and
    /// circle.
    fn heading(
        &self,
        me: Point3<f32>,
        target: Vector2<f32>,
        attacking: bool,
        clockwise: bool,
    ) -> Vector2<f32> {
        let at = Point3::new(target.x, 0.0, target.y);
        let direction = if attacking {
            steering::blend(&[
                (
                    steering::keep_distance(
                        me,
                        at,
                        self.attack_range,
                        self.attack_range * STANDOFF_BAND_FRACTION,
                    ),
                    1.0,
                ),
                (steering::strafe(me, at, clockwise), 1.0),
            ])
        } else {
            steering::seek(me, at)
        };
        Vector2::new(direction.x, direction.z)
    }

    /// Where the quarry is at `time`: holding station ahead while the
    /// pursuer closes, then running perpendicular to the line between the
    /// two.
    ///
    /// It waits first on purpose. A quarry already running is a quarry the
    /// pursuer never catches, and the whole point of the scenario is the
    /// corner taken at close range — the bearing to something a metre away
    /// swings at 5 rad/s when it breaks at 5 m/s, and at ten times that
    /// range it barely swings at all.
    fn target_at(&self, time: f32) -> Vector2<f32> {
        let sideways = (time - self.break_time).max(0.0);
        Vector2::new(self.lead, sideways * self.target_speed * self.break_sign)
    }
}

/// Wrap an angle into `(-PI, PI]`.
fn wrap_angle(angle: f32) -> f32 {
    let tau = std::f32::consts::TAU;
    let mut a = angle % tau;
    if a > std::f32::consts::PI {
        a -= tau;
    } else if a <= -std::f32::consts::PI {
        a += tau;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::animation::foot_placer::capture_point::stance_offset;
    use crate::animation::foot_placer::config::FootPlacerConfig;
    use crate::animation::foot_placer::placer::{facing_from_yaw, FootSide};
    use crate::animation::foot_placer::scenarios;
    use crate::animation::foot_placer::sim::{simulate_rig, takeoffs, Frame, GaitParams, RigDims};

    const FPS: f32 = 30.0;
    const FRAMES: usize = 180;

    /// Every corner worth running: both circling directions, both break
    /// directions, and a break at each point of the gait cycle. The last
    /// is the one that matters — a corner taken just after a foot planted
    /// is a different event from one taken just before it lifts, and only
    /// a sweep covers both.
    fn corners() -> Vec<Chase> {
        let mut out = Vec::new();
        for clockwise in [true, false] {
            for sign in [1.0f32, -1.0] {
                for step in 0..16 {
                    out.push(
                        Chase {
                            break_time: 1.2 + step as f32 * 0.05,
                            clockwise,
                            ..Chase::default()
                        }
                        .breaking(sign),
                    );
                }
            }
        }
        out
    }

    /// Walk one chase on one rig and return the recording.
    fn run(chase: &Chase, dims: RigDims, gait: GaitParams) -> Vec<Frame> {
        let track = chase.track(FRAMES, FPS);
        simulate_rig(
            dims,
            FootPlacerConfig::default(),
            FRAMES,
            |_| 1.0 / FPS,
            gait,
            |x, z| Some(scenarios::flat(x, z)),
            |f| Input {
                velocity: track[f].velocity,
                yaw: track[f].yaw,
                intent: track[f].intent,
            },
            |_| false,
        )
    }

    /// A peeper: long legs, narrow hips, a long stride. The rig the corner
    /// was reported on.
    fn peeper() -> (RigDims, GaitParams) {
        (RigDims::peeper(), GaitParams::peeper())
    }

    fn player() -> (RigDims, GaitParams) {
        (RigDims::player(), GaitParams::walk())
    }

    fn label(events: &[(FootSide, usize)]) -> String {
        events
            .iter()
            .map(|(side, f)| format!("{}{f} ", if *side == FootSide::Left { "L" } else { "R" }))
            .collect()
    }

    /// The heart of it: a gait that has lost its rhythm is one where the
    /// same foot steps twice running. Nothing about a corner licenses
    /// that — a turn may reposition a foot early, but the foot it
    /// repositions is still the one whose turn it was.
    #[test]
    fn a_corner_never_makes_the_same_foot_step_twice() {
        for (name, (dims, gait)) in [("peeper", peeper()), ("player", player())] {
            for chase in corners() {
                let events = takeoffs(&run(&chase, dims, gait));
                assert!(
                    events.len() >= 6,
                    "{name} barely stepped over {chase:?}: {}",
                    label(&events)
                );
                for pair in events.windows(2) {
                    assert_ne!(
                        pair[0].0,
                        pair[1].0,
                        "{name} stepped twice on the same foot rounding {chase:?}: {}",
                        label(&events)
                    );
                }
            }
        }
    }

    /// A corner may cost one early step — the turn release exists to
    /// reposition a foot the body has rotated away from — but it must cost
    /// exactly that. Two feet released within a stagger of each other is
    /// the two-footed hop, and a schedule that never recovers is the
    /// dragging foot.
    #[test]
    fn a_corner_costs_one_early_step_and_then_the_rhythm_returns() {
        let (dims, gait) = peeper();
        for chase in corners() {
            let recording = run(&chase, dims, gait);
            let events = takeoffs(&recording);
            let corner = (chase.break_time * FPS) as usize;

            let mut early = 0;
            for pair in events.windows(2) {
                let gap = (pair[1].1 - pair[0].1) as f32;
                let timing = recording[pair[1].1].timing.expect("timing is published");
                let speed = planar_speed(&recording[pair[1].1]).max(0.1);
                // The schedule's own half-cycle, in frames: what this gap
                // would be if the clock alone were pacing the gait.
                let scheduled = 0.5 * timing.cycle_distance / speed * FPS;
                let ratio = gap / scheduled;

                assert!(
                    gap >= 3.0,
                    "two takeoffs {gap} frames apart rounding {chase:?}: {}",
                    label(&events)
                );
                if !(0.6..1.7).contains(&ratio) {
                    early += 1;
                    // Whatever the corner costs, it is paid at the corner.
                    // A cadence still off a second later is a gait that
                    // did not recover.
                    assert!(
                        pair[1].1 < corner + FPS as usize,
                        "cadence still {ratio:.2}x the schedule at frame {} rounding {chase:?}: {}",
                        pair[1].1,
                        label(&events)
                    );
                }
            }
            assert!(
                early <= 1,
                "{early} off-beat steps rounding {chase:?}: {}",
                label(&events)
            );
        }
    }

    /// The corner must not be taken by stretching a planted leg past what
    /// it has. A leg at full extension has no bend left, and the foot on
    /// the end of it is being dragged rather than stood on — which is what
    /// sticky feet look like from the inside.
    #[test]
    fn a_corner_never_stretches_a_planted_leg_past_its_budget() {
        let cfg = FootPlacerConfig::default();
        for (name, (dims, gait)) in [("peeper", peeper()), ("player", player())] {
            for chase in corners() {
                for frame in run(&chase, dims, gait) {
                    let facing = facing_from_yaw(frame.yaw);
                    for foot in [&frame.left, &frame.right] {
                        if !foot.is_planted() {
                            continue;
                        }
                        let offset = stance_offset(facing, dims.hip_width, foot.side.side_sign());
                        let hip = Point3::new(
                            frame.pelvis.x + offset.x,
                            frame.pelvis.y,
                            frame.pelvis.z + offset.y,
                        );
                        let extension = (foot.planted_position - hip).magnitude() / dims.leg_length;
                        assert!(
                            extension <= cfg.max_leg_stretch_ratio + 1e-3,
                            "{name} stood on a leg stretched to {extension:.3} of its length \
                             rounding {chase:?}",
                        );
                    }
                }
            }
        }
    }

    /// The corner's steady-state cousin: a creature holding its standoff
    /// circles the target, which is a turn that never ends. At a peeper's
    /// standoff and speed that is about 4 rad/s held indefinitely — a far
    /// harsher test of the turn release than any single corner, and the
    /// gait has to keep its rhythm through all of it.
    #[test]
    fn an_endless_orbit_keeps_its_rhythm() {
        let (dims, gait) = peeper();
        for clockwise in [true, false] {
            let orbit = Chase {
                // A quarry that never breaks: the creature arrives at its
                // standoff and circles from there on.
                break_time: f32::MAX,
                clockwise,
                ..Chase::default()
            };
            let events = takeoffs(&run(&orbit, dims, gait));
            assert!(events.len() >= 8, "barely stepped: {}", label(&events));
            for pair in events.windows(2) {
                assert_ne!(
                    pair[0].0,
                    pair[1].0,
                    "same foot twice while circling clockwise={clockwise}: {}",
                    label(&events)
                );
            }
            // Once the orbit settles, every gap is the same gap.
            let settled: Vec<usize> = events
                .windows(2)
                .skip(2)
                .map(|pair| pair[1].1 - pair[0].1)
                .collect();
            let shortest = *settled.iter().min().expect("several steps");
            let longest = *settled.iter().max().expect("several steps");
            assert!(
                longest - shortest <= 2,
                "circling cadence wandered between {shortest} and {longest} frames: {}",
                label(&events)
            );
        }
    }

    fn planar_speed(frame: &Frame) -> f32 {
        Vector2::new(frame.velocity.x, frame.velocity.z).magnitude()
    }
}
