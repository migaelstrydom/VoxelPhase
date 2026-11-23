use nalgebra::{Point3, Vector3};
use specs::{
    Component, DenseVecStorage, Join, LendJoin, Read, ReadStorage, System, Write, WriteStorage,
};

use super::perception::Perception;
use crate::character::{CharacterIntent, CharacterState};
use crate::components::RigidBodyComponent;
use crate::damage::Dead;
use crate::systems::PhysicsResource;
use crate::time::Time;

/// A creature that moves by rolling rather than walking.
///
/// Deliberately *not* driven by `CharacterControlSystem`. That system is built
/// around a walk FSM — grounded, coyote time, jump buffering — and it steers by
/// overwriting linear velocity, which would make a sphere slide across the
/// ground without turning. A boulder forced through a walk cycle is the wrong
/// abstraction, and it would look it.
///
/// What *is* reused is the seam that matters: this reads the same
/// [`CharacterIntent`] the keyboard and `BrainSystem` write. The intent layer is
/// agnostic at both ends — any producer, any consumer — so the brain, the
/// perception, the steering and the damage all work here unchanged. Only the
/// last step, turning a direction into motion, is rolling-specific.
///
/// Its drive comes from angular impulse and nothing else. Friction against the
/// terrain converts the spin into travel, which means the creature is subject
/// to the real surface: it struggles out of a grenade crater and skids on
/// the way. It steers by its spin: each step it turns its spin toward the
/// spin that rolls where it wants to go, so it sheds travel it does not want
/// (sideways, or too fast) as it builds the travel it does, and the ground
/// carries its travel after its spin.
///
/// Close to what it hunts it may lunge ([`Lunge`]): it slows to stalk, then
/// surges along the line to it with all the torque its friction could take,
/// as fast as the ground's grip allows.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Roller {
    /// Angular impulse applied per second while it cruises, N·m. Scales
    /// with mass, so heavier rollers need proportionally more.
    pub torque: f32,
    /// Angular impulse applied per second while it surges, N·m: the most
    /// its own friction could carry into travel, so that the ground's grip
    /// is all that limits it.
    pub surge_torque: f32,
    /// Its top speed rolling where it wants to go, m/s.
    pub speed: f32,
    /// Its radius, m: what couples its spin to its travel.
    pub radius: f32,
    /// How it lunges at what it hunts, if it does.
    pub lunge: Option<Lunge>,
    /// Where it is in its lunge.
    pub lunging: LungePhase,
}

/// A roller's lunge. Within `stalk_range` of what it hunts it slows to
/// `stalk_speed`, which is the warning. Between `nearest` and `range` of
/// it, rolling at it at `closing` m/s or more, it surges along the line to
/// it toward `speed` for `seconds`, keeping to that line, so that stepping
/// aside beats it; then it rests for `rest` before it can lunge again.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lunge {
    /// Centre-to-centre distance to its target, m, within which it stalks.
    pub stalk_range: f32,
    /// Its top speed while it stalks, m/s.
    pub stalk_speed: f32,
    /// The nearest it surges from, m, centre to centre: nearer, it has too
    /// little run-up to strike hard.
    pub nearest: f32,
    /// Centre-to-centre distance to its target, m, within which it surges.
    pub range: f32,
    /// The least speed, m/s, it must be rolling at its target to surge;
    /// negative for one that may drift away from it a little, as one
    /// circling it does, though not roll away (just past it, or backing
    /// off).
    pub closing: f32,
    /// The speed it surges toward, m/s.
    pub speed: f32,
    /// Seconds it surges along its line.
    pub seconds: f32,
    /// Seconds after a surge before it can lunge again.
    pub rest: f32,
}

/// Where a roller is in its lunge.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum LungePhase {
    /// Free to lunge once its target comes within range.
    #[default]
    Ready,
    /// Surging along `line` (level, unit length) through `through`, where
    /// its target stood when it set off, with `left` seconds of it to go.
    Surging {
        left: f32,
        line: Vector3<f32>,
        through: Point3<f32>,
    },
    /// Resting after a lunge, `left` seconds before it can lunge again.
    Resting { left: f32 },
}

/// What a roller hunts, as it bears on this step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quarry {
    /// Where it is from the roller, level, m.
    pub offset: Vector3<f32>,
    /// Whether it is down: a roller does not lunge at a body knocked down.
    pub down: bool,
}

/// How a roller drives on a step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Drive {
    /// The velocity it wants to roll at, level, m/s.
    pub velocity: Vector3<f32>,
    /// The torque it turns its spin toward that velocity with, N·m.
    pub torque: f32,
}

/// Gravity magnitude used to derive the traction limit. Matches
/// `PhysicsConfig::default().gravity`.
const GRAVITY: f32 = 9.81;

/// Moment of inertia of a solid sphere, as a fraction of `m r²`.
const SOLID_SPHERE_INERTIA: f32 = 0.4;

/// How hard a surging roller steers back onto its line, 1/s: the speed
/// toward it it wants for each metre it is off it. Without it a roller that
/// set off rolling across its line drifts off it and misses a target that
/// stood still.
const LINE_HOLD: f32 = 2.0;

impl Roller {
    /// Torques for a roller of the given radius and mass, tuned so
    /// `surface_speed` is its flat-ground top speed and `spin_up_time`
    /// roughly how long it takes to get there from rest.
    ///
    /// The torque follows from rolling without slipping, which couples the
    /// creature's spin to its travel. Driving the body with torque `τ`:
    ///
    /// ```text
    ///   linear:   f = m a                 (f = friction at the contact patch)
    ///   angular:  τ − f r = I α,  α = a/r,  I = 0.4 m r²
    ///   ⇒         τ = 1.4 m r a
    /// ```
    ///
    /// The `1.4` is the part worth keeping in mind: torque has to accelerate
    /// the creature's linear mass as well as spin it up. Sizing torque from the
    /// moment of inertia alone — as if the sphere were spinning in free space —
    /// under-drives it by that factor and produces a creature that visibly
    /// labours to get going.
    ///
    /// `friction` must match the collider's, since it sets the traction limit
    /// below.
    pub fn new(
        radius: f32,
        mass: f32,
        surface_speed: f32,
        spin_up_time: f32,
        friction: f32,
    ) -> Self {
        let radius = radius.max(1e-3);

        // Torque beyond what friction can transmit spins the creature in place
        // instead of moving it faster, so an over-eager `spin_up_time` would
        // trade acceleration for a comic wheelspin. Clamping means the number
        // in the level file means what it says right up to the point where the
        // ground stops cooperating.
        let requested_accel = surface_speed / spin_up_time.max(1e-3);
        let traction_limit = friction * GRAVITY;
        let rolling_torque = |accel: f32| (1.0 + SOLID_SPHERE_INERTIA) * mass * radius * accel;

        Self {
            torque: rolling_torque(requested_accel.min(traction_limit)),
            surge_torque: rolling_torque(traction_limit),
            speed: surface_speed,
            radius,
            lunge: None,
            lunging: LungePhase::Ready,
        }
    }

    /// Lunge at what it hunts as `lunge` says.
    pub fn with_lunge(mut self, lunge: Lunge) -> Self {
        self.lunge = Some(lunge);
        self
    }

    /// Carry its lunge `dt` seconds on, at `position` and moving at
    /// `velocity`, wanting to head along `heading` (level, unit length;
    /// `None` to stand), with `quarry` if it sees one: how it drives on
    /// this step. `None` leaves it to roll as it will.
    pub fn drive(
        &mut self,
        heading: Option<Vector3<f32>>,
        quarry: Option<Quarry>,
        position: Point3<f32>,
        velocity: Vector3<f32>,
        dt: f32,
    ) -> Option<Drive> {
        let torque = self.torque;
        let cruise = |speed: f32| {
            heading.map(|heading| Drive {
                velocity: heading * speed,
                torque,
            })
        };
        let Some(lunge) = self.lunge else {
            return cruise(self.speed);
        };
        let surge_torque = self.surge_torque;
        let surge = |line: Vector3<f32>, through: Point3<f32>| {
            let off = position - through;
            let off = Vector3::new(off.x, 0.0, off.z);
            let across = off - line * off.dot(&line);
            Drive {
                velocity: line * lunge.speed - across * LINE_HOLD,
                torque: surge_torque,
            }
        };
        let stalking = quarry.is_some_and(|quarry| quarry.offset.norm() <= lunge.stalk_range);
        let speed = if stalking {
            lunge.stalk_speed.min(self.speed)
        } else {
            self.speed
        };
        let (phase, drive) = match self.lunging {
            LungePhase::Surging {
                left,
                line,
                through,
            } if left > dt => (
                LungePhase::Surging {
                    left: left - dt,
                    line,
                    through,
                },
                Some(surge(line, through)),
            ),
            LungePhase::Surging { .. } => (LungePhase::Resting { left: lunge.rest }, cruise(speed)),
            LungePhase::Resting { left } if left > dt => {
                (LungePhase::Resting { left: left - dt }, cruise(speed))
            }
            _ => match quarry.zip(lunge.line(quarry, velocity)) {
                Some((quarry, line)) => (
                    LungePhase::Surging {
                        left: lunge.seconds,
                        line,
                        through: position + quarry.offset,
                    },
                    Some(surge(line, position + quarry.offset)),
                ),
                None => (LungePhase::Ready, cruise(speed)),
            },
        };
        self.lunging = phase;
        drive
    }

    /// The angular impulse, N·m·s, that turns a roller of `mass` spinning
    /// at `spin` toward rolling at `drive.velocity`, over `dt`: about the
    /// axis that rolls toward the difference, at most `drive.torque · dt`,
    /// and no more than closes it. Spin about the vertical rolls nowhere
    /// and is left alone.
    ///
    /// Steering the spin rather than the travel cancels travel it does not
    /// want as surely as it builds the travel it does, so a roller turning
    /// after its target brakes its sideways travel instead of circling it;
    /// and a spin already rolling as wanted asks for nothing more, however
    /// far the travel lags it on poor ground.
    pub fn spin_impulse(
        &self,
        drive: Drive,
        spin: Vector3<f32>,
        mass: f32,
        dt: f32,
    ) -> Vector3<f32> {
        let wanted = Vector3::y().cross(&drive.velocity) / self.radius;
        let short = wanted - (spin - Vector3::y() * spin.y);
        let inertia = SOLID_SPHERE_INERTIA * mass * self.radius * self.radius;
        short.try_normalize(1e-6).map_or(Vector3::zeros(), |axis| {
            axis * (short.norm() * inertia).min(drive.torque * dt)
        })
    }
}

impl Lunge {
    /// The line (level, unit length) a roller moving at `velocity` surges
    /// along at `quarry`: toward it, if it is up, between `nearest` and
    /// `range` of it, and the roller rolling at it at `closing` or more.
    fn line(&self, quarry: Option<Quarry>, velocity: Vector3<f32>) -> Option<Vector3<f32>> {
        let within = |distance: f32| (self.nearest..=self.range).contains(&distance);
        let quarry = quarry.filter(|quarry| !quarry.down && within(quarry.offset.norm()))?;
        let aim = quarry.offset.try_normalize(1e-4)?;
        (velocity.dot(&aim) >= self.closing).then_some(aim)
    }
}

/// Drives every roller's spin toward rolling where it wants to go.
pub struct RollerLocomotionSystem;

impl<'a> System<'a> for RollerLocomotionSystem {
    type SystemData = (
        Read<'a, Time>,
        Write<'a, PhysicsResource>,
        WriteStorage<'a, Roller>,
        ReadStorage<'a, CharacterIntent>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Perception>,
        ReadStorage<'a, CharacterState>,
        ReadStorage<'a, Dead>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, mut physics_res, mut rollers, intents, rigid_bodies, perceptions, states, deads) =
            data;
        let dt = time.delta_seconds();
        let level = |v: Vector3<f32>| Vector3::new(v.x, 0.0, v.z);

        for (roller, intent, rb, perception, _) in (
            &mut rollers,
            &intents,
            &rigid_bodies,
            (&perceptions).maybe(),
            !&deads,
        )
            .join()
        {
            let Some(body) = physics_res.world.body(rb.0) else {
                continue;
            };
            let (position, velocity, spin, mass) = (
                body.position(),
                body.linear_velocity(),
                body.angular_velocity(),
                body.mass(),
            );
            let quarry = perception
                .and_then(|perception| perception.target)
                .filter(|target| target.visible)
                .map(|target| Quarry {
                    offset: level(target.position - position),
                    down: states
                        .get(target.entity)
                        .is_some_and(|state| state.locomotion.is_down()),
                });
            let heading = level(intent.direction).try_normalize(1e-4);
            let Some(drive) = roller.drive(heading, quarry, position, velocity, dt) else {
                continue;
            };
            let impulse = roller.spin_impulse(drive, spin, mass, dt);
            // Goes through the world rather than the body so the creature
            // wakes itself. A roller spawns asleep — it is placed at rest —
            // and would otherwise stay that way, spinning up a velocity the
            // integrator ignores, until something bumped it awake.
            if impulse != Vector3::zeros() {
                physics_res.world.apply_angular_impulse(rb.0, impulse);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RADIUS: f32 = 0.6;
    const MASS: f32 = 2171.0;

    #[test]
    fn a_bigger_roller_needs_more_torque_for_the_same_speed() {
        let small = Roller::new(0.5, 100.0, 6.0, 0.7, 1.2);
        let large = Roller::new(1.5, 900.0, 6.0, 0.7, 1.2);
        assert!(
            large.torque > small.torque,
            "torque must scale with mass and radius or heavy rollers won't move"
        );
    }

    /// Flat-ground linear acceleration implied by a drive torque, inverting
    /// the rolling-without-slipping relation in `Roller::new`.
    fn implied_accel(torque: f32) -> f32 {
        torque / ((1.0 + SOLID_SPHERE_INERTIA) * MASS * RADIUS)
    }

    #[test]
    fn torque_delivers_the_requested_spin_up_time() {
        // The bug this pins: sizing torque from the moment of inertia alone
        // ignores the linear mass the contact patch must also accelerate, and
        // under-drives the creature by 3.5x.
        let (speed, spin_up) = (4.5, 0.7);
        let roller = Roller::new(RADIUS, MASS, speed, spin_up, 1.2);
        let time_to_speed = speed / implied_accel(roller.torque);
        assert!(
            (time_to_speed - spin_up).abs() < 0.01,
            "asked for {spin_up}s to reach {speed} m/s, got {time_to_speed}s"
        );
    }

    #[test]
    fn a_shorter_spin_up_time_accelerates_harder() {
        let brisk = Roller::new(RADIUS, MASS, 4.5, 0.4, 1.2);
        let lazy = Roller::new(RADIUS, MASS, 4.5, 1.4, 1.2);
        assert!(brisk.torque > lazy.torque);
        assert_eq!(
            brisk.speed, lazy.speed,
            "spin_up_time must not change top speed — the two dials are separate"
        );
    }

    /// Neither its cruise nor its surge asks the ground for more than its
    /// own friction could transmit, and a surge asks for all of it.
    #[test]
    fn acceleration_is_capped_by_what_friction_can_transmit() {
        let friction = 1.2;
        // An absurdly short spin-up: the ground cannot deliver it.
        let roller = Roller::new(RADIUS, MASS, 4.5, 0.001, friction);
        let limit = friction * GRAVITY;
        assert!(implied_accel(roller.torque) <= limit * 1.001);
        assert!((implied_accel(roller.surge_torque) - limit).abs() < 1e-3);
    }

    const DT: f32 = 0.125;

    fn roller() -> Roller {
        Roller::new(RADIUS, MASS, 4.5, 0.7, 1.2)
    }

    /// The spin that rolls at `velocity` without slipping.
    fn rolling_at(velocity: Vector3<f32>) -> Vector3<f32> {
        Vector3::y().cross(&velocity) / RADIUS
    }

    /// From rest, a roller spins up about the axis that rolls it where it
    /// wants to go, as hard as its torque allows.
    #[test]
    fn a_roller_spins_up_to_roll_where_it_wants_to_go() {
        let roller = roller();
        let drive = Drive {
            velocity: Vector3::z() * 4.5,
            torque: roller.torque,
        };
        let impulse = roller.spin_impulse(drive, Vector3::zeros(), MASS, DT);
        assert!(
            (impulse.normalize() - Vector3::x()).norm() < 1e-5,
            "{impulse:?}"
        );
        assert!((impulse.norm() - roller.torque * DT).abs() < 1e-2);
    }

    /// Rolling across where it wants to go, a roller turns its spin against
    /// its travel across as well as toward the travel it wants: it brakes
    /// out of the turn rather than circling.
    #[test]
    fn a_roller_turning_brakes_its_travel_across() {
        let roller = roller();
        let across = rolling_at(Vector3::z() * 4.5);
        let drive = Drive {
            velocity: Vector3::x() * 4.5,
            torque: roller.torque,
        };
        let impulse = roller.spin_impulse(drive, across, MASS, DT);
        assert!(
            impulse.dot(&across) < 0.0,
            "against the travel across: {impulse:?}"
        );
        assert!(
            impulse.dot(&rolling_at(Vector3::x())) > 0.0,
            "toward the travel wanted: {impulse:?}"
        );
    }

    /// A spin close to the one wanted is closed exactly, not overshot; one
    /// already rolling as wanted asks for nothing, whatever it spins about
    /// the vertical.
    #[test]
    fn a_roller_closes_its_spin_without_overshooting() {
        let roller = roller();
        let drive = Drive {
            velocity: Vector3::z() * 4.5,
            torque: roller.torque,
        };
        let wanted = rolling_at(drive.velocity);
        let nearly = wanted * 0.999;
        let inertia = SOLID_SPHERE_INERTIA * MASS * RADIUS * RADIUS;
        let impulse = roller.spin_impulse(drive, nearly, MASS, DT);
        assert!(
            (impulse - (wanted - nearly) * inertia).norm() < 1e-2,
            "{impulse:?}"
        );
        let twisting = wanted + Vector3::y() * 3.0;
        assert_eq!(
            roller.spin_impulse(drive, twisting, MASS, DT),
            Vector3::zeros()
        );
    }

    const LUNGE: Lunge = Lunge {
        stalk_range: 8.0,
        stalk_speed: 1.5,
        nearest: 2.0,
        range: 5.0,
        closing: 1.0,
        speed: 6.0,
        seconds: 0.5,
        rest: 2.0,
    };

    fn lunging_roller() -> Roller {
        roller().with_lunge(LUNGE)
    }

    fn quarry(offset: Vector3<f32>) -> Option<Quarry> {
        Some(Quarry {
            offset,
            down: false,
        })
    }

    /// Within stalking range of what it hunts, a roller wants no more than
    /// its stalking speed; further off, its own.
    #[test]
    fn a_roller_slows_to_stalk_what_it_hunts() {
        let speed_with = |offset: Vector3<f32>| {
            let heading = Some(Vector3::x());
            let drive = lunging_roller().drive(
                heading,
                quarry(offset),
                Point3::origin(),
                Vector3::zeros(),
                DT,
            );
            drive.unwrap().velocity.norm()
        };
        assert_eq!(speed_with(Vector3::x() * 7.0), LUNGE.stalk_speed);
        assert_eq!(speed_with(Vector3::x() * 9.0), 4.5);
    }

    /// Rolling at what it hunts and within range, a roller surges along the
    /// line to it with its surge torque. Not at one that is down, nor out of
    /// range, nor too near, nor at one it is not rolling at (one just past).
    #[test]
    fn a_roller_surges_at_a_quarry_it_is_rolling_at_within_range() {
        let ahead = Vector3::new(4.0, 0.0, 0.0);
        let heading = Some(Vector3::new(0.6, 0.0, 0.8));
        let drive = |quarry: Option<Quarry>, velocity: Vector3<f32>| {
            lunging_roller()
                .drive(heading, quarry, Point3::origin(), velocity, DT)
                .unwrap()
        };
        let surge = drive(quarry(ahead), Vector3::new(1.5, 0.0, 0.5));
        assert_eq!(surge.velocity, Vector3::x() * LUNGE.speed);
        assert_eq!(surge.torque, roller().surge_torque);
        let cruising = |quarry, velocity| drive(quarry, velocity).torque == roller().torque;
        let down = Some(Quarry {
            offset: ahead,
            down: true,
        });
        assert!(cruising(down, Vector3::x() * 1.5), "down");
        assert!(
            cruising(quarry(ahead * 1.5), Vector3::x() * 1.5),
            "out of range"
        );
        assert!(
            cruising(quarry(ahead * 0.25), Vector3::x() * 1.5),
            "too near"
        );
        assert!(cruising(quarry(ahead), Vector3::x() * 0.5), "not closing");
        assert!(cruising(quarry(ahead), -Vector3::x() * 1.5), "rolled past");
        assert!(cruising(None, Vector3::x() * 1.5), "nothing seen");
    }

    /// A surge keeps to its line for its time, wherever the roller would
    /// head; then it rests, cruising, and can lunge again once rested.
    #[test]
    fn a_surge_keeps_to_its_line_then_rests() {
        let (ahead, closing) = (Vector3::x() * 4.0, Vector3::x() * 1.5);
        let mut roller = lunging_roller();
        let surging: Vec<bool> = (0..22)
            .map(|step| {
                let heading = if step == 0 {
                    Vector3::x()
                } else {
                    Vector3::z()
                };
                let drive = roller
                    .drive(Some(heading), quarry(ahead), Point3::origin(), closing, DT)
                    .unwrap();
                let surging = drive.torque == roller.surge_torque;
                if surging {
                    assert_eq!(drive.velocity, Vector3::x() * LUNGE.speed, "step {step}");
                }
                surging
            })
            .collect();
        let expected: Vec<bool> = (0..22).map(|step| !(4..20).contains(&step)).collect();
        assert_eq!(surging, expected);
    }

    /// A surging roller off its line steers back onto it: across the line
    /// toward it, as well as along it.
    #[test]
    fn a_surge_steers_back_onto_its_line() {
        let mut roller = lunging_roller();
        let heading = Some(Vector3::x());
        let start = roller.drive(
            heading,
            quarry(Vector3::x() * 4.0),
            Point3::origin(),
            Vector3::x() * 1.5,
            DT,
        );
        assert_eq!(start.unwrap().velocity, Vector3::x() * LUNGE.speed);
        let drifted = Point3::new(1.0, 0.0, 0.5);
        let drive = roller
            .drive(heading, None, drifted, Vector3::x() * 3.0, DT)
            .unwrap();
        assert_eq!(
            drive.velocity,
            Vector3::new(LUNGE.speed, 0.0, -0.5 * LINE_HOLD)
        );
    }

    /// A roller without a lunge rolls at its own speed, however close.
    #[test]
    fn a_roller_without_a_lunge_never_lunges() {
        let close = quarry(Vector3::x());
        let drive = roller().drive(
            Some(Vector3::x()),
            close,
            Point3::origin(),
            Vector3::x() * 4.0,
            DT,
        );
        assert_eq!(
            drive,
            Some(Drive {
                velocity: Vector3::x() * 4.5,
                torque: roller().torque,
            })
        );
    }

    /// With nowhere to go and no lunge under way, a roller does not drive.
    #[test]
    fn a_roller_with_no_heading_lets_itself_roll() {
        assert_eq!(
            lunging_roller().drive(None, None, Point3::origin(), Vector3::x(), DT),
            None
        );
    }
}
