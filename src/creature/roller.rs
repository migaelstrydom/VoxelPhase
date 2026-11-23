use nalgebra::Vector3;
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
/// to the real surface: it accelerates down into a grenade crater, struggles
/// out of it, and skids on the way. That behaviour is free, and it is the
/// reason this creature is the first one.
///
/// Close to what it hunts it may lunge ([`Lunge`]): its spikes bite into the
/// ground and it bursts forward, faster than any spin the ground's friction
/// could give it, so that it strikes hard enough to knock a body down. The
/// lunge is the one motion that is not spin: friction alone gives a roller
/// on grass about 4 m/s², which no reader would call sudden.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Roller {
    /// Angular impulse applied per second while the creature wants to move.
    /// Scales with mass, so heavier rollers need proportionally more.
    pub torque: f32,
    /// Spin rate (rad/s) past which no further torque is applied. This is the
    /// creature's top speed, expressed the only way a rolling body can express
    /// one — `LocomotionConfig::walk_speed` has no meaning without a gait.
    pub max_spin: f32,
    /// Its radius, m: what turns a lunge's burst into the spin that rolls
    /// with it.
    pub radius: f32,
    /// How it lunges at what it hunts, if it does.
    pub lunge: Option<Lunge>,
    /// Where it is in its lunge.
    pub lunging: LungePhase,
}

/// A roller's lunge: rolling at what it hunts at `closing` m/s or more,
/// once within `range` of it, it bursts to `speed` along the line to it and
/// keeps to that line for `seconds`, unsteered, so that stepping aside
/// beats it; then it rests for `rest` before it can lunge again.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lunge {
    /// Centre-to-centre distance to its target, m, within which it lunges.
    pub range: f32,
    /// The least speed, m/s, it must be rolling at its target to lunge: a
    /// roller that has just rolled past does not burst back at it.
    pub closing: f32,
    /// The speed it bursts to, m/s.
    pub speed: f32,
    /// Seconds it keeps to its line.
    pub seconds: f32,
    /// Seconds after a lunge before it can lunge again.
    pub rest: f32,
}

/// Where a roller is in its lunge.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum LungePhase {
    /// Free to lunge once its target comes within range.
    #[default]
    Ready,
    /// Lunging, with `left` seconds of it to go.
    Lunging { left: f32 },
    /// Resting after a lunge, `left` seconds before it can lunge again.
    Resting { left: f32 },
}

/// What a roller does with this step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Roll {
    /// Drive toward its intent, as it always does.
    Drive,
    /// Lunge: its velocity changes by `burst`, m/s, at once, with the spin
    /// that rolls with it.
    Burst { burst: Vector3<f32> },
    /// Keep to the line of a lunge: no drive, so nothing steers it.
    Coast,
}

/// Gravity magnitude used to derive the traction limit. Matches
/// `PhysicsConfig::default().gravity`.
const GRAVITY: f32 = 9.81;

/// Moment of inertia of a solid sphere, as a fraction of `m r²`.
const SOLID_SPHERE_INERTIA: f32 = 0.4;

impl Roller {
    /// Torque and spin cap for a roller of the given radius and mass, tuned so
    /// `surface_speed` is roughly its flat-ground top speed and `spin_up_time`
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
        let max_spin = surface_speed / radius;

        // Torque beyond what friction can transmit spins the creature in place
        // instead of moving it faster, so an over-eager `spin_up_time` would
        // trade acceleration for a comic wheelspin. Clamping means the number
        // in the level file means what it says right up to the point where the
        // ground stops cooperating.
        let requested_accel = surface_speed / spin_up_time.max(1e-3);
        let traction_limit = friction * GRAVITY;
        let accel = requested_accel.min(traction_limit);

        Self {
            torque: (1.0 + SOLID_SPHERE_INERTIA) * mass * radius * accel,
            max_spin,
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

    /// Carry its lunge `dt` seconds on, moving at `velocity`, with its
    /// target at `target` from it (level) if it can see one: what it does
    /// with this step.
    pub fn roll(&mut self, target: Option<Vector3<f32>>, velocity: Vector3<f32>, dt: f32) -> Roll {
        let Some(lunge) = self.lunge else {
            return Roll::Drive;
        };
        let (phase, roll) = match self.lunging {
            LungePhase::Lunging { left } if left > dt => {
                (LungePhase::Lunging { left: left - dt }, Roll::Coast)
            }
            LungePhase::Lunging { .. } => (LungePhase::Resting { left: lunge.rest }, Roll::Drive),
            LungePhase::Resting { left } if left > dt => {
                (LungePhase::Resting { left: left - dt }, Roll::Drive)
            }
            _ => match lunge.burst(target, velocity) {
                Some(burst) => (
                    LungePhase::Lunging {
                        left: lunge.seconds,
                    },
                    Roll::Burst { burst },
                ),
                None => (LungePhase::Ready, Roll::Drive),
            },
        };
        self.lunging = phase;
        roll
    }
}

impl Lunge {
    /// The change of velocity, m/s, a roller moving at `velocity` bursts
    /// with at a target `target` from it (level): up to `speed` along the
    /// line to it. `None` out of range, or not rolling at it.
    fn burst(&self, target: Option<Vector3<f32>>, velocity: Vector3<f32>) -> Option<Vector3<f32>> {
        let target = target.filter(|target| target.norm() <= self.range)?;
        let aim = target.try_normalize(1e-4)?;
        let closing = velocity.dot(&aim);
        (closing >= self.closing).then(|| aim * (self.speed - closing).max(0.0))
    }
}

/// Applies rolling motion from intent.
///
/// The impulse axis is the intent direction rotated a quarter turn about world
/// up: to roll *toward* +Z a sphere spins about +X. Steering is a consequence
/// of where the torque points, so there is no separate turn control — a roller
/// arcs into its turns, which is exactly what a boulder should do.
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
            let (position, velocity, mass) = (body.position(), body.linear_velocity(), body.mass());
            // It does not lunge at a body already knocked down.
            let target = perception
                .and_then(|perception| perception.target)
                .filter(|target| {
                    target.visible
                        && !states
                            .get(target.entity)
                            .is_some_and(|state| state.locomotion.is_down())
                })
                .map(|target| {
                    let to = target.position - position;
                    Vector3::new(to.x, 0.0, to.z)
                });
            match roller.roll(target, velocity, dt) {
                Roll::Drive => {}
                Roll::Coast => continue,
                Roll::Burst { burst } => {
                    // The spin that rolls with the burst, so the spikes do not
                    // skid: Δω = Δv / r about the axis left of the burst, on a
                    // solid sphere's inertia.
                    let spin = Vector3::y().cross(&burst) / roller.radius;
                    let inertia = SOLID_SPHERE_INERTIA * mass * roller.radius * roller.radius;
                    physics_res.world.apply_impulse(rb.0, burst * mass);
                    physics_res
                        .world
                        .apply_angular_impulse(rb.0, spin * inertia);
                    continue;
                }
            }
            let Some(direction) = intent.direction.try_normalize(1e-4) else {
                continue;
            };

            // Rolling toward `direction` means spinning about the axis to its
            // left: up × direction.
            let axis = Vector3::y().cross(&direction);
            let Some(body) = physics_res.world.body(rb.0) else {
                continue;
            };

            // Only the component of spin that drives this direction counts
            // toward the cap. A roller already spinning the other way is
            // under its cap on this axis and gets full torque to reverse,
            // which is what stops it drifting past a target it overshot.
            let spin_along_axis = body.angular_velocity().dot(&axis);
            if spin_along_axis >= roller.max_spin {
                continue;
            }

            // Goes through the world rather than the body so the creature wakes
            // itself. A roller spawns asleep — it is placed at rest — and would
            // otherwise stay that way, spinning up a velocity the integrator
            // ignores, until something bumped it awake.
            physics_res
                .world
                .apply_angular_impulse(rb.0, axis * roller.torque * dt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bigger_roller_needs_more_torque_for_the_same_speed() {
        let small = Roller::new(0.5, 100.0, 6.0, 0.7, 1.2);
        let large = Roller::new(1.5, 900.0, 6.0, 0.7, 1.2);
        assert!(
            large.torque > small.torque,
            "torque must scale with mass and radius or heavy rollers won't move"
        );
    }

    #[test]
    fn spin_cap_falls_as_radius_grows_for_the_same_surface_speed() {
        let small = Roller::new(0.5, 100.0, 6.0, 0.7, 1.2);
        let large = Roller::new(1.5, 100.0, 6.0, 0.7, 1.2);
        assert!(
            large.max_spin < small.max_spin,
            "a larger wheel covers the same ground with less spin"
        );
        assert!((small.max_spin - 12.0).abs() < 1e-4, "6 m/s at r=0.5");
    }

    /// Flat-ground linear acceleration implied by a roller's drive torque,
    /// inverting the rolling-without-slipping relation in `Roller::new`.
    fn implied_accel(roller: &Roller, radius: f32, mass: f32) -> f32 {
        roller.torque / ((1.0 + SOLID_SPHERE_INERTIA) * mass * radius)
    }

    #[test]
    fn torque_delivers_the_requested_spin_up_time() {
        // The bug this pins: sizing torque from the moment of inertia alone
        // ignores the linear mass the contact patch must also accelerate, and
        // under-drives the creature by 3.5x.
        let (radius, mass, speed, spin_up) = (0.6, 2171.0, 4.5, 0.7);
        let roller = Roller::new(radius, mass, speed, spin_up, 1.2);

        let accel = implied_accel(&roller, radius, mass);
        let time_to_speed = speed / accel;

        assert!(
            (time_to_speed - spin_up).abs() < 0.01,
            "asked for {spin_up}s to reach {speed} m/s, got {time_to_speed}s"
        );
    }

    #[test]
    fn a_shorter_spin_up_time_accelerates_harder() {
        let (radius, mass) = (0.6, 2171.0);
        let brisk = Roller::new(radius, mass, 4.5, 0.4, 1.2);
        let lazy = Roller::new(radius, mass, 4.5, 1.4, 1.2);
        assert!(brisk.torque > lazy.torque);
        assert!(
            (brisk.max_spin - lazy.max_spin).abs() < 1e-4,
            "spin_up_time must not change top speed — the two dials are separate"
        );
    }

    #[test]
    fn acceleration_is_capped_by_what_friction_can_transmit() {
        let (radius, mass, friction) = (0.6, 2171.0, 1.2);
        // An absurdly short spin-up: the ground cannot deliver it.
        let roller = Roller::new(radius, mass, 4.5, 0.001, friction);

        let accel = implied_accel(&roller, radius, mass);
        assert!(
            accel <= friction * GRAVITY * 1.001,
            "torque beyond the traction limit spins the creature in place"
        );
    }

    const LUNGE: Lunge = Lunge {
        range: 3.0,
        closing: 1.0,
        speed: 8.0,
        seconds: 0.5,
        rest: 2.0,
    };

    fn lunging_roller() -> Roller {
        Roller::new(0.6, 2171.0, 4.5, 0.7, 1.2).with_lunge(LUNGE)
    }

    /// Rolling at its target and within range, a roller bursts to its lunge
    /// speed along the line to it; out of range, or not rolling at it (one
    /// just past), it drives as it always does.
    #[test]
    fn a_roller_bursts_at_a_target_it_is_rolling_at_within_range() {
        let ahead = Vector3::new(2.0, 0.0, 0.0);
        let at = |velocity| lunging_roller().roll(Some(ahead), velocity, 0.1);
        assert_eq!(
            at(Vector3::new(3.0, 0.0, 1.0)),
            Roll::Burst {
                burst: Vector3::new(5.0, 0.0, 0.0)
            }
        );
        assert_eq!(at(Vector3::new(0.5, 0.0, 0.0)), Roll::Drive, "not closing");
        assert_eq!(at(Vector3::new(-3.0, 0.0, 0.0)), Roll::Drive, "rolled past");
        let far = lunging_roller().roll(Some(ahead * 2.0), Vector3::x() * 3.0, 0.1);
        assert_eq!(far, Roll::Drive, "out of range");
        let blind = lunging_roller().roll(None, Vector3::x() * 3.0, 0.1);
        assert_eq!(blind, Roll::Drive, "nothing seen");
        let fast = lunging_roller().roll(Some(ahead), Vector3::x() * 9.0, 0.1);
        assert_eq!(
            fast,
            Roll::Burst {
                burst: Vector3::zeros()
            },
            "already faster"
        );
    }

    /// After its burst a roller keeps to its line, unsteered, for the
    /// lunge's time; then it rests, driving as it always does, and can
    /// lunge again once it has rested.
    #[test]
    fn a_lunge_keeps_to_its_line_then_rests() {
        const DT: f32 = 0.125;
        let (ahead, closing) = (Vector3::new(2.0, 0.0, 0.0), Vector3::x() * 3.0);
        let mut roller = lunging_roller();
        assert!(matches!(
            roller.roll(Some(ahead), closing, DT),
            Roll::Burst { .. }
        ));
        let mut steps = |roll| {
            1 + (0..100)
                .take_while(|_| roller.roll(Some(ahead), closing, DT) == roll)
                .count()
        };
        assert_eq!(steps(Roll::Coast), 4, "it keeps to its line for its time");
        assert_eq!(
            steps(Roll::Drive),
            16,
            "then it rests before it can lunge again"
        );
    }

    /// A roller without a lunge rolls as it always does, however close.
    #[test]
    fn a_roller_without_a_lunge_never_lunges() {
        let mut roller = Roller::new(0.6, 2171.0, 4.5, 0.7, 1.2);
        let roll = roller.roll(Some(Vector3::x()), Vector3::x() * 4.0, 0.1);
        assert_eq!(roll, Roll::Drive);
    }

    #[test]
    fn roll_axis_is_perpendicular_to_travel_and_to_up() {
        let direction: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
        let axis = Vector3::y().cross(&direction);
        assert!((axis - Vector3::new(1.0, 0.0, 0.0)).magnitude() < 1e-6);
        assert!(axis.dot(&direction).abs() < 1e-6);
        assert!(axis.dot(&Vector3::y()).abs() < 1e-6);
    }
}
