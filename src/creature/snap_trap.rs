//! A trap that lies open and snaps shut on whoever steps into it.
//!
//! ```text
//!   Perception ──target within `radius`──▶ bite: damage, and a cycle begins
//!
//!   seconds since the bite ──Cycle::moment──▶ Moment
//!     slam    the jaw falls shut, faster as it goes
//!     hold    shut on its catch, which kicks and goes limp; at the end the
//!             jaw parts a crack and the catch is drawn in
//!     creak   the jaw eases open again; then it lies open, waiting
//!
//!   lying open, the jaw rises and falls a little with its breath; with
//!   someone near it narrows and quivers, which is all the warning it gives
//!
//!   Moment ──TrapRig::poses──▶ PosedModel: the jaw on its hinge, the muscle
//!                              stretched between the jaws, the flesh
//!                              breathing in the bed, the catch
//! ```
//!
//! The trap is one model drawn part by part ([`PosedModel`], in the
//! entity's own frame); what each part does is said once by the creature
//! that is a trap ([`TrapPart`]), and nothing here knows it is a mussel.

use std::f32::consts::{PI, TAU};

use nalgebra::{Unit, UnitQuaternion, Vector3};
use specs::{
    Component, DenseVecStorage, Join, LendJoin, Read, ReadStorage, System, Write, WriteStorage,
};

use super::perception::{PerceivedTarget, Perception};
use crate::components::{PosedModel, Position};
use crate::damage::{DamageKind, DamageQueue, Dead};
use crate::model::Transform;
use crate::time::Time;

/// How far above or below the trap its victim may be and still be in it, m.
const DEPTH: f32 = 1.0;

/// Breaths a second of the flesh in a trap.
const BREATH_RATE: f32 = 0.22;

/// How far an open jaw sinks with each breath, as a share of its gape.
const IDLE_GAPE: f32 = 0.07;

/// How near something must be for the trap to grow wary, as a multiple of
/// its bite radius, and how fast it grows wary or calms again, 1/s.
const WARY_RANGE: f32 = 4.0;
const WARY_RATE: f32 = 3.0;
/// How far a wary jaw narrows, as a share of its gape, and how far and how
/// fast it quivers, rad/s.
const WARY_NARROW: f32 = 0.1;
const QUIVER: f32 = 0.035;
const QUIVER_RATE: f32 = 57.0;

/// The shudder as the jaw lands: how far it bounces open, as a share of its
/// gape, how fast that dies away, 1/s, and how fast it rattles, rad/s.
const SHUDDER: f32 = 0.035;
const SHUDDER_DECAY: f32 = 14.0;
const SHUDDER_RATE: f32 = 46.0;

/// Seconds at the end of the hold over which the catch is drawn in, and how
/// far the jaw parts to let it through, as a share of its gape.
const GULP: f32 = 0.7;
const GULP_GAPE: f32 = 0.05;

/// A catch's kicking: kicks a second, and how fast they die away, 1/s.
const KICK_RATE: f32 = 3.2;
const KICK_DECAY: f32 = 2.6;

/// The hinge a trap's upper jaw turns on, in the trap's own frame.
#[derive(Clone, Copy, Debug)]
pub struct Hinge {
    /// A point on the hinge's line.
    pub pivot: Vector3<f32>,
    /// The way the line runs: the jaw opens by turning right-handedly
    /// about it.
    pub axis: Unit<Vector3<f32>>,
    /// How far the jaw stands open when the trap lies open, radians.
    pub gape: f32,
}

impl Hinge {
    /// Where something that rides the jaw is when the jaw stands `open` of
    /// its gape (0 shut, 1 wide): it is modelled where it lies with the jaw
    /// shut.
    pub fn pose(&self, open: f32) -> Transform {
        let turn = UnitQuaternion::from_axis_angle(&self.axis, self.gape * open);
        Transform {
            translation: self.pivot - turn * self.pivot,
            rotation: turn,
            scale: Vector3::repeat(1.0),
        }
    }
}

/// How one part of a trap's model moves.
#[derive(Clone, Copy, Debug)]
pub enum TrapPart {
    /// It does not: the lower jaw, and what is fixed in it.
    Bed,
    /// The upper jaw, turning on the hinge. Modelled where it lies shut.
    Jaw,
    /// Flesh lying in the bed: it swells and sinks as it breathes, and is
    /// pressed down under the jaw when that shuts.
    Flesh {
        /// The point it swells about and is pressed down onto.
        root: Vector3<f32>,
        /// How far a breath swells it along each axis, as a share.
        breath: Vector3<f32>,
        /// How far behind the trap's breath its own runs, radians.
        lag: f32,
        /// How tall it is under the shut jaw, as a share of its height.
        pressed: f32,
    },
    /// A muscle from the bed to the jaw, stretched between them however
    /// wide the jaw stands. Modelled with its root at its origin, running
    /// `length` along +Y.
    Muscle {
        /// Where it is rooted in the bed.
        root: Vector3<f32>,
        /// Where it holds the jaw, with the jaw shut.
        hold: Vector3<f32>,
        /// How long it is modelled, m.
        length: f32,
    },
    /// What the trap has caught: out between its jaws while it is shut,
    /// drawn in before it opens, and not there while it lies open. Modelled
    /// where it lies once it has gone limp.
    Catch {
        /// Where the jaws hold it: what it kicks about.
        held: Vector3<f32>,
        /// How far its first kick turns it about the hinge's axis, radians.
        kick: f32,
        /// How far it is turned about the hinge's axis as it is drawn in,
        /// radians, and how far it is drawn, m.
        lift: f32,
        drawn: Vector3<f32>,
        /// The share of its size it is crushed to by the time it is in.
        shrunk: f32,
    },
}

/// What a trap's catch is doing between its jaws.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Caught {
    /// Seconds since the jaws shut on it.
    pub held_for: f32,
    /// How far it has been drawn in, 0 to 1.
    pub swallowed: f32,
}

/// A moment in a trap's life: all that the poses of its parts hang on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Moment {
    /// How far open the jaw stands, 0 shut to 1 wide.
    pub gape: f32,
    /// Its catch, while it has one between its jaws.
    pub caught: Option<Caught>,
    /// Where it is in its breath, radians.
    pub breath: f32,
}

impl Moment {
    /// Lying open, between breaths.
    pub const OPEN: Moment = Moment {
        gape: 1.0,
        caught: None,
        breath: 0.0,
    };

    /// Shut on a catch that has gone limp.
    pub const SHUT: Moment = Moment {
        gape: 0.0,
        caught: Some(Caught {
            held_for: 60.0,
            swallowed: 0.0,
        }),
        breath: 0.0,
    };
}

/// A trap's model as something that moves: its hinge, and what each part of
/// the model does.
#[derive(Clone, Debug)]
pub struct TrapRig {
    pub hinge: Hinge,
    /// How each part of the model moves, in part order.
    pub parts: Vec<TrapPart>,
}

impl TrapRig {
    /// Each part's pose at `moment`, in part order, in the trap's own
    /// frame.
    pub fn poses(&self, moment: &Moment) -> Vec<Transform> {
        self.parts
            .iter()
            .map(|part| self.pose(part, moment))
            .collect()
    }

    fn pose(&self, part: &TrapPart, moment: &Moment) -> Transform {
        match *part {
            TrapPart::Bed => Transform::default(),
            TrapPart::Jaw => self.hinge.pose(moment.gape),
            TrapPart::Flesh {
                root,
                breath,
                lag,
                pressed,
            } => {
                let mut scale = Vector3::repeat(1.0) + breath * (moment.breath - lag).sin();
                scale.y *= pressed + (1.0 - pressed) * moment.gape;
                Transform {
                    translation: root - root.component_mul(&scale),
                    rotation: UnitQuaternion::identity(),
                    scale,
                }
            }
            TrapPart::Muscle { root, hold, length } => {
                let jaw = self.hinge.pose(moment.gape);
                let span = jaw.translation + jaw.rotation * hold - root;
                let stretch = span.norm() / length;
                // What it gains in length it loses in girth.
                let girth = stretch.max(1e-3).sqrt().recip();
                Transform {
                    translation: root,
                    rotation: UnitQuaternion::rotation_between(&Vector3::y(), &span)
                        .unwrap_or_else(UnitQuaternion::identity),
                    scale: Vector3::new(girth, stretch, girth),
                }
            }
            TrapPart::Catch {
                held,
                kick,
                lift,
                drawn,
                shrunk,
            } => {
                let Some(caught) = moment.caught else {
                    return Transform {
                        translation: held,
                        rotation: UnitQuaternion::identity(),
                        scale: Vector3::zeros(),
                    };
                };
                let kicking = kick
                    * (-KICK_DECAY * caught.held_for).exp()
                    * (TAU * KICK_RATE * caught.held_for).sin();
                let turn = UnitQuaternion::from_axis_angle(
                    &self.hinge.axis,
                    kicking + lift * caught.swallowed,
                );
                let size = 1.0 - (1.0 - shrunk) * caught.swallowed;
                Transform {
                    translation: held - turn * held * size + drawn * caught.swallowed,
                    rotation: turn,
                    scale: Vector3::repeat(size),
                }
            }
        }
    }
}

/// How long each stretch of a bite lasts, seconds.
#[derive(Clone, Copy, Debug)]
pub struct Cycle {
    /// The jaw falling shut.
    pub slam: f32,
    /// From the bite until the jaw starts to open: the slam is part of it.
    pub hold: f32,
    /// The jaw easing open again.
    pub creak: f32,
}

impl Cycle {
    /// Seconds from a bite until the trap lies open again.
    pub fn length(&self) -> f32 {
        self.hold + self.creak
    }

    /// How far open the jaw stands `since` seconds after a bite, 0 to 1,
    /// and what has become of its catch.
    fn jaw(&self, since: f32) -> (f32, Option<Caught>) {
        if since < self.slam {
            let fallen = since / self.slam;
            return (1.0 - fallen * fallen, None);
        }
        if since < self.hold {
            let held_for = since - self.slam;
            let gulp = GULP.min(self.hold - self.slam);
            let swallowed = smoothstep((since - (self.hold - gulp)) / gulp.max(1e-6));
            let shudder =
                SHUDDER * (-SHUDDER_DECAY * held_for).exp() * (SHUDDER_RATE * held_for).sin().abs();
            let parted = GULP_GAPE * (PI * swallowed).sin();
            return (
                shudder + parted,
                Some(Caught {
                    held_for,
                    swallowed,
                }),
            );
        }
        (smoothstep((since - self.hold) / self.creak), None)
    }
}

fn smoothstep(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A trap and where it stands in its cycle.
#[derive(Component, Clone, Debug)]
#[storage(DenseVecStorage)]
pub struct SnapTrap {
    /// Distance from its middle within which it bites, m.
    pub radius: f32,
    /// Hit points a bite takes off.
    pub damage: f32,
    /// How long a bite takes it.
    pub cycle: Cycle,
    /// How its model moves.
    pub rig: TrapRig,
    /// Seconds since it bit, until it lies open again.
    since_bite: Option<f32>,
    /// Seconds it has lain there: the clock its flesh breathes by.
    clock: f32,
    /// How wary it is of something near, 0 to 1.
    wary: f32,
}

impl SnapTrap {
    /// A trap lying open.
    pub fn new(radius: f32, damage: f32, cycle: Cycle, rig: TrapRig) -> Self {
        Self {
            radius,
            damage,
            cycle,
            rig,
            since_bite: None,
            clock: 0.0,
            wary: 0.0,
        }
    }

    /// Whether it lies open, ready to bite.
    pub fn is_open(&self) -> bool {
        self.since_bite.is_none()
    }

    /// Advance by `dt` with something in it or not, and something near it
    /// or not. Returns whether it bit.
    fn tick(&mut self, dt: f32, occupied: bool, near: bool) -> bool {
        self.clock += dt;
        let wary = if near || occupied { 1.0 } else { 0.0 };
        self.wary += (wary - self.wary) * (1.0 - (-WARY_RATE * dt).exp());
        match self.since_bite {
            None if occupied => {
                self.since_bite = Some(0.0);
                true
            }
            None => false,
            Some(since) => {
                let since = since + dt;
                self.since_bite = (since < self.cycle.length()).then_some(since);
                false
            }
        }
    }

    /// The moment it is at.
    pub fn moment(&self) -> Moment {
        let (gape, caught) = match self.since_bite {
            Some(since) => self.cycle.jaw(since),
            None => (1.0, None),
        };
        let breath = self.clock * BREATH_RATE * TAU;
        // With nothing between its jaws they are as open as it holds them,
        // and the slam and the creak run from and to that.
        let gape = match caught {
            Some(_) => gape,
            None => gape * self.held_open(breath),
        };
        Moment {
            gape,
            caught,
            breath,
        }
    }

    /// How far open it holds its jaw lying in wait, as a share of its gape:
    /// sinking a little as it breathes out, narrowed and quivering when wary.
    fn held_open(&self, breath: f32) -> f32 {
        let exhaled = 0.5 - 0.5 * breath.cos();
        let quiver = QUIVER * (self.clock * QUIVER_RATE).sin();
        1.0 - IDLE_GAPE * exhaled - self.wary * (WARY_NARROW + quiver)
    }

    /// Each part of its model's pose now, in part order.
    pub fn poses(&self) -> Vec<Transform> {
        self.rig.poses(&self.moment())
    }
}

/// Springs and resets every trap, and poses its model.
pub struct SnapTrapSystem;

impl<'a> System<'a> for SnapTrapSystem {
    type SystemData = (
        Read<'a, Time>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Perception>,
        WriteStorage<'a, SnapTrap>,
        WriteStorage<'a, PosedModel>,
        Write<'a, DamageQueue>,
        ReadStorage<'a, Dead>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, positions, perceptions, mut traps, mut posed, mut queue, deads) = data;
        for (position, perception, trap, posed, _) in (
            &positions,
            &perceptions,
            &mut traps,
            (&mut posed).maybe(),
            !&deads,
        )
            .join()
        {
            let level =
                |target: &PerceivedTarget| (target.position.y - position.0.y).abs() <= DEPTH;
            let victim = perception
                .target
                .filter(|target| target.visible && target.distance <= trap.radius && level(target));
            let near = perception.target.is_some_and(|target| {
                target.visible && target.distance <= WARY_RANGE * trap.radius && level(&target)
            });
            let bit = trap.tick(time.delta_seconds(), victim.is_some(), near);
            if let Some(victim) = victim.filter(|_| bit) {
                queue.push(victim.entity, trap.damage, DamageKind::Melee);
            }
            if let Some(posed) = posed {
                posed.poses = trap.poses();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;
    use std::sync::Arc;

    use nalgebra::Point3;
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;
    use crate::components::PoseFrame;
    use crate::creature::PerceivedTarget;
    use crate::model::{Model, ModelPart};

    /// A hinge along the back of a trap a metre deep, its jaw opening a
    /// quarter turn upward.
    fn hinge() -> Hinge {
        Hinge {
            pivot: Vector3::new(0.0, 0.1, -0.5),
            axis: -Vector3::x_axis(),
            gape: FRAC_PI_2,
        }
    }

    fn trap() -> SnapTrap {
        SnapTrap::new(
            0.6,
            30.0,
            Cycle {
                slam: 0.1,
                hold: 2.0,
                creak: 1.0,
            },
            TrapRig {
                hinge: hinge(),
                parts: vec![TrapPart::Bed, TrapPart::Jaw],
            },
        )
    }

    #[test]
    fn the_system_bites_whoever_steps_in_and_shuts_the_jaw_of_its_model() {
        let mut world = World::new();
        world.register::<Position>();
        world.register::<Perception>();
        world.register::<SnapTrap>();
        world.register::<PosedModel>();
        world.register::<Dead>();
        world.insert(Time::fixed(0.02));
        world.insert(DamageQueue::default());
        let victim = world.create_entity().build();
        let model = Model::flat(vec![ModelPart::new(Vec::new()), ModelPart::new(Vec::new())]);
        let trap = world
            .create_entity()
            .with(Position(Vector3::zeros()))
            .with(Perception {
                target: Some(PerceivedTarget {
                    entity: victim,
                    position: Point3::new(0.2, 0.0, 0.0),
                    distance: 0.2,
                    visible: true,
                    since_seen: 0.0,
                }),
                ..Perception::ground_creature(3.0)
            })
            .with(trap())
            .with(PosedModel::at_rest(Arc::new(model), PoseFrame::Entity))
            .build();
        let jaw_turn = |world: &World| {
            let posed = world.read_storage::<PosedModel>();
            posed.get(trap).unwrap().poses[1].rotation.angle()
        };

        // The frame it bites, its jaw still stands open, as open as it
        // holds it lying in wait.
        SnapTrapSystem.run_now(&world);
        let held = 1.0 - IDLE_GAPE - WARY_NARROW - QUIVER;
        assert!(jaw_turn(&world) > held * FRAC_PI_2 - 1e-3);
        // A fifth of a second on, it has shut, and it bit the once.
        for _ in 0..10 {
            SnapTrapSystem.run_now(&world);
        }
        assert!(
            jaw_turn(&world) < 0.1,
            "its jaw stands {}",
            jaw_turn(&world)
        );
        assert_eq!(world.write_resource::<DamageQueue>().drain().len(), 1);
    }

    #[test]
    fn it_bites_once_holds_shut_and_then_opens() {
        let mut trap = trap();
        assert!(!trap.tick(0.1, false, false), "sprung by nothing");
        assert!(trap.is_open());
        assert!(trap.tick(0.1, true, true));
        assert!(!trap.is_open());
        // Shut, it does not bite again whoever is still in it.
        assert!(!trap.tick(1.0, true, true));
        assert!(trap.moment().gape < 0.01, "it should be holding shut");
        // Nor while it is easing open.
        assert!(!trap.tick(1.5, true, true));
        assert!(!trap.is_open());
        let half_way = trap.moment().gape;
        assert!((0.2..0.8).contains(&half_way), "easing open at {half_way}");
        assert!(!trap.tick(0.6, true, true));
        assert!(trap.is_open(), "it should have opened");
        let gape = trap.moment().gape;
        assert!(
            gape > 1.0 - IDLE_GAPE - WARY_NARROW - QUIVER,
            "open to {gape}"
        );
        // And is ready again.
        assert!(trap.tick(0.1, true, true));
    }

    #[test]
    fn lying_open_it_breathes_and_narrows_its_jaw_when_someone_comes_near() {
        let mut trap = trap();
        let mut gapes = Vec::new();
        for _ in 0..(2.0 / (BREATH_RATE * 0.05)) as usize {
            trap.tick(0.05, false, false);
            gapes.push(trap.moment().gape);
        }
        let (low, high) = gapes
            .iter()
            .fold((f32::MAX, f32::MIN), |(lo, hi), g| (lo.min(*g), hi.max(*g)));
        assert!(high > 0.99 && low < 1.0 - 0.9 * IDLE_GAPE, "{low}..{high}");

        let calm: f32 = gapes.iter().sum::<f32>() / gapes.len() as f32;
        let mut wary = Vec::new();
        for _ in 0..gapes.len() {
            trap.tick(0.05, false, true);
            wary.push(trap.moment().gape);
        }
        let wary: f32 = wary.iter().sum::<f32>() / wary.len() as f32;
        assert!(wary < calm - 0.7 * WARY_NARROW, "calm {calm}, wary {wary}");
        assert!(trap.is_open(), "only coming near does not spring it");
    }

    #[test]
    fn the_jaw_slams_fast_and_creaks_open_slowly() {
        let cycle = trap().cycle;
        let mut last = 1.0;
        let mut steepest_fall: f32 = 0.0;
        let mut steepest_rise: f32 = 0.0;
        let dt = 0.01;
        for step in 1..=(cycle.length() / dt) as usize {
            let since = step as f32 * dt;
            let (gape, caught) = cycle.jaw(since);
            assert!((0.0..=1.0).contains(&gape), "gape {gape} at {since} s");
            // It has its catch for as long as it is shut, and no longer.
            assert_eq!(
                caught.is_some(),
                (cycle.slam..cycle.hold).contains(&since),
                "at {since} s"
            );
            if since <= cycle.slam {
                steepest_fall = steepest_fall.max((last - gape) / dt);
            }
            if since > cycle.hold {
                steepest_rise = steepest_rise.max((gape - last) / dt);
            }
            last = gape;
        }
        assert!(
            steepest_fall > 6.0 * steepest_rise,
            "it shuts at {steepest_fall} and opens at {steepest_rise} gapes a second"
        );
        // Drawn right in by the end of the hold.
        let (_, caught) = cycle.jaw(cycle.hold - 1e-3);
        assert!(caught.unwrap().swallowed > 0.99);
    }

    #[test]
    fn the_jaw_turns_on_its_hinge() {
        let rig = trap().rig;
        // The jaw's lip, a metre out from the hinge and level with it.
        let lip = Vector3::new(0.3, 0.1, 0.5);
        let at = |moment: &Moment| {
            let jaw = &rig.poses(moment)[1];
            jaw.translation + jaw.rotation * lip
        };
        assert!((at(&Moment::SHUT) - lip).norm() < 1e-5);
        // Wide open, the lip stands a metre straight over the hinge.
        let open = at(&Moment::OPEN);
        assert!(
            (open - Vector3::new(0.3, 1.1, -0.5)).norm() < 1e-4,
            "the lip is at {open:?}"
        );
        // And the hinge's own line never moves.
        let pivot = rig.hinge.pivot + Vector3::x() * 0.7;
        let jaw = &rig.poses(&Moment::OPEN)[1];
        assert!((jaw.translation + jaw.rotation * pivot - pivot).norm() < 1e-5);
        // The bed lies still.
        let bed = &rig.poses(&Moment::OPEN)[0];
        assert_eq!(bed.translation, Vector3::zeros());
        assert_eq!(bed.rotation, UnitQuaternion::identity());
        assert_eq!(bed.scale, Vector3::repeat(1.0));
    }

    #[test]
    fn a_muscle_stays_fast_to_both_jaws_and_a_catch_is_gone_when_it_opens() {
        let (root, hold) = (Vector3::new(0.1, 0.0, -0.2), Vector3::new(0.1, 0.2, -0.2));
        let rig = TrapRig {
            hinge: hinge(),
            parts: vec![
                TrapPart::Muscle {
                    root,
                    hold,
                    length: 0.4,
                },
                TrapPart::Catch {
                    held: Vector3::new(0.0, 0.1, 0.5),
                    kick: 0.3,
                    lift: 0.0,
                    drawn: Vector3::new(0.0, 0.0, -0.8),
                    shrunk: 0.5,
                },
            ],
        };
        for gape in [0.0, 0.4, 1.0] {
            let moment = Moment {
                gape,
                ..Moment::OPEN
            };
            let poses = rig.poses(&moment);
            let muscle = &poses[0];
            let tip = muscle.translation
                + muscle.rotation * Vector3::new(0.0, 0.4, 0.0).component_mul(&muscle.scale);
            let jaw = rig.hinge.pose(gape);
            let held = jaw.translation + jaw.rotation * hold;
            assert!((tip - held).norm() < 1e-4, "its end is at {tip:?}");
        }
        let foot = Vector3::new(0.0, 0.1, 1.2);
        let limp = &rig.poses(&Moment::SHUT)[1];
        assert!((limp.translation + limp.rotation * foot - foot).norm() < 1e-4);
        assert_eq!(rig.poses(&Moment::OPEN)[1].scale, Vector3::zeros());
    }
}
