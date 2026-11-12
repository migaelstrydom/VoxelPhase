//! Lobbing: a creature that throws bombs at what it sees.
//!
//! ```text
//!   Perception ──target in range──▶ a grenade, launched on the arc that
//!                                   lands on the target after `flight_time`
//!
//!   seconds to the next throw ──▶ LobberBody ──▶ PosedModel: a bag that sags
//!   seconds since the last ─────▶                after a throw and swells
//!   what it watches ────────────▶                tight as the next comes,
//!                                                cheeks that fill it, eyes
//!                                                that follow the target,
//!                                                and the whole of it
//!                                                squatting and trembling
//!                                                as it winds up, heaving
//!                                                up as the bomb leaves
//! ```
//!
//! The bombs are the game's own grenades, so they fall, bounce, go off and
//! carve the ground exactly as the player's do: a lobber pins a route down,
//! and breaks it up while it does.

use std::f32::consts::TAU;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use specs::{
    Component, DenseVecStorage, Entities, Join, LazyUpdate, Read, ReadStorage, System, Write,
    WriteStorage,
};

use super::perception::Perception;
use crate::aim::launch::gravity_along;
use crate::components::{Orientation, PosedModel, Position};
use crate::damage::Dead;
use crate::model::Transform;
use crate::projectile::{spawn_grenade, GrenadeConfig, GrenadeModelResource};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// Breaths a second, and how much of the way from slack to tight a breath
/// moves the bag.
const BREATH_RATE: f32 = 0.3;
const BREATH_DEPTH: f32 = 0.07;

/// Puffs of the cheeks a second: just after a throw, and as the next is due.
const PUFF_RATE: (f32, f32) = (0.9, 2.6);

/// How far past slack a dead lobber's bag sinks, as a share of the way from
/// slack to tight.
const DEAD_SAG: f32 = 0.6;

/// Seconds before a throw it squats into it, how far it sinks and spreads
/// at the bottom of the squat (shares of its height and width), and how
/// fast and how hard it trembles there.
const WINDUP_SECONDS: f32 = 0.55;
const WINDUP_SQUAT: f32 = 0.09;
const WINDUP_SPREAD: f32 = 0.05;
const TREMBLE_RATE: f32 = 11.0;
const TREMBLE: f32 = 0.015;

/// How far it stretches up as a bomb leaves (a share of its height), how
/// fast that dies away (seconds to fall to a third), and how many times a
/// second it bounces as it settles.
const HEAVE: f32 = 0.16;
const HEAVE_DECAY: f32 = 0.2;
const HEAVE_RATE: f32 = 2.4;

/// A part of a lobber's model that swells and sags.
#[derive(Debug, Clone, Copy)]
pub struct Swell {
    /// Which part of the model it is.
    pub part: usize,
    /// The point of it that stays where it is as it swells, in the
    /// entity's frame: where a bag sits on the ground.
    pub seat: Vector3<f32>,
    /// Its size when it is empty and when it is full, as scales of the
    /// size it was sculpted.
    pub slack: Vector3<f32>,
    pub tight: Vector3<f32>,
}

impl Swell {
    /// Its pose `full` of the way from slack to tight. Below zero is
    /// emptier than slack.
    pub fn pose(&self, full: f32) -> Transform {
        let scale = self.slack + (self.tight - self.slack) * full;
        Transform {
            translation: self.seat - self.seat.component_mul(&scale),
            rotation: UnitQuaternion::identity(),
            scale,
        }
    }
}

/// An eye in a lobber's model that turns to what it watches.
#[derive(Debug, Clone, Copy)]
pub struct Gaze {
    /// Which part of the model it is: an eye about its own middle, looking
    /// along +Z.
    pub part: usize,
    /// Its middle, and the way it looks at rest, in the entity's frame.
    pub socket: Vector3<f32>,
    pub ahead: Vector3<f32>,
    /// How far off that it can turn, radians.
    pub swivel: f32,
}

impl Gaze {
    /// Its pose looking toward `target` (in the entity's frame), as far as
    /// its swivel lets it; at rest with nothing to look at.
    pub fn pose(&self, target: Option<Vector3<f32>>) -> Transform {
        let ahead = self.ahead.normalize();
        let wanted = target
            .and_then(|target| (target - self.socket).try_normalize(1e-4))
            .unwrap_or(ahead);
        let off = ahead.dot(&wanted).clamp(-1.0, 1.0).acos();
        let look = if off <= self.swivel {
            wanted
        } else {
            // Round toward it as far as the socket allows.
            let round = (wanted - ahead * ahead.dot(&wanted))
                .try_normalize(1e-5)
                .unwrap_or_else(Vector3::y);
            ahead * self.swivel.cos() + round * self.swivel.sin()
        };
        Transform {
            translation: self.socket,
            rotation: UnitQuaternion::face_towards(&look, &Vector3::y()),
            scale: Vector3::repeat(1.0),
        }
    }
}

/// The parts of a lobber's model that show what it is about to do.
#[derive(Debug, Clone, Copy)]
pub struct LobberBody {
    /// The bag that empties with a throw and fills for the next.
    pub bag: Swell,
    /// Cheeks that blow the bag full, if it has them.
    pub cheeks: Option<Swell>,
    /// Eyes that follow what it throws at.
    pub eyes: [Option<Gaze>; 2],
    /// The part that carries the rest (pipes, head, limbs), if it has one:
    /// it moves only with the heave of the whole.
    pub frame: Option<usize>,
}

/// Where a lobber is in a throw, for its body to show.
#[derive(Debug, Clone, Copy)]
pub struct Throw {
    /// How far it is from one throw to the next, 0 to 1.
    pub fullness: f32,
    /// Seconds since it last threw, if it has.
    pub since: Option<f32>,
    /// Seconds until it throws, if it has something to throw at.
    pub until: Option<f32>,
}

impl Throw {
    /// How the whole body is scaled about its seat this instant: squatting
    /// and trembling into a throw, stretched up and bouncing after one.
    pub fn heave(&self, clock: f32) -> Vector3<f32> {
        let windup = self.until.map_or(0.0, |until| {
            let into = (1.0 - until / WINDUP_SECONDS).clamp(0.0, 1.0);
            into * into * (3.0 - 2.0 * into)
        });
        let tremble = TREMBLE * windup * (clock * TREMBLE_RATE * TAU).sin();
        let release = self.since.map_or(0.0, |since| {
            HEAVE * (-since / HEAVE_DECAY).exp() * (since * HEAVE_RATE * TAU).cos()
        });
        let up = 1.0 - WINDUP_SQUAT * windup + tremble + release;
        let across = 1.0 + WINDUP_SPREAD * windup - 0.5 * release;
        Vector3::new(across, up, across)
    }
}

impl LobberBody {
    /// Pose the parts it moves, in `poses` (one to a part of the model):
    /// where it is in `throw`, `clock` seconds into its breathing, watching
    /// `target` (in the entity's frame).
    pub fn pose(
        &self,
        poses: &mut [Transform],
        throw: Throw,
        clock: f32,
        target: Option<Vector3<f32>>,
    ) {
        let fullness = throw.fullness.clamp(0.0, 1.0);
        // It fills fast at first and strains at the end.
        let filled = 1.0 - (1.0 - fullness) * (1.0 - fullness);
        let breath = BREATH_DEPTH * (clock * BREATH_RATE * TAU).sin();
        set(poses, self.bag.part, self.bag.pose(filled + breath));
        if let Some(cheeks) = self.cheeks {
            // Quicker and harder as the bag tightens against it.
            let rate = PUFF_RATE.0 + (PUFF_RATE.1 - PUFF_RATE.0) * fullness;
            let puff = 0.5 + 0.5 * (clock * rate * TAU).sin();
            set(
                poses,
                cheeks.part,
                cheeks.pose((0.35 + 0.65 * fullness) * puff),
            );
        }
        for eye in self.eyes.iter().flatten() {
            set(poses, eye.part, eye.pose(target));
        }
        if let Some(frame) = self.frame {
            set(poses, frame, Transform::default());
        }
        self.heave(poses, throw.heave(clock));
    }

    /// Scale every part it moves by `scale` about the bag's seat, the point
    /// it squats on.
    fn heave(&self, poses: &mut [Transform], scale: Vector3<f32>) {
        let seat = self.bag.seat;
        let parts = [
            Some(self.bag.part),
            self.cheeks.map(|cheeks| cheeks.part),
            self.frame,
        ]
        .into_iter()
        .chain(self.eyes.iter().map(|eye| eye.map(|eye| eye.part)))
        .flatten();
        for part in parts {
            if let Some(pose) = poses.get_mut(part) {
                pose.translation = seat + (pose.translation - seat).component_mul(&scale);
                pose.scale = pose.scale.component_mul(&scale);
            }
        }
    }

    /// Pose the parts it moves as a dead lobber's lie: the bag sunk, the
    /// cheeks empty, the eyes on nothing.
    pub fn slump(&self, poses: &mut [Transform]) {
        set(poses, self.bag.part, self.bag.pose(-DEAD_SAG));
        if let Some(cheeks) = self.cheeks {
            set(poses, cheeks.part, cheeks.pose(-DEAD_SAG));
        }
        for eye in self.eyes.iter().flatten() {
            set(poses, eye.part, eye.pose(None));
        }
    }
}

/// Put `pose` in `poses` for `part`, if the model has such a part.
fn set(poses: &mut [Transform], part: usize, pose: Transform) {
    if let Some(slot) = poses.get_mut(part) {
        *slot = pose;
    }
}

/// How a creature lobs.
#[derive(Component, Debug, Clone, Copy)]
#[storage(DenseVecStorage)]
pub struct Lobber {
    /// Nearest and furthest it will throw, m: nearer, the bomb would land on
    /// itself.
    pub range: (f32, f32),
    /// Seconds between throws.
    pub interval: f32,
    /// Seconds until it may throw again.
    pub remaining: f32,
    /// Seconds a bomb is in the air: long is a high slow arc, with time to
    /// get out from under it.
    pub flight_time: f32,
    /// Where a bomb leaves it, from its position.
    pub muzzle: Vector3<f32>,
    /// The parts of its [`PosedModel`] that show a throw coming, if it has
    /// any.
    pub body: Option<LobberBody>,
    /// Seconds since it last threw, if it has.
    pub since_throw: Option<f32>,
}

impl Lobber {
    /// How far it is from its last throw to its next, 0 to 1.
    pub fn fullness(&self) -> f32 {
        if self.interval <= 0.0 {
            return 1.0;
        }
        (1.0 - self.remaining / self.interval).clamp(0.0, 1.0)
    }
}

/// The velocity that carries a bomb from `from` to `to` in `flight_time`
/// under `gravity`.
pub fn lob(
    from: Point3<f32>,
    to: Point3<f32>,
    flight_time: f32,
    gravity: Vector3<f32>,
) -> Vector3<f32> {
    (to - from) / flight_time - gravity * (0.5 * flight_time)
}

/// Throws every lobber's bombs, and poses the bodies that show them coming.
pub struct LobberSystem;

impl<'a> System<'a> for LobberSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        Option<Read<'a, GrenadeConfig>>,
        Read<'a, GrenadeModelResource>,
        Write<'a, PhysicsResource>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Orientation>,
        ReadStorage<'a, Perception>,
        WriteStorage<'a, Lobber>,
        WriteStorage<'a, PosedModel>,
        ReadStorage<'a, Dead>,
        Read<'a, LazyUpdate>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            time,
            config,
            model,
            mut physics,
            positions,
            orientations,
            perceptions,
            mut lobbers,
            mut posed,
            deads,
            lazy,
        ) = data;
        // A world with no grenades to throw (a tool that only looks) has no
        // lobbing either.
        let grenades = config.zip(model.model.clone());
        let gravity = grenades
            .as_ref()
            .map(|(config, _)| gravity_along(physics.world.config().gravity, config.gravity));

        for (entity, position, perception, lobber, _) in
            (&entities, &positions, &perceptions, &mut lobbers, !&deads).join()
        {
            lobber.remaining = (lobber.remaining - time.delta_seconds()).max(0.0);
            if let Some(since) = &mut lobber.since_throw {
                *since += time.delta_seconds();
            }
            let target = perception.target.filter(|target| target.visible);
            let in_range = target.is_some_and(|target| {
                target.distance >= lobber.range.0 && target.distance <= lobber.range.1
            });

            if let (Some(body), Some(posed)) = (lobber.body, posed.get_mut(entity)) {
                let facing = orientations
                    .get(entity)
                    .map_or_else(UnitQuaternion::identity, |orientation| orientation.0);
                let watched =
                    target.map(|target| facing.inverse() * (target.position.coords - position.0));
                // No two breathe together.
                let clock = time.total_seconds() + 1.7 * entity.id() as f32;
                let throw = Throw {
                    fullness: lobber.fullness(),
                    since: lobber.since_throw,
                    until: in_range.then_some(lobber.remaining),
                };
                body.pose(&mut posed.poses, throw, clock, watched);
            }

            let (Some(target), Some((config, model)), Some(gravity)) = (target, &grenades, gravity)
            else {
                continue;
            };
            if lobber.remaining > 0.0 || !in_range {
                continue;
            }
            lobber.remaining = lobber.interval;
            lobber.since_throw = Some(0.0);
            let origin = Point3::from(position.0 + lobber.muzzle);
            spawn_grenade(
                lazy.create_entity(&entities),
                &mut physics.world,
                config,
                model.clone(),
                origin,
                lob(origin, target.position, lobber.flight_time, gravity),
            );
        }

        for (lobber, posed, _) in (&lobbers, &mut posed, &deads).join() {
            if let Some(body) = lobber.body {
                body.slump(&mut posed.poses);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A lobber whose model is a bag (part 0), cheeks (part 1) and an eye
    /// (part 2).
    fn body() -> LobberBody {
        LobberBody {
            bag: Swell {
                part: 0,
                seat: Vector3::new(0.0, -0.4, 0.0),
                slack: Vector3::new(1.0, 0.85, 1.0),
                tight: Vector3::new(1.05, 1.1, 1.05),
            },
            cheeks: Some(Swell {
                part: 1,
                seat: Vector3::new(0.0, 0.3, 0.4),
                slack: Vector3::repeat(0.8),
                tight: Vector3::repeat(1.2),
            }),
            eyes: [
                Some(Gaze {
                    part: 2,
                    socket: Vector3::new(0.0, 0.4, 0.5),
                    ahead: Vector3::z(),
                    swivel: 0.6,
                }),
                None,
            ],
            frame: Some(3),
        }
    }

    /// At `fullness`, with no throw lately and none coming.
    fn resting(fullness: f32) -> Throw {
        Throw {
            fullness,
            since: None,
            until: None,
        }
    }

    fn lobber(remaining: f32) -> Lobber {
        Lobber {
            range: (4.0, 20.0),
            interval: 4.0,
            remaining,
            flight_time: 1.8,
            muzzle: Vector3::y(),
            body: Some(body()),
            since_throw: None,
        }
    }

    #[test]
    fn a_lobbed_bomb_lands_on_what_it_was_thrown_at() {
        let gravity = Vector3::new(0.0, -12.0, 0.0);
        let (from, to) = (Point3::new(1.0, 3.0, -2.0), Point3::new(9.0, 0.5, 4.0));
        let flight = 1.8;
        let velocity = lob(from, to, flight, gravity);
        // Flown in small steps, as the physics flies it.
        let (mut at, mut speed) = (from, velocity);
        let dt = 1.0e-3;
        for _ in 0..(flight / dt).round() as usize {
            speed += gravity * dt;
            at += speed * dt;
        }
        assert!((at - to).norm() < 0.05, "landed at {at:?}");
        assert!(velocity.y > 0.0, "thrown downward");
    }

    #[test]
    fn its_bag_sags_after_a_throw_and_swells_tight_as_the_next_comes() {
        let body = body();
        let mut poses = vec![Transform::default(); 4];
        // At the same instant of its breath, from the throw to the next.
        let mut last = 0.0;
        for step in 0..=8 {
            let lobber = lobber(4.0 - 0.5 * step as f32);
            body.pose(&mut poses, resting(lobber.fullness()), 0.0, None);
            let height = poses[0].scale.y;
            assert!(height > last, "it sank at step {step}: {height}");
            last = height;
            // Wherever it has got to, it still sits where it sat.
            let seat = poses[0]
                .to_matrix()
                .transform_point(&Point3::from(body.bag.seat));
            assert!((seat.coords - body.bag.seat).norm() < 1e-5);
        }
        body.pose(&mut poses, resting(0.0), 0.0, None);
        assert!((poses[0].scale - body.bag.slack).norm() < 1e-5);
        body.pose(&mut poses, resting(1.0), 0.0, None);
        assert!((poses[0].scale - body.bag.tight).norm() < 1e-5);

        // A breath moves it a little, and never as far as a throw does.
        let breathing: Vec<f32> = (0..40)
            .map(|tick| {
                body.pose(&mut poses, resting(0.5), tick as f32 * 0.1, None);
                poses[0].scale.y
            })
            .collect();
        let (low, high) = breathing
            .iter()
            .fold((f32::MAX, f32::MIN), |(lo, hi), h| (lo.min(*h), hi.max(*h)));
        assert!(high > low, "it does not breathe");
        assert!(high - low < 0.25 * (body.bag.tight.y - body.bag.slack.y));

        // Dead, it lies lower than it ever did alive.
        body.slump(&mut poses);
        assert!(poses[0].scale.y < body.bag.slack.y);
    }

    #[test]
    fn it_squats_into_a_throw_and_heaves_up_as_the_bomb_leaves() {
        let body = body();
        let mut poses = vec![Transform::default(); 4];
        let height = |poses: &[Transform]| poses[3].scale.y;
        let at = |throw: Throw, poses: &mut Vec<Transform>| {
            // At a crossing of its tremble, so only the squat shows.
            body.pose(poses, throw, 0.0, None);
            height(poses)
        };
        let still = at(resting(0.5), &mut poses);
        assert!((still - 1.0).abs() < 1e-5, "at rest it is as sculpted");
        let due = |until| Throw {
            fullness: 1.0,
            since: None,
            until: Some(until),
        };
        assert!(
            (at(due(2.0), &mut poses) - 1.0).abs() < 1e-5,
            "too early to squat"
        );
        let squat = at(due(0.0), &mut poses);
        assert!(
            squat < 1.0 - 0.8 * WINDUP_SQUAT,
            "it did not squat: {squat}"
        );
        let thrown = |since| Throw {
            fullness: 0.0,
            since: Some(since),
            until: None,
        };
        let heave = at(thrown(0.0), &mut poses);
        assert!(heave > 1.0 + 0.9 * HEAVE, "it did not heave: {heave}");
        assert!(
            (at(thrown(2.0), &mut poses) - 1.0).abs() < 0.01,
            "it never settled"
        );

        // Squatting or heaving, it stays on its seat.
        body.pose(&mut poses, due(0.0), 0.0, None);
        let seat = poses[0]
            .to_matrix()
            .transform_point(&Point3::from(body.bag.seat));
        assert!((seat.coords - body.bag.seat).norm() < 1e-5);
    }

    #[test]
    fn its_eye_turns_to_what_it_watches_as_far_as_its_socket_lets_it() {
        let eye = body().eyes[0].unwrap();
        let look = |target| eye.pose(target).rotation * Vector3::z();
        assert!((look(None) - Vector3::z()).norm() < 1e-5);
        // A little to one side: straight at it.
        let near = eye.socket + Vector3::new(0.4, 0.1, 2.0);
        assert!(look(Some(near)).dot(&(near - eye.socket).normalize()) > 0.999);
        // Behind it: as far round as it goes, and the right way round.
        let behind = eye.socket + Vector3::new(1.0, 0.0, -3.0);
        let turned = look(Some(behind));
        assert!((turned.dot(&Vector3::z()).acos() - eye.swivel).abs() < 1e-3);
        assert!(turned.x > 0.0, "it turned the wrong way round");
        assert!((eye.pose(Some(behind)).translation - eye.socket).norm() < 1e-6);
    }
}
