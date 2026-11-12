//! A stilt walker's animation driver.
//!
//! ```text
//!   Grounding ─┐
//!   intent ────┼─► LeggedLocomotion ─► feet ─┐
//!   velocity ──┘        (shared)             ├─► StiltSkeleton ─┐
//!   pelvis, yaw ─────────────────────────────┘                  ├─► poses
//!   Mood ──► neck pitch, lids, what the eyes watch ─────────────┤    (one to a part
//!   clock ─► blinks, glances, breath, gasps ────────────────────┘     of the Dress)
//! ```
//!
//! Walking is entirely [`crate::animation::LeggedLocomotion`], the same
//! component the player and the heart critter use. What is the walker's own
//! is above the waist: a neck that rears back and spears forward, and the
//! small motions that make it look alive when it is doing nothing at all.

use std::f32::consts::TAU;
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use specs::{Component, VecStorage};

use crate::animation::rig::right_vector;
use crate::animation::{FootSide, LegMode, LeggedLocomotion, LocomotionCtx};
use crate::character::{CharacterIntent, Grounding};
use crate::model::{MeshPrimitive, Transform};
use crate::sensing::{ContactCandidate, Probe};

use super::carriage::{CarriageMotion, Gait};
use super::config::StiltRigConfig;
use super::drape::DrapeMotion;
use super::dress::{Dress, Drive, Joint};
use super::skeleton::{aim, FootPose, StiltSkeleton};

/// Exponential rate (per second) at which the legs tuck when the ground
/// leaves and untuck when it comes back.
const TUCK_RATE: f32 = 14.0;

/// How far under the hips a tucked foot rides, as a fraction of standing
/// height.
const TUCK_HEIGHT_FRACTION: f32 = 0.55;

/// Seconds a blink takes, and the least and most between two.
const BLINK_SECONDS: f32 = 0.16;
const BLINK_EVERY: (f32, f32) = (1.8, 6.0);

/// Seconds an idle eye rests on one spot, least and most, how far off
/// straight ahead it wanders as a share of its swivel, and how fast it
/// moves to a new spot or to what it watches, 1/s. Fast: an eye darts.
const GLANCE_EVERY: (f32, f32) = (0.4, 2.2);
const GLANCE_REACH: f32 = 0.7;
const EYE_RATE: f32 = 18.0;

/// Breaths a second at rest.
const BREATH_RATE: f32 = 0.35;

/// What the creature's head is doing, as something other than the rig
/// decided.
///
/// Plain numbers and a place rather than a behaviour or an attack: the rig
/// has no business knowing what a brain is, and everything it needs to
/// draw an expression is *how awake*, *how far through a lunge* and *what
/// it is looking at*.
#[derive(Clone, Copy, Debug, Default)]
pub struct Mood {
    /// How alert the creature is, in `[0, 1]`. At 0 its eyes are shut and
    /// it is dozing on its feet; at 1 they are wide open.
    pub alertness: f32,
    /// Where a strike is, in `[-1, 1]`: negative reared back, positive
    /// thrust out. See [`crate::creature::MeleeAttack::thrust`].
    pub thrust: f32,
    /// What it is watching, if anything: its eyes turn to it.
    pub watching: Option<Point3<f32>>,
}

/// A stilt walker's animation state.
#[derive(Component)]
#[storage(VecStorage)]
pub struct StiltAnimator {
    pub config: StiltRigConfig,
    /// Feet, probes and the support frame. The same component the player
    /// walks on.
    pub locomotion: LeggedLocomotion,
    pub skeleton: StiltSkeleton,
    /// What it is drawn as.
    dress: Option<Arc<Dress>>,
    /// How its dress's drape, if it wears one, hangs now.
    drape: DrapeMotion,

    /// How tucked the legs are, in `[0, 1]`.
    tuck: f32,
    /// How shut the eyes are held, in `[0, 1]`. Chases the mood rather
    /// than snapping to it.
    lid_close: f32,
    /// How the body rides the legs this frame.
    carriage: CarriageMotion,
    /// Where a strike is, as the mood last gave it.
    thrust: f32,
    /// What the eyes are turned to, as the mood last gave it.
    watching: Option<Point3<f32>>,

    /// Seconds it has been alive: the clock its breath and gasps run on.
    clock: f32,
    /// Seconds until it next blinks, and seconds into a blink if it is in
    /// one.
    next_blink: f32,
    blinking: Option<f32>,
    /// Where its eyes rest when it watches nothing, as a turn to the side
    /// and upward in shares of their swivel; where they are now; and
    /// seconds until they move on.
    glance_goal: (f32, f32),
    glance: (f32, f32),
    next_glance: f32,
    /// The chance its blinks and glances are drawn from.
    chance: u32,
}

impl StiltAnimator {
    /// A walker whose physics body sits at `body_position`, riding
    /// `ground_clearance` above the surface it rests on.
    pub fn new(
        config: StiltRigConfig,
        body_position: Point3<f32>,
        ground_clearance: f32,
        yaw: f32,
    ) -> Self {
        let locomotion = LeggedLocomotion::new(
            config.leg_dims(),
            config.foot_placer,
            body_position,
            ground_clearance,
            yaw,
        );
        let pelvis = locomotion.pelvis_for(body_position);
        let skeleton = StiltSkeleton::new(&config, pelvis, facing_from(yaw));
        // No two walkers blink together: each draws from where it stood.
        let chance = (body_position.x.to_bits() ^ body_position.z.to_bits().rotate_left(13)) | 1;

        Self {
            config,
            locomotion,
            skeleton,
            dress: None,
            drape: DrapeMotion::default(),
            tuck: 0.0,
            lid_close: 1.0,
            carriage: CarriageMotion::default(),
            thrust: 0.0,
            watching: None,
            clock: 0.0,
            next_blink: BLINK_EVERY.0,
            blinking: None,
            glance_goal: (0.0, 0.0),
            glance: (0.0, 0.0),
            next_glance: GLANCE_EVERY.0,
            chance,
        }
    }

    /// Drawn as `dress`.
    pub fn wearing(mut self, dress: Arc<Dress>) -> Self {
        self.dress = Some(dress);
        self
    }

    /// What it is drawn as, if anything.
    pub fn dress(&self) -> Option<&Arc<Dress>> {
        self.dress.as_ref()
    }

    /// Where the rig's pelvis belongs, given where the physics body is.
    pub fn pelvis_for(&self, body_position: Point3<f32>) -> Point3<f32> {
        self.locomotion.pelvis_for(body_position)
    }

    /// Aim the foot probes for the coming frame.
    pub fn configure_probes(&self, pelvis: Point3<f32>, yaw: f32) -> Vec<Probe> {
        self.locomotion.configure_probes(pelvis, yaw)
    }

    /// Advance one frame.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        dt: f32,
        pelvis: Point3<f32>,
        yaw: f32,
        velocity: Vector3<f32>,
        grounding: &Grounding,
        intent: &CharacterIntent,
        contacts: &[ContactCandidate],
        mood: Mood,
    ) {
        // A gait means something only in the frame of whatever is holding
        // the body up.
        let support_velocity = self.locomotion.observe_support(grounding, dt);
        let velocity = velocity - support_velocity;
        let airborne = !grounding.is_grounded;

        self.locomotion.process_contacts(contacts);
        self.locomotion.tick(&LocomotionCtx {
            dt,
            pelvis,
            yaw,
            velocity,
            support_velocity,
            intent_direction: intent.direction,
            mode: if airborne {
                LegMode::Off
            } else {
                LegMode::Step
            },
            step_height: self.config.step_height,
            stride_gain: self.config.stride_gain,
            pose_tag: if airborne { "airborne" } else { "grounded" },
        });

        let tuck_target = if airborne { 1.0 } else { 0.0 };
        self.tuck += (tuck_target - self.tuck) * approach(TUCK_RATE, dt);

        let lid_target = 1.0 - mood.alertness.clamp(0.0, 1.0);
        self.lid_close += (lid_target - self.lid_close) * approach(self.config.lid_rate, dt);
        self.thrust = mood.thrust.clamp(-1.0, 1.0);
        self.watching = mood.watching;
        self.idle(dt);

        let facing = facing_from(yaw);
        let feet = self.foot_poses(pelvis, facing);
        // It leans away from whichever foot is in the air, as anything on
        // two legs must, sinks as both come down, and its top lags behind
        // whatever its legs do.
        let lifted = (feet[0].position.y - feet[1].position.y) / self.config.step_height.max(1e-3);
        self.carriage.step(
            &self.config.carriage,
            self.config.standing_height(),
            dt,
            &Gait {
                velocity,
                facing,
                lifted: lifted * (1.0 - self.tuck),
                clock: self.clock,
            },
        );
        let carried = pelvis - Vector3::y() * self.carriage.dip();
        self.skeleton.update(
            &self.config,
            carried,
            facing,
            self.neck_pitch(self.thrust),
            self.carriage.tilt(facing),
            feet,
        );
        if let Some(drape) = self.dress.as_ref().and_then(|dress| dress.drape.as_ref()) {
            let legs = [FootSide::Left, FootSide::Right].map(|side| {
                let leg = self.skeleton.leg(side);
                [leg.hip, leg.knee, leg.foot]
            });
            self.drape.step(drape, &self.skeleton.hips(), &legs, dt);
        }
    }

    /// Its dress's drape as it hangs now, in the frame at `origin` turned
    /// by `turn` in the world: into `out`, reusing what it holds. Leaves
    /// `out` alone for a walker that wears none.
    pub fn drape_into(
        &self,
        origin: &Vector3<f32>,
        turn: &UnitQuaternion<f32>,
        out: &mut Vec<MeshPrimitive>,
    ) {
        let Some(drape) = self.dress.as_ref().and_then(|dress| dress.drape.as_ref()) else {
            return;
        };
        let hips = self.skeleton.hips();
        let back = turn.inverse();
        let place = Transform {
            translation: back * (hips.translation - origin),
            rotation: back * hips.rotation,
            scale: Vector3::repeat(1.0),
        };
        self.drape.shape(drape, &place, out);
    }

    /// The small motions nothing asks for: blinks and glances.
    fn idle(&mut self, dt: f32) {
        self.clock += dt;

        self.blinking = match self.blinking {
            Some(into) if into + dt < BLINK_SECONDS => Some(into + dt),
            Some(_) => None,
            None => {
                self.next_blink -= dt;
                (self.next_blink <= 0.0).then(|| {
                    self.next_blink = self.draw(BLINK_EVERY);
                    0.0
                })
            }
        };

        self.next_glance -= dt;
        if self.next_glance <= 0.0 {
            self.next_glance = self.draw(GLANCE_EVERY);
            self.glance_goal = (
                self.draw((-GLANCE_REACH, GLANCE_REACH)),
                self.draw((-0.5 * GLANCE_REACH, 0.5 * GLANCE_REACH)),
            );
        }
        let step = approach(EYE_RATE, dt);
        self.glance.0 += (self.glance_goal.0 - self.glance.0) * step;
        self.glance.1 += (self.glance_goal.1 - self.glance.1) * step;
    }

    /// A number from `range`, the next of this walker's own.
    fn draw(&mut self, range: (f32, f32)) -> f32 {
        self.chance ^= self.chance << 13;
        self.chance ^= self.chance >> 17;
        self.chance ^= self.chance << 5;
        let share = (self.chance >> 8) as f32 / (1u32 << 24) as f32;
        range.0 + (range.1 - range.0) * share
    }

    /// How shut the eyes are this instant, in `[0, 1]`: how they are held,
    /// and any blink over that.
    pub fn lid_close(&self) -> f32 {
        let blink = self.blinking.map_or(0.0, |into| {
            // Down fast, up slower.
            let t = into / BLINK_SECONDS;
            if t < 0.35 {
                t / 0.35
            } else {
                1.0 - (t - 0.35) / 0.65
            }
        });
        self.lid_close.max(blink)
    }

    /// Where the neck is pointed, in radians forward of vertical.
    ///
    /// Not smoothed: the swing timing lives in the attack, and easing it
    /// here would put the visible lunge out of step with the blow that
    /// actually lands.
    fn neck_pitch(&self, thrust: f32) -> f32 {
        let config = &self.config;
        let extreme = if thrust >= 0.0 {
            config.strike_pitch
        } else {
            config.rear_pitch
        };
        config.rest_pitch + (extreme - config.rest_pitch) * thrust.abs()
    }

    /// Where each foot should be drawn: where the gait put it, pulled
    /// toward a tucked stance under the hip by however airborne the
    /// creature is.
    fn foot_poses(&self, pelvis: Point3<f32>, facing: Vector3<f32>) -> [FootPose; 2] {
        let right = right_vector(facing);
        let tucked_drop = Vector3::y() * (self.config.standing_height() * TUCK_HEIGHT_FRACTION);

        [FootSide::Left, FootSide::Right].map(|side| {
            let foot = self.locomotion.foot(side);
            let hip = pelvis + right * (side.side_sign() * self.config.hip_width);
            let tucked = hip - tucked_drop;

            FootPose {
                position: foot.position + (tucked - foot.position) * self.tuck,
                forward: foot.forward,
                up: foot.up.lerp(&Vector3::y(), self.tuck),
            }
        })
    }

    /// [`Self::poses`] measured from `origin`, along the world's axes: what
    /// a [`PoseFrame::Position`] model of a body at `origin` is posed with.
    ///
    /// [`PoseFrame::Position`]: crate::components::PoseFrame::Position
    pub fn poses_from(&self, origin: &Vector3<f32>) -> Vec<Transform> {
        let mut poses = self.poses();
        for pose in &mut poses {
            pose.translation -= origin;
        }
        poses
    }

    /// Where each part of the dress is this frame, in the world, in the
    /// order of the dress's parts. Empty for a walker with no dress.
    pub fn poses(&self) -> Vec<Transform> {
        let Some(dress) = &self.dress else {
            return Vec::new();
        };
        let head = self.skeleton.head();
        let breath = (self.clock * BREATH_RATE * TAU).sin();
        dress
            .worn
            .iter()
            .map(|worn| {
                let mut pose = match worn.joint {
                    Joint::Hips => self.skeleton.hips(),
                    Joint::Neck => self.skeleton.neck(),
                    Joint::Head => head.clone(),
                    Joint::Eye { socket, swivel } => self.eye(&head, socket, Vector3::z(), swivel),
                    Joint::SideEye {
                        socket,
                        out,
                        swivel,
                    } => self.eye(&head, socket, out, swivel),
                    Joint::Hinge { pivot, axis, drive } => {
                        let turn =
                            UnitQuaternion::from_scaled_axis(axis.normalize() * self.drive(drive));
                        Transform {
                            translation: head.translation + head.rotation * pivot,
                            rotation: head.rotation * turn,
                            scale: head.scale,
                        }
                    }
                    Joint::Thigh(side) => self.skeleton.thigh(side),
                    Joint::Shin(side) => self.skeleton.shin(side),
                    Joint::Foot(side) => self.skeleton.foot(side),
                };
                // A breath swells a body across more than along.
                let swell = worn.breath * breath;
                pose.scale = Vector3::new(1.0 + swell, 1.0 + 0.3 * swell, 1.0 + swell);
                pose
            })
            .collect()
    }

    /// An eye at `socket` in the head, which at rest looks out along `out`
    /// in the head's frame: turned to what the walker watches, or to
    /// wherever it is idly glancing, and never further than `swivel` off
    /// its rest.
    fn eye(
        &self,
        head: &Transform,
        socket: Vector3<f32>,
        out: Vector3<f32>,
        swivel: f32,
    ) -> Transform {
        let at = Point3::from(head.translation + head.rotation * socket);
        let ahead = head.rotation * out.try_normalize(1e-6).unwrap_or_else(Vector3::z);
        let up = head.rotation * Vector3::y();
        // Square to both: the head's own side, for an eye that looks ahead.
        let side = up
            .cross(&ahead)
            .try_normalize(1e-5)
            .unwrap_or_else(|| head.rotation * Vector3::x());
        let wanted = match self.watching {
            Some(target) => (target - at).try_normalize(1e-4).unwrap_or(ahead),
            None => {
                (ahead + (side * self.glance.0 + up * self.glance.1) * swivel.tan()).normalize()
            }
        };
        // Held within its swivel: an eye cannot look out through the back
        // of its own socket.
        let off = ahead.dot(&wanted).clamp(-1.0, 1.0).acos();
        let look = if off <= swivel {
            wanted
        } else {
            let round = (wanted - ahead * ahead.dot(&wanted))
                .try_normalize(1e-5)
                .unwrap_or(up);
            ahead * swivel.cos() + round * swivel.sin()
        };
        aim(at, look, up)
    }

    /// How far a hinge on the head is turned this instant, radians.
    fn drive(&self, drive: Drive) -> f32 {
        match drive {
            Drive::Lid { shut } => shut * self.lid_close(),
            Drive::Gasp { open, rate } => {
                // Open for most of each gasp, snapping shut between.
                let phase = (self.clock * rate).fract();
                open * (phase * TAU * 0.5).sin().max(0.0).powf(0.6)
            }
            Drive::Strike { open } => open * (-self.thrust).max(0.0),
        }
    }
}

/// The fraction of the way to a target an exponential chase covers in
/// `dt` at `rate` per second.
#[inline]
fn approach(rate: f32, dt: f32) -> f32 {
    1.0 - (-rate * dt).exp()
}

#[inline]
fn facing_from(yaw: f32) -> Vector3<f32> {
    Vector3::new(yaw.sin(), 0.0, yaw.cos())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::animation::stilts::Worn;
    use crate::model::{Model, ModelPart};

    /// A walker dressed in three empty parts: a head, an eye and a lid.
    fn walker() -> StiltAnimator {
        walker_at(Point3::new(0.0, 1.0, 0.0))
    }

    /// [`walker`], its body at `body`.
    fn walker_at(body: Point3<f32>) -> StiltAnimator {
        let dress = Dress {
            model: Arc::new(Model::flat(vec![
                ModelPart::new(Vec::new()),
                ModelPart::new(Vec::new()),
                ModelPart::new(Vec::new()),
            ])),
            worn: vec![
                Worn::on(Joint::Head),
                Worn::on(Joint::Eye {
                    socket: Vector3::new(0.0, 0.0, 0.1),
                    swivel: 0.6,
                }),
                Worn::on(Joint::Hinge {
                    pivot: Vector3::zeros(),
                    axis: Vector3::x(),
                    drive: Drive::Lid { shut: 1.2 },
                }),
            ],
            drape: None,
        };
        StiltAnimator::new(StiltRigConfig::default(), body, 1.0, 0.0).wearing(Arc::new(dress))
    }

    /// Stand `walker` still for `seconds` in `mood`.
    fn stand(walker: &mut StiltAnimator, seconds: f32, mood: Mood) {
        let dt = 1.0 / 60.0;
        for _ in 0..(seconds / dt) as usize {
            let pelvis = walker.pelvis_for(Point3::new(0.0, 1.0, 0.0));
            walker.update(
                dt,
                pelvis,
                0.0,
                Vector3::zeros(),
                &Grounding::on(Vector3::y()),
                &CharacterIntent::default(),
                &[],
                mood,
            );
        }
    }

    #[test]
    fn poses_measured_from_the_body_stay_within_the_walker_wherever_it_stands() {
        let body = Point3::new(40.0, 3.0, -25.0);
        let walker = walker_at(body);
        let reach = StiltRigConfig::default().body_height();
        let (world, from_body) = (walker.poses(), walker.poses_from(&body.coords));
        assert_eq!(from_body.len(), 3);
        for (world, from_body) in world.iter().zip(&from_body) {
            assert!(from_body.translation.norm() < reach, "{from_body:?}");
            let back = from_body.translation + body.coords;
            assert!((back - world.translation).norm() < 1.0e-4);
        }
    }

    #[test]
    fn an_eye_turns_to_what_is_watched_but_no_further_than_its_swivel() {
        let mut walker = walker();
        let eye_at = |walker: &StiltAnimator| walker.poses()[1].clone();
        // Something a little to one side: the eye looks straight at it.
        let near = eye_at(&walker).translation + Vector3::new(0.3, 0.0, 2.0);
        let mood = |watching| Mood {
            alertness: 1.0,
            thrust: 0.0,
            watching: Some(Point3::from(watching)),
        };
        stand(&mut walker, 0.5, mood(near));
        let eye = eye_at(&walker);
        let look = eye.rotation * Vector3::z();
        assert!(look.dot(&(near - eye.translation).normalize()) > 0.999);

        // Something behind it: as far round as the socket allows.
        let behind = eye.translation + Vector3::new(1.0, 0.0, -3.0);
        stand(&mut walker, 0.5, mood(behind));
        let look = eye_at(&walker).rotation * Vector3::z();
        let ahead = walker.poses()[0].rotation * Vector3::z();
        let off = look.dot(&ahead).acos();
        assert!((off - 0.6).abs() < 0.01, "turned {off} rad");
        assert!(look.x > 0.0, "it turned the wrong way round");
    }

    #[test]
    fn a_dozing_walkers_lids_are_shut_and_an_alert_ones_blink() {
        let mut walker = walker();
        let lid = |walker: &StiltAnimator| {
            let (head, lid) = (walker.poses()[0].clone(), walker.poses()[2].clone());
            (head.rotation.inverse() * lid.rotation).angle()
        };
        stand(&mut walker, 1.0, Mood::default());
        assert!(
            (lid(&walker) - 1.2).abs() < 0.02,
            "dozing, lids at {}",
            lid(&walker)
        );

        let awake = Mood {
            alertness: 1.0,
            ..Mood::default()
        };
        // Wide open most of the time, and shut at some instant in any
        // stretch longer than the longest wait between blinks.
        let (mut open, mut most_shut) = (0, 0.0f32);
        stand(&mut walker, 1.0, awake);
        for _ in 0..(1.5 * BLINK_EVERY.1 * 60.0) as usize {
            stand(&mut walker, 1.0 / 60.0, awake);
            most_shut = most_shut.max(lid(&walker));
            open += usize::from(lid(&walker) < 0.05);
        }
        assert!(most_shut > 1.0, "it never blinked: {most_shut}");
        assert!(
            open > (BLINK_EVERY.1 * 60.0) as usize,
            "its eyes were shut too long"
        );
    }

    #[test]
    fn a_side_eye_looks_out_of_the_heads_side_and_turns_no_further_than_its_swivel() {
        let dress = Dress {
            model: Arc::new(Model::flat(vec![
                ModelPart::new(Vec::new()),
                ModelPart::new(Vec::new()),
            ])),
            worn: vec![
                Worn::on(Joint::Head),
                Worn::on(Joint::SideEye {
                    socket: Vector3::new(0.1, 0.0, 0.0),
                    out: Vector3::x(),
                    swivel: 0.3,
                }),
            ],
            drape: None,
        };
        let mut walker = StiltAnimator::new(
            StiltRigConfig::default(),
            Point3::new(0.0, 1.0, 0.0),
            1.0,
            0.0,
        )
        .wearing(Arc::new(dress));
        let mood = |watching: Vector3<f32>| Mood {
            alertness: 1.0,
            thrust: 0.0,
            watching: Some(Point3::from(watching)),
        };
        // The head's own side and front, and where the eye is.
        let frame = |walker: &StiltAnimator| {
            let (head, eye) = (walker.poses()[0].clone(), walker.poses()[1].clone());
            (
                head.rotation * Vector3::x(),
                head.rotation * Vector3::z(),
                eye,
            )
        };

        // Something a little ahead of square to its side: it looks straight
        // at it.
        let (side, ahead, eye) = frame(&walker);
        let beside = eye.translation + side * 3.0 + ahead * 0.5;
        stand(&mut walker, 0.5, mood(beside));
        let (_, _, eye) = frame(&walker);
        let look = eye.rotation * Vector3::z();
        assert!(look.dot(&(beside - eye.translation).normalize()) > 0.999);

        // Something dead ahead of the head: as far round as its swivel
        // lets it, and still out of the head's side.
        let (side, ahead, eye) = frame(&walker);
        stand(&mut walker, 0.5, mood(eye.translation + ahead * 5.0));
        let (side_now, ahead_now, eye) = frame(&walker);
        let look = eye.rotation * Vector3::z();
        let off = look.dot(&side_now).clamp(-1.0, 1.0).acos();
        assert!((off - 0.3).abs() < 0.01, "turned {off} rad");
        assert!(look.dot(&ahead_now) > 0.0, "it turned the wrong way round");
        // The head has not turned to help: it only sways as it stands.
        let idle_sway = StiltRigConfig::default().carriage.idle_sway;
        assert!((side - side_now).norm() < 2.5 * idle_sway);
    }

    #[test]
    fn a_walker_with_no_dress_has_nothing_to_pose() {
        let bare = StiltAnimator::new(StiltRigConfig::default(), Point3::origin(), 1.0, 0.0);
        assert!(bare.poses().is_empty());
    }
}
