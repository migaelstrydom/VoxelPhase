//! The peeper's animation driver.
//!
//! ```text
//!   Grounding ─┐
//!   intent ────┼─► LeggedLocomotion ─► feet ─┐
//!   velocity ──┘        (shared)             ├─► PeeperSkeleton ─► mesh
//!   pelvis, yaw ─────────────────────────────┤
//!   Mood ────────► neck pitch, eyelid ───────┘
//! ```
//!
//! Walking is entirely [`crate::animation::LeggedLocomotion`], the same
//! component the player and the heart critter use. What is peeper-specific
//! is above the waist: a neck that rears back and spears forward, and a
//! lid that says how awake the thing is.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

use crate::animation::rig::right_vector;
use crate::animation::{FootSide, LeggedLocomotion, LocomotionCtx};
use crate::character::{CharacterIntent, Grounding};
use crate::rendering::vertex::Vertex;
use crate::sensing::{ContactCandidate, Probe};

use super::config::PeeperRigConfig;
use super::mesh::generate_peeper_mesh;
use super::skeleton::{FootPose, PeeperSkeleton};

/// Exponential rate (per second) at which the legs tuck when the ground
/// leaves and untuck when it comes back.
const TUCK_RATE: f32 = 14.0;

/// How far under the hips a tucked foot rides, as a fraction of standing
/// height.
const TUCK_HEIGHT_FRACTION: f32 = 0.55;

/// What the creature's head is doing, as something other than the rig
/// decided.
///
/// Two plain numbers rather than a behaviour or an attack: the rig has no
/// business knowing what a brain is, and everything it needs to draw an
/// expression is *how awake* and *how far through a lunge*.
#[derive(Clone, Copy, Debug, Default)]
pub struct Mood {
    /// How alert the creature is, in `[0, 1]`. At 0 the eye is shut and
    /// the peeper is dozing on its feet; at 1 it is wide open.
    pub alertness: f32,
    /// Where a strike is, in `[-1, 1]`: negative reared back, positive
    /// thrust out. See [`crate::creature::MeleeAttack::thrust`].
    pub thrust: f32,
}

/// A peeper's animation state.
#[derive(Component)]
#[storage(VecStorage)]
pub struct PeeperAnimator {
    pub config: PeeperRigConfig,
    /// Feet, probes and the support frame. The same component the player
    /// walks on.
    pub locomotion: LeggedLocomotion,
    pub skeleton: PeeperSkeleton,

    /// How tucked the legs are, in `[0, 1]`.
    tuck: f32,
    /// How shut the eye is, in `[0, 1]`. Chases the mood rather than
    /// snapping to it, so the lid reads as a blink.
    lid_close: f32,

    cached_vertices: Vec<Vertex>,
    cached_indices: Vec<u32>,
}

impl PeeperAnimator {
    /// A peeper whose physics body sits at `body_position`, riding
    /// `ground_clearance` above the surface it rests on.
    pub fn new(
        config: PeeperRigConfig,
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
        let skeleton = PeeperSkeleton::new(&config, pelvis, facing_from(yaw));

        Self {
            config,
            locomotion,
            skeleton,
            tuck: 0.0,
            lid_close: 1.0,
            cached_vertices: Vec::new(),
            cached_indices: Vec::new(),
        }
    }

    /// Where the rig's pelvis belongs, given where the physics body is.
    pub fn pelvis_for(&self, body_position: Point3<f32>) -> Point3<f32> {
        self.locomotion.pelvis_for(body_position)
    }

    /// Aim the foot probes for the coming frame.
    pub fn configure_probes(&self, pelvis: Point3<f32>, yaw: f32) -> Vec<Probe> {
        self.locomotion.configure_probes(
            pelvis,
            yaw,
            [self.skeleton.legs[0].foot, self.skeleton.legs[1].foot],
        )
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
            airborne,
            step_height: self.config.step_height,
            stride_gain: self.config.stride_gain,
            pose_tag: if airborne { "airborne" } else { "grounded" },
        });

        let tuck_target = if airborne { 1.0 } else { 0.0 };
        self.tuck += (tuck_target - self.tuck) * approach(TUCK_RATE, dt);

        let lid_target = 1.0 - mood.alertness.clamp(0.0, 1.0);
        self.lid_close += (lid_target - self.lid_close) * approach(self.config.lid_rate, dt);

        let facing = facing_from(yaw);
        let feet = self.foot_poses(pelvis, facing);
        self.skeleton.update(
            dt,
            &self.config,
            pelvis,
            facing,
            self.neck_pitch(mood.thrust),
            feet,
        );
    }

    /// How the eye currently sits, in `[0, 1]` from wide open to shut.
    pub fn lid_close(&self) -> f32 {
        self.lid_close
    }

    /// Where the neck is pointed, in radians forward of vertical.
    ///
    /// Not smoothed: the swing timing lives in the attack, and easing it
    /// here would put the visible lunge out of step with the blow that
    /// actually lands.
    fn neck_pitch(&self, thrust: f32) -> f32 {
        let config = &self.config;
        let thrust = thrust.clamp(-1.0, 1.0);
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

    /// Mesh vertices and indices for rendering, regenerated from the
    /// current pose.
    pub fn mesh(&mut self) -> (&[Vertex], &[u32]) {
        let (vertices, indices) =
            generate_peeper_mesh(&self.skeleton, &self.config, self.lid_close);
        self.cached_vertices = vertices;
        self.cached_indices = indices;
        (&self.cached_vertices, &self.cached_indices)
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
