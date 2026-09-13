//! The heart critter's animation driver.
//!
//! ```text
//!   Grounding ─┐
//!   intent ────┼─► LeggedLocomotion ─► feet ─┐
//!   velocity ──┘        (shared)             ├─► CritterSkeleton ─► mesh
//!   pelvis, yaw ─────────────────────────────┘
//! ```
//!
//! Thin by design. Everything hard about walking is in
//! [`crate::animation::LeggedLocomotion`], which the humanoid uses too;
//! what is left here is a body to hang off the feet and a rule for what
//! the legs do when there is no ground under them.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

use crate::animation::rig::right_vector;
use crate::animation::{FootSide, LeggedLocomotion, LocomotionCtx};
use crate::character::{CharacterIntent, Grounding};
use crate::rendering::vertex::Vertex;
use crate::sensing::{ContactCandidate, Probe};

use super::config::CritterRigConfig;
use super::mesh::generate_critter_mesh;
use super::skeleton::{CritterSkeleton, FootPose};

/// Exponential rate (per second) at which the legs tuck when the ground
/// leaves and untuck when it comes back.
const TUCK_RATE: f32 = 14.0;

/// How far under the hips a tucked foot rides, as a fraction of standing
/// height. Well under 1, so an airborne critter is visibly bunched up.
const TUCK_HEIGHT_FRACTION: f32 = 0.5;

/// A heart critter's animation state.
#[derive(Component)]
#[storage(VecStorage)]
pub struct CritterAnimator {
    pub config: CritterRigConfig,
    /// Feet, probes and the support frame. The same component the player
    /// walks on.
    pub locomotion: LeggedLocomotion,
    pub skeleton: CritterSkeleton,

    /// How tucked the legs are, in `[0, 1]`. Chases grounding rather than
    /// snapping to it, so a critter skipping over a gap does not flick
    /// between stances.
    tuck: f32,

    cached_vertices: Vec<Vertex>,
    cached_indices: Vec<u32>,
}

impl CritterAnimator {
    /// A critter whose physics body sits at `body_position`, riding
    /// `ground_clearance` above the surface it rests on.
    pub fn new(
        config: CritterRigConfig,
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
        let skeleton = CritterSkeleton::new(&config, pelvis, facing_from(yaw));

        Self {
            config,
            locomotion,
            skeleton,
            tuck: 0.0,
            cached_vertices: Vec::new(),
            cached_indices: Vec::new(),
        }
    }

    /// Where the rig's pelvis belongs, given where the physics body is.
    pub fn pelvis_for(&self, body_position: Point3<f32>) -> Point3<f32> {
        self.locomotion.pelvis_for(body_position)
    }

    /// Aim the foot probes for the coming frame. While the placer is
    /// suspended it has no anchors to offer, so they aim at the feet
    /// currently being drawn.
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
    ) {
        // A gait means something only in the frame of whatever is holding
        // the body up: a critter riding a platform at 3 m/s is standing
        // still and its feet should behave that way.
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

        let target = if airborne { 1.0 } else { 0.0 };
        self.tuck += (target - self.tuck) * (1.0 - (-TUCK_RATE * dt).exp());

        let facing = facing_from(yaw);
        let feet = self.foot_poses(pelvis, facing);
        self.skeleton.update(dt, &self.config, pelvis, facing, feet);
    }

    /// Where each foot should be drawn: where the gait put it, pulled
    /// toward a tucked stance under the hip by however airborne the
    /// critter is.
    ///
    /// Without the tuck an airborne critter's feet stay at the world
    /// positions they were planted at, and the legs are drawn trailing at
    /// full stretch behind a body that has flown off without them.
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
        let (vertices, indices) = generate_critter_mesh(&self.skeleton, &self.config);
        self.cached_vertices = vertices;
        self.cached_indices = indices;
        (&self.cached_vertices, &self.cached_indices)
    }
}

#[inline]
fn facing_from(yaw: f32) -> Vector3<f32> {
    Vector3::new(yaw.sin(), 0.0, yaw.cos())
}
