//! Lower-body / core pose state machine.
//!
//! `PoseState` is the authoritative FSM for feet, pelvis, head, and the
//! shoulder-twist overlay. It reads `PlayerState` via the driver (see
//! `CharacterAnimator::update`) and emits a `PoseFragment`.
//!
//! Upper-body hand/arm channels are also populated here for now; a
//! future `UpperState` FSM will take over those channels.

use nalgebra::{Point3, Vector2, Vector3};

use super::gait::GaitCycle;
use super::stride_wheel;
use crate::animation::config::CharacterRigConfig;
use crate::animation::pose::{Cycle, CycleKind, FeetPose, HandsPose, PoseFragment};
use crate::animation::state::{AnimationState, HandState};

/// Gait preset inside `PoseState::Grounded`.
///
/// Only `Idle` is distinguished today — `Walk`/`Sprint`/`Crouch` all
/// produce the walking pose. Per-preset parameters attach later.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Gait {
    Idle,
    Walk,
    Sprint,
    Crouch { walking: bool },
}

/// Kind of airborne motion. All three currently share one "falling" pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AirKind {
    Jump,
    LongJump,
    Fall,
}

/// Snapshot of facing and planar speed at liftoff. Latched on
/// `Grounded → Launching` so the airborne pose does not jitter as live
/// velocity changes.
#[derive(Debug, Clone, Copy)]
pub struct Takeoff {
    pub facing: Vector3<f32>,
    pub air_speed: f32,
}

impl Default for Takeoff {
    fn default() -> Self {
        Self {
            facing: Vector3::new(0.0, 0.0, 1.0),
            air_speed: 0.0,
        }
    }
}

/// Lower-body / core animation state.
#[derive(Debug, Clone, Copy)]
pub enum PoseState {
    Grounded {
        gait: Gait,
    },
    Launching {
        kind: AirKind,
        takeoff: Takeoff,
        t: f32,
    },
    Airborne {
        kind: AirKind,
        takeoff: Takeoff,
    },
    Landing {
        kind: AirKind,
        t: f32,
    },
}

/// Per-frame tick inputs. `tick` is currently a no-op; the struct exists
/// so the FSM signature is stable before Launching/Landing timers are
/// wired in.
#[allow(dead_code)]
pub struct TickCtx<'a> {
    pub dt: f32,
    pub velocity: Vector3<f32>,
    pub horizontal_speed: f32,
    pub rig: &'a CharacterRigConfig,
}

/// Per-frame sample inputs.
pub struct SampleCtx<'a> {
    pub rig: &'a CharacterRigConfig,
    pub anim: &'a AnimationState,
    pub velocity: Vector3<f32>,
    pub leg_gait: &'a GaitCycle,
    pub arm_gait: &'a GaitCycle,
}

impl PoseState {
    /// Advance any FSM-internal timers. No variant currently owns a timer
    /// — states live or die on variant swaps issued by the driver mapping.
    #[allow(clippy::needless_pass_by_value)]
    pub fn tick(self, _ctx: &TickCtx<'_>) -> Self {
        self
    }

    /// Emit the pose fragment this state contributes.
    pub fn sample(&self, ctx: &SampleCtx<'_>) -> PoseFragment {
        match self {
            PoseState::Grounded { gait: Gait::Idle } => sample_idle(ctx),
            PoseState::Grounded { .. } => sample_walking(ctx),
            PoseState::Launching { .. }
            | PoseState::Airborne { .. }
            | PoseState::Landing { .. } => sample_airborne(ctx),
        }
    }

    /// Cycle this state exposes for upper-body synchronisation. Only
    /// populated for non-Idle Grounded states (stride cycle); returns
    /// `None` otherwise.
    pub fn cycle(&self, anim: &AnimationState) -> Option<Cycle> {
        match self {
            PoseState::Grounded { gait: Gait::Idle } => None,
            PoseState::Grounded { .. } => Some(Cycle {
                phase: anim.wheel_angle,
                kind: CycleKind::Stride,
            }),
            _ => None,
        }
    }
}

fn sample_idle(ctx: &SampleCtx<'_>) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let facing = anim.facing;
    let right = facing.cross(&Vector3::y());
    let left = -right;

    let chest = anim.pelvis_position + Vector3::y() * rig.torso_height;
    let arm_hang = rig.arm_length();
    let left_shoulder = chest + left * rig.shoulder_width;
    let right_shoulder = chest + right * rig.shoulder_width;

    let hands = HandsPose {
        left: left_shoulder - Vector3::y() * arm_hang,
        right: right_shoulder - Vector3::y() * arm_hang,
    };

    // Feet snap to the planted position. The driver re-plants (with a
    // small hysteresis threshold) before sampling.
    let feet = FeetPose {
        left: anim.left.planted_position,
        right: anim.right.planted_position,
    };

    PoseFragment {
        feet: Some(feet),
        hands: Some(hands),
        pelvis_offset: None,
        shoulder_twist: Some(0.0),
        head_tilt: Some(Vector2::new(0.0, 0.0)),
        head_bob: Some(0.0),
    }
}

fn sample_walking(ctx: &SampleCtx<'_>) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let facing = anim.facing;
    let wheel_angle = anim.wheel_angle;

    let right = facing.cross(&Vector3::y());
    let left = -right;
    let left_hip = anim.pelvis_position - right * rig.hip_width;
    let right_hip = anim.pelvis_position + right * rig.hip_width;

    let mut left_foot = anim.left.clone();
    let mut right_foot = anim.right.clone();
    stride_wheel::update_foot(
        &mut left_foot,
        ctx.leg_gait,
        wheel_angle,
        stride_wheel::LEFT_PHASE,
        Point3::from(left_hip.coords),
        facing,
        -1.0,
    );
    stride_wheel::update_foot(
        &mut right_foot,
        ctx.leg_gait,
        wheel_angle,
        stride_wheel::RIGHT_PHASE,
        Point3::from(right_hip.coords),
        facing,
        1.0,
    );

    let shoulder_twist = stride_wheel::compute_shoulder_twist(wheel_angle, rig.shoulder_twist_max);

    let chest = anim.pelvis_position + Vector3::y() * rig.torso_height;
    let cos_twist = shoulder_twist.cos();
    let sin_twist = shoulder_twist.sin();
    let left_offset = left * cos_twist + facing * sin_twist;
    let right_offset = right * cos_twist - facing * sin_twist;
    let left_shoulder = chest + left_offset * rig.shoulder_width;
    let right_shoulder = chest + right_offset * rig.shoulder_width;

    // Arms swing OPPOSITE to legs for natural counter-balance.
    let mut left_hand = HandState::new(anim.left_hand.position);
    let mut right_hand = HandState::new(anim.right_hand.position);
    stride_wheel::update_hand(
        &mut left_hand,
        ctx.arm_gait,
        wheel_angle,
        stride_wheel::RIGHT_PHASE,
        left_shoulder,
        facing,
        -1.0,
    );
    stride_wheel::update_hand(
        &mut right_hand,
        ctx.arm_gait,
        wheel_angle,
        stride_wheel::LEFT_PHASE,
        right_shoulder,
        facing,
        1.0,
    );

    let head_tilt = stride_wheel::compute_head_tilt(ctx.velocity, facing, rig.head_tilt_factor);
    let head_bob = stride_wheel::compute_head_bob(wheel_angle, rig.head_bob_amplitude);

    PoseFragment {
        feet: Some(FeetPose {
            left: left_foot.position,
            right: right_foot.position,
        }),
        hands: Some(HandsPose {
            left: left_hand.position,
            right: right_hand.position,
        }),
        pelvis_offset: None,
        shoulder_twist: Some(shoulder_twist),
        head_tilt: Some(head_tilt),
        head_bob: Some(head_bob),
    }
}

fn sample_airborne(ctx: &SampleCtx<'_>) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let facing = anim.facing;
    let right = facing.cross(&Vector3::y());
    let left = -right;

    let left_hip = anim.pelvis_position - right * rig.hip_width;
    let right_hip = anim.pelvis_position + right * rig.hip_width;
    let hang_distance = rig.standing_height();

    let feet = FeetPose {
        left: Point3::new(
            left_hip.x,
            anim.pelvis_position.y - hang_distance,
            left_hip.z,
        ),
        right: Point3::new(
            right_hip.x,
            anim.pelvis_position.y - hang_distance,
            right_hip.z,
        ),
    };

    let chest = anim.pelvis_position + Vector3::y() * rig.torso_height;
    let left_shoulder = chest + left * rig.shoulder_width;
    let right_shoulder = chest + right * rig.shoulder_width;
    let arm_hang = rig.arm_length() * 0.9;

    let hands = HandsPose {
        left: left_shoulder + left * 0.1 - Vector3::y() * arm_hang,
        right: right_shoulder + right * 0.1 - Vector3::y() * arm_hang,
    };

    PoseFragment {
        feet: Some(feet),
        hands: Some(hands),
        pelvis_offset: None,
        shoulder_twist: Some(0.0),
        head_tilt: Some(Vector2::new(-0.05, 0.0)),
        head_bob: Some(0.0),
    }
}
