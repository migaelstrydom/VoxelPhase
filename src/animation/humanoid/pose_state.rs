//! Lower-body / core pose state machine.
//!
//! `PoseState` is the authoritative FSM for feet, pelvis, head-tilt, and
//! head-bob. It reads `PlayerState` via the driver (see
//! `CharacterAnimator::update`) and emits a `PoseFragment`. Upper-body
//! channels (hands, shoulder twist) are produced by `UpperState`.

use nalgebra::{Point3, Vector2, Vector3};

use super::gait::GaitCycle;
use super::stride_wheel;
use crate::animation::config::{CharacterRigConfig, GaitPreset};
use crate::animation::pose::{Cycle, CycleKind, FeetPose, PoseFragment};
use crate::animation::state::AnimationState;

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

/// `Gait` reduced to its crossfade-triggering identity. The `walking`
/// flag on `Crouch` is a parameter, not a state, so it collapses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GaitKey {
    Idle,
    Walk,
    Sprint,
    Crouch,
}

impl Gait {
    fn key(self) -> GaitKey {
        match self {
            Gait::Idle => GaitKey::Idle,
            Gait::Walk => GaitKey::Walk,
            Gait::Sprint => GaitKey::Sprint,
            Gait::Crouch { .. } => GaitKey::Crouch,
        }
    }
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

/// Coarse discriminant used to decide when a crossfade fires. Two
/// `PoseState`s with equal keys are treated as the same pose for
/// transition purposes; continuous parameters (`t`, `takeoff`) are
/// intentionally ignored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoseKey {
    Grounded(GaitKey),
    Launching(AirKind),
    Airborne(AirKind),
    Landing(AirKind),
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
            PoseState::Grounded { gait: Gait::Idle } => sample_idle(ctx, None),
            PoseState::Grounded { gait } => {
                let preset = ctx.rig.gait_presets.for_gait(*gait);
                match gait {
                    Gait::Crouch { walking: false } => sample_idle(ctx, Some(&preset)),
                    _ => sample_walking(ctx, &preset),
                }
            }
            PoseState::Launching { .. }
            | PoseState::Airborne { .. }
            | PoseState::Landing { .. } => sample_airborne(ctx),
        }
    }

    /// Coarse key used to gate crossfade triggers. Changes in this key
    /// are what fire a blend; changes in continuous params (e.g. a
    /// Launching timer) are not.
    pub fn transition_key(&self) -> PoseKey {
        match *self {
            PoseState::Grounded { gait } => PoseKey::Grounded(gait.key()),
            PoseState::Launching { kind, .. } => PoseKey::Launching(kind),
            PoseState::Airborne { kind, .. } => PoseKey::Airborne(kind),
            PoseState::Landing { kind, .. } => PoseKey::Landing(kind),
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

fn sample_idle(ctx: &SampleCtx<'_>, preset: Option<&GaitPreset>) -> PoseFragment {
    let anim = ctx.anim;

    // Feet snap to the planted position. The driver re-plants (with a
    // small hysteresis threshold) before sampling.
    let feet = FeetPose {
        left: anim.left.planted_position,
        right: anim.right.planted_position,
    };

    let pelvis_offset = preset
        .map(|p| p.pelvis_crouch_offset)
        .filter(|v| *v != 0.0)
        .map(|v| Vector3::new(0.0, -v, 0.0));

    let torso_pitch = preset.map(|p| p.torso_pitch).unwrap_or(0.0);

    PoseFragment {
        feet: Some(feet),
        hands: None,
        pelvis_offset,
        shoulder_twist: None,
        head_tilt: Some(Vector2::new(0.0, 0.0)),
        head_bob: Some(0.0),
        torso_pitch: Some(torso_pitch),
    }
}

fn sample_walking(ctx: &SampleCtx<'_>, preset: &GaitPreset) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let facing = anim.facing;
    let wheel_angle = anim.wheel_angle;

    let right = facing.cross(&Vector3::y());
    let left_hip = anim.pelvis_position - right * rig.hip_width;
    let right_hip = anim.pelvis_position + right * rig.hip_width;

    let leg_gait = GaitCycle::walking(
        rig.standing_height(),
        preset.stride_length,
        preset.step_height,
    );

    let mut left_foot = anim.left.clone();
    let mut right_foot = anim.right.clone();
    stride_wheel::update_foot(
        &mut left_foot,
        &leg_gait,
        wheel_angle,
        stride_wheel::LEFT_PHASE,
        Point3::from(left_hip.coords),
        facing,
        -1.0,
    );
    stride_wheel::update_foot(
        &mut right_foot,
        &leg_gait,
        wheel_angle,
        stride_wheel::RIGHT_PHASE,
        Point3::from(right_hip.coords),
        facing,
        1.0,
    );

    let head_tilt = stride_wheel::compute_head_tilt(ctx.velocity, facing, rig.head_tilt_factor);
    let head_bob = stride_wheel::compute_head_bob(wheel_angle, preset.head_bob_amplitude);

    let pelvis_offset = if preset.pelvis_crouch_offset != 0.0 {
        Some(Vector3::new(0.0, -preset.pelvis_crouch_offset, 0.0))
    } else {
        None
    };

    PoseFragment {
        feet: Some(FeetPose {
            left: left_foot.position,
            right: right_foot.position,
        }),
        hands: None,
        pelvis_offset,
        shoulder_twist: None,
        head_tilt: Some(head_tilt),
        head_bob: Some(head_bob),
        torso_pitch: Some(preset.torso_pitch),
    }
}

fn sample_airborne(ctx: &SampleCtx<'_>) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let right = anim.facing.cross(&Vector3::y());

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

    PoseFragment {
        feet: Some(feet),
        hands: None,
        pelvis_offset: None,
        shoulder_twist: None,
        head_tilt: Some(Vector2::new(-0.05, 0.0)),
        head_bob: Some(0.0),
        torso_pitch: Some(0.0),
    }
}
