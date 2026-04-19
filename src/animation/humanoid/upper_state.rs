//! Upper-body pose state machine (arms + torso overlay).
//!
//! `UpperState` owns the `hands` and `shoulder_twist` channels. It runs
//! orthogonally to `PoseState`: the driver samples both and composes the
//! fragments with overlay semantics (UpperState wins on its channels).

use nalgebra::{Point3, Vector3};

use super::gait::GaitCycle;
use super::pose_state::{AirKind, Takeoff};
use super::stride_wheel;
use crate::animation::config::{CharacterRigConfig, GaitPreset};
use crate::animation::pose::{Cycle, CycleKind, HandsPose, PoseFragment};
use crate::animation::state::{AnimationState, HandState};
use crate::physics::{ConstraintHandle, RigidBodyHandle};
use crate::player::grab::GrabConfig;

/// Reference horizontal speed used to scale the LongJump reach-forward
/// magnitude. Chosen as a nominal walk speed — the rig config does not
/// expose one directly today.
const LONG_JUMP_REFERENCE_SPEED: f32 = 3.0;

/// Coarse discriminant used to decide when a crossfade fires. Continuous
/// parameters (`elapsed`, `current_hold_height`, etc.) do not change the
/// key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpperKey {
    Swinging,
    Reaching,
    Holding,
    Braced,
}

/// Upper-body animation state.
#[derive(Debug, Clone, Copy)]
pub enum UpperState {
    /// Normal arm carriage. Coupled to `PoseState::cycle()` — swings on
    /// Stride, hangs at rest otherwise.
    Swinging,
    /// Right hand animating toward a grab target.
    Reaching {
        elapsed: f32,
        target: Option<(RigidBodyHandle, Point3<f32>)>,
    },
    /// Right hand locked to a held object via `constraint`.
    Holding {
        target_body: RigidBodyHandle,
        constraint: ConstraintHandle,
        current_hold_height: f32,
    },
    /// Arms spread outward and down (falling / landing).
    Braced,
}

/// Per-frame tick inputs for `UpperState`.
pub struct UpperTickCtx {
    pub dt: f32,
}

/// Per-frame sample inputs for `UpperState`.
pub struct UpperSampleCtx<'a> {
    pub rig: &'a CharacterRigConfig,
    pub anim: &'a AnimationState,
    pub grab: &'a GrabConfig,
    /// The cycle `PoseState` exposes this frame, if any.
    pub cycle: Option<Cycle>,
    /// Active gait preset. `None` when the lower body is airborne — in
    /// that case the upper body reads rig-level defaults.
    pub preset: Option<GaitPreset>,
    /// Visual pelvis offset the lower body is emitting this frame
    /// (e.g. crouch drop). Shoulders and hand targets are computed against
    /// the offset pelvis so the upper body tracks a crouched torso.
    pub pelvis_offset: Vector3<f32>,
    /// Forward torso lean in radians (positive = hunched forward). Used to
    /// tilt the shoulder frame so hand targets sit in the pitched torso
    /// frame.
    pub torso_pitch: f32,
    /// Airborne context (kind + takeoff) when `PoseState` is in
    /// `Launching` or `Airborne`. Drives per-AirKind hand variations in
    /// `Braced` (e.g. LongJump reach-forward).
    pub airborne: Option<(AirKind, Takeoff)>,
}

impl UpperSampleCtx<'_> {
    fn arm_swing_amplitude(&self) -> f32 {
        self.preset
            .map(|p| p.arm_swing_amplitude)
            .unwrap_or(self.rig.arm_swing_amplitude)
    }

    fn shoulder_twist_max(&self) -> f32 {
        self.preset
            .map(|p| p.shoulder_twist_max)
            .unwrap_or(self.rig.shoulder_twist_max)
    }

    fn arm_gait(&self) -> GaitCycle {
        GaitCycle::arm_swing(
            self.rig.arm_length(),
            self.arm_swing_amplitude(),
            self.rig.shoulder_width,
        )
    }
}

impl UpperState {
    /// Advance any FSM-internal timers. `Reaching::elapsed` is authored
    /// by the grab system on `PlayerState` — the driver re-syncs each
    /// frame — so no variant currently owns a timer here.
    #[allow(clippy::needless_pass_by_value)]
    pub fn tick(self, _ctx: &UpperTickCtx) -> Self {
        self
    }

    /// Coarse key used to gate crossfade triggers.
    pub fn transition_key(&self) -> UpperKey {
        match self {
            UpperState::Swinging => UpperKey::Swinging,
            UpperState::Reaching { .. } => UpperKey::Reaching,
            UpperState::Holding { .. } => UpperKey::Holding,
            UpperState::Braced => UpperKey::Braced,
        }
    }

    /// Emit the upper-body fragment (hands + shoulder twist).
    pub fn sample(&self, ctx: &UpperSampleCtx<'_>) -> PoseFragment {
        match self {
            UpperState::Swinging => sample_swinging(ctx),
            UpperState::Braced => sample_braced(ctx),
            UpperState::Reaching { elapsed, target } => sample_reaching(ctx, *elapsed, *target),
            UpperState::Holding {
                current_hold_height,
                ..
            } => sample_holding(ctx, *current_hold_height),
        }
    }
}

/// Shoulder positions for a twist-aware rig.
struct Shoulders {
    left: Point3<f32>,
    right: Point3<f32>,
}

fn shoulders_with_twist(ctx: &UpperSampleCtx<'_>, twist: f32) -> Shoulders {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let facing = anim.facing;
    let right = facing.cross(&Vector3::y());
    let left = -right;

    // Pelvis origin for the torso stack, shifted by any visual crouch offset.
    let pelvis = anim.pelvis_position + ctx.pelvis_offset;

    // Rotate the torso-local up axis forward by `torso_pitch` around the
    // lateral (right) axis. The chest sits on the pitched up-axis.
    let pitch = ctx.torso_pitch;
    let torso_up = Vector3::y() * pitch.cos() + facing * pitch.sin();
    let chest = pelvis + torso_up * rig.torso_height;

    let cos_twist = twist.cos();
    let sin_twist = twist.sin();
    let left_offset = left * cos_twist + facing * sin_twist;
    let right_offset = right * cos_twist - facing * sin_twist;

    Shoulders {
        left: chest + left_offset * rig.shoulder_width,
        right: chest + right_offset * rig.shoulder_width,
    }
}

/// Natural hand carriage when the rig is not grabbing — either swinging
/// opposite to the legs (stride cycle active) or hanging at rest.
fn natural_hands(ctx: &UpperSampleCtx<'_>, shoulders: &Shoulders) -> HandsPose {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let facing = anim.facing;

    match ctx.cycle {
        Some(Cycle {
            phase,
            kind: CycleKind::Stride,
        }) => {
            let arm_gait = ctx.arm_gait();
            let mut left_hand = HandState::new(anim.left_hand.position);
            let mut right_hand = HandState::new(anim.right_hand.position);
            // Arms swing OPPOSITE to legs for counter-balance.
            stride_wheel::update_hand(
                &mut left_hand,
                &arm_gait,
                phase,
                stride_wheel::RIGHT_PHASE,
                shoulders.left,
                facing,
                -1.0,
            );
            stride_wheel::update_hand(
                &mut right_hand,
                &arm_gait,
                phase,
                stride_wheel::LEFT_PHASE,
                shoulders.right,
                facing,
                1.0,
            );
            HandsPose {
                left: left_hand.position,
                right: right_hand.position,
            }
        }
        _ => {
            let arm_hang = rig.arm_length();
            HandsPose {
                left: shoulders.left - Vector3::y() * arm_hang,
                right: shoulders.right - Vector3::y() * arm_hang,
            }
        }
    }
}

fn sample_swinging(ctx: &UpperSampleCtx<'_>) -> PoseFragment {
    let twist = match ctx.cycle {
        Some(Cycle {
            phase,
            kind: CycleKind::Stride,
        }) => stride_wheel::compute_shoulder_twist(phase, ctx.shoulder_twist_max()),
        _ => 0.0,
    };
    let shoulders = shoulders_with_twist(ctx, twist);
    let hands = natural_hands(ctx, &shoulders);

    PoseFragment {
        feet: None,
        hands: Some(hands),
        pelvis_offset: None,
        shoulder_twist: Some(twist),
        head_tilt: None,
        head_bob: None,
        torso_pitch: None,
    }
}

fn sample_braced(ctx: &UpperSampleCtx<'_>) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let facing = anim.facing;
    let right = facing.cross(&Vector3::y());
    let left = -right;

    let shoulders = shoulders_with_twist(ctx, 0.0);
    let arm_hang = rig.arm_length() * 0.9;
    let arm_reach = rig.arm_length();

    let hands = match ctx.airborne.map(|(k, t)| (k, t)) {
        Some((AirKind::LongJump, takeoff)) => {
            // Reach both hands forward along the latched facing, scaled by
            // takeoff speed. Small up-bias keeps them above belt line.
            let forward = Vector3::new(takeoff.facing.x, 0.0, takeoff.facing.z)
                .try_normalize(1e-4)
                .unwrap_or(facing);
            let scale = (takeoff.air_speed / LONG_JUMP_REFERENCE_SPEED).clamp(0.8, 1.4);
            let reach = forward * (arm_reach * 0.75 * scale) + Vector3::y() * 0.05;
            HandsPose {
                left: shoulders.left + reach,
                right: shoulders.right + reach,
            }
        }
        Some((AirKind::Jump, _)) => {
            // Arms slightly forward and up for a deliberate hop.
            let forward_bias = facing * (arm_reach * 0.25);
            let up_bias = Vector3::y() * (arm_reach * 0.15);
            HandsPose {
                left: shoulders.left + left * 0.1 - Vector3::y() * arm_hang
                    + forward_bias
                    + up_bias,
                right: shoulders.right + right * 0.1 - Vector3::y() * arm_hang
                    + forward_bias
                    + up_bias,
            }
        }
        _ => HandsPose {
            left: shoulders.left + left * 0.1 - Vector3::y() * arm_hang,
            right: shoulders.right + right * 0.1 - Vector3::y() * arm_hang,
        },
    };

    PoseFragment {
        feet: None,
        hands: Some(hands),
        pelvis_offset: None,
        shoulder_twist: Some(0.0),
        head_tilt: None,
        head_bob: None,
        torso_pitch: None,
    }
}

/// Clamp a hand target to the arm's maximum reach from its shoulder.
fn clamp_to_reach(shoulder: Point3<f32>, target: Point3<f32>, max_reach: f32) -> Point3<f32> {
    let to_target = target - shoulder;
    let dist = to_target.magnitude();
    if dist > max_reach && dist > 1e-6 {
        shoulder + to_target * (max_reach / dist)
    } else {
        target
    }
}

fn sample_reaching(
    ctx: &UpperSampleCtx<'_>,
    elapsed: f32,
    target: Option<(RigidBodyHandle, Point3<f32>)>,
) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let grab = ctx.grab;

    // Shoulder twist and natural carriage continue (left hand keeps
    // swinging while the right hand reaches).
    let twist = match ctx.cycle {
        Some(Cycle {
            phase,
            kind: CycleKind::Stride,
        }) => stride_wheel::compute_shoulder_twist(phase, ctx.shoulder_twist_max()),
        _ => 0.0,
    };
    let shoulders = shoulders_with_twist(ctx, twist);
    let natural = natural_hands(ctx, &shoulders);

    let facing = anim.facing;
    // Reach target is a world-space grab target — stays in world frame so
    // the hand actually reaches the object regardless of crouch.
    let reach_height = target
        .map(|(_body, hit)| hit.y - anim.pelvis_position.y)
        .unwrap_or(0.0);
    let reach_target =
        anim.pelvis_position + facing * grab.hold_distance + Vector3::y() * reach_height;
    let rest_hand = anim.pelvis_position + ctx.pelvis_offset + Vector3::y() * 0.1;
    let t = (elapsed / grab.reach_duration).min(1.0);
    let right_target = Point3::from(rest_hand.coords.lerp(&reach_target.coords, t));
    let right = clamp_to_reach(shoulders.right, right_target, rig.arm_length());

    PoseFragment {
        feet: None,
        hands: Some(HandsPose {
            left: natural.left,
            right,
        }),
        pelvis_offset: None,
        shoulder_twist: Some(twist),
        head_tilt: None,
        head_bob: None,
        torso_pitch: None,
    }
}

fn sample_holding(ctx: &UpperSampleCtx<'_>, current_hold_height: f32) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let grab = ctx.grab;

    let twist = match ctx.cycle {
        Some(Cycle {
            phase,
            kind: CycleKind::Stride,
        }) => stride_wheel::compute_shoulder_twist(phase, ctx.shoulder_twist_max()),
        _ => 0.0,
    };
    let shoulders = shoulders_with_twist(ctx, twist);
    let natural = natural_hands(ctx, &shoulders);

    let hold_point = anim.pelvis_position
        + anim.facing * grab.hold_distance
        + Vector3::y() * current_hold_height;
    let right = clamp_to_reach(shoulders.right, hold_point, rig.arm_length());

    PoseFragment {
        feet: None,
        hands: Some(HandsPose {
            left: natural.left,
            right,
        }),
        pelvis_offset: None,
        shoulder_twist: Some(twist),
        head_tilt: None,
        head_bob: None,
        torso_pitch: None,
    }
}
