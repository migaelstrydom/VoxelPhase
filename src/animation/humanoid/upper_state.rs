//! Upper-body pose state machine (arms + torso overlay).
//!
//! `UpperState` owns the `hands` and `shoulder_twist` channels. It runs
//! orthogonally to `PoseState`: the driver samples both and composes the
//! fragments with overlay semantics (UpperState wins on its channels).

use std::f32::consts::PI;

use nalgebra::{Point3, Vector3};

use super::body_frame::BodyFrame;
use super::gait::GaitCycle;
use super::pose_state::{smoothstep, AirKind, Stroke, Takeoff};
use super::stride_sync;
use crate::animation::config::{CharacterRigConfig, GaitPreset};
use crate::animation::pose::{Cycle, CycleKind, HandsPose, PoseFragment};
use crate::animation::state::{AnimationState, HandState};
use crate::character::grab::GrabConfig;
use crate::physics::{ConstraintHandle, RigidBodyHandle};

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
    Stroking,
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
    /// Swimming: a front crawl, or sculling to tread water. Coupled to the
    /// `Stroke` cycle `PoseState::Swimming` exposes.
    Stroking,
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
    /// The stroke the lower body is swimming, when it is.
    pub stroke: Option<Stroke>,
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
    /// by the grab system on `CharacterState` — the driver re-syncs each
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
            UpperState::Stroking => UpperKey::Stroking,
        }
    }

    /// Emit the upper-body fragment (hands + shoulder twist).
    pub fn sample(&self, ctx: &UpperSampleCtx<'_>) -> PoseFragment {
        match self {
            UpperState::Swinging => sample_swinging(ctx),
            UpperState::Braced => sample_braced(ctx),
            UpperState::Stroking => sample_stroking(ctx),
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

/// The torso's frame this frame: the body's own, leaned by `torso_pitch`.
fn torso_frame(ctx: &UpperSampleCtx<'_>) -> BodyFrame {
    BodyFrame::new(ctx.anim.facing, ctx.anim.body_pitch).leaned(ctx.torso_pitch)
}

fn shoulders_with_twist(ctx: &UpperSampleCtx<'_>, twist: f32) -> Shoulders {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let torso = torso_frame(ctx);
    let (right, left, facing) = (torso.right, torso.left(), torso.front);

    // Pelvis origin for the torso stack, shifted by any visual crouch offset.
    let pelvis = anim.pelvis_position + ctx.pelvis_offset;
    let chest = pelvis + torso.up * rig.torso_height;

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

    let arm_hang = rig.arm_length();
    let rest_left = shoulders.left - Vector3::y() * arm_hang;
    let rest_right = shoulders.right - Vector3::y() * arm_hang;

    match ctx.cycle {
        Some(Cycle {
            phase,
            kind: CycleKind::Stride,
        }) => {
            let arm_gait = ctx.arm_gait();
            let mut left_hand = HandState::new(rest_left);
            let mut right_hand = HandState::new(rest_right);
            // Arms swing OPPOSITE to legs for counter-balance.
            stride_sync::update_hand(
                &mut left_hand,
                &arm_gait,
                phase,
                stride_sync::RIGHT_PHASE,
                shoulders.left,
                facing,
                -1.0,
            );
            stride_sync::update_hand(
                &mut right_hand,
                &arm_gait,
                phase,
                stride_sync::LEFT_PHASE,
                shoulders.right,
                facing,
                1.0,
            );
            // Blend between rest-hang and the swinging target by
            // `stride_activity`. The gait FSM no longer gates whether
            // arms swing — arms follow the feet, amplitude fades in and
            // out with the step cadence.
            let a = anim.stride_activity.clamp(0.0, 1.0);
            wading_carriage(
                ctx,
                shoulders,
                HandsPose {
                    left: Point3::from(rest_left.coords.lerp(&left_hand.position.coords, a)),
                    right: Point3::from(rest_right.coords.lerp(&right_hand.position.coords, a)),
                },
            )
        }
        _ => wading_carriage(
            ctx,
            shoulders,
            HandsPose {
                left: rest_left,
                right: rest_right,
            },
        ),
    }
}

/// Hands lifted clear of the water a character is wading through.
///
/// Nobody wading chest deep lets their arms hang in it: they come up and out
/// to the sides, forearms riding just over the surface, still swinging a
/// little with the stride. Blended in by wade depth, so a paddle at the
/// water's edge leaves the arms alone and a chest-deep wade holds them up.
fn wading_carriage(ctx: &UpperSampleCtx<'_>, shoulders: &Shoulders, dry: HandsPose) -> HandsPose {
    let anim = ctx.anim;
    let lift = smoothstep(WADE_ARMS_FROM, WADE_ARMS_FULL, anim.wade);
    let Some(surface) = anim.water_surface.filter(|_| lift > 0.0) else {
        return dry;
    };
    let torso = torso_frame(ctx);
    let reach = ctx.rig.arm_length();
    let carried = |shoulder: Point3<f32>, hand: Point3<f32>, side: Vector3<f32>| {
        // Out to the side and a little ahead, keeping the stride's swing.
        let swing = (hand - shoulder).dot(&torso.front);
        let mut held = shoulder + side * (reach * 0.55) + torso.front * (reach * 0.3 + swing * 0.4);
        held.y = held
            .y
            .min(shoulder.y - reach * 0.2)
            .max(surface + WADE_HAND_CLEARANCE);
        Point3::from(hand.coords.lerp(&held.coords, lift))
    };
    HandsPose {
        left: carried(shoulders.left, dry.left, torso.left()),
        right: carried(shoulders.right, dry.right, torso.right),
    }
}

/// Wade depth at which the arms start to come up out of the water.
const WADE_ARMS_FROM: f32 = 0.3;
/// Wade depth by which they are all the way up.
const WADE_ARMS_FULL: f32 = 0.75;
/// How far over the surface a wader holds its hands, in metres.
const WADE_HAND_CLEARANCE: f32 = 0.04;

fn sample_swinging(ctx: &UpperSampleCtx<'_>) -> PoseFragment {
    let twist = match ctx.cycle {
        Some(Cycle {
            phase,
            kind: CycleKind::Stride,
        }) => {
            stride_sync::compute_shoulder_twist(phase, ctx.shoulder_twist_max())
                * ctx.anim.stride_activity.clamp(0.0, 1.0)
        }
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
        }) => {
            stride_sync::compute_shoulder_twist(phase, ctx.shoulder_twist_max())
                * ctx.anim.stride_activity.clamp(0.0, 1.0)
        }
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
        }) => {
            stride_sync::compute_shoulder_twist(phase, ctx.shoulder_twist_max())
                * ctx.anim.stride_activity.clamp(0.0, 1.0)
        }
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

/// Swimming arms, in the torso's frame, so they stroke along the body however
/// far it lies over.
///
/// ```text
///   crawl, one arm (the other half a cycle behind):
///
///        recovery: out of the water, elbow high
///      ╭──────────────────────────────╮
///   hip                             overhead ── catch
///      ╰──────────────────────────────╯
///        pull: under the belly, back to the hip
/// ```
///
/// Treading, both hands scull in front of the chest. A body that has just
/// fallen in throws its arms overhead as it goes under, then sweeps them down
/// to its sides to haul itself back up (`WaterEntry`).
fn sample_stroking(ctx: &UpperSampleCtx<'_>) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let phase = match ctx.cycle {
        Some(Cycle {
            phase,
            kind: CycleKind::Stroke,
        }) => phase,
        _ => anim.stroke_phase,
    };
    let stroke = ctx.stroke.unwrap_or(Stroke::Tread);
    let reach = rig.arm_length();

    // The crawl rolls the shoulders into each pull.
    let twist = match stroke {
        Stroke::Crawl => SWIM_ROLL * phase.cos(),
        Stroke::Tread => 0.0,
    };
    let shoulders = shoulders_with_twist(ctx, twist);
    let torso = torso_frame(ctx);

    let hand = |shoulder: Point3<f32>, side: Vector3<f32>, arm_phase: f32| -> Point3<f32> {
        match stroke {
            Stroke::Crawl => {
                let arm_phase = arm_phase.rem_euclid(2.0 * PI);
                let (along, below, out) = if arm_phase < PI {
                    // Pull: from the catch overhead, under the body, to the hip.
                    let s = arm_phase / PI;
                    let along = CRAWL_CATCH + (CRAWL_FINISH - CRAWL_CATCH) * s;
                    (along, CRAWL_PULL_DEPTH * (PI * s).sin(), 0.05)
                } else {
                    // Recovery: from the hip, up over the water, to the catch.
                    let u = (arm_phase - PI) / PI;
                    let along = CRAWL_FINISH + (CRAWL_CATCH - CRAWL_FINISH) * u;
                    (
                        along,
                        -CRAWL_RECOVERY_LIFT * (PI * u).sin(),
                        0.05 + 0.2 * (PI * u).sin(),
                    )
                };
                shoulder
                    + torso.up * (reach * along)
                    + torso.front * (reach * below)
                    + side * (reach * out)
            }
            Stroke::Tread => {
                // A flat figure-of-eight: hands sweep in and out together.
                let sweep = arm_phase.sin();
                shoulder + side * (reach * (0.45 + 0.15 * sweep)) + torso.front * (reach * 0.4)
                    - torso.up * (reach * (0.3 + 0.05 * (2.0 * arm_phase).cos()))
            }
        }
    };
    let (left_phase, right_phase) = match stroke {
        Stroke::Crawl => (phase, phase + PI),
        Stroke::Tread => (phase, phase),
    };
    let mut left = hand(shoulders.left, torso.left(), left_phase);
    let mut right = hand(shoulders.right, torso.right, right_phase);

    // Fallen in: arms thrown up overhead, then swept down to the sides.
    let (plunge, surge) = anim.water_entry.map_or((0.0, 0.0), |entry| {
        let p = entry.progress();
        let up = smoothstep(0.0, 0.12, p) * (1.0 - smoothstep(0.3, 0.55, p));
        let down = smoothstep(0.35, 0.55, p) * (1.0 - smoothstep(0.7, 1.0, p));
        (entry.strength * up, entry.strength * down)
    });
    let overhead = |shoulder: Point3<f32>, side: Vector3<f32>| {
        shoulder + Vector3::y() * (reach * 0.9) + side * (reach * 0.25)
    };
    let pressed = |shoulder: Point3<f32>, side: Vector3<f32>| {
        shoulder - Vector3::y() * (reach * 0.7) + side * (reach * 0.35)
    };
    for (hand, shoulder, side) in [
        (&mut left, shoulders.left, torso.left()),
        (&mut right, shoulders.right, torso.right),
    ] {
        *hand = Point3::from(hand.coords.lerp(&overhead(shoulder, side).coords, plunge));
        *hand = Point3::from(hand.coords.lerp(&pressed(shoulder, side).coords, surge));
    }

    PoseFragment {
        feet: None,
        hands: Some(HandsPose {
            left: clamp_to_reach(shoulders.left, left, reach),
            right: clamp_to_reach(shoulders.right, right, reach),
        }),
        pelvis_offset: None,
        shoulder_twist: Some(twist),
        head_tilt: None,
        head_bob: None,
        torso_pitch: None,
    }
}

/// How far the crawl rolls the shoulders into each pull, in radians.
const SWIM_ROLL: f32 = 0.35;
/// Where a crawling hand enters the water, up the body from its shoulder, in
/// arm lengths.
const CRAWL_CATCH: f32 = 0.95;
/// Where the pull finishes, down by the hip, in arm lengths.
const CRAWL_FINISH: f32 = -0.75;
/// How deep under the body the pull reaches, in arm lengths.
const CRAWL_PULL_DEPTH: f32 = 0.45;
/// How high over the back the recovery lifts the hand, in arm lengths.
const CRAWL_RECOVERY_LIFT: f32 = 0.35;
