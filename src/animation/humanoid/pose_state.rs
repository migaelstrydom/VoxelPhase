//! Lower-body / core pose.
//!
//! `PoseState` draws the feet, pelvis, head tilt and head bob, and emits a
//! `PoseFragment`. It is a mapping of the character's locomotion state (see
//! `CharacterAnimator::update`); only a jump's wind-up keeps a clock of its
//! own. Upper-body channels (hands, shoulder twist) are produced by
//! `UpperState`.

use std::f32::consts::PI;

use nalgebra::{Point3, Vector2, Vector3};

use super::body_frame::BodyFrame;
use super::hang;
use super::stride_sync;
use crate::animation::config::{CharacterRigConfig, GaitPreset};
use crate::animation::pose::{Cycle, CycleKind, FeetPose, FootAnchor, PoseFragment};
use crate::animation::state::{AnimationState, LandingSquash, WaterEntry, WaterExit};

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

/// Kind of airborne motion. Drives per-kind anticipation / follow-through
/// durations and pose magnitudes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AirKind {
    Jump,
    LongJump,
    /// A planned hop's running leap (`PoseState::Hop`): no wind-up, as the
    /// body has already left the ground, and a light landing back into the
    /// run.
    Leap,
    Fall,
}

impl AirKind {
    /// Launching (anticipation) duration, seconds. Only `Jump` and
    /// `LongJump` are ever used — walking off a ledge produces `Fall`
    /// which skips Launching entirely, but a default is defined for
    /// completeness.
    pub fn launch_duration(self) -> f32 {
        match self {
            AirKind::Jump => 0.14,
            AirKind::LongJump => 0.16,
            AirKind::Leap | AirKind::Fall => 0.0,
        }
    }

    /// The squash a landing from this kind of flight plays.
    pub fn landing_squash(self) -> LandingSquash {
        LandingSquash::new(self.landing_squash_depth(), self.landing_duration())
    }

    /// Landing (follow-through) duration, seconds.
    fn landing_duration(self) -> f32 {
        match self {
            AirKind::Jump => 0.12,
            AirKind::LongJump => 0.20,
            AirKind::Leap => 0.08,
            AirKind::Fall => 0.15,
        }
    }

    /// Peak pelvis dip during Launching, metres.
    pub fn launch_crouch_depth(self) -> f32 {
        match self {
            AirKind::Jump => 0.10,
            AirKind::LongJump => 0.15,
            AirKind::Leap | AirKind::Fall => 0.0,
        }
    }

    /// Initial pelvis squash depth on landing, metres.
    fn landing_squash_depth(self) -> f32 {
        match self {
            AirKind::Jump => 0.08,
            AirKind::LongJump => 0.14,
            AirKind::Leap => 0.03,
            AirKind::Fall => 0.10,
        }
    }
}

/// How a swimmer is moving its limbs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stroke {
    /// Lying flat and going somewhere: front crawl, flutter kick.
    Crawl,
    /// Upright and staying put: sculling hands, a bicycle kick.
    Tread,
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
    Hop,
    Airborne(AirKind),
    Swimming(Stroke),
    Hanging,
}

/// Lower-body / core animation state.
#[derive(Debug, Clone, Copy)]
pub enum PoseState {
    /// On foot, by gait.
    Grounded { gait: Gait },
    /// A jump's wind-up, played for its own duration from the jump.
    Launching {
        kind: AirKind,
        takeoff: Takeoff,
        t: f32,
    },
    /// A planned hop's leap. The legs are the foot placer's: the feet it
    /// flies onto the landing are drawn where it has them, as on the ground.
    Hop { takeoff: Takeoff },
    /// In the air after a jump's wind-up, or falling: the legs hang from the
    /// hips.
    Airborne { kind: AirKind, takeoff: Takeoff },
    /// In the water with nothing underfoot. The body's own pitch lays the rig
    /// flat or stands it up; this chooses what the limbs do.
    Swimming { stroke: Stroke },
    /// Hands on an edge: hanging under them, or climbing over it
    /// (`hang::sample_hanging`, from `AnimationState::hang`).
    Hanging,
}

/// Per-frame tick inputs.
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
    /// Advance the wind-up's clock, which the driver reads to tell when it
    /// has played out.
    pub fn tick(self, ctx: &TickCtx<'_>) -> Self {
        match self {
            PoseState::Launching { kind, takeoff, t } => PoseState::Launching {
                kind,
                takeoff,
                t: t + ctx.dt,
            },
            other => other,
        }
    }

    /// Whether the wind-up has played out; never, for any other pose.
    pub fn timer_expired(&self) -> bool {
        match *self {
            PoseState::Launching { kind, t, .. } => t >= kind.launch_duration(),
            PoseState::Grounded { .. }
            | PoseState::Hop { .. }
            | PoseState::Airborne { .. }
            | PoseState::Swimming { .. }
            | PoseState::Hanging => false,
        }
    }

    /// The kind of flight this pose is in, if it is in the air: what a
    /// landing from it squashes by.
    pub fn air_kind(&self) -> Option<AirKind> {
        match *self {
            PoseState::Launching { kind, .. } | PoseState::Airborne { kind, .. } => Some(kind),
            PoseState::Hop { .. } => Some(AirKind::Leap),
            PoseState::Grounded { .. } | PoseState::Swimming { .. } | PoseState::Hanging => None,
        }
    }

    /// The takeoff latched by the flight this pose is in, if any.
    pub fn takeoff(&self) -> Option<Takeoff> {
        match *self {
            PoseState::Launching { takeoff, .. }
            | PoseState::Hop { takeoff }
            | PoseState::Airborne { takeoff, .. } => Some(takeoff),
            PoseState::Grounded { .. } | PoseState::Swimming { .. } | PoseState::Hanging => None,
        }
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
            PoseState::Launching { kind, t, .. } => sample_launching(ctx, *kind, *t),
            PoseState::Hop { takeoff } => {
                sample_airborne(ctx, AirKind::Leap, *takeoff, ctx.anim.feet_flown)
            }
            PoseState::Airborne { kind, takeoff } => sample_airborne(ctx, *kind, *takeoff, false),
            PoseState::Swimming { stroke } => sample_swimming(ctx, *stroke),
            PoseState::Hanging => match &ctx.anim.hang {
                Some(hang) => hang::sample_hanging(hang, ctx.anim, ctx.rig),
                None => sample_airborne(ctx, AirKind::Fall, Takeoff::default(), false),
            },
        }
    }

    /// Coarse key used to gate crossfade triggers. Changes in this key
    /// are what fire a blend; changes in continuous params (e.g. a
    /// Launching timer) are not.
    pub fn transition_key(&self) -> PoseKey {
        match *self {
            PoseState::Grounded { gait } => PoseKey::Grounded(gait.key()),
            PoseState::Launching { kind, .. } => PoseKey::Launching(kind),
            PoseState::Hop { .. } => PoseKey::Hop,
            PoseState::Airborne { kind, .. } => PoseKey::Airborne(kind),
            PoseState::Swimming { stroke } => PoseKey::Swimming(stroke),
            PoseState::Hanging => PoseKey::Hanging,
        }
    }

    /// Short comma-free name for the variant, for recorders and reports.
    pub fn tag(&self) -> &'static str {
        match self {
            PoseState::Grounded { gait } => match gait {
                Gait::Idle => "idle",
                Gait::Walk => "walk",
                Gait::Sprint => "sprint",
                Gait::Crouch { walking: false } => "crouch",
                Gait::Crouch { walking: true } => "crouch_walk",
            },
            PoseState::Launching { .. } => "launching",
            PoseState::Hop { .. } => "hop",
            PoseState::Airborne { .. } => "airborne",
            PoseState::Swimming {
                stroke: Stroke::Crawl,
            } => "crawl",
            PoseState::Swimming {
                stroke: Stroke::Tread,
            } => "tread",
            PoseState::Hanging => "hanging",
        }
    }

    /// Cycle this state exposes for upper-body synchronisation.
    /// Populated for all `Grounded` states (including Idle) — the phase
    /// always exists; the upper body scales amplitude by
    /// `AnimationState::stride_activity` so rest-pose falls out of the
    /// same math. A Hop keeps the stride: it flows out of the run and back
    /// into it, its feet swinging as steps do. Other flights have none.
    pub fn cycle(&self, anim: &AnimationState) -> Option<Cycle> {
        match self {
            PoseState::Grounded { .. } | PoseState::Hop { .. } => Some(Cycle {
                phase: anim.stride_phase,
                kind: CycleKind::Stride,
            }),
            PoseState::Swimming { .. } => Some(Cycle {
                phase: anim.stroke_phase,
                kind: CycleKind::Stroke,
            }),
            PoseState::Launching { .. } | PoseState::Airborne { .. } | PoseState::Hanging => None,
        }
    }
}

fn sample_idle(ctx: &SampleCtx<'_>, preset: Option<&GaitPreset>) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;

    // Feet come from `FootPlacer`, which the driver has ticked and
    // mirrored into `AnimationState` before sampling. Idle cases
    // converge to neutral stance via the placer's settle rule.
    let feet = FeetPose {
        left: anim.left.position,
        right: anim.right.position,
        anchor: FootAnchor::World,
    };

    let pelvis_offset = preset
        .map(|p| p.pelvis_crouch_offset)
        .filter(|v| *v != 0.0)
        .map(|v| Vector3::new(0.0, -v, 0.0));

    let torso_pitch = preset.map(|p| p.torso_pitch).unwrap_or(0.0) + wade_lean(anim.wade);

    // Head channels go through the same formulas as sample_walking —
    // `stride_activity` is near zero at true idle (head_bob fades to 0),
    // and `compute_head_tilt` already returns zero for near-zero
    // velocity. On a slope slide, activity stays at 1 while feet step,
    // so head tilt/bob stay continuous instead of flickering.
    let head_bob_amplitude = preset.map(|p| p.head_bob_amplitude).unwrap_or(0.0);
    let head_bob =
        stride_sync::compute_head_bob(anim.stride_phase, head_bob_amplitude) * anim.stride_activity;
    let head_tilt = stride_sync::compute_head_tilt(ctx.velocity, anim.facing, rig.head_tilt_factor);

    PoseFragment {
        feet: Some(feet),
        hands: None,
        pelvis_offset,
        shoulder_twist: None,
        head_tilt: Some(head_tilt),
        head_bob: Some(head_bob),
        torso_pitch: Some(torso_pitch),
    }
}

fn sample_walking(ctx: &SampleCtx<'_>, preset: &GaitPreset) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let facing = anim.facing;
    let stride_phase = anim.stride_phase;

    // Feet come from `FootPlacer` via `AnimationState`; stride xy is
    // no longer derived from the wheel. The wheel still parameterises
    // stylistic channels (head_bob, arm swing).
    let feet = FeetPose {
        left: anim.left.position,
        right: anim.right.position,
        anchor: FootAnchor::World,
    };

    let head_tilt = stride_sync::compute_head_tilt(ctx.velocity, facing, rig.head_tilt_factor);
    let head_bob = stride_sync::compute_head_bob(stride_phase, preset.head_bob_amplitude)
        * anim.stride_activity;

    let pelvis_offset = if preset.pelvis_crouch_offset != 0.0 {
        Some(Vector3::new(0.0, -preset.pelvis_crouch_offset, 0.0))
    } else {
        None
    };

    PoseFragment {
        feet: Some(feet),
        hands: None,
        pelvis_offset,
        shoulder_twist: None,
        head_tilt: Some(head_tilt),
        head_bob: Some(head_bob),
        torso_pitch: Some(preset.torso_pitch + wade_lean(anim.wade)),
    }
}

/// How far a wader leans into the water, in radians, at wade depth `wade`.
/// Pushing a chest through water is done leaning into it.
pub fn wade_lean(wade: f32) -> f32 {
    const MAX_LEAN: f32 = 0.22;
    MAX_LEAN * wade.clamp(0.0, 1.0)
}

/// Airborne pose varies by `AirKind`:
/// - `Jump`: tucked legs, small forward torso lean, neutral head.
/// - `LongJump`: legs slightly extended forward along takeoff facing, body
///   committed forward, head pitched down.
/// - `Leap`: a hop's, legs as `Jump`'s.
/// - `Fall`: dangly legs, slight backward head tilt (arms handled by
///   `UpperState::Braced`).
///
/// With `flown`, the foot placer is swinging the feet onto the flight's
/// landing, and they are drawn where it has them, as on the ground.
fn sample_airborne(
    ctx: &SampleCtx<'_>,
    kind: AirKind,
    takeoff: Takeoff,
    flown: bool,
) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let right = anim.facing.cross(&Vector3::y());
    let left_hip = anim.pelvis_position - right * rig.hip_width;
    let right_hip = anim.pelvis_position + right * rig.hip_width;

    let (hang_factor, feet_forward, torso_pitch, head_tilt) = match kind {
        AirKind::Jump => (0.7, 0.0, 0.08, Vector2::new(0.0, 0.0)),
        AirKind::LongJump => (0.85, 0.18, 0.20, Vector2::new(0.08, 0.0)),
        AirKind::Leap => (0.9, 0.0, 0.12, Vector2::new(0.03, 0.0)),
        AirKind::Fall => (0.85, 0.0, -0.05, Vector2::new(-0.05, 0.0)),
    };

    let hang_distance = rig.standing_height() * hang_factor;
    let forward = match kind {
        AirKind::LongJump => {
            let f = takeoff.facing;
            let planar = Vector3::new(f.x, 0.0, f.z);
            planar
                .try_normalize(1e-4)
                .unwrap_or_else(|| Vector3::new(0.0, 0.0, 1.0))
        }
        _ => Vector3::zeros(),
    };
    let foot_push = forward * feet_forward;

    let feet = if flown {
        FeetPose {
            left: anim.left.position,
            right: anim.right.position,
            anchor: FootAnchor::Flown,
        }
    } else {
        FeetPose {
            left: Point3::new(
                left_hip.x + foot_push.x,
                anim.pelvis_position.y - hang_distance,
                left_hip.z + foot_push.z,
            ),
            right: Point3::new(
                right_hip.x + foot_push.x,
                anim.pelvis_position.y - hang_distance,
                right_hip.z + foot_push.z,
            ),
            anchor: FootAnchor::Hips,
        }
    };

    let fragment = PoseFragment {
        feet: Some(feet),
        hands: None,
        pelvis_offset: None,
        shoulder_twist: None,
        head_tilt: Some(head_tilt),
        head_bob: Some(0.0),
        torso_pitch: Some(torso_pitch),
    };
    match anim.water_exit {
        Some(exit) if !flown => kicked_out(ctx, fragment, &exit),
        _ => fragment,
    }
}

/// The legs and trunk of a jump out of the water over the airborne pose
/// (`WaterExit`): knees drawn up and scissored apart, then kicked down
/// straight and together with the chest lifted and the chin up, held through
/// the surface and let go. Along the body's own axis, so a jump out of a
/// crawl kicks behind it as the body stands up.
fn kicked_out(ctx: &SampleCtx<'_>, mut fragment: PoseFragment, exit: &WaterExit) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let frame = BodyFrame::new(anim.facing, anim.body_pitch);
    let leg = rig.leg_length();
    let (gather, drive) = (exit_gather(exit), exit_drive(exit));
    let pelvis = anim.pelvis_position;

    if let Some(feet) = fragment.feet.as_mut() {
        for (foot, side, scissor) in [
            (&mut feet.left, frame.left(), 1.0),
            (&mut feet.right, frame.right, -1.0),
        ] {
            let hip = pelvis + side * rig.hip_width;
            let tucked =
                hip - frame.up * (leg * 0.6) + frame.front * (KICK_SCISSOR * scissor) + side * 0.05;
            let kicked = pelvis - frame.up * (leg * 0.98) + side * 0.03;
            *foot = Point3::from(foot.coords.lerp(&tucked.coords, gather));
            *foot = Point3::from(foot.coords.lerp(&kicked.coords, drive));
        }
    }
    let lerp = |base: f32, tucked: f32, kicked: f32| {
        let after_gather = base + (tucked - base) * gather;
        after_gather + (kicked - after_gather) * drive
    };
    fragment.torso_pitch = fragment.torso_pitch.map(|p| lerp(p, 0.2, -0.1));
    fragment.head_tilt = fragment
        .head_tilt
        .map(|tilt| Vector2::new(lerp(tilt.x, 0.08, -0.15), tilt.y));
    fragment
}

/// How far apart the gathered legs scissor, front and back, in metres.
const KICK_SCISSOR: f32 = 0.16;

/// How far a jump out of the water has gathered, in [0, 1]: up at once on
/// the press, then folded into the kick.
pub fn exit_gather(exit: &WaterExit) -> f32 {
    smoothstep(0.0, EXIT_GATHERED, exit.elapsed)
        * (1.0 - smoothstep(EXIT_GATHERED, EXIT_KICKED, exit.elapsed))
}

/// How far a jump out of the water has kicked and pressed, in [0, 1]: a
/// snap through the gather's end, held while the body breaks the surface,
/// then let go into the airborne pose.
pub fn exit_drive(exit: &WaterExit) -> f32 {
    exit_kick(exit) * (1.0 - smoothstep(0.32, WaterExit::DURATION, exit.elapsed))
}

/// How far through the kick and the sweep of the arms a jump out of the
/// water is, in [0, 1].
pub fn exit_kick(exit: &WaterExit) -> f32 {
    smoothstep(EXIT_GATHERED, EXIT_KICKED, exit.elapsed)
}

/// Seconds after the press a jump out of the water is fully gathered.
const EXIT_GATHERED: f32 = 0.07;
/// Seconds after the press its kick and sweep have finished.
const EXIT_KICKED: f32 = 0.17;

/// Launching (anticipation) pose. Feet stay planted, pelvis dips over a
/// rise-and-fall arc peaking at half-duration, torso leans slightly forward.
fn sample_launching(ctx: &SampleCtx<'_>, kind: AirKind, t: f32) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let duration = kind.launch_duration().max(1e-4);
    // Triangle profile: 0 → peak at duration/2 → 0.
    let progress = (t / duration).clamp(0.0, 1.0);
    let envelope = 1.0 - (2.0 * progress - 1.0).abs();
    let dip = kind.launch_crouch_depth() * envelope;

    // Feet track the pelvis each frame: the body is already rising when
    // Launching begins, so a planted anchor would stretch the legs.
    // Feet dip with the squat offset so the compressed silhouette reads.
    let right = anim.facing.cross(&Vector3::y());
    let pelvis = anim.pelvis_position + Vector3::new(0.0, -dip, 0.0);
    let hang = rig.standing_height();
    let left_hip = pelvis - right * rig.hip_width;
    let right_hip = pelvis + right * rig.hip_width;
    let feet = FeetPose {
        left: Point3::new(left_hip.x, pelvis.y - hang, left_hip.z),
        right: Point3::new(right_hip.x, pelvis.y - hang, right_hip.z),
        anchor: FootAnchor::Hips,
    };

    PoseFragment {
        feet: Some(feet),
        hands: None,
        pelvis_offset: Some(Vector3::new(0.0, -dip, 0.0)),
        shoulder_twist: None,
        head_tilt: Some(Vector2::new(0.0, 0.0)),
        head_bob: Some(0.0),
        torso_pitch: Some(0.10 * envelope),
    }
}

/// Swimming lower body: legs trailing from the hips down the body's own axis,
/// kicking across it.
///
/// - `Crawl`: a flutter kick, three beats a leg per stroke cycle, legs in
///   antiphase, feet moving across the body's front-back axis.
/// - `Tread`: a bicycle kick, feet circling under the hips with the knees
///   bent.
///
/// A body that has just fallen in straightens and closes its legs as it goes
/// under, and opens them into the kick as it comes back up (`WaterEntry`).
fn sample_swimming(ctx: &SampleCtx<'_>, stroke: Stroke) -> PoseFragment {
    let rig = ctx.rig;
    let anim = ctx.anim;
    let frame = BodyFrame::new(anim.facing, anim.body_pitch);
    let leg = rig.leg_length();
    let phase = anim.stroke_phase;
    let left_hip = anim.pelvis_position + frame.left() * rig.hip_width;
    let right_hip = anim.pelvis_position + frame.right * rig.hip_width;

    let foot = |hip: Point3<f32>, side: f32| -> Point3<f32> {
        match stroke {
            Stroke::Crawl => {
                let kick = (3.0 * phase + if side < 0.0 { 0.0 } else { PI }).sin();
                hip - frame.up * (leg * 0.94) + frame.front * (SWIM_FLUTTER * kick)
            }
            Stroke::Tread => {
                let turn = 2.0 * phase + if side < 0.0 { 0.0 } else { PI };
                hip - frame.up * (leg * 0.8)
                    + frame.front * (SWIM_TREAD_CIRCLE * turn.cos())
                    + frame.right * (side * SWIM_TREAD_CIRCLE * turn.sin())
            }
        }
    };
    let mut left = foot(left_hip, -1.0);
    let mut right = foot(right_hip, 1.0);

    // Plunging: legs straight and together, pointing down the body.
    let plunge = anim.water_entry.map_or(0.0, |entry| legs_together(&entry));
    if plunge > 0.0 {
        let straight = anim.pelvis_position - frame.up * (leg * 0.98);
        left = Point3::from(
            left.coords
                .lerp(&(straight + frame.left() * 0.03).coords, plunge),
        );
        right = Point3::from(
            right
                .coords
                .lerp(&(straight + frame.right * 0.03).coords, plunge),
        );
    }

    let (torso_pitch, head_tilt) = match stroke {
        // Head up enough to see where it is going, rolling to breathe.
        Stroke::Crawl => (0.0, Vector2::new(-0.05, 0.03 * phase.sin())),
        // Leaning into the sculling hands, chin up out of the water.
        Stroke::Tread => (0.12, Vector2::new(-0.03, 0.0)),
    };

    PoseFragment {
        feet: Some(FeetPose {
            left,
            right,
            anchor: FootAnchor::Hips,
        }),
        hands: None,
        pelvis_offset: None,
        shoulder_twist: None,
        head_tilt: Some(head_tilt),
        head_bob: Some(0.0),
        torso_pitch: Some(torso_pitch),
    }
}

/// Distance a crawling foot kicks either side of the body's axis, in metres.
const SWIM_FLUTTER: f32 = 0.08;
/// Radius of a treading foot's circle, in metres.
const SWIM_TREAD_CIRCLE: f32 = 0.09;

/// How straight and together a plunging body's legs are, in [0, 1]: at once
/// on entry, opening into the kick through the middle of the recovery.
fn legs_together(entry: &WaterEntry) -> f32 {
    entry.strength * (1.0 - smoothstep(0.25, 0.6, entry.progress()))
}

pub fn smoothstep(from: f32, to: f32, x: f32) -> f32 {
    let t = ((x - from) / (to - from)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
