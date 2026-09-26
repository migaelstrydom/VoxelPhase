//! Character animation driver.
//!
//! This is the single source of truth for character animation.
//! All animation logic flows through this animator.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

use super::config::{CharacterRigConfig, GaitPreset};
use super::foot_placer::{FootPlacer, FootSide, PlacerFoot};
use super::humanoid::pose_state::{AirKind, Gait, PoseState, SampleCtx, Stroke, Takeoff, TickCtx};
use super::humanoid::skeleton::{generate_character_mesh, Skeleton};
use super::humanoid::stride_sync;
use super::humanoid::upper_state::{UpperSampleCtx, UpperState, UpperTickCtx};
use super::legged::{LeggedLocomotion, LocomotionCtx};
use super::pose::{Crossfade, Linear, PoseFragment};
use super::state::{AnimationState, FootState, WaterEntry};
use crate::character::grab::GrabConfig;
use crate::character::{
    AirSteering, ArmState, CharacterIntent, CharacterState, Grounding, Immersion, LocomotionState,
};
use crate::rendering::vertex::Vertex;
use crate::sensing::{ContactCandidate, Probe};

/// Duration of the crossfade when either FSM changes variant kind.
const TRANSITION_BLEND_DURATION: f32 = 0.1;

/// Duration of a crossfade into, out of, or within swimming. Limbs move
/// through water slowly, and a stroke changing to a tread is a body settling,
/// not a snap.
const SWIM_BLEND_DURATION: f32 = 0.3;

/// Distance a stroke cycle carries a swimmer, in metres: sets the stroke rate
/// from the speed.
const STROKE_LENGTH: f32 = 1.8;

/// Slowest stroke rate, in cycles per second: treading water, or a crawl
/// barely under way.
const TREAD_RATE: f32 = 0.7;

/// Extra step height a wader lifts its feet by at chest depth, in metres.
/// Striding through water is done knees high.
const WADE_KNEE_LIFT: f32 = 0.08;

/// Time constant the wade depth is eased with, in seconds.
const WADE_EASE: f32 = 0.15;

/// What the animator is told about the physics body each frame.
pub struct BodyReading<'a> {
    /// Where the rig's pelvis belongs: see [`CharacterAnimator::pelvis_for`].
    pub pelvis: Point3<f32>,
    /// Heading, in radians about +y, with 0 facing +z.
    pub yaw: f32,
    pub velocity: Vector3<f32>,
    /// The body's long axis — its local +Y — in world space. Upright it is
    /// world up; a swimmer's lies along its heading.
    pub up: Vector3<f32>,
    pub grounding: &'a Grounding,
    pub immersion: &'a Immersion,
}

/// The character animation driver.
///
/// Owns all character-specific state and logic:
/// - Configures probes based on animation state
/// - Processes probe results
/// - Updates gait and foot positions
/// - Computes skeleton pose
#[derive(Component)]
#[storage(VecStorage)]
pub struct CharacterAnimator {
    pub config: CharacterRigConfig,
    pub state: AnimationState,
    pub skeleton: Skeleton,

    /// Current lower-body / core pose FSM variant.
    pub pose_state: PoseState,
    /// Current upper-body FSM variant.
    pub upper_state: UpperState,
    /// Active blend for the lower-body FSM. `Some` only during the
    /// short window after a variant-kind change; `None` between transitions.
    pose_crossfade: Option<Crossfade<Linear>>,
    /// Active blend for the upper-body FSM.
    upper_crossfade: Option<Crossfade<Linear>>,

    /// Feet, probes and the support frame. Shared with every other
    /// two-legged rig; the humanoid on top of it is what this animator
    /// adds.
    pub locomotion: LeggedLocomotion,

    // Mesh caching
    cached_vertices: Vec<Vertex>,
    cached_indices: Vec<u32>,
}

impl CharacterAnimator {
    /// Create a new character animator for a body at `body_position` whose
    /// origin rides `ground_clearance` above the surface it rests on — for a
    /// capsule collider, its half height.
    ///
    /// The clearance is what lets the animator place the pelvis where the rig
    /// expects it rather than where the physics body's origin happens to be.
    pub fn new(
        config: CharacterRigConfig,
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
        let pelvis_position = locomotion.pelvis_for(body_position);
        // The rest pose is built facing the way the character does. Placing
        // the first stance along a fixed axis leaves a character that spawns
        // facing anywhere else standing with its feet across its own path,
        // and it walks the first half-second out of that.
        let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());

        let state = AnimationState::new(pelvis_position, config.leg_length());
        let skeleton = Skeleton::new(&config, pelvis_position, facing);

        Self {
            config,
            state,
            skeleton,
            pose_state: PoseState::Grounded { gait: Gait::Idle },
            upper_state: UpperState::Swinging,
            pose_crossfade: None,
            upper_crossfade: None,
            locomotion,
            cached_vertices: Vec::new(),
            cached_indices: Vec::new(),
        }
    }

    /// Where the rig's pelvis belongs, given where the physics body is and
    /// which way its long axis points. The pelvis hangs below the body's
    /// centre along that axis, so a swimmer's lies behind it.
    pub fn pelvis_for(&self, body_position: Point3<f32>, body_up: Vector3<f32>) -> Point3<f32> {
        let drop = body_position.y - self.locomotion.pelvis_for(body_position).y;
        body_position - body_up * drop
    }

    /// The foot placer driving the legs.
    pub fn foot_placer(&self) -> &FootPlacer {
        self.locomotion.placer()
    }

    /// Configure foot probes for the next frame. While the placer is
    /// suspended (airborne) the probes aim at the feet the airborne
    /// sampler is drawing, since the placer has no anchors to offer.
    pub fn configure_probes(&self, pelvis_position: Point3<f32>, yaw: f32) -> Vec<Probe> {
        self.locomotion.configure_probes(
            pelvis_position,
            yaw,
            [self.state.left.position, self.state.right.position],
        )
    }

    /// Process probe results and update animation.
    pub fn update(
        &mut self,
        dt: f32,
        body: &BodyReading<'_>,
        character_state: &CharacterState,
        target: &CharacterIntent,
        grab_config: &GrabConfig,
        contacts: &[ContactCandidate],
    ) {
        let (pelvis_position, yaw, grounding) = (body.pelvis, body.yaw, body.grounding);
        // Everything below the neck is animated in the frame of whatever is
        // holding the character up. A body riding a platform has the
        // platform's velocity and is nonetheless standing still: it should
        // read as idle, its arms should not swing, and its feet should stay
        // where they were put. Nothing carries an airborne character, so this
        // is the world frame the moment support is lost.
        let support_velocity = self.locomotion.observe_support(grounding, dt);
        let velocity = body.velocity - support_velocity;
        let speed = Vector3::new(velocity.x, 0.0, velocity.z).magnitude();
        let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());

        self.state.facing = facing;
        self.state.pelvis_position = pelvis_position;
        self.state.velocity = velocity;
        self.state.body_pitch = body.up.dot(&facing).atan2(body.up.y);
        self.state.water_surface = body.immersion.water.map(|w| w.surface);
        self.locomotion.process_contacts(contacts);
        self.state.left.normal = self.locomotion.ground(FootSide::Left).normal_or_up();
        self.state.right.normal = self.locomotion.ground(FootSide::Right).normal_or_up();
        // Support is decided by the contacts the physics engine solved, not by
        // where the gait happened to aim a probe — see
        // `docs/TRACTION_DRIVE_DESIGN.md` §6.6. The probes below remain the
        // rangefinder: how high the ground is under a landing target, and
        // which way it faces.
        self.state.is_grounded = grounding.is_grounded;

        // Snapshot the foot-centre y for a potential Landing splice this
        // frame. `foot.position.y` represents the foot centre (sole is one
        // radius below), so a probe contact is the correct value directly.
        let landing_ground_y = self.locomotion.ground_height(pelvis_position);

        self.ease_wade(landing_ground_y, dt);

        // A body in the water with nothing under it looks like a swimmer
        // whatever the motion FSM calls it: one still falling into the water,
        // or one standing up out of it before its feet find the floor.
        let afloat = !grounding.is_grounded
            && self
                .state
                .water_surface
                .is_some_and(|surface| surface > pelvis_position.y);

        // Map CharacterState → next PoseState variant.
        let new_pose = next_pose_state(
            self.pose_state,
            character_state,
            target,
            speed,
            self.config.idle_threshold,
            facing,
            landing_ground_y,
            afloat,
        );
        self.advance_stroke(&new_pose, speed, dt);
        self.track_water_entry(&new_pose, dt);

        // Tick the foot placer before sampling so the pose layer reads a
        // current foot position. Airborne states suspend the placer; feet
        // come from the airborne/landing samplers in those cases.
        self.tick_locomotion(
            dt,
            pelvis_position,
            yaw,
            velocity,
            support_velocity,
            target,
            &new_pose,
        );
        mirror_placer_into_state(&mut self.state, self.locomotion.placer());

        // Stride phase is derived directly from the placer's stepping
        // state — arm swing / shoulder twist / head bob lock to real
        // foot events rather than a speed-driven wheel. At idle (both
        // feet planted) the phase holds, so arms coast instead of
        // snapping to rest.
        self.state.stride_phase =
            stride_sync::phase_from_placer(self.locomotion.placer(), self.state.stride_phase, dt);

        // Stride activity snaps to 1 while anything is stepping and
        // decays exponentially to 0 once both feet are planted. Upper
        // body amplitudes (arm swing, shoulder twist, head bob) are
        // scaled by it so the rest→swing→rest transition is continuous
        // — matters most on slope slides where gait can flicker between
        // Idle and Walk even though the body is clearly moving.
        let placer = self.locomotion.placer();
        let stepping = !placer.left.is_planted() || !placer.right.is_planted();
        if stepping {
            self.state.stride_activity = 1.0;
        } else {
            let decay_tc = 0.25;
            self.state.stride_activity *= (-dt / decay_tc).exp();
        }

        // Pick the upper variant alongside the new pose variant so Idle-vs-
        // airborne gating sees the just-computed pose.
        let new_upper = map_upper_state(&character_state.arm, new_pose);

        let pose_sample_ctx = SampleCtx {
            rig: &self.config,
            anim: &self.state,
            velocity,
        };

        // Snapshot outgoing fragments BEFORE the variant swap, so the
        // crossfade `from` reflects what the old state was producing.
        let blend = blend_duration(&self.pose_state, &new_pose);
        begin_crossfade_if_changed(
            &mut self.pose_crossfade,
            self.pose_state.transition_key(),
            new_pose.transition_key(),
            pelvis_position,
            self.config.leg_length(),
            blend,
            || self.pose_state.sample(&pose_sample_ctx),
        );
        begin_crossfade_if_changed(
            &mut self.upper_crossfade,
            self.upper_state.transition_key(),
            new_upper.transition_key(),
            pelvis_position,
            self.config.leg_length(),
            blend,
            || {
                let ctx = build_upper_ctx(&self.pose_state, &self.config, &self.state, grab_config);
                self.upper_state.sample(&ctx)
            },
        );

        // Swap in new variants, then tick.
        self.pose_state = new_pose;
        self.upper_state = new_upper;

        let pose_tick_ctx = TickCtx {
            dt,
            velocity,
            horizontal_speed: speed,
            rig: &self.config,
        };
        self.pose_state = self.pose_state.tick(&pose_tick_ctx);
        self.upper_state = self.upper_state.tick(&UpperTickCtx { dt });

        // Sample the (now current) states.
        let to_pose = self.pose_state.sample(&pose_sample_ctx);
        let upper_sample_ctx =
            build_upper_ctx(&self.pose_state, &self.config, &self.state, grab_config);
        let to_upper = self.upper_state.sample(&upper_sample_ctx);

        // Apply any running crossfades. `is_active` is checked AFTER ticking
        // so the final frame of a blend lands on weight=1.0 cleanly.
        let pose_fragment = blend_through(&mut self.pose_crossfade, to_pose, pelvis_position, dt);
        let upper_fragment =
            blend_through(&mut self.upper_crossfade, to_upper, pelvis_position, dt);

        let fragment = pose_fragment.compose(&upper_fragment);
        self.apply_fragment_to_state(&fragment);
        self.skeleton
            .apply_fragment(&fragment, &self.state, &self.config);
    }

    /// Advance legged locomotion. Runs before pose sampling; its output is
    /// mirrored into `AnimationState` and read by the grounded pose
    /// samplers. Airborne states suspend stepping so feet don't step
    /// against a body that isn't on the ground.
    #[allow(clippy::too_many_arguments)]
    fn tick_locomotion(
        &mut self,
        dt: f32,
        pelvis_position: Point3<f32>,
        yaw: f32,
        velocity: Vector3<f32>,
        support_velocity: Vector3<f32>,
        target: &CharacterIntent,
        next_pose: &PoseState,
    ) {
        let airborne = matches!(
            next_pose,
            PoseState::Launching { .. } | PoseState::Airborne { .. } | PoseState::Swimming { .. }
        );

        let preset = gait_preset_for(next_pose, &self.config);
        let step_height = preset
            .map(|p| p.step_height)
            .unwrap_or(self.config.step_height)
            + WADE_KNEE_LIFT * self.state.wade;
        // `stride_gain` scales the capture-point target: 1.0 plants at
        // the stopping foothold, values below 1 let the body pass over
        // the foot. Airborne and landing states fall back to the walk
        // preset — any nonzero value is fine since the placer is
        // suspended during Launching/Airborne.
        let stride_gain = preset
            .map(|p| p.stride_gain)
            .unwrap_or(self.config.gait_presets.walk.stride_gain);

        self.locomotion.tick(&LocomotionCtx {
            dt,
            pelvis: pelvis_position,
            yaw,
            velocity,
            support_velocity,
            intent_direction: target.direction,
            airborne,
            step_height,
            stride_gain,
            pose_tag: next_pose.tag(),
        });
    }

    /// Ease the wade depth toward how deep the water stands over the ground
    /// under the character, as a fraction of its chest height.
    fn ease_wade(&mut self, ground_y: f32, dt: f32) {
        let chest = self.config.standing_height() + self.config.torso_height;
        let target = self.state.water_surface.map_or(0.0, |surface| {
            ((surface - ground_y) / chest).clamp(0.0, 1.0)
        });
        let blend = 1.0 - (-dt / WADE_EASE).exp();
        self.state.wade += (target - self.state.wade) * blend;
    }

    /// Advance the stroke clock while swimming: a cycle per `STROKE_LENGTH`
    /// travelled, never slower than a tread. Out of the water it holds, so a
    /// swimmer who stands and goes straight back in picks up where it was.
    fn advance_stroke(&mut self, pose: &PoseState, speed: f32, dt: f32) {
        if !matches!(pose, PoseState::Swimming { .. }) {
            return;
        }
        let rate = (speed / STROKE_LENGTH).max(TREAD_RATE);
        self.state.stroke_phase =
            (self.state.stroke_phase + rate * std::f32::consts::TAU * dt) % std::f32::consts::TAU;
    }

    /// Start a water entry when the body goes from the air into the water,
    /// and play it out; any recovery ends the moment the body is out of the
    /// water again.
    fn track_water_entry(&mut self, pose: &PoseState, dt: f32) {
        let swimming = matches!(pose, PoseState::Swimming { .. });
        let was_falling = matches!(
            self.pose_state,
            PoseState::Airborne { .. } | PoseState::Launching { .. }
        );
        self.state.water_entry = if !swimming {
            None
        } else if was_falling {
            WaterEntry::at(-self.state.velocity.y)
        } else {
            self.state.water_entry.and_then(|entry| entry.advanced(dt))
        };
    }

    /// Start recording placer input, or stop and write what was kept.
    pub fn toggle_recording(&mut self) {
        self.locomotion.toggle_recording();
    }

    /// What the recorder is doing and what it costs, for the debug overlay.
    pub fn recording_status(&self) -> Option<String> {
        self.locomotion.recording_status()
    }

    /// Mirror fragment channels into `AnimationState` so next-frame probes
    /// and any external readers see a coherent snapshot.
    fn apply_fragment_to_state(&mut self, fragment: &PoseFragment) {
        if let Some(feet) = &fragment.feet {
            self.state.left.position = feet.left;
            self.state.right.position = feet.right;
        }
        if let Some(hands) = &fragment.hands {
            self.state.left_hand.position = hands.left;
            self.state.right_hand.position = hands.right;
        }
        if let Some(twist) = fragment.shoulder_twist {
            self.state.shoulder_twist = twist;
        }
        if let Some(tilt) = fragment.head_tilt {
            self.state.head_tilt = tilt;
        }
        if let Some(bob) = fragment.head_bob {
            self.state.head_bob = bob;
        }
    }

    /// Whether something is holding the character up, as the Support Set
    /// resolved it this frame.
    pub fn is_grounded(&self) -> bool {
        self.state.is_grounded
    }

    /// Get mesh vertices and indices for rendering.
    pub fn mesh(&mut self) -> (&[Vertex], &[u32]) {
        self.regenerate_mesh();
        (&self.cached_vertices, &self.cached_indices)
    }

    /// Regenerate the mesh from current skeleton state.
    fn regenerate_mesh(&mut self) {
        let (vertices, indices) = generate_character_mesh(&self.skeleton, &self.config);
        self.cached_vertices = vertices;
        self.cached_indices = indices;
    }
}

/// Open a fresh crossfade into a new FSM variant when the transition key
/// has changed. The `sample_from` closure produces the outgoing fragment
/// — it is only evaluated when a crossfade is actually needed.
fn begin_crossfade_if_changed<K: PartialEq, F: FnOnce() -> PoseFragment>(
    slot: &mut Option<Crossfade<Linear>>,
    old_key: K,
    new_key: K,
    pelvis: Point3<f32>,
    reach: f32,
    duration: f32,
    sample_from: F,
) {
    if old_key != new_key {
        *slot = Some(Crossfade {
            from: sample_from(),
            from_pelvis: pelvis,
            reach,
            to_duration: duration,
            elapsed: 0.0,
            policy: Linear,
        });
    }
}

/// How long a change from `from` to `to` should take to blend.
fn blend_duration(from: &PoseState, to: &PoseState) -> f32 {
    let swimming = |p: &PoseState| matches!(p, PoseState::Swimming { .. });
    if swimming(from) || swimming(to) {
        SWIM_BLEND_DURATION
    } else {
        TRANSITION_BLEND_DURATION
    }
}

/// Build the `UpperSampleCtx` derived from the current `PoseState` — the
/// cycle, preset, pelvis offset, torso pitch and airborne context the
/// upper body should track this frame.
fn build_upper_ctx<'a>(
    pose: &PoseState,
    rig: &'a CharacterRigConfig,
    anim: &'a AnimationState,
    grab: &'a GrabConfig,
) -> UpperSampleCtx<'a> {
    let preset = gait_preset_for(pose, rig);
    let (pelvis_offset, torso_pitch) = upper_torso_inputs(preset);
    UpperSampleCtx {
        rig,
        anim,
        grab,
        cycle: pose.cycle(anim),
        preset,
        pelvis_offset,
        torso_pitch,
        airborne: airborne_ctx_for(pose),
        stroke: match pose {
            PoseState::Swimming { stroke } => Some(*stroke),
            _ => None,
        },
    }
}

/// Advance a crossfade (if any) and return the fragment the driver should
/// use this frame. Clears the slot once the blend has fully resolved.
fn blend_through(
    slot: &mut Option<Crossfade<Linear>>,
    to: PoseFragment,
    current_pelvis: Point3<f32>,
    dt: f32,
) -> PoseFragment {
    match slot.as_mut() {
        Some(cf) => {
            let blended = cf.sample(&to, current_pelvis);
            cf.tick(dt);
            if !cf.is_active() {
                *slot = None;
            }
            blended
        }
        None => to,
    }
}

/// Look up the active `GaitPreset` for the current `PoseState`. Returns
/// `None` when the lower body is not Grounded — the upper body falls back
/// to rig-level defaults in that case.
fn gait_preset_for(pose: &PoseState, rig: &CharacterRigConfig) -> Option<GaitPreset> {
    match pose {
        PoseState::Grounded { gait } => Some(rig.gait_presets.for_gait(*gait)),
        _ => None,
    }
}

/// Pelvis offset and torso pitch that `UpperState` should apply this frame.
/// Mirrors the values `PoseState::Grounded::sample` emits into the pose
/// fragment so the upper body tracks a crouched/pitched torso.
fn upper_torso_inputs(preset: Option<GaitPreset>) -> (Vector3<f32>, f32) {
    let pelvis_offset = preset
        .map(|p| Vector3::new(0.0, -p.pelvis_crouch_offset, 0.0))
        .unwrap_or_else(Vector3::zeros);
    let torso_pitch = preset.map(|p| p.torso_pitch).unwrap_or(0.0);
    (pelvis_offset, torso_pitch)
}

/// Airborne context (kind + latched takeoff) to expose to `UpperState`,
/// when the lower body is mid-air. `Launching` and `Airborne` both count
/// — both reflect a committed takeoff that Braced should read.
fn airborne_ctx_for(pose: &PoseState) -> Option<(AirKind, Takeoff)> {
    match pose {
        PoseState::Launching { kind, takeoff, .. } | PoseState::Airborne { kind, takeoff } => {
            Some((*kind, *takeoff))
        }
        _ => None,
    }
}

/// Copy the placer's current per-foot position into `AnimationState` so
/// the pose layer samples coherent foot channels this frame. Planted
/// position mirrors too, primarily for debug overlays that still read it.
fn mirror_placer_into_state(state: &mut AnimationState, placer: &FootPlacer) {
    copy_placer_foot(&mut state.left, &placer.left);
    copy_placer_foot(&mut state.right, &placer.right);
}

fn copy_placer_foot(foot: &mut FootState, placer: &PlacerFoot) {
    foot.position = placer.position;
    foot.planted_position = placer.planted_position;
    foot.up = placer.up;
    foot.forward = placer.forward;
}

/// Map `ArmState` (+ current `PoseState`) to an `UpperState` variant.
/// `Idle + grounded` → `Swinging`; anything else with `Idle` (airborne,
/// launching, landing) → `Braced`. `Reaching`/`Holding` mirror their
/// `ArmState` shape.
fn map_upper_state(arm: &ArmState, pose: PoseState) -> UpperState {
    match arm {
        ArmState::Idle => match pose {
            PoseState::Grounded { .. } => UpperState::Swinging,
            PoseState::Swimming { .. } => UpperState::Stroking,
            _ => UpperState::Braced,
        },
        ArmState::Reaching { elapsed, target } => UpperState::Reaching {
            elapsed: *elapsed,
            target: *target,
        },
        ArmState::Holding {
            target_body,
            constraint,
            current_hold_height,
        } => UpperState::Holding {
            target_body: *target_body,
            constraint: *constraint,
            current_hold_height: *current_hold_height,
        },
    }
}

/// Decide the next `PoseState`, honouring in-flight Launching/Landing
/// timers so anticipation/follow-through play through to completion even
/// if the underlying player state moves on. `landing_ground_y` is the
/// impact y baked into any fresh `Landing` splice.
fn next_pose_state(
    current: PoseState,
    character: &CharacterState,
    target: &CharacterIntent,
    speed: f32,
    idle_threshold: f32,
    facing: Vector3<f32>,
    landing_ground_y: f32,
    afloat: bool,
) -> PoseState {
    // Launching: hold for the full anticipation window unless physics
    // reports an early touchdown (rare — e.g. hit ceiling, dropped back).
    if let PoseState::Launching { kind, takeoff, t } = current {
        if matches!(character.locomotion, LocomotionState::Grounded) {
            return PoseState::Landing {
                kind,
                t: 0.0,
                ground_y: landing_ground_y,
            };
        }
        if !current.timer_expired() {
            return PoseState::Launching { kind, takeoff, t };
        }
        // Timer expired: fall through to the player-driven mapping (which
        // will typically observe Airborne and produce Airborne { kind }).
    }

    // Landing: hold for the full follow-through window. No early exits —
    // even if the player input changes, we keep squashing.
    if let PoseState::Landing { kind, t, ground_y } = current {
        if !current.timer_expired() {
            return PoseState::Landing { kind, t, ground_y };
        }
    }

    let moving = speed > idle_threshold || target.direction.magnitude_squared() > 0.001;

    // In the water, the stroke follows the intent the way the gait does on
    // land: going somewhere is a crawl, staying put is a tread.
    let swimming = PoseState::Swimming {
        stroke: if target.direction.magnitude_squared() > 0.001 {
            Stroke::Crawl
        } else {
            Stroke::Tread
        },
    };

    let mapped = match character.locomotion {
        // CoyoteTime exists to bridge one-frame ground-contact losses
        // (seams, lips). Treat it as grounded: flicking to Airborne here
        // would suspend the foot placer and splice a spurious Landing on
        // every blip. The fall pose starts only when the grace period
        // genuinely expires into Airborne.
        LocomotionState::Grounded | LocomotionState::CoyoteTime(_) => {
            let gait = if target.crouch {
                Gait::Crouch { walking: moving }
            } else if !moving {
                Gait::Idle
            } else if target.sprint {
                Gait::Sprint
            } else {
                Gait::Walk
            };
            PoseState::Grounded { gait }
        }
        LocomotionState::Launching { steering, .. } => {
            let kind = match steering {
                AirSteering::Locked { .. } => AirKind::LongJump,
                AirSteering::Responsive => AirKind::Jump,
            };
            let takeoff = Takeoff {
                facing,
                air_speed: character.air_speed,
            };
            PoseState::Launching {
                kind,
                takeoff,
                t: 0.0,
            }
        }
        LocomotionState::Swimming => swimming,
        // Falling in, or standing up on the way out: upright in the water,
        // so treading whatever the intent — only a swimmer lying down crawls.
        LocomotionState::Airborne { .. } if afloat => PoseState::Swimming {
            stroke: Stroke::Tread,
        },
        LocomotionState::Airborne { steering, .. } => {
            let kind = match (current, steering) {
                (PoseState::Launching { kind, .. }, _) => kind,
                (PoseState::Airborne { kind, .. }, _) => kind,
                (_, AirSteering::Locked { .. }) => AirKind::LongJump,
                (_, AirSteering::Responsive) => AirKind::Fall,
            };
            let takeoff = match current {
                PoseState::Launching { takeoff, .. } | PoseState::Airborne { takeoff, .. } => {
                    takeoff
                }
                _ => Takeoff {
                    facing,
                    air_speed: character.air_speed,
                },
            };
            PoseState::Airborne { kind, takeoff }
        }
    };

    // Splice Landing on the Airborne-ish → Grounded edge. Uses the prior
    // airborne `kind` so a LongJump ends in a heavy squash.
    if let PoseState::Grounded { .. } = mapped {
        let prev_air_kind = match current {
            PoseState::Airborne { kind, .. } | PoseState::Launching { kind, .. } => Some(kind),
            _ => None,
        };
        if let Some(kind) = prev_air_kind {
            return PoseState::Landing {
                kind,
                t: 0.0,
                ground_y: landing_ground_y,
            };
        }
    }

    mapped
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One-frame ground-contact losses (seams) put locomotion in
    /// CoyoteTime; the pose must stay Grounded — flicking to Airborne
    /// suspends the placer and splices a spurious Landing on return.
    #[test]
    fn coyote_time_keeps_grounded_pose() {
        let mut character = CharacterState::default();
        character.locomotion = LocomotionState::CoyoteTime(0.1);
        let mut target = CharacterIntent::default();
        target.direction = Vector3::new(0.0, 0.0, 1.0);

        let next = next_pose_state(
            PoseState::Grounded { gait: Gait::Walk },
            &character,
            &target,
            2.0,
            0.1,
            Vector3::new(0.0, 0.0, 1.0),
            0.0,
            false,
        );

        assert!(matches!(next, PoseState::Grounded { gait: Gait::Walk }));
    }

    /// A capsule's centre rides half its height up; the rig's pelvis belongs
    /// one `standing_height` above the soles, which is less. The animator owns
    /// the difference, and getting it wrong costs the legs every millimetre of
    /// bend they have.
    #[test]
    fn the_pelvis_hangs_below_the_body_by_the_knee_bend_it_needs() {
        let config = CharacterRigConfig::default();
        let standing = config.standing_height();
        let clearance = 0.5;
        let animator =
            CharacterAnimator::new(config, Point3::new(0.0, clearance, 0.0), clearance, 0.0);

        let pelvis = animator.pelvis_for(Point3::new(4.0, clearance, -2.0), Vector3::y());
        assert_eq!((pelvis.x, pelvis.z), (4.0, -2.0), "only height is adjusted");
        assert!(
            (pelvis.y - standing).abs() < 1e-6,
            "a body resting on y=0 should put the pelvis at its standing height, got {}",
            pelvis.y
        );
    }

    /// A body whose origin rides lower than the rig is tall has no offset to
    /// give: lifting the pelvis to suit would put the feet under the floor.
    #[test]
    fn a_body_that_rides_low_keeps_its_own_origin() {
        let config = CharacterRigConfig::default();
        let clearance = config.standing_height() * 0.5;
        let animator = CharacterAnimator::new(config, Point3::origin(), clearance, 0.0);
        assert_eq!(
            animator.pelvis_for(Point3::origin(), Vector3::y()),
            Point3::origin()
        );
    }
}
