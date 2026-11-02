//! Character animation driver.
//!
//! This is the single source of truth for character animation.
//! All animation logic flows through this animator.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

use super::config::{CharacterRigConfig, GaitPreset};
use super::foot_placer::{FlightLanding, FootPlacer, FootSide, LegMode, PlacerFoot, StepId};
use super::humanoid::pose_state::{AirKind, Gait, PoseState, SampleCtx, Stroke, Takeoff, TickCtx};
use super::humanoid::skeleton::{generate_character_mesh, Skeleton};
use super::humanoid::stride_sync;
use super::humanoid::upper_state::{UpperSampleCtx, UpperState, UpperTickCtx};
use super::legged::{LeggedLocomotion, LocomotionCtx};
use super::pose::{Crossfade, Linear, PoseFragment};
use super::state::{AnimationState, FootState, ThrowGesture, WaterEntry, WaterExit};
use crate::character::grab::GrabConfig;
use crate::character::locomotion::{Jump, JumpKind, TakeoffPhase};
use crate::character::{
    ArmState, CharacterIntent, CharacterState, Foot, Grounding, Immersion, LocomotionState,
    ThrowStroke, ThrowingHand,
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
    /// The fastest the body has fallen, in m/s, since this flight began;
    /// zero out of the air. How hard a flight goes into the water: by the
    /// time the water takes the body, it has already slowed it.
    descent: f32,

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
            descent: 0.0,
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

    /// Configure foot probes for the next frame, as the legs' mode aims
    /// them ([`LeggedLocomotion::configure_probes`]).
    pub fn configure_probes(&self, pelvis_position: Point3<f32>, yaw: f32) -> Vec<Probe> {
        self.locomotion.configure_probes(pelvis_position, yaw)
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
        let (yaw, grounding) = (body.yaw, body.grounding);
        // The probes are read first: how low the pelvis hangs depends on how
        // far the ground under each hip has fallen away.
        self.locomotion.process_contacts(contacts);
        let pelvis_position = self
            .locomotion
            .lower_pelvis(body.pelvis, yaw, grounding, dt);
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
        self.state.left.normal = self.locomotion.ground(FootSide::Left).normal_or_up();
        self.state.right.normal = self.locomotion.ground(FootSide::Right).normal_or_up();
        // Support is decided by the contacts the physics engine solved, not by
        // where the gait happened to aim a probe — see
        // `docs/TRACTION_DRIVE_DESIGN.md` §6.6. The probes below remain the
        // rangefinder: how high the ground is under a landing target, and
        // which way it faces.
        self.state.is_grounded = grounding.is_grounded;

        let ground_y = self.locomotion.ground_height(pelvis_position);
        self.ease_wade(ground_y, dt);

        let new_pose = next_pose_state(
            self.pose_state,
            character_state,
            target,
            speed,
            self.config.idle_threshold,
            facing,
        );
        self.advance_stroke(&new_pose, speed, dt);
        self.play_transients(&new_pose, dt);

        // Tick the foot placer before sampling so the pose layer reads a
        // current foot position. Off the legs, the feet come from the
        // airborne and swimming samplers.
        self.tick_locomotion(
            dt,
            pelvis_position,
            yaw,
            velocity,
            support_velocity,
            target,
            &new_pose,
            leg_mode(&character_state.locomotion),
        );
        let flown_before = self.state.feet_flown;
        mirror_placer_into_state(&mut self.state, self.locomotion.placer());
        // Feet the placer starts or stops flying to a landing come from it
        // or from the hips without the pose changing: blend between the two.
        if self.state.feet_flown != flown_before {
            let before = AnimationState {
                feet_flown: flown_before,
                ..self.state.clone()
            };
            let from = self.pose_state.sample(&SampleCtx {
                rig: &self.config,
                anim: &before,
                velocity,
            });
            self.pose_crossfade = Some(Crossfade {
                from,
                from_pelvis: pelvis_position,
                reach: self.config.leg_length(),
                to_duration: TRANSITION_BLEND_DURATION,
                elapsed: 0.0,
                policy: Linear,
            });
        }

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

        let new_upper = map_upper_state(&character_state.arm, &character_state.locomotion);

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
        let fragment = match self.state.landing {
            Some(squash) => squash.applied_to(fragment),
            None => fragment,
        };
        self.apply_fragment_to_state(&fragment);
        self.skeleton
            .apply_fragment(&fragment, &self.state, &self.config);
    }

    /// Advance legged locomotion in `mode`. Runs before pose sampling; its
    /// output is mirrored into `AnimationState` and read by the grounded
    /// pose samplers.
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
        mode: LegMode,
    ) {
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
            mode,
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

    /// Start the transients a change from the current pose to `next` plays,
    /// and play out those running: a landing squash coming down from the air
    /// onto the feet, scaled by the flight, and a water entry coming down
    /// into the water, as hard as the flight came down. Each ends the moment
    /// the pose it plays on does.
    fn play_transients(&mut self, next: &PoseState, dt: f32) {
        let from_air = self.pose_state.air_kind();
        let descent = self.descent.max(-self.state.velocity.y);
        self.descent = match next.air_kind() {
            Some(_) => descent,
            None => 0.0,
        };
        self.state.landing = match (next, from_air) {
            (PoseState::Grounded { .. }, Some(kind)) => Some(kind.landing_squash()),
            (PoseState::Grounded { .. }, None) => {
                self.state.landing.and_then(|squash| squash.advanced(dt))
            }
            (
                PoseState::Launching { .. }
                | PoseState::Hop { .. }
                | PoseState::Airborne { .. }
                | PoseState::Swimming { .. },
                _,
            ) => None,
        };
        self.state.water_exit = match (next, &self.pose_state) {
            (
                PoseState::Airborne {
                    kind: AirKind::Jump,
                    ..
                },
                PoseState::Swimming { .. },
            ) => Some(WaterExit::default()),
            (PoseState::Airborne { .. }, _) => {
                self.state.water_exit.and_then(|exit| exit.advanced(dt))
            }
            (
                PoseState::Grounded { .. }
                | PoseState::Launching { .. }
                | PoseState::Hop { .. }
                | PoseState::Swimming { .. },
                _,
            ) => None,
        };
        self.state.water_entry = match (next, from_air) {
            (PoseState::Swimming { .. }, Some(_)) => WaterEntry::at(descent),
            (PoseState::Swimming { .. }, None) => {
                self.state.water_entry.and_then(|entry| entry.advanced(dt))
            }
            (
                PoseState::Grounded { .. }
                | PoseState::Launching { .. }
                | PoseState::Hop { .. }
                | PoseState::Airborne { .. },
                _,
            ) => None,
        };
    }

    /// Play out the throws in progress and take up any new ones, before
    /// [`Self::update`]. `strokes` are the character's throws in hand this
    /// frame; a gesture keeps the stroke's clock while it is in hand and its
    /// own afterwards, through the follow-through.
    pub fn observe_throws<'s>(
        &mut self,
        strokes: impl IntoIterator<Item = &'s ThrowStroke>,
        dt: f32,
    ) {
        let throws = &mut self.state.throws;
        for gesture in throws.iter_mut() {
            gesture.elapsed += dt;
        }
        throws.retain(|gesture| !gesture.finished());
        for stroke in strokes {
            if let Some(gesture) = throws.iter_mut().find(|g| g.stroke == stroke.id) {
                gesture.elapsed = stroke.elapsed;
                continue;
            }
            // A new throw takes its hand from whatever that hand was
            // still finishing.
            throws.retain(|gesture| gesture.hand != stroke.hand);
            throws.push(ThrowGesture {
                stroke: stroke.id,
                hand: stroke.hand,
                elapsed: stroke.elapsed,
                windup: stroke.release_at,
                aim: stroke.aim,
            });
        }
    }

    /// Where a hand is, and which way its forearm runs out of it: what
    /// something held in it hangs from.
    pub fn grip(&self, hand: ThrowingHand) -> (Point3<f32>, Vector3<f32>) {
        let s = &self.skeleton;
        let (elbow, hand) = match hand {
            ThrowingHand::Left => (s.left_elbow, s.left_hand),
            ThrowingHand::Right => (s.right_elbow, s.right_hand),
        };
        let forearm = (hand - elbow)
            .try_normalize(1e-6)
            .unwrap_or_else(Vector3::y);
        (hand, forearm)
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
    match (from, to) {
        // A jump out of the water is a burst, not a body settling.
        (
            PoseState::Swimming { .. },
            PoseState::Airborne {
                kind: AirKind::Jump,
                ..
            },
        ) => TRANSITION_BLEND_DURATION,
        _ if swimming(from) || swimming(to) => SWIM_BLEND_DURATION,
        _ => TRANSITION_BLEND_DURATION,
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
    state.feet_flown = placer.flies_to_landing();
}

fn copy_placer_foot(foot: &mut FootState, placer: &PlacerFoot) {
    foot.position = placer.position;
    foot.planted_position = placer.planted_position;
    foot.up = placer.up;
    foot.forward = placer.forward;
}

/// The arms' role: what the grab system has them doing, and otherwise the
/// locomotion state's carriage. They swing on foot and through a Hop, which
/// flows out of the run and back into it, brace through a jump or a fall,
/// and stroke in the water. Which grabs a state allows is
/// `CharacterState::arm_state_allowed`.
fn map_upper_state(arm: &ArmState, state: &LocomotionState) -> UpperState {
    match arm {
        ArmState::Idle => match state {
            LocomotionState::OnFoot(_) | LocomotionState::Hop(_) => UpperState::Swinging,
            LocomotionState::Jump(_) | LocomotionState::Fall(_) => UpperState::Braced,
            LocomotionState::Swim(_) => UpperState::Stroking,
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

/// The legs' mode in `state`: the gait on foot, a Hop's feet flown onto its
/// landing, and off in a jump, a fall or the water.
fn leg_mode(state: &LocomotionState) -> LegMode {
    match state {
        LocomotionState::OnFoot(_) => LegMode::Step,
        LocomotionState::Hop(hop) => LegMode::Fly(hop.landing.map(|landing| FlightLanding {
            side: match landing.foot {
                Foot::Left => FootSide::Left,
                Foot::Right => FootSide::Right,
            },
            at: landing.at,
            in_seconds: landing.in_seconds,
            footfall: landing.footfall.map(StepId),
        })),
        LocomotionState::Jump(_) | LocomotionState::Fall(_) | LocomotionState::Swim(_) => {
            LegMode::Off
        }
    }
}

/// The pose for `character`'s locomotion state: on foot by gait, a Hop's
/// leap, a jump's wind-up and then its flight, a fall, a swim by stroke. A
/// jump that leaves from the ground winds up first, for the wind-up's own
/// duration, while it is still a Jump; one from grace is already in the air.
/// A flight's kind and takeoff carry over from the pose before.
fn next_pose_state(
    current: PoseState,
    character: &CharacterState,
    target: &CharacterIntent,
    speed: f32,
    idle_threshold: f32,
    facing: Vector3<f32>,
) -> PoseState {
    let moving = speed > idle_threshold || target.direction.magnitude_squared() > 0.001;
    let takeoff = current.takeoff().unwrap_or(Takeoff {
        facing,
        air_speed: character.air_speed,
    });
    let jump_kind = |kind: JumpKind| match kind {
        JumpKind::Jump | JumpKind::OutOfWater => AirKind::Jump,
        JumpKind::LongJump => AirKind::LongJump,
    };

    match character.locomotion {
        LocomotionState::OnFoot(_) => {
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
        LocomotionState::Hop(_) => PoseState::Hop { takeoff },
        LocomotionState::Jump(Jump { kind, phase, .. }) => match current {
            PoseState::Launching { .. } if !current.timer_expired() => current,
            PoseState::Launching { kind, takeoff, .. } | PoseState::Airborne { kind, takeoff } => {
                PoseState::Airborne { kind, takeoff }
            }
            PoseState::Grounded { .. } | PoseState::Hop { .. } | PoseState::Swimming { .. } => {
                match phase {
                    TakeoffPhase::Launch { .. } => PoseState::Launching {
                        kind: jump_kind(kind),
                        takeoff,
                        t: 0.0,
                    },
                    TakeoffPhase::Flight => PoseState::Airborne {
                        kind: jump_kind(kind),
                        takeoff,
                    },
                }
            }
        },
        LocomotionState::Fall(_) => PoseState::Airborne {
            kind: AirKind::Fall,
            takeoff,
        },
        // In the water, the stroke follows the intent the way the gait does
        // on land: going somewhere is a crawl, staying put is a tread.
        LocomotionState::Swim(_) => PoseState::Swimming {
            stroke: if target.direction.magnitude_squared() > 0.001 {
                Stroke::Crawl
            } else {
                Stroke::Tread
            },
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::animation::state::LandingSquash;
    use crate::character::locomotion::{AirSteering, Fall, Hop, OnFoot, Swim};

    /// The pose `state` maps to from `current`, walking ahead.
    fn pose_in(state: LocomotionState, current: PoseState) -> PoseState {
        let character = CharacterState {
            locomotion: state,
            ..CharacterState::default()
        };
        next_pose_state(
            current,
            &character,
            &CharacterIntent::default(),
            5.0,
            0.1,
            Vector3::z(),
        )
    }

    fn hop(phase: TakeoffPhase) -> LocomotionState {
        LocomotionState::Hop(Hop {
            phase,
            landing: None,
            rise_held: None,
        })
    }

    fn jump(phase: TakeoffPhase) -> LocomotionState {
        LocomotionState::Jump(Jump {
            kind: JumpKind::Jump,
            phase,
            steering: AirSteering::Responsive,
        })
    }

    const WALK: PoseState = PoseState::Grounded { gait: Gait::Walk };

    /// A jump out of the water flies at once: there is no ground under a
    /// swimmer to wind up against.
    #[test]
    fn a_jump_out_of_the_water_flies_without_a_wind_up() {
        let next = pose_in(
            LocomotionState::Jump(Jump::out_of_water()),
            PoseState::Swimming {
                stroke: Stroke::Crawl,
            },
        );
        assert!(
            matches!(
                next,
                PoseState::Airborne {
                    kind: AirKind::Jump,
                    ..
                }
            ),
            "{next:?}"
        );
    }

    /// A hop a plan took off on is a running leap: the capsule is already
    /// rising as it begins, so a wind-up crouch would sink the body into the
    /// gap it is clearing.
    #[test]
    fn a_planned_takeoff_leaps_without_a_wind_up() {
        for phase in [TakeoffPhase::launch(0.1), TakeoffPhase::Flight] {
            let next = pose_in(hop(phase), WALK);
            assert!(matches!(next, PoseState::Hop { .. }), "{next:?}");
        }
    }

    /// A jump from the ground winds up for the wind-up's own duration, even
    /// once the body is in the air, and then flies; one from grace is already
    /// in the air.
    #[test]
    fn a_jump_winds_up_for_its_own_duration_then_flies() {
        let mut pose = pose_in(jump(TakeoffPhase::launch(0.2)), WALK);
        assert!(
            matches!(
                pose,
                PoseState::Launching {
                    kind: AirKind::Jump,
                    ..
                }
            ),
            "{pose:?}"
        );
        let (dt, wind_up) = (1.0 / 60.0, AirKind::Jump.launch_duration());
        let mut t = 0.0;
        while t + dt < wind_up {
            pose = pose_in(jump(TakeoffPhase::Flight), pose);
            assert!(
                matches!(pose, PoseState::Launching { .. }),
                "at {t:.3} s: {pose:?}"
            );
            pose = PoseState::Launching {
                kind: AirKind::Jump,
                takeoff: Takeoff::default(),
                t: t + dt,
            };
            t += dt;
        }
        pose = pose_in(
            jump(TakeoffPhase::Flight),
            PoseState::Launching {
                kind: AirKind::Jump,
                takeoff: Takeoff::default(),
                t: wind_up,
            },
        );
        assert!(
            matches!(
                pose,
                PoseState::Airborne {
                    kind: AirKind::Jump,
                    ..
                }
            ),
            "{pose:?}"
        );

        let from_grace = pose_in(jump(TakeoffPhase::Flight), WALK);
        assert!(matches!(
            from_grace,
            PoseState::Airborne {
                kind: AirKind::Jump,
                ..
            }
        ));
    }

    /// A Hop that misses its landing falls, and its legs come off the
    /// placer's: the fall pose hangs them from the hips.
    #[test]
    fn a_missed_hop_falls_with_the_hops_takeoff() {
        let takeoff = Takeoff {
            facing: Vector3::x(),
            air_speed: 6.0,
        };
        let next = pose_in(LocomotionState::Fall(Fall), PoseState::Hop { takeoff });
        let PoseState::Airborne {
            kind,
            takeoff: kept,
        } = next
        else {
            panic!("{next:?}");
        };
        assert_eq!(kind, AirKind::Fall);
        assert_eq!(kept.air_speed, 6.0);
    }

    /// The arms swing on foot and through a Hop, brace in a jump or a fall
    /// and stroke in the water.
    #[test]
    fn the_arms_follow_the_locomotion_state() {
        let cases = [
            (
                LocomotionState::OnFoot(OnFoot::default()),
                UpperState::Swinging,
            ),
            (hop(TakeoffPhase::Flight), UpperState::Swinging),
            (jump(TakeoffPhase::Flight), UpperState::Braced),
            (LocomotionState::Fall(Fall), UpperState::Braced),
            (LocomotionState::Swim(Swim), UpperState::Stroking),
        ];
        for (state, arms) in cases {
            let mapped = map_upper_state(&ArmState::Idle, &state);
            assert_eq!(mapped.transition_key(), arms.transition_key(), "{state:?}");
        }
    }

    /// Coming down onto the feet squashes by what landed: a Hop lightly, a
    /// long jump heavily. The squash plays out on the feet and ends with
    /// them.
    #[test]
    fn a_landing_squashes_by_what_landed() {
        let config = CharacterRigConfig::default();
        let mut animator = CharacterAnimator::new(config, Point3::new(0.0, 1.0, 0.0), 1.0, 0.0);
        let drop = |squash: LandingSquash| {
            -squash
                .applied_to(PoseFragment::default())
                .pelvis_offset
                .unwrap()
                .y
        };
        let mut squash_from = |pose: PoseState| {
            animator.pose_state = pose;
            animator.play_transients(&WALK, 1.0 / 60.0);
            animator.state.landing.expect("a landing squashes")
        };
        let leap = squash_from(PoseState::Hop {
            takeoff: Takeoff::default(),
        });
        let long_jump = squash_from(PoseState::Airborne {
            kind: AirKind::LongJump,
            takeoff: Takeoff::default(),
        });
        assert!(drop(leap) > 0.0 && drop(leap) < drop(long_jump));

        animator.pose_state = WALK;
        let mut frames = 0;
        while animator.state.landing.is_some() {
            animator.play_transients(&WALK, 1.0 / 60.0);
            frames += 1;
            assert!(frames < 60, "the squash plays out");
        }
        animator.state.landing = Some(long_jump);
        animator.play_transients(
            &PoseState::Hop {
                takeoff: Takeoff::default(),
            },
            1.0 / 60.0,
        );
        assert_eq!(animator.state.landing, None, "it ends with the feet");
    }

    /// A flight goes into the water as hard as it came down, though the water
    /// has slowed the body by the time it takes it.
    #[test]
    fn a_fall_into_the_water_plunges_as_hard_as_it_came_down() {
        let config = CharacterRigConfig::default();
        let mut animator = CharacterAnimator::new(config, Point3::new(0.0, 1.0, 0.0), 1.0, 0.0);
        let falling = PoseState::Airborne {
            kind: AirKind::Fall,
            takeoff: Takeoff::default(),
        };
        let tread = PoseState::Swimming {
            stroke: Stroke::Tread,
        };
        for vy in [-2.0, -5.0, -3.0] {
            animator.state.velocity.y = vy;
            animator.play_transients(&falling, 1.0 / 60.0);
            animator.pose_state = falling;
        }
        animator.state.velocity.y = -2.0;
        animator.play_transients(&tread, 1.0 / 60.0);
        let entry = animator.state.water_entry.expect("a plunge");
        assert_eq!(entry.strength, WaterEntry::at(5.0).unwrap().strength);
    }

    /// A jump out of the water kicks and presses its way out, quickly
    /// blended in, and plays until it is done or the body is down; swimming
    /// off an edge does not.
    #[test]
    fn a_jump_out_of_the_water_kicks_its_way_out() {
        let config = CharacterRigConfig::default();
        let mut animator = CharacterAnimator::new(config, Point3::new(0.0, 1.0, 0.0), 1.0, 0.0);
        let tread = PoseState::Swimming {
            stroke: Stroke::Tread,
        };
        let air = |kind| PoseState::Airborne {
            kind,
            takeoff: Takeoff::default(),
        };
        let dt = 1.0 / 60.0;

        animator.pose_state = tread;
        animator.play_transients(&air(AirKind::Fall), dt);
        assert!(animator.state.water_exit.is_none());
        assert_eq!(
            blend_duration(&tread, &air(AirKind::Fall)),
            SWIM_BLEND_DURATION
        );

        animator.play_transients(&air(AirKind::Jump), dt);
        assert!(animator.state.water_exit.is_some());
        assert_eq!(
            blend_duration(&tread, &air(AirKind::Jump)),
            TRANSITION_BLEND_DURATION
        );
        animator.pose_state = air(AirKind::Jump);
        let frames = (WaterExit::DURATION / dt).ceil() as usize;
        for _ in 0..frames {
            animator.play_transients(&air(AirKind::Jump), dt);
        }
        assert!(animator.state.water_exit.is_none());

        animator.pose_state = tread;
        animator.play_transients(&air(AirKind::Jump), dt);
        animator.pose_state = air(AirKind::Jump);
        animator.play_transients(&WALK, dt);
        assert!(animator.state.water_exit.is_none());
    }

    /// One-frame ground-contact losses (seams) put locomotion in grace; the
    /// pose must stay Grounded, as the legs keep stepping through it.
    #[test]
    fn grace_keeps_grounded_pose() {
        let mut character = CharacterState::default();
        character.locomotion = LocomotionState::OnFoot(OnFoot { grace: Some(0.1) });
        let mut target = CharacterIntent::default();
        target.direction = Vector3::new(0.0, 0.0, 1.0);

        let next = next_pose_state(
            PoseState::Grounded { gait: Gait::Walk },
            &character,
            &target,
            2.0,
            0.1,
            Vector3::new(0.0, 0.0, 1.0),
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
