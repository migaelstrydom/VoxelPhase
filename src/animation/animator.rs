//! Character animation driver.
//!
//! This is the single source of truth for character animation.
//! All animation logic flows through this animator.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

use super::config::CharacterRigConfig;
use super::humanoid::gait::GaitCycle;
use super::humanoid::pose_state::{AirKind, Gait, PoseState, SampleCtx, Takeoff, TickCtx};
use super::humanoid::skeleton::{generate_character_mesh, Skeleton};
use super::humanoid::stride_wheel;
use super::humanoid::upper_state::{UpperSampleCtx, UpperState, UpperTickCtx};
use super::pose::{Crossfade, Linear, PoseFragment};
use super::state::{AnimationState, FootState};
use crate::player::grab::GrabConfig;
use crate::player::{AirSteering, ArmState, LocomotionState, PlayerState, PlayerTargetState};
use crate::rendering::vertex::Vertex;
use crate::sensing::{ContactCandidate, Probe};

/// Minimum movement (metres) required before an idle foot replants.
/// Keeps planting stable against probe jitter.
const IDLE_PLANT_SNAP: f32 = 0.03;

/// Duration of the crossfade when either FSM changes variant kind.
const TRANSITION_BLEND_DURATION: f32 = 0.1;

/// Probe tags used by the character animator.
pub mod probe_tags {
    pub const FOOT_LEFT: u32 = 0;
    pub const FOOT_RIGHT: u32 = 1;
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
    pub gait: GaitCycle,
    pub arm_gait: GaitCycle,

    /// Current lower-body / core pose FSM variant.
    pub pose_state: PoseState,
    /// Current upper-body FSM variant.
    pub upper_state: UpperState,
    /// Active blend for the lower-body FSM. `Some` only during the
    /// short window after a variant-kind change; `None` between transitions.
    pose_crossfade: Option<Crossfade<Linear>>,
    /// Active blend for the upper-body FSM.
    upper_crossfade: Option<Crossfade<Linear>>,
    /// Previous-frame `PlayerState.locomotion`, kept so the driver can
    /// detect variant edges (airborne→grounded, grounded→launching) for
    /// Landing/Launching splicing.
    last_locomotion: LocomotionState,

    // Mesh caching
    cached_vertices: Vec<Vertex>,
    cached_indices: Vec<u32>,
}

impl CharacterAnimator {
    /// Create a new character animator at the given position.
    pub fn new(config: CharacterRigConfig, pelvis_position: Point3<f32>) -> Self {
        let leg_length = config.leg_length();
        let facing = Vector3::new(0.0, 0.0, 1.0);

        let state = AnimationState::new(pelvis_position, leg_length);
        let skeleton = Skeleton::new(&config, pelvis_position, facing);
        let gait = GaitCycle::walking(
            config.standing_height(),
            config.stride_length,
            config.step_height,
        );
        let arm_gait = GaitCycle::arm_swing(config.arm_length(), config.arm_swing_amplitude);

        Self {
            config,
            state,
            skeleton,
            gait,
            arm_gait,
            pose_state: PoseState::Grounded { gait: Gait::Idle },
            upper_state: UpperState::Swinging,
            pose_crossfade: None,
            upper_crossfade: None,
            last_locomotion: LocomotionState::Grounded,
            cached_vertices: Vec::new(),
            cached_indices: Vec::new(),
        }
    }

    /// Configure probes for the next frame based on current animation state.
    pub fn configure_probes(&self, pelvis_position: Point3<f32>, yaw: f32) -> Vec<Probe> {
        let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());
        let right = facing.cross(&Vector3::y());
        let left_hip_offset = -right * self.config.hip_width;
        let right_hip_offset = right * self.config.hip_width;

        let probe_length = self.config.probe_length();

        let mut probes = Vec::with_capacity(2);

        // Left foot probe
        let left_probe = self.configure_foot_probe(
            probe_tags::FOOT_LEFT,
            pelvis_position + left_hip_offset,
            &self.state.left,
            stride_wheel::is_swinging(self.state.wheel_angle, stride_wheel::LEFT_PHASE),
            probe_length,
        );
        probes.push(left_probe);

        // Right foot probe
        let right_probe = self.configure_foot_probe(
            probe_tags::FOOT_RIGHT,
            pelvis_position + right_hip_offset,
            &self.state.right,
            stride_wheel::is_swinging(self.state.wheel_angle, stride_wheel::RIGHT_PHASE),
            probe_length,
        );
        probes.push(right_probe);

        probes
    }

    /// Configure a single foot probe based on foot state.
    fn configure_foot_probe(
        &self,
        tag: u32,
        hip_position: Point3<f32>,
        foot: &super::state::FootState,
        is_swinging: bool,
        length: f32,
    ) -> Probe {
        let (origin, direction) = if is_swinging {
            // Probe ahead toward target with downward bias
            let to_target = foot.position - hip_position;
            (hip_position, to_target.normalize())
        } else {
            // Probe straight down from hip position
            (hip_position, Vector3::new(0.0, -1.0, 0.0))
        };

        Probe {
            tag,
            origin,
            direction,
            length,
        }
    }

    /// Process probe results and update animation.
    pub fn update(
        &mut self,
        dt: f32,
        pelvis_position: Point3<f32>,
        yaw: f32,
        velocity: Vector3<f32>,
        player_state: &PlayerState,
        target: &PlayerTargetState,
        grab_config: &GrabConfig,
        contacts: &[ContactCandidate],
    ) {
        let speed = Vector3::new(velocity.x, 0.0, velocity.z).magnitude();
        let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());

        self.state.facing = facing;
        self.state.pelvis_position = pelvis_position;

        self.process_contacts(contacts);
        let has_ground_contact =
            self.state.left.ground_contact.is_some() || self.state.right.ground_contact.is_some();
        self.state.is_grounded = has_ground_contact;

        // Map PlayerState → next PoseState variant.
        let new_pose = map_pose_state(
            self.pose_state,
            player_state,
            target,
            speed,
            self.config.idle_threshold,
            facing,
        );

        // Re-plant feet when idle. Hysteresis prevents per-frame jitter
        // driven by probe noise.
        if matches!(new_pose, PoseState::Grounded { gait: Gait::Idle }) {
            replant_foot(&mut self.state.left, IDLE_PLANT_SNAP);
            replant_foot(&mut self.state.right, IDLE_PLANT_SNAP);
        }

        // Advance stride wheel only when a gait cycle is actually playing.
        let running_stride = matches!(
            new_pose,
            PoseState::Grounded { gait } if !matches!(gait, Gait::Idle)
        );
        if running_stride {
            stride_wheel::advance_wheel(
                &mut self.state.wheel_angle,
                speed,
                dt,
                self.config.body_radius,
            );
        }

        // Pick the upper variant alongside the new pose variant so Idle-vs-
        // airborne gating sees the just-computed pose.
        let new_upper = map_upper_state(&player_state.arm, new_pose);

        let pose_sample_ctx = SampleCtx {
            rig: &self.config,
            anim: &self.state,
            velocity,
            leg_gait: &self.gait,
        };

        // Snapshot outgoing fragments BEFORE the variant swap, so the
        // crossfade `from` reflects what the old state was producing.
        if self.pose_state.transition_key() != new_pose.transition_key() {
            let from = self.pose_state.sample(&pose_sample_ctx);
            self.pose_crossfade = Some(Crossfade {
                from,
                from_pelvis: pelvis_position,
                to_duration: TRANSITION_BLEND_DURATION,
                elapsed: 0.0,
                policy: Linear,
            });
        }
        let old_cycle = self.pose_state.cycle(&self.state);
        if self.upper_state.transition_key() != new_upper.transition_key() {
            let old_upper_ctx = UpperSampleCtx {
                rig: &self.config,
                anim: &self.state,
                arm_gait: &self.arm_gait,
                grab: grab_config,
                cycle: old_cycle,
            };
            let from = self.upper_state.sample(&old_upper_ctx);
            self.upper_crossfade = Some(Crossfade {
                from,
                from_pelvis: pelvis_position,
                to_duration: TRANSITION_BLEND_DURATION,
                elapsed: 0.0,
                policy: Linear,
            });
        }

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
        let cycle = self.pose_state.cycle(&self.state);
        let upper_sample_ctx = UpperSampleCtx {
            rig: &self.config,
            anim: &self.state,
            arm_gait: &self.arm_gait,
            grab: grab_config,
            cycle,
        };
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

        self.last_locomotion = player_state.locomotion;
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

        if let Some(normal) = self.state.left.ground_normal {
            self.state.left.normal = normal;
        }
        if let Some(normal) = self.state.right.ground_normal {
            self.state.right.normal = normal;
        }
    }

    /// Process contact candidates from probes.
    fn process_contacts(&mut self, contacts: &[ContactCandidate]) {
        // Clear previous contacts
        self.state.left.ground_contact = None;
        self.state.left.ground_normal = None;
        self.state.right.ground_contact = None;
        self.state.right.ground_normal = None;

        // Find best contact for each foot (closest)
        let mut best_left: Option<&ContactCandidate> = None;
        let mut best_right: Option<&ContactCandidate> = None;

        for contact in contacts {
            match contact.tag {
                probe_tags::FOOT_LEFT => {
                    if best_left.map_or(true, |b| contact.distance < b.distance) {
                        best_left = Some(contact);
                    }
                }
                probe_tags::FOOT_RIGHT => {
                    if best_right.map_or(true, |b| contact.distance < b.distance) {
                        best_right = Some(contact);
                    }
                }
                _ => {}
            }
        }

        // Store contacts in state
        if let Some(contact) = best_left {
            self.state.left.ground_contact = Some(contact.point);
            self.state.left.ground_normal = Some(contact.normal);
        }
        if let Some(contact) = best_right {
            self.state.right.ground_contact = Some(contact.point);
            self.state.right.ground_normal = Some(contact.normal);
        }
    }

    /// Whether the character has any ground contact.
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

/// Replant an idle foot at its latest ground contact if the contact has
/// shifted beyond `snap_threshold`. Smaller shifts are ignored to avoid
/// per-frame jitter from probe noise.
fn replant_foot(foot: &mut FootState, snap_threshold: f32) {
    let desired = foot.ground_contact.unwrap_or(foot.position);
    if (desired - foot.planted_position).magnitude() > snap_threshold {
        foot.planted_position = desired;
    }
}

/// Map `ArmState` (+ current `PoseState`) to an `UpperState` variant.
/// `Idle + airborne` → `Braced`; `Idle + grounded` → `Swinging`;
/// `Reaching`/`Holding` mirror their `ArmState` shape.
fn map_upper_state(arm: &ArmState, pose: PoseState) -> UpperState {
    match arm {
        ArmState::Idle => {
            if matches!(pose, PoseState::Grounded { .. }) {
                UpperState::Swinging
            } else {
                UpperState::Braced
            }
        }
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

/// Map `PlayerState` (+ intent) to a `PoseState` variant.
fn map_pose_state(
    prev: PoseState,
    player: &PlayerState,
    target: &PlayerTargetState,
    speed: f32,
    idle_threshold: f32,
    facing: Vector3<f32>,
) -> PoseState {
    let moving = speed > idle_threshold || target.direction.magnitude_squared() > 0.001;

    match player.locomotion {
        LocomotionState::Grounded => {
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
                air_speed: player.air_speed,
            };
            // Preserve any in-flight Launching timer from the previous frame.
            let t = match prev {
                PoseState::Launching { t, .. } => t,
                _ => 0.0,
            };
            PoseState::Launching { kind, takeoff, t }
        }
        LocomotionState::Airborne { steering, .. } => {
            let kind = match (prev, steering) {
                (PoseState::Launching { kind, .. }, _) => kind,
                (PoseState::Airborne { kind, .. }, _) => kind,
                (_, AirSteering::Locked { .. }) => AirKind::LongJump,
                (_, AirSteering::Responsive) => AirKind::Fall,
            };
            let takeoff = match prev {
                PoseState::Launching { takeoff, .. } | PoseState::Airborne { takeoff, .. } => {
                    takeoff
                }
                _ => Takeoff {
                    facing,
                    air_speed: player.air_speed,
                },
            };
            PoseState::Airborne { kind, takeoff }
        }
        LocomotionState::CoyoteTime(_) => {
            let takeoff = Takeoff {
                facing,
                air_speed: player.air_speed,
            };
            PoseState::Airborne {
                kind: AirKind::Fall,
                takeoff,
            }
        }
    }
}
