//! Biped animation controller.
//!
//! This is the single source of truth for biped animation.
//! All animation logic flows through this controller.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

use super::config::BipedConfig;
use super::gait::GaitCycle;
use super::skeleton::{generate_biped_mesh, BipedSkeleton};
use super::state::{BipedState, LocomotionMode};
use super::stride_wheel;
use crate::rendering::vertex::Vertex;
use crate::sensing::{ContactCandidate, Probe};

/// Probe tags used by the biped controller.
pub mod probe_tags {
    pub const FOOT_LEFT: u32 = 0;
    pub const FOOT_RIGHT: u32 = 1;
}

/// The biped animation controller.
///
/// Owns all biped-specific state and logic:
/// - Configures probes based on animation state
/// - Processes probe results
/// - Updates gait and foot positions
/// - Computes skeleton pose
#[derive(Component)]
#[storage(VecStorage)]
pub struct BipedController {
    pub config: BipedConfig,
    pub state: BipedState,
    pub skeleton: BipedSkeleton,
    pub gait: GaitCycle,

    // Mesh caching
    cached_vertices: Vec<Vertex>,
    cached_indices: Vec<u32>,
}

impl BipedController {
    /// Create a new biped controller at the given position.
    pub fn new(config: BipedConfig, pelvis_position: Point3<f32>) -> Self {
        let leg_length = config.leg_length();
        let facing = Vector3::new(0.0, 0.0, 1.0);

        let state = BipedState::new(pelvis_position, leg_length);
        let skeleton = BipedSkeleton::new(&config, pelvis_position, facing);
        let gait = GaitCycle::walking(
            config.standing_height(),
            config.stride_length,
            config.step_height,
        );

        Self {
            config,
            state,
            skeleton,
            gait,
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
        let probe_radius = self.config.probe_radius;

        let mut probes = Vec::with_capacity(2);

        // Left foot probe
        let left_probe = self.configure_foot_probe(
            probe_tags::FOOT_LEFT,
            pelvis_position + left_hip_offset,
            &self.state.left,
            stride_wheel::is_swinging(self.state.wheel_angle, stride_wheel::LEFT_PHASE),
            probe_length,
            probe_radius,
        );
        probes.push(left_probe);

        // Right foot probe
        let right_probe = self.configure_foot_probe(
            probe_tags::FOOT_RIGHT,
            pelvis_position + right_hip_offset,
            &self.state.right,
            stride_wheel::is_swinging(self.state.wheel_angle, stride_wheel::RIGHT_PHASE),
            probe_length,
            probe_radius,
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
        radius: f32,
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
            radius,
        }
    }

    /// Process probe results and update animation.
    pub fn update(
        &mut self,
        dt: f32,
        pelvis_position: Point3<f32>,
        yaw: f32,
        speed: f32,
        contacts: &[ContactCandidate],
    ) {
        // Update facing direction
        let facing = Vector3::new(yaw.sin(), 0.0, yaw.cos());
        self.state.facing = facing;

        // Sync pelvis position from physics
        self.state.pelvis_position = pelvis_position;

        // Process probe contacts
        self.process_contacts(contacts);
        let has_ground_contact =
            self.state.left.ground_contact.is_some() || self.state.right.ground_contact.is_some();

        // Determine locomotion mode
        let new_mode = self.determine_locomotion_mode(speed, has_ground_contact);
        self.state.set_mode(new_mode, dt);

        // Update based on mode
        match self.state.mode {
            LocomotionMode::Idle => {
                stride_wheel::handle_idle(&mut self.state, &self.config);
            }
            LocomotionMode::Walking => {
                self.update_walking(dt, speed);
            }
            LocomotionMode::Falling => {
                // Keep feet hanging below pelvis
                let right = self.state.facing.cross(&Vector3::y());
                let left_hip = self.state.pelvis_position - right * self.config.hip_width;
                let right_hip = self.state.pelvis_position + right * self.config.hip_width;
                let hang_distance = self.config.standing_height();

                self.state.left.position =
                    Point3::new(left_hip.x, pelvis_position.y - hang_distance, left_hip.z);
                self.state.right.position =
                    Point3::new(right_hip.x, pelvis_position.y - hang_distance, right_hip.z);
            }
        }

        // Update skeleton from state
        self.skeleton.update_from_state(&self.state, &self.config);
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

    /// Determine locomotion mode from speed and contacts.
    fn determine_locomotion_mode(&self, speed: f32, has_ground_contact: bool) -> LocomotionMode {
        if !has_ground_contact {
            LocomotionMode::Falling
        } else if speed < self.config.idle_threshold {
            LocomotionMode::Idle
        } else {
            LocomotionMode::Walking
        }
    }

    /// Update walking animation.
    fn update_walking(&mut self, dt: f32, speed: f32) {
        let radius = self.config.body_radius;

        // Advance stride wheel (radius = body_radius for proper ground contact velocity)
        stride_wheel::advance_wheel(&mut self.state.wheel_angle, speed, dt, radius);

        // Compute hip positions
        let right = self.state.facing.cross(&Vector3::y());
        let left_hip = self.state.pelvis_position - right * self.config.hip_width;
        let right_hip = self.state.pelvis_position + right * self.config.hip_width;

        let facing = self.state.facing;
        let wheel_angle = self.state.wheel_angle;

        // Update left foot from gait cycle
        stride_wheel::update_foot(
            &mut self.state.left,
            &self.gait,
            wheel_angle,
            stride_wheel::LEFT_PHASE,
            Point3::from(left_hip.coords),
            facing,
            -1.0, // Left side
        );

        // Update right foot from gait cycle
        stride_wheel::update_foot(
            &mut self.state.right,
            &self.gait,
            wheel_angle,
            stride_wheel::RIGHT_PHASE,
            Point3::from(right_hip.coords),
            facing,
            1.0, // Right side
        );
    }

    /// Whether the character has any ground contact.
    pub fn is_grounded(&self) -> bool {
        self.state.is_grounded
    }

    /// Set pelvis position (called by collision system).
    pub fn set_pelvis_position(&mut self, position: Point3<f32>) {
        self.state.pelvis_position = position;
        self.skeleton.pelvis = position;
    }

    /// Get mesh vertices and indices for rendering.
    pub fn mesh(&mut self) -> (&[Vertex], &[u32]) {
        self.regenerate_mesh();
        (&self.cached_vertices, &self.cached_indices)
    }

    /// Regenerate the mesh from current skeleton state.
    fn regenerate_mesh(&mut self) {
        let (vertices, indices) = generate_biped_mesh(&self.skeleton, &self.config);
        self.cached_vertices = vertices;
        self.cached_indices = indices;
    }
}
