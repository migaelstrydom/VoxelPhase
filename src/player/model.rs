//! Player model definition and animation state.
//!
//! This module provides compile-time safe types for the player's model parts
//! and procedural animation state.

use std::f32::consts::{PI, TAU};

use nalgebra::{UnitQuaternion, Vector3};
use specs::{Component, VecStorage};

use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::model::{MeshPrimitive, Model, ModelPart, Transform};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;

/// Known parts of the player model - compile-time checked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PlayerPart {
    Body = 0,
    Nose = 1,
    LeftEye = 2,
    RightEye = 3,
}

impl PlayerPart {
    pub const COUNT: usize = 4;

    pub fn all() -> [PlayerPart; Self::COUNT] {
        [Self::Body, Self::Nose, Self::LeftEye, Self::RightEye]
    }
}

/// Material IDs for the player model.
pub struct PlayerMaterials {
    pub body: MaterialId,
    pub nose: MaterialId,
    pub eye: MaterialId,
}

/// Configuration for building the player model.
#[derive(Clone, Debug)]
pub struct PlayerModelConfig {
    pub body_radius: f32,
    pub body_colour: Colour,
    pub nose_colour: Colour,
    pub eye_colour: Colour,
}

impl Default for PlayerModelConfig {
    fn default() -> Self {
        Self {
            body_radius: 0.5,
            body_colour: Colour::RED,
            nose_colour: Colour::GREEN,
            eye_colour: Colour::WHITE, // White so texture shows through
        }
    }
}

/// Procedural animation state for the player.
///
/// This is player-specific, not a generic animation system.
/// It handles breathing, blinking, and nose compression animations.
#[derive(Component, Clone, Debug)]
#[storage(VecStorage)]
pub struct PlayerAnimationState {
    /// Breathing cycle phase [0, 1)
    pub breathe_phase: f32,
    /// Time between breaths
    pub breathe_period: f32,

    /// Time until next blink check
    pub blink_timer: f32,
    /// Time between blinks
    pub blink_interval: f32,
    /// How long a blink lasts
    pub blink_duration: f32,
    /// Currently blinking
    pub is_blinking: bool,

    /// Nose compression from collisions [0.5, 1.0]
    pub nose_scale: f32,
}

impl Default for PlayerAnimationState {
    fn default() -> Self {
        Self {
            breathe_phase: 0.0,
            breathe_period: 2.0, // 2 seconds per breath cycle

            blink_timer: 3.0,
            blink_interval: 3.0, // Blink every 3 seconds
            blink_duration: 0.15,
            is_blinking: false,

            nose_scale: 1.0,
        }
    }
}

impl PlayerAnimationState {
    /// Update the animation state with the given delta time.
    pub fn update(&mut self, delta_time: f32) {
        // Update breathing
        self.breathe_phase += delta_time / self.breathe_period;
        if self.breathe_phase >= 1.0 {
            self.breathe_phase -= 1.0;
        }

        // Update blinking
        self.blink_timer -= delta_time;
        if self.blink_timer <= 0.0 {
            if self.is_blinking {
                // End blink
                self.is_blinking = false;
                self.blink_timer = self.blink_interval;
            } else {
                // Start blink
                self.is_blinking = true;
                self.blink_timer = self.blink_duration;
            }
        }

        // Nose scale recovers towards 1.0
        self.nose_scale += (1.0 - self.nose_scale) * delta_time * 2.0;
    }

    /// Compress the nose (called on collision).
    #[allow(dead_code)]
    pub fn compress_nose(&mut self, amount: f32) {
        self.nose_scale = (self.nose_scale - amount).max(0.5);
    }

    /// Get the transform modifier for a specific part.
    pub fn part_transform(&self, part: PlayerPart) -> Transform {
        match part {
            PlayerPart::Body => {
                // Breathing: oscillate scale on X axis
                let breathe_scale = 1.0 + 0.03 * (self.breathe_phase * TAU).sin();
                Transform {
                    scale: Vector3::new(breathe_scale, 1.0, 1.0),
                    ..Default::default()
                }
            }
            PlayerPart::Nose => {
                // Nose compression: squish in Z, expand in X/Y
                let inv = 1.0 / self.nose_scale.max(0.5);
                Transform {
                    scale: Vector3::new(inv, inv, self.nose_scale),
                    ..Default::default()
                }
            }
            PlayerPart::LeftEye | PlayerPart::RightEye => {
                // Blinking: flip 180° around X axis
                if self.is_blinking {
                    Transform {
                        rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI),
                        ..Default::default()
                    }
                } else {
                    Transform::default()
                }
            }
        }
    }
}

/// Build the player model given pre-registered materials.
pub fn build_player_model(config: &PlayerModelConfig, materials: &PlayerMaterials) -> Model {
    let r = config.body_radius;

    let parts = vec![
        // Body (index 0)
        ModelPart::new(vec![MeshPrimitive {
            vertices: generate_sphere_vertices(r, 30, 21, config.body_colour),
            indices: generate_sphere_indices(30, 21),
            material: materials.body,
        }]),
        // Nose (index 1)
        ModelPart::new(vec![MeshPrimitive {
            vertices: generate_sphere_vertices(0.25, 24, 15, config.nose_colour),
            indices: generate_sphere_indices(24, 15),
            material: materials.nose,
        }])
        .with_transform(Transform::translation(0.0, 0.0, r)),
        // Left Eye (index 2)
        ModelPart::new(vec![MeshPrimitive {
            vertices: generate_sphere_vertices(0.2, 16, 10, config.eye_colour),
            indices: generate_sphere_indices(16, 10),
            material: materials.eye,
        }])
        .with_transform(Transform {
            translation: Vector3::new(-0.175, 0.2, r * 0.8),
            rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -PI / 2.0),
            scale: Vector3::new(1.0, 1.0, 1.0),
        }),
        // Right Eye (index 3)
        ModelPart::new(vec![MeshPrimitive {
            vertices: generate_sphere_vertices(0.2, 16, 10, config.eye_colour),
            indices: generate_sphere_indices(16, 10),
            material: materials.eye,
        }])
        .with_transform(Transform {
            translation: Vector3::new(0.175, 0.2, r * 0.8),
            rotation: UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -PI / 2.0),
            scale: Vector3::new(1.0, 1.0, 1.0),
        }),
    ];

    Model::flat(parts)
}
