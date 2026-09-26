//! What one frame of a run looked like, in numbers.

use image::RgbaImage;
use nalgebra::{Point3, UnitQuaternion, Vector3};

/// One frame of the character, as the game left it.
#[derive(Clone, Debug)]
pub struct FrameRecord {
    pub index: usize,
    /// Simulated seconds since the script started.
    pub time: f32,
    pub beat: &'static str,
    /// The motion FSM's state.
    pub locomotion: String,
    /// The lower-body animation FSM's state.
    pub pose: &'static str,
    /// The upper-body animation FSM's state.
    pub upper: String,
    /// Physics body centre.
    pub body: Point3<f32>,
    pub velocity: Vector3<f32>,
    /// How far the capsule's axis leans from vertical, in degrees.
    pub body_pitch: f32,
    /// The pitch the controller is asking for, in degrees.
    pub pitch_asked: f32,
    /// Whether the Support Set is holding the body up.
    pub grounded: bool,
    /// Water surface over the body's column as drawn, if there is water there.
    pub surface: Option<f32>,
    /// The still level of that water, which the motion FSM decides against.
    pub level: Option<f32>,
    /// Floor under that water.
    pub floor: Option<f32>,
    /// The pelvis the rig drew, and its head.
    pub pelvis: Point3<f32>,
    pub head: Point3<f32>,
}

impl FrameRecord {
    pub fn planar_speed(&self) -> f32 {
        Vector3::new(self.velocity.x, 0.0, self.velocity.z).magnitude()
    }

    /// Still depth of the water at the body's column, level to floor.
    pub fn water_depth(&self) -> Option<f32> {
        Some(self.level? - self.floor?)
    }
}

/// Angle between a body's axis and world up, in degrees.
pub fn tilt_degrees(rotation: &UnitQuaternion<f32>) -> f32 {
    (rotation * Vector3::y())
        .y
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

/// A filmstrip frame and what to caption it with.
pub struct Tile {
    pub caption: String,
    pub image: RgbaImage,
}

/// A whole run.
pub struct Take {
    pub scenario: String,
    pub frames: Vec<FrameRecord>,
    pub tiles: Vec<Tile>,
}
