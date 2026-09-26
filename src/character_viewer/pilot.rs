//! A brain that reads a script: the character_viewer's stand-in for the
//! keyboard.

use nalgebra::Vector3;

use crate::anim_viewer::{Beat, Script};
use crate::character::CharacterIntent;

/// Turns a script into one `CharacterIntent` per frame.
///
/// It fills exactly what `PlayerInputSystem` fills, edges included, so the
/// character is driven through the same seam a creature's brain uses and the
/// systems downstream cannot tell it from a player.
pub struct Pilot<'a> {
    script: &'a Script,
    /// Accumulated rotation of a turning beat's direction, in radians.
    turn: f32,
    previous: Option<&'a Beat>,
}

impl<'a> Pilot<'a> {
    pub fn new(script: &'a Script) -> Self {
        Self {
            script,
            turn: 0.0,
            previous: None,
        }
    }

    /// The intent for the frame at `time`, and the beat it belongs to. `None`
    /// once the script has run out.
    pub fn intent(&mut self, time: f32, dt: f32) -> Option<(CharacterIntent, &'a Beat)> {
        let (beat, first_frame) = self.script.at(time, dt)?;
        self.turn += beat.turn_rate * dt;
        let was_crouching = self.previous.map_or(false, |b| b.crouch);
        let was_jumping = self.previous.map_or(false, |b| b.jump);
        let released = was_jumping && (first_frame && !beat.jump);
        self.previous = Some(beat);

        let intent = CharacterIntent {
            direction: rotate_y(beat.direction, self.turn),
            jump: beat.jump && first_frame,
            jump_held: beat.jump,
            jump_released: released,
            crouch: beat.crouch,
            crouch_just_pressed: beat.crouch && !was_crouching,
            sprint: beat.sprint,
            ..Default::default()
        };
        Some((intent, beat))
    }
}

/// Rotate a horizontal direction about +y.
fn rotate_y(direction: Vector3<f32>, angle: f32) -> Vector3<f32> {
    let (sin, cos) = angle.sin_cos();
    Vector3::new(
        direction.x * cos + direction.z * sin,
        direction.y,
        -direction.x * sin + direction.z * cos,
    )
}
