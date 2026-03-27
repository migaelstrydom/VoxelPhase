use super::InputState;
use specs::{ReadExpect, System, Write};
use winit::event::MouseButton;
use winit::keyboard::KeyCode;

/// High-level gameplay actions derived from raw input.
/// This decouples game logic from specific key bindings.
#[derive(Debug, Default, Clone)]
pub struct GameplayActions {
    /// Move forward relative to camera direction
    pub move_forward: bool,
    /// Move backward relative to camera direction
    pub move_backward: bool,
    /// Strafe left relative to camera direction
    pub move_left: bool,
    /// Strafe right relative to camera direction
    pub move_right: bool,

    /// Jump (just pressed this frame)
    pub jump: bool,

    /// Camera look delta (mouse movement)
    pub camera_delta: (f32, f32),

    /// Zoom camera in (Q key)
    pub zoom_in: bool,
    /// Zoom camera out (E key)
    pub zoom_out: bool,

    /// Throw action (just pressed this frame). Context-dependent: grab-throw
    /// if holding an object, grenade throw otherwise.
    pub throw: bool,

    /// Grab button held this frame.
    pub grab_held: bool,
    /// Grab button just pressed this frame.
    pub grab_just_pressed: bool,
    /// Grab button just released this frame.
    pub grab_just_released: bool,
}

impl GameplayActions {
    /// Convert raw input state to gameplay actions.
    /// This adapter maps keyboard/mouse inputs to game-specific actions.
    pub fn from_input_state(input: &InputState) -> Self {
        Self {
            // Movement - W/S/A/D or arrow keys
            move_forward: input.is_key_pressed(KeyCode::KeyW)
                || input.is_key_pressed(KeyCode::ArrowUp),
            move_backward: input.is_key_pressed(KeyCode::KeyS)
                || input.is_key_pressed(KeyCode::ArrowDown),
            move_left: input.is_key_pressed(KeyCode::KeyA)
                || input.is_key_pressed(KeyCode::ArrowLeft),
            move_right: input.is_key_pressed(KeyCode::KeyD)
                || input.is_key_pressed(KeyCode::ArrowRight),

            // Jump - Space only (right-click freed for grab)
            jump: input.is_key_just_pressed(KeyCode::Space),

            // Camera control
            camera_delta: input.mouse_delta(),
            zoom_in: input.is_key_pressed(KeyCode::KeyQ),
            zoom_out: input.is_key_pressed(KeyCode::KeyE),

            // Combat actions
            throw: (input.is_key_just_pressed(KeyCode::ShiftLeft)
                || input.is_mouse_button_just_pressed(MouseButton::Left))
                && input.is_mouse_captured(),

            // Grab - Right mouse button
            grab_held: input.is_mouse_button_pressed(MouseButton::Right)
                && input.is_mouse_captured(),
            grab_just_pressed: input.is_mouse_button_just_pressed(MouseButton::Right)
                && input.is_mouse_captured(),
            grab_just_released: input.is_mouse_button_just_released(MouseButton::Right),
        }
    }
}

/// System that converts InputState to GameplayActions each frame.
/// This runs early in the frame so all other systems can read the GameplayActions resource.
pub struct InputActionSystem;

impl<'a> System<'a> for InputActionSystem {
    type SystemData = (ReadExpect<'a, InputState>, Write<'a, GameplayActions>);

    fn run(&mut self, (input, mut actions): Self::SystemData) {
        *actions = GameplayActions::from_input_state(&input);
    }
}
