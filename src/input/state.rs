use std::collections::HashSet;
use winit::event::{ElementState, MouseButton};
use winit::keyboard::KeyCode;

/// Tracks the current state of all input devices.
/// This is an ECS resource that systems can read to query input.
#[derive(Debug, Default)]
pub struct InputState {
    /// Currently pressed keys
    pressed_keys: HashSet<KeyCode>,
    /// Keys that were just pressed this frame
    just_pressed_keys: HashSet<KeyCode>,
    /// Keys that were just released this frame
    just_released_keys: HashSet<KeyCode>,

    /// Mouse movement delta since last frame
    mouse_delta: (f32, f32),
    /// Accumulated mouse delta (for when we process input faster than we consume it)
    mouse_delta_accumulator: (f32, f32),

    /// Currently pressed mouse buttons
    pressed_mouse_buttons: HashSet<MouseButton>,
    /// Mouse buttons just pressed this frame
    just_pressed_mouse_buttons: HashSet<MouseButton>,
    /// Mouse buttons just released this frame
    just_released_mouse_buttons: HashSet<MouseButton>,

    /// Whether the mouse is captured (for relative mouse mode)
    mouse_captured: bool,
}

impl InputState {
    pub fn new() -> Self {
        Self::default()
    }

    // === Frame lifecycle ===

    /// Call at the start of each frame to prepare for new input events.
    /// Clears per-frame state like "just pressed" while keeping held state.
    pub fn begin_frame(&mut self) {
        self.just_pressed_keys.clear();
        self.just_released_keys.clear();
        self.just_pressed_mouse_buttons.clear();
        self.just_released_mouse_buttons.clear();

        // Transfer accumulated mouse delta and reset accumulator
        self.mouse_delta = self.mouse_delta_accumulator;
        self.mouse_delta_accumulator = (0.0, 0.0);
    }

    // === Event handlers (called from event loop) ===

    /// Handle a keyboard event from winit
    pub fn handle_keyboard_input(&mut self, key_code: KeyCode, state: ElementState) {
        match state {
            ElementState::Pressed => {
                if self.pressed_keys.insert(key_code) {
                    // Key was not already pressed, so this is a fresh press
                    self.just_pressed_keys.insert(key_code);
                }
            }
            ElementState::Released => {
                if self.pressed_keys.remove(&key_code) {
                    self.just_released_keys.insert(key_code);
                }
            }
        }
    }

    /// Handle mouse motion event from winit (device event for relative motion)
    pub fn handle_mouse_motion(&mut self, delta_x: f64, delta_y: f64) {
        self.mouse_delta_accumulator.0 += delta_x as f32;
        self.mouse_delta_accumulator.1 += delta_y as f32;
    }

    /// Handle mouse button event from winit
    pub fn handle_mouse_button(&mut self, button: MouseButton, state: ElementState) {
        match state {
            ElementState::Pressed => {
                if self.pressed_mouse_buttons.insert(button) {
                    self.just_pressed_mouse_buttons.insert(button);
                }
            }
            ElementState::Released => {
                if self.pressed_mouse_buttons.remove(&button) {
                    self.just_released_mouse_buttons.insert(button);
                }
            }
        }
    }

    // === Query methods (called from systems) ===

    /// Returns true if the key is currently held down
    #[inline]
    pub fn is_key_pressed(&self, key: KeyCode) -> bool {
        self.pressed_keys.contains(&key)
    }

    /// Returns true if the key was just pressed this frame
    #[inline]
    pub fn is_key_just_pressed(&self, key: KeyCode) -> bool {
        self.just_pressed_keys.contains(&key)
    }

    /// Returns true if the key was just released this frame
    #[inline]
    pub fn is_key_just_released(&self, key: KeyCode) -> bool {
        self.just_released_keys.contains(&key)
    }

    /// Returns the mouse movement delta since last frame
    #[inline]
    pub fn mouse_delta(&self) -> (f32, f32) {
        self.mouse_delta
    }

    /// Returns true if the mouse button is currently held down
    #[inline]
    pub fn is_mouse_button_pressed(&self, button: MouseButton) -> bool {
        self.pressed_mouse_buttons.contains(&button)
    }

    /// Returns true if the mouse button was just pressed this frame
    #[inline]
    pub fn is_mouse_button_just_pressed(&self, button: MouseButton) -> bool {
        self.just_pressed_mouse_buttons.contains(&button)
    }

    /// Returns true if the mouse button was just released this frame
    #[inline]
    pub fn is_mouse_button_just_released(&self, button: MouseButton) -> bool {
        self.just_released_mouse_buttons.contains(&button)
    }

    // === Mouse capture ===

    pub fn set_mouse_captured(&mut self, captured: bool) {
        self.mouse_captured = captured;
    }

    pub fn is_mouse_captured(&self) -> bool {
        self.mouse_captured
    }
}
