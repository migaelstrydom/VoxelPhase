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

    /// Drop every held key and mouse button, reporting each as released.
    ///
    /// The window losing focus is the case this exists for: the OS stops
    /// delivering events to us, so the `Released` half of anything held at that
    /// moment never arrives. Without this the key stays in `pressed_keys`
    /// forever, which pins `is_key_pressed` true *and* — because `just_pressed`
    /// is gated on the set insert succeeding — silently kills every future
    /// "just pressed" edge for that key.
    ///
    /// Held state is surfaced as `just_released` so edge-triggered consumers
    /// (jump cutoff, grab release) unwind rather than being left mid-action.
    pub fn release_all(&mut self) {
        for key in self.pressed_keys.drain() {
            self.just_released_keys.insert(key);
        }
        for button in self.pressed_mouse_buttons.drain() {
            self.just_released_mouse_buttons.insert(button);
        }

        // Motion accumulated while focus was leaving would arrive as one large
        // jump on the next frame the camera reads.
        self.mouse_delta = (0.0, 0.0);
        self.mouse_delta_accumulator = (0.0, 0.0);
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A press whose release never arrives leaves the button in the held set,
    /// and every later press then fails the insert that gates `just_pressed`.
    /// The button is silently dead from that point on — this is what a window
    /// focus change used to do to throw and grab.
    #[test]
    fn a_lost_release_deafens_the_button_to_every_later_press() {
        let mut input = InputState::new();

        input.handle_mouse_button(MouseButton::Left, ElementState::Pressed);
        assert!(input.is_mouse_button_just_pressed(MouseButton::Left));

        // Focus is lost here: winit delivers the release to someone else.
        input.begin_frame();

        input.handle_mouse_button(MouseButton::Left, ElementState::Pressed);
        assert!(
            !input.is_mouse_button_just_pressed(MouseButton::Left),
            "precondition: a stranded press swallows the next press"
        );

        // Recovering the held state restores the edge.
        input.release_all();
        input.begin_frame();
        input.handle_mouse_button(MouseButton::Left, ElementState::Pressed);
        assert!(input.is_mouse_button_just_pressed(MouseButton::Left));
    }

    #[test]
    fn a_lost_release_pins_a_key_down_until_released() {
        let mut input = InputState::new();

        input.handle_keyboard_input(KeyCode::KeyW, ElementState::Pressed);
        input.begin_frame();
        assert!(input.is_key_pressed(KeyCode::KeyW));

        input.release_all();
        assert!(!input.is_key_pressed(KeyCode::KeyW));
    }

    /// Edge-triggered consumers unwind rather than being left mid-action.
    #[test]
    fn release_all_reports_held_input_as_released() {
        let mut input = InputState::new();

        input.handle_keyboard_input(KeyCode::Space, ElementState::Pressed);
        input.handle_mouse_button(MouseButton::Right, ElementState::Pressed);
        input.begin_frame();

        input.release_all();

        assert!(input.is_key_just_released(KeyCode::Space));
        assert!(input.is_mouse_button_just_released(MouseButton::Right));
    }

    #[test]
    fn release_all_drops_motion_accumulated_while_focus_was_leaving() {
        let mut input = InputState::new();
        input.set_mouse_captured(true);

        input.handle_mouse_motion(400.0, -250.0);
        input.release_all();
        input.begin_frame();

        assert_eq!(input.mouse_delta(), (0.0, 0.0));
    }

    /// Nothing held means nothing to unwind — `release_all` on a quiet frame
    /// must not manufacture release edges.
    #[test]
    fn release_all_on_idle_input_is_a_no_op() {
        let mut input = InputState::new();
        input.release_all();

        assert!(!input.is_key_just_released(KeyCode::KeyW));
        assert!(!input.is_mouse_button_just_released(MouseButton::Left));
    }
}
