use winit::{
    event::{DeviceEvent, ElementState, Event, KeyEvent, MouseButton, WindowEvent},
    event_loop::EventLoopWindowTarget,
    keyboard::{KeyCode, PhysicalKey},
    window::{CursorGrabMode, Window},
};

use specs::{World, WorldExt};

use super::debug_snapshot::{request_snapshot, write_finished_snapshot};
use super::placer_recording::toggle_nearest_recording;
use super::slow_motion::next_speed;
use crate::debug::{DebugLines, DebugLog, DebugOverlays};
use crate::input::InputState;
use crate::rendering::renderer::Renderer;
use crate::time::Time;

/// Result of handling an event
pub enum EventResult {
    Continue,
    Exit,
}

/// Handles window and input events, updating the ECS world state
pub struct EventHandler;

impl EventHandler {
    pub fn handle(
        event: Event<()>,
        window: &Window,
        world: &mut World,
        elwt: &EventLoopWindowTarget<()>,
    ) -> EventResult {
        match event {
            Event::WindowEvent {
                event: WindowEvent::CloseRequested,
                ..
            } => {
                log::info!("Close requested");
                elwt.exit();
                EventResult::Exit
            }

            Event::WindowEvent {
                event:
                    WindowEvent::KeyboardInput {
                        event:
                            KeyEvent {
                                physical_key: PhysicalKey::Code(key_code),
                                state,
                                repeat,
                                ..
                            },
                        ..
                    },
                ..
            } => Self::handle_keyboard(key_code, state, repeat, window, world, elwt),

            Event::WindowEvent {
                event: WindowEvent::MouseInput { state, button, .. },
                ..
            } => Self::handle_mouse_button(button, state, window, world),

            // The output and everything sized to it are rebuilt before the
            // next frame; a minimised window (no area) draws nothing until it
            // comes back.
            Event::WindowEvent {
                event: WindowEvent::Resized(size),
                ..
            } => {
                world
                    .write_resource::<Renderer>()
                    .request_resize(size.width, size.height);
                EventResult::Continue
            }

            // Focus loss strands anything currently held: the matching release
            // is delivered to whoever took focus, never to us.
            Event::WindowEvent {
                event: WindowEvent::Focused(false),
                ..
            } => {
                let mut input = world.write_resource::<InputState>();
                input.release_all();
                EventResult::Continue
            }

            Event::DeviceEvent {
                event: DeviceEvent::MouseMotion { delta },
                ..
            } => Self::handle_mouse_motion(delta, world),

            Event::AboutToWait => EventResult::Continue,

            Event::WindowEvent {
                event: WindowEvent::RedrawRequested,
                ..
            } => EventResult::Continue,

            _ => EventResult::Continue,
        }
    }

    fn handle_keyboard(
        key_code: KeyCode,
        state: ElementState,
        repeat: bool,
        window: &Window,
        world: &mut World,
        elwt: &EventLoopWindowTarget<()>,
    ) -> EventResult {
        if key_code == KeyCode::Escape && state == ElementState::Pressed {
            let captured = world.read_resource::<InputState>().is_mouse_captured();
            match escape_action(captured, repeat) {
                EscapeAction::ReleaseMouse => set_mouse_captured(window, world, false),
                EscapeAction::Quit => {
                    log::info!("Escape pressed - exiting");
                    elwt.exit();
                    return EventResult::Exit;
                }
                EscapeAction::Nothing => {}
            }
            return EventResult::Continue;
        }

        let mut input = world.write_resource::<InputState>();
        input.handle_keyboard_input(key_code, state);
        EventResult::Continue
    }

    fn handle_mouse_button(
        button: MouseButton,
        state: ElementState,
        window: &Window,
        world: &mut World,
    ) -> EventResult {
        if button == MouseButton::Left && state == ElementState::Pressed {
            let input = world.read_resource::<InputState>();
            let is_captured = input.is_mouse_captured();
            drop(input);

            if !is_captured {
                set_mouse_captured(window, world, true);
                return EventResult::Continue;
            }
        }

        let mut input = world.write_resource::<InputState>();
        input.handle_mouse_button(button, state);
        EventResult::Continue
    }

    fn handle_mouse_motion(delta: (f64, f64), world: &mut World) -> EventResult {
        let mut input = world.write_resource::<InputState>();
        if input.is_mouse_captured() {
            input.handle_mouse_motion(delta.0, delta.1);
        }
        EventResult::Continue
    }
}

/// What a press of Escape does.
#[derive(Debug, PartialEq, Eq)]
enum EscapeAction {
    /// Give the mouse back to the system.
    ReleaseMouse,
    /// Leave the game.
    Quit,
    /// Nothing.
    Nothing,
}

/// The first press releases a captured mouse and the next quits. Only a fresh
/// press counts: held, the system's key repeat would deliver a second press
/// after the first released the mouse, and that one would quit.
fn escape_action(mouse_captured: bool, repeat: bool) -> EscapeAction {
    match (repeat, mouse_captured) {
        (true, _) => EscapeAction::Nothing,
        (false, true) => EscapeAction::ReleaseMouse,
        (false, false) => EscapeAction::Quit,
    }
}

/// Whether this frame's debug log is wanted: F3 was pressed on it. False in a
/// world with no input, as an offline harness's may be.
pub fn debug_log_requested(world: &World) -> bool {
    world
        .try_fetch::<InputState>()
        .is_some_and(|input| input.is_key_just_pressed(KeyCode::F3))
}

/// Clears per-frame state after systems have processed
pub fn clear_frame_state(world: &mut World) {
    let should_print_debug = debug_log_requested(world);
    let (should_write_recording, should_slow_down) = {
        let input = world.read_resource::<InputState>();
        (
            input.is_key_just_pressed(KeyCode::F4),
            input.is_key_just_pressed(KeyCode::F9),
        )
    };

    // F4 starts the foot placer's recording, and writes it on the next press.
    // Recording keeps a ring in memory, so the second press is the one to make
    // once the feet have just done the thing you want explained — the run-up
    // to it is already held.
    if should_write_recording {
        toggle_nearest_recording(world);
    }

    if should_slow_down {
        next_speed(&mut world.write_resource::<Time>());
    }

    // F3 prints the debug log and keeps it, with the next frame's picture.
    write_finished_snapshot(world);
    if should_print_debug {
        let log = world.read_resource::<DebugLog>().text();
        request_snapshot(world, log);
    }
    {
        let mut debug_log = world.write_resource::<DebugLog>();
        debug_log.print_and_clear(should_print_debug);
    }

    {
        let mut input = world.write_resource::<InputState>();
        input.begin_frame();
    }
    {
        let mut debug = world.write_resource::<DebugLines>();
        debug.clear();
    }
    {
        let mut overlays = world.write_resource::<DebugOverlays>();
        overlays.clear();
    }
}

/// Sets the mouse capture state for the window
pub fn set_mouse_captured(window: &Window, world: &mut World, captured: bool) {
    let result = if captured {
        window
            .set_cursor_grab(CursorGrabMode::Confined)
            .or_else(|_| window.set_cursor_grab(CursorGrabMode::Locked))
    } else {
        window.set_cursor_grab(CursorGrabMode::None)
    };

    if let Err(e) = result {
        log::warn!("Failed to set cursor grab mode: {}", e);
    }

    window.set_cursor_visible(!captured);

    let mut input = world.write_resource::<InputState>();
    if !captured {
        // Releasing the cursor hands the next events to the desktop, so treat
        // it the same as losing focus rather than leaving the player holding
        // whatever they had down at the time.
        input.release_all();
    }
    input.set_mouse_captured(captured);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Holding Escape releases the mouse and stops there.
    #[test]
    fn a_held_escape_releases_the_mouse_and_does_not_quit() {
        assert_eq!(escape_action(true, false), EscapeAction::ReleaseMouse);
        assert_eq!(escape_action(false, true), EscapeAction::Nothing);
        assert_eq!(escape_action(false, false), EscapeAction::Quit);
    }
}
