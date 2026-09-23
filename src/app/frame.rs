//! One frame of the game, in the order its systems must run.

use std::time::{Duration, Instant};

use specs::{Dispatcher, World, WorldExt};

use crate::debug::DebugLog;

use super::event_handler::clear_frame_state;
use super::system_timing::{SystemTime, SystemTimings};

/// How many of a frame's slowest systems F3 prints.
const LOGGED_SYSTEMS: usize = 8;

/// Wall-clock time of a frame's two halves, and of every system in them.
#[derive(Debug, Clone, Default)]
pub struct FrameTiming {
    /// Every parallel system — input, AI, physics, terrain, particles — and
    /// the maintain that applies what they spawned.
    pub simulate: Duration,
    /// The thread-local systems, which is the render, and the maintain after.
    pub render: Duration,
    /// Every system's own time, slowest first.
    pub systems: Vec<SystemTime>,
}

/// Run one frame: simulate, apply, draw, clear.
///
/// The caller advances `Time` first; everything after that is here, so the
/// game and an offline harness cannot run the frame in different orders.
pub fn run_frame(world: &mut World, dispatcher: &mut Dispatcher) -> FrameTiming {
    let start = Instant::now();

    // `dispatch` would run the thread-local systems too, and the
    // render is one of them — calling it here and again below drew
    // the frame twice. Run the parallel systems on their own, so
    // that the explicit `dispatch_thread_local` below is the
    // frame's only render.
    dispatcher.dispatch_par(world);

    // Before rendering, not after. `RenderSystem` is thread-local
    // and draws by joining over storages, so an entity a system
    // created through `LazyUpdate` this frame is invisible until
    // `maintain` applies it. Fracture is where that showed: the
    // compound's model drops a piece the instant it breaks off,
    // while the piece's own entity arrived a frame later, so a
    // structure blinked out at the moment it came apart.
    world.maintain();
    let simulated = Instant::now();

    dispatcher.dispatch_thread_local(world);
    world.maintain();
    let rendered = Instant::now();

    let systems = world.read_resource::<SystemTimings>().take();
    log_slowest_systems(&systems, &mut world.write_resource::<DebugLog>());

    clear_frame_state(world);

    FrameTiming {
        simulate: simulated - start,
        render: rendered - simulated,
        systems,
    }
}

/// The frame's slowest systems, ranked, so a slow frame in the game says which
/// system it was without a harness.
fn log_slowest_systems(systems: &[SystemTime], debug_log: &mut DebugLog) {
    for (rank, system) in systems.iter().take(LOGGED_SYSTEMS).enumerate() {
        debug_log.add(
            format!("Systems/Slowest/{}", rank + 1),
            format!(
                "{} {:.3} ms",
                system.name,
                system.elapsed.as_secs_f64() * 1000.0
            ),
        );
    }
}
