//! Wall-clock time per ECS system, per frame.
//!
//! ```text
//!   TimedDispatcherBuilder ─wraps every system─▶ Timed<S> ─record─▶ SystemTimings
//!                                                                      │ take, once a frame
//!                                                           run_frame ◀┘
//! ```

mod builder;
mod timed;
mod timings;

pub use builder::TimedDispatcherBuilder;
pub use timings::{SystemTime, SystemTimings};
