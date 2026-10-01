//! Physics performance bench: scripted scenarios (igloo blast, stacks, glass, grenade volleys) on real level terrain, reported per FrameProfile stage.

mod box_stack;
mod glass_shatter;
mod grenade;
mod igloo_blast;
mod record;
mod report;
mod runner;
mod scenario;
mod volley;

pub use crate::perf::Ground;
pub use box_stack::BoxStack;
pub use glass_shatter::GlassShatter;
pub use grenade::WithGrenade;
pub use igloo_blast::IglooBlast;
pub use record::{FrameRecord, PerfRun};
pub use report::{scaling_table, summary, timeline, write_csv, DEFAULT_WINDOW};
pub use runner::{run, RunConfig};
pub use scenario::{Disturbance, PerfScenario};
pub use volley::WithVolley;
