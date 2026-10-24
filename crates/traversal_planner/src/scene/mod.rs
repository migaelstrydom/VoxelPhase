//! Scene files: a section (authored from shapes or captured in the game), a
//! body state, an intent and the parameters. One format for the CLI, the web
//! tool, the fixtures and the game's capture key.

mod authored;
mod caster;
mod file;

pub use authored::{AuthoredGround, Form, Plane, Shape};
pub use caster::ShapeCaster;
pub use file::{Ground, Intent, Outcome, TraversalScene};
