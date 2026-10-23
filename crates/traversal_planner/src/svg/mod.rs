//! Drawing a scene's section (and later its plan) as an SVG: a side view over
//! a top-down strip, sharing the `s` axis, at true aspect.

mod canvas;
mod palette;
mod section_view;

pub use section_view::render_scene;
