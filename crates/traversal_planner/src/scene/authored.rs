use serde::{Deserialize, Serialize};

use crate::section::Footing;

/// The outline of a shape seen from the side, in path coordinates. Floors fill
/// down to the ground's base; roofs fill up from their underside.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Form {
    /// A flat-topped block from `s = from` to `s = to`: a plank, a step, a
    /// stone.
    Block {
        from: f32,
        to: f32,
        /// Height of its top at `d = 0`.
        top: f32,
        /// Rise of its top per metre of `d`: a stone tilted across the path.
        #[serde(default)]
        tilt: f32,
    },
    /// A top rising evenly from `start_top` at `from` to `end_top` at `to`.
    Ramp {
        from: f32,
        to: f32,
        start_top: f32,
        end_top: f32,
    },
    /// A ceiling from `from` to `to` with its underside at `bottom`.
    Roof { from: f32, to: f32, bottom: f32 },
}

/// One authored obstacle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Shape {
    pub form: Form,
    /// The shape's extent across the path, `(from, to)` in `d`; `None` spans
    /// the whole strip, so a plain gait scene is a pure side view.
    #[serde(default)]
    pub across: Option<(f32, f32)>,
    pub footing: Footing,
}

/// Ground built from shapes over a flat base.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AuthoredGround {
    /// Height of the open ground between shapes; `None` for a void, as
    /// under stepping stones over a chasm.
    pub base: Option<f32>,
    pub shapes: Vec<Shape>,
}
