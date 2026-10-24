use serde::{Deserialize, Serialize};

use crate::section::Footing;

/// The outline of a shape seen from the side, in path coordinates. Floors fill
/// down to the ground's base; roofs fill up from their underside; a slab is
/// solid only between its underside and its top.
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
    /// A flat slab from `from` to `to`, its underside at `bottom` and its top
    /// at `top`: a roof a body can stand on, or a floor it can pass under.
    Slab {
        from: f32,
        to: f32,
        bottom: f32,
        top: f32,
    },
}

/// How far a shape with no `across` extends to either side: past any strip.
const UNBOUNDED_ACROSS: f32 = 1.0e3;

/// One authored obstacle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Shape {
    pub form: Form,
    /// The shape's extent across the path, `(from, to)` in `d`; `None` spans
    /// the whole strip, so a plain gait scene is a pure side view.
    #[serde(default)]
    pub across: Option<(f32, f32)>,
    /// Turn of the whole shape about its footprint's centre, in radians,
    /// counterclockwise seen from above (from `+s` toward `+d`): a plank
    /// laid askew. `form` and `across` describe the shape before the turn.
    #[serde(default)]
    pub yaw: f32,
    pub footing: Footing,
}

/// A plane over the `(s, d)` ground plan: `z = z0 + slope_s * s + slope_d * d`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    pub z0: f32,
    pub slope_s: f32,
    pub slope_d: f32,
}

impl Plane {
    pub fn at(&self, s: f32, d: f32) -> f32 {
        self.z0 + self.slope_s * s + self.slope_d * d
    }
}

impl Shape {
    /// The corners of the shape's footprint in `(s, d)`, counterclockwise
    /// seen from above, after the turn.
    pub fn footprint(&self) -> [(f32, f32); 4] {
        let (from, to) = self.extent_along();
        let (left, right) = self.extent_across();
        [(from, left), (to, left), (to, right), (from, right)].map(|corner| self.turned(corner))
    }

    /// The plane of the shape's top, after the turn: solid below it. A roof
    /// has none.
    pub fn top(&self) -> Option<Plane> {
        let (z0, slope_s, slope_d) = match self.form {
            Form::Block { top, tilt, .. } => (top, 0.0, tilt),
            Form::Ramp {
                from,
                to,
                start_top,
                end_top,
            } => {
                let slope = (end_top - start_top) / (to - from);
                (start_top - slope * from, slope, 0.0)
            }
            Form::Slab { top, .. } => (top, 0.0, 0.0),
            Form::Roof { .. } => return None,
        };
        Some(self.turned_plane(z0, slope_s, slope_d))
    }

    /// The plane of the shape's underside, after the turn: solid above it.
    /// Only roofs and slabs have one.
    pub fn underside(&self) -> Option<Plane> {
        match self.form {
            Form::Roof { bottom, .. } | Form::Slab { bottom, .. } => {
                Some(self.turned_plane(bottom, 0.0, 0.0))
            }
            Form::Block { .. } | Form::Ramp { .. } => None,
        }
    }

    /// The plane `z = z0 + slope_s * s + slope_d * d` of the unturned shape,
    /// turned with it.
    fn turned_plane(&self, z0: f32, slope_s: f32, slope_d: f32) -> Plane {
        // The unturned plane read at the point that the turn carries onto
        // each world point: its slopes turn with the shape, and its height
        // at the world origin comes from the origin's unturned position.
        let (cos, sin) = (self.yaw.cos(), self.yaw.sin());
        let (origin_s, origin_d) = self.unturned((0.0, 0.0));
        Plane {
            z0: z0 + slope_s * origin_s + slope_d * origin_d,
            slope_s: slope_s * cos - slope_d * sin,
            slope_d: slope_s * sin + slope_d * cos,
        }
    }

    fn extent_along(&self) -> (f32, f32) {
        match self.form {
            Form::Block { from, to, .. }
            | Form::Ramp { from, to, .. }
            | Form::Roof { from, to, .. }
            | Form::Slab { from, to, .. } => (from, to),
        }
    }

    fn extent_across(&self) -> (f32, f32) {
        self.across.unwrap_or((-UNBOUNDED_ACROSS, UNBOUNDED_ACROSS))
    }

    /// The point the turn pivots on: the footprint's centre.
    fn pivot(&self) -> (f32, f32) {
        let (from, to) = self.extent_along();
        let (left, right) = self.extent_across();
        (0.5 * (from + to), 0.5 * (left + right))
    }

    /// Where the turn carries an unturned point.
    fn turned(&self, (s, d): (f32, f32)) -> (f32, f32) {
        let (pivot_s, pivot_d) = self.pivot();
        let (cos, sin) = (self.yaw.cos(), self.yaw.sin());
        let (ds, dd) = (s - pivot_s, d - pivot_d);
        (pivot_s + cos * ds - sin * dd, pivot_d + sin * ds + cos * dd)
    }

    /// The unturned point the turn carries onto `(s, d)`.
    fn unturned(&self, (s, d): (f32, f32)) -> (f32, f32) {
        let (pivot_s, pivot_d) = self.pivot();
        let (cos, sin) = (self.yaw.cos(), self.yaw.sin());
        let (ds, dd) = (s - pivot_s, d - pivot_d);
        (pivot_s + cos * ds + sin * dd, pivot_d - sin * ds + cos * dd)
    }
}

/// Ground built from shapes over a flat base.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AuthoredGround {
    /// Height of the open ground between shapes; `None` for a void, as
    /// under stepping stones over a chasm.
    pub base: Option<f32>,
    pub shapes: Vec<Shape>,
}
