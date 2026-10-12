//! Which side of the water a blended surface lies on, as the camera sees it.
//!
//! Water is drawn inside the HDR scene pass, over a copy of what lies behind
//! it (see `Renderer::record_scene_draws`). Everything blended must therefore
//! be put down either before that copy — beyond the surface, so the water
//! refracts and tints it — or after the water, over it:
//!
//! ```text
//!   camera above water               camera under water
//!   ──────────────────               ──────────────────
//!   above the surface   → Near       above the surface   → Beyond
//!   below the surface   → Beyond     below the surface   → Near
//!   crossing it         → Across     crossing it         → Across
//! ```
//!
//! That holds per pixel, not only per draw: along any view ray a point above a
//! level surface is met before the surface is, when the camera is above it.
//! So the divide needs no depth sort against the water, only each surface's
//! height against the level under it. A surface that crosses the level is
//! drawn on both sides, each half clipped at the level ([`Side::clip_planes`]).
//!
//! The divide is taken from the water of the frame: the still draws (a
//! bench's pool, the sea's ring) and every tile of the field's water that
//! stands still, one 8 m tile and one level each, and every column of its
//! running water at that column's surface. A surface through running water
//! is clipped level at the surface over its centre, which the water's own
//! slope across it moves by no more than a column's step.

use nalgebra::{Vector2, Vector3};
use rustc_hash::FxHashMap;

use crate::water::geometry::{Column, CHUNK_COLUMNS, COLUMN_SIZE};

/// Width of a water tile, m.
const TILE_EXTENT: f32 = CHUNK_COLUMNS as f32 * COLUMN_SIZE;

/// A clip plane `(n, d)` passing `dot(n, p) + d >= 0`; the one that clips
/// nothing.
pub const NO_CLIP: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

/// A square of water at one level: a tile of still water, or a column of
/// running water.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaterPatch {
    /// The square's lowest x and z corner.
    pub min: Vector2<f32>,
    /// The square's highest x and z corner.
    pub max: Vector2<f32>,
    /// The surface over the square, without swell or ripples.
    pub level: f32,
}

impl WaterPatch {
    /// The patch covering water tile `(x, z)` at `level`.
    pub fn tile(x: i32, z: i32, level: f32) -> Self {
        let min = Vector2::new(x as f32, z as f32) * TILE_EXTENT;
        Self {
            min,
            max: min + Vector2::repeat(TILE_EXTENT),
            level,
        }
    }

    /// The patch covering one column at `level`.
    pub fn column(column: Column, level: f32) -> Self {
        let (x, z) = column.min_corner();
        let min = Vector2::new(x, z);
        Self {
            min,
            max: min + Vector2::repeat(COLUMN_SIZE),
            level,
        }
    }

    /// The tile the patch lies in.
    fn tile_key(&self) -> (i32, i32) {
        tile_of(self.min + Vector2::repeat(0.5 * COLUMN_SIZE))
    }

    fn contains(&self, xz: Vector2<f32>) -> bool {
        xz.x >= self.min.x && xz.x < self.max.x && xz.y >= self.min.y && xz.y < self.max.y
    }
}

/// Where a blended surface lies against the water.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Side {
    /// Wholly beyond the surface: drawn before the water, under it.
    Beyond,
    /// Wholly this side of the surface, or where there is no water: drawn
    /// after the water, over it.
    Near,
    /// Through the surface at `level`: drawn on both sides, clipped.
    Across { level: f32, camera_submerged: bool },
}

impl Side {
    /// Clip planes for the half drawn beyond the water and the half drawn
    /// near it, as `gl_ClipDistance` reads them.
    pub fn clip_planes(self) -> ([f32; 4], [f32; 4]) {
        match self {
            Side::Beyond | Side::Near => (NO_CLIP, NO_CLIP),
            Side::Across {
                level,
                camera_submerged,
            } => {
                let below = [0.0, -1.0, 0.0, level];
                let above = [0.0, 1.0, 0.0, -level];
                if camera_submerged {
                    (above, below)
                } else {
                    (below, above)
                }
            }
        }
    }

    /// Whether any of the surface is drawn before the water.
    pub fn reaches_beyond(self) -> bool {
        !matches!(self, Side::Near)
    }

    /// Whether any of the surface is drawn after the water.
    pub fn reaches_near(self) -> bool {
        !matches!(self, Side::Beyond)
    }
}

/// The tile holding a point.
fn tile_of(xz: Vector2<f32>) -> (i32, i32) {
    (
        (xz.x / TILE_EXTENT).floor() as i32,
        (xz.y / TILE_EXTENT).floor() as i32,
    )
}

/// The frame's water surfaces, and which side of them the camera is on.
#[derive(Debug, Clone, Default)]
pub struct WaterDivide {
    /// The patches, by the tile they lie in: a point is looked up among its
    /// own tile's, however much running water the frame holds.
    patches: FxHashMap<(i32, i32), Vec<WaterPatch>>,
    camera_submerged: bool,
}

impl WaterDivide {
    /// No water: everything is Near.
    pub fn none() -> Self {
        Self::default()
    }

    pub fn new(patches: Vec<WaterPatch>, camera: Vector3<f32>) -> Self {
        let mut by_tile: FxHashMap<(i32, i32), Vec<WaterPatch>> = FxHashMap::default();
        for patch in patches {
            by_tile.entry(patch.tile_key()).or_default().push(patch);
        }
        let mut divide = Self {
            patches: by_tile,
            camera_submerged: false,
        };
        divide.camera_submerged = divide
            .level_under(camera)
            .is_some_and(|level| camera.y < level);
        divide
    }

    /// Whether there is any water to divide by.
    pub fn is_empty(&self) -> bool {
        self.patches.is_empty()
    }

    /// Where a point lies, such as a particle's centre.
    pub fn of_point(&self, point: Vector3<f32>) -> Side {
        match self.of_sphere(point, 0.0) {
            Side::Across { .. } => Side::Near,
            side => side,
        }
    }

    /// Where a sphere lies, such as a mesh's bounds.
    pub fn of_sphere(&self, centre: Vector3<f32>, radius: f32) -> Side {
        let Some(level) = self.level_under(centre) else {
            return Side::Near;
        };
        let submerged_top = centre.y + radius < level;
        let clear_bottom = centre.y - radius >= level;
        match (submerged_top, clear_bottom, self.camera_submerged) {
            (true, _, false) | (_, true, true) => Side::Beyond,
            (_, true, false) | (true, _, true) => Side::Near,
            _ => Side::Across {
                level,
                camera_submerged: self.camera_submerged,
            },
        }
    }

    /// The water level over a point. Patches may stack, as a pool on an
    /// island over the sea does; a patch holds no floors to tell which body
    /// a point is in, so it is taken to be in the one whose level is nearest.
    fn level_under(&self, point: Vector3<f32>) -> Option<f32> {
        let xz = Vector2::new(point.x, point.z);
        self.patches
            .get(&tile_of(xz))?
            .iter()
            .filter(|patch| patch.contains(xz))
            .map(|patch| patch.level)
            .min_by(|a, b| (a - point.y).abs().total_cmp(&(b - point.y).abs()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pond(level: f32, camera: Vector3<f32>) -> WaterDivide {
        WaterDivide::new(vec![WaterPatch::tile(0, 0, level)], camera)
    }

    #[test]
    fn with_no_water_everything_is_near() {
        let divide = WaterDivide::none();
        assert_eq!(
            divide.of_sphere(Vector3::new(1.0, -5.0, 1.0), 1.0),
            Side::Near
        );
    }

    #[test]
    fn from_above_what_is_under_the_surface_is_beyond_it() {
        let divide = pond(2.0, Vector3::new(4.0, 6.0, 4.0));
        assert_eq!(divide.of_point(Vector3::new(3.0, 1.0, 3.0)), Side::Beyond);
        assert_eq!(divide.of_point(Vector3::new(3.0, 3.0, 3.0)), Side::Near);
    }

    #[test]
    fn from_below_the_sides_swap() {
        let divide = pond(2.0, Vector3::new(4.0, 1.0, 4.0));
        assert_eq!(divide.of_point(Vector3::new(3.0, 1.0, 3.0)), Side::Near);
        assert_eq!(divide.of_point(Vector3::new(3.0, 3.0, 3.0)), Side::Beyond);
    }

    #[test]
    fn outside_every_tile_is_near() {
        let divide = pond(2.0, Vector3::new(4.0, 6.0, 4.0));
        assert_eq!(divide.of_point(Vector3::new(-3.0, 1.0, 3.0)), Side::Near);
    }

    #[test]
    fn a_floe_crosses_the_surface_and_is_clipped_at_it() {
        let divide = pond(2.0, Vector3::new(4.0, 6.0, 4.0));
        let side = divide.of_sphere(Vector3::new(3.0, 2.1, 3.0), 0.5);
        assert!(side.reaches_beyond() && side.reaches_near());
        let (beyond, near) = side.clip_planes();
        let at = |plane: [f32; 4], y: f32| plane[1] * y + plane[3];
        assert!(at(beyond, 1.9) > 0.0 && at(beyond, 2.1) < 0.0);
        assert!(at(near, 2.1) > 0.0 && at(near, 1.9) < 0.0);
    }

    #[test]
    fn a_pool_over_the_sea_divides_by_the_nearest_surface() {
        let divide = WaterDivide::new(
            vec![WaterPatch::tile(0, 0, 0.0), WaterPatch::tile(0, 0, 11.5)],
            Vector3::new(4.0, 20.0, 4.0),
        );
        assert_eq!(divide.of_point(Vector3::new(3.0, 11.0, 3.0)), Side::Beyond);
        assert_eq!(divide.of_point(Vector3::new(3.0, 5.0, 3.0)), Side::Near);
        assert_eq!(divide.of_point(Vector3::new(3.0, -1.0, 3.0)), Side::Beyond);
    }

    #[test]
    fn a_floe_in_a_river_is_divided_by_the_surface_over_it() {
        let river = (0..4)
            .map(|i| WaterPatch::column(Column::new(i, 0), 2.0 - 0.1 * i as f32))
            .collect();
        let divide = WaterDivide::new(river, Vector3::new(4.0, 6.0, 4.0));
        let side = divide.of_sphere(Vector3::new(1.25, 1.8, 0.25), 0.3);
        assert_eq!(
            side,
            Side::Across {
                level: 1.8,
                camera_submerged: false
            }
        );
        assert_eq!(divide.of_point(Vector3::new(1.25, 1.8, 0.75)), Side::Near);
    }
}
