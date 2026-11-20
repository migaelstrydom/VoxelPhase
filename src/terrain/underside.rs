//! A hung terrain's underside, column by column.
//!
//! A terrain with an `underside` hangs in the air: a column is solid from the
//! underside its keels make up to its surface, and only inside their
//! outlines. Each keel holding a column bounds the solid twice, from below by
//! its underside and at the side by its outline, and the solid is the union
//! of what the keels hold:
//!
//! ```text
//!   density = min( surface − y ,  max over keels of min( y − underside , inland ) )
//!                  └ heightfield ┘  └──────────────── this module ─────────────────┘
//! ```
//!
//! Every term is a distance in voxels, so marching cubes puts the island's
//! edge on its outline rather than where the lattice switches columns on and
//! off: the coast is as smooth as the outline at any resolution. That holds
//! only where the surface runs on past the outline; a surface that drops away
//! at the coast takes the edge back to the lattice.

use smallvec::SmallVec;

use super::csg::debias_height;
use super::generation::{encoded_slope, to_voxel_material, NEIGHBOUR_OFFSETS};
use super::landform::Keel;
use super::voxel::VoxelMaterial;
use crate::level::{UndersideFeature, VoxelMaterialId};

/// The keels holding one column, as each meets it.
pub(super) struct ColumnUnderside {
    holds: SmallVec<[Hold; 2]>,
}

/// One keel's hold on a column.
struct Hold {
    /// Height of the keel's underside over the column, off the lattice
    /// planes.
    underside: f32,
    /// Vertical metres per metre of distance from the underside, as the
    /// heightfield's own stretch: `sqrt(1 + slope²)` with smooth slopes.
    stretch: f32,
    /// Distance inside the keel's outline, m; negative outside it.
    inland: f32,
    /// What the keel's underside is made of.
    material: VoxelMaterial,
}

impl ColumnUnderside {
    /// The keels holding the column at (x, z): every keel whose side lies
    /// less than a voxel outside it.
    pub(super) fn at(
        x: f32,
        z: f32,
        step: f32,
        keels: &[UndersideFeature],
        smooth_slopes: bool,
    ) -> Self {
        let holds = keels
            .iter()
            .filter_map(|feature| {
                let (keel, material) = keel_of(feature);
                let here = keel.sample(x, z);
                // A voxel or more outside the outline, the side is at least a
                // voxel from every sample in the column: plain air.
                if here.inland <= -step {
                    return None;
                }
                let underside = debias_underside(here.underside, step);
                let stretch = if smooth_slopes {
                    let around = NEIGHBOUR_OFFSETS
                        .map(|(dx, dz)| keel.sample(x + dx * step, z + dz * step).underside);
                    encoded_slope(underside, &around, step).hypot(1.0)
                } else {
                    1.0
                };
                Some(Hold {
                    underside,
                    stretch,
                    inland: here.inland,
                    material: to_voxel_material(material),
                })
            })
            .collect();
        Self { holds }
    }

    /// Whether no keel holds the column: nothing in it is near solid.
    pub(super) fn is_empty(&self) -> bool {
        self.holds.is_empty()
    }

    /// Index of the lowest lattice plane whose sample lies within a voxel's
    /// distance above a keel's underside: below it the column is plain air.
    pub(super) fn lowest_index(&self, step: f32) -> i32 {
        self.holds
            .iter()
            .map(|hold| ((hold.underside - step * hold.stretch) / step).floor() as i32 + 1)
            .min()
            .unwrap_or(i32::MAX)
    }

    /// Signed distance from the solid the keels hold at height `y`, in
    /// voxels and unclamped: positive inside.
    pub(super) fn density_at(&self, y: f32, step: f32) -> f32 {
        self.holds
            .iter()
            .map(|hold| hold.density_at(y, step))
            .fold(f32::NEG_INFINITY, f32::max)
    }

    /// The material at height `y` where it lies nearer the underside of the
    /// keel holding it than the `surface` above; `None` leaves it to the
    /// surface's layers.
    pub(super) fn material_at(&self, y: f32, step: f32, surface: f32) -> Option<VoxelMaterial> {
        let hold = self
            .holds
            .iter()
            .max_by(|a, b| a.density_at(y, step).total_cmp(&b.density_at(y, step)))?;
        (y - hold.underside < surface - y).then_some(hold.material)
    }
}

impl Hold {
    /// Distance in voxels from the solid this keel holds: above its
    /// underside and inside its outline.
    fn density_at(&self, y: f32, step: f32) -> f32 {
        ((y - self.underside) / (step * self.stretch)).min(self.inland / step)
    }
}

/// An underside moved off the lattice planes, downwards so that a sample on
/// it belongs to the solid, as `debias_height` does for a surface.
fn debias_underside(underside: f32, step: f32) -> f32 {
    -debias_height(-underside, step)
}

/// The shape and material of an underside feature.
fn keel_of(feature: &UndersideFeature) -> (Keel<'_>, VoxelMaterialId) {
    match *feature {
        UndersideFeature::Keel {
            ref outline,
            rim,
            bottom,
            taper,
            warp,
            roughness,
            seed,
            material,
        } => (
            Keel {
                outline,
                rim,
                bottom,
                taper,
                warp,
                roughness,
                seed,
            },
            material,
        ),
    }
}
