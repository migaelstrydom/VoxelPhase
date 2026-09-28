//! The two signed-distance writes every volumetric feature is built from.
//!
//! Both operations share one convention: a voxel's density is the signed
//! distance to the surface, negated and expressed in voxels, so `+1` is a full
//! voxel inside and `-1` a full voxel outside. Marching cubes interpolates the
//! zero crossing between two samples, which is what puts a surface at its
//! authored height rather than at the nearest lattice plane — the sub-voxel
//! encoding the whole terrain layer depends on.
//!
//! Keeping the writes here rather than in one feature's file is what lets a new
//! primitive be a distance function and nothing else.

use nalgebra::Point3;

use super::chunk_grid::ChunkGrid;
use super::voxel::{Voxel, VoxelMaterial};

/// How close to the iso-surface a lattice sample is allowed to land, as a
/// fraction of a voxel.
///
/// Marching cubes degenerates when a surface passes exactly through a sample:
/// two edges of the same cell then interpolate to the same point, and the
/// triangle between them has zero area and an edge no neighbour can match.
/// Authored geometry is *made* of round numbers — a deck at y = 12, a plateau
/// at y = 8, a tread every metre — so unlike noisy terrain it hits that case
/// constantly, and a swept surface on a gentle slope hits it periodically all
/// the way along.
///
/// Samples inside this band are therefore pushed to the *inside* of the solid:
/// a point exactly on a surface belongs to it, whether that surface is a deck,
/// a plateau or a bore. Ties going the same way for both is what stops a shaft
/// sunk to exactly the height of the slab it pierces from keeping a one-plane
/// lid over its mouth.
///
/// The cost is that a coincident surface sits up to this fraction of a voxel
/// proud of its authored position — half a centimetre at metre voxels, and
/// nowhere else.
pub const SURFACE_BAND: f32 = 0.01;

/// Move a signed distance out of the degenerate band, into the solid.
fn debias(distance: f32, step: f32) -> f32 {
    let band = SURFACE_BAND * step;
    if distance.abs() < band {
        -band
    } else {
        distance
    }
}

/// Move an authored surface height out of the degenerate band, onto the inside
/// of the solid below it.
///
/// The heightfield's counterpart to the debias [`union_solid`] applies. A
/// column samples the distance `y - height` at every lattice plane, so nudging
/// the height once is that same correction applied to whichever sample would
/// have landed in the band — and doing it to the height rather than to each
/// distance keeps the indices a column derives from it (the topmost solid
/// voxel, the partial-air cap above it) consistent with the surface they
/// describe.
pub(super) fn debias_height(height: f32, step: f32) -> f32 {
    let band = SURFACE_BAND * step;
    let plane = (height / step).round() * step;
    if (height - plane).abs() < band {
        plane + band
    } else {
        height
    }
}

/// Inclusive-exclusive voxel index range covering a coordinate span.
///
/// Sample index `i` on an axis denotes grid-local position `i * step`, so the
/// voxel lattice is anchored to the grid origin and stays aligned between
/// chunks regardless of where the authored bounds happen to fall.
pub(super) fn index_range(min: f32, max: f32, step: f32) -> (i32, i32) {
    ((min / step).floor() as i32, (max / step).ceil() as i32)
}

/// Inclusive-exclusive index range of every lattice sample in a coordinate
/// span, and the samples just outside it on either side: the ones a surface
/// on the span's ends interpolates to. [`index_range`] stops short of the
/// sample on (or past) `max`, so a solid filling it ends half a voxel short.
pub(super) fn sample_range(min: f32, max: f32, step: f32) -> (i32, i32) {
    ((min / step).floor() as i32, (max / step).ceil() as i32 + 1)
}

/// Union a solid into the grid using SDF-style density.
///
/// Writes a smoothly-varying density based on the signed distance `sdf`
/// (negative inside the solid, positive outside). Voxels are only updated when
/// the new density is greater than what is already there — so features layer
/// correctly and never clobber deeper geometry.
///
/// The distance is debiased before it is encoded, so no feature can leave a
/// sample sitting exactly on its own surface. See [`SURFACE_BAND`].
pub(super) fn union_solid(
    grid: &mut ChunkGrid,
    pos: Point3<f32>,
    sdf: f32,
    step: f32,
    material: VoxelMaterial,
) {
    let new_density = (-debias(sdf, step) / step).clamp(-1.0, 1.0);
    let existing = grid.get(pos);
    if new_density <= existing.density {
        return;
    }
    let mat = if new_density > 0.0 {
        material
    } else {
        VoxelMaterial::Air
    };
    grid.set(
        pos,
        Voxel {
            density: new_density,
            material: mat,
        },
    );
}

/// Density a sample takes from a carve surface `sdf_carve` away from it,
/// negative inside the region being removed.
///
/// Combine it with a sample's existing density using `min`, which is CSG
/// subtraction: the result is solid only where the old solid was and the carve
/// is not.
///
/// Separate from [`carve_with_sdf`] because the runtime destruction path
/// (`Chunk::damage_sphere`) carves the same way but writes through the chunk's
/// octree rather than the grid, and the two must encode a cut identically. When
/// they did not, craters left a step in the density field that marching cubes
/// read as a slope, tilting the normals of flat ground for two voxels around
/// every blast.
pub(super) fn carve_density(sdf_carve: f32, step: f32) -> f32 {
    (debias(sdf_carve, step) / step).clamp(-1.0, 1.0)
}

/// Carve a volume out of the grid using SDF-style density.
///
/// `sdf_carve` is the signed distance to the carve surface (negative inside the
/// region being removed). Uses CSG subtraction semantics so existing solids
/// outside the carve are preserved, and voxels near the cut get a smooth
/// partial density for marching cubes to interpolate.
///
/// Debiased on the same terms as [`union_solid`], and in the same direction: a
/// sample exactly on the carve surface is inside the region being removed.
pub(super) fn carve_with_sdf(grid: &mut ChunkGrid, pos: Point3<f32>, sdf_carve: f32, step: f32) {
    let existing = grid.get(pos);
    let new_density = existing.density.min(carve_density(sdf_carve, step));
    if new_density >= existing.density {
        return;
    }
    let mat = if new_density > 0.0 {
        existing.material
    } else {
        VoxelMaterial::Air
    };
    grid.set(
        pos,
        Voxel {
            density: new_density,
            material: mat,
        },
    );
}
