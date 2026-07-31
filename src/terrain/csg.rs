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

/// Inclusive-exclusive voxel index range covering a coordinate span.
///
/// Sample index `i` on an axis denotes grid-local position `i * step`, so the
/// voxel lattice is anchored to the grid origin and stays aligned between
/// chunks regardless of where the authored bounds happen to fall.
pub(super) fn index_range(min: f32, max: f32, step: f32) -> (i32, i32) {
    ((min / step).floor() as i32, (max / step).ceil() as i32)
}

/// Union a solid into the grid using SDF-style density.
///
/// Writes a smoothly-varying density based on the signed distance `sdf`
/// (negative inside the solid, positive outside). Voxels are only updated when
/// the new density is greater than what is already there — so features layer
/// correctly and never clobber deeper geometry.
pub(super) fn union_solid(
    grid: &mut ChunkGrid,
    pos: Point3<f32>,
    sdf: f32,
    step: f32,
    material: VoxelMaterial,
    health: u8,
) {
    let new_density = (-sdf / step).clamp(-1.0, 1.0);
    let existing = grid.get(pos);
    if new_density <= existing.density {
        return;
    }
    let (mat, hp) = if new_density > 0.0 {
        (material, health)
    } else {
        (VoxelMaterial::Air, 0)
    };
    grid.set(
        pos,
        Voxel {
            density: new_density,
            material: mat,
            health: hp,
        },
    );
}

/// Carve a volume out of the grid using SDF-style density.
///
/// `sdf_carve` is the signed distance to the carve surface (negative inside the
/// region being removed). Uses CSG subtraction semantics so existing solids
/// outside the carve are preserved, and voxels near the cut get a smooth
/// partial density for marching cubes to interpolate.
pub(super) fn carve_with_sdf(grid: &mut ChunkGrid, pos: Point3<f32>, sdf_carve: f32, step: f32) {
    let carve_density = (sdf_carve / step).clamp(-1.0, 1.0);
    let existing = grid.get(pos);
    let new_density = existing.density.min(carve_density);
    if new_density >= existing.density {
        return;
    }
    let (mat, hp) = if new_density > 0.0 {
        (existing.material, existing.health)
    } else {
        (VoxelMaterial::Air, 0)
    };
    grid.set(
        pos,
        Voxel {
            density: new_density,
            material: mat,
            health: hp,
        },
    );
}
