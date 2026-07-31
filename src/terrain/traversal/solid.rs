//! The contract a traversal primitive meets, and the rasteriser that turns one
//! into voxels.

use nalgebra::Point3;

use crate::collision::AABB;

use super::super::chunk_grid::ChunkGrid;
use super::super::csg::{carve_with_sdf, index_range, union_solid};
use super::super::voxel::{DurabilityConfig, VoxelMaterial};

/// One evaluation of a traversal primitive's field.
pub struct Sample {
    /// Signed distance to the primitive's surface: negative inside, positive
    /// outside, in metres.
    pub distance: f32,
    /// Height of the walking surface this point sits under.
    ///
    /// Used only to age the voxel — durability rises with depth below a
    /// surface — so an approximation is fine where a primitive has no single
    /// surface directly overhead.
    pub surface_y: f32,
}

/// A traversal primitive, expressed as a signed-distance field in
/// segment-local coordinates.
///
/// Two methods is the whole contract: [`bounds`](Self::bounds) says where to
/// look, [`sample`](Self::sample) says what is there. Everything else — the
/// voxel lattice, the sub-voxel encoding, the CSG operation, the clip against
/// the segment's extent — belongs to the rasteriser, so primitives stay
/// comparable and a new one cannot get the encoding subtly wrong.
pub trait TraversalSolid {
    /// Signed distance and surface reference at a segment-local position.
    fn sample(&self, p: Point3<f32>) -> Sample;

    /// Segment-local extent the primitive can write within.
    ///
    /// Must be padded by at least one voxel beyond the surface on every side,
    /// or the partial-density shell that marching cubes interpolates through
    /// falls outside the iterated region and the surface snaps back to the
    /// lattice.
    fn bounds(&self, voxel_size: f32) -> AABB;
}

/// How close to the iso-surface a lattice sample is allowed to land, as a
/// fraction of a voxel.
///
/// Marching cubes degenerates when a surface passes exactly through a sample:
/// two edges of the same cell then interpolate to the same point, and the
/// triangle between them has zero area and an edge no neighbour can match. A
/// route is *made* of round numbers — a deck at y = 12, a tread every metre —
/// so unlike noisy terrain it hits that case constantly, and a swept deck on a
/// gentle slope hits it periodically all the way along.
///
/// Samples inside this band are therefore pushed to the *inside* of the
/// primitive: a point exactly on a primitive's surface belongs to it, whether
/// that primitive is a deck or a bore. Ties going the same way for both is what
/// stops a shaft sunk to exactly the height of the slab it pierces from keeping
/// a one-plane lid over its mouth.
///
/// The cost is that a coincident surface sits up to this fraction of a voxel
/// proud of its authored position — half a centimetre at metre voxels, and
/// nowhere else.
const SURFACE_BAND: f32 = 0.01;

/// Move a sample out of the degenerate band, into the primitive.
fn debias(distance: f32, step: f32) -> f32 {
    let band = SURFACE_BAND * step;
    if distance.abs() < band {
        -band
    } else {
        distance
    }
}

/// Add a traversal primitive to the grid.
///
/// Samples strictly **on the voxel lattice** (`i * voxel_size`). That is not
/// incidental: a chunk stores one voxel per lattice cell, so a sample taken
/// half a voxel off the lattice is evaluated at one position and stored at
/// another, and the sub-voxel offset the density encodes is wrong by exactly
/// that amount. On-lattice sampling is what makes a deck's surface land at its
/// authored height at any resolution.
pub fn rasterise(
    grid: &mut ChunkGrid,
    solid: &dyn TraversalSolid,
    material: VoxelMaterial,
    durability: &DurabilityConfig,
    clip: &AABB,
) {
    let step = grid.voxel_size();
    let floor_y = clip.min.y;
    for_each_lattice_point(solid, clip, step, |p, sample| {
        let health = durability.health_at(p.y, sample.surface_y, floor_y);
        union_solid(
            grid,
            p,
            debias(sample.distance, step),
            step,
            material,
            health,
        );
    });
}

/// Remove a traversal primitive from the grid — the same field, subtracted.
pub fn excavate(grid: &mut ChunkGrid, solid: &dyn TraversalSolid, clip: &AABB) {
    let step = grid.voxel_size();
    for_each_lattice_point(solid, clip, step, |p, sample| {
        carve_with_sdf(grid, p, debias(sample.distance, step), step);
    });
}

/// Walk the lattice points inside a primitive's clipped bounds, evaluating it
/// at each one.
fn for_each_lattice_point(
    solid: &dyn TraversalSolid,
    clip: &AABB,
    step: f32,
    mut visit: impl FnMut(Point3<f32>, Sample),
) {
    let region = solid.bounds(step);
    let (ix0, ix1) = index_range(
        region.min.x.max(clip.min.x),
        region.max.x.min(clip.max.x),
        step,
    );
    let (iy0, iy1) = index_range(
        region.min.y.max(clip.min.y),
        region.max.y.min(clip.max.y),
        step,
    );
    let (iz0, iz1) = index_range(
        region.min.z.max(clip.min.z),
        region.max.z.min(clip.max.z),
        step,
    );

    // Inclusive on both ends: the padding shell is exactly one voxel wide, so
    // dropping the far plane would drop the partial densities that put the
    // surface between lattice planes.
    for ix in ix0..=ix1 {
        for iy in iy0..=iy1 {
            for iz in iz0..=iz1 {
                let p = Point3::new(ix as f32 * step, iy as f32 * step, iz as f32 * step);
                let sample = solid.sample(p);
                visit(p, sample);
            }
        }
    }
}

/// Signed distance to an axis-aligned slab of half-thickness `half` centred on
/// `centre`, along one axis. Negative inside.
pub(super) fn slab(value: f32, centre: f32, half: f32) -> f32 {
    (value - centre).abs() - half
}

/// Signed distance to the half-space below `top`. Negative below.
pub(super) fn below(value: f32, top: f32) -> f32 {
    value - top
}

/// Signed distance to the half-space above `bottom`. Negative above.
pub(super) fn above(value: f32, bottom: f32) -> f32 {
    bottom - value
}
