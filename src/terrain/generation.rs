//! Procedural terrain generation using composable features.
//!
//! Generation writes into a [`ChunkGrid`], which allocates chunks on demand, so
//! the cost of a level is set by the volume its features actually fill rather
//! than by the bounding box they sit in.
//!
//! # Coordinates
//!
//! Every position here is **grid-local**, and every noise function is sampled at
//! a grid-local position from a feature-local seed. That is what makes a
//! generated region identical wherever the grid is later placed; sampling world
//! position instead would silently change terrain whenever it moved.

use nalgebra::Point3;

use super::chunk::{ChunkCoord, CHUNK_VOXELS};
use super::chunk_grid::ChunkGrid;
use super::csg::{
    carve_with_sdf, debias_height, index_range, paint_solid, sample_range, union_solid,
    SURFACE_BAND,
};
use super::landform::{Basin, Channel, Landmass};
use super::traversal::{excavate, rasterise, route_plan, RoutePart};
use super::underside::ColumnUnderside;
use super::voxel::{Voxel, VoxelMaterial};
use crate::collision::AABB;
use crate::level::{
    CaveDepthPoint, CaveMouth, CaveRegion, MaterialLayer, Terrain, TerrainFeature, VolumeFeature,
    VoxelMaterialId,
};
use crate::utils::noise::{fbm_2d_periodic, fbm_3d};

/// Generate terrain into a chunk grid from a `Terrain` description.
///
/// `bounds` is the grid-local extent to generate within; features are clipped
/// to it, and the terrain's solid ends exactly on its faces.
pub fn generate_terrain(grid: &mut ChunkGrid, terrain: &Terrain, bounds: &AABB) {
    let step = grid.voxel_size();

    generate_heightfield(grid, terrain, bounds, step);

    // Pass 2: volumetric features — place or carve voxels in 3D. Each feature
    // derives its own iteration box, so only the chunks it overlaps are visited.
    for volume in &terrain.volumes {
        apply_volume(grid, volume, terrain, bounds, step);
    }

    clip_to_bounds(grid, bounds, step);
}

/// Pass 3: cut the solid off at the bounds' faces.
///
/// A sample is solid or air, and marching cubes puts the wall between a solid
/// one and the air past it halfway along, so a solid that simply stops at the
/// last sample inside ends half a voxel short of a face on the lattice, or
/// past one between lattice planes. Carving every sample within a voxel of a
/// face by its distance to that face puts the wall on the face.
///
/// ```text
///   sample   47.0   47.5   48.0 (face)   48.5
///   before    +1     +1     (none)        (none)   wall at 47.75
///   after     +1     +1     ≈0⁻           (none)   wall at 48.0
/// ```
fn clip_to_bounds(grid: &mut ChunkGrid, bounds: &AABB, step: f32) {
    let ranges = [
        sample_range(bounds.min.x, bounds.max.x, step),
        sample_range(bounds.min.y, bounds.max.y, step),
        sample_range(bounds.min.z, bounds.max.z, step),
    ];
    let (min, max) = (
        [bounds.min.x, bounds.min.y, bounds.min.z],
        [bounds.max.x, bounds.max.y, bounds.max.z],
    );
    for axis in 0..3 {
        for (face, inward) in [(min[axis], 1.0f32), (max[axis], -1.0)] {
            // Only the samples within a voxel of the face take a density
            // other than the full solid the carve leaves alone.
            let (lo, hi) = index_range(face - step, face + step, step);
            let mut slab = ranges;
            slab[axis] = (lo.max(ranges[axis].0), (hi + 1).min(ranges[axis].1));
            for i in slab[0].0..slab[0].1 {
                for j in slab[1].0..slab[1].1 {
                    for k in slab[2].0..slab[2].1 {
                        let p = Point3::new(i as f32 * step, j as f32 * step, k as f32 * step);
                        // Negative outside the bounds: the region removed.
                        let inside = (p[axis] - face) * inward;
                        carve_with_sdf(grid, p, inside, step);
                    }
                }
            }
        }
    }
}

/// Pass 1: heightfield — fill columns based on feature-driven height, from
/// the floor of the bounds or, on a hung terrain, from its underside
/// (`super::underside`).
///
/// Walks one chunk column at a time. Heights are evaluated once per voxel
/// column and reused for every chunk in that column, and each chunk is resolved
/// once rather than per voxel write.
fn generate_heightfield(grid: &mut ChunkGrid, terrain: &Terrain, bounds: &AABB, step: f32) {
    let floor_y = bounds.min.y;
    let n = CHUNK_VOXELS as i32;

    // Up to the sample on or past each far face, for `clip_to_bounds` to cut
    // the wall back to it.
    let (ix0, ix1) = sample_range(bounds.min.x, bounds.max.x, step);
    let (iy0, iy1) = sample_range(bounds.min.y, bounds.max.y, step);
    let (iz0, iz1) = sample_range(bounds.min.z, bounds.max.z, step);

    let chunk_x = (div_floor(ix0, n), div_floor(ix1 - 1, n));
    let chunk_z = (div_floor(iz0, n), div_floor(iz1 - 1, n));

    // Per voxel column within one chunk column.
    let mut columns: Vec<Option<Column>> = Vec::new();

    for cx in chunk_x.0..=chunk_x.1 {
        let xs = (cx * n).max(ix0);
        let xe = ((cx + 1) * n).min(ix1);
        if xs >= xe {
            continue;
        }

        for cz in chunk_z.0..=chunk_z.1 {
            let zs = (cz * n).max(iz0);
            let ze = ((cz + 1) * n).min(iz1);
            if zs >= ze {
                continue;
            }

            // Evaluate the heightfield once for this chunk column.
            columns.clear();
            let mut top_index = i32::MIN;
            let mut lowest_index = i32::MAX;
            for ix in xs..xe {
                for iz in zs..ze {
                    let column = Column::at(ix as f32 * step, iz as f32 * step, iy0, step, terrain);
                    if let Some(column) = &column {
                        top_index = top_index.max(column.air_top + 1);
                        lowest_index = lowest_index.min(column.lowest);
                    }
                    columns.push(column);
                }
            }
            if lowest_index > top_index {
                continue;
            }

            // Only the chunks between the lowest solid (the floor, or a
            // hung terrain's underside) and the highest surface (plus the
            // partial-air voxels above it) hold anything.
            let cy_lo = div_floor(lowest_index, n);
            let cy_hi = div_floor(top_index.min(iy1 - 1), n);

            for cy in cy_lo..=cy_hi {
                let ys = (cy * n).max(iy0);
                let ye = ((cy + 1) * n).min(iy1);
                let reached = columns
                    .iter()
                    .flatten()
                    .any(|column| column.lowest < ye && column.air_top >= ys);
                if ys >= ye || !reached {
                    continue;
                }

                let chunk = grid.chunk_or_insert(ChunkCoord::new(cx, cy, cz));
                let mut index = 0usize;
                for ix in xs..xe {
                    let x = ix as f32 * step;
                    for iz in zs..ze {
                        let z = iz as f32 * step;
                        let Some(column) = &columns[index] else {
                            index += 1;
                            continue;
                        };
                        index += 1;

                        let top_voxel_y = column.top as f32 * step;
                        // A column wholly below the floor holds nothing.
                        if top_voxel_y >= column.height {
                            continue;
                        }

                        for iy in ys.max(column.lowest)..ye.min(column.air_top + 1) {
                            let y = iy as f32 * step;
                            let density = column.density_at(y, step);
                            if density > 0.0 {
                                let material = if y <= floor_y + terrain.bedrock_thickness {
                                    VoxelMaterial::Bedrock
                                } else {
                                    column.material_at(y, step)
                                };
                                let mut voxel = Voxel::solid(material);
                                // Near the surface, the SDF density lands
                                // marching cubes' triangle at y = height instead
                                // of at the midpoint between solid and air.
                                voxel.density = density.max(f32::MIN_POSITIVE);
                                chunk.set_voxel(Point3::new(x, y, z), voxel);
                            } else {
                                // The matching partial-air voxels above it.
                                chunk.set_voxel(
                                    Point3::new(x, y, z),
                                    Voxel {
                                        density: density.min(0.0),
                                        material: VoxelMaterial::Air,
                                    },
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

/// One voxel column of the heightfield: where its surface is and how it is
/// encoded.
struct Column<'a> {
    /// Surface height, debiased off the lattice planes.
    height: f32,
    /// Index of the topmost solid voxel: the last strictly below `height`.
    top: i32,
    /// Index of the topmost partial-air voxel above the surface.
    air_top: i32,
    /// Vertical metres per metre of distance from the surface:
    /// `sqrt(1 + slope²)` with `Terrain::smooth_slopes`, else 1, where the
    /// slope is the one the column's own surface keeps (`encoded_slope`).
    stretch: f32,
    /// The layers the column is laid with.
    layers: &'a [MaterialLayer],
    /// The layers on the column's side where a drop to a neighbour bares it:
    /// those of a surface as steep as the drop.
    face_layers: &'a [MaterialLayer],
    /// The lowest neighbouring surface. The column's voxels above it are
    /// exposed on that side, on the face of the drop.
    face_below: f32,
    /// Height of the voxel that depth is measured down from: the column's own
    /// top voxel, or the top voxel of the sheer face it lies under
    /// (`face_top`).
    depth_origin: f32,
    /// Whether the column lies under the top of a sheer face, every voxel of
    /// it on that face, its top one included.
    under_face: bool,
    /// What the column hangs on, when the terrain has an underside; `None`
    /// stands it on the floor.
    underside: Option<ColumnUnderside>,
    /// Index of the lowest voxel the column writes: the floor's, or the
    /// lowest a voxel's distance above its underside.
    lowest: i32,
}

impl<'a> Column<'a> {
    /// The column at (x, z), or `None` where a hung terrain holds nothing
    /// near it.
    fn at(x: f32, z: f32, iy0: i32, step: f32, terrain: &'a Terrain) -> Option<Self> {
        let underside = (!terrain.underside.is_empty())
            .then(|| ColumnUnderside::at(x, z, step, &terrain.underside, terrain.smooth_slopes));
        if underside.as_ref().is_some_and(ColumnUnderside::is_empty) {
            return None;
        }
        let lowest = underside
            .as_ref()
            .map_or(iy0, |underside| underside.lowest_index(step).max(iy0));
        // Debiased before anything is derived from it: an authored height
        // that lands exactly on a lattice plane degenerates marching cubes, and
        // the column's indices must describe the height actually encoded.
        let surface = surface_at(x, z, terrain);
        let height = debias_height(surface.height, step);
        let needs_slope = terrain.smooth_slopes
            || terrain
                .surface_bands
                .iter()
                .any(|band| band.steeper_than.is_some());
        let neighbours = if needs_slope {
            Some(neighbour_heights(x, z, step, terrain))
        } else {
            None
        };
        let slope = neighbours.map_or(0.0, |around| surface_slope(&around, step));
        let stretch = match neighbours {
            Some(around) if terrain.smooth_slopes => {
                encoded_slope(height, &around, step).hypot(1.0)
            }
            _ => 1.0,
        };
        let top = top_voxel_index(height, step).max(iy0);
        // The last lattice plane within one voxel's distance above the surface.
        let air_top = ((((height + step * stretch) / step).ceil() as i32) - 1).max(top + 1);
        let layers = surface_layers(height, slope, surface.cut, terrain);
        let face_below =
            neighbours.map_or(height, |around| around.into_iter().fold(height, f32::min));
        let face_slope = (height - face_below) / step;
        // A rise or drop is a face where it is a wall, and deeper than the top
        // layer, which then could not show on it.
        let sheer = layers
            .first()
            .map_or(1.0, |layer| layer.depth)
            .max(WALL_SLOPE * step);
        let face_above = neighbours
            .filter(|_| height - face_below >= sheer)
            .and_then(|around| face_top(x, z, height, around, step, sheer, terrain));
        Some(Self {
            height,
            top,
            air_top,
            stretch,
            layers,
            face_layers: surface_layers(height, face_slope.max(slope), surface.cut, terrain),
            face_below,
            depth_origin: top_voxel_index(face_above.unwrap_or(height), step) as f32 * step,
            under_face: face_above.is_some(),
            underside,
            lowest,
        })
    }

    /// The material at height `y`, by its depth below `depth_origin`.
    ///
    /// Where the column stands above a neighbour, its side is bared down to
    /// that neighbour's surface, and that side is a face as steep as the drop.
    /// Laying it with the column's own layers is what put soil on a cliff: the
    /// last column short of a band's `steeper_than` showed its whole grass and
    /// dirt down the drop beside the first column past it, and along a lip
    /// running across the lattice those columns alternate, hanging teeth of
    /// turf down the cliff. The top voxel keeps the column's own material,
    /// so the turf still runs to the edge.
    ///
    /// A column under the top of a sheer face is all face, and its depth runs
    /// from the top of the face: measured from its own top, each column down a
    /// face only a few voxels wide began its layers again, and a ramp's flank
    /// wore stripes of turf and soil parallel to the ramp.
    ///
    /// In a hung column, what lies nearer the underside than the surface is
    /// the underside's own material.
    fn material_at(&self, y: f32, step: f32) -> VoxelMaterial {
        if let Some(material) = self
            .underside
            .as_ref()
            .and_then(|underside| underside.material_at(y, step, self.height))
        {
            return material;
        }
        let depth = self.depth_origin - y;
        let on_face = self.under_face || (depth >= step && y > self.face_below);
        material_at_depth(
            depth,
            if on_face {
                self.face_layers
            } else {
                self.layers
            },
        )
    }

    /// Density at height `y`: the signed distance to the surface in voxels,
    /// saturating at one. Vertical distance over `stretch` is the distance
    /// across a uniform slope, which lets marching cubes place a steep face
    /// between columns where it crosses, not at the midpoint between them.
    ///
    /// Held out of [`SURFACE_BAND`] on its own side: `debias_height` keeps the
    /// vertical distance out of it, and the stretch would shrink it back in.
    ///
    /// A hung column is bounded by its underside as well (`underside`).
    fn density_at(&self, y: f32, step: f32) -> f32 {
        let mut density = (self.height - y) / (step * self.stretch);
        if let Some(underside) = &self.underside {
            density = density.min(underside.density_at(y, step));
        }
        let density = density.clamp(-1.0, 1.0);
        density.signum() * density.abs().max(SURFACE_BAND)
    }
}

/// Floor division for signed integers (`i32::div_floor` is unstable).
fn div_floor(a: i32, b: i32) -> i32 {
    let q = a / b;
    if (a % b != 0) && ((a < 0) != (b < 0)) {
        q - 1
    } else {
        q
    }
}

/// Compute the terrain surface height at a given (x, z) position by
/// accumulating all heightfield feature contributions.
fn height_at(x: f32, z: f32, terrain: &Terrain) -> f32 {
    surface_at(x, z, terrain).height
}

/// The heightfield's surface over one point.
#[derive(Debug, Clone, Copy)]
struct Surface {
    /// Height, every feature's contribution accumulated.
    height: f32,
    /// How far channels and basins have cut it down, m, less what later
    /// features have filled back in.
    cut: f32,
}

/// The heightfield's surface at (x, z).
fn surface_at(x: f32, z: f32, terrain: &Terrain) -> Surface {
    let mut surface = Surface {
        height: terrain.base_height,
        cut: 0.0,
    };
    for feature in &terrain.features {
        let change = contribute(feature, x, z, surface.height);
        surface.cut = if cuts_a_bed(feature) {
            surface.cut - change
        } else {
            (surface.cut - change.max(0.0)).max(0.0)
        };
        surface.height += change;
    }
    surface
}

/// Whether a feature cuts a watercourse's bed: only ever lowers the ground,
/// to where water runs or stands.
fn cuts_a_bed(feature: &TerrainFeature) -> bool {
    matches!(
        feature,
        TerrainFeature::Channel { .. } | TerrainFeature::Basin { .. }
    )
}

/// The layers that lay a column whose surface stands at `height` with
/// `slope`, cut down `cut` m by watercourses: the first surface band holding
/// it, or the terrain's own.
fn surface_layers(height: f32, slope: f32, cut: f32, terrain: &Terrain) -> &[MaterialLayer] {
    terrain
        .surface_bands
        .iter()
        .find(|band| band.holds(height, slope, cut))
        .map_or(&terrain.material_layers, |band| &band.layers)
}

/// Index of the topmost voxel strictly below a surface at `height`.
fn top_voxel_index(height: f32, step: f32) -> i32 {
    ((height / step).ceil() as i32) - 1
}

/// Directions of a column's neighbours, in voxels: +x, -x, +z, -z.
pub(super) const NEIGHBOUR_OFFSETS: [(f32, f32); 4] =
    [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)];

/// Heights of the heightfield a voxel away from (x, z), in the order of
/// [`NEIGHBOUR_OFFSETS`].
fn neighbour_heights(x: f32, z: f32, step: f32, terrain: &Terrain) -> [f32; 4] {
    NEIGHBOUR_OFFSETS.map(|(dx, dz)| height_at(x + dx * step, z + dz * step, terrain))
}

/// Slope, rise over run, from which a drop between neighbouring columns is a
/// wall rather than a hillside: a hill's flank keeps each column's own layers.
const WALL_SLOPE: f32 = 3.0;

/// Most voxels `face_top` climbs looking for the top of a face.
const FACE_REACH: usize = 16;

/// The top of the sheer face over a column at (x, z), if it lies under one:
/// climbing to the highest neighbour while each rises at least `sheer`. None
/// where no neighbour rises that far.
fn face_top(
    x: f32,
    z: f32,
    height: f32,
    around: [f32; 4],
    step: f32,
    sheer: f32,
    terrain: &Terrain,
) -> Option<f32> {
    let (mut x, mut z, mut top, mut around) = (x, z, height, around);
    let mut climbed = false;
    for _ in 0..FACE_REACH {
        let (uphill, highest) =
            around
                .into_iter()
                .enumerate()
                .fold(
                    (0, f32::MIN),
                    |best, (i, h)| if h > best.1 { (i, h) } else { best },
                );
        if highest - top < sheer {
            break;
        }
        let (dx, dz) = NEIGHBOUR_OFFSETS[uphill];
        x += dx * step;
        z += dz * step;
        top = highest;
        climbed = true;
        around = neighbour_heights(x, z, step, terrain);
    }
    climbed.then_some(top)
}

/// The slope a column's own surface keeps, for its encoding: along each axis
/// the gentler of its two sides, and none where they fall opposite ways.
///
/// On an even slope that is the slope. At a brink it is the flat side's: the
/// last column of a plateau above a sheer drop is not on a slope, and encoded
/// as one (a stretch of 16 over an 8 m wall of 0.25 m voxels) its density
/// barely changed with height beside a neighbour's that changed fully, and
/// the gradient normals along the rim swung back into the plateau and drew a
/// black line down its edge.
pub(super) fn encoded_slope(height: f32, around: &[f32; 4], step: f32) -> f32 {
    let axis = |ahead: f32, behind: f32| {
        let rise_ahead = ahead - height;
        let rise_behind = height - behind;
        if rise_ahead * rise_behind <= 0.0 {
            0.0
        } else {
            rise_ahead.abs().min(rise_behind.abs())
        }
    };
    axis(around[0], around[1]).hypot(axis(around[2], around[3])) / step
}

/// Steepness of the heightfield, rise over run, from the heights a voxel away
/// on either side (`neighbour_heights`).
fn surface_slope(around: &[f32; 4], step: f32) -> f32 {
    let along_x = around[0] - around[1];
    let along_z = around[2] - around[3];
    along_x.hypot(along_z) / (2.0 * step)
}

/// Compute the additive height contribution of a single terrain feature.
///
/// `current_h` is the accumulated height so far, needed by features like
/// `Plateau` that set an absolute height rather than adding a delta.
fn contribute(feature: &TerrainFeature, x: f32, z: f32, current_h: f32) -> f32 {
    match *feature {
        TerrainFeature::Hill {
            center: (cx, cz),
            radius,
            height,
        } => {
            let dx = x - cx;
            let dz = z - cz;
            let dist = (dx * dx + dz * dz).sqrt();
            if dist >= radius {
                0.0
            } else {
                let t = dist / radius;
                // Cosine falloff: smooth dome shape
                height * (1.0 + (t * std::f32::consts::PI).cos()) * 0.5
            }
        }

        TerrainFeature::Crater {
            center: (cx, cz),
            radius,
            depth,
        } => {
            let dx = x - cx;
            let dz = z - cz;
            let dist = (dx * dx + dz * dz).sqrt();
            if dist >= radius {
                0.0
            } else {
                let t = dist / radius;
                -depth * (1.0 + (t * std::f32::consts::PI).cos()) * 0.5
            }
        }

        TerrainFeature::Plateau {
            min: (min_x, min_z),
            max: (max_x, max_z),
            height,
        } => {
            if x >= min_x && x <= max_x && z >= min_z && z <= max_z {
                // Set absolute height: return delta needed
                height - current_h
            } else {
                0.0
            }
        }

        TerrainFeature::Wall {
            from: (fx, fz),
            to: (tx, tz),
            height,
            thickness,
        } => {
            let dist = point_to_segment_dist(x, z, fx, fz, tx, tz);
            let half_t = thickness * 0.5;
            if dist >= half_t {
                0.0
            } else {
                let t = dist / half_t;
                // Smooth falloff at the edges
                height * (1.0 + (t * std::f32::consts::PI).cos()) * 0.5
            }
        }

        TerrainFeature::Ramp {
            from: (fx, fz),
            to: (tx, tz),
            start_height,
            end_height,
            width,
            flat_width,
        } => {
            let (proj_t, perp_dist) = point_to_segment_projection(x, z, fx, fz, tx, tz);
            let half_w = width * 0.5;
            let half_flat = (flat_width * 0.5).clamp(0.0, half_w);
            if perp_dist >= half_w || proj_t < 0.0 || proj_t > 1.0 {
                0.0
            } else {
                let ramp_h = start_height + (end_height - start_height) * proj_t;
                let edge_t = if perp_dist <= half_flat {
                    0.0
                } else {
                    (perp_dist - half_flat) / (half_w - half_flat)
                };
                let edge_falloff = (1.0 + (edge_t * std::f32::consts::PI).cos()) * 0.5;
                // Set absolute height along the ramp
                (ramp_h - current_h) * edge_falloff
            }
        }

        TerrainFeature::TerrainRoughness {
            frequency,
            amplitude,
            octaves,
            seed,
        } => {
            let n = fbm_2d_periodic(x * frequency, z * frequency, octaves, 0.5, 2.0, seed, None);
            // fbm_2d_periodic returns [0, 1]; centre around 0
            (n - 0.5) * 2.0 * amplitude
        }

        TerrainFeature::Cliff {
            from: (fx, fz),
            to: (tx, tz),
            low_height,
            high_height,
            high_side: (hx, hz),
            steepness,
            end_falloff,
            roughness,
            roughness_seed,
        } => {
            // Signed distance from the cliff edge line: positive on the
            // high_side, negative on the low side.
            let signed_dist = signed_dist_to_line(x, z, fx, fz, tx, tz, hx, hz);

            // Sigmoid transition: maps signed distance to [0, 1] where
            // 0 = fully low side, 1 = fully high side.
            let t = sigmoid(signed_dist * steepness);

            // Fade the cliff effect beyond the segment endpoints.
            let end_factor = if end_falloff > 0.0 {
                let edge_dx = tx - fx;
                let edge_dz = tz - fz;
                let len_sq = edge_dx * edge_dx + edge_dz * edge_dz;
                if len_sq < 1e-10 {
                    1.0
                } else {
                    // Unclamped parameter along the edge
                    let raw_t = ((x - fx) * edge_dx + (z - fz) * edge_dz) / len_sq;
                    if raw_t >= 0.0 && raw_t <= 1.0 {
                        1.0
                    } else {
                        let edge_len = len_sq.sqrt();
                        let overshoot = if raw_t < 0.0 {
                            -raw_t * edge_len
                        } else {
                            (raw_t - 1.0) * edge_len
                        };
                        if overshoot >= end_falloff {
                            0.0
                        } else {
                            let ft = overshoot / end_falloff;
                            (1.0 + (ft * std::f32::consts::PI).cos()) * 0.5
                        }
                    }
                }
            } else {
                1.0
            };

            if end_factor < 1e-6 {
                return 0.0;
            }

            // Noise displacement localised to the cliff face.
            // 4*t*(1-t) peaks at 1.0 at the transition midpoint and
            // falls to 0.0 on both flat sides.
            let noise_offset = if roughness > 0.0 {
                let face_factor = 4.0 * t * (1.0 - t);
                let n = fbm_2d_periodic(x * 0.3, z * 0.3, 3, 0.5, 2.0, roughness_seed, None);
                (n - 0.5) * roughness * face_factor
            } else {
                0.0
            };

            let target = low_height + (high_height - low_height) * t + noise_offset;
            (target - current_h) * end_factor
        }

        TerrainFeature::Landmass {
            ref outline,
            shore_height,
            height,
            rise,
            skirt,
            warp,
            relief,
            seed,
        } => Landmass {
            outline,
            shore_height,
            height,
            rise,
            skirt,
            warp,
            relief,
            seed,
        }
        .contribute(x, z, current_h),

        TerrainFeature::Channel {
            ref points,
            width,
            bank,
            warp,
            seed,
        } => Channel {
            points,
            width,
            bank,
            warp,
            seed,
        }
        .contribute(x, z, current_h),

        TerrainFeature::Basin {
            center,
            radius,
            floor,
            warp,
            seed,
        } => Basin {
            center,
            radius,
            floor,
            warp,
            seed,
        }
        .contribute(x, z, current_h),
    }
}

// ---------------------------------------------------------------------------
// Pass 2: volumetric features
// ---------------------------------------------------------------------------

/// Apply a single volumetric feature to the SVO.
fn apply_volume(
    grid: &mut ChunkGrid,
    volume: &VolumeFeature,
    terrain: &Terrain,
    bounds: &AABB,
    step: f32,
) {
    // Traversal primitives are all one shape: a signed-distance field written
    // through the shared rasteriser. They carry their own material rather than
    // layering by depth, because a route reads as built rather than as ground.
    if let Some(plan) = route_plan(volume) {
        let material = to_voxel_material(plan.material);
        for part in &plan.parts {
            match part {
                RoutePart::Void(solid) => excavate(grid, solid.as_ref(), bounds),
                RoutePart::Solid(solid) => rasterise(grid, solid.as_ref(), material, bounds),
            }
        }
        return;
    }

    let layers = &terrain.material_layers;
    match *volume {
        VolumeFeature::Island {
            center: (cx, cy, cz),
            half_extents: (hx, hy, hz),
            edge_noise,
        } => {
            apply_island(
                grid, cx, cy, cz, hx, hy, hz, edge_noise, layers, bounds, step,
            );
        }

        VolumeFeature::Pillar {
            center: (cx, cz),
            height,
            radius,
        } => {
            apply_pillar(grid, cx, cz, height, radius, layers, bounds, step);
        }

        VolumeFeature::Tunnel {
            center: (cx, cz),
            direction: (dx, dz),
            length,
            radius,
            depth,
        } => {
            apply_tunnel(grid, cx, cz, dx, dz, length, radius, depth, bounds, step);
        }

        VolumeFeature::Arch {
            from: (fx, fy, fz),
            to: (tx, ty, tz),
            radius,
            thickness,
        } => {
            apply_arch(
                grid, fx, fy, fz, tx, ty, tz, radius, thickness, layers, bounds, step,
            );
        }

        VolumeFeature::Caves {
            frequency,
            octaves,
            seed,
            ref depth_curve,
            ref region,
            ref material_layers,
            floor_bias,
            ref mouths,
        } => {
            let cave_layers = if material_layers.is_empty() {
                layers
            } else {
                material_layers
            };
            apply_caves(
                grid,
                terrain,
                frequency,
                octaves,
                seed,
                depth_curve,
                region.as_ref(),
                cave_layers,
                floor_bias,
                mouths,
                bounds,
                step,
            );
        }

        VolumeFeature::Overhang {
            from: (fx, fz),
            to: (tx, tz),
            height,
            depth,
            thickness,
            direction: (dx, dz),
            noise,
            noise_seed,
        } => {
            apply_overhang(
                grid, fx, fz, tx, tz, height, depth, thickness, dx, dz, noise, noise_seed, layers,
                bounds, step,
            );
        }

        VolumeFeature::Foundation {
            center: (cx, cy, cz),
            half_extents: (hx, hz),
            depth,
        } => {
            apply_foundation(grid, cx, cy, cz, hx, hz, depth, bounds, step);
        }

        // Traversal primitives are handled above, before the match.
        VolumeFeature::Path { .. }
        | VolumeFeature::Platform { .. }
        | VolumeFeature::Staircase { .. }
        | VolumeFeature::Shaft { .. } => {}
    }
}

/// Floating solid mass — a rounded box with optional noisy edges.
fn apply_island(
    grid: &mut ChunkGrid,
    cx: f32,
    cy: f32,
    cz: f32,
    hx: f32,
    hy: f32,
    hz: f32,
    edge_noise: f32,
    layers: &[MaterialLayer],
    bounds: &AABB,
    step: f32,
) {
    let noise_seed = ((cx * 1000.0) as u32).wrapping_add((cz * 1000.0) as u32);
    // Expand iteration bounds slightly for noise displacement
    let margin = edge_noise * 2.0;
    let min_x = (cx - hx - margin).max(bounds.min.x);
    let max_x = (cx + hx + margin).min(bounds.max.x);
    let min_y = (cy - hy - margin).max(bounds.min.y);
    let max_y = (cy + hy + margin).min(bounds.max.y);
    let min_z = (cz - hz - margin).max(bounds.min.z);
    let max_z = (cz + hz + margin).min(bounds.max.z);

    let mut x = min_x;
    while x < max_x {
        let mut y = min_y;
        while y < max_y {
            let mut z = min_z;
            while z < max_z {
                // Signed distance to rounded box (box with spherical edges)
                let dx = (x - cx).abs() - hx;
                let dy = (y - cy).abs() - hy;
                let dz = (z - cz).abs() - hz;
                let outside = nalgebra::Vector3::new(dx.max(0.0), dy.max(0.0), dz.max(0.0)).norm();
                let inside = dx.max(dy).max(dz).min(0.0);
                let mut sd = outside + inside;

                // Displace surface with noise for organic edges
                if edge_noise > 0.0 {
                    let n = fbm_2d_periodic(
                        x * 0.5 + y * 0.3,
                        z * 0.5 + y * 0.3,
                        3,
                        0.5,
                        2.0,
                        noise_seed,
                        None,
                    );
                    sd += (n - 0.5) * edge_noise * 2.0;
                }

                let depth = (-sd).max(0.0);
                let material = material_at_depth(depth, layers);
                union_solid(grid, Point3::new(x, y, z), sd, step, material);

                z += step;
            }
            y += step;
        }
        x += step;
    }
}

/// Vertical column rising from the heightfield surface.
fn apply_pillar(
    grid: &mut ChunkGrid,
    cx: f32,
    cz: f32,
    height: f32,
    radius: f32,
    layers: &[MaterialLayer],
    bounds: &AABB,
    step: f32,
) {
    // Pad by one voxel so the SDF shell on the pillar exterior lands inside
    // the iteration region (needed for MC to find the smooth boundary).
    let min_x = (cx - radius - step).max(bounds.min.x);
    let max_x = (cx + radius + step).min(bounds.max.x);
    let min_z = (cz - radius - step).max(bounds.min.z);
    let max_z = (cz + radius + step).min(bounds.max.z);
    let top_y = height.min(bounds.max.y);
    let max_y = (top_y + step).min(bounds.max.y);

    let mut x = min_x;
    while x < max_x {
        let mut z = min_z;
        while z < max_z {
            let dx = x - cx;
            let dz = z - cz;
            let dist = (dx * dx + dz * dz).sqrt();
            // Open at bottom, capped at top. SDF = max(radial, top_cap).
            let radial_sd = dist - radius;

            let mut y = bounds.min.y;
            while y < max_y {
                let top_sd = y - top_y;
                let sd = radial_sd.max(top_sd);
                let depth = (top_y - y).max(0.0);
                let material = material_at_depth(depth, layers);
                union_solid(grid, Point3::new(x, y, z), sd, step, material);
                y += step;
            }

            z += step;
        }
        x += step;
    }
}

/// Turn the ground inside a box, top at `top_y`, to bedrock.
///
/// Only the material changes. The shape is the ground's own, so the block adds
/// no surface: a top authored a few centimetres above uneven ground would
/// otherwise lay a sliver of solid over it, and marching cubes meshes such a
/// sliver badly.
fn apply_foundation(
    grid: &mut ChunkGrid,
    cx: f32,
    top_y: f32,
    cz: f32,
    hx: f32,
    hz: f32,
    depth: f32,
    bounds: &AABB,
    step: f32,
) {
    let span = |lo: f32, hi: f32, clip_lo: f32, clip_hi: f32| {
        index_range(lo.max(clip_lo), hi.min(clip_hi), step)
    };
    let (x0, x1) = span(cx - hx, cx + hx, bounds.min.x, bounds.max.x);
    let (y0, y1) = span(top_y - depth, top_y, bounds.min.y, bounds.max.y);
    let (z0, z1) = span(cz - hz, cz + hz, bounds.min.z, bounds.max.z);

    for ix in x0..=x1 {
        for iz in z0..=z1 {
            for iy in y0..=y1 {
                let p = Point3::new(ix as f32, iy as f32, iz as f32) * step;
                let inside = (p.x - cx).abs() <= hx
                    && (p.z - cz).abs() <= hz
                    && (top_y - depth..=top_y).contains(&p.y);
                if inside {
                    paint_solid(grid, p, VoxelMaterial::Bedrock);
                }
            }
        }
    }
}

/// Horizontal bore that carves a cylindrical tunnel through existing terrain.
fn apply_tunnel(
    grid: &mut ChunkGrid,
    cx: f32,
    cz: f32,
    dx: f32,
    dz: f32,
    length: f32,
    radius: f32,
    depth: f32,
    bounds: &AABB,
    step: f32,
) {
    // Normalize direction
    let dir_len = (dx * dx + dz * dz).sqrt();
    if dir_len < 1e-6 {
        return;
    }
    let ndx = dx / dir_len;
    let ndz = dz / dir_len;

    let half_len = length * 0.5;
    // Start and end points of the tunnel axis
    let ax = cx - ndx * half_len;
    let az = cz - ndz * half_len;
    let bx = cx + ndx * half_len;
    let bz = cz + ndz * half_len;

    // Bounding box of the tunnel, padded by one voxel for the SDF shell.
    let min_x = ax.min(bx) - radius - step;
    let max_x = ax.max(bx) + radius + step;
    let min_z = az.min(bz) - radius - step;
    let max_z = az.max(bz) + radius + step;
    let min_y = depth - radius - step;
    let max_y = depth + radius + step;

    let min_x = min_x.max(bounds.min.x);
    let max_x = max_x.min(bounds.max.x);
    let min_z = min_z.max(bounds.min.z);
    let max_z = max_z.min(bounds.max.z);
    let min_y = min_y.max(bounds.min.y);
    let max_y = max_y.min(bounds.max.y);

    let mut x = min_x;
    while x < max_x {
        let mut y = min_y;
        while y < max_y {
            let mut z = min_z;
            while z < max_z {
                // Project (x, z) onto the tunnel axis segment. Out-of-segment
                // positions aren't carved (skip).
                let (t, perp_xz) = point_to_segment_projection(x, z, ax, az, bx, bz);
                if t < 0.0 || t > 1.0 {
                    z += step;
                    continue;
                }
                // Signed distance to the cylindrical bore (positive outside).
                let dy = y - depth;
                let dist = (perp_xz * perp_xz + dy * dy).sqrt();
                let sd = dist - radius;
                carve_with_sdf(grid, Point3::new(x, y, z), sd, step);

                z += step;
            }
            y += step;
        }
        x += step;
    }
}

/// Distance from `(u, v)` in the arch's plane to its centre line: the upper
/// half of an ellipse with semi-axes `a` along the span and `b` up, centred on
/// the origin. Below the springing line the nearest point is a foot.
///
/// Inigo Quilez's first-order ellipse distance, exact for a circle. It must be
/// a distance in every direction: the legs are steep, and an offset measured
/// vertically draws them as slivers.
fn distance_to_arc(u: f32, v: f32, a: f32, b: f32) -> f32 {
    if v < 0.0 {
        return (u.abs() - a).hypot(v);
    }
    let k0 = (u / a).hypot(v / b);
    let k1 = (u / (a * a)).hypot(v / (b * b));
    if k1 < 1e-6 {
        return a.min(b);
    }
    (k0 * (k0 - 1.0) / k1).abs()
}

/// Curved bridge (half-ellipse) between two 3D points.
fn apply_arch(
    grid: &mut ChunkGrid,
    fx: f32,
    fy: f32,
    fz: f32,
    tx: f32,
    ty: f32,
    tz: f32,
    radius: f32,
    thickness: f32,
    layers: &[MaterialLayer],
    bounds: &AABB,
    step: f32,
) {
    // The arch is a semicircular arc in the vertical plane between from and to,
    // peaking at `radius` above the midpoint height.
    let mid_x = (fx + tx) * 0.5;
    let mid_y = (fy + ty) * 0.5;
    let mid_z = (fz + tz) * 0.5;

    let span_x = tx - fx;
    let span_y = ty - fy;
    let span_z = tz - fz;
    let span_len = (span_x * span_x + span_y * span_y + span_z * span_z).sqrt();
    if span_len < 1e-6 {
        return;
    }

    let half_span = span_len * 0.5;
    let peak_y = mid_y + radius;
    let half_t = thickness * 0.5;

    // Bounding box
    let min_x = fx.min(tx) - half_t;
    let max_x = fx.max(tx) + half_t;
    let min_y = fy.min(ty) - half_t;
    let max_y = peak_y + half_t;
    let min_z = fz.min(tz) - half_t;
    let max_z = fz.max(tz) + half_t;

    let min_x = min_x.max(bounds.min.x);
    let max_x = max_x.min(bounds.max.x);
    let min_y = min_y.max(bounds.min.y);
    let max_y = max_y.min(bounds.max.y);
    let min_z = min_z.max(bounds.min.z);
    let max_z = max_z.min(bounds.max.z);

    // Normalized span direction (horizontal)
    let dir_x = span_x / span_len;
    let dir_z = span_z / span_len;

    let mut x = min_x;
    while x < max_x {
        let mut y = min_y;
        while y < max_y {
            let mut z = min_z;
            while z < max_z {
                // Project onto the horizontal span axis
                let rel_x = x - mid_x;
                let rel_z = z - mid_z;
                let along = rel_x * dir_x + rel_z * dir_z;
                let perp_h =
                    ((rel_x - along * dir_x).powi(2) + (rel_z - along * dir_z).powi(2)).sqrt();

                let to_curve = distance_to_arc(along, y - mid_y, half_span, radius);
                // Square cross-section: the larger of the offset across the
                // span and the offset from the curve within its plane.
                let sd = perp_h.max(to_curve) - half_t;

                let material = material_at_depth(0.5, layers);
                union_solid(grid, Point3::new(x, y, z), sd, step, material);

                z += step;
            }
            y += step;
        }
        x += step;
    }
}

/// Solid rock lip extending horizontally from a cliff edge.
///
/// The lip is thickest at the cliff edge and tapers linearly to zero at
/// the outer extent (`depth`). Noise displaces the underside for an
/// organic, weathered look.
#[allow(clippy::too_many_arguments)]
fn apply_overhang(
    grid: &mut ChunkGrid,
    fx: f32,
    fz: f32,
    tx: f32,
    tz: f32,
    height: f32,
    depth: f32,
    thickness: f32,
    dx: f32,
    dz: f32,
    noise: f32,
    noise_seed: u32,
    layers: &[MaterialLayer],
    bounds: &AABB,
    step: f32,
) {
    // Normalise the outward direction
    let dir_len = (dx * dx + dz * dz).sqrt();
    if dir_len < 1e-6 {
        return;
    }
    let ndx = dx / dir_len;
    let ndz = dz / dir_len;

    // Edge direction (along the cliff line)
    let edx = tx - fx;
    let edz = tz - fz;
    let edge_len = (edx * edx + edz * edz).sqrt();
    if edge_len < 1e-6 {
        return;
    }

    // Bounding box of the overhang volume
    let corners_x = [fx, tx, fx + ndx * depth, tx + ndx * depth];
    let corners_z = [fz, tz, fz + ndz * depth, tz + ndz * depth];
    let bb_min_x = corners_x.iter().copied().reduce(f32::min).unwrap() - step;
    let bb_max_x = corners_x.iter().copied().reduce(f32::max).unwrap() + step;
    let bb_min_z = corners_z.iter().copied().reduce(f32::min).unwrap() - step;
    let bb_max_z = corners_z.iter().copied().reduce(f32::max).unwrap() + step;
    let bb_min_y = height - thickness - noise;
    let bb_max_y = height + step;

    let min_x = bb_min_x.max(bounds.min.x);
    let max_x = bb_max_x.min(bounds.max.x);
    let min_z = bb_min_z.max(bounds.min.z);
    let max_z = bb_max_z.min(bounds.max.z);
    let min_y = bb_min_y.max(bounds.min.y);
    let max_y = bb_max_y.min(bounds.max.y);

    let mut x = min_x;
    while x < max_x {
        let mut z = min_z;
        while z < max_z {
            // Project (x,z) onto the cliff edge to get the "along" parameter
            // and the perpendicular outward distance.
            let (along_t, _perp_to_edge) = point_to_segment_projection(x, z, fx, fz, tx, tz);
            if along_t < 0.0 || along_t > 1.0 {
                z += step;
                continue;
            }

            // Closest point on the edge
            let ex = fx + along_t * edx;
            let ez = fz + along_t * edz;

            // Outward distance from the edge in the lip direction
            let rel_x = x - ex;
            let rel_z = z - ez;
            let outward = rel_x * ndx + rel_z * ndz;

            if outward < 0.0 || outward > depth {
                z += step;
                continue;
            }

            // Taper: full thickness at the edge, zero at the outer extent
            let taper_t = 1.0 - outward / depth;
            let local_thickness = thickness * taper_t;

            // Noise displacement on the underside
            let underside_offset = if noise > 0.0 {
                let n = fbm_2d_periodic(x * 0.5, z * 0.5, 3, 0.5, 2.0, noise_seed, None);
                (n - 0.5) * noise * 2.0 * taper_t
            } else {
                0.0
            };

            let top_y = height;
            let bottom_y = height - local_thickness + underside_offset;
            let mid_y = (top_y + bottom_y) * 0.5;
            let half_thick = (top_y - bottom_y) * 0.5;

            let mut y = min_y;
            while y < max_y {
                // Signed distance to the vertical slab at this (x,z).
                let sd = (y - mid_y).abs() - half_thick;
                let depth_in_lip = (top_y - y).max(0.0);
                let material = material_at_depth(depth_in_lip, layers);
                union_solid(grid, Point3::new(x, y, z), sd, step, material);
                y += step;
            }

            z += step;
        }
        x += step;
    }
}

/// 3D noise-driven cave carving with depth-dependent threshold.
///
/// For each solid voxel, computes its depth below the heightfield surface,
/// evaluates 3D FBM noise, and carves to air if the noise exceeds the
/// depth-curve threshold. When a `CaveRegion` is specified, the threshold
/// fades toward 1.0 (no carving) outside the region. After carving, fixes
/// up surface materials using cave-specific layers based
/// on material type and depth from the cave wall.
fn apply_caves(
    grid: &mut ChunkGrid,
    terrain: &Terrain,
    frequency: f32,
    octaves: u32,
    seed: u32,
    depth_curve: &[CaveDepthPoint],
    region: Option<&CaveRegion>,
    cave_layers: &[MaterialLayer],
    floor_bias: f32,
    mouths: &[CaveMouth],
    bounds: &AABB,
    step: f32,
) {
    // Compute iteration bounds — clip to the region's outer envelope if present.
    let (min_x, min_y, min_z, max_x, max_y, max_z) = if let Some(r) = region {
        let outer = r.radius + r.falloff;
        (
            (r.center.0 - outer).max(bounds.min.x),
            (r.center.1 - outer).max(bounds.min.y),
            (r.center.2 - outer).max(bounds.min.z),
            (r.center.0 + outer).min(bounds.max.x),
            (r.center.1 + outer).min(bounds.max.y),
            (r.center.2 + outer).min(bounds.max.z),
        )
    } else {
        (
            bounds.min.x,
            bounds.min.y,
            bounds.min.z,
            bounds.max.x,
            bounds.max.y,
            bounds.max.z,
        )
    };

    // Pass A: carve caves based on 3D noise and depth curve.
    let mut x = min_x;
    while x < max_x {
        let mut z = min_z;
        while z < max_z {
            let surface_y = height_at(x, z, terrain);

            let mut y = min_y;
            while y < max_y {
                let depth = surface_y - y;
                if depth <= 0.0 {
                    y += step;
                    continue;
                }

                if grid.get(Point3::new(x, y, z)).density <= 0.0 {
                    y += step;
                    continue;
                }

                let mut threshold = sample_depth_curve(depth, depth_curve);

                // Blend threshold toward 1.0 based on distance from region center.
                if let Some(r) = region {
                    let dx = x - r.center.0;
                    let dy = y - r.center.1;
                    let dz = z - r.center.2;
                    let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                    if dist > r.radius {
                        if r.falloff <= 0.0 || dist >= r.radius + r.falloff {
                            y += step;
                            continue;
                        }
                        let fade = (dist - r.radius) / r.falloff;
                        threshold = threshold + (1.0 - threshold) * fade;
                    }
                }

                if threshold >= 1.0 {
                    y += step;
                    continue;
                }

                let mut noise = fbm_3d(
                    x * frequency,
                    y * frequency,
                    z * frequency,
                    octaves,
                    0.5,
                    2.0,
                    seed,
                );

                // Bias noise downward below the cave midpoint to flatten floors.
                // Voxels near the bottom of a cave get a lower noise value,
                // making them less likely to be carved.
                if floor_bias > 0.0 {
                    let cave_mid_y = surface_y - depth * 0.5;
                    if y < cave_mid_y {
                        let below_t = (cave_mid_y - y) / (cave_mid_y - min_y).max(1.0);
                        noise -= floor_bias * below_t;
                    }
                }

                // SDF-style carve: distance proxy is (threshold - noise),
                // scaled so a full-voxel gradient maps to density ±1. Noise
                // gradients aren't true distances, but at unit step this gives
                // a reasonable smooth cave wall for MC interpolation.
                let sdf_carve = threshold - noise;
                carve_with_sdf(grid, Point3::new(x, y, z), sdf_carve, step);

                y += step;
            }
            z += step;
        }
        x += step;
    }

    // Pass A2: bore the mouths in from the open. The loader holds a region
    // to every cave with mouths: the centre is where they lead.
    let (mut min, mut max) = (
        Point3::new(min_x, min_y, min_z),
        Point3::new(max_x, max_y, max_z),
    );
    if let Some(r) = region {
        let noise = CaveNoise {
            frequency,
            octaves,
            seed,
        };
        let centre = Point3::new(r.center.0, r.center.1, r.center.2);
        for mouth in mouths {
            let (lo, hi) = carve_cave_mouth(grid, mouth, centre, &noise, bounds, step);
            min = min.inf(&lo);
            max = max.sup(&hi);
        }
    }

    // Pass B: reassign materials on cave-exposed surfaces.
    fixup_cave_materials(
        grid,
        cave_layers,
        min.x,
        min.z,
        max.x,
        max.z,
        min.y,
        max.y,
        step,
    );
}

/// The noise a cave carves by, which its mouths roughen their walls with too.
struct CaveNoise {
    frequency: f32,
    octaves: u32,
    seed: u32,
}

impl CaveNoise {
    /// The noise at `p`, centred on zero: about -1 to 1.
    fn centred(&self, p: Point3<f32>) -> f32 {
        let f = self.frequency;
        fbm_3d(p.x * f, p.y * f, p.z * f, self.octaves, 0.5, 2.0, self.seed) * 2.0 - 1.0
    }
}

/// How far the cave's noise moves a mouth's walls in and out, as a share of
/// its smaller half-extent.
const MOUTH_ROUGHNESS: f32 = 0.35;

/// How much finer than the cave's own noise a mouth's roughness is. At the
/// cave's scale a mouth's few metres of wall take one gentle bulge and the
/// opening reads as a bored arch; finer, it breaks into the crags of a cleft.
const MOUTH_DETAIL: f32 = 3.0;

/// Bore `mouth` from its point in the open to `centre`: a passage of elliptical
/// section, its walls roughened by the cave's noise. Returns the box it
/// carved within, clipped to `bounds`.
fn carve_cave_mouth(
    grid: &mut ChunkGrid,
    mouth: &CaveMouth,
    centre: Point3<f32>,
    noise: &CaveNoise,
    bounds: &AABB,
    step: f32,
) -> (Point3<f32>, Point3<f32>) {
    let start = Point3::new(mouth.from.0, mouth.from.1, mouth.from.2);
    let axis = centre - start;
    let length_sq = axis.norm_squared().max(1e-6);
    let smaller = mouth.half_width.min(mouth.half_height);
    let reach = mouth.half_width.max(mouth.half_height) * (1.0 + MOUTH_ROUGHNESS) + step;
    let pad = nalgebra::Vector3::repeat(reach);
    let lo = (start.inf(&centre) - pad).sup(&bounds.min);
    let hi = (start.sup(&centre) + pad).inf(&bounds.max);

    let (i0, i1) = index_range(lo.x, hi.x, step);
    let (j0, j1) = index_range(lo.y, hi.y, step);
    let (k0, k1) = index_range(lo.z, hi.z, step);
    for i in i0..=i1 {
        for j in j0..=j1 {
            for k in k0..=k1 {
                let p = Point3::new(i as f32 * step, j as f32 * step, k as f32 * step);
                let t = ((p - start).dot(&axis) / length_sq).clamp(0.0, 1.0);
                let off = p - (start + axis * t);
                let across = off.x.hypot(off.z) / mouth.half_width;
                let up = off.y / mouth.half_height;
                // Distance to the elliptical wall, in metres of the smaller
                // half-extent: exact on the axes, close enough between.
                let wall = (across.hypot(up) - 1.0) * smaller
                    + MOUTH_ROUGHNESS * smaller * noise.centred(p * MOUTH_DETAIL);
                if wall < step {
                    carve_with_sdf(grid, p, wall, step);
                }
            }
        }
    }
    (lo, hi)
}

/// After cave carving, reassign materials on newly-exposed underground surfaces
/// using cave-specific layers.
///
/// The first solid run from the top of each column is the terrain surface —
/// those voxels keep the terrain's material layers. Only after passing through
/// an underground air gap (a cave) do we switch to cave layers.
fn fixup_cave_materials(
    grid: &mut ChunkGrid,
    cave_layers: &[MaterialLayer],
    min_x: f32,
    min_z: f32,
    max_x: f32,
    max_z: f32,
    min_y: f32,
    max_y: f32,
    step: f32,
) {
    let mut x = min_x;
    while x < max_x {
        let mut z = min_z;
        while z < max_z {
            let mut depth_below_surface = 0.0_f32;
            let mut in_solid = false;
            // Whether we've passed through at least one underground air gap.
            // The first solid run is the terrain surface; subsequent runs
            // after air gaps are cave walls/ceilings.
            let mut seen_underground_air = false;

            let mut y = max_y - step;
            while y >= min_y {
                let voxel = grid.get(Point3::new(x, y, z));
                if voxel.density > 0.0 {
                    if !in_solid {
                        depth_below_surface = 0.0;
                        in_solid = true;
                    }
                    // The first run is the terrain's own surface, laid by the
                    // heightfield with its bands and faces; it keeps them.
                    if !seen_underground_air {
                        y -= step;
                        continue;
                    }
                    let material = material_at_depth(depth_below_surface, cave_layers);
                    // The world's floor is not a surface to re-skin: a cave that
                    // reaches bedrock must not turn it into ordinary rock.
                    if voxel.material != material && !voxel.material.is_indestructible() {
                        // Preserve the SDF density while updating the material.
                        grid.set(
                            Point3::new(x, y, z),
                            Voxel {
                                density: voxel.density,
                                material,
                            },
                        );
                    }
                    depth_below_surface += step;
                } else {
                    if in_solid {
                        seen_underground_air = true;
                    }
                    in_solid = false;
                }
                y -= step;
            }
            z += step;
        }
        x += step;
    }
}

/// Linearly interpolate the carve threshold from a depth curve.
///
/// For depths before the first point, uses the first point's threshold.
/// For depths beyond the last point, uses the last point's threshold.
fn sample_depth_curve(depth: f32, curve: &[CaveDepthPoint]) -> f32 {
    if curve.is_empty() {
        return 1.0;
    }
    if depth <= curve[0].depth {
        return curve[0].threshold;
    }
    for i in 1..curve.len() {
        if depth <= curve[i].depth {
            let prev = &curve[i - 1];
            let t = (depth - prev.depth) / (curve[i].depth - prev.depth);
            return prev.threshold + (curve[i].threshold - prev.threshold) * t;
        }
    }
    curve[curve.len() - 1].threshold
}

/// Map a depth below the surface to a `VoxelMaterial` using the level's
/// material layers. Falls through to Rock if no layer matches.
fn material_at_depth(depth: f32, layers: &[MaterialLayer]) -> VoxelMaterial {
    for layer in layers {
        if depth < layer.depth {
            return to_voxel_material(layer.material);
        }
    }
    // Default if no layers defined or depth exceeds all layers
    if depth < 1.0 {
        VoxelMaterial::Grass
    } else if depth < 4.0 {
        VoxelMaterial::Dirt
    } else {
        VoxelMaterial::Rock
    }
}

pub(super) fn to_voxel_material(id: VoxelMaterialId) -> VoxelMaterial {
    match id {
        VoxelMaterialId::Grass => VoxelMaterial::Grass,
        VoxelMaterialId::Dirt => VoxelMaterial::Dirt,
        VoxelMaterialId::Rock => VoxelMaterial::Rock,
        VoxelMaterialId::Ite => VoxelMaterial::Ite,
        VoxelMaterialId::Limestone => VoxelMaterial::Limestone,
        VoxelMaterialId::Slate => VoxelMaterial::Slate,
        VoxelMaterialId::Sand => VoxelMaterial::Sand,
    }
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

/// Signed perpendicular distance from point (px, pz) to the infinite line
/// through (ax, az)→(bx, bz). Positive when the point is on the same side
/// as the `high_side` direction (hx, hz).
fn signed_dist_to_line(
    px: f32,
    pz: f32,
    ax: f32,
    az: f32,
    bx: f32,
    bz: f32,
    hx: f32,
    hz: f32,
) -> f32 {
    let edx = bx - ax;
    let edz = bz - az;
    let len = (edx * edx + edz * edz).sqrt();
    if len < 1e-10 {
        return 0.0;
    }
    // Outward normal of the edge (perpendicular in 2D)
    let nx = -edz / len;
    let nz = edx / len;
    // Flip so the normal points toward the high side
    let dot_high = nx * hx + nz * hz;
    let sign = if dot_high >= 0.0 { 1.0 } else { -1.0 };
    let nx = nx * sign;
    let nz = nz * sign;
    // Signed distance: positive on high side
    (px - ax) * nx + (pz - az) * nz
}

/// Sigmoid function mapping any real value to (0, 1).
fn sigmoid(x: f32) -> f32 {
    1.0 / (1.0 + (-x).exp())
}

/// Shortest distance from point (px, pz) to line segment (ax, az)→(bx, bz).
fn point_to_segment_dist(px: f32, pz: f32, ax: f32, az: f32, bx: f32, bz: f32) -> f32 {
    let (_, dist) = point_to_segment_projection(px, pz, ax, az, bx, bz);
    dist
}

/// Project point (px, pz) onto segment (ax, az)→(bx, bz).
/// Returns (t, perpendicular_distance) where t is the clamped [0,1] parameter.
pub(super) fn point_to_segment_projection(
    px: f32,
    pz: f32,
    ax: f32,
    az: f32,
    bx: f32,
    bz: f32,
) -> (f32, f32) {
    let abx = bx - ax;
    let abz = bz - az;
    let len_sq = abx * abx + abz * abz;
    if len_sq < 1e-10 {
        let dx = px - ax;
        let dz = pz - az;
        return (0.0, (dx * dx + dz * dz).sqrt());
    }
    let apx = px - ax;
    let apz = pz - az;
    let t = ((apx * abx + apz * abz) / len_sq).clamp(0.0, 1.0);
    let closest_x = ax + t * abx;
    let closest_z = az + t * abz;
    let dx = px - closest_x;
    let dz = pz - closest_z;
    (t, (dx * dx + dz * dz).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::level::{Extent, SurfaceBand, UndersideFeature};

    /// A plateau laid with grass over dirt, its cliffs bare rock. Its coast
    /// is warped, so the lip where the slope passes the band's runs across
    /// the lattice, and its rise is short, so the slope passes the band's
    /// within a column, as on the island's massif.
    fn cliffed_plateau(voxel_size: f32) -> Terrain {
        Terrain {
            bedrock_thickness: 0.0,
            underside: Vec::new(),
            voxel_size,
            bounds: Extent {
                min: (-8.0, 0.0, -8.0),
                max: (8.0, 10.0, 8.0),
            },
            base_height: 2.0,
            material_layers: vec![
                MaterialLayer {
                    depth: 0.6,
                    material: VoxelMaterialId::Grass,
                },
                MaterialLayer {
                    depth: 999.0,
                    material: VoxelMaterialId::Dirt,
                },
            ],
            surface_bands: vec![SurfaceBand {
                below: None,
                steeper_than: Some(1.5),
                cut_deeper_than: None,
                layers: vec![MaterialLayer {
                    depth: 999.0,
                    material: VoxelMaterialId::Rock,
                }],
            }],
            smooth_slopes: true,
            features: vec![TerrainFeature::Landmass {
                outline: vec![(-5.0, -5.0), (5.0, -5.0), (5.0, 5.0), (-5.0, 5.0)],
                shore_height: 2.0,
                height: 7.0,
                rise: 1.5,
                skirt: 0.0,
                warp: 1.0,
                relief: 0.0,
                seed: 4,
            }],
            volumes: Vec::new(),
        }
    }

    /// A ramp standing 6.55 m over open ground at its high end, its flank a
    /// sheer drop a few 0.25 m columns wide, laid grass over dirt over rock.
    fn sheer_ramp() -> Terrain {
        Terrain {
            bedrock_thickness: 0.0,
            underside: Vec::new(),
            voxel_size: 0.25,
            bounds: Extent {
                min: (0.0, 0.0, -4.0),
                max: (16.0, 8.0, 4.0),
            },
            base_height: 0.0,
            material_layers: [
                (1.0, VoxelMaterialId::Grass),
                (4.0, VoxelMaterialId::Dirt),
                (999.0, VoxelMaterialId::Rock),
            ]
            .into_iter()
            .map(|(depth, material)| MaterialLayer { depth, material })
            .collect(),
            surface_bands: Vec::new(),
            smooth_slopes: true,
            features: vec![TerrainFeature::Ramp {
                from: (7.0, 0.0),
                to: (13.0, 0.0),
                start_height: 0.0,
                end_height: 7.15,
                width: 5.0,
                flat_width: 4.0,
            }],
            volumes: Vec::new(),
        }
    }

    /// Land at 3 m hung on a keel inside an 8.1 m square (off the lattice):
    /// a rim at 1 m, a bottom at -8.1 m. The surface runs on past the
    /// outline, so the keel's side is what ends the land.
    fn hung_island() -> Terrain {
        Terrain {
            bedrock_thickness: 2.0,
            underside: vec![UndersideFeature::Keel {
                outline: vec![(-8.1, -8.1), (8.1, -8.1), (8.1, 8.1), (-8.1, 8.1)],
                rim: 1.0,
                bottom: -8.1,
                taper: 6.0,
                warp: 0.0,
                roughness: 0.0,
                seed: 0,
                material: VoxelMaterialId::Slate,
            }],
            voxel_size: 0.5,
            bounds: Extent {
                min: (-40.0, -40.0, -40.0),
                max: (40.0, 10.0, 40.0),
            },
            base_height: 3.0,
            material_layers: vec![
                MaterialLayer {
                    depth: 0.6,
                    material: VoxelMaterialId::Grass,
                },
                MaterialLayer {
                    depth: 999.0,
                    material: VoxelMaterialId::Dirt,
                },
            ],
            surface_bands: Vec::new(),
            smooth_slopes: true,
            features: Vec::new(),
            volumes: Vec::new(),
        }
    }

    fn generated(terrain: &Terrain) -> ChunkGrid {
        let (min, max) = (terrain.bounds.min, terrain.bounds.max);
        let bounds = AABB::new(
            Point3::new(min.0, min.1, min.2),
            Point3::new(max.0, max.1, max.2),
        );
        let mut grid = ChunkGrid::new(terrain.voxel_size);
        generate_terrain(&mut grid, terrain, &bounds);
        grid
    }

    /// Where the surface marching cubes draws crosses the lattice edge from
    /// `a` to the next sample along `axis`.
    fn crossing(grid: &ChunkGrid, a: Point3<f32>, axis: usize, step: f32) -> f32 {
        let mut b = a;
        b[axis] += step;
        let (da, db) = (grid.get(a).density, grid.get(b).density);
        assert!(
            da * db < 0.0,
            "no surface between {a} ({da}) and {b} ({db})"
        );
        a[axis] + step * da / (da - db)
    }

    /// A hung terrain is solid only between its keel's underside and its
    /// surface, inside the keel's outline: not around the island, and not
    /// down to the floor of its bounds, where a slab would stand.
    #[test]
    fn a_keel_hangs_the_land_in_the_air() {
        let terrain = hung_island();
        let grid = generated(&terrain);
        let solid = |x: f32, y: f32, z: f32| grid.get(Point3::new(x, y, z)).density > 0.0;

        assert!(solid(0.0, 0.0, 0.0) && solid(0.0, 2.5, 0.0) && solid(7.5, 2.0, 0.0));
        assert!(!solid(0.0, -9.0, 0.0), "below the keel");
        assert!(!solid(0.0, -39.0, 0.0), "the floor of the bounds");
        assert!(!solid(0.0, 3.5, 0.0), "above the surface");
        for y in [-8.0, -2.0, 0.0, 2.0, 2.5] {
            assert!(!solid(9.0, y, 0.0), "beside the island at {y} m");
            assert!(!solid(20.0, y, 20.0), "far off it at {y} m");
        }
        let allocated = grid.allocated_bounds().expect("an island");
        assert!(
            allocated.min.y > -20.0 && allocated.max.x < 20.0,
            "chunks are allocated only round the island: {allocated:?}"
        );
    }

    /// The island's edge and underside are encoded as distances, so marching
    /// cubes draws them where the keel puts them, between samples, and not
    /// where the lattice switches columns on and off: the coast of a warped
    /// outline is then a curve, not a staircase.
    #[test]
    fn a_keels_side_and_underside_lie_where_it_puts_them() {
        let terrain = hung_island();
        let step = terrain.voxel_size;
        let grid = generated(&terrain);

        let side = crossing(&grid, Point3::new(8.0, 2.0, 0.0), 0, step);
        assert!((side - 8.1).abs() < 0.02, "side at x = {side}");
        let underside = crossing(&grid, Point3::new(0.0, -8.5, 0.0), 1, step);
        assert!(
            (underside + 8.1).abs() < 0.02,
            "underside at y = {underside}"
        );
    }

    /// What lies nearer the underside than the surface is the keel's
    /// material; the surface's own layers lay the rest, turf on top. The
    /// bedrock band at the floor of the bounds is far below and lays nothing.
    #[test]
    fn a_keels_underside_is_its_own_material() {
        let terrain = hung_island();
        let grid = generated(&terrain);
        let material = |x: f32, y: f32| grid.get(Point3::new(x, y, 0.0)).material;

        assert_eq!(material(0.0, 2.5), VoxelMaterial::Grass);
        assert_eq!(material(0.0, -1.0), VoxelMaterial::Dirt);
        assert_eq!(material(0.0, -7.5), VoxelMaterial::Slate);
        // 0.6 m in from the rim the belly is down at -0.7 m: soil over the
        // keel's stone, parted halfway.
        assert_eq!(material(7.5, 2.0), VoxelMaterial::Dirt);
        assert_eq!(material(7.5, -0.5), VoxelMaterial::Slate);
    }

    /// A brook cut 1 m into a lawn at 3 m, and beside it a crater no
    /// watercourse cut, as deep: only the brook's bed is laid as a bed.
    fn brook_and_crater() -> Terrain {
        let layers = |material| {
            vec![
                MaterialLayer {
                    depth: 0.6,
                    material,
                },
                MaterialLayer {
                    depth: 999.0,
                    material: VoxelMaterialId::Dirt,
                },
            ]
        };
        Terrain {
            bedrock_thickness: 0.0,
            underside: Vec::new(),
            voxel_size: 0.5,
            bounds: Extent {
                min: (0.0, 0.0, -8.0),
                max: (24.0, 6.0, 8.0),
            },
            base_height: 3.0,
            material_layers: layers(VoxelMaterialId::Grass),
            surface_bands: vec![SurfaceBand {
                below: None,
                steeper_than: None,
                cut_deeper_than: Some(0.25),
                layers: layers(VoxelMaterialId::Sand),
            }],
            smooth_slopes: true,
            features: vec![
                TerrainFeature::Crater {
                    center: (18.0, 0.0),
                    radius: 4.0,
                    depth: 1.0,
                },
                TerrainFeature::Channel {
                    points: vec![(0.0, 2.0, -4.0), (12.0, 2.0, -4.0)],
                    width: 2.0,
                    bank: 1.0,
                    warp: 0.0,
                    seed: 0,
                },
            ],
            volumes: Vec::new(),
        }
    }

    /// A band can lay what watercourses cut, wherever it lies: a brook's
    /// bed is sand, the lawn it runs through and a hollow as deep that no
    /// watercourse cut are turf.
    #[test]
    fn a_watercourse_bed_is_laid_by_the_band_for_what_is_cut() {
        let terrain = brook_and_crater();
        let grid = generated(&terrain);
        let top = |x: f32, z: f32| {
            let height = debias_height(height_at(x, z, &terrain), terrain.voxel_size);
            let y = top_voxel_index(height, terrain.voxel_size) as f32 * terrain.voxel_size;
            grid.get(Point3::new(x, y, z)).material
        };
        assert_eq!(top(6.0, -4.0), VoxelMaterial::Sand, "the brook's bed");
        assert_eq!(top(6.0, 4.0), VoxelMaterial::Grass, "the lawn");
        assert_eq!(top(18.0, 0.0), VoxelMaterial::Grass, "the crater's floor");
        assert!(height_at(18.0, 0.0, &terrain) < height_at(6.0, -4.0, &terrain) + 0.1);
    }

    /// A column partway down a sheer flank takes its layers from the top of
    /// the flank, not from its own top: laid from its own, it began with turf
    /// again, and the flank wore stripes of turf and soil along the ramp.
    #[test]
    fn a_sheer_flank_is_laid_from_its_top() {
        let terrain = sheer_ramp();
        let step = terrain.voxel_size;
        let bounds = AABB::new(Point3::new(0.0, 0.0, -4.0), Point3::new(16.0, 8.0, 4.0));
        let mut grid = ChunkGrid::new(step);
        generate_terrain(&mut grid, &terrain, &bounds);

        let top_material = |x: f32, z: f32| {
            let height = debias_height(height_at(x, z, &terrain), step);
            let y = top_voxel_index(height, step) as f32 * step;
            (height, grid.get(Point3::new(x, y, z)).material)
        };
        // The flank's top column, where the flat run ends.
        let (brink, at_brink) = top_material(12.5, 2.0);
        // The next column out, partway down the drop.
        let (partway, below_brink) = top_material(12.5, 2.25);
        assert!(
            brink - partway > 1.0 && partway > 1.0,
            "expected a sheer flank: brink {brink}, partway {partway}"
        );
        assert_eq!(at_brink, VoxelMaterial::Grass, "the turf runs to the brink");
        assert_eq!(
            below_brink,
            VoxelMaterial::Dirt,
            "{:.2} m below the brink is soil, not turf",
            brink - partway
        );
    }

    /// A column flat on one side is not on a slope, and encodes no stretch:
    /// a plateau's last column above a sheer drop once took the drop's
    /// stretch, and its rim's normals swung back into the plateau.
    #[test]
    fn a_brink_or_a_foot_is_encoded_straight_down() {
        let step = 0.25;
        // +x, -x, +z, -z: the plateau inward, the drop outward.
        assert_eq!(encoded_slope(8.0, &[8.0, 0.0, 8.0, 8.0], step), 0.0);
        // The ground at the wall's foot.
        assert_eq!(encoded_slope(0.0, &[8.0, 0.0, 0.0, 0.0], step), 0.0);
        // A ridge falls both ways and is no slope either.
        assert_eq!(encoded_slope(1.0, &[0.5, 0.5, 1.0, 1.0], step), 0.0);
        // An even slope keeps its slope, and a bending one the gentler side's.
        assert_eq!(encoded_slope(1.0, &[1.5, 0.5, 1.0, 1.0], step), 2.0);
        assert_eq!(encoded_slope(1.0, &[1.25, 0.0, 1.0, 1.0], step), 1.0);
    }

    /// No soil below a column's top voxel opens onto a cliff. The last column
    /// short of the cliff band once laid its grass and dirt down the drop
    /// beside the first column past it: teeth of turf hanging down the cliff
    /// wherever the lip ran across the lattice.
    #[test]
    fn a_drop_beside_a_column_is_laid_as_a_face() {
        let voxel_size = 0.5;
        let terrain = cliffed_plateau(voxel_size);
        let bounds = AABB::new(Point3::new(-8.0, 0.0, -8.0), Point3::new(8.0, 10.0, 8.0));
        let mut grid = ChunkGrid::new(voxel_size);
        generate_terrain(&mut grid, &terrain, &bounds);

        let solid = |p: Point3<f32>| grid.get(p).density > 0.0;
        let (mut soil, mut bared) = (0, 0);
        let n = (8.0 / voxel_size) as i32;
        // Clear of the bounds' walls and floor, which open every side.
        for ix in -n + 2..n - 1 {
            for iz in -n + 2..n - 1 {
                for iy in (1.0 / voxel_size) as i32..(10.0 / voxel_size) as i32 {
                    let p = Point3::new(ix as f32, iy as f32, iz as f32) * voxel_size;
                    let voxel = grid.get(p);
                    let is_soil =
                        matches!(voxel.material, VoxelMaterial::Grass | VoxelMaterial::Dirt);
                    // The top voxel is the turf's edge, and keeps its soil.
                    let covered = solid(p + nalgebra::Vector3::y() * voxel_size);
                    if voxel.density <= 0.0 || !is_soil || !covered {
                        continue;
                    }
                    soil += 1;
                    // A side open here and a voxel down is a drop of two
                    // voxels or more: a slope of 2, past the band's 1.5, so
                    // a cliff. A shallower drop is a bank the band leaves to
                    // the soil.
                    let sides = [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)];
                    if sides.iter().any(|&(dx, dz)| {
                        let beside = p + nalgebra::Vector3::new(dx, 0.0, dz) * voxel_size;
                        !solid(beside) && !solid(beside - nalgebra::Vector3::y() * voxel_size)
                    }) {
                        bared += 1;
                    }
                }
            }
        }
        assert!(
            soil > 0,
            "the plateau lays no soil, so the check proves nothing"
        );
        assert_eq!(
            bared, 0,
            "{bared} of {soil} buried soil voxels open onto a cliff"
        );
    }

    /// A foundation turns the ground inside it to bedrock, however deep, and
    /// changes nothing else: no density anywhere, so no surface, and no
    /// material outside it. Its top is authored above the ground's, as an
    /// author placing it under a building on uneven ground would.
    #[test]
    fn a_foundation_turns_ground_to_bedrock_and_changes_no_shape() {
        let voxel_size = 0.5;
        let flat = Terrain {
            features: Vec::new(),
            ..cliffed_plateau(voxel_size)
        };
        let with_foundation = Terrain {
            features: Vec::new(),
            volumes: vec![VolumeFeature::Foundation {
                center: (0.0, 2.2, 0.0),
                half_extents: (3.0, 2.0),
                depth: 1.7,
            }],
            ..cliffed_plateau(voxel_size)
        };
        let bounds = AABB::new(Point3::new(-8.0, 0.0, -8.0), Point3::new(8.0, 10.0, 8.0));
        let generate = |terrain: &Terrain| {
            let mut grid = ChunkGrid::new(voxel_size);
            generate_terrain(&mut grid, terrain, &bounds);
            grid
        };
        let (before, after) = (generate(&flat), generate(&with_foundation));

        let n = (8.0 / voxel_size) as i32;
        let (mut painted, mut reshaped, mut changed_outside) = (0, 0, 0);
        for ix in -n + 1..n {
            for iz in -n + 1..n {
                for iy in 1..(10.0 / voxel_size) as i32 {
                    let p = Point3::new(ix as f32, iy as f32, iz as f32) * voxel_size;
                    let in_block =
                        p.x.abs() <= 3.0 && p.z.abs() <= 2.0 && (0.5..=2.2).contains(&p.y);
                    let (old, new) = (before.get(p), after.get(p));
                    if old.density != new.density {
                        reshaped += 1;
                    }
                    if in_block && old.density > 0.0 {
                        painted += 1;
                        assert_eq!(new.material, VoxelMaterial::Bedrock, "{p:?} isn't bedrock");
                    } else if old.material != new.material {
                        changed_outside += 1;
                    }
                }
            }
        }
        assert!(
            painted > 0,
            "no ground fell inside the block, so the check proves nothing"
        );
        assert_eq!(reshaped, 0, "{reshaped} samples changed density");
        assert_eq!(
            changed_outside, 0,
            "{changed_outside} samples outside the ground in the block changed material"
        );
    }

    /// A cave mouth bores from the open through the plateau's cliff to the
    /// cave's centre, so the cave opens onto the face where the level put it,
    /// not wherever its noise happens to reach.
    #[test]
    fn a_cave_mouth_opens_onto_the_cliff() {
        let voxel_size = 0.5;
        let mut terrain = cliffed_plateau(voxel_size);
        // Carve nothing by noise: the chamber is the mouth's own end.
        terrain.volumes.push(VolumeFeature::Caves {
            frequency: 0.2,
            octaves: 2,
            seed: 1,
            depth_curve: vec![CaveDepthPoint {
                depth: 0.0,
                threshold: 1.0,
            }],
            region: Some(CaveRegion {
                center: (0.0, 3.5, 0.0),
                radius: 2.0,
                falloff: 1.0,
            }),
            material_layers: Vec::new(),
            floor_bias: 0.0,
            mouths: vec![CaveMouth {
                from: (7.0, 3.5, 0.0),
                half_width: 1.2,
                half_height: 1.5,
            }],
        });
        let bounds = AABB::new(Point3::new(-8.0, 0.0, -8.0), Point3::new(8.0, 10.0, 8.0));
        let mut grid = ChunkGrid::new(voxel_size);
        generate_terrain(&mut grid, &terrain, &bounds);

        // Every voxel on the axis, from outside the cliff to the centre, is open.
        for i in 0..=14 {
            let p = Point3::new(7.0 - i as f32 * 0.5, 3.5, 0.0);
            assert!(
                grid.get(p).density <= 0.0,
                "solid at {p:?} on the mouth's axis"
            );
        }
        // And the rock above the passage still stands.
        assert!(
            grid.get(Point3::new(1.0, 6.0, 0.0)).density > 0.0,
            "the passage's roof fell in"
        );
    }
}
