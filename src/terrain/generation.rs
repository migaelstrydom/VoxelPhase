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
use super::csg::{carve_with_sdf, debias_height, index_range, union_solid};
use super::traversal::{excavate, rasterise, route_plan, RoutePart};
use super::voxel::{Voxel, VoxelMaterial};
use crate::collision::AABB;
use crate::level::{
    CaveDepthPoint, CaveRegion, MaterialLayer, Terrain, TerrainFeature, VolumeFeature,
    VoxelMaterialId,
};
use crate::utils::noise::{fbm_2d_periodic, fbm_3d};

/// Generate terrain into a chunk grid from a `Terrain` description.
///
/// `bounds` is the grid-local extent to generate within; features are clipped
/// to it and nothing outside is written.
pub fn generate_terrain(grid: &mut ChunkGrid, terrain: &Terrain, bounds: &AABB) {
    let step = grid.voxel_size();

    generate_heightfield(grid, terrain, bounds, step);

    // Pass 2: volumetric features — place or carve voxels in 3D. Each feature
    // derives its own iteration box, so only the chunks it overlaps are visited.
    for volume in &terrain.volumes {
        apply_volume(grid, volume, terrain, bounds, step);
    }
}

/// Pass 1: heightfield — fill columns based on feature-driven height.
///
/// Walks one chunk column at a time. Heights are evaluated once per voxel
/// column and reused for every chunk in that column, and each chunk is resolved
/// once rather than per voxel write.
fn generate_heightfield(grid: &mut ChunkGrid, terrain: &Terrain, bounds: &AABB, step: f32) {
    let floor_y = bounds.min.y;
    let n = CHUNK_VOXELS as i32;

    let (ix0, ix1) = index_range(bounds.min.x, bounds.max.x, step);
    let (iy0, iy1) = index_range(bounds.min.y, bounds.max.y, step);
    let (iz0, iz1) = index_range(bounds.min.z, bounds.max.z, step);

    let chunk_x = (div_floor(ix0, n), div_floor(ix1 - 1, n));
    let chunk_z = (div_floor(iz0, n), div_floor(iz1 - 1, n));

    // Per voxel column within one chunk column: surface height and the index of
    // the topmost solid voxel.
    let mut heights: Vec<f32> = Vec::new();
    let mut tops: Vec<i32> = Vec::new();

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
            heights.clear();
            tops.clear();
            let mut top_index = iy0;
            for ix in xs..xe {
                for iz in zs..ze {
                    // Debiased before anything is derived from it: an authored
                    // height that lands exactly on a lattice plane degenerates
                    // marching cubes, and the column's indices below must
                    // describe the height actually encoded.
                    let height =
                        debias_height(height_at(ix as f32 * step, iz as f32 * step, terrain), step);
                    // Topmost solid voxel: the last index strictly below `height`.
                    let top = (((height / step).ceil() as i32) - 1).max(iy0);
                    heights.push(height);
                    tops.push(top);
                    top_index = top_index.max(top + 1);
                }
            }

            // Only the chunks between the floor and the highest surface (plus the
            // partial-air voxel above it) hold anything.
            let cy_lo = div_floor(iy0, n);
            let cy_hi = div_floor(top_index.min(iy1 - 1), n);

            for cy in cy_lo..=cy_hi {
                let ys = (cy * n).max(iy0);
                let ye = ((cy + 1) * n).min(iy1);
                if ys >= ye {
                    continue;
                }

                let chunk = grid.chunk_or_insert(ChunkCoord::new(cx, cy, cz));
                let mut column = 0usize;
                for ix in xs..xe {
                    let x = ix as f32 * step;
                    for iz in zs..ze {
                        let z = iz as f32 * step;
                        let height = heights[column];
                        let top = tops[column];
                        column += 1;

                        let top_voxel_y = top as f32 * step;
                        // Index of the partial-air voxel capping this column,
                        // present only where the column has at least one solid
                        // voxel and the cap still lies inside the bounds.
                        let air_index = (top_voxel_y < height && top + 1 < iy1).then_some(top + 1);

                        for iy in ys..ye.min(top + 2) {
                            let y = iy as f32 * step;
                            if y < height {
                                let depth = top_voxel_y - y;
                                let material = if y <= floor_y + terrain.bedrock_thickness {
                                    VoxelMaterial::Bedrock
                                } else {
                                    material_at_depth(depth, &terrain.material_layers)
                                };
                                let mut voxel = Voxel::solid(material);
                                // Topmost voxel: encode the sub-voxel surface offset
                                // as an SDF density so marching cubes lands the
                                // triangle at y = height instead of at the midpoint
                                // between solid and air corners.
                                if y + step > height {
                                    voxel.density =
                                        ((height - y) / step).clamp(f32::MIN_POSITIVE, 1.0);
                                }
                                chunk.set_voxel(Point3::new(x, y, z), voxel);
                            } else if air_index == Some(iy) {
                                // Matching partial-air voxel directly above the
                                // surface so the MC edge interpolates to exactly
                                // y = height.
                                chunk.set_voxel(
                                    Point3::new(x, y, z),
                                    Voxel {
                                        density: ((height - y) / step).clamp(-1.0, 0.0),
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
    let mut h = terrain.base_height;
    for feature in &terrain.features {
        h += contribute(feature, x, z, h);
    }
    h
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
        } => {
            let (proj_t, perp_dist) = point_to_segment_projection(x, z, fx, fz, tx, tz);
            let half_w = width * 0.5;
            if perp_dist >= half_w || proj_t < 0.0 || proj_t > 1.0 {
                0.0
            } else {
                let ramp_h = start_height + (end_height - start_height) * proj_t;
                let edge_t = perp_dist / half_w;
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

/// Curved bridge (circular arc) between two 3D points.
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

                // Only evaluate inside the span extent; outside we leave the
                // voxel alone so the arch doesn't stamp air past its ends.
                if along.abs() > half_span + step {
                    z += step;
                    continue;
                }

                let t_norm = (along / half_span).clamp(-1.0, 1.0);
                let arc_y = mid_y + radius * (1.0 - t_norm * t_norm).max(0.0).sqrt();
                // Signed distance to the sweep tube: take the larger of the
                // horizontal and vertical offsets minus the tube half-thickness.
                // Uses box-like metric rather than euclidean for simplicity.
                let dy = (y - arc_y).abs();
                let sd = perp_h.max(dy) - half_t;

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

    // Pass B: reassign materials on cave-exposed surfaces.
    fixup_cave_materials(
        grid,
        &terrain.material_layers,
        cave_layers,
        min_x,
        min_z,
        max_x,
        max_z,
        min_y,
        max_y,
        step,
    );
}

/// After cave carving, reassign materials on newly-exposed underground surfaces
/// using cave-specific layers.
///
/// The first solid run from the top of each column is the terrain surface —
/// those voxels keep the terrain's material layers. Only after passing through
/// an underground air gap (a cave) do we switch to cave layers.
fn fixup_cave_materials(
    grid: &mut ChunkGrid,
    terrain_layers: &[MaterialLayer],
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
                    let layers = if seen_underground_air {
                        cave_layers
                    } else {
                        terrain_layers
                    };
                    let material = material_at_depth(depth_below_surface, layers);
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

fn to_voxel_material(id: VoxelMaterialId) -> VoxelMaterial {
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
fn point_to_segment_projection(px: f32, pz: f32, ax: f32, az: f32, bx: f32, bz: f32) -> (f32, f32) {
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
