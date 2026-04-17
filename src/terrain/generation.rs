//! Procedural terrain generation using composable features.

use nalgebra::Point3;

use super::svo::SparseVoxelOctree;
use super::voxel::{DurabilityConfig, Voxel, VoxelMaterial, INDESTRUCTIBLE};
use crate::collision::AABB;
use crate::level::{
    CaveDepthPoint, CaveRegion, MaterialLayer, Terrain, TerrainFeature, VolumeFeature,
    VoxelMaterialId,
};
use crate::utils::noise::{fbm_2d_periodic, fbm_3d};

/// Generate terrain into an SVO from a `Terrain` description.
pub fn generate_terrain(
    svo: &mut SparseVoxelOctree,
    terrain: &Terrain,
    durability: &DurabilityConfig,
) {
    let bounds = *svo.bounds();
    let floor_y = bounds.min.y;
    let step = svo.min_voxel_size();

    svo.fill(Voxel::air());

    // Pass 1: heightfield — fill columns based on feature-driven height.
    let mut x = bounds.min.x;
    while x < bounds.max.x {
        let mut z = bounds.min.z;
        while z < bounds.max.z {
            let height = height_at(x, z, terrain);

            let top_voxel_y =
                bounds.min.y + (((height - bounds.min.y) / step).ceil() - 1.0).max(0.0) * step;

            let mut y = bounds.min.y;
            while y < height {
                let depth = top_voxel_y - y;
                let material = material_at_depth(depth, &terrain.material_layers);
                let hp = durability.health_at(y, top_voxel_y, floor_y);
                let mut voxel = Voxel::solid(material, hp);
                // Topmost voxel: encode the sub-voxel surface offset as an SDF
                // density so marching cubes lands the triangle at y = height
                // instead of at the midpoint between solid and air corners.
                if y + step > height {
                    voxel.density = ((height - y) / step).clamp(f32::MIN_POSITIVE, 1.0);
                }
                svo.set(Point3::new(x, y, z), voxel);
                y += step;
            }

            // Matching partial-air voxel directly above the surface so the MC
            // edge interpolates to exactly y = height.
            if y > bounds.min.y && y < bounds.max.y {
                let air = Voxel {
                    density: ((height - y) / step).clamp(-1.0, 0.0),
                    material: VoxelMaterial::Air,
                    health: 0,
                };
                svo.set(Point3::new(x, y, z), air);
            }

            z += step;
        }
        x += step;
    }

    // Pass 2: volumetric features — place or carve voxels in 3D.
    for volume in &terrain.volumes {
        apply_volume(svo, volume, terrain, durability, &bounds, step);
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

/// Union a solid into the SVO using SDF-style density.
///
/// Writes a smoothly-varying density based on the signed distance `sdf`
/// (negative inside the solid, positive outside). Voxels are only updated
/// when the new density is greater than what's already there — so features
/// layer correctly and never clobber deeper geometry.
fn union_solid(
    svo: &mut SparseVoxelOctree,
    pos: Point3<f32>,
    sdf: f32,
    step: f32,
    material: VoxelMaterial,
    health: u8,
) {
    let new_density = (-sdf / step).clamp(-1.0, 1.0);
    let existing = svo.get(pos);
    if new_density <= existing.density {
        return;
    }
    let (mat, hp) = if new_density > 0.0 {
        (material, health)
    } else {
        (VoxelMaterial::Air, 0)
    };
    svo.set(
        pos,
        Voxel {
            density: new_density,
            material: mat,
            health: hp,
        },
    );
}

/// Carve a volume out of the SVO using SDF-style density.
///
/// `sdf_carve` is the signed distance to the carve surface (negative inside
/// the region being removed). Uses CSG subtraction semantics so existing
/// solids outside the carve are preserved, and voxels near the cut get a
/// smooth partial density for MC to interpolate.
fn carve_with_sdf(svo: &mut SparseVoxelOctree, pos: Point3<f32>, sdf_carve: f32, step: f32) {
    let carve_density = (sdf_carve / step).clamp(-1.0, 1.0);
    let existing = svo.get(pos);
    let new_density = existing.density.min(carve_density);
    if new_density >= existing.density {
        return;
    }
    let (mat, hp) = if new_density > 0.0 {
        (existing.material, existing.health)
    } else {
        (VoxelMaterial::Air, 0)
    };
    svo.set(
        pos,
        Voxel {
            density: new_density,
            material: mat,
            health: hp,
        },
    );
}

/// Apply a single volumetric feature to the SVO.
fn apply_volume(
    svo: &mut SparseVoxelOctree,
    volume: &VolumeFeature,
    terrain: &Terrain,
    durability: &DurabilityConfig,
    bounds: &AABB,
    step: f32,
) {
    let layers = &terrain.material_layers;
    match *volume {
        VolumeFeature::Island {
            center: (cx, cy, cz),
            half_extents: (hx, hy, hz),
            edge_noise,
        } => {
            apply_island(
                svo, cx, cy, cz, hx, hy, hz, edge_noise, layers, durability, bounds, step,
            );
        }

        VolumeFeature::Pillar {
            center: (cx, cz),
            height,
            radius,
        } => {
            apply_pillar(
                svo, cx, cz, height, radius, layers, durability, bounds, step,
            );
        }

        VolumeFeature::Tunnel {
            center: (cx, cz),
            direction: (dx, dz),
            length,
            radius,
            depth,
        } => {
            apply_tunnel(svo, cx, cz, dx, dz, length, radius, depth, bounds, step);
        }

        VolumeFeature::Arch {
            from: (fx, fy, fz),
            to: (tx, ty, tz),
            radius,
            thickness,
        } => {
            apply_arch(
                svo, fx, fy, fz, tx, ty, tz, radius, thickness, layers, durability, bounds, step,
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
                svo,
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
                svo, fx, fz, tx, tz, height, depth, thickness, dx, dz, noise, noise_seed, layers,
                durability, bounds, step,
            );
        }
    }
}

/// Floating solid mass — a rounded box with optional noisy edges.
fn apply_island(
    svo: &mut SparseVoxelOctree,
    cx: f32,
    cy: f32,
    cz: f32,
    hx: f32,
    hy: f32,
    hz: f32,
    edge_noise: f32,
    layers: &[MaterialLayer],
    durability: &DurabilityConfig,
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
                let hp = durability.health_at(y, cy + hy, bounds.min.y);
                union_solid(svo, Point3::new(x, y, z), sd, step, material, hp);

                z += step;
            }
            y += step;
        }
        x += step;
    }
}

/// Vertical column rising from the heightfield surface.
fn apply_pillar(
    svo: &mut SparseVoxelOctree,
    cx: f32,
    cz: f32,
    height: f32,
    radius: f32,
    layers: &[MaterialLayer],
    durability: &DurabilityConfig,
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
                let hp = durability.health_at(y, top_y, bounds.min.y);
                union_solid(svo, Point3::new(x, y, z), sd, step, material, hp);
                y += step;
            }

            z += step;
        }
        x += step;
    }
}

/// Horizontal bore that carves a cylindrical tunnel through existing terrain.
fn apply_tunnel(
    svo: &mut SparseVoxelOctree,
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
                carve_with_sdf(svo, Point3::new(x, y, z), sd, step);

                z += step;
            }
            y += step;
        }
        x += step;
    }
}

/// Curved bridge (circular arc) between two 3D points.
fn apply_arch(
    svo: &mut SparseVoxelOctree,
    fx: f32,
    fy: f32,
    fz: f32,
    tx: f32,
    ty: f32,
    tz: f32,
    radius: f32,
    thickness: f32,
    layers: &[MaterialLayer],
    durability: &DurabilityConfig,
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
                let hp = durability.health_at(y, arc_y + half_t, bounds.min.y);
                union_solid(svo, Point3::new(x, y, z), sd, step, material, hp);

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
    svo: &mut SparseVoxelOctree,
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
    durability: &DurabilityConfig,
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
                let hp = durability.health_at(y, top_y, bounds.min.y);
                union_solid(svo, Point3::new(x, y, z), sd, step, material, hp);
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
/// up surface materials using cave-specific layers and assigns health based
/// on material type and depth from the cave wall.
fn apply_caves(
    svo: &mut SparseVoxelOctree,
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

                if svo.get(Point3::new(x, y, z)).density <= 0.0 {
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
                carve_with_sdf(svo, Point3::new(x, y, z), sdf_carve, step);

                y += step;
            }
            z += step;
        }
        x += step;
    }

    // Pass B: reassign materials and health on cave-exposed surfaces.
    fixup_cave_materials(
        svo,
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
/// using cave-specific layers, and set health based on material type and depth
/// from the cave wall.
///
/// The first solid run from the top of each column is the terrain surface —
/// those voxels keep the terrain's material layers. Only after passing through
/// an underground air gap (a cave) do we switch to cave layers.
fn fixup_cave_materials(
    svo: &mut SparseVoxelOctree,
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
                let voxel = svo.get(Point3::new(x, y, z));
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
                    let health = if seen_underground_air {
                        material_health(material, depth_below_surface)
                    } else {
                        voxel.health
                    };
                    if voxel.material != material || voxel.health != health {
                        // Preserve the SDF density while updating material/health.
                        svo.set(
                            Point3::new(x, y, z),
                            Voxel {
                                density: voxel.density,
                                material,
                                health,
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

/// Compute voxel health from material base health and depth from the nearest
/// exposed surface. Surface voxels get the material's base health; deeper
/// voxels get progressively more, capped at 254.
fn material_health(material: VoxelMaterial, depth_from_surface: f32) -> u8 {
    let base = material.base_health();
    if base == 0 {
        return 0;
    }
    let depth_bonus = (depth_from_surface * 0.5) as u8;
    base.saturating_add(depth_bonus).min(INDESTRUCTIBLE - 1)
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
