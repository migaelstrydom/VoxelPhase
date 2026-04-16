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
                svo.set(Point3::new(x, y, z), Voxel::solid(material, hp));
                y += step;
            }

            z += step;
        }
        x += step;
    }

    // Pass 2: volumetric features — place or carve voxels in 3D.
    for volume in &terrain.volumes {
        apply_volume(
            svo,
            volume,
            terrain,
            durability,
            &bounds,
            step,
        );
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
    }
}

// ---------------------------------------------------------------------------
// Pass 2: volumetric features
// ---------------------------------------------------------------------------

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

                if sd < 0.0 {
                    // Depth from island surface for material selection
                    let depth = (-sd).max(0.0);
                    let material = material_at_depth(depth, layers);
                    let hp = durability.health_at(y, cy + hy, bounds.min.y);
                    svo.set(Point3::new(x, y, z), Voxel::solid(material, hp));
                }

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
    let min_x = (cx - radius).max(bounds.min.x);
    let max_x = (cx + radius).min(bounds.max.x);
    let min_z = (cz - radius).max(bounds.min.z);
    let max_z = (cz + radius).min(bounds.max.z);
    let top_y = height.min(bounds.max.y);

    let mut x = min_x;
    while x < max_x {
        let mut z = min_z;
        while z < max_z {
            let dx = x - cx;
            let dz = z - cz;
            let dist_sq = dx * dx + dz * dz;
            if dist_sq >= radius * radius {
                z += step;
                continue;
            }

            // Fill from the bottom of the world up to the specified height
            let mut y = bounds.min.y;
            while y < top_y {
                let depth = top_y - y;
                let material = material_at_depth(depth, layers);
                let hp = durability.health_at(y, top_y, bounds.min.y);
                svo.set(Point3::new(x, y, z), Voxel::solid(material, hp));
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

    // Bounding box of the tunnel
    let min_x = ax.min(bx) - radius;
    let max_x = ax.max(bx) + radius;
    let min_z = az.min(bz) - radius;
    let max_z = az.max(bz) + radius;
    let min_y = depth - radius;
    let max_y = depth + radius;

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
                // Project (x, z) onto the tunnel axis segment
                let (t, perp_xz) = point_to_segment_projection(x, z, ax, az, bx, bz);
                if t < 0.0 || t > 1.0 {
                    z += step;
                    continue;
                }
                // Distance in the YZ cross-section (circular bore)
                let dy = y - depth;
                let dist = (perp_xz * perp_xz + dy * dy).sqrt();
                if dist < radius {
                    svo.set(Point3::new(x, y, z), Voxel::air());
                }

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
                let perp_h = (rel_x - along * dir_x).powi(2) + (rel_z - along * dir_z).powi(2);
                let perp_h = perp_h.sqrt();

                if along.abs() <= half_span && perp_h <= half_t {
                    // Arc height at this position along the span
                    let t = along / half_span; // [-1, 1]
                    let arc_y = mid_y + radius * (1.0 - t * t).max(0.0).sqrt();
                    let dy = (y - arc_y).abs();
                    if dy <= half_t {
                        let material = material_at_depth(0.5, layers);
                        let hp = durability.health_at(y, arc_y + half_t, bounds.min.y);
                        svo.set(Point3::new(x, y, z), Voxel::solid(material, hp));
                    }
                }

                z += step;
            }
            y += step;
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
        (bounds.min.x, bounds.min.y, bounds.min.z, bounds.max.x, bounds.max.y, bounds.max.z)
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

                if noise > threshold {
                    svo.set(Point3::new(x, y, z), Voxel::air());
                }

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
                        svo.set(Point3::new(x, y, z), Voxel::solid(material, health));
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
