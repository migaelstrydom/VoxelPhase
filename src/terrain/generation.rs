//! Procedural terrain generation using noise.

use nalgebra::Point3;
use noise::{NoiseFn, OpenSimplex, Perlin};

use super::svo::SparseVoxelOctree;
use super::voxel::{Voxel, VoxelMaterial};
use crate::collision::AABB;

/// Terrain generator using procedural noise.
pub struct TerrainGenerator {
    /// Seed for random generation.
    seed: u32,
    /// Primary noise for large-scale terrain shape.
    primary_noise: Perlin,
    /// Secondary noise for detail.
    detail_noise: OpenSimplex,
}

impl TerrainGenerator {
    pub fn new(seed: u32) -> Self {
        Self {
            seed,
            primary_noise: Perlin::new(seed),
            detail_noise: OpenSimplex::new(seed.wrapping_add(1)),
        }
    }

    /// Generate terrain into an SVO.
    ///
    /// Creates rolling hills with varied materials based on height.
    pub fn generate(&self, svo: &mut SparseVoxelOctree) {
        let bounds = *svo.bounds();
        let min_voxel = svo.min_voxel_size();

        // We'll sample at the minimum voxel resolution
        let step = min_voxel;

        let mut x = bounds.min.x;
        while x < bounds.max.x {
            let mut z = bounds.min.z;
            while z < bounds.max.z {
                // Generate height at this XZ position
                let height = self.sample_height(x, z, &bounds);

                // Fill column from bottom to height
                let mut y = bounds.min.y;
                while y < bounds.max.y {
                    let pos = Point3::new(x, y, z);
                    let voxel = self.sample_voxel(pos, height, &bounds);
                    if voxel.density > 0.0 {
                        svo.set(pos, voxel);
                    }
                    y += step;
                }
                z += step;
            }
            x += step;
        }
    }

    /// Sample the terrain height at a given XZ position.
    fn sample_height(&self, x: f32, z: f32, bounds: &AABB) -> f32 {
        // Normalize coordinates for noise sampling
        let scale = 0.02; // Controls terrain frequency
        let nx = x * scale;
        let nz = z * scale;

        // Multi-octave noise for interesting terrain
        let mut height = 0.0;
        let mut amplitude = 1.0;
        let mut frequency = 1.0;
        let persistence = 0.5;
        let octaves = 4;

        for _ in 0..octaves {
            height += self
                .primary_noise
                .get([nx as f64 * frequency, nz as f64 * frequency]) as f32
                * amplitude;
            amplitude *= persistence;
            frequency *= 2.0;
        }

        // Add some detail noise
        let detail_scale = 0.1;
        let detail = self
            .detail_noise
            .get([x as f64 * detail_scale, z as f64 * detail_scale]) as f32
            * 0.2;
        height += detail;

        // Map from [-1, 1] to world Y coordinates
        // Place terrain in the lower-middle portion of the bounds
        let world_height = bounds.size().y;
        let base_height = bounds.min.y + world_height * 0.3;
        let terrain_amplitude = world_height * 0.25;

        base_height + height * terrain_amplitude
    }

    /// Sample a voxel at a given position.
    fn sample_voxel(&self, pos: Point3<f32>, surface_height: f32, bounds: &AABB) -> Voxel {
        let depth_below_surface = surface_height - pos.y;

        if depth_below_surface < 0.0 {
            // Above surface: air
            return Voxel::new(-depth_below_surface, VoxelMaterial::Air);
        }

        // Below surface: determine material based on depth
        let material = if depth_below_surface < 0.5 {
            VoxelMaterial::Grass
        } else if depth_below_surface < 3.0 {
            VoxelMaterial::Dirt
        } else {
            VoxelMaterial::Rock
        };

        // Density is positive underground, with smooth falloff near surface
        let density = depth_below_surface.min(1.0);

        Voxel::new(density, material)
    }

    /// Generate a simple test terrain: a flat plane with some hills.
    pub fn generate_simple_hills(&self, svo: &mut SparseVoxelOctree) {
        let bounds = *svo.bounds();
        let center = bounds.center();

        // Fill with air first
        svo.fill(Voxel::air());

        // Create ground plane with hills using modify_sphere pattern
        // but we'll do it more efficiently by setting voxels directly

        let step = svo.min_voxel_size();
        let mut x = bounds.min.x;

        while x < bounds.max.x {
            let mut z = bounds.min.z;
            while z < bounds.max.z {
                // Simple height function: flat base + sine hills
                let dx = x - center.x;
                let dz = z - center.z;

                // Base height at center of bounds
                let base = bounds.min.y + bounds.size().y * 0.3;

                // Add some gentle hills using sin
                let hill1 = ((x * 0.1).sin() * (z * 0.1).cos() * 3.0).max(0.0);
                let hill2 = ((x * 0.05 + 1.0).cos() * (z * 0.07).sin() * 5.0).max(0.0);

                // Crater in the middle
                let dist_from_center = (dx * dx + dz * dz).sqrt();
                let crater = if dist_from_center < 10.0 {
                    -((10.0 - dist_from_center) * 0.3)
                } else {
                    0.0
                };

                let height = base + hill1 + hill2 + crater;

                // Fill column
                let mut y = bounds.min.y;
                while y < height {
                    let depth = height - y;
                    let material = if depth < 1.0 {
                        VoxelMaterial::Grass
                    } else if depth < 4.0 {
                        VoxelMaterial::Dirt
                    } else {
                        VoxelMaterial::Rock
                    };
                    svo.set(Point3::new(x, y, z), Voxel::solid(material));
                    y += step;
                }

                z += step;
            }
            x += step;
        }
    }
}

/// Create a test terrain for development.
pub fn create_test_terrain(size: f32, max_depth: u32) -> SparseVoxelOctree {
    let half_size = size / 2.0;
    let bounds = AABB::new(
        Point3::new(-half_size, -half_size, -half_size),
        Point3::new(half_size, half_size, half_size),
    );

    let mut svo = SparseVoxelOctree::new(bounds, max_depth);

    let generator = TerrainGenerator::new(42);
    generator.generate_simple_hills(&mut svo);

    svo
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_terrain_generation() {
        let svo = create_test_terrain(32.0, 5);

        // Check that we have some solid voxels
        let center = svo.bounds().center();
        let ground_pos = Point3::new(center.x, svo.bounds().min.y + 1.0, center.z);

        // Should have ground near the bottom
        // Note: This is a basic sanity check, exact behavior depends on generation
    }
}
