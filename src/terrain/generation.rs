//! Procedural terrain generation using noise.

use nalgebra::Point3;

use super::svo::SparseVoxelOctree;
use super::voxel::{Voxel, VoxelMaterial};
use crate::collision::AABB;

/// Terrain generator using procedural noise.
pub struct TerrainGenerator {}

impl TerrainGenerator {
    pub fn new(_seed: u32) -> Self {
        Self {}
    }

    /// Sample a voxel at a given position.
    #[allow(unused)] // Visible for testing
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
