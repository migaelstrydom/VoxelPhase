//! Sparse Voxel Octree for efficient terrain storage.
//!
//! The SVO stores voxel data hierarchically, allowing:
//! - Efficient memory usage (empty regions use minimal storage)
//! - Fast spatial queries (octree traversal)
//! - Easy modification (update nodes, propagate changes)
//!
//! Mesh regions are used for incremental terrain rebuilding. Each region
//! at `mesh_depth` level owns a mesh for its subtree, enabling partial
//! updates when terrain is modified.

use std::collections::{HashMap, HashSet};

use nalgebra::{Point3, Vector3};

use super::marching_cubes::MarchingCubes;
use super::voxel::Voxel;
use crate::collision::AABB;
use crate::rendering::vertex::Vertex;

/// Key identifying a mesh region (octant coordinates at mesh_depth).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MeshRegionKey {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

/// Mesh data for a single region, ready for rendering.
pub struct RegionMesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

/// A node in the Sparse Voxel Octree.
#[derive(Debug, Clone)]
pub enum SvoNode {
    /// Leaf node: entire region is uniform (same voxel value).
    Leaf(Voxel),
    /// Interior node: subdivided into 8 children.
    Interior {
        /// Child nodes, indexed by octant (see `octant_index`).
        children: Box<[SvoNode; 8]>,
    },
}

impl SvoNode {
    /// Create a uniform leaf node.
    #[allow(unused)]
    pub fn uniform(voxel: Voxel) -> Self {
        SvoNode::Leaf(voxel)
    }

    /// Create an air (empty) leaf node.
    pub fn empty() -> Self {
        SvoNode::Leaf(Voxel::air())
    }

    /// Check if this node is a leaf.
    #[allow(unused)]
    pub fn is_leaf(&self) -> bool {
        matches!(self, SvoNode::Leaf(_))
    }

    /// Try to collapse this node if all children are identical leaves.
    /// Returns true if the node was collapsed.
    pub fn try_collapse(&mut self) -> bool {
        if let SvoNode::Interior { children } = self {
            // Check if all children are identical leaves
            if let SvoNode::Leaf(first) = &children[0] {
                let all_same = children[1..].iter().all(|c| {
                    if let SvoNode::Leaf(v) = c {
                        v == first
                    } else {
                        false
                    }
                });

                if all_same {
                    *self = SvoNode::Leaf(*first);
                    return true;
                }
            }
        }
        false
    }
}

impl Default for SvoNode {
    fn default() -> Self {
        SvoNode::empty()
    }
}

/// Sparse Voxel Octree for terrain data.
pub struct SparseVoxelOctree {
    /// Root node of the octree.
    root: SvoNode,
    /// World-space bounds of the entire octree.
    bounds: AABB,
    /// Maximum depth of the tree (determines minimum voxel size).
    max_depth: u32,

    // Mesh region storage for incremental rebuilding
    /// Depth at which mesh regions are defined (configurable).
    mesh_depth: u32,
    /// Sparse storage of region meshes, keyed by region coordinates.
    mesh_regions: HashMap<MeshRegionKey, RegionMesh>,
    /// Regions that need mesh rebuilding.
    dirty_regions: HashSet<MeshRegionKey>,
}

/// Default mesh depth if not specified.
const DEFAULT_MESH_DEPTH: u32 = 3;

impl SparseVoxelOctree {
    /// Create a new SVO with the given world bounds and maximum depth.
    ///
    /// The minimum voxel size will be `bounds.size() / 2^max_depth`.
    /// Uses default mesh depth of 3.
    pub fn new(bounds: AABB, max_depth: u32) -> Self {
        Self::with_mesh_depth(bounds, max_depth, DEFAULT_MESH_DEPTH)
    }

    /// Create a new SVO with configurable mesh depth.
    ///
    /// `mesh_depth` determines the granularity of incremental mesh updates.
    /// Lower values = larger regions = fewer meshes but more work per update.
    /// Higher values = smaller regions = more meshes but less work per update.
    /// Must be <= max_depth.
    pub fn with_mesh_depth(bounds: AABB, max_depth: u32, mesh_depth: u32) -> Self {
        let mesh_depth = mesh_depth.min(max_depth);
        Self {
            root: SvoNode::empty(),
            bounds,
            max_depth,
            mesh_depth,
            mesh_regions: HashMap::new(),
            dirty_regions: HashSet::new(),
        }
    }

    /// Get the world bounds of this SVO.
    pub fn bounds(&self) -> &AABB {
        &self.bounds
    }

    /// Get the maximum depth.
    pub fn max_depth(&self) -> u32 {
        self.max_depth
    }

    /// Get the minimum voxel size (at max depth).
    pub fn min_voxel_size(&self) -> f32 {
        let size = self.bounds.size();
        let divisions = (1 << self.max_depth) as f32;
        size.x.min(size.y).min(size.z) / divisions
    }

    /// Get the voxel at a world position.
    pub fn get(&self, position: Point3<f32>) -> Voxel {
        if !self.bounds.contains_point(position) {
            return Voxel::air();
        }

        self.get_recursive(&self.root, &self.bounds, position, 0)
    }

    fn get_recursive(
        &self,
        node: &SvoNode,
        node_bounds: &AABB,
        position: Point3<f32>,
        depth: u32,
    ) -> Voxel {
        match node {
            SvoNode::Leaf(voxel) => *voxel,
            SvoNode::Interior { children } => {
                if depth >= self.max_depth {
                    // Shouldn't happen, but return air if we go too deep
                    return Voxel::air();
                }

                let octant = octant_index(node_bounds, position);
                let child_bounds = child_bounds(node_bounds, octant);
                self.get_recursive(&children[octant], &child_bounds, position, depth + 1)
            }
        }
    }

    /// Set the voxel at a world position.
    pub fn set(&mut self, position: Point3<f32>, voxel: Voxel) {
        if !self.bounds.contains_point(position) {
            return;
        }

        let bounds = self.bounds;
        Self::set_recursive(&mut self.root, &bounds, position, voxel, 0, self.max_depth);
    }

    fn set_recursive(
        node: &mut SvoNode,
        node_bounds: &AABB,
        position: Point3<f32>,
        voxel: Voxel,
        depth: u32,
        max_depth: u32,
    ) {
        if depth >= max_depth {
            // At maximum depth, set the leaf value
            *node = SvoNode::Leaf(voxel);
            return;
        }

        // Ensure we have an interior node to descend into
        if let SvoNode::Leaf(leaf_voxel) = node {
            // Subdivide: create 8 children with the current leaf value
            let leaf = *leaf_voxel;
            *node = SvoNode::Interior {
                children: Box::new([
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                ]),
            };
        }

        if let SvoNode::Interior { children } = node {
            let octant = octant_index(node_bounds, position);
            let child_bounds = child_bounds(node_bounds, octant);
            Self::set_recursive(
                &mut children[octant],
                &child_bounds,
                position,
                voxel,
                depth + 1,
                max_depth,
            );

            // Try to collapse if all children are now identical
            node.try_collapse();
        }
    }

    /// Modify voxels within a sphere (for explosions, digging).
    /// Calls the modifier function for each voxel in the sphere.
    pub fn modify_sphere<F>(&mut self, center: Point3<f32>, radius: f32, mut modifier: F)
    where
        F: FnMut(Point3<f32>, Voxel) -> Voxel,
    {
        let sphere_bounds =
            AABB::from_center_half_extents(center, nalgebra::Vector3::new(radius, radius, radius));

        if !self.bounds.intersects(&sphere_bounds) {
            return;
        }

        let bounds = self.bounds;
        let max_depth = self.max_depth;
        Self::modify_sphere_recursive(
            &mut self.root,
            &bounds,
            center,
            radius,
            &mut modifier,
            0,
            max_depth,
        );
    }

    fn modify_sphere_recursive<F>(
        node: &mut SvoNode,
        node_bounds: &AABB,
        center: Point3<f32>,
        radius: f32,
        modifier: &mut F,
        depth: u32,
        max_depth: u32,
    ) where
        F: FnMut(Point3<f32>, Voxel) -> Voxel,
    {
        // Check if this node's bounds intersect the sphere
        if !node_bounds.intersects_sphere(center, radius) {
            return;
        }

        if depth >= max_depth {
            // At max depth, modify this voxel
            if let SvoNode::Leaf(voxel) = node {
                let node_center = node_bounds.center();
                *voxel = modifier(node_center, *voxel);
            }
            return;
        }

        // Check if sphere completely contains this node
        let node_center = node_bounds.center();
        let node_radius = node_bounds.half_extents().magnitude();
        let dist_to_center = (node_center - center).magnitude();

        if dist_to_center + node_radius <= radius {
            // Node is completely inside sphere, modify as leaf
            if let SvoNode::Leaf(voxel) = node {
                *voxel = modifier(node_center, *voxel);
            } else {
                // Convert to leaf with modified value
                let voxel = modifier(node_center, Voxel::air());
                *node = SvoNode::Leaf(voxel);
            }
            return;
        }

        // Need to subdivide and recurse
        if let SvoNode::Leaf(leaf_voxel) = node {
            let leaf = *leaf_voxel;
            *node = SvoNode::Interior {
                children: Box::new([
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                    SvoNode::Leaf(leaf),
                ]),
            };
        }

        if let SvoNode::Interior { children } = node {
            for (i, child) in children.iter_mut().enumerate() {
                let child_bounds = child_bounds(node_bounds, i);
                Self::modify_sphere_recursive(
                    child,
                    &child_bounds,
                    center,
                    radius,
                    modifier,
                    depth + 1,
                    max_depth,
                );
            }
            node.try_collapse();
        }
    }

    /// Fill the entire SVO with a uniform voxel.
    pub fn fill(&mut self, voxel: Voxel) {
        self.root = SvoNode::Leaf(voxel);
    }

    /// Sample voxels in a regular grid within the given bounds.
    /// Returns a 3D array of voxels for mesh generation.
    pub fn sample_grid(&self, bounds: &AABB, resolution: usize) -> Vec<Vec<Vec<Voxel>>> {
        let mut grid = vec![vec![vec![Voxel::air(); resolution]; resolution]; resolution];

        let size = bounds.size();
        let step = nalgebra::Vector3::new(
            size.x / (resolution - 1) as f32,
            size.y / (resolution - 1) as f32,
            size.z / (resolution - 1) as f32,
        );

        for x in 0..resolution {
            for y in 0..resolution {
                for z in 0..resolution {
                    let pos = Point3::new(
                        bounds.min.x + x as f32 * step.x,
                        bounds.min.y + y as f32 * step.y,
                        bounds.min.z + z as f32 * step.z,
                    );
                    grid[x][y][z] = self.get(pos);
                }
            }
        }

        grid
    }

    // === Mesh Region Methods ===

    /// Get the size of each mesh region in world units.
    pub fn mesh_region_size(&self) -> f32 {
        self.bounds.size().x / (1 << self.mesh_depth) as f32
    }

    /// Get the voxel size at maximum depth.
    pub fn voxel_size(&self) -> f32 {
        self.bounds.size().x / (1 << self.max_depth) as f32
    }

    /// Convert a world position to a mesh region key.
    pub fn world_to_region(&self, pos: Point3<f32>) -> MeshRegionKey {
        let region_size = self.mesh_region_size();
        let relative = pos - self.bounds.min;
        let regions_per_axis = 1u32 << self.mesh_depth;
        MeshRegionKey {
            x: ((relative.x / region_size).floor() as u32).min(regions_per_axis - 1),
            y: ((relative.y / region_size).floor() as u32).min(regions_per_axis - 1),
            z: ((relative.z / region_size).floor() as u32).min(regions_per_axis - 1),
        }
    }

    /// Get the world-space bounds for a mesh region.
    pub fn region_bounds(&self, key: MeshRegionKey) -> AABB {
        let region_size = self.mesh_region_size();
        let min = self.bounds.min
            + Vector3::new(
                key.x as f32 * region_size,
                key.y as f32 * region_size,
                key.z as f32 * region_size,
            );
        let max = min + Vector3::new(region_size, region_size, region_size);
        AABB::new(min, max)
    }

    /// Mark mesh regions overlapping the given AABB as dirty.
    ///
    /// Expands the affected area by one voxel in each direction to account for
    /// marching cubes cells that span region boundaries. A cell at position (x,y,z)
    /// depends on voxels at corners (x,y,z) through (x+1,y+1,z+1), so modifications
    /// near boundaries can affect triangles in neighboring regions.
    pub fn mark_regions_dirty(&mut self, affected: &AABB) {
        // Expand by one voxel to catch boundary-spanning marching cubes cells
        let voxel_size = self.voxel_size();
        let expanded = AABB::new(
            affected.min - Vector3::new(voxel_size, voxel_size, voxel_size),
            affected.max + Vector3::new(voxel_size, voxel_size, voxel_size),
        );

        let min_region = self.world_to_region(expanded.min);
        let max_region = self.world_to_region(expanded.max);

        for x in min_region.x..=max_region.x {
            for y in min_region.y..=max_region.y {
                for z in min_region.z..=max_region.z {
                    self.dirty_regions.insert(MeshRegionKey { x, y, z });
                }
            }
        }
    }

    /// Mark all regions as dirty (for initial build or full rebuild).
    pub fn mark_all_regions_dirty(&mut self) {
        let regions_per_axis = 1u32 << self.mesh_depth;
        for x in 0..regions_per_axis {
            for y in 0..regions_per_axis {
                for z in 0..regions_per_axis {
                    self.dirty_regions.insert(MeshRegionKey { x, y, z });
                }
            }
        }
    }

    /// Check if any regions need rebuilding.
    pub fn has_dirty_regions(&self) -> bool {
        !self.dirty_regions.is_empty()
    }

    /// Get the number of dirty regions.
    pub fn dirty_region_count(&self) -> usize {
        self.dirty_regions.len()
    }

    /// Rebuild mesh for a single region.
    pub fn rebuild_region(&mut self, key: MeshRegionKey) {
        let region_bounds = self.region_bounds(key);
        let voxel_size = self.voxel_size();

        // Sample with 1-voxel padding for marching cubes neighbor lookups
        let padded_bounds = AABB::new(
            region_bounds.min - Vector3::new(voxel_size, voxel_size, voxel_size),
            region_bounds.max + Vector3::new(voxel_size, voxel_size, voxel_size),
        );

        // Calculate sample resolution: cells per region + 2 for padding on each side
        let cells_per_region = 1usize << (self.max_depth - self.mesh_depth);
        let sample_resolution = cells_per_region + 3; // +1 for grid points, +2 for padding

        let grid = self.sample_grid(&padded_bounds, sample_resolution);

        // Run marching cubes
        let marching_cubes = MarchingCubes::new();
        let mesh = marching_cubes.generate(&grid, padded_bounds.min, voxel_size);

        // Filter triangles to only those with centroids inside region_bounds
        let filtered = self.filter_mesh_to_bounds(&mesh, &region_bounds);

        if filtered.vertices.is_empty() {
            self.mesh_regions.remove(&key);
        } else {
            self.mesh_regions.insert(key, filtered);
        }
    }

    /// Filter mesh triangles to those owned by this region.
    ///
    /// Uses a deterministic "minimum vertex" strategy: each triangle is assigned
    /// to the region containing its lexicographically smallest vertex (by x, y, z).
    /// This ensures consistent assignment regardless of which region generates the
    /// triangle, avoiding gaps at region boundaries.
    fn filter_mesh_to_bounds(
        &self,
        mesh: &super::marching_cubes::MarchingCubesMesh,
        bounds: &AABB,
    ) -> RegionMesh {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        // Process triangles (every 3 indices)
        for tri_indices in mesh.indices.chunks(3) {
            if tri_indices.len() < 3 {
                continue;
            }

            let i0 = tri_indices[0] as usize;
            let i1 = tri_indices[1] as usize;
            let i2 = tri_indices[2] as usize;

            let p0 = mesh.positions[i0];
            let p1 = mesh.positions[i1];
            let p2 = mesh.positions[i2];

            // Find the lexicographically minimum vertex (deterministic ownership)
            let min_vertex = [p0, p1, p2]
                .into_iter()
                .min_by(|a, b| {
                    a.x.partial_cmp(&b.x)
                        .unwrap()
                        .then(a.y.partial_cmp(&b.y).unwrap())
                        .then(a.z.partial_cmp(&b.z).unwrap())
                })
                .unwrap();

            // Only include if the minimum vertex is inside this region's bounds
            if bounds.contains_point(min_vertex) {
                let base = vertices.len() as u32;

                // Add vertices for this triangle
                for &idx in &[i0, i1, i2] {
                    vertices.push(Vertex {
                        pos: nalgebra::Vector4::new(
                            mesh.positions[idx].x,
                            mesh.positions[idx].y,
                            mesh.positions[idx].z,
                            1.0,
                        ),
                        color: nalgebra::Vector4::new(
                            mesh.colors[idx][0],
                            mesh.colors[idx][1],
                            mesh.colors[idx][2],
                            mesh.colors[idx][3],
                        ),
                        tex_coords: nalgebra::Vector2::new(
                            mesh.positions[idx].x * 0.1,
                            mesh.positions[idx].z * 0.1,
                        ),
                        normal: mesh.normals[idx],
                    });
                }

                indices.push(base);
                indices.push(base + 1);
                indices.push(base + 2);
            }
        }

        RegionMesh { vertices, indices }
    }

    /// Rebuild all dirty regions and return the keys that were rebuilt.
    pub fn rebuild_dirty_regions(&mut self) -> Vec<MeshRegionKey> {
        let dirty: Vec<_> = self.dirty_regions.drain().collect();
        for key in &dirty {
            self.rebuild_region(*key);
        }
        dirty
    }

    /// Get read-only access to the mesh regions.
    pub fn mesh_regions(&self) -> &HashMap<MeshRegionKey, RegionMesh> {
        &self.mesh_regions
    }
}

/// Compute which octant a point falls into within a bounding box.
/// Returns an index 0-7 based on which side of each axis midpoint the point is.
///
/// Octant indexing:
/// - bit 0: X (0 = min side, 1 = max side)
/// - bit 1: Y (0 = min side, 1 = max side)
/// - bit 2: Z (0 = min side, 1 = max side)
fn octant_index(bounds: &AABB, point: Point3<f32>) -> usize {
    let center = bounds.center();
    let mut index = 0;
    if point.x >= center.x {
        index |= 1;
    }
    if point.y >= center.y {
        index |= 2;
    }
    if point.z >= center.z {
        index |= 4;
    }
    index
}

/// Compute the bounds of a child octant.
fn child_bounds(parent: &AABB, octant: usize) -> AABB {
    let center = parent.center();
    let min = Point3::new(
        if octant & 1 == 0 {
            parent.min.x
        } else {
            center.x
        },
        if octant & 2 == 0 {
            parent.min.y
        } else {
            center.y
        },
        if octant & 4 == 0 {
            parent.min.z
        } else {
            center.z
        },
    );
    let max = Point3::new(
        if octant & 1 == 0 {
            center.x
        } else {
            parent.max.x
        },
        if octant & 2 == 0 {
            center.y
        } else {
            parent.max.y
        },
        if octant & 4 == 0 {
            center.z
        } else {
            parent.max.z
        },
    );
    AABB::new(min, max)
}

#[cfg(test)]
mod tests {
    use super::super::voxel::VoxelMaterial;
    use super::*;

    #[test]
    fn test_octant_index() {
        let bounds = AABB::new(Point3::origin(), Point3::new(2.0, 2.0, 2.0));

        assert_eq!(octant_index(&bounds, Point3::new(0.5, 0.5, 0.5)), 0);
        assert_eq!(octant_index(&bounds, Point3::new(1.5, 0.5, 0.5)), 1);
        assert_eq!(octant_index(&bounds, Point3::new(0.5, 1.5, 0.5)), 2);
        assert_eq!(octant_index(&bounds, Point3::new(1.5, 1.5, 0.5)), 3);
        assert_eq!(octant_index(&bounds, Point3::new(0.5, 0.5, 1.5)), 4);
        assert_eq!(octant_index(&bounds, Point3::new(1.5, 1.5, 1.5)), 7);
    }

    #[test]
    fn test_svo_get_set() {
        let bounds = AABB::new(Point3::origin(), Point3::new(16.0, 16.0, 16.0));
        let mut svo = SparseVoxelOctree::new(bounds, 4);

        // Initially all air
        assert!(!svo.get(Point3::new(8.0, 8.0, 8.0)).is_solid());

        // Set a voxel
        svo.set(
            Point3::new(8.0, 8.0, 8.0),
            Voxel::solid(VoxelMaterial::Rock),
        );
        assert!(svo.get(Point3::new(8.0, 8.0, 8.0)).is_solid());

        // Nearby should still be air (at max depth)
        // Note: due to octree granularity, this depends on voxel size
    }

    #[test]
    fn test_svo_modify_sphere() {
        let bounds = AABB::new(Point3::origin(), Point3::new(16.0, 16.0, 16.0));
        let mut svo = SparseVoxelOctree::new(bounds, 4);

        // Fill with rock
        svo.fill(Voxel::solid(VoxelMaterial::Rock));

        // Carve out a sphere of air
        svo.modify_sphere(Point3::new(8.0, 8.0, 8.0), 4.0, |pos, _| {
            let dist = (pos - Point3::new(8.0, 8.0, 8.0)).magnitude();
            if dist < 4.0 {
                Voxel::air()
            } else {
                Voxel::solid(VoxelMaterial::Rock)
            }
        });

        // Center should be air now
        assert!(!svo.get(Point3::new(8.0, 8.0, 8.0)).is_solid());
        // Edge should still be rock
        assert!(svo.get(Point3::new(1.0, 1.0, 1.0)).is_solid());
    }
}
