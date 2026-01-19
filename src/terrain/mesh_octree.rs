//! Adaptive Mesh Octree for efficient terrain rendering and collision queries.
//!
//! This structure stores triangle meshes in an octree where leaf nodes adapt their
//! size based on triangle count. Key features:
//!
//! - **Adaptive subdivision**: Leaves split when triangle count exceeds threshold
//! - **Complete spatial queries**: Each leaf stores references to ALL triangles
//!   that intersect its bounds, not just those it "owns"
//! - **Efficient updates**: Only affected regions are rebuilt when terrain changes
//!
//! Triangle ownership (for canonical storage) uses the "minimum vertex" rule:
//! each triangle is owned by the leaf containing its lexicographically smallest
//! vertex. This ensures no duplicates while the intersection references ensure
//! complete query results.

use std::collections::HashSet;

use nalgebra::{Point3, Vector3};

use super::marching_cubes::MarchingCubes;
use super::voxel::Voxel;
use crate::collision::{Triangle, AABB};
use crate::rendering::vertex::Vertex;

/// Maximum triangles per leaf before splitting.
pub const MAX_TRIANGLES_PER_LEAF: usize = 100;

/// Maximum depth of the mesh octree (prevents infinite subdivision).
const MAX_MESH_DEPTH: u32 = 10;

/// Reference to a triangle owned by another leaf.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TriangleRef {
    /// Path from root to the owning leaf, encoded as 3 bits per level.
    /// Bits 0-2 = level 0 octant, bits 3-5 = level 1 octant, etc.
    path: u64,
    /// Depth of the owning leaf (number of valid octant indices in path).
    depth: u8,
    /// Index of the triangle within the owning leaf's vertex/index arrays.
    triangle_index: u32,
}

impl TriangleRef {
    fn new(path: u64, depth: u8, triangle_index: u32) -> Self {
        Self {
            path,
            depth,
            triangle_index,
        }
    }

    /// Get the octant index at a given level of the path.
    fn octant_at_level(&self, level: u8) -> usize {
        ((self.path >> (level * 3)) & 0b111) as usize
    }
}

/// Data stored in a mesh leaf node.
#[derive(Debug, Clone)]
pub struct MeshLeaf {
    /// Vertices for triangles owned by this leaf (canonical storage).
    pub vertices: Vec<Vertex>,
    /// Indices into vertices array (groups of 3 for triangles).
    pub indices: Vec<u32>,
    /// References to triangles from neighboring leaves that intersect this leaf.
    pub neighbor_refs: Vec<TriangleRef>,
}

impl MeshLeaf {
    fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            neighbor_refs: Vec::new(),
        }
    }

    /// Number of triangles owned by this leaf.
    pub fn owned_triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// Get the vertices for a specific owned triangle.
    pub fn get_triangle_vertices(&self, triangle_index: usize) -> [&Vertex; 3] {
        let base = triangle_index * 3;
        let i0 = self.indices[base] as usize;
        let i1 = self.indices[base + 1] as usize;
        let i2 = self.indices[base + 2] as usize;
        [&self.vertices[i0], &self.vertices[i1], &self.vertices[i2]]
    }

    /// Convert an owned triangle to a collision Triangle.
    pub fn to_collision_triangle(&self, triangle_index: usize) -> Triangle {
        let [v0, v1, v2] = self.get_triangle_vertices(triangle_index);
        Triangle::new(
            Point3::new(v0.pos.x, v0.pos.y, v0.pos.z),
            Point3::new(v1.pos.x, v1.pos.y, v1.pos.z),
            Point3::new(v2.pos.x, v2.pos.y, v2.pos.z),
        )
    }

    /// Compute AABB for a specific owned triangle.
    fn triangle_aabb(&self, triangle_index: usize) -> AABB {
        let [v0, v1, v2] = self.get_triangle_vertices(triangle_index);
        let min = Point3::new(
            v0.pos.x.min(v1.pos.x).min(v2.pos.x),
            v0.pos.y.min(v1.pos.y).min(v2.pos.y),
            v0.pos.z.min(v1.pos.z).min(v2.pos.z),
        );
        let max = Point3::new(
            v0.pos.x.max(v1.pos.x).max(v2.pos.x),
            v0.pos.y.max(v1.pos.y).max(v2.pos.y),
            v0.pos.z.max(v1.pos.z).max(v2.pos.z),
        );
        AABB::new(min, max)
    }
}

/// Content of a mesh octree node.
#[derive(Debug, Clone)]
pub enum MeshNodeContent {
    /// Empty node (no geometry in this region).
    Empty,
    /// Leaf node with triangle data.
    Leaf(MeshLeaf),
    /// Interior node with 8 children.
    Interior(Box<[MeshNode; 8]>),
}

/// A node in the mesh octree.
#[derive(Debug, Clone)]
pub struct MeshNode {
    /// World-space bounds of this node.
    pub bounds: AABB,
    /// Node content (empty, leaf, or interior).
    pub content: MeshNodeContent,
}

impl MeshNode {
    fn empty(bounds: AABB) -> Self {
        Self {
            bounds,
            content: MeshNodeContent::Empty,
        }
    }
}

/// Adaptive mesh octree for terrain rendering and collision.
pub struct MeshOctree {
    /// Root node of the octree.
    root: MeshNode,
    /// World-space bounds of the entire octree.
    bounds: AABB,
}

impl MeshOctree {
    /// Create a new empty mesh octree with the given bounds.
    pub fn new(bounds: AABB) -> Self {
        Self {
            root: MeshNode::empty(bounds),
            bounds,
        }
    }

    // === Mesh Generation ===

    /// Generate mesh for a region from voxel data.
    ///
    /// This samples the SVO, runs marching cubes, and stores the resulting
    /// triangles in the appropriate octree leaves, splitting as needed.
    pub fn generate_from_voxels<F>(&mut self, region: &AABB, voxel_size: f32, sample_voxel: F)
    where
        F: Fn(Point3<f32>) -> Voxel,
    {
        // Sample voxels in a grid covering the region (with padding for marching cubes)
        let padded_bounds = AABB::new(
            region.min - Vector3::new(voxel_size, voxel_size, voxel_size),
            region.max + Vector3::new(voxel_size, voxel_size, voxel_size),
        );

        let size = padded_bounds.size();
        let resolution_x = (size.x / voxel_size).ceil() as usize + 1;
        let resolution_y = (size.y / voxel_size).ceil() as usize + 1;
        let resolution_z = (size.z / voxel_size).ceil() as usize + 1;

        // Sample voxel grid
        let mut grid = vec![vec![vec![Voxel::air(); resolution_z]; resolution_y]; resolution_x];
        for x in 0..resolution_x {
            for y in 0..resolution_y {
                for z in 0..resolution_z {
                    let pos = Point3::new(
                        padded_bounds.min.x + x as f32 * voxel_size,
                        padded_bounds.min.y + y as f32 * voxel_size,
                        padded_bounds.min.z + z as f32 * voxel_size,
                    );
                    grid[x][y][z] = sample_voxel(pos);
                }
            }
        }

        // Run marching cubes
        let marching_cubes = MarchingCubes::new();
        let mesh = marching_cubes.generate(&grid, padded_bounds.min, voxel_size);

        // Convert to vertices and insert into octree
        let vertices: Vec<Vertex> = mesh
            .positions
            .iter()
            .enumerate()
            .map(|(i, pos)| Vertex {
                pos: nalgebra::Vector4::new(pos.x, pos.y, pos.z, 1.0),
                color: nalgebra::Vector4::new(
                    mesh.colors[i][0],
                    mesh.colors[i][1],
                    mesh.colors[i][2],
                    mesh.colors[i][3],
                ),
                tex_coords: nalgebra::Vector2::new(pos.x * 0.1, pos.z * 0.1),
                normal: mesh.normals[i],
            })
            .collect();

        // Clear existing geometry in the region
        Self::clear_region_recursive(&mut self.root, region);

        // Insert triangles
        for tri_idx in (0..mesh.indices.len()).step_by(3) {
            let i0 = mesh.indices[tri_idx] as usize;
            let i1 = mesh.indices[tri_idx + 1] as usize;
            let i2 = mesh.indices[tri_idx + 2] as usize;

            let v0 = &vertices[i0];
            let v1 = &vertices[i1];
            let v2 = &vertices[i2];

            // Find min vertex for ownership
            let positions = [
                Point3::new(v0.pos.x, v0.pos.y, v0.pos.z),
                Point3::new(v1.pos.x, v1.pos.y, v1.pos.z),
                Point3::new(v2.pos.x, v2.pos.y, v2.pos.z),
            ];
            let min_vertex = positions
                .into_iter()
                .min_by(|a, b| {
                    a.x.partial_cmp(&b.x)
                        .unwrap()
                        .then(a.y.partial_cmp(&b.y).unwrap())
                        .then(a.z.partial_cmp(&b.z).unwrap())
                })
                .unwrap();

            if self.bounds.contains_point(min_vertex) {
                Self::insert_triangle_recursive(&mut self.root, v0, v1, v2, min_vertex, 0);
            }
        }

        // After all triangles inserted, build neighbor references
        self.rebuild_neighbor_refs();
    }

    /// Clear all geometry that intersects a region (associated function).
    ///
    /// Removes triangles whose AABB intersects the region, not just those
    /// whose min-vertex is inside. This is necessary because a triangle
    /// extending into the region may have had its voxels modified.
    fn clear_region_recursive(node: &mut MeshNode, region: &AABB) {
        if !node.bounds.intersects(region) {
            return;
        }

        match &mut node.content {
            MeshNodeContent::Empty => {}
            MeshNodeContent::Leaf(leaf) => {
                // Remove triangles that intersect the region (by AABB)
                let mut keep_indices = Vec::new();
                let mut keep_vertices = Vec::new();
                let mut vertex_map = std::collections::HashMap::new();

                for tri in 0..leaf.owned_triangle_count() {
                    let tri_aabb = leaf.triangle_aabb(tri);
                    if !region.intersects(&tri_aabb) {
                        // Keep this triangle - it doesn't touch the dirty region
                        let base = tri * 3;
                        for offset in 0..3 {
                            let old_idx = leaf.indices[base + offset] as usize;
                            let new_idx = *vertex_map.entry(old_idx).or_insert_with(|| {
                                let idx = keep_vertices.len();
                                keep_vertices.push(leaf.vertices[old_idx].clone());
                                idx
                            });
                            keep_indices.push(new_idx as u32);
                        }
                    }
                }

                leaf.vertices = keep_vertices;
                leaf.indices = keep_indices;
                leaf.neighbor_refs.clear();

                // Convert to empty if no triangles remain
                if leaf.vertices.is_empty() {
                    node.content = MeshNodeContent::Empty;
                }
            }
            MeshNodeContent::Interior(children) => {
                for child in children.iter_mut() {
                    Self::clear_region_recursive(child, region);
                }
                // Try to collapse if all children are empty
                if children
                    .iter()
                    .all(|c| matches!(c.content, MeshNodeContent::Empty))
                {
                    node.content = MeshNodeContent::Empty;
                }
            }
        }
    }

    /// Insert a triangle into the octree (associated function).
    fn insert_triangle_recursive(
        node: &mut MeshNode,
        v0: &Vertex,
        v1: &Vertex,
        v2: &Vertex,
        min_vertex: Point3<f32>,
        depth: u32,
    ) {
        // Ensure node is at least a leaf
        if matches!(node.content, MeshNodeContent::Empty) {
            node.content = MeshNodeContent::Leaf(MeshLeaf::new());
        }

        match &mut node.content {
            MeshNodeContent::Empty => unreachable!(),
            MeshNodeContent::Leaf(leaf) => {
                // Add triangle to this leaf
                let base_idx = leaf.vertices.len() as u32;
                leaf.vertices.push(v0.clone());
                leaf.vertices.push(v1.clone());
                leaf.vertices.push(v2.clone());
                leaf.indices.push(base_idx);
                leaf.indices.push(base_idx + 1);
                leaf.indices.push(base_idx + 2);

                // Check if we need to split
                if leaf.owned_triangle_count() > MAX_TRIANGLES_PER_LEAF && depth < MAX_MESH_DEPTH {
                    Self::split_leaf(node, depth);
                }
            }
            MeshNodeContent::Interior(children) => {
                // Find which child contains the min vertex
                let octant = octant_index(&node.bounds, min_vertex);
                Self::insert_triangle_recursive(
                    &mut children[octant],
                    v0,
                    v1,
                    v2,
                    min_vertex,
                    depth + 1,
                );
            }
        }
    }

    /// Split a leaf node into 8 children (associated function).
    fn split_leaf(node: &mut MeshNode, depth: u32) {
        let MeshNodeContent::Leaf(leaf) =
            std::mem::replace(&mut node.content, MeshNodeContent::Empty)
        else {
            return;
        };

        // Create 8 child nodes
        let children: [MeshNode; 8] =
            std::array::from_fn(|i| MeshNode::empty(child_bounds(&node.bounds, i)));
        node.content = MeshNodeContent::Interior(Box::new(children));

        // Re-insert all triangles
        let MeshNodeContent::Interior(children) = &mut node.content else {
            unreachable!()
        };

        for tri in 0..leaf.indices.len() / 3 {
            let base = tri * 3;
            let i0 = leaf.indices[base] as usize;
            let i1 = leaf.indices[base + 1] as usize;
            let i2 = leaf.indices[base + 2] as usize;
            let v0 = &leaf.vertices[i0];
            let v1 = &leaf.vertices[i1];
            let v2 = &leaf.vertices[i2];

            let positions = [
                Point3::new(v0.pos.x, v0.pos.y, v0.pos.z),
                Point3::new(v1.pos.x, v1.pos.y, v1.pos.z),
                Point3::new(v2.pos.x, v2.pos.y, v2.pos.z),
            ];
            let min_vertex = positions
                .into_iter()
                .min_by(|a, b| {
                    a.x.partial_cmp(&b.x)
                        .unwrap()
                        .then(a.y.partial_cmp(&b.y).unwrap())
                        .then(a.z.partial_cmp(&b.z).unwrap())
                })
                .unwrap();

            let octant = octant_index(&node.bounds, min_vertex);
            Self::insert_triangle_recursive(
                &mut children[octant],
                v0,
                v1,
                v2,
                min_vertex,
                depth + 1,
            );
        }
    }

    /// Rebuild neighbor references for all leaves.
    ///
    /// This ensures each leaf has references to all triangles that intersect
    /// its bounds, enabling complete spatial queries.
    fn rebuild_neighbor_refs(&mut self) {
        // First, collect all triangles with their owners and AABBs
        let mut all_triangles: Vec<(u64, u8, u32, AABB)> = Vec::new();
        Self::collect_triangles(&self.root, 0, 0, &mut all_triangles);

        // Clear existing neighbor refs
        Self::clear_neighbor_refs(&mut self.root);

        // For each triangle, find all leaves it intersects and add refs
        for (path, depth, tri_idx, aabb) in all_triangles {
            Self::add_neighbor_refs_for_triangle(&mut self.root, 0, 0, path, depth, tri_idx, &aabb);
        }
    }

    fn collect_triangles(
        node: &MeshNode,
        depth: u8,
        path: u64,
        out: &mut Vec<(u64, u8, u32, AABB)>,
    ) {
        match &node.content {
            MeshNodeContent::Empty => {}
            MeshNodeContent::Leaf(leaf) => {
                for tri in 0..leaf.owned_triangle_count() {
                    let aabb = leaf.triangle_aabb(tri);
                    out.push((path, depth, tri as u32, aabb));
                }
            }
            MeshNodeContent::Interior(children) => {
                for (i, child) in children.iter().enumerate() {
                    let child_path = path | ((i as u64) << (depth * 3));
                    Self::collect_triangles(child, depth + 1, child_path, out);
                }
            }
        }
    }

    fn clear_neighbor_refs(node: &mut MeshNode) {
        match &mut node.content {
            MeshNodeContent::Empty => {}
            MeshNodeContent::Leaf(leaf) => {
                leaf.neighbor_refs.clear();
            }
            MeshNodeContent::Interior(children) => {
                for child in children.iter_mut() {
                    Self::clear_neighbor_refs(child);
                }
            }
        }
    }

    fn add_neighbor_refs_for_triangle(
        node: &mut MeshNode,
        current_depth: u8,
        current_path: u64,
        owner_path: u64,
        owner_depth: u8,
        tri_idx: u32,
        tri_aabb: &AABB,
    ) {
        if !node.bounds.intersects(tri_aabb) {
            return;
        }

        match &mut node.content {
            MeshNodeContent::Empty => {}
            MeshNodeContent::Leaf(leaf) => {
                // Add ref if this is not the owning leaf
                if current_path != owner_path || current_depth != owner_depth {
                    leaf.neighbor_refs
                        .push(TriangleRef::new(owner_path, owner_depth, tri_idx));
                }
            }
            MeshNodeContent::Interior(children) => {
                for (i, child) in children.iter_mut().enumerate() {
                    let child_path = current_path | ((i as u64) << (current_depth * 3));
                    Self::add_neighbor_refs_for_triangle(
                        child,
                        current_depth + 1,
                        child_path,
                        owner_path,
                        owner_depth,
                        tri_idx,
                        tri_aabb,
                    );
                }
            }
        }
    }

    // === Spatial Queries ===

    /// Query all triangles intersecting an AABB.
    ///
    /// Returns collision triangles for intersection testing.
    pub fn query_aabb(&self, query: &AABB) -> Vec<Triangle> {
        let mut seen = HashSet::new();
        let mut triangles = Vec::new();
        self.query_aabb_recursive(&self.root, 0, 0, query, &mut seen, &mut triangles);
        triangles
    }

    fn query_aabb_recursive(
        &self,
        node: &MeshNode,
        depth: u8,
        path: u64,
        query: &AABB,
        seen: &mut HashSet<(u64, u8, u32)>,
        out: &mut Vec<Triangle>,
    ) {
        if !node.bounds.intersects(query) {
            return;
        }

        match &node.content {
            MeshNodeContent::Empty => {}
            MeshNodeContent::Leaf(leaf) => {
                // Add owned triangles
                for tri in 0..leaf.owned_triangle_count() {
                    let key = (path, depth, tri as u32);
                    if seen.insert(key) {
                        let aabb = leaf.triangle_aabb(tri);
                        if query.intersects(&aabb) {
                            out.push(leaf.to_collision_triangle(tri));
                        }
                    }
                }

                // Add neighbor triangles
                for tri_ref in &leaf.neighbor_refs {
                    let key = (tri_ref.path, tri_ref.depth, tri_ref.triangle_index);
                    if seen.insert(key) {
                        if let Some(triangle) = self.resolve_triangle_ref(tri_ref) {
                            out.push(triangle);
                        }
                    }
                }
            }
            MeshNodeContent::Interior(children) => {
                for (i, child) in children.iter().enumerate() {
                    let child_path = path | ((i as u64) << (depth * 3));
                    self.query_aabb_recursive(child, depth + 1, child_path, query, seen, out);
                }
            }
        }
    }

    /// Resolve a triangle reference to actual triangle data.
    fn resolve_triangle_ref(&self, tri_ref: &TriangleRef) -> Option<Triangle> {
        let mut node = &self.root;
        for level in 0..tri_ref.depth {
            match &node.content {
                MeshNodeContent::Interior(children) => {
                    let octant = tri_ref.octant_at_level(level);
                    node = &children[octant];
                }
                _ => return None,
            }
        }

        if let MeshNodeContent::Leaf(leaf) = &node.content {
            if (tri_ref.triangle_index as usize) < leaf.owned_triangle_count() {
                return Some(leaf.to_collision_triangle(tri_ref.triangle_index as usize));
            }
        }

        None
    }

    /// Get all vertices and indices for rendering.
    ///
    /// Combines all triangles from all leaves into flat arrays suitable for GPU upload.
    pub fn get_render_data(&self) -> (Vec<Vertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        Self::collect_render_data(&self.root, &mut vertices, &mut indices);
        (vertices, indices)
    }

    fn collect_render_data(node: &MeshNode, vertices: &mut Vec<Vertex>, indices: &mut Vec<u32>) {
        match &node.content {
            MeshNodeContent::Empty => {}
            MeshNodeContent::Leaf(leaf) => {
                let base = vertices.len() as u32;
                vertices.extend_from_slice(&leaf.vertices);
                indices.extend(leaf.indices.iter().map(|i| i + base));
            }
            MeshNodeContent::Interior(children) => {
                for child in children.iter() {
                    Self::collect_render_data(child, vertices, indices);
                }
            }
        }
    }

    /// Get render data for leaves intersecting a frustum (for frustum culling).
    ///
    /// `frustum_planes` should be 6 planes in the order: left, right, bottom, top, near, far.
    /// Each plane is (normal, distance) where the plane equation is dot(normal, point) + distance >= 0
    /// for points inside the frustum.
    #[allow(dead_code)]
    pub fn get_render_data_frustum_culled(
        &self,
        frustum_planes: &[(Vector3<f32>, f32); 6],
    ) -> (Vec<Vertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();
        Self::collect_render_data_culled(&self.root, frustum_planes, &mut vertices, &mut indices);
        (vertices, indices)
    }

    #[allow(dead_code)]
    fn collect_render_data_culled(
        node: &MeshNode,
        frustum_planes: &[(Vector3<f32>, f32); 6],
        vertices: &mut Vec<Vertex>,
        indices: &mut Vec<u32>,
    ) {
        // Test AABB against frustum
        if !aabb_intersects_frustum(&node.bounds, frustum_planes) {
            return;
        }

        match &node.content {
            MeshNodeContent::Empty => {}
            MeshNodeContent::Leaf(leaf) => {
                let base = vertices.len() as u32;
                vertices.extend_from_slice(&leaf.vertices);
                indices.extend(leaf.indices.iter().map(|i| i + base));
            }
            MeshNodeContent::Interior(children) => {
                for child in children.iter() {
                    Self::collect_render_data_culled(child, frustum_planes, vertices, indices);
                }
            }
        }
    }

    // === Statistics ===

    /// Count total triangles in the octree.
    pub fn triangle_count(&self) -> usize {
        Self::count_triangles(&self.root)
    }

    fn count_triangles(node: &MeshNode) -> usize {
        match &node.content {
            MeshNodeContent::Empty => 0,
            MeshNodeContent::Leaf(leaf) => leaf.owned_triangle_count(),
            MeshNodeContent::Interior(children) => {
                children.iter().map(|c| Self::count_triangles(c)).sum()
            }
        }
    }

    /// Count leaf nodes.
    pub fn leaf_count(&self) -> usize {
        Self::count_leaves(&self.root)
    }

    fn count_leaves(node: &MeshNode) -> usize {
        match &node.content {
            MeshNodeContent::Empty => 0,
            MeshNodeContent::Leaf(_) => 1,
            MeshNodeContent::Interior(children) => {
                children.iter().map(|c| Self::count_leaves(c)).sum()
            }
        }
    }
}

/// Compute which octant a point falls into within a bounding box.
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

/// Test if an AABB intersects a frustum defined by 6 planes.
fn aabb_intersects_frustum(aabb: &AABB, planes: &[(Vector3<f32>, f32); 6]) -> bool {
    for (normal, distance) in planes {
        // Find the corner of the AABB most in the direction of the plane normal
        let p = Point3::new(
            if normal.x >= 0.0 {
                aabb.max.x
            } else {
                aabb.min.x
            },
            if normal.y >= 0.0 {
                aabb.max.y
            } else {
                aabb.min.y
            },
            if normal.z >= 0.0 {
                aabb.max.z
            } else {
                aabb.min.z
            },
        );

        // If this corner is outside the plane, the AABB is outside the frustum
        if normal.dot(&p.coords) + distance < 0.0 {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_vertex(x: f32, y: f32, z: f32) -> Vertex {
        Vertex {
            pos: nalgebra::Vector4::new(x, y, z, 1.0),
            color: nalgebra::Vector4::new(1.0, 1.0, 1.0, 1.0),
            tex_coords: nalgebra::Vector2::new(0.0, 0.0),
            normal: nalgebra::Vector3::new(0.0, 1.0, 0.0),
        }
    }

    #[test]
    fn test_insert_and_query() {
        let bounds = AABB::new(Point3::origin(), Point3::new(10.0, 10.0, 10.0));
        let mut octree = MeshOctree::new(bounds);

        // Insert a triangle
        let v0 = test_vertex(1.0, 1.0, 1.0);
        let v1 = test_vertex(2.0, 1.0, 1.0);
        let v2 = test_vertex(1.5, 2.0, 1.0);

        let min_vertex = Point3::new(1.0, 1.0, 1.0);
        MeshOctree::insert_triangle_recursive(&mut octree.root, &v0, &v1, &v2, min_vertex, 0);
        octree.rebuild_neighbor_refs();

        assert_eq!(octree.triangle_count(), 1);

        // Query should find the triangle
        let query = AABB::new(Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 3.0, 3.0));
        let results = octree.query_aabb(&query);
        assert_eq!(results.len(), 1);

        // Query outside should find nothing
        let query = AABB::new(Point3::new(5.0, 5.0, 5.0), Point3::new(6.0, 6.0, 6.0));
        let results = octree.query_aabb(&query);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_splitting() {
        let bounds = AABB::new(Point3::origin(), Point3::new(10.0, 10.0, 10.0));
        let mut octree = MeshOctree::new(bounds);

        // Insert more triangles than MAX_TRIANGLES_PER_LEAF
        for i in 0..(MAX_TRIANGLES_PER_LEAF + 10) {
            let x = (i % 10) as f32;
            let y = ((i / 10) % 10) as f32;
            let z = (i / 100) as f32;
            let v0 = test_vertex(x, y, z);
            let v1 = test_vertex(x + 0.1, y, z);
            let v2 = test_vertex(x + 0.05, y + 0.1, z);
            let min_vertex = Point3::new(x, y, z);
            MeshOctree::insert_triangle_recursive(&mut octree.root, &v0, &v1, &v2, min_vertex, 0);
        }

        assert_eq!(octree.triangle_count(), MAX_TRIANGLES_PER_LEAF + 10);
        assert!(
            octree.leaf_count() > 1,
            "Should have split into multiple leaves"
        );
    }

    #[test]
    fn test_boundary_triangle_query() {
        let bounds = AABB::new(Point3::origin(), Point3::new(10.0, 10.0, 10.0));
        let mut octree = MeshOctree::new(bounds);

        // Insert a triangle that spans the center boundary
        // Min vertex at (4.0, 1.0, 1.0) - left of center
        // Other vertices at (6.0, ...) - right of center
        let v0 = test_vertex(4.0, 1.0, 1.0);
        let v1 = test_vertex(6.0, 1.0, 1.0);
        let v2 = test_vertex(5.0, 2.0, 1.0);
        let min_vertex = Point3::new(4.0, 1.0, 1.0);
        MeshOctree::insert_triangle_recursive(&mut octree.root, &v0, &v1, &v2, min_vertex, 0);
        octree.rebuild_neighbor_refs();

        // Query the right side should still find the triangle
        let query = AABB::new(Point3::new(5.5, 0.0, 0.0), Point3::new(7.0, 3.0, 3.0));
        let results = octree.query_aabb(&query);
        assert_eq!(results.len(), 1, "Should find triangle that spans boundary");
    }
}
