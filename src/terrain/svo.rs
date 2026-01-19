//! Sparse Voxel Octree for efficient terrain storage.
//!
//! The SVO stores voxel data hierarchically, allowing:
//! - Efficient memory usage (empty regions use minimal storage)
//! - Fast spatial queries (octree traversal)
//! - Easy modification (update nodes, propagate changes)

use nalgebra::Point3;

use super::voxel::Voxel;
use crate::collision::AABB;

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
    /// Create an air (empty) leaf node.
    pub fn empty() -> Self {
        SvoNode::Leaf(Voxel::air())
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
}

impl SparseVoxelOctree {
    /// Create a new SVO with the given world bounds and maximum depth.
    ///
    /// The minimum voxel size will be `bounds.size() / 2^max_depth`.
    pub fn new(bounds: AABB, max_depth: u32) -> Self {
        Self {
            root: SvoNode::empty(),
            bounds,
            max_depth,
        }
    }

    /// Get the world bounds of this SVO.
    pub fn bounds(&self) -> &AABB {
        &self.bounds
    }

    /// Get the minimum voxel size (at max depth).
    pub fn min_voxel_size(&self) -> f32 {
        let size = self.bounds.size();
        let divisions = (1 << self.max_depth) as f32;
        size.x.min(size.y).min(size.z) / divisions
    }

    /// Get the voxel size at maximum depth.
    pub fn voxel_size(&self) -> f32 {
        self.bounds.size().x / (1 << self.max_depth) as f32
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
