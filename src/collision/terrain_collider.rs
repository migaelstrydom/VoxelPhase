//! Terrain collision structure using SVO for spatial indexing.
//!
//! This combines the SVO's spatial hierarchy with triangle storage for
//! efficient collision queries. Triangles are indexed by the SVO leaf
//! regions they belong to, enabling O(log n) queries.

use std::collections::HashMap;

use nalgebra::Point3;

use super::{
    sphere_triangle_collision, swept_sphere_triangle, ContactPoint, SweptContact, Triangle, AABB,
};

/// A terrain collider that uses SVO-based spatial indexing for fast queries.
///
/// Triangles are stored in a flat array and indexed by their position in the
/// SVO hierarchy. Queries traverse the SVO to find relevant regions, then
/// only check triangles in those regions.
pub struct TerrainCollider {
    /// All triangles in the terrain.
    triangles: Vec<Triangle>,

    /// Spatial index: maps region keys to triangle indices.
    /// The key is computed from the SVO position at a fixed depth.
    spatial_index: HashMap<RegionKey, Vec<usize>>,

    /// World bounds of the terrain.
    bounds: AABB,

    /// Depth of spatial indexing (determines region size).
    /// Higher = smaller regions = more precise but more memory.
    index_depth: u32,

    /// Size of each indexed region.
    region_size: f32,
}

/// Key for spatial indexing - identifies a region in 3D grid space.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct RegionKey {
    x: i32,
    y: i32,
    z: i32,
}

impl RegionKey {
    fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    /// Get all region keys that overlap with an AABB.
    fn keys_overlapping(aabb: &AABB, region_size: f32) -> impl Iterator<Item = RegionKey> {
        let min_x = (aabb.min.x / region_size).floor() as i32;
        let min_y = (aabb.min.y / region_size).floor() as i32;
        let min_z = (aabb.min.z / region_size).floor() as i32;
        let max_x = (aabb.max.x / region_size).floor() as i32;
        let max_y = (aabb.max.y / region_size).floor() as i32;
        let max_z = (aabb.max.z / region_size).floor() as i32;

        (min_x..=max_x).flat_map(move |x| {
            (min_y..=max_y).flat_map(move |y| (min_z..=max_z).map(move |z| RegionKey::new(x, y, z)))
        })
    }
}

impl TerrainCollider {
    /// Create a new terrain collider for the given bounds.
    ///
    /// `index_depth` controls the granularity of spatial indexing.
    /// Higher values = smaller regions = faster queries but more memory.
    /// Typical values: 4-6 for a 64-unit world.
    pub fn new(bounds: AABB, index_depth: u32) -> Self {
        let world_size = bounds.size().x.max(bounds.size().y).max(bounds.size().z);
        let region_size = world_size / (1 << index_depth) as f32;

        Self {
            triangles: Vec::new(),
            spatial_index: HashMap::new(),
            bounds,
            index_depth,
            region_size,
        }
    }

    /// Add triangles to the collider.
    ///
    /// Each triangle is indexed by all regions it overlaps.
    pub fn add_triangles(&mut self, triangles: impl IntoIterator<Item = Triangle>) {
        for triangle in triangles {
            let idx = self.triangles.len();
            self.triangles.push(triangle);

            // Index by all regions the triangle overlaps (using its bounding box)
            let tri_aabb = triangle.aabb();
            for key in RegionKey::keys_overlapping(&tri_aabb, self.region_size) {
                self.spatial_index.entry(key).or_default().push(idx);
            }
        }
    }

    /// Clear all triangles and rebuild from scratch.
    pub fn clear(&mut self) {
        self.triangles.clear();
        self.spatial_index.clear();
    }

    /// Get the number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Query for sphere collision contacts.
    ///
    /// Returns all contact points where the sphere intersects triangles.
    pub fn query_sphere(&self, center: Point3<f32>, radius: f32) -> Vec<ContactPoint> {
        let query_aabb =
            AABB::from_center_half_extents(center, nalgebra::Vector3::new(radius, radius, radius));

        let mut contacts = Vec::new();
        let mut checked = std::collections::HashSet::new();

        // Get all regions that overlap the query
        for key in RegionKey::keys_overlapping(&query_aabb, self.region_size) {
            if let Some(indices) = self.spatial_index.get(&key) {
                for &idx in indices {
                    // Avoid checking the same triangle twice
                    if checked.insert(idx) {
                        if let Some(contact) =
                            sphere_triangle_collision(center, radius, &self.triangles[idx])
                        {
                            contacts.push(contact);
                        }
                    }
                }
            }
        }

        contacts
    }

    /// Query for swept sphere collision (continuous collision detection).
    ///
    /// Returns the earliest contact along the sphere's path from start to end.
    pub fn query_swept_sphere(
        &self,
        start: Point3<f32>,
        end: Point3<f32>,
        radius: f32,
    ) -> Option<SweptContact> {
        // Build AABB encompassing the entire sweep
        let min_x = start.x.min(end.x) - radius;
        let min_y = start.y.min(end.y) - radius;
        let min_z = start.z.min(end.z) - radius;
        let max_x = start.x.max(end.x) + radius;
        let max_y = start.y.max(end.y) + radius;
        let max_z = start.z.max(end.z) + radius;

        let query_aabb = AABB::new(
            Point3::new(min_x, min_y, min_z),
            Point3::new(max_x, max_y, max_z),
        );

        let mut earliest: Option<SweptContact> = None;
        let mut checked = std::collections::HashSet::new();

        // Get all regions that overlap the query
        for key in RegionKey::keys_overlapping(&query_aabb, self.region_size) {
            if let Some(indices) = self.spatial_index.get(&key) {
                for &idx in indices {
                    if checked.insert(idx) {
                        if let Some(contact) =
                            swept_sphere_triangle(start, end, radius, &self.triangles[idx])
                        {
                            if earliest.as_ref().map_or(true, |e| contact.t < e.t) {
                                earliest = Some(contact);
                            }
                        }
                    }
                }
            }
        }

        earliest
    }

    /// Convert a world point to a region key.
    fn point_to_key(&self, point: Point3<f32>) -> RegionKey {
        RegionKey::new(
            (point.x / self.region_size).floor() as i32,
            (point.y / self.region_size).floor() as i32,
            (point.z / self.region_size).floor() as i32,
        )
    }

    /// Get statistics about the spatial index.
    pub fn stats(&self) -> TerrainColliderStats {
        let total_triangles = self.triangles.len();
        let occupied_regions = self.spatial_index.len();
        let max_triangles_per_region = self
            .spatial_index
            .values()
            .map(|v| v.len())
            .max()
            .unwrap_or(0);
        let avg_triangles_per_region = if occupied_regions > 0 {
            total_triangles as f32 / occupied_regions as f32
        } else {
            0.0
        };

        TerrainColliderStats {
            total_triangles,
            occupied_regions,
            max_triangles_per_region,
            avg_triangles_per_region,
            region_size: self.region_size,
        }
    }
}

/// Statistics about the terrain collider's spatial index.
#[derive(Debug, Clone)]
pub struct TerrainColliderStats {
    pub total_triangles: usize,
    pub occupied_regions: usize,
    pub max_triangles_per_region: usize,
    pub avg_triangles_per_region: f32,
    pub region_size: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_terrain_collider_basic() {
        let bounds = AABB::new(
            Point3::new(-10.0, -10.0, -10.0),
            Point3::new(10.0, 10.0, 10.0),
        );
        let mut collider = TerrainCollider::new(bounds, 4);

        // Add a simple ground triangle
        let triangle = Triangle::new(
            Point3::new(-5.0, 0.0, -5.0),
            Point3::new(5.0, 0.0, -5.0),
            Point3::new(0.0, 0.0, 5.0),
        );
        collider.add_triangles([triangle]);

        assert_eq!(collider.triangle_count(), 1);

        // Query above the triangle - should find contact
        let contacts = collider.query_sphere(Point3::new(0.0, 0.4, 0.0), 0.5);
        assert!(!contacts.is_empty());

        // Query far away - should find nothing
        let contacts = collider.query_sphere(Point3::new(100.0, 100.0, 100.0), 0.5);
        assert!(contacts.is_empty());
    }

    #[test]
    fn test_swept_collision() {
        let bounds = AABB::new(
            Point3::new(-10.0, -10.0, -10.0),
            Point3::new(10.0, 10.0, 10.0),
        );
        let mut collider = TerrainCollider::new(bounds, 4);

        // Ground plane
        let triangle = Triangle::new(
            Point3::new(-10.0, 0.0, -10.0),
            Point3::new(10.0, 0.0, -10.0),
            Point3::new(0.0, 0.0, 10.0),
        );
        collider.add_triangles([triangle]);

        // Fast projectile from above
        let contact = collider.query_swept_sphere(
            Point3::new(0.0, 50.0, 0.0),
            Point3::new(0.0, -50.0, 0.0),
            0.1,
        );

        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!(c.t > 0.0 && c.t < 1.0);
    }
}
