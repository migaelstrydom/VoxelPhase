/// Reuses static-geometry sweep queries across the substeps of a frame.
///
/// Static geometry does not change between substeps, but a body sweeping
/// across a frame queries an almost identical region each substep. On real
/// terrain a query costs ~5.4µs regardless of how few triangles it returns,
/// while each swept triangle test costs ~80ns — so the query, not the sweep,
/// dominates, and trading a larger triangle list for fewer queries is a clear
/// win.
///
/// A miss therefore fetches a region extended along the direction of travel by
/// the frame's substep count worth of displacement, so the remaining substeps
/// of that frame are served from the cache. Entries are dropped at frame boundaries,
/// where the geometry may legitimately have changed.
use nalgebra::Vector3;
use rustc_hash::FxHashMap;

use crate::collision::{Triangle, AABB};
use crate::physics::handle::ColliderHandle;
use crate::physics::static_geometry::StaticGeometry;

#[derive(Default)]
pub struct SweptPatchCache {
    entries: FxHashMap<ColliderHandle, CachedRegion>,
    /// Substeps of travel a fetched region is extended to cover, taken from
    /// the frame's declared substep count so one fetch spans the frame.
    lookahead_substeps: f32,
}

struct CachedRegion {
    /// Region the triangles were fetched for. Serves a sweep only while that
    /// sweep's own query box lies entirely inside this one.
    bounds: AABB,
    triangles: Vec<Triangle>,
}

impl SweptPatchCache {
    pub fn new() -> Self {
        Self {
            entries: FxHashMap::default(),
            lookahead_substeps: 1.0,
        }
    }

    /// Drop every cached region and size lookahead to the coming frame.
    /// Called once per frame, before substepping.
    pub fn begin_frame(&mut self, substeps: u32) {
        self.entries.clear();
        self.lookahead_substeps = substeps.max(1) as f32;
    }

    /// Triangles covering `query` for `collider`, fetching only on a miss.
    ///
    /// `displacement` is this substep's travel; it directs the extra reach a
    /// fetched region is given so later substeps of the same frame hit.
    pub fn triangles(
        &mut self,
        collider: ColliderHandle,
        query: &AABB,
        displacement: &Vector3<f32>,
        static_geometry: &dyn StaticGeometry,
    ) -> &[Triangle] {
        let hit = self
            .entries
            .get(&collider)
            .is_some_and(|entry| contains(&entry.bounds, query));

        if !hit {
            let bounds = extended(query, &(displacement * self.lookahead_substeps));
            let triangles = static_geometry.query_region_triangles(&bounds);
            self.entries
                .insert(collider, CachedRegion { bounds, triangles });
        }

        &self.entries[&collider].triangles
    }
}

/// Whether `outer` fully encloses `inner`.
fn contains(outer: &AABB, inner: &AABB) -> bool {
    (0..3).all(|axis| outer.min[axis] <= inner.min[axis] && outer.max[axis] >= inner.max[axis])
}

/// Grow an AABB along `reach`, extending only the faces it points toward.
fn extended(query: &AABB, reach: &Vector3<f32>) -> AABB {
    let mut min = query.min;
    let mut max = query.max;
    for axis in 0..3 {
        if reach[axis] >= 0.0 {
            max[axis] += reach[axis];
        } else {
            min[axis] += reach[axis];
        }
    }
    AABB::new(min, max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    fn aabb(min: [f32; 3], max: [f32; 3]) -> AABB {
        AABB::new(
            Point3::new(min[0], min[1], min[2]),
            Point3::new(max[0], max[1], max[2]),
        )
    }

    #[test]
    fn extended_grows_only_toward_travel() {
        let base = aabb([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let out = extended(&base, &Vector3::new(2.0, -3.0, 0.0));
        assert_eq!(out.max.x, 3.0);
        assert_eq!(out.min.x, 0.0);
        assert_eq!(out.min.y, -3.0);
        assert_eq!(out.max.y, 1.0);
        assert_eq!(out.min.z, 0.0);
        assert_eq!(out.max.z, 1.0);
    }

    #[test]
    fn contains_requires_full_enclosure() {
        let outer = aabb([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
        assert!(contains(&outer, &aabb([1.0, 1.0, 1.0], [2.0, 2.0, 2.0])));
        assert!(contains(&outer, &outer));
        assert!(!contains(&outer, &aabb([-0.1, 1.0, 1.0], [2.0, 2.0, 2.0])));
        assert!(!contains(&outer, &aabb([1.0, 1.0, 1.0], [2.0, 10.1, 2.0])));
    }
}
