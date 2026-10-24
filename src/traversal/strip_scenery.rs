use nalgebra::Point3;

use crate::collision::{MeshPatch, SurfaceId, Triangle, AABB};
use crate::physics::{StaticGeometry, StaticSurface};

/// Side of the grid cells triangles are filed in, in metres: a few of the
/// strip's casts wide.
const CELL: f32 = 0.25;

/// The static geometry of one region, fetched once and filed in a grid, so
/// each of the strip's casts finds its triangles without a query of its own
/// into the terrain: the strip makes thousands of narrow queries over the
/// same few metres. Kept from frame to frame, and read through a
/// [`SceneryView`] over the geometry it came from.
pub struct StripScenery {
    region: AABB,
    triangles: Vec<(Triangle, SurfaceId)>,
    bounds: Vec<AABB>,
    /// Triangle indices by cell, row after row along `x`.
    cells: Vec<Vec<u32>>,
    origin: (f32, f32),
    columns: usize,
    rows: usize,
}

impl StripScenery {
    /// The geometry of `source` within `region`.
    pub fn fetch(source: &dyn StaticGeometry, region: AABB) -> Self {
        let triangles = source.query_region_surfaces(&region);
        let bounds: Vec<AABB> = triangles
            .iter()
            .map(|(triangle, _)| triangle_bounds(triangle))
            .collect();
        let columns = (((region.max.x - region.min.x) / CELL).ceil() as usize).max(1);
        let rows = (((region.max.z - region.min.z) / CELL).ceil() as usize).max(1);
        let mut scenery = Self {
            region,
            triangles,
            bounds,
            cells: vec![Vec::new(); columns * rows],
            origin: (region.min.x, region.min.z),
            columns,
            rows,
        };
        for index in 0..scenery.bounds.len() {
            let ((x0, z0), (x1, z1)) = scenery.cell_span(&scenery.bounds[index]);
            for row in z0..=z1 {
                for column in x0..=x1 {
                    scenery.cells[row * columns + column].push(index as u32);
                }
            }
        }
        scenery
    }

    /// Whether `aabb` lies within the fetched region, so this can answer it.
    pub fn covers(&self, aabb: &AABB) -> bool {
        aabb.min.x >= self.region.min.x
            && aabb.min.y >= self.region.min.y
            && aabb.min.z >= self.region.min.z
            && aabb.max.x <= self.region.max.x
            && aabb.max.y <= self.region.max.y
            && aabb.max.z <= self.region.max.z
    }

    /// The cells, as `(column, row)` from first to last, a box overlaps.
    fn cell_span(&self, aabb: &AABB) -> ((usize, usize), (usize, usize)) {
        let column =
            |x: f32| (((x - self.origin.0) / CELL).floor().max(0.0) as usize).min(self.columns - 1);
        let row =
            |z: f32| (((z - self.origin.1) / CELL).floor().max(0.0) as usize).min(self.rows - 1);
        (
            (column(aabb.min.x), row(aabb.min.z)),
            (column(aabb.max.x), row(aabb.max.z)),
        )
    }

    /// The triangles meeting `aabb`, which this covers, in the order they
    /// were fetched.
    fn surfaces_in(&self, aabb: &AABB) -> Vec<(Triangle, SurfaceId)> {
        let ((x0, z0), (x1, z1)) = self.cell_span(aabb);
        let mut found: Vec<u32> = (z0..=z1)
            .flat_map(|row| (x0..=x1).map(move |column| row * self.columns + column))
            .flat_map(|cell| self.cells[cell].iter().copied())
            .filter(|&index| self.bounds[index as usize].intersects(aabb))
            .collect();
        // In the order the source gave them, as a sweep expects; a triangle
        // filed in several cells once.
        found.sort_unstable();
        found.dedup();
        found
            .into_iter()
            .map(|index| self.triangles[index as usize])
            .collect()
    }
}

fn triangle_bounds(triangle: &Triangle) -> AABB {
    let min = Point3::new(
        triangle.v0.x.min(triangle.v1.x).min(triangle.v2.x),
        triangle.v0.y.min(triangle.v1.y).min(triangle.v2.y),
        triangle.v0.z.min(triangle.v1.z).min(triangle.v2.z),
    );
    let max = Point3::new(
        triangle.v0.x.max(triangle.v1.x).max(triangle.v2.x),
        triangle.v0.y.max(triangle.v1.y).max(triangle.v2.y),
        triangle.v0.z.max(triangle.v1.z).max(triangle.v2.z),
    );
    AABB::new(min, max)
}

/// The kept [`StripScenery`] over the geometry it came from: queries it
/// covers it answers, the rest go to the source.
pub struct SceneryView<'a> {
    pub strip: &'a StripScenery,
    pub source: &'a dyn StaticGeometry,
}

impl StaticGeometry for SceneryView<'_> {
    fn query_region(&self, aabb: &AABB) -> MeshPatch {
        self.source.query_region(aabb)
    }

    fn query_region_triangles(&self, aabb: &AABB) -> Vec<Triangle> {
        self.query_region_surfaces(aabb)
            .into_iter()
            .map(|(triangle, _)| triangle)
            .collect()
    }

    fn query_region_surfaces(&self, aabb: &AABB) -> Vec<(Triangle, SurfaceId)> {
        if self.strip.covers(aabb) {
            self.strip.surfaces_in(aabb)
        } else {
            self.source.query_region_surfaces(aabb)
        }
    }

    fn surface(&self, id: SurfaceId) -> Option<StaticSurface> {
        self.source.surface(id)
    }
}
