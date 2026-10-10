//! Point queries against the water. Physics and gameplay see water only
//! through here.

use nalgebra::{Point3, Vector3};

use super::buoyancy::{WaterSample as SurfaceSample, WaterSurface};
use super::field::{WaterField, WET_DEPTH};
use super::ids::{StoreId, WaterBodyId};

/// The water at a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaterSample {
    /// Surface height here, including swell and ripples.
    pub surface: f32,
    /// Floor the water rests on.
    pub floor: f32,
    /// Bulk velocity: the depth-averaged current.
    pub velocity: Vector3<f32>,
    /// How fast the surface rises here, m/s: the swell's heave.
    pub rise: f32,
    /// The body this point belongs to.
    pub body: WaterBodyId,
}

/// The one body the field's water belongs to: a field has no stores, so
/// all its water is one body.
pub const FIELD_BODY: WaterBodyId = StoreId(0);

/// Point queries against one [`WaterField`].
#[derive(Clone, Copy)]
pub struct WaterQuery<'a> {
    field: &'a WaterField,
}

impl<'a> WaterQuery<'a> {
    pub fn new(field: &'a WaterField) -> Self {
        Self { field }
    }

    /// The level of the water at a 3D point. `None` if the point's span
    /// holds no water.
    pub fn level_at(&self, point: Point3<f32>) -> Option<f32> {
        self.field.level_at(point)
    }

    /// Water at a 3D point, or `None` if the point's span holds no water.
    ///
    /// The span is the one whose band holds the point: the air it is in, or
    /// for a point just inside the ground, the span resting on that ground.
    /// So a crate under a floating island reads the sea, and one on the
    /// island reads the island's pool.
    pub fn sample(&self, point: Point3<f32>) -> Option<WaterSample> {
        let cell = self.field.cell_at(point)?;
        let cells = self.field.cells();
        if cells.depth(cell) <= WET_DEPTH {
            return None;
        }
        let current = self.field.velocity(cell);
        Some(WaterSample {
            surface: cells.surface(cell) as f32,
            floor: cells.bed(cell),
            velocity: Vector3::new(current.x, 0.0, current.y),
            rise: 0.0,
            body: FIELD_BODY,
        })
    }
}

impl WaterSurface for WaterQuery<'_> {
    fn sample(&self, point: Point3<f32>) -> Option<SurfaceSample> {
        WaterQuery::sample(self, point).map(|s| SurfaceSample {
            surface_level: s.surface,
            floor_level: s.floor,
            velocity: s.velocity,
            surface_rise: s.rise,
        })
    }
}
