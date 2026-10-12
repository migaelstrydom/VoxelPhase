//! Point queries against the water. Physics and gameplay see water only
//! through here.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use super::buoyancy::{WaterSample as SurfaceSample, WaterSurface};
use super::falling::{FallingWater, Strike};
use super::field::{WaterField, WET_DEPTH};
use super::ids::{SurfaceId, WaterBodyId};
use super::waves::SurfaceWaves;

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
pub const FIELD_BODY: WaterBodyId = SurfaceId(0);

/// Point queries against one [`WaterField`] and the waves over it.
#[derive(Clone, Copy)]
pub struct WaterQuery<'a> {
    field: &'a WaterField,
    waves: &'a SurfaceWaves,
    falling: &'a FallingWater,
    /// Seconds simulated: the swell's clock.
    clock: f32,
}

impl<'a> WaterQuery<'a> {
    pub fn new(
        field: &'a WaterField,
        waves: &'a SurfaceWaves,
        falling: &'a FallingWater,
        clock: f32,
    ) -> Self {
        Self {
            field,
            waves,
            falling,
            clock,
        }
    }

    /// The falling water striking a sphere at `centre` of `radius`.
    pub fn falling(&self, centre: Point3<f32>, radius: f32) -> SmallVec<[Strike; 4]> {
        self.falling.strikes(centre, radius)
    }

    /// The still level of the water at a 3D point, without swell or
    /// ripples. `None` if the point's span holds no water.
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
        let still = cells.surface(cell) as f32;
        let depth = cells.depth(cell) as f32;
        let swell = self.waves.swell_at(still);
        let (x, z, t) = (point.x, point.z, self.clock);
        Some(WaterSample {
            surface: still + swell.height(x, z, t, depth) + self.waves.ripple_at(x, z),
            floor: cells.bed(cell),
            velocity: Vector3::new(current.x, 0.0, current.y),
            rise: swell.rise_rate(x, z, t, depth),
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

    fn falling(&self, centre: Point3<f32>, radius: f32) -> SmallVec<[Strike; 4]> {
        WaterQuery::falling(self, centre, radius)
    }
}
