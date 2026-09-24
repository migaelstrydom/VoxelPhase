//! Point queries against the water. Physics and gameplay see water only
//! through here.

use nalgebra::{Point3, Vector3};

use super::buoyancy::{WaterSample as SurfaceSample, WaterSurface};
use super::geometry::Column;
use super::ids::WaterBodyId;
use super::world::WaterWorld;

/// The water at a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaterSample {
    /// Surface height here, including swell and ripples.
    pub surface: f32,
    /// Floor the water rests on (`floor_min` of the span).
    pub floor: f32,
    /// Bulk velocity: the reach's velocity, or the basin's potential-flow
    /// current.
    pub velocity: Vector3<f32>,
    /// The body this point belongs to.
    pub body: WaterBodyId,
}

/// Point queries against one [`WaterWorld`].
#[derive(Clone, Copy)]
pub struct WaterQuery<'a> {
    world: &'a WaterWorld,
}

impl<'a> WaterQuery<'a> {
    pub fn new(world: &'a WaterWorld) -> Self {
        Self { world }
    }

    /// The still level of the water at a 3D point, without swell or ripples:
    /// what the hydrology holds. `None` if the point's span holds no water.
    pub fn level_at(&self, point: Point3<f32>) -> Option<f32> {
        let graph = self.world.geometry().graph();
        let span = graph.span_at(Column::containing(point.x, point.z), point.y)?;
        let level = self.world.level(graph.owner(span).body?)?;
        (level > graph.span(span).floor_min).then_some(level)
    }

    /// Water at a 3D point, or `None` if the point's span holds no water. The
    /// surface includes swell and ripples, exactly as drawn.
    ///
    /// The span is the one whose band holds the point: the air it is in, or
    /// for a point just inside the ground, the span resting on that ground.
    /// So a crate under a floating island reads the sea, and one on the
    /// island reads the island's pool.
    pub fn sample(&self, point: Point3<f32>) -> Option<WaterSample> {
        let graph = self.world.geometry().graph();
        let column = Column::containing(point.x, point.z);
        let span = graph.span_at(column, point.y)?;
        let body = graph.owner(span).body?;
        let level = self.world.level(body)?;
        let floor = graph.span(span).floor_min;
        if level <= floor {
            return None;
        }
        let swell =
            self.world
                .swell(body)
                .height(point.x, point.z, self.world.clock(), level - floor);
        let ripple = self.world.ripples().height_at(body, point.x, point.z);
        Some(WaterSample {
            surface: level + swell + ripple,
            floor,
            velocity: Vector3::zeros(),
            body,
        })
    }
}

impl WaterSurface for WaterQuery<'_> {
    fn sample(&self, point: Point3<f32>) -> Option<SurfaceSample> {
        WaterQuery::sample(self, point).map(|s| SurfaceSample {
            surface_level: s.surface,
            floor_level: s.floor,
        })
    }
}
