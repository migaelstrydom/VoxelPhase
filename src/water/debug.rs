//! Drawing the water network into the scene, for finding where water went.
//!
//! The ledger says *that* water went missing; these overlays say *where*.
//! Everything here writes into [`DebugOverlays`], so the game and the offline
//! viewers draw the same shapes.

use nalgebra::Point3;

use crate::debug::DebugOverlays;
use crate::rendering::Colour;

use super::geometry::{Column, Drain, WaterGeometry, COLUMN_SIZE};

/// What the network debug view draws, and how far from its centre.
#[derive(Debug, Clone, Copy)]
pub struct WaterNetworkDebug {
    /// Drainage directions: an arrow from each span's floor towards the span
    /// it drains to.
    pub drainage: bool,
    /// Horizontal radius around the centre, in metres.
    pub radius: f32,
}

impl Default for WaterNetworkDebug {
    fn default() -> Self {
        Self {
            drainage: false,
            radius: 12.0,
        }
    }
}

const DRAIN_COLOUR: Colour = Colour::rgb(0.25, 0.55, 1.0);
const POOLED_COLOUR: Colour = Colour::rgb(0.1, 0.3, 0.9);
const OUTLET_COLOUR: Colour = Colour::rgb(1.0, 0.5, 0.1);
const PENDING_COLOUR: Colour = Colour::rgb(0.9, 0.9, 0.2);

impl WaterNetworkDebug {
    /// Draw the enabled layers around `centre`.
    pub fn draw(
        &self,
        overlays: &mut DebugOverlays,
        geometry: &WaterGeometry,
        centre: Point3<f32>,
    ) {
        if self.drainage {
            self.draw_drainage(overlays, geometry, centre);
        }
    }

    /// Drain arrows for every span within reach of `centre` whose floor is
    /// within the radius vertically too, so a cave's drains do not draw
    /// through the ground over it.
    fn draw_drainage(
        &self,
        overlays: &mut DebugOverlays,
        geometry: &WaterGeometry,
        centre: Point3<f32>,
    ) {
        let graph = geometry.graph();
        let drainage = geometry.drainage();
        let reach = (self.radius / COLUMN_SIZE).ceil() as i32;
        let middle = Column::containing(centre.x, centre.z);
        for dk in -reach..=reach {
            for di in -reach..=reach {
                let column = middle.offset(di, dk);
                let (x, z) = column.centre();
                if (x - centre.x).hypot(z - centre.z) > self.radius {
                    continue;
                }
                for span in graph.refs(column) {
                    let floor = graph.span(span).floor_c;
                    if (floor - centre.y).abs() > self.radius {
                        continue;
                    }
                    let from = Point3::new(x, floor + 0.05, z);
                    let pooled = drainage.fill(graph, span) > graph.span(span).floor_min + 0.05;
                    match drainage.drain(graph, span) {
                        Drain::To { .. } => {
                            let Some(next) = drainage.downstream(graph, span) else {
                                continue;
                            };
                            let (nx, nz) = next.column.centre();
                            let to = Point3::new(nx, graph.span(next).floor_c + 0.05, nz);
                            // Three quarters of the way, so neighbouring arrows
                            // do not run into one another.
                            let tip = from + (to - from) * 0.75;
                            let colour = if pooled { POOLED_COLOUR } else { DRAIN_COLOUR };
                            overlays.add_line(from, tip, colour);
                        }
                        Drain::Outlet => overlays.add_sphere(from, 0.08, OUTLET_COLOUR),
                        Drain::Pending => overlays.add_sphere(from, 0.06, PENDING_COLOUR),
                        Drain::Sealed => overlays.add_sphere(from, 0.06, Colour::RED),
                    }
                }
            }
        }
    }
}
