use nalgebra::{Point3, Vector3};

use crate::physics::{ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc};
use crate::rendering::substance::{self, ColliderSubstance};

use super::scenario::PerfScenario;
use crate::perf::Ground;

/// Lift under the bottom row, so no crate starts the run embedded in the terrain.
const CLEARANCE: f32 = 0.02;
/// Horizontal gap between neighbouring crates in a row, so none starts the
/// run overlapping the next.
const ROW_GAP: f32 = 0.01;

/// A pyramid of crates standing on the ground.
///
/// ```text
///            ┌─┐
///          ┌─┼─┼─┐            `layers` rows, each one crate shorter than the
///        ┌─┼─┼─┼─┼─┐          row it sits on and offset by half a crate, so
///        └─┴─┴─┴─┴─┘          every crate carries the weight of those above
/// ```
///
/// Where the igloo blast is many independent bodies, this is one contact
/// graph as deep as the pyramid is tall: the case shock propagation and the
/// solver's iteration count exist for. Sleep is off by default so the stack
/// keeps being solved after it settles — a sleeping stack costs nothing.
pub struct BoxStack {
    /// Rows in the pyramid; the bottom row holds this many crates.
    pub layers: usize,
    /// Half the edge length of each cubic crate, in metres.
    pub half_extent: f32,
    /// Let the settled stack fall asleep, as it would in the game.
    pub sleep: bool,
}

impl Default for BoxStack {
    fn default() -> Self {
        Self {
            layers: 10,
            half_extent: 0.25,
            sleep: false,
        }
    }
}

impl BoxStack {
    fn crate_count(&self) -> usize {
        self.layers * (self.layers + 1) / 2
    }

    /// Centre-to-centre distance between neighbours in a row.
    fn pitch(&self) -> f32 {
        self.half_extent * 2.0 + ROW_GAP
    }

    /// Offset along the row of crate `index` in a row of `count` crates,
    /// centred on the site.
    fn row_offset(&self, index: usize, count: usize) -> f32 {
        (index as f32 - (count as f32 - 1.0) * 0.5) * self.pitch()
    }

    /// The highest ground under the bottom row, so every crate of it starts
    /// just clear of the terrain.
    fn floor_height(&self, ground: &Ground) -> f32 {
        let half = self.half_extent;
        (0..self.layers)
            .flat_map(|index| {
                let x = self.row_offset(index, self.layers);
                [(-half, -half), (-half, half), (half, -half), (half, half)]
                    .map(|(dx, dz)| ground.surface_point(x + dx, dz).y)
            })
            .fold(f32::NEG_INFINITY, f32::max)
    }
}

impl PerfScenario for BoxStack {
    fn name(&self) -> &str {
        "box_stack"
    }

    fn describe(&self) -> String {
        format!(
            "pyramid of {} layers × {} crates of {:.2} m, sleep {}",
            self.layers,
            self.crate_count(),
            self.half_extent * 2.0,
            if self.sleep { "on" } else { "off" }
        )
    }

    fn physics_config(&self) -> PhysicsConfig {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = self.sleep;
        config
    }

    fn populate(&self, world: &mut PhysicsWorld, ground: &Ground) {
        let pine = substance::PINE;
        let site = ground.surface_point(0.0, 0.0);
        let floor = self.floor_height(ground) + CLEARANCE;
        let height = self.half_extent * 2.0;

        for layer in 0..self.layers {
            let count = self.layers - layer;
            let y = floor + self.half_extent + layer as f32 * height;
            for index in 0..count {
                let centre = Point3::new(site.x + self.row_offset(index, count), y, site.z);
                let body = world.create_body(RigidBodyDesc::dynamic().position(centre));
                world.attach_collider(
                    body,
                    ColliderDesc::box_shape(Vector3::repeat(self.half_extent)).of(&pine),
                );
            }
        }
    }
}
