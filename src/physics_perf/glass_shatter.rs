use std::sync::Arc;

use nalgebra::{Point3, Vector2, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::app::spawnables::GlassSheetDef;
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc};
use crate::rendering::substance::{self, ColliderSubstance};

use super::scenario::PerfScenario;
use crate::perf::Ground;

/// Gap between neighbouring panes' edges when more than one is built.
const PANE_GAP: f32 = 1.0;
/// How far in front of the pane the blow came from; shards fly away from it.
const BLOW_STANDOFF: f32 = 0.5;
/// Lift under every pane, so no shard starts the run embedded in the terrain.
const CLEARANCE: f32 = 0.02;

/// Windows whose every shard has just been knocked out.
///
/// ```text
///      ┌───────┬──────┐          the game's own pane, crazed around a hit
///      │ ╲  │ ╱  ╲    │          into its Voronoi web — each cell a thin
///      │──✸───┼──     │   ──▶    convex hull prism — and every shard sent
///      │ ╱  │ ╲  ╱    │          off away from the blow with random spin
///      └───────┴──────┘
/// ```
///
/// The igloo blast is boxes; this is the hulls fracture actually produces.
/// Thin hull against terrain and hull against hull take the general
/// collision path, not the box fast paths. In the game a hit only frees the
/// shards within its hole and the rest stay in the pane; here all of them go,
/// the worst a pane can do in one frame.
pub struct GlassShatter {
    /// How many panes to shatter at once, standing in a row.
    pub panes: usize,
    /// Mean speed a shard leaves the blow with, in m/s.
    pub blast_speed: f32,
    /// Largest spin a shard leaves the blow with, in rad/s.
    pub max_spin: f32,
    /// Seed for the hit points, the crack layouts and the per-shard jitter.
    pub seed: u64,
}

impl Default for GlassShatter {
    fn default() -> Self {
        Self {
            panes: 1,
            blast_speed: 6.0,
            max_spin: 10.0,
            seed: 7,
        }
    }
}

impl GlassShatter {
    /// The game's default window: upright, facing `+Z`.
    fn pane() -> GlassSheetDef {
        ron::from_str("(pos: (0.0, 0.0, 0.0))").expect("a glass sheet with every default parses")
    }

    fn pane_offset(&self, pane: usize) -> f32 {
        let spacing = Self::pane().size.0 + PANE_GAP;
        (pane as f32 - (self.panes as f32 - 1.0) * 0.5) * spacing
    }

    /// Centre of pane `pane`, standing with its bottom edge just clear of
    /// the highest ground under it.
    fn pane_centre(&self, pane: usize, ground: &Ground) -> Point3<f32> {
        let (width, height) = Self::pane().size;
        let x = self.pane_offset(pane);
        let bottom = [x - width * 0.5, x, x + width * 0.5]
            .map(|x| ground.surface_point(x, 0.0))
            .into_iter()
            .max_by(|a, b| a.y.total_cmp(&b.y))
            .expect("three samples");
        bottom + Vector3::new(0.0, height * 0.5 + CLEARANCE, 0.0)
    }
}

impl PerfScenario for GlassShatter {
    fn name(&self) -> &str {
        "glass_shatter"
    }

    fn describe(&self) -> String {
        let pane = Self::pane();
        format!(
            "{} pane(s) of {:.1} × {:.1} m, {:.0} mm thick, blow {:.1} m/s, spin ≤ {:.1} rad/s, seed {}",
            self.panes,
            pane.size.0,
            pane.size.1,
            pane.thickness * 1000.0,
            self.blast_speed,
            self.max_spin,
            self.seed
        )
    }

    fn populate(&self, world: &mut PhysicsWorld, ground: &Ground) {
        let glass = substance::GLASS;
        let pane = Self::pane();
        let (half_width, half_height) = (pane.size.0 * 0.5, pane.size.1 * 0.5);
        let mut rng = StdRng::seed_from_u64(self.seed);

        for index in 0..self.panes {
            let centre = self.pane_centre(index, ground);
            let hit = Vector2::new(
                rng.gen_range(-half_width..half_width) * 0.5,
                rng.gen_range(-half_height..half_height) * 0.5,
            );
            let blow = centre + Vector3::new(hit.x, hit.y, -BLOW_STANDOFF);

            for (hull, offset) in pane.shards(hit, rng.gen()) {
                let position = centre + offset;
                let away = (position - blow)
                    .try_normalize(1e-6)
                    .unwrap_or_else(Vector3::z);
                let speed = self.blast_speed * rng.gen_range(0.7..1.3);
                let spin = random_unit(&mut rng) * rng.gen_range(0.0..self.max_spin);

                let body = world.create_body(
                    RigidBodyDesc::dynamic()
                        .linear_damping(0.01)
                        .angular_damping(0.005)
                        .position(position)
                        .linear_velocity(away * speed)
                        .angular_velocity(spin),
                );
                world.attach_collider(body, ColliderDesc::convex_hull(Arc::new(hull)).of(&glass));
            }
        }
    }
}

fn random_unit(rng: &mut StdRng) -> Vector3<f32> {
    loop {
        let candidate = Vector3::new(
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        );
        if let Some(unit) = candidate
            .try_normalize(1e-3)
            .filter(|_| candidate.norm() <= 1.0)
        {
            return unit;
        }
    }
}
