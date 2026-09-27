use nalgebra::{Point3, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::physics::{PhysicsConfig, PhysicsWorld};
use crate::projectile::{create_grenade_body, GrenadeConfig};

use super::scenario::{Disturbance, PerfScenario};
use crate::perf::Ground;

/// Widest angle, either side of straight ahead, a thrower stands at.
const ARC_HALF_ANGLE: f32 = std::f32::consts::FRAC_PI_3;
/// Height above the ground a grenade leaves the hand at.
const HAND_HEIGHT: f32 = 1.5;
/// Height above the ground at the site the throws aim for.
const TARGET_HEIGHT: f32 = 0.5;
/// How far either side of the site a throw's aim point strays, in metres.
const AIM_SCATTER: f32 = 1.0;
/// Closest two throwers stand, in metres. Grenades that start touching are
/// clamped at time zero by continuous collision detection and never fly.
const THROWER_SPACING: f32 = 1.0;
/// How much further back each rank of throwers stands than the one before.
const RANK_GAP: f32 = 1.0;

/// Any scenario, with a volley of the game's grenades thrown into it.
///
/// ```text
///                  site
///            ╭─────  ▣  ─────╮        `count` throwers on an arc `range`
///           ╱     ↗  ↑  ↖     ╲       out in front of the site (-Z), each
///          ●       ●  ●       ●       throwing one real grenade body at the
///         ●    ●    ●    ●    ●       game's speed on the low arc that lands
///                                     it; a full arc spills into a rank behind
/// ```
///
/// A thrown grenade is the fastest thing in the game, so this is the load
/// on continuous collision detection: small spheres at 20 m/s sweeping
/// through terrain and whatever the scenario put at the site. The grenades
/// do not go off — they land and stay, as they would with the fuse cut.
pub struct WithVolley {
    pub scenario: Box<dyn PerfScenario>,
    /// How many grenades are thrown, all at once.
    pub count: usize,
    /// Distance from the site to the throwers, in metres.
    pub range: f32,
    /// Seed for the aim points.
    pub seed: u64,
    /// `<scenario>+volley`, kept so `name` can lend it out.
    name: String,
}

impl WithVolley {
    pub fn new(scenario: Box<dyn PerfScenario>, count: usize, range: f32, seed: u64) -> Self {
        let name = format!("{}+volley", scenario.name());
        Self {
            scenario,
            count,
            range,
            seed,
            name,
        }
    }

    /// Where every thrower stands, relative to the site: rank by rank, each
    /// arc filled at no closer than `THROWER_SPACING` before the next begins.
    fn stances(&self) -> Vec<(f32, f32)> {
        let mut stances = Vec::with_capacity(self.count);
        let mut radius = self.range;
        while stances.len() < self.count {
            let fits = (2.0 * ARC_HALF_ANGLE * radius / THROWER_SPACING) as usize + 1;
            let rank = fits.min(self.count - stances.len());
            for index in 0..rank {
                let angle = if rank > 1 {
                    ARC_HALF_ANGLE * (2.0 * index as f32 / (rank - 1) as f32 - 1.0)
                } else {
                    0.0
                };
                stances.push((radius * angle.sin(), -radius * angle.cos()));
            }
            radius += RANK_GAP;
        }
        stances
    }
}

impl PerfScenario for WithVolley {
    fn name(&self) -> &str {
        &self.name
    }

    fn describe(&self) -> String {
        format!(
            "{}; volley of {} grenade(s) at {:.0} m/s from {:.0} m",
            self.scenario.describe(),
            self.count,
            GrenadeConfig::default().throw_speed,
            self.range
        )
    }

    fn physics_config(&self) -> PhysicsConfig {
        self.scenario.physics_config()
    }

    fn populate(&self, world: &mut PhysicsWorld, ground: &Ground) {
        self.scenario.populate(world, ground);

        let config = GrenadeConfig::default();
        let site = ground.surface_point(0.0, 0.0);
        let mut rng = StdRng::seed_from_u64(self.seed);
        for (x, z) in self.stances() {
            let hand = ground.surface_point(x, z) + Vector3::new(0.0, HAND_HEIGHT, 0.0);
            let aim = site
                + Vector3::new(
                    rng.gen_range(-AIM_SCATTER..AIM_SCATTER),
                    TARGET_HEIGHT,
                    rng.gen_range(-AIM_SCATTER..AIM_SCATTER),
                );
            let velocity = low_arc(hand, aim, config.throw_speed, config.gravity);
            create_grenade_body(world, &config, hand, velocity);
        }
    }

    fn disturbances(&self, ground: &Ground) -> Vec<Disturbance> {
        self.scenario.disturbances(ground)
    }
}

/// The launch velocity at `speed` whose flatter arc under `gravity` passes
/// through `to`, or the 45° throw when `to` is out of reach.
fn low_arc(from: Point3<f32>, to: Point3<f32>, speed: f32, gravity: f32) -> Vector3<f32> {
    let offset = to - from;
    let across = Vector3::new(offset.x, 0.0, offset.z);
    let distance = across.norm();
    let heading = across.try_normalize(1e-6).unwrap_or_else(Vector3::z);

    let v2 = speed * speed;
    let discriminant = v2 * v2 - gravity * (gravity * distance * distance + 2.0 * offset.y * v2);
    let elevation = if discriminant >= 0.0 && distance > 1e-6 {
        ((v2 - discriminant.sqrt()) / (gravity * distance)).atan()
    } else {
        std::f32::consts::FRAC_PI_4
    };
    (heading * elevation.cos() + Vector3::y() * elevation.sin()) * speed
}
