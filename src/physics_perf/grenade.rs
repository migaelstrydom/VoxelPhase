use nalgebra::Vector3;

use crate::explosion::Explosion;
use crate::physics::{PhysicsConfig, PhysicsWorld};

use super::scenario::{Disturbance, PerfScenario};
use crate::perf::Ground;

/// Any scenario, with one of the game's grenades going off in it partway through.
///
/// ```text
///   t = 0            settle / sleep            t = at
///   ├──────────────────────────────────────────✸────── spike, then recovery
/// ```
///
/// The shove is the game's own (`Explosion::physics_impulse`), so a grenade
/// here pushes bodies exactly as one in the game does. Only the physics half
/// goes off: the terrain is not cratered.
pub struct WithGrenade {
    pub scenario: Box<dyn PerfScenario>,
    /// Simulated time the grenade goes off, in seconds.
    pub at: f32,
    /// Where it goes off, relative to the ground surface at the site.
    pub offset: Vector3<f32>,
    /// `<scenario>+grenade`, kept so `name` can lend it out.
    name: String,
}

impl WithGrenade {
    pub fn new(scenario: Box<dyn PerfScenario>, at: f32, offset: Vector3<f32>) -> Self {
        let name = format!("{}+grenade", scenario.name());
        Self {
            scenario,
            at,
            offset,
            name,
        }
    }
}

impl PerfScenario for WithGrenade {
    fn name(&self) -> &str {
        &self.name
    }

    fn describe(&self) -> String {
        format!(
            "{}; grenade at {:.2} s, ({:.1}, {:.1}, {:.1}) from the site",
            self.scenario.describe(),
            self.at,
            self.offset.x,
            self.offset.y,
            self.offset.z
        )
    }

    fn physics_config(&self) -> PhysicsConfig {
        self.scenario.physics_config()
    }

    fn populate(&self, world: &mut PhysicsWorld, ground: &Ground) {
        self.scenario.populate(world, ground);
    }

    fn disturbances(&self, ground: &Ground) -> Vec<Disturbance> {
        let centre = ground.surface_point(0.0, 0.0) + self.offset;
        let mut disturbances = self.scenario.disturbances(ground);
        disturbances.push(Disturbance {
            at: self.at,
            impulse: Explosion::new(centre).physics_impulse(),
        });
        disturbances
    }
}
