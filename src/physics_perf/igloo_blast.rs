use nalgebra::{Point3, UnitQuaternion, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::app::spawnables::IglooDef;
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc};
use crate::rendering::substance::{self, ColliderSubstance};

use super::scenario::PerfScenario;
use crate::perf::Ground;

/// Distance between neighbouring domes when more than one is built.
const DOME_SPACING: f32 = 8.0;
/// Height of the blast above the igloo floor.
const BLAST_HEIGHT: f32 = 0.6;
/// Lift under every block, so none starts the run embedded in the terrain.
const CLEARANCE: f32 = 0.02;

/// Igloos whose every block has just been blown loose.
///
/// ```text
///          ↖  ↑  ↗
///        ╭─┴─┬─┴─╮        every block of the dome, as the separate box
///      ← ┤   ✸   ├ →      body the fracture system spawns it as, sent
///        ╰───┴───╯        outward from a blast inside with random spin
/// ```
///
/// The blocks fly apart, land, and slide on ice-low friction until they
/// sleep: the flight is many independent fast bodies, the landing many
/// independent terrain contacts, and almost none of it is one stack.
pub struct IglooBlast {
    /// How many igloos to blow up at once, laid out in a row.
    pub domes: usize,
    /// Mean outward speed a block leaves the blast with, in m/s.
    pub blast_speed: f32,
    /// Largest spin a block leaves the blast with, in rad/s.
    pub max_spin: f32,
    /// Seed for the per-block jitter, so repeated runs simulate the same thing.
    pub seed: u64,
}

impl Default for IglooBlast {
    fn default() -> Self {
        Self {
            domes: 1,
            blast_speed: 7.0,
            max_spin: 8.0,
            seed: 7,
        }
    }
}

impl IglooBlast {
    fn igloo() -> IglooDef {
        ron::from_str("(pos: (0.0, 0.0, 0.0))").expect("an igloo with every default parses")
    }

    fn dome_offset(&self, dome: usize) -> f32 {
        (dome as f32 - (self.domes as f32 - 1.0) * 0.5) * DOME_SPACING
    }

    fn blocks_per_dome() -> usize {
        Self::igloo().block_boxes().len()
    }
}

impl PerfScenario for IglooBlast {
    fn name(&self) -> &str {
        "igloo_blast"
    }

    fn describe(&self) -> String {
        format!(
            "{} dome(s) × {} blocks, blast {:.1} m/s, spin ≤ {:.1} rad/s, seed {}",
            self.domes,
            Self::blocks_per_dome(),
            self.blast_speed,
            self.max_spin,
            self.seed
        )
    }

    fn populate(&self, world: &mut PhysicsWorld, ground: &Ground) {
        let ice = substance::ICE;
        let blocks = Self::igloo().block_boxes();
        let mut rng = StdRng::seed_from_u64(self.seed);

        for dome in 0..self.domes {
            let floor = ground.surface_point(self.dome_offset(dome), 0.0);
            let blast = floor + Vector3::new(0.0, BLAST_HEIGHT, 0.0);

            for (pose, half_extents) in &blocks {
                let centre = floor + pose.translation.vector + Vector3::new(0.0, CLEARANCE, 0.0);
                let outward = (centre - blast)
                    .try_normalize(1e-6)
                    .unwrap_or_else(Vector3::y);
                let speed = self.blast_speed * rng.gen_range(0.7..1.3);
                let spin = random_direction(&mut rng) * rng.gen_range(0.0..self.max_spin);

                let body = world.create_body(
                    RigidBodyDesc::dynamic()
                        .linear_damping(0.01)
                        .angular_damping(0.005)
                        .position(Point3::from(centre.coords))
                        .rotation(pose.rotation)
                        .linear_velocity(outward * speed)
                        .angular_velocity(spin),
                );
                world.attach_collider(body, ColliderDesc::box_shape(*half_extents).of(&ice));
            }
        }
    }
}

fn random_direction(rng: &mut StdRng) -> Vector3<f32> {
    let rotation = UnitQuaternion::from_euler_angles(
        rng.gen_range(-std::f32::consts::PI..std::f32::consts::PI),
        rng.gen_range(-std::f32::consts::PI..std::f32::consts::PI),
        rng.gen_range(-std::f32::consts::PI..std::f32::consts::PI),
    );
    rotation * Vector3::x()
}
