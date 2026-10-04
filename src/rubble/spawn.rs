//! Handing fragments from the blast that cut them to what they become.
//!
//! ```text
//!   ExplosionSystem ──TerrainWorld::detonate──▶ fragments + the blast's shove
//!                                                   │ RubbleQueue
//!   RubbleSpawnSystem ◀─────────────────────────────┘
//!     GradeRules::grade(Measure::of(fragment))
//!       ├── Dust    ──▶ a crumble emitter at its centroid
//!       └── Scree   ──▶ FallingScree + TerrainMeshInstance, thrown by the
//!           Boulder      blast's shove (boulders fly as scree until they
//!                        become bodies, Phase 3)
//! ```

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use specs::{Builder, Entities, LazyUpdate, Read, System, Write};

use super::dust::{Crumble, CrumbleSize};
use super::grade::{Grade, GradeRules, Measure};
use super::scree::{FallingScree, Flight};
use crate::components::{Orientation, Position, TerrainMeshInstance};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::PhysicsImpulse;
use crate::rendering::material::MaterialId;
use crate::terrain::{Fragment, FragmentMesh};

/// A fragment and the blast that cut it loose.
pub struct Cut {
    pub fragment: Fragment,
    /// The shove the blast gave everything around it.
    pub shove: PhysicsImpulse,
}

/// Fragments cut loose this frame, waiting to be turned into rubble.
#[derive(Default)]
pub struct RubbleQueue {
    pending: Vec<Cut>,
}

impl RubbleQueue {
    /// Queue what one blast, which shoved its surroundings with `shove`, cut
    /// loose.
    pub fn push_blast(
        &mut self,
        shove: PhysicsImpulse,
        fragments: impl IntoIterator<Item = Fragment>,
    ) {
        self.pending.extend(
            fragments
                .into_iter()
                .map(|fragment| Cut { fragment, shove }),
        );
    }

    pub fn drain(&mut self) -> std::vec::Drain<'_, Cut> {
        self.pending.drain(..)
    }
}

/// How a fragment is set flying.
#[derive(Debug, Clone, Copy)]
pub struct Launch {
    /// Bounds on how fast a piece spins as it breaks away, rad/s. A random
    /// tumble, so that a shelf does not fall as a flat card.
    pub min_spin: f32,
    pub max_spin: f32,
    /// The fastest a piece leaves the blast, m/s. A sliver weighs little, and
    /// the shove that sends a barrel a few metres would send it off the map.
    pub max_speed: f32,
}

impl Default for Launch {
    fn default() -> Self {
        Self {
            min_spin: 0.5,
            max_spin: 3.0,
            max_speed: 12.0,
        }
    }
}

impl Launch {
    /// How `fragment`, of `mass` kg with its centroid at `origin`, leaves the
    /// blast: its velocity and its spin.
    fn of(
        &self,
        shove: &PhysicsImpulse,
        mass: f32,
        origin: Point3<f32>,
        rng: &mut impl Rng,
    ) -> (Vector3<f32>, Vector3<f32>) {
        let velocity = shove
            .impulse_at(origin)
            .map_or(Vector3::zeros(), |impulse| impulse / mass.max(f32::EPSILON))
            .cap_magnitude(self.max_speed);
        let axis = Vector3::new(
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        )
        .try_normalize(1e-3)
        .unwrap_or(Vector3::y());
        (velocity, axis * rng.gen_range(self.min_spin..self.max_spin))
    }
}

/// Seed for the spin of everything cut loose: a blast replays the same.
const SPIN_SEED: u64 = 0x5c4ee;

/// What one fragment becomes.
pub enum Plan {
    /// A crumble of `size` at `at`.
    Dust { at: Point3<f32>, size: CrumbleSize },
    /// Scree on `flight`, drawn with `mesh`.
    Scree {
        flight: Flight,
        mesh: FragmentMesh,
        size: CrumbleSize,
    },
}

/// Decides what each fragment becomes and how it leaves the blast. Shared by
/// the game's spawn system and the offline harness, so both see the same
/// rubble.
pub struct RubblePlanner {
    pub grades: GradeRules,
    pub launch: Launch,
    rng: StdRng,
}

impl Default for RubblePlanner {
    fn default() -> Self {
        Self {
            grades: GradeRules::default(),
            launch: Launch::default(),
            rng: StdRng::seed_from_u64(SPIN_SEED),
        }
    }
}

impl RubblePlanner {
    /// What `cut` becomes. Boulders fly as scree until they become bodies.
    pub fn plan(&mut self, cut: &Cut) -> Plan {
        let fragment = &cut.fragment;
        let size = CrumbleSize::of(fragment);
        let dust = || Plan::Dust {
            at: fragment.world_centroid(),
            size,
        };
        match self.grades.grade(Measure::of(fragment)) {
            Grade::Dust => dust(),
            Grade::Scree | Grade::Boulder => {
                let mesh = fragment.mesh();
                if mesh.indices.is_empty() {
                    return dust();
                }
                let mass = fragment.volume() * fragment.material().mass_density();
                let (velocity, spin) = self.launch.of(&cut.shove, mass, mesh.origin, &mut self.rng);
                let flight = Flight::new(
                    mesh.origin,
                    velocity,
                    spin,
                    mesh.vertices.iter().map(|v| v.pos),
                );
                Plan::Scree { flight, mesh, size }
            }
        }
    }
}

/// Turns each queued fragment into rubble.
#[derive(Default)]
pub struct RubbleSpawnSystem {
    planner: RubblePlanner,
    crumble: Crumble,
}

impl<'a> System<'a> for RubbleSpawnSystem {
    type SystemData = (Entities<'a>, Read<'a, LazyUpdate>, Write<'a, RubbleQueue>);

    fn run(&mut self, (entities, lazy, mut queue): Self::SystemData) {
        for cut in queue.drain() {
            match self.planner.plan(&cut) {
                Plan::Dust { at, size } => {
                    lazy.create_entity(&entities)
                        .with(Position(at.coords))
                        .with(self.crumble.emitter(size))
                        .build();
                }
                Plan::Scree { flight, mesh, size } => {
                    lazy.create_entity(&entities)
                        .with(Position(mesh.origin.coords))
                        .with(Orientation::default())
                        .with(TerrainMeshInstance {
                            anchor: mesh.origin.coords,
                            model: model_of(mesh),
                        })
                        .with(FallingScree { flight, size })
                        .build();
                }
            }
        }
    }
}

/// A fragment's mesh as a one-primitive model.
fn model_of(mesh: FragmentMesh) -> Arc<Model> {
    Arc::new(Model::flat(vec![ModelPart::new(vec![MeshPrimitive {
        vertices: mesh.vertices,
        indices: mesh.indices,
        // Not read: a terrain mesh is drawn with the terrain's surface.
        material: MaterialId(0),
    }])]))
}
