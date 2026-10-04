//! Handing fragments from the blast that cut them to what they become.
//!
//! ```text
//!   ExplosionSystem ──TerrainWorld::detonate──▶ fragments + the blast's shove
//!                                                   │ RubbleQueue
//!   RubbleSpawnSystem ◀─────────────────────────────┘
//!     RubblePlanner::plan(cut), one blast at a time
//!       GradeRules::grade(Measure::of(fragment))
//!       ├── Dust    ──▶ a crumble emitter at its centroid
//!       ├── Scree   ──▶ FallingScree + TerrainMeshInstance, thrown by the
//!       │               blast's shove
//!       └── Boulder ──▶ BrickShaper ──▶ a body in the physics world
//!                       + TerrainMeshInstance + Debris; the shove queued
//!                       for the next physics step throws it. One too
//!                       intricate for one body is Fragment::split into
//!                       parts, each graded and planned in turn. Past
//!                       `max_boulders` in one blast, the smallest fall as
//!                       scree.
//! ```

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use specs::{Builder, Entities, LazyUpdate, Read, System, Write};

use super::boulder::Boulder;
use super::brick_shaper::{BrickShaper, Shape};
use super::dust::{Crumble, CrumbleSize};
use super::grade::{Grade, GradeRules, Measure};
use super::scree::{FallingScree, Flight};
use crate::components::{Orientation, Position, RigidBodyComponent, TerrainMeshInstance, Velocity};
use crate::fracture::Debris;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::PhysicsImpulse;
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;
use crate::terrain::{Fragment, FragmentMesh};

/// What one blast cut loose.
pub struct Cut {
    /// The shove the blast gave everything around it.
    pub shove: PhysicsImpulse,
    pub fragments: Vec<Fragment>,
}

/// Blasts this frame, waiting for what they cut loose to become rubble.
#[derive(Default)]
pub struct RubbleQueue {
    pending: Vec<Cut>,
}

impl RubbleQueue {
    /// Queue what one blast, which shoved its surroundings with `shove`, cut
    /// loose.
    pub fn push_blast(&mut self, shove: PhysicsImpulse, fragments: Vec<Fragment>) {
        if !fragments.is_empty() {
            self.pending.push(Cut { shove, fragments });
        }
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
    /// The fastest a piece's furthest point turns as it breaks away, m/s,
    /// which slows the spin of a long one. A slab tumbling at a pebble's
    /// rate reads as weightless, and a boulder's end would swing through
    /// the gap it starts with from the ground in its first frame.
    pub max_edge_speed: f32,
    /// The fastest a piece leaves the blast, m/s. A sliver weighs little, and
    /// the shove that sends a barrel a few metres would send it off the map.
    pub max_speed: f32,
}

impl Default for Launch {
    fn default() -> Self {
        Self {
            min_spin: 0.5,
            max_spin: 3.0,
            max_edge_speed: 1.5,
            max_speed: 12.0,
        }
    }
}

impl Launch {
    /// How a piece of `mass` kg with its centroid at `origin` and its
    /// furthest point `reach` metres from it leaves the blast: its velocity
    /// and its spin.
    fn of(
        &self,
        shove: &PhysicsImpulse,
        mass: f32,
        origin: Point3<f32>,
        reach: f32,
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
        let spin = rng
            .gen_range(self.min_spin..self.max_spin)
            .min(self.max_edge_speed / reach.max(f32::EPSILON));
        (velocity, axis * spin)
    }
}

/// Seed for the spin of everything cut loose: a blast replays the same.
const SPIN_SEED: u64 = 0x5c4ee;

/// A piece of rubble: a fragment, or a part of one, and what it becomes.
pub struct Piece {
    pub fragment: Fragment,
    /// Which of the cut's fragments it is, or is a part of.
    pub source: usize,
    pub plan: Plan,
}

/// What one piece becomes.
pub enum Plan {
    /// A crumble of `size` at `at`.
    Dust { at: Point3<f32>, size: CrumbleSize },
    /// Scree on `flight`, drawn with `mesh`.
    Scree {
        flight: Flight,
        mesh: FragmentMesh,
        size: CrumbleSize,
    },
    /// A body.
    Boulder(Boulder),
}

/// Decides what each fragment becomes and how it leaves the blast. Shared by
/// the game's spawn system and the offline harness, so both see the same
/// rubble.
pub struct RubblePlanner {
    pub grades: GradeRules,
    pub launch: Launch,
    pub bricks: BrickShaper,
    /// Most boulders one blast makes. A blast through a honeycomb can cut
    /// dozens of pieces loose, and each body costs the solver every frame it
    /// moves; the smallest past the cap fall as scree.
    pub max_boulders: usize,
    rng: StdRng,
}

impl Default for RubblePlanner {
    fn default() -> Self {
        Self {
            grades: GradeRules::default(),
            launch: Launch::default(),
            bricks: BrickShaper::default(),
            max_boulders: 8,
            rng: StdRng::seed_from_u64(SPIN_SEED),
        }
    }
}

impl RubblePlanner {
    /// What `cut`'s fragments become: one piece each, or several for a
    /// boulder too intricate for one body, cut where its convex pieces meet.
    pub fn plan(&mut self, cut: Cut) -> Vec<Piece> {
        let mut grades: Vec<Grade> = cut
            .fragments
            .iter()
            .map(|f| self.grades.grade(Measure::of(f)))
            .collect();
        let mut boulders: Vec<usize> = (0..grades.len())
            .filter(|&i| grades[i] == Grade::Boulder)
            .collect();
        boulders.sort_by(|&a, &b| {
            cut.fragments[b]
                .volume()
                .total_cmp(&cut.fragments[a].volume())
        });
        for &i in boulders.iter().skip(self.max_boulders) {
            grades[i] = Grade::Scree;
        }
        let mut pieces = Vec::new();
        for (source, (fragment, grade)) in cut.fragments.into_iter().zip(grades).enumerate() {
            self.plan_piece(fragment, source, grade, &cut.shove, &mut pieces);
        }
        pieces
    }

    /// What `fragment`, graded `grade`, becomes, onto `pieces`.
    fn plan_piece(
        &mut self,
        fragment: Fragment,
        source: usize,
        grade: Grade,
        shove: &PhysicsImpulse,
        pieces: &mut Vec<Piece>,
    ) {
        let size = CrumbleSize::of(&fragment);
        let dust = Plan::Dust {
            at: fragment.world_centroid(),
            size,
        };
        if grade == Grade::Dust {
            pieces.push(Piece {
                fragment,
                source,
                plan: dust,
            });
            return;
        }
        let mesh = fragment.mesh();
        if mesh.indices.is_empty() {
            pieces.push(Piece {
                fragment,
                source,
                plan: dust,
            });
            return;
        }
        let bricks = if grade == Grade::Boulder {
            let surface = mesh.vertices.iter().map(|v| mesh.origin + v.pos);
            match self.bricks.shape(&fragment.occupancy(), surface) {
                Shape::Whole(bricks) => Some(bricks),
                Shape::Parts(parts) => {
                    for part in fragment.split(&parts) {
                        let grade = self.grades.grade(Measure::of(&part));
                        self.plan_piece(part, source, grade, shove, pieces);
                    }
                    return;
                }
            }
        } else {
            None
        };

        let mass = fragment.volume() * fragment.material().mass_density();
        let reach = mesh
            .vertices
            .iter()
            .map(|v| v.pos.norm())
            .fold(0.0, f32::max);
        let (velocity, spin) = self
            .launch
            .of(shove, mass, mesh.origin, reach, &mut self.rng);
        let plan = match bricks {
            Some(bricks) => Plan::Boulder(Boulder {
                bricks,
                mesh,
                mass,
                spin,
                material: fragment.material(),
                volume: fragment.volume(),
                size,
            }),
            None => Plan::Scree {
                flight: Flight::new(
                    mesh.origin,
                    velocity,
                    spin,
                    mesh.vertices.iter().map(|v| v.pos),
                ),
                mesh,
                size,
            },
        };
        pieces.push(Piece {
            fragment,
            source,
            plan,
        });
    }
}

/// Turns each queued fragment into rubble.
#[derive(Default)]
pub struct RubbleSpawnSystem {
    planner: RubblePlanner,
    crumble: Crumble,
}

impl<'a> System<'a> for RubbleSpawnSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, LazyUpdate>,
        Write<'a, RubbleQueue>,
        Write<'a, PhysicsResource>,
    );

    fn run(&mut self, (entities, lazy, mut queue, mut physics): Self::SystemData) {
        for cut in queue.drain() {
            for piece in self.planner.plan(cut) {
                match piece.plan {
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
                    Plan::Boulder(boulder) => {
                        let volume = boulder.volume;
                        let placed = boulder.place(&mut physics.world);
                        let entity = lazy
                            .create_entity(&entities)
                            .with(Position(placed.centre.coords))
                            .with(Velocity(Vector3::zeros()))
                            .with(Orientation::default())
                            .with(RigidBodyComponent(placed.body))
                            .with(TerrainMeshInstance {
                                anchor: placed.centre.coords,
                                model: model_of(placed.mesh),
                            })
                            .build();
                        // A boulder came off no object, so it is its own
                        // origin: only the budget's overall cap applies.
                        lazy.insert(entity, Debris::new(entity, volume));
                    }
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
