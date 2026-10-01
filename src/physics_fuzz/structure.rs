//! What a fuzz case knocks down: one of the bench harness's structures, with
//! its proportions drawn from the seed.
//!
//! The scenarios are used only to build bodies. Every case stands on the
//! same wide floor, so a structure that spreads as it falls never runs off
//! the edge of its own scenario's quad.

use std::fmt;

use rand::rngs::StdRng;
use rand::Rng;

use crate::physics::bench_harness::framework::PhysicsBenchScenario;
use crate::physics::bench_harness::scenarios::{
    BoxGridScenario, HoneycombWallScenario, JengaTowerScenario, TempleScenario,
    VoussoirArchScenario,
};
use crate::physics::{PhysicsWorld, RigidBodyHandle};

/// The kinds of structure a case can be built around.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StructureKind {
    Jenga,
    Arch,
    Temple,
    HoneycombWall,
    BoxPile,
}

impl StructureKind {
    pub const ALL: [StructureKind; 5] = [
        StructureKind::Jenga,
        StructureKind::Arch,
        StructureKind::Temple,
        StructureKind::HoneycombWall,
        StructureKind::BoxPile,
    ];

    pub fn name(self) -> &'static str {
        match self {
            StructureKind::Jenga => "jenga",
            StructureKind::Arch => "arch",
            StructureKind::Temple => "temple",
            StructureKind::HoneycombWall => "wall",
            StructureKind::BoxPile => "pile",
        }
    }

    pub fn parse(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|kind| kind.name() == name)
    }
}

/// A structure, as drawn from a seed.
pub enum Structure {
    Jenga(JengaTowerScenario),
    Arch(VoussoirArchScenario),
    Temple(TempleScenario),
    HoneycombWall(HoneycombWallScenario),
    BoxPile(BoxGridScenario),
}

impl Structure {
    /// Draw a structure of `kind` with proportions from `rng`.
    pub fn generate(kind: StructureKind, rng: &mut StdRng) -> Self {
        match kind {
            StructureKind::Jenga => {
                let mut jenga = JengaTowerScenario::new(rng.gen_range(6..=18));
                jenga.friction = rng.gen_range(0.3..0.8);
                Structure::Jenga(jenga)
            }
            StructureKind::Arch => {
                let mut arch = VoussoirArchScenario::new();
                arch.num_voussoirs = rng.gen_range(5..=13) | 1;
                arch.friction = rng.gen_range(0.5..0.9);
                Structure::Arch(arch)
            }
            StructureKind::Temple => {
                let mut temple = TempleScenario::new();
                temple.front_columns = rng.gen_range(2..=4);
                temple.side_columns = rng.gen_range(2..=4);
                Structure::Temple(temple)
            }
            StructureKind::HoneycombWall => {
                let mut wall = HoneycombWallScenario::new();
                wall.columns = rng.gen_range(3..=7);
                wall.rows = rng.gen_range(3..=7);
                Structure::HoneycombWall(wall)
            }
            StructureKind::BoxPile => {
                Structure::BoxPile(BoxGridScenario::new(rng.gen_range(2..=5)))
            }
        }
    }

    /// Build the structure's bodies into `world` and return them.
    pub fn build(&self, world: &mut PhysicsWorld) -> Vec<RigidBodyHandle> {
        self.scenario().setup(world);
        world
            .bodies()
            .iter()
            .filter(|(_, body)| body.is_dynamic())
            .map(|(index, _)| RigidBodyHandle(index))
            .collect()
    }

    fn scenario(&self) -> &dyn PhysicsBenchScenario {
        match self {
            Structure::Jenga(s) => s,
            Structure::Arch(s) => s,
            Structure::Temple(s) => s,
            Structure::HoneycombWall(s) => s,
            Structure::BoxPile(s) => s,
        }
    }
}

impl fmt::Display for Structure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Structure::Jenga(s) => write!(f, "jenga {} layers μ {:.2}", s.layers, s.friction),
            Structure::Arch(s) => {
                write!(f, "arch {} voussoirs μ {:.2}", s.num_voussoirs, s.friction)
            }
            Structure::Temple(s) => {
                write!(f, "temple {}×{} columns", s.front_columns, s.side_columns)
            }
            Structure::HoneycombWall(s) => write!(f, "wall {}×{}", s.columns, s.rows),
            Structure::BoxPile(s) => write!(f, "pile {0}×{0}", s.grid_size),
        }
    }
}
