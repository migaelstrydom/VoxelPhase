//! Runs a scenario: builds its terrain, sets off its blasts, audits after each.

use nalgebra::Point3;

use super::scenario::{Blast, Scenario};
use crate::explosion::Explosion;
use crate::rubble::{Cut, Outcome, Plan, RubblePlanner, ScreeRules};
use crate::terrain::{TerrainWorld, VoxelMaterial};

/// The frame rate scree is flown at, as in the game.
const FRAME_SECONDS: f32 = 1.0 / 60.0;

/// One fragment a blast cut loose, as the report needs it.
#[derive(Debug, Clone, Copy)]
pub struct FragmentRecord {
    pub samples: usize,
    pub volume: f32,
    pub centroid: Point3<f32>,
    pub material: VoxelMaterial,
    pub fate: Fate,
}

/// What became of a fragment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Fate {
    /// Crumbled where it broke.
    Dust,
    /// Fell as scree and crumbled where it landed, `frames` later, `drop`
    /// metres below where it broke.
    Landed { frames: usize, drop: f32 },
    /// Fell as scree and never landed: out of time, or out of the world.
    Expired { frames: usize },
}

/// What becomes of `cut` in the game, played out against `terrain`.
fn fate_of(cut: &Cut, planner: &mut RubblePlanner, terrain: &TerrainWorld) -> Fate {
    let Plan::Scree { mut flight, .. } = planner.plan(cut) else {
        return Fate::Dust;
    };
    let rules = ScreeRules::default();
    let floor_y = terrain.bounds().min.y - rules.floor_margin;
    let start = flight.position;
    for frames in 1.. {
        match flight.step(FRAME_SECONDS, &rules, terrain, floor_y) {
            Outcome::Flying => {}
            Outcome::Landed(at) => {
                return Fate::Landed {
                    frames,
                    drop: start.y - at.y,
                };
            }
            Outcome::Expired => return Fate::Expired { frames },
        }
    }
    unreachable!("a flight ends within its lifetime")
}

/// What one blast did.
#[derive(Debug, Clone)]
pub struct BlastRecord {
    pub blast: Blast,
    pub fragments: Vec<FragmentRecord>,
    /// Samples standing free anywhere in the terrain after this blast.
    pub loose: usize,
    /// Samples drawn paper-thin anywhere in the terrain after this blast.
    pub paper_thin: usize,
}

/// A scenario played out.
#[derive(Debug, Clone)]
pub struct Run {
    pub scenario: &'static str,
    /// Samples standing free in the terrain as authored: floating islands.
    pub loose_before: usize,
    /// Samples drawn paper-thin in the terrain as authored.
    pub paper_thin_before: usize,
    pub blasts: Vec<BlastRecord>,
    /// Open mesh edges once the terrain has remeshed after the last blast.
    pub open_edges: usize,
}

impl Run {
    /// Every fragment cut loose, in order.
    pub fn fragments(&self) -> impl Iterator<Item = &FragmentRecord> {
        self.blasts.iter().flat_map(|b| b.fragments.iter())
    }

    /// Breaches of what every run is held to: no blast leaves more terrain
    /// standing free, or more drawn paper-thin, than there was before it; and
    /// every piece of scree falls clear of where it broke and lands. One that
    /// lands on its first frame started inside the ground, and one that never
    /// lands fell through it.
    pub fn violations(&self) -> Vec<String> {
        let mut found = Vec::new();
        for (index, record) in self.blasts.iter().enumerate() {
            for fragment in &record.fragments {
                let problem = match fragment.fate {
                    Fate::Landed { frames: 1, .. } => "landed on its first frame",
                    Fate::Expired { .. } => "never landed",
                    _ => continue,
                };
                found.push(format!(
                    "blast {index}: scree of {} samples at {:?} {problem}",
                    fragment.samples, fragment.centroid
                ));
            }
        }
        let (mut loose, mut thin) = (self.loose_before, self.paper_thin_before);
        for (index, record) in self.blasts.iter().enumerate() {
            if record.loose > loose {
                found.push(format!(
                    "blast {index} at {:?} left {} samples standing free, up from {loose}",
                    record.blast.centre, record.loose
                ));
            }
            if record.paper_thin > thin {
                found.push(format!(
                    "blast {index} at {:?} left {} samples paper-thin, up from {thin}",
                    record.blast.centre, record.paper_thin
                ));
            }
            (loose, thin) = (record.loose, record.paper_thin);
        }
        if self.open_edges > 0 {
            found.push(format!(
                "{} open mesh edges after the last blast",
                self.open_edges
            ));
        }
        found
    }
}

/// Play `scenario` and record what each blast did.
pub fn run(scenario: &Scenario) -> Result<Run, String> {
    let mut terrain = scenario.terrain()?;
    terrain.update();
    let loose_before = terrain.loose_samples();
    let paper_thin_before = terrain.paper_thin_samples();

    let mut planner = RubblePlanner::default();
    let blasts = scenario
        .blasts
        .iter()
        .map(|&blast| {
            let shove = Explosion::new(blast.centre).physics_impulse();
            let fragments = terrain.detonate(blast.centre, &blast.charge);
            // Scree flies against the terrain as remeshed after the blast.
            terrain.update();
            let fragments = fragments
                .into_iter()
                .map(|fragment| {
                    let cut = Cut { fragment, shove };
                    let fate = fate_of(&cut, &mut planner, &terrain);
                    let fragment = &cut.fragment;
                    FragmentRecord {
                        samples: fragment.sample_count(),
                        volume: fragment.volume(),
                        centroid: fragment.world_centroid(),
                        material: fragment.material(),
                        fate,
                    }
                })
                .collect();
            BlastRecord {
                blast,
                fragments,
                loose: terrain.loose_samples(),
                paper_thin: terrain.paper_thin_samples(),
            }
        })
        .collect();
    terrain.update();

    Ok(Run {
        scenario: scenario.name,
        loose_before,
        paper_thin_before,
        blasts,
        open_edges: terrain.open_edge_count(),
    })
}
