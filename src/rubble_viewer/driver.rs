//! Runs a scenario: builds its terrain, sets off its blasts, audits after each.

use nalgebra::Point3;

use super::scenario::{Blast, Scenario};
use crate::terrain::{Fragment, VoxelMaterial};

/// One fragment a blast cut loose, as the report needs it.
#[derive(Debug, Clone, Copy)]
pub struct FragmentRecord {
    pub samples: usize,
    pub volume: f32,
    pub centroid: Point3<f32>,
    pub material: VoxelMaterial,
}

impl From<&Fragment> for FragmentRecord {
    fn from(fragment: &Fragment) -> Self {
        Self {
            samples: fragment.sample_count(),
            volume: fragment.volume(),
            centroid: fragment.world_centroid(),
            material: fragment.material(),
        }
    }
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
    /// standing free, or more drawn paper-thin, than there was before it.
    pub fn violations(&self) -> Vec<String> {
        let mut found = Vec::new();
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

    let blasts = scenario
        .blasts
        .iter()
        .map(|&blast| {
            let fragments = terrain.detonate(blast.centre, &blast.charge);
            BlastRecord {
                blast,
                fragments: fragments.iter().map(FragmentRecord::from).collect(),
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
