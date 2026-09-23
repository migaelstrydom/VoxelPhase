use std::time::Instant;

use nalgebra::Point3;

use crate::perf::Ground;
use crate::terrain::{BlastConfig, TerrainWorld};

use super::fingerprint::terrain_fingerprint;
use super::record::{BlastRecord, TerrainRun};

/// The charge to set off and how often to repeat the sweep.
#[derive(Debug, Clone)]
pub struct RunConfig {
    pub blast: BlastConfig,
    /// One line naming the charge, for the report.
    pub charge: String,
    /// Sweeps to run, each on freshly built terrain; each blast reports the
    /// median of them, which keeps one scheduler hiccup out of the numbers.
    pub repeats: usize,
}

/// Set off one charge at each of `sites` in turn, as the game would — a
/// `detonate` followed by the frame's `update` — and time both.
pub fn run(ground: &Ground, sites: &[Point3<f32>], config: &RunConfig) -> TerrainRun {
    let repeats = config.repeats.max(1);
    let takes: Vec<Take> = (0..repeats)
        .map(|_| run_once(ground, sites, &config.blast))
        .collect();
    let fingerprint = takes[0].fingerprint;
    let deterministic = takes.iter().all(|take| take.fingerprint == fingerprint);

    TerrainRun {
        level: ground.level_name().to_string(),
        charge: config.charge.clone(),
        triangles_before: ground.terrain().triangle_count(),
        triangles_after: takes[0].triangles,
        open_edges_after: takes[0].open_edges,
        repeats,
        fingerprint,
        deterministic,
        blasts: median_blasts(takes.into_iter().map(|take| take.blasts).collect()),
    }
}

/// One sweep, before the repeats are folded together.
struct Take {
    blasts: Vec<BlastRecord>,
    fingerprint: u64,
    triangles: usize,
    open_edges: usize,
}

fn run_once(ground: &Ground, sites: &[Point3<f32>], blast: &BlastConfig) -> Take {
    let mut terrain = ground.fresh_terrain();
    let blasts = sites
        .iter()
        .map(|&site| detonate_and_rebuild(&mut terrain, site, blast))
        .collect();

    Take {
        blasts,
        fingerprint: terrain_fingerprint(&terrain),
        triangles: terrain.triangle_count(),
        open_edges: terrain.open_edge_count(),
    }
}

fn detonate_and_rebuild(
    terrain: &mut TerrainWorld,
    site: Point3<f32>,
    blast: &BlastConfig,
) -> BlastRecord {
    let started = Instant::now();
    terrain.detonate(site, blast);
    terrain.update();
    let wall = started.elapsed();

    // `last_update_timings` outlives idle updates, so only a rebuild this
    // update reported through `dirty_regions` belongs to this blast.
    let rebuilt = !terrain.dirty_regions().is_empty();
    BlastRecord {
        site,
        wall,
        timings: rebuilt.then(|| terrain.last_update_timings()).flatten(),
    }
}

/// Blast by blast, the take whose wall clock is the median.
///
/// A whole record is picked rather than a median per field, so its stages
/// still add up to its total. Every take set off the same charges on the same
/// terrain, so only the clock differs between them.
fn median_blasts(takes: Vec<Vec<BlastRecord>>) -> Vec<BlastRecord> {
    let count = takes.iter().map(Vec::len).min().unwrap_or(0);
    (0..count)
        .map(|index| {
            let mut same_blast: Vec<&BlastRecord> = takes.iter().map(|take| &take[index]).collect();
            same_blast.sort_by_key(|record| record.wall);
            same_blast[same_blast.len() / 2].clone()
        })
        .collect()
}
