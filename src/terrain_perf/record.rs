use std::time::Duration;

use nalgebra::Point3;

use crate::terrain::UpdateTimings;

/// One charge set off and the rebuild it caused.
#[derive(Debug, Clone)]
pub struct BlastRecord {
    /// Where the charge went off.
    pub site: Point3<f32>,
    /// Wall clock of `detonate` plus `update`, measured outside the terrain.
    pub wall: Duration,
    /// The terrain's own breakdown. `None` when the blast removed nothing, so
    /// there was no rebuild to time.
    pub timings: Option<UpdateTimings>,
}

impl BlastRecord {
    pub fn chunks_dirtied(&self) -> usize {
        self.timings.map_or(0, |t| t.chunks_dirtied)
    }

    /// Time the terrain's own breakdown does not account for.
    pub fn unaccounted(&self) -> Duration {
        let accounted = self.timings.map_or(Duration::ZERO, |t| t.total());
        self.wall.saturating_sub(accounted)
    }
}

/// Every blast of one sweep, each the median of the repeats.
#[derive(Debug, Clone)]
pub struct TerrainRun {
    /// Which level, for the report header.
    pub level: String,
    /// One line on the charge used.
    pub charge: String,
    /// Triangles in the level before the first blast.
    pub triangles_before: usize,
    /// Triangles after the last.
    pub triangles_after: usize,
    /// Open mesh edges after the last blast; should stay at the level's own
    /// count.
    pub open_edges_after: usize,
    /// How many runs each blast's numbers are the median of.
    pub repeats: usize,
    /// Hash of the rebuilt terrain; see `terrain_fingerprint`.
    pub fingerprint: u64,
    /// Whether every repeat produced the same fingerprint.
    pub deterministic: bool,
    pub blasts: Vec<BlastRecord>,
}
