//! The geometry layer as one unit: spans, the rasteriser that keeps them
//! current, and the drainage field over them.
//!
//! ```text
//!   TerrainWorld ──build──▶ SpanRasteriser ──▶ SpanGraph ──▶ DrainageField
//!        │ rebuilt chunks,       │ re-pair          │ remap        │ repair
//!        └─ changed regions ─────┘                  └──────────────┘
//! ```
//!
//! The network above sees only the graph, the field and each update's remap.

use std::time::{Duration, Instant};

use crate::terrain::TerrainWorld;

use super::drainage::{DrainageField, Outlets, RepairStats};
use super::rasteriser::{RasterStats, RebuildTimings, SpanRasteriser, SpanRemap};
use super::span_graph::SpanGraph;

/// What one terrain update did to the geometry.
#[derive(Debug, Clone)]
pub struct GeometryUpdate {
    /// Where every re-paired span went. Owners are already rewritten.
    pub remap: SpanRemap,
    pub repair: RepairStats,
    pub rebuild: RebuildTimings,
    /// Time spent repairing the drainage field.
    pub drainage: Duration,
}

/// Spans and drainage over a level's terrain, kept current as it is blown
/// apart.
pub struct WaterGeometry {
    rasteriser: SpanRasteriser,
    graph: SpanGraph,
    drainage: DrainageField,
}

impl WaterGeometry {
    /// Rasterise and flood the whole level.
    pub fn build(terrain: &TerrainWorld, outlets: Outlets) -> Self {
        let (rasteriser, graph) = SpanRasteriser::build(terrain);
        let drainage = DrainageField::build(&graph, outlets);
        Self {
            rasteriser,
            graph,
            drainage,
        }
    }

    /// Catch up with the terrain's most recent update. `None` when it rebuilt
    /// nothing.
    pub fn update(&mut self, terrain: &TerrainWorld) -> Option<GeometryUpdate> {
        let rebuilt = terrain.rebuilt_chunks();
        if rebuilt.is_empty() {
            return None;
        }
        let remap =
            self.rasteriser
                .rebuild(terrain, &mut self.graph, rebuilt, terrain.changed_regions());
        let started = Instant::now();
        let repair = self.drainage.repair(&self.graph, &remap);
        Some(GeometryUpdate {
            remap,
            repair,
            rebuild: self.rasteriser.last_rebuild(),
            drainage: started.elapsed(),
        })
    }

    pub fn graph(&self) -> &SpanGraph {
        &self.graph
    }

    /// The graph, for writing span owners.
    pub fn graph_mut(&mut self) -> &mut SpanGraph {
        &mut self.graph
    }

    pub fn drainage(&self) -> &DrainageField {
        &self.drainage
    }

    /// The field, for resolving pending flats while routing.
    pub fn drainage_mut(&mut self) -> &mut DrainageField {
        &mut self.drainage
    }

    /// Graph and field together, for routing: resolving a flat reads the one
    /// and writes the other.
    pub fn routing(&mut self) -> (&SpanGraph, &mut DrainageField) {
        (&self.graph, &mut self.drainage)
    }

    pub fn stats(&self) -> RasterStats {
        self.rasteriser.stats()
    }

    /// Columns whose spans needed a parity repair.
    pub fn repaired_columns(&self) -> Vec<super::span::Column> {
        self.rasteriser.repaired_columns().collect()
    }
}
