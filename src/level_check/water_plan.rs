//! Where water would collect, and where the authored water is: the span
//! graph, the drainage field and the level's basins, reduced to what a plan
//! drawing can show.

use crate::level::Level;
use crate::terrain::TerrainWorld;
use crate::water::geometry::{Column, Drain, COLUMN_SIZE};
use crate::water::network::CrestKind;
use crate::water::WaterWorld;

/// Still water shallower than this is not drawn, in metres.
const MIN_DEPTH: f32 = 0.05;

/// One column's worth of plan.
#[derive(Debug, Clone, Copy)]
pub struct PlanCell {
    pub column: Column,
    /// Depth of still water at steady state if the column were filled to its
    /// fill level: how deep a depression is there. Zero where water runs off.
    pub pooled: f32,
    /// More than one span: an overhang, a bridge, an island over ground.
    pub layered: bool,
    /// No path to any outlet: a sealed pocket.
    pub sealed: bool,
}

/// One cell of a crest, for drawing.
#[derive(Debug, Clone, Copy)]
pub struct PlanCrest {
    pub column: Column,
    pub saddle: f32,
    /// Drains away (an outlet) rather than into a depression of its own.
    pub outlet: bool,
}

/// The water-relevant geometry of a level, per column.
pub struct WaterPlan {
    pub cells: Vec<PlanCell>,
    /// Channels the level's first frame routes: each reach's centreline in
    /// plan, (x, z).
    pub channels: Vec<Vec<(f32, f32)>>,
    /// Columns under the authored water.
    pub wet: Vec<Column>,
    pub crests: Vec<PlanCrest>,
    /// Width of a cell, in metres.
    pub cell: f32,
    pub parity_repairs: Vec<Column>,
}

impl WaterPlan {
    pub fn from_level(level: &Level, terrain: &TerrainWorld) -> Self {
        let config = level.water.clone().unwrap_or_default();
        let (mut water, _) = WaterWorld::from_config(&config, terrain);
        // One frame, for the outflows that link at once to lay their channels.
        water.step(1.0 / 60.0);
        let channels = water
            .network()
            .stores()
            .filter_map(|(_, s)| s.as_reach())
            .map(|r| r.centreline.points.iter().map(|p| (p.x, p.z)).collect())
            .collect();
        let mut wet = Vec::new();
        let mut crests = Vec::new();
        for (_, basin) in water.basins() {
            let surface = basin.level();
            wet.extend(
                basin
                    .region
                    .iter()
                    .filter(|r| r.shape.floor_min < surface)
                    .map(|r| r.span.column),
            );
            crests.extend(basin.crests.iter().map(|c| PlanCrest {
                column: c.inside.column,
                saddle: c.saddle,
                outlet: c.kind == CrestKind::Outlet,
            }));
        }
        wet.sort_unstable();
        wet.dedup();

        let geometry = water.geometry();
        let graph = geometry.graph();
        let drainage = geometry.drainage();
        let (min, max) = graph.column_bounds();
        let mut cells = Vec::new();
        for k in min.k..=max.k {
            for i in min.i..=max.i {
                let column = Column::new(i, k);
                let spans = graph.spans(column);
                if spans.is_empty() {
                    continue;
                }
                let mut pooled: f32 = 0.0;
                let mut sealed = false;
                for span in graph.refs(column) {
                    let fill = drainage.fill(graph, span);
                    sealed |= drainage.drain(graph, span) == Drain::Sealed;
                    if fill.is_finite() {
                        pooled = pooled.max(fill - graph.span(span).floor_min);
                    }
                }
                let pooled = if pooled >= MIN_DEPTH { pooled } else { 0.0 };
                let layered = spans.len() > 1;
                if pooled > 0.0 || layered || sealed {
                    cells.push(PlanCell {
                        column,
                        pooled,
                        layered,
                        sealed,
                    });
                }
            }
        }
        Self {
            cells,
            channels,
            wet,
            crests,
            cell: COLUMN_SIZE,
            parity_repairs: geometry.repaired_columns(),
        }
    }
}
