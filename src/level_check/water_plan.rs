//! Where water would collect: the span graph and drainage field, reduced to
//! what a plan drawing can show.

use crate::terrain::TerrainWorld;
use crate::water::geometry::{Column, Drain, Outlets, WaterGeometry, COLUMN_SIZE};

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

/// The water-relevant geometry of a level, per column.
pub struct WaterPlan {
    pub cells: Vec<PlanCell>,
    /// Width of a cell, in metres.
    pub cell: f32,
    pub parity_repairs: Vec<Column>,
}

impl WaterPlan {
    pub fn from_terrain(terrain: &TerrainWorld) -> Self {
        let geometry = WaterGeometry::build(terrain, Outlets::default());
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
            cell: COLUMN_SIZE,
            parity_repairs: geometry.repaired_columns(),
        }
    }
}
