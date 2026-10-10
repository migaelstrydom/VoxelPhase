//! Where water would collect, and where the water stands once the level has
//! opened: the span graph, its spill levels and the field, reduced to what a
//! plan drawing can show.

use crate::level::Level;
use crate::terrain::TerrainWorld;
use crate::water::field::{SpillLevels, WET_DEPTH};
use crate::water::geometry::{Column, COLUMN_SIZE};
use crate::water::WaterWorld;

/// Still water shallower than this is not drawn, in metres.
const MIN_DEPTH: f32 = 0.05;

/// Water faster than this is drawn as running, m/s.
const RUNNING_SPEED: f32 = 0.3;

/// One column's worth of plan.
#[derive(Debug, Clone, Copy)]
pub struct PlanCell {
    pub column: Column,
    /// Depth of still water if the column were filled to where it spills:
    /// how deep a depression is there. Zero where water runs off.
    pub pooled: f32,
    /// More than one span: an overhang, a bridge, an island over ground.
    pub layered: bool,
    /// No way out to the map's edge: a sealed pocket.
    pub sealed: bool,
}

/// The water-relevant geometry of a level, per column.
pub struct WaterPlan {
    pub cells: Vec<PlanCell>,
    /// Columns under water once the level has opened.
    pub wet: Vec<Column>,
    /// Of those, the columns whose water runs.
    pub running: Vec<Column>,
    /// Width of a cell, in metres.
    pub cell: f32,
    pub parity_repairs: Vec<Column>,
}

impl WaterPlan {
    pub fn from_level(level: &Level, terrain: &TerrainWorld) -> Self {
        let config = level.water.clone().unwrap_or_default();
        let world = WaterWorld::from_config(&config, terrain);
        let field = world.field();
        let (graph, cells) = (field.graph(), field.cells());
        let spill = SpillLevels::find(graph, cells, field.sea().as_ref());
        let (min, max) = graph.column_bounds();
        let mut plan = Self {
            cells: Vec::new(),
            wet: Vec::new(),
            running: Vec::new(),
            cell: COLUMN_SIZE,
            parity_repairs: field.repaired_columns().collect(),
        };
        for k in min.k..=max.k {
            for i in min.i..=max.i {
                let column = Column::new(i, k);
                let spans = graph.spans(column);
                if spans.is_empty() {
                    continue;
                }
                let mut pooled: f32 = 0.0;
                let mut sealed = false;
                let (mut wet, mut running) = (false, false);
                for span in graph.refs(column) {
                    let fill = spill.at(cells, graph, span);
                    sealed |= fill.is_infinite();
                    if fill.is_finite() {
                        pooled = pooled.max(fill - graph.span(span).floor_min);
                    }
                    if let Some(at) = cells.index(graph, span.column, span.ordinal) {
                        if cells.depth(at) > (MIN_DEPTH as f64).max(WET_DEPTH) {
                            wet = true;
                            running |= field.velocity(at).norm() > RUNNING_SPEED;
                        }
                    }
                }
                if wet {
                    plan.wet.push(column);
                }
                if running {
                    plan.running.push(column);
                }
                let pooled = if pooled >= MIN_DEPTH { pooled } else { 0.0 };
                let layered = spans.len() > 1;
                if pooled > 0.0 || layered || sealed {
                    plan.cells.push(PlanCell {
                        column,
                        pooled,
                        layered,
                        sealed,
                    });
                }
            }
        }
        plan
    }
}
