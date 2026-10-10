//! Where each authored body of water actually ends up.
//!
//! A `Pool` is a seed and a surface level. The field fills its hollow from
//! the seed, but never above where the hollow spills: water above that would
//! run off, so the level opens with the pool at its lip instead. Nothing else
//! in the pipeline says so, so it is reported here, with where each pool
//! stands once the level has opened.

use nalgebra::Point3;

use crate::level::{Level, WaterBody};
use crate::terrain::TerrainWorld;
use crate::water::field::{pool_seed, SpillLevels, WET_DEPTH};
use crate::water::WaterWorld;

use super::report::{Report, Section};

/// Farthest below its authored surface a pool may open, m.
const POOL_TOLERANCE: f32 = 0.05;

/// Place every body and report where each went.
///
/// Returns `None` for a level without water, so the section is simply absent
/// rather than empty.
pub fn check_water(level: &Level, terrain: &TerrainWorld, report: &mut Report) -> Option<Section> {
    let config = level.water.as_ref()?;
    let mut section = Section::new("Water");

    if let Some(ocean) = &config.ocean {
        section.row(
            "Ocean",
            format!(
                "sea level {:.1}, open to {:?}",
                ocean.level, ocean.open_edges
            ),
        );
    }

    let world = WaterWorld::from_config(config, terrain);
    let field = world.field();
    let (graph, cells) = (field.graph(), field.cells());
    let spill = SpillLevels::find(graph, cells, field.sea().as_ref());
    let (settled, took) = world.opened();
    section.row(
        "Opening",
        format!(
            "{:.0} m³ after {settled:.0} s settled, in {:.2} s",
            world.volume(),
            took.as_secs_f64()
        ),
    );
    if world.still_settling() {
        report.warn(
            "water",
            format!(
                "the water did not come to rest in {settled:.0} s; the level opens still settling"
            ),
        );
    }

    let pools = config.bodies.iter().filter_map(|body| match body {
        WaterBody::Pool {
            seed,
            surface_level,
        } => Some((*seed, *surface_level)),
        _ => None,
    });
    for (i, ((x, z), authored)) in pools.enumerate() {
        let name = format!("Pool {i}");
        let Some(span) = pool_seed(graph, x, z, authored) else {
            report.error(
                "water",
                format!("pool at ({x:.1}, {z:.1}) has no ground under {authored:.2} to fill"),
            );
            section.row(name, "no ground to fill".to_string());
            continue;
        };
        let spills = spill.at(cells, graph, span);
        let stands = cells
            .index(graph, span.column, span.ordinal)
            .filter(|&at| cells.depth(at) > WET_DEPTH)
            .map(|at| cells.surface(at) as f32);
        let spill_text = if spills.is_finite() {
            format!("spills at {spills:.2}")
        } else {
            "sealed".to_string()
        };
        let stands_text = stands.map_or("dry".to_string(), |s| format!("stands at {s:.2}"));
        section.row(
            name,
            format!("authored {authored:.2} at ({x:.1}, {z:.1}) · {spill_text} · {stands_text}"),
        );
        if stands.is_none_or(|s| s < authored - POOL_TOLERANCE) {
            report.error(
                "water",
                format!(
                    "pool at ({x:.1}, {z:.1}): authored at {authored:.2}, opens {stands_text} \
                     ({spill_text}). Raise the lip or lower the surface."
                ),
            );
        }
    }

    for (i, source) in field.sources().iter().enumerate() {
        let p = source.origin;
        let name = format!("Source {i}");
        let lands = source.landing.first().map(|&at| {
            let chunk = &cells.chunks[at.slot as usize];
            let (local, _) = chunk.column_of(at.flat as usize);
            let (lx, lz) = chunk.coord.column(local).centre();
            Point3::new(lx, cells.bed(at), lz)
        });
        let text = match lands {
            Some(at) => format!(
                "{:.2} m³/s from ({:.1}, {:.1}, {:.1}), lands at ({:.1}, {:.1}, {:.1})",
                source.discharge, p.x, p.y, p.z, at.x, at.y, at.z
            ),
            None => {
                report.warn(
                    "water",
                    format!(
                        "source at ({:.1}, {:.1}, {:.1}) falls out of the world",
                        p.x, p.y, p.z
                    ),
                );
                format!(
                    "{:.2} m³/s from ({:.1}, {:.1}, {:.1}), lands nowhere",
                    source.discharge, p.x, p.y, p.z
                )
            }
        };
        section.row(name, text);
    }

    let repaired: Vec<_> = field.repaired_columns().collect();
    if let Some(first) = repaired.first() {
        let (x, z) = first.centre();
        report.error(
            "water",
            format!(
                "{} columns needed a parity repair, the first at ({x:.2}, {z:.2}): the terrain mesh \
                 has an open edge there",
                repaired.len()
            ),
        );
    }

    let volume = world.volume();
    let error = field.ledger().error(volume);
    if error.abs() > 1.0e-6 * volume.max(1.0) {
        report.error(
            "water",
            format!("the water ledger is out by {error:+.3e} m³ at load"),
        );
    }

    section.note(
        "Each pool fills its hollow from its seed, no higher than where the hollow spills; \
         the level then settles as it opens. A pool authored above its spill opens at it.",
    );
    Some(section)
}
