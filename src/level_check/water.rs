//! Where each authored body of water actually ends up.
//!
//! A `Pool` is a seed and a surface level. Its basin is flooded from the seed
//! over the span graph, exactly as the game does it, and stops at every crest:
//! a pool can no longer escape its basin, but it can be authored above where
//! it spills, and then the game opens by draining it to the lip. Nothing else
//! in the pipeline says so, so it is reported here with the lip's location.

use crate::level::Level;
use crate::terrain::TerrainWorld;
use crate::water::network::{CrestKind, Store};
use crate::water::WaterWorld;

use super::report::{Report, Section};

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

    let (world, errors) = WaterWorld::from_config(config, terrain);
    for error in errors {
        report.warn("water", error.to_string());
    }

    for (id, basin) in world.basins() {
        let level = basin.level();
        let spill = basin
            .crests
            .iter()
            .find(|c| c.kind == CrestKind::Outlet)
            .copied();
        let spill_text = match spill {
            Some(c) => {
                let (x, z) = c.inside.column.centre();
                format!("spills at {:.2} by ({x:.1}, {z:.1})", c.saddle)
            }
            None => "no outlet".to_string(),
        };
        section.row(
            format!("Basin {}", id.0),
            format!(
                "surface {level:.2} · {:.0} m³ · {} spans · {} · {} merge saddles",
                basin.volume,
                basin.region.len(),
                spill_text,
                basin.merge_saddles.len()
            ),
        );
        // A basin a source feeds stands over its outlet by design.
        let fed = world
            .network()
            .links()
            .any(|(link, l)| l.down == id && world.link_discharge(link).is_some_and(|q| q > 0.0));
        if let Some(crest) = spill.filter(|c| c.saddle < level && !fed) {
            let (x, z) = crest.inside.column.centre();
            let keep = basin.hypsometry.volume(crest.saddle);
            report.error(
                "water",
                format!(
                    "basin {}: authored at {level:.2}, above its outlet at {:.2} by ({x:.1}, {z:.1}); \
                     the level opens by draining {:.0} m³ over it. Raise the lip or lower the surface.",
                    id.0,
                    crest.saddle,
                    basin.volume - keep,
                ),
            );
        }
    }

    for (i, source) in world.sources().iter().enumerate() {
        let p = source.position;
        let name = format!("Source {i}");
        if source.buried {
            report.error(
                "water",
                format!(
                    "source at ({:.1}, {:.1}, {:.1}) is sealed in rock: no air within reach \
                     along its direction, so nothing flows",
                    p.x, p.y, p.z
                ),
            );
            section.row(name, "sealed in rock".to_string());
            continue;
        }
        let lands = source
            .link
            .and_then(|l| world.network().link(l))
            .and_then(|l| l.fall.as_ref())
            .and_then(|f| f.landing());
        let text = match lands {
            Some(at) => format!(
                "{:.2} m³/s from ({:.1}, {:.1}, {:.1}), lands at ({:.1}, {:.1}, {:.1})",
                source.discharge, p.x, p.y, p.z, at.x, at.y, at.z
            ),
            None => format!(
                "{:.2} m³/s from ({:.1}, {:.1}, {:.1})",
                source.discharge, p.x, p.y, p.z
            ),
        };
        section.row(name, text);
    }
    // With a sea, a closed edge needs scenery: water should not run off it.
    if config.ocean.is_some() {
        let lost = world
            .network()
            .links()
            .filter(|(_, l)| matches!(world.network().store(l.down), Some(Store::Sink)))
            .filter_map(|(link, _)| world.link_discharge(link))
            .sum::<f64>();
        if lost > 0.0 {
            report.warn(
                "water",
                format!(
                    "{lost:.2} m³/s runs off a closed edge of the map at rest; \
                     give that edge scenery or open it onto the sea"
                ),
            );
        }
    }
    if let Some(steady) = world.steady_report() {
        if !steady.converged {
            report.warn(
                "water",
                format!(
                    "the water did not come to rest in {} sweeps; the level opens still settling",
                    steady.sweeps
                ),
            );
        }
    }

    let repaired = world.geometry().repaired_columns();
    if !repaired.is_empty() {
        let (x, z) = repaired[0].centre();
        report.error(
            "water",
            format!(
                "{} columns needed a parity repair, the first at ({x:.2}, {z:.2}): the terrain mesh \
                 has an open edge there",
                repaired.len()
            ),
        );
    }

    let balance = world.balance();
    if !balance.is_balanced() {
        report.error(
            "water",
            format!("the water ledger does not balance at load: {balance}"),
        );
    }

    section.note(
        "Each pool is flooded over the span graph from its seed and stops at every crest. A \
         pool above its lowest outlet drains to it on the first frame.",
    );
    Some(section)
}
