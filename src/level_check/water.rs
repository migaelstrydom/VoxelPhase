//! Where each authored body of water actually ends up.
//!
//! A `Pool` is a seed and a surface level. Its basin is flooded from the seed
//! over the span graph, exactly as the game does it, and stops at every crest:
//! a pool can no longer escape its basin, but it can be authored above where
//! it spills, and then the game opens by draining it to the lip. Nothing else
//! in the pipeline says so, so it is reported here with the lip's location.

use crate::level::Level;
use crate::terrain::TerrainWorld;
use crate::water::network::CrestKind;
use crate::water::WaterWorld;

use super::report::{Report, Section};

/// Place every body and report where each went.
///
/// Returns `None` for a level without water, so the section is simply absent
/// rather than empty.
pub fn check_water(level: &Level, terrain: &TerrainWorld, report: &mut Report) -> Option<Section> {
    let config = level.water.as_ref()?;
    let mut section = Section::new("Water");

    if let Some(level_y) = config.ocean_level {
        section.row("Ocean", format!("surface {level_y:.1}"));
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
        if let Some(crest) = spill.filter(|c| c.saddle < level) {
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
