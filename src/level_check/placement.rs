//! Checks that things are where the author meant them to be.
//!
//! All three checks reduce to two questions about a point: is it inside rock,
//! and is there ground under it. The answers come from the meshed terrain, not
//! the voxel grid, so they agree with what the player will collide with.

use nalgebra::Point3;

use crate::level::{Level, ObjectPlacement};
use crate::terrain::TerrainManager;

use super::report::Report;

/// How far the player may spawn above ground before it is treated as a mistake.
///
/// There is no fall-damage system, so this is not about injury: it flags a
/// spawn hanging over a void or over terrain that was never generated, which is
/// the actual failure mode. Generous, because a small drop onto ground is a
/// normal way to author a spawn.
pub const SPAWN_DROP_LIMIT: f32 = 20.0;

/// How far an object may sit above ground before it is worth a warning.
///
/// Dropping objects onto terrain is legitimate, so this is deliberately larger
/// than the spawn limit and only ever a warning.
pub const OBJECT_DROP_LIMIT: f32 = 50.0;

/// Fraction of a voxel a point must be below the surface to count as buried.
///
/// Objects are routinely authored resting exactly on the surface, so "the
/// centre is solid" alone would flag half a level. Requiring rock above the
/// point as well distinguishes buried from resting, and — because the point
/// itself must be solid too — leaves an object sitting inside a cave alone.
const BURIAL_MARGIN_VOXELS: f32 = 0.5;

/// Check the player spawn: not inside rock, and with ground beneath it.
pub fn check_player_spawn(level: &Level, terrain: &TerrainManager, report: &mut Report) {
    let (x, y, z) = level.player_spawn;
    let spawn = Point3::new(x, y, z);

    if is_buried(terrain, spawn) {
        report.error(
            "spawn",
            format!("player_spawn ({x}, {y}, {z}) is inside solid terrain"),
        );
    }

    match surface_below(terrain, spawn) {
        Some(surface) => {
            let drop = y - surface;
            if drop > SPAWN_DROP_LIMIT {
                report.error(
                    "spawn",
                    format!(
                        "player_spawn ({x}, {y}, {z}) is {drop:.1} m above the terrain below it \
                         (limit {SPAWN_DROP_LIMIT:.0} m)"
                    ),
                );
            }
        }
        None => report.error(
            "spawn",
            format!("player_spawn ({x}, {y}, {z}) has no terrain beneath it"),
        ),
    }
}

/// Check every authored object's placement.
///
/// Terrain-anchored objects are skipped: their height is resolved from the
/// surface at spawn time, so it cannot be authored wrongly.
pub fn check_objects(level: &Level, terrain: &TerrainManager, report: &mut Report) {
    for (index, object) in level.objects.iter().enumerate() {
        let info = object.describe();
        let ObjectPlacement::Free(pos) = info.placement else {
            continue;
        };
        let kind = info.kind;
        let at = format!(
            "{kind} #{index} at ({:.1}, {:.1}, {:.1})",
            pos.x, pos.y, pos.z
        );

        if is_buried(terrain, pos) {
            report.error("objects", format!("{at} is inside solid terrain"));
            continue;
        }

        match surface_below(terrain, pos) {
            Some(surface) => {
                let drop = pos.y - surface;
                if drop > OBJECT_DROP_LIMIT {
                    report.warn(
                        "objects",
                        format!("{at} is {drop:.1} m above the terrain below it"),
                    );
                }
            }
            None => report.warn("objects", format!("{at} has no terrain beneath it")),
        }
    }
}

/// Whether a point is meaningfully inside rock rather than resting on it.
fn is_buried(terrain: &TerrainManager, p: Point3<f32>) -> bool {
    let margin = terrain.voxel_size() * BURIAL_MARGIN_VOXELS;
    terrain.is_mesh_solid_at(p.x, p.y, p.z) && terrain.is_mesh_solid_at(p.x, p.y + margin, p.z)
}

/// Height of the highest upward-facing terrain surface below a point.
fn surface_below(terrain: &TerrainManager, p: Point3<f32>) -> Option<f32> {
    terrain
        .mesh_surface_heights_at(p.x, p.z)
        .into_iter()
        .find(|h| *h <= p.y)
}
