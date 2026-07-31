//! Checks that things are where the author meant them to be.
//!
//! All three checks reduce to two questions about a point: is it inside rock,
//! and is there ground under it. The answers come from the meshed terrain, not
//! the voxel grid, so they agree with what the player will collide with.

use nalgebra::Point3;

use crate::level::{Level, ObjectPlacement, Orientable};
use crate::terrain::{TerrainWorld, SURFACE_BAND};

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
pub fn check_player_spawn(level: &Level, terrain: &TerrainWorld, report: &mut Report) {
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
pub fn check_objects(level: &Level, terrain: &TerrainWorld, report: &mut Report) {
    for (index, (segment, object)) in level.objects().enumerate() {
        let info = object.describe();
        let ObjectPlacement::Free(pos) = info.placement else {
            continue;
        };
        let kind = info.kind;
        let at = format!(
            "{kind} #{} in '{}' at ({:.1}, {:.1}, {:.1})",
            index + 1,
            level.segments[segment].name,
            pos.x,
            pos.y,
            pos.z
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
fn is_buried(terrain: &TerrainWorld, p: Point3<f32>) -> bool {
    let margin = terrain.voxel_size() * BURIAL_MARGIN_VOXELS;
    terrain.is_mesh_solid_at(p.x, p.y, p.z) && terrain.is_mesh_solid_at(p.x, p.y + margin, p.z)
}

/// Height of the highest upward-facing terrain surface below a point.
///
/// "Below" is generous by one surface band, because meshing nudges a surface
/// that lands exactly on a lattice plane onto the solid side of it (see
/// `terrain::csg::SURFACE_BAND`). An object authored to rest at `y = 0` on
/// ground authored at `y = 0` therefore sits a fraction of a voxel *under* the
/// meshed surface, and a strict `h <= p.y` would report solid ground as a void.
/// Authoring around that epsilon is the wrong way round — the tolerance belongs
/// here, in the check.
fn surface_below(terrain: &TerrainWorld, p: Point3<f32>) -> Option<f32> {
    let tolerance = terrain.voxel_size() * SURFACE_BAND;
    terrain
        .mesh_surface_heights_at(p.x, p.z)
        .into_iter()
        .find(|h| *h <= p.y + tolerance)
}

/// Warn about objects that keep their world orientation while their segment
/// turns around them.
///
/// The trap this makes visible: an object is authored segment-locally, so an
/// author reasonably expects a wall drawn along the segment's `+X` to still run
/// along it after the segment is placed at a quarter turn. For the spawnables
/// that do not yet carry a yaw, it does not — and nothing else in the pipeline
/// would say so.
pub fn check_object_orientation(level: &Level, report: &mut Report) {
    for (index, object) in level.objects() {
        let frame = level.frame(index);
        if frame.quarter_turns() == 0 {
            continue;
        }
        if object.orientability() != Orientable::Fixed {
            continue;
        }
        report.warn(
            "objects",
            format!(
                "{} in segment '{}' has a meaningful horizontal axis but no yaw, so it keeps \
                 its world orientation while the segment turns {:.0}° around it; author it in \
                 an unrotated segment until it gains one",
                object.describe().kind,
                level.segments[index].name,
                frame.yaw_degrees(),
            ),
        );
    }
}
