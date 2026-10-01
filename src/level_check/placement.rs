//! Checks that things are where the author meant them to be.
//!
//! All three checks reduce to two questions about a point: is it inside rock,
//! and is there ground under it. The answers come from the meshed terrain, not
//! the voxel grid, so they agree with what the player will collide with.

use nalgebra::Point3;

use crate::level::{Footprint, Level, ObjectPlacement, Orientable, Support};
use crate::terrain::TerrainWorld;

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

/// How far above a point the ground may still count as being "below" it, as a
/// fraction of a voxel.
///
/// A surface authored exactly on a lattice plane is meshed a `SURFACE_BAND`
/// fraction of a voxel proud of where it was authored, and the bands *stack*:
/// a plinth unioned onto a carved floor is debiased once per write, so the
/// offset at a given column is some small multiple of the band that no caller
/// can predict. Expressing the tolerance as a multiple of the band therefore
/// puts the commonest case there is — an object authored flush with the ground
/// it stands on — permanently on a knife edge, resolved by float rounding.
///
/// A tenth of a voxel is an order of magnitude clear of any plausible stack and
/// still three orders below the thing this check exists to catch, which is an
/// object hanging over a void.
const SURFACE_TOLERANCE_VOXELS: f32 = 0.1;

/// How far the ground may step across an object's footprint before it is worth
/// a warning, in metres.
///
/// A rigid object sits at one height, so ground that falls away under part of
/// it leaves that part in the air by however far it fell. Terrain roughness
/// moves a surface by a few tens of centimetres and beds an object in rather
/// than lifting it, which is normal authoring and must stay quiet. Half a metre
/// is above that and well below the two-metre bench drop that started this: it
/// is the height at which "it is sitting on uneven ground" turns into "one end
/// of it is floating".
const FOOTPRINT_STEP_LIMIT: f32 = 0.5;

/// What share of an object's footprint has to read solid before the object is
/// called embedded in the terrain.
///
/// An object genuinely driven into a rise has a whole side of it in the rock,
/// which is a quarter of a square grid of samples or a quadrant of a disc. One
/// or two isolated points is a different thing, and measurably so: on a mesh
/// with open edges, a column whose surface is emitted *twice* inverts the ray
/// parity test and reads solid all the way up. Sampling a crate standing in an
/// open cave found exactly two such points among sixteen, both of them the two
/// with a doubled surface height, while every neighbour at the same height read
/// clear.
///
/// So the threshold is not a fudge factor for a noisy check — it is the
/// difference between a shape intersecting terrain and the mesh being locally
/// broken, and the second is what the open-edge count is for.
const BURIED_FOOTPRINT_SHARE: f32 = 0.25;

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
/// surface at spawn time, so it cannot be authored wrongly. A dropped object's
/// height is derived too, and what it lands on may be another object, which
/// the terrain cannot speak for: only where its fall starts is checked here,
/// and the rest trial judges where it lands.
pub fn check_objects(level: &Level, terrain: &TerrainWorld, report: &mut Report) {
    for (index, (segment, object)) in level.objects().enumerate() {
        let info = object.describe();
        let (pos, dropped) = match info.placement {
            ObjectPlacement::Free(pos) => (pos, false),
            ObjectPlacement::Dropped(pos) => (pos, true),
            ObjectPlacement::TerrainAnchored { .. } => continue,
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
        if dropped {
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

        check_footprint(terrain, pos, info.footprint, info.support, &at, report);
    }
}

/// Check the ground across everything the object covers, not just under the
/// point it was authored at.
///
/// The failure this exists for: an object long enough to cross a terrain
/// feature is authored at one end, that end is over good ground, and the check
/// passes while most of the object hangs in the air. A twelve-block domino row
/// authored on one quarry bench and running onto the next reads as perfect and
/// leaves eight blocks two metres up.
///
/// It reports the *spread* of ground under the footprint rather than the gap to
/// the object's base, which means it needs no view on where an object's bottom
/// is — a question the placement point does not answer, since some objects are
/// authored at their base and others at their centre.
fn check_footprint(
    terrain: &TerrainWorld,
    pos: Point3<f32>,
    footprint: Footprint,
    support: Support,
    at: &str,
    report: &mut Report,
) {
    if support == Support::Spanning {
        return;
    }

    let step = terrain.voxel_size_at(pos);
    if !footprint.spans_more_than(step) {
        return;
    }

    let mut lowest = f32::MAX;
    let mut highest = f32::MIN;
    let mut unsupported = 0;
    let mut buried = 0;

    let samples = footprint.samples((pos.x, pos.z), step);
    for (x, z) in &samples {
        let p = Point3::new(*x, pos.y, *z);
        if is_buried(terrain, p) {
            buried += 1;
            continue;
        }
        match surface_below(terrain, p) {
            Some(h) => {
                lowest = lowest.min(h);
                highest = highest.max(h);
            }
            None => unsupported += 1,
        }
    }

    // Warnings rather than errors throughout, and deliberately: a footprint is
    // an approximation of the object's shape, sampled coarsely. The authored
    // point is exact and keeps its errors; what is inferred from a disc laid
    // over a dolos should not be able to fail a level on its own.
    let total = samples.len();
    if buried as f32 >= total as f32 * BURIED_FOOTPRINT_SHARE {
        report.warn(
            "objects",
            format!("{at} runs into solid terrain across {buried} of the {total} points it covers"),
        );
    }
    if unsupported > 0 {
        report.warn(
            "objects",
            format!("{at} has no terrain beneath {unsupported} of the {total} points it covers"),
        );
    }
    if lowest <= highest && highest - lowest > FOOTPRINT_STEP_LIMIT {
        report.warn(
            "objects",
            format!(
                "{at} covers ground that steps by {:.1} m ({:.1} m to {:.1} m), so part of it \
                 will not be resting on anything",
                highest - lowest,
                lowest,
                highest
            ),
        );
    }
}

/// Whether a point is meaningfully inside rock rather than resting on it.
fn is_buried(terrain: &TerrainWorld, p: Point3<f32>) -> bool {
    let margin = terrain.voxel_size_at(p) * BURIAL_MARGIN_VOXELS;
    terrain.is_mesh_solid_at(p.x, p.y, p.z) && terrain.is_mesh_solid_at(p.x, p.y + margin, p.z)
}

/// Height of the highest upward-facing terrain surface below a point.
///
/// "Below" is generous by a fraction of a voxel, because meshing nudges a
/// surface that lands exactly on a lattice plane onto the solid side of it (see
/// `terrain::csg::SURFACE_BAND`). An object authored to rest at `y = 0` on
/// ground authored at `y = 0` therefore sits a fraction of a voxel *under* the
/// meshed surface, and a strict `h <= p.y` would report solid ground as a void.
/// Authoring around that epsilon is the wrong way round — the tolerance belongs
/// here, in the check.
///
/// The offset is a fraction of the resolution of the segment the object is *in*,
/// so the tolerance has to be too. Taking the world's finest instead makes it
/// too small everywhere but the finest segment, and an object resting on a
/// coarse bench is then reported as floating over a void.
fn surface_below(terrain: &TerrainWorld, p: Point3<f32>) -> Option<f32> {
    let tolerance = terrain.voxel_size_at(p) * SURFACE_TOLERANCE_VOXELS;
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
