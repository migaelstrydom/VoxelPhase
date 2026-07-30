//! Schematic export: a level as a picture, without a graphics device.
//!
//! ```text
//!   ┌──────────────────────────────┐   top-down (xz): surface height as a
//!   │  plan          ▓▓▒▒░░        │   shaded heightmap, objects as markers,
//!   │                ▓▓▓▒▒         │   chunk lattice as a faint grid
//!   ├──────────────────────────────┤
//!   │  elevation    ▁▂▄▆█▆▄▂▁      │   (xy): the vertical envelope of the
//!   └──────────────────────────────┘   terrain, projected along z
//! ```
//!
//! The audience is someone who cannot run the game and wants to know whether a
//! level is roughly what they intended, so legibility beats fidelity: labelled
//! axes, a scale bar and a legend matter more than shading quality.

use std::fmt::Write as _;
use std::path::Path;

use crate::level::{Level, ObjectPlacement};
use crate::terrain::TerrainManager;

/// Target number of heightmap samples along the longest horizontal axis.
///
/// Sampling is a voxel-column walk per cell, and every cell becomes SVG markup,
/// so this trades render time and file size against detail.
const SAMPLES_ACROSS: f32 = 200.0;

/// Distinct shades in the height ramp. Coarse enough that flat ground collapses
/// into long runs, which is what keeps the file small.
const SHADE_STEPS: usize = 24;

/// Pixels per world metre is chosen so the plan view fits this width.
const PLAN_WIDTH_PX: f32 = 900.0;

/// Height of the elevation panel, in pixels.
const ELEVATION_HEIGHT_PX: f32 = 260.0;

const MARGIN: f32 = 60.0;
const PANEL_GAP: f32 = 70.0;

/// A terrain surface height sampled on a regular xz lattice.
///
/// `None` where the column holds no solid voxel at all — void, not ground at
/// height zero, and the difference matters when reading the picture.
struct HeightField {
    /// Sample spacing in world metres.
    cell: f32,
    /// Number of samples along x and z.
    nx: usize,
    nz: usize,
    /// World position of sample (0, 0).
    origin_x: f32,
    origin_z: f32,
    /// Row-major, `nz` rows of `nx`.
    heights: Vec<Option<f32>>,
    /// Range of the sampled heights, for the colour ramp.
    min: f32,
    max: f32,
}

impl HeightField {
    fn sample(terrain: &TerrainManager) -> Self {
        let bounds = terrain.bounds();
        let size = bounds.size();

        let cell = (size.x.max(size.z) / SAMPLES_ACROSS).max(terrain.voxel_size());
        let nx = (size.x / cell).ceil() as usize + 1;
        let nz = (size.z / cell).ceil() as usize + 1;

        let mut heights = Vec::with_capacity(nx * nz);
        let (mut min, mut max) = (f32::INFINITY, f32::NEG_INFINITY);

        for iz in 0..nz {
            let z = bounds.min.z + iz as f32 * cell;
            for ix in 0..nx {
                let x = bounds.min.x + ix as f32 * cell;
                let h = terrain.approx_surface_height_at(x, z);
                if let Some(h) = h {
                    min = min.min(h);
                    max = max.max(h);
                }
                heights.push(h);
            }
        }

        if !min.is_finite() {
            min = 0.0;
            max = 1.0;
        }

        Self {
            cell,
            nx,
            nz,
            origin_x: bounds.min.x,
            origin_z: bounds.min.z,
            heights,
            min,
            max: max.max(min + 1e-3),
        }
    }

    fn at(&self, ix: usize, iz: usize) -> Option<f32> {
        self.heights[iz * self.nx + ix]
    }

    /// Quantised shade index, or `None` for void.
    fn shade(&self, ix: usize, iz: usize) -> Option<usize> {
        let h = self.at(ix, iz)?;
        let t = (h - self.min) / (self.max - self.min);
        Some(((t * (SHADE_STEPS - 1) as f32).round() as usize).min(SHADE_STEPS - 1))
    }

    /// Per-x-column vertical envelope: the lowest and highest surface height
    /// found anywhere along z. Reads as the level's vertical structure.
    fn elevation_envelope(&self) -> Vec<Option<(f32, f32)>> {
        (0..self.nx)
            .map(|ix| {
                let mut lo = f32::INFINITY;
                let mut hi = f32::NEG_INFINITY;
                for iz in 0..self.nz {
                    if let Some(h) = self.at(ix, iz) {
                        lo = lo.min(h);
                        hi = hi.max(h);
                    }
                }
                lo.is_finite().then_some((lo, hi))
            })
            .collect()
    }
}

/// Colour of a shade index: dark green low ground through brown to pale rock.
fn shade_colour(step: usize) -> String {
    let t = step as f32 / (SHADE_STEPS - 1) as f32;
    // Three-stop ramp, linear between stops.
    let stops = [
        (0.0, (46.0, 74.0, 52.0)),
        (0.5, (120.0, 104.0, 66.0)),
        (1.0, (222.0, 218.0, 205.0)),
    ];
    let (lo, hi) = if t < 0.5 {
        (stops[0], stops[1])
    } else {
        (stops[1], stops[2])
    };
    let span = hi.0 - lo.0;
    let f = if span > 0.0 { (t - lo.0) / span } else { 0.0 };
    let mix = |a: f32, b: f32| (a + (b - a) * f).round() as u8;
    format!(
        "#{:02x}{:02x}{:02x}",
        mix(lo.1 .0, hi.1 .0),
        mix(lo.1 .1, hi.1 .1),
        mix(lo.1 .2, hi.1 .2)
    )
}

/// Write a two-panel schematic of `level` to `path`.
pub fn write_schematic(
    level: &Level,
    terrain: &TerrainManager,
    path: &Path,
) -> std::io::Result<()> {
    let svg = render(level, terrain);
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    std::fs::write(path, svg)
}

fn render(level: &Level, terrain: &TerrainManager) -> String {
    let field = HeightField::sample(terrain);
    let bounds = terrain.bounds();
    let size = bounds.size();

    // One scale for both panels so horizontal distances read the same in each.
    let scale = PLAN_WIDTH_PX / size.x;
    let plan_h = size.z * scale;

    let plan_top = MARGIN + 40.0;
    let elev_top = plan_top + plan_h + PANEL_GAP;
    let key_top = elev_top + ELEVATION_HEIGHT_PX + PANEL_GAP;
    let doc_w = PLAN_WIDTH_PX + 2.0 * MARGIN;
    let doc_h = key_top + key_height(level.objects.len()) + MARGIN;

    let mut s = String::with_capacity(1 << 16);
    let _ = write!(
        s,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{doc_w:.0}" height="{doc_h:.0}" viewBox="0 0 {doc_w:.0} {doc_h:.0}">
<style>
  text {{ font-family: -apple-system, "Helvetica Neue", sans-serif; fill: #222; }}
  .title {{ font-size: 20px; font-weight: 600; }}
  .panel {{ font-size: 14px; font-weight: 600; }}
  .axis {{ font-size: 11px; fill: #555; }}
  .marker {{ font-size: 10px; fill: #111; }}
</style>
<rect width="100%" height="100%" fill="#f6f5f2"/>
<text class="title" x="{MARGIN:.0}" y="{:.0}">{}</text>
<text class="axis" x="{MARGIN:.0}" y="{:.0}">{} triangles · voxel {:.2} m · bounds ({:.0}, {:.0}, {:.0}) .. ({:.0}, {:.0}, {:.0}) · sample {:.2} m</text>
"##,
        MARGIN - 18.0,
        escape(&level.name),
        MARGIN,
        terrain.triangle_count(),
        terrain.voxel_size(),
        bounds.min.x,
        bounds.min.y,
        bounds.min.z,
        bounds.max.x,
        bounds.max.y,
        bounds.max.z,
        field.cell,
    );

    plan_view(&mut s, level, terrain, &field, plan_top, scale);
    elevation_view(&mut s, level, terrain, &field, elev_top, scale);
    object_key(&mut s, level, key_top);

    s.push_str("</svg>\n");
    s
}

/// Objects are numbered in both panels rather than named: a level's objects
/// cluster, and thirty overlapping labels are worse than none. The numbers are
/// resolved by the key below the panels, which also carries their coordinates.
const KEY_COLUMNS: usize = 3;
const KEY_ROW_HEIGHT: f32 = 15.0;

fn key_rows(object_count: usize) -> usize {
    object_count.div_ceil(KEY_COLUMNS)
}

fn key_height(object_count: usize) -> f32 {
    24.0 + key_rows(object_count) as f32 * KEY_ROW_HEIGHT
}

/// Numbered list of the level's objects, in columns.
fn object_key(s: &mut String, level: &Level, top: f32) {
    let _ = writeln!(
        s,
        "<text class=\"panel\" x=\"{MARGIN:.0}\" y=\"{top:.0}\">Objects</text>"
    );
    if level.objects.is_empty() {
        let _ = writeln!(
            s,
            "<text class=\"axis\" x=\"{MARGIN:.0}\" y=\"{:.0}\">none</text>",
            top + 18.0
        );
        return;
    }

    let rows = key_rows(level.objects.len());
    let column_width = PLAN_WIDTH_PX / KEY_COLUMNS as f32;

    for (index, object) in level.objects.iter().enumerate() {
        let info = object.describe();
        let position = match info.placement {
            ObjectPlacement::Free(p) => format!("({:.1}, {:.1}, {:.1})", p.x, p.y, p.z),
            ObjectPlacement::TerrainAnchored { x, z } => {
                format!("({x:.1}, on terrain, {z:.1})")
            }
        };
        let x = MARGIN + (index / rows) as f32 * column_width;
        let y = top + 18.0 + (index % rows) as f32 * KEY_ROW_HEIGHT;
        let _ = writeln!(
            s,
            "<text class=\"axis\" x=\"{x:.1}\" y=\"{y:.1}\">{}. {} {}</text>",
            index + 1,
            escape(info.kind),
            position
        );
    }
}

/// Top-down panel: heightmap, chunk lattice, objects, spawn, scale bar.
fn plan_view(
    s: &mut String,
    level: &Level,
    terrain: &TerrainManager,
    field: &HeightField,
    top: f32,
    scale: f32,
) {
    let bounds = terrain.bounds();
    let size = bounds.size();
    let height = size.z * scale;

    // World (x, z) → panel pixels. +z runs down the page, which keeps the plan
    // in the same handedness as looking down at the world from above.
    let px = |x: f32| MARGIN + (x - bounds.min.x) * scale;
    let pz = |z: f32| top + (z - bounds.min.z) * scale;

    let _ = writeln!(
        s,
        "<text class=\"panel\" x=\"{:.0}\" y=\"{:.0}\">Plan — looking down (+x right, +z down)</text>",
        MARGIN,
        top - 10.0
    );
    let _ = writeln!(
        s,
        "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"#e8e6e1\"/>",
        MARGIN,
        top,
        size.x * scale,
        height
    );

    // Heightmap: one rect per run of equal shade along a row.
    let cell_px = field.cell * scale;
    for iz in 0..field.nz {
        let mut ix = 0;
        while ix < field.nx {
            let Some(shade) = field.shade(ix, iz) else {
                ix += 1;
                continue;
            };
            let mut run = 1;
            while ix + run < field.nx && field.shade(ix + run, iz) == Some(shade) {
                run += 1;
            }
            let x0 = px(field.origin_x + ix as f32 * field.cell);
            let y0 = pz(field.origin_z + iz as f32 * field.cell);
            let _ = writeln!(
                s,
                "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"{}\"/>",
                x0,
                y0,
                cell_px * run as f32 + 0.6,
                cell_px + 0.6,
                shade_colour(shade)
            );
            ix += run;
        }
    }

    // Chunk lattice. Derived bounds are chunk-aligned, so stepping from the
    // minimum corner lands exactly on the chunk boundaries.
    let extent = terrain.chunk_extent();
    let mut x = bounds.min.x;
    while x <= bounds.max.x + 0.01 {
        let _ = writeln!(
            s,
            "<line stroke=\"#ffffff\" stroke-opacity=\"0.35\" stroke-width=\"0.7\" x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\"/>",
            px(x),
            top,
            px(x),
            top + height
        );
        x += extent;
    }
    let mut z = bounds.min.z;
    while z <= bounds.max.z + 0.01 {
        let _ = writeln!(
            s,
            "<line stroke=\"#ffffff\" stroke-opacity=\"0.35\" stroke-width=\"0.7\" x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\"/>",
            MARGIN,
            pz(z),
            MARGIN + size.x * scale,
            pz(z)
        );
        z += extent;
    }

    // Objects, numbered against the key below the panels.
    for (index, object) in level.objects.iter().enumerate() {
        let info = object.describe();
        let (x, z) = info.placement.xz();
        object_marker(s, px(x), pz(z), index, &info.placement);
    }

    // Player spawn.
    let (sx, _, sz) = level.player_spawn;
    spawn_marker(s, px(sx), pz(sz));

    let _ = writeln!(
        s,
        "<rect fill=\"none\" stroke=\"#444\" stroke-width=\"1\" x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\"/>",
        MARGIN,
        top,
        size.x * scale,
        height
    );

    axis_ticks(
        s,
        bounds.min.x,
        bounds.max.x,
        terrain.chunk_extent(),
        |v| (px(v), top + height + 14.0),
        ("x", (MARGIN + size.x * scale + 16.0, top + height + 14.0)),
    );
    axis_ticks(
        s,
        bounds.min.z,
        bounds.max.z,
        terrain.chunk_extent(),
        |v| (MARGIN - 14.0, pz(v) + 3.0),
        ("z", (MARGIN - 14.0, top - 8.0)),
    );

    scale_bar(s, MARGIN, top + height + 34.0, scale);
    legend(s, MARGIN + 300.0, top + height + 34.0, field);
}

/// Elevation panel: the terrain's vertical envelope projected along z.
fn elevation_view(
    s: &mut String,
    level: &Level,
    terrain: &TerrainManager,
    field: &HeightField,
    top: f32,
    scale: f32,
) {
    let bounds = terrain.bounds();
    let size = bounds.size();
    let vscale = ELEVATION_HEIGHT_PX / size.y;

    let px = |x: f32| MARGIN + (x - bounds.min.x) * scale;
    let py = |y: f32| top + (bounds.max.y - y) * vscale;

    let _ = writeln!(
        s,
        "<text class=\"panel\" x=\"{:.0}\" y=\"{:.0}\">Elevation — terrain envelope along z (+x right, +y up)</text>\n\
         <rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"#e8e6e1\"/>",
        MARGIN,
        top - 10.0,
        MARGIN,
        top,
        size.x * scale,
        ELEVATION_HEIGHT_PX
    );

    // The band between the lowest and highest surface at each x. A flat plain
    // is a thin line; a tower or a cave system is a tall band. Columns with no
    // terrain at all break the band, so a void reads as a gap rather than as
    // ground at some interpolated height.
    for run in contiguous_runs(&field.elevation_envelope()) {
        let mut points = String::new();
        for (ix, (_, hi)) in &run {
            let _ = write!(
                points,
                "{:.1},{:.1} ",
                px(field.origin_x + *ix as f32 * field.cell),
                py(*hi)
            );
        }
        for (ix, (lo, _)) in run.iter().rev() {
            let _ = write!(
                points,
                "{:.1},{:.1} ",
                px(field.origin_x + *ix as f32 * field.cell),
                py(*lo)
            );
        }
        let _ = writeln!(
            s,
            "<polygon points=\"{}\" fill=\"#8c7a52\" fill-opacity=\"0.85\" stroke=\"#5b4d33\" stroke-width=\"0.8\"/>",
            points.trim()
        );
    }

    // y = 0 reference.
    let _ = writeln!(
        s,
        "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#1c6ea4\" stroke-width=\"0.8\" stroke-dasharray=\"4 3\"/>\n\
         <text class=\"axis\" x=\"{:.1}\" y=\"{:.1}\">y=0</text>",
        MARGIN,
        py(0.0),
        MARGIN + size.x * scale,
        py(0.0),
        MARGIN + size.x * scale + 4.0,
        py(0.0) + 3.0
    );

    // Objects at their authored height. An anchored object has no authored
    // height, so it is drawn at the surface it will be dropped onto.
    for (index, object) in level.objects.iter().enumerate() {
        let info = object.describe();
        let (x, y) = match info.placement {
            ObjectPlacement::Free(p) => (p.x, p.y),
            ObjectPlacement::TerrainAnchored { x, z } => {
                (x, terrain.approx_surface_height_at(x, z).unwrap_or(0.0))
            }
        };
        object_marker(s, px(x), py(y), index, &info.placement);
    }

    let (sx, sy, _) = level.player_spawn;
    spawn_marker(s, px(sx), py(sy));

    let _ = writeln!(
        s,
        "<rect fill=\"none\" stroke=\"#444\" stroke-width=\"1\" x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\"/>",
        MARGIN,
        top,
        size.x * scale,
        ELEVATION_HEIGHT_PX
    );

    axis_ticks(
        s,
        bounds.min.x,
        bounds.max.x,
        terrain.chunk_extent(),
        |v| (px(v), top + ELEVATION_HEIGHT_PX + 14.0),
        (
            "x",
            (
                MARGIN + size.x * scale + 16.0,
                top + ELEVATION_HEIGHT_PX + 14.0,
            ),
        ),
    );
    axis_ticks(
        s,
        bounds.min.y,
        bounds.max.y,
        terrain.chunk_extent(),
        |v| (MARGIN - 14.0, py(v) + 3.0),
        ("y", (MARGIN - 14.0, top - 8.0)),
    );
}

/// A numbered dot for one object. Colour distinguishes an authored height from
/// one the spawner will resolve from the terrain.
fn object_marker(s: &mut String, x: f32, y: f32, index: usize, placement: &ObjectPlacement) {
    let fill = match placement {
        ObjectPlacement::Free(_) => "#c1440e",
        ObjectPlacement::TerrainAnchored { .. } => "#1c6ea4",
    };
    let _ = writeln!(
        s,
        "<circle cx=\"{x:.1}\" cy=\"{y:.1}\" r=\"3.2\" fill=\"{fill}\" stroke=\"#fff\" stroke-width=\"0.8\"/>\n\
         <text class=\"marker\" x=\"{:.1}\" y=\"{:.1}\">{}</text>",
        x + 4.5,
        y - 3.5,
        index + 1
    );
}

/// Split a sampled profile into runs of consecutive columns that have terrain,
/// each carrying its sample index.
fn contiguous_runs(profile: &[Option<(f32, f32)>]) -> Vec<Vec<(usize, (f32, f32))>> {
    let mut runs: Vec<Vec<(usize, (f32, f32))>> = Vec::new();
    let mut current: Vec<(usize, (f32, f32))> = Vec::new();

    for (ix, entry) in profile.iter().enumerate() {
        match entry {
            Some(band) => current.push((ix, *band)),
            None => {
                if current.len() > 1 {
                    runs.push(std::mem::take(&mut current));
                } else {
                    current.clear();
                }
            }
        }
    }
    if current.len() > 1 {
        runs.push(current);
    }
    runs
}

/// A green diamond, used for the player spawn in both panels.
fn spawn_marker(s: &mut String, x: f32, y: f32) {
    let _ = writeln!(
        s,
        "<polygon points=\"{:.1},{:.1} {:.1},{:.1} {:.1},{:.1} {:.1},{:.1}\" fill=\"#1f9d55\" stroke=\"#fff\" stroke-width=\"1\"/>\n\
         <text class=\"marker\" x=\"{:.1}\" y=\"{:.1}\">spawn</text>",
        x, y - 6.0, x + 6.0, y, x, y + 6.0, x - 6.0, y,
        x + 8.0, y + 3.0
    );
}

/// Tick labels every `step` world units along one axis, plus the axis name at
/// an explicitly chosen spot — the end of an axis is often crowded.
fn axis_ticks(
    s: &mut String,
    from: f32,
    to: f32,
    step: f32,
    place: impl Fn(f32) -> (f32, f32),
    label: (&str, (f32, f32)),
) {
    let mut v = from;
    while v <= to + 0.01 {
        let (x, y) = place(v);
        let _ = writeln!(
            s,
            "<text class=\"axis\" x=\"{x:.1}\" y=\"{y:.1}\" text-anchor=\"middle\">{v:.0}</text>"
        );
        v += step;
    }

    let (name, (x, y)) = label;
    let _ = writeln!(
        s,
        "<text class=\"axis\" x=\"{x:.1}\" y=\"{y:.1}\" text-anchor=\"middle\" font-weight=\"600\">{name}</text>"
    );
}

/// A labelled bar showing how long a round number of metres is.
fn scale_bar(s: &mut String, x: f32, y: f32, scale: f32) {
    // Longest round distance that stays under 160 px.
    let metres = [5.0_f32, 10.0, 20.0, 50.0, 100.0, 200.0]
        .into_iter()
        .rev()
        .find(|m| m * scale <= 160.0)
        .unwrap_or(5.0);
    let _ = writeln!(
        s,
        "<line x1=\"{x:.1}\" y1=\"{y:.1}\" x2=\"{:.1}\" y2=\"{y:.1}\" stroke=\"#222\" stroke-width=\"2\"/>\n\
         <text class=\"axis\" x=\"{:.1}\" y=\"{:.1}\">{metres:.0} m</text>",
        x + metres * scale,
        x + metres * scale + 6.0,
        y + 4.0
    );
}

/// Marker key plus the height range the shading covers.
fn legend(s: &mut String, x: f32, y: f32, field: &HeightField) {
    let _ = writeln!(
        s,
        "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"3.2\" fill=\"#c1440e\"/><text class=\"axis\" x=\"{:.1}\" y=\"{:.1}\">object</text>\n\
         <circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"3.2\" fill=\"#1c6ea4\"/><text class=\"axis\" x=\"{:.1}\" y=\"{:.1}\">terrain-anchored</text>",
        x, y, x + 7.0, y + 4.0,
        x + 70.0, y, x + 77.0, y + 4.0
    );

    // The height ramp itself, as a strip of its own shades.
    let strip_x = x + 300.0;
    for step in 0..SHADE_STEPS {
        let _ = writeln!(
            s,
            "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"5\" height=\"10\" fill=\"{}\"/>",
            strip_x + step as f32 * 5.0,
            y - 6.0,
            shade_colour(step)
        );
    }
    let _ = writeln!(
        s,
        "<text class=\"axis\" x=\"{:.1}\" y=\"{:.1}\" text-anchor=\"end\">surface y {:.0} m</text>\n\
         <text class=\"axis\" x=\"{:.1}\" y=\"{:.1}\">{:.0} m</text>",
        strip_x - 6.0,
        y + 4.0,
        field.min,
        strip_x + SHADE_STEPS as f32 * 5.0 + 4.0,
        y + 4.0,
        field.max
    );
}

/// Escape the five characters that matter in XML text content.
fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shade_ramp_spans_dark_to_pale() {
        assert_eq!(shade_colour(0), "#2e4a34");
        assert_eq!(shade_colour(SHADE_STEPS - 1), "#dedacd");
    }

    #[test]
    fn text_is_escaped() {
        assert_eq!(escape("a & b <c>"), "a &amp; b &lt;c&gt;");
    }
}
