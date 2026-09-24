//! Schematic export: a level as a picture, without a graphics device.
//!
//! ```text
//!   ┌──────────────────────────────┐   top-down (xz): surface height as a
//!   │  plan   ┌────────┐  ┌─────┐  │   shaded heightmap, one outlined box per
//!   │         │ plaza ▓│╌╌│tower│  │   segment, anchors as arrows, connections
//!   │         └────────┘  └─────┘  │   as dashed links carrying their gap
//!   ├──────────────────────────────┤
//!   │  elevation    ▁▂▄▆█▆▄▂▁      │   (xy): the vertical envelope of the
//!   └──────────────────────────────┘   terrain, projected along z
//! ```
//!
//! The audience is someone who cannot run the game and wants to know whether a
//! level is roughly what they intended, so legibility beats fidelity: labelled
//! axes, a scale bar and a legend matter more than shading quality. In
//! particular the picture has to answer *what connects to what, in what order* —
//! which is the question the RON is worst at.

use std::fmt::Write as _;
use std::path::Path;

use nalgebra::Point3;

use crate::level::{world_anchor, Level, ObjectPlacement, Placement, VolumeFeature};
use crate::terrain::{outward, SegmentFrame, TerrainWorld};
use crate::water::geometry::Column;

use super::water_plan::WaterPlan;

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

/// Smallest vertical world span the elevation panel will show.
///
/// The panel fits its range to the terrain rather than to the derived bounds,
/// which for a flat level would otherwise squash 10 m of content into a strip of
/// a 96 m range. The floor stops the opposite failure: a perfectly flat level
/// magnified until its noise looks like mountains.
const MIN_ELEVATION_SPAN: f32 = 16.0;

/// Fraction of the fitted vertical range added as headroom above and below.
const ELEVATION_PADDING: f32 = 0.12;

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
    /// Sample the whole level on one world-space lattice.
    ///
    /// Deliberately not per-segment: `TerrainWorld` already answers a
    /// world-space column query by fanning out over segments, and one shared
    /// lattice is what keeps the plan a single readable picture when segments
    /// sit at different resolutions.
    fn sample(terrain: &TerrainWorld) -> Self {
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

/// The vertical world range the elevation panel draws, fitted to content.
struct VerticalRange {
    lo: f32,
    hi: f32,
}

impl VerticalRange {
    /// Fit to the sampled terrain and the authored object heights, padded, with
    /// a floor so a flat level is not magnified absurdly.
    fn fit(field: &HeightField, object_heights: &[f32]) -> Self {
        let mut lo = field.min;
        let mut hi = field.max;
        for y in object_heights {
            lo = lo.min(*y);
            hi = hi.max(*y);
        }
        // y = 0 is drawn as a reference line, so it has to be in range.
        lo = lo.min(0.0);
        hi = hi.max(0.0);

        let pad = (hi - lo) * ELEVATION_PADDING;
        let (mut lo, mut hi) = (lo - pad, hi + pad);

        let short = MIN_ELEVATION_SPAN - (hi - lo);
        if short > 0.0 {
            lo -= short / 2.0;
            hi += short / 2.0;
        }
        Self { lo, hi }
    }

    fn span(&self) -> f32 {
        self.hi - self.lo
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
pub fn write_schematic(level: &Level, terrain: &TerrainWorld, path: &Path) -> std::io::Result<()> {
    let water = WaterPlan::from_terrain(terrain);
    let svg = render(level, terrain, &water);
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    std::fs::write(path, svg)
}

fn render(level: &Level, terrain: &TerrainWorld, water: &WaterPlan) -> String {
    let field = HeightField::sample(terrain);
    let bounds = terrain.bounds();
    let size = bounds.size();

    let object_heights: Vec<f32> = level
        .objects()
        .filter_map(|(_, o)| match o.describe().placement {
            ObjectPlacement::Free(p) => Some(p.y),
            ObjectPlacement::TerrainAnchored { .. } => None,
        })
        .collect();
    let vertical = VerticalRange::fit(&field, &object_heights);

    // One scale for both panels so horizontal distances read the same in each.
    let scale = PLAN_WIDTH_PX / size.x;
    let plan_h = size.z * scale;

    let plan_top = MARGIN + 40.0;
    let elev_top = plan_top + plan_h + PANEL_GAP;
    let key_top = elev_top + ELEVATION_HEIGHT_PX + PANEL_GAP;
    let doc_w = PLAN_WIDTH_PX + 2.0 * MARGIN;
    let doc_h = key_top + key_height(level.object_count()) + MARGIN;

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
  .segname {{ font-size: 12px; font-weight: 600; fill: #123; }}
</style>
<rect width="100%" height="100%" fill="#f6f5f2"/>
<text class="title" x="{MARGIN:.0}" y="{:.0}">{}</text>
<text class="axis" x="{MARGIN:.0}" y="{:.0}">{}{} triangles · finest voxel {:.2} m · bounds ({:.0}, {:.0}, {:.0}) .. ({:.0}, {:.0}, {:.0}) · sample {:.2} m</text>
"##,
        MARGIN - 18.0,
        escape(&level.name),
        MARGIN,
        plural(terrain.segments().len(), "segment"),
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

    plan_view(&mut s, level, terrain, &field, water, plan_top, scale);
    elevation_view(&mut s, level, terrain, &field, &vertical, elev_top, scale);
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
    if level.object_count() == 0 {
        let _ = writeln!(
            s,
            "<text class=\"axis\" x=\"{MARGIN:.0}\" y=\"{:.0}\">none</text>",
            top + 18.0
        );
        return;
    }

    let rows = key_rows(level.object_count());
    let column_width = PLAN_WIDTH_PX / KEY_COLUMNS as f32;

    for (index, (segment, object)) in level.objects().enumerate() {
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
            "<text class=\"axis\" x=\"{x:.1}\" y=\"{y:.1}\">{}. {} {} [{}]</text>",
            index + 1,
            escape(info.kind),
            position,
            escape(&level.segments[segment].name),
        );
    }
}

/// Reserves screen space so that labels in a crowded region do not overprint
/// each other.
///
/// The densest part of a level is the part an author most wants to read, so
/// dropping a colliding label is better than drawing it: the marker still shows
/// the position, and the key below carries the exact coordinates.
struct LabelSpace {
    cell_w: f32,
    cell_h: f32,
    taken: std::collections::HashSet<(i32, i32)>,
}

impl LabelSpace {
    fn new(cell_w: f32, cell_h: f32) -> Self {
        Self {
            cell_w,
            cell_h,
            taken: std::collections::HashSet::new(),
        }
    }

    /// Claim the cell at a pixel position, returning false if it was taken.
    fn claim(&mut self, x: f32, y: f32) -> bool {
        self.taken.insert((
            (x / self.cell_w).floor() as i32,
            (y / self.cell_h).floor() as i32,
        ))
    }

    /// Claim every cell a piece of text would cover, returning false unless all
    /// of them were free.
    ///
    /// A single cell is not enough for a name: at 30 px per cell, two anchors
    /// facing each other across a 5 m join land in *different* cells and their
    /// text still overprints, which is how `east_ledge` and `west_landing`
    /// became `east_ledgelanding`. That arrangement recurs at every join, so it
    /// is the one worth reserving properly for.
    fn claim_text(&mut self, x: f32, y: f32, text: &str) -> bool {
        let width = text.chars().count() as f32 * TEXT_CHAR_WIDTH;
        let row = (y / self.cell_h).floor() as i32;
        let first = (x / self.cell_w).floor() as i32;
        let last = ((x + width) / self.cell_w).floor() as i32;
        if (first..=last).any(|c| self.taken.contains(&(c, row))) {
            return false;
        }
        for c in first..=last {
            self.taken.insert((c, row));
        }
        true
    }
}

/// Nominal advance width of one character at the marker font size.
///
/// The renderer is a browser and the font is whatever it has, so this is an
/// estimate — but a generous estimate is exactly right here, since reserving
/// slightly too much drops a label and reserving too little overprints two.
const TEXT_CHAR_WIDTH: f32 = 5.6;

/// `"1 segment"`, `"4 segments"`.
fn plural(n: usize, noun: &str) -> String {
    if n == 1 {
        format!("{n} {noun} · ")
    } else {
        format!("{n} {noun}s · ")
    }
}

/// Top-down panel: heightmap, segment outlines, anchors, connections, objects.
fn plan_view(
    s: &mut String,
    level: &Level,
    terrain: &TerrainWorld,
    field: &HeightField,
    water: &WaterPlan,
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

    water_overlay(s, water, &px, &pz, scale);

    // Segment outlines over their solid extent, so the picture shows the areas
    // a level is built from rather than one undifferentiated heightmap.
    for segment in terrain.segments() {
        let Some(extent) = segment.solid_bounds() else {
            continue;
        };
        let (x0, z0) = (px(extent.min.x), pz(extent.min.z));
        let (x1, z1) = (px(extent.max.x), pz(extent.max.z));
        let _ = writeln!(
            s,
            "<rect x=\"{x0:.1}\" y=\"{z0:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"none\" \
             stroke=\"#1a3552\" stroke-width=\"1.6\" stroke-dasharray=\"6 3\"/>\n\
             <text class=\"segname\" x=\"{:.1}\" y=\"{:.1}\">{}</text>",
            x1 - x0,
            z1 - z0,
            x0 + 5.0,
            z0 + 14.0,
            escape(segment.name()),
        );
    }

    // The route, over the heightmap and under the labels: it is the thing a
    // reader is looking for, and nothing else in the picture is this colour.
    route_overlay(s, level, &px, &pz, scale);

    // Every piece of text in this panel shares one reservation grid, claimed in
    // decreasing order of importance: a gap figure matters more than an anchor
    // name, which matters more than an object's index (the key below carries
    // that anyway).
    let mut labels = LabelSpace::new(30.0, 12.0);

    connection_links(s, level, &px, &pz, &mut labels);

    for segment in terrain.segments() {
        for anchor in segment.anchors() {
            let world = anchor.world_frame(segment.frame());
            // The segment name is already printed on its outline, so the anchor
            // carries only its own name.
            anchor_marker(s, &px, &pz, &world, anchor.name(), scale, &mut labels);
        }
    }

    // Objects, numbered against the key below the panels.
    for (index, (_, object)) in level.objects().enumerate() {
        let info = object.describe();
        let (x, z) = info.placement.xz();
        object_marker(s, px(x), pz(z), index, &info.placement, &mut labels);
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

    let step = tick_step(size.x.max(size.z));
    axis_ticks(
        s,
        bounds.min.x,
        bounds.max.x,
        step,
        |v| (px(v), top + height + 14.0),
        ("x", (MARGIN + size.x * scale + 16.0, top + height + 14.0)),
    );
    axis_ticks(
        s,
        bounds.min.z,
        bounds.max.z,
        step,
        |v| (MARGIN - 14.0, pz(v) + 3.0),
        ("z", (MARGIN - 14.0, top - 8.0)),
    );

    scale_bar(s, MARGIN, top + height + 34.0, scale);
    legend(s, MARGIN + 190.0, top + height + 34.0, field);
}

/// Where water would collect, over the heightmap: blue as deep as a
/// depression is, violet where a column holds more than one layer of air,
/// red where a pocket is sealed or its spans needed a parity repair.
fn water_overlay(
    s: &mut String,
    water: &WaterPlan,
    px: &impl Fn(f32) -> f32,
    pz: &impl Fn(f32) -> f32,
    scale: f32,
) {
    let size = water.cell * scale + 0.4;
    let mut runs: Vec<(Column, usize, String)> = Vec::new();
    for cell in &water.cells {
        let colour = if cell.sealed {
            "rgba(200,40,40,0.8)".to_string()
        } else if cell.pooled > 0.0 {
            let alpha = 0.25 + 0.55 * (cell.pooled / 2.0).min(1.0);
            format!("rgba(40,110,220,{alpha:.2})")
        } else {
            "rgba(130,70,190,0.35)".to_string()
        };
        match runs.last_mut() {
            Some((start, len, c))
                if *c == colour
                    && start.k == cell.column.k
                    && start.i + *len as i32 == cell.column.i =>
            {
                *len += 1;
            }
            _ => runs.push((cell.column, 1, colour)),
        }
    }
    for (start, len, colour) in runs {
        let (x, z) = start.min_corner();
        let _ = writeln!(
            s,
            "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{size:.1}\" fill=\"{colour}\"/>",
            px(x),
            pz(z),
            water.cell * scale * len as f32 + 0.4,
        );
    }
    for column in &water.parity_repairs {
        let (x, z) = column.centre();
        let _ = writeln!(
            s,
            "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"4\" fill=\"none\" stroke=\"#c00\" stroke-width=\"1.5\"/>",
            px(x),
            pz(z)
        );
    }
}

/// The traversal primitives, drawn as the route they are.
///
/// This is the one thing the plan panel could not previously show: a level
/// shaped only by heightfields reads as a heightmap, and a reader has to open
/// the RON to find out where the author intended the player to *go*. A deck
/// drawn at its real width, in a colour nothing else uses, means the route can
/// be traced with a finger.
///
/// Everything is drawn in world space, so a primitive in a quarter-turned
/// segment appears turned — which also makes this a visual check on the
/// orientation work.
fn route_overlay(
    s: &mut String,
    level: &Level,
    px: &impl Fn(f32) -> f32,
    pz: &impl Fn(f32) -> f32,
    scale: f32,
) {
    for (index, segment) in level.segments.iter().enumerate() {
        let frame = level.frame(index);
        let world = |x: f32, z: f32| {
            let w = frame.to_world(Point3::new(x, 0.0, z));
            (px(w.x), pz(w.z))
        };

        for volume in &segment.terrain.volumes {
            match volume {
                VolumeFeature::Path { points, width, .. } => {
                    let pts: Vec<String> = points
                        .iter()
                        .map(|p| {
                            let (x, y) = world(p.0, p.2);
                            format!("{x:.1},{y:.1}")
                        })
                        .collect();
                    // Stroked at the deck's real width, with a thin centreline
                    // over it so a narrow path is still visible at any scale.
                    let _ = writeln!(
                        s,
                        "<polyline points=\"{}\" fill=\"none\" stroke=\"{ROUTE_FILL}\" \
                         stroke-width=\"{:.1}\" stroke-linejoin=\"round\" \
                         stroke-linecap=\"round\" opacity=\"0.75\"/>\n\
                         <polyline points=\"{}\" fill=\"none\" stroke=\"{ROUTE_LINE}\" \
                         stroke-width=\"1.2\" stroke-linejoin=\"round\"/>",
                        pts.join(" "),
                        (width * scale).max(2.0),
                        pts.join(" "),
                    );
                }

                VolumeFeature::Platform {
                    center,
                    half_extents,
                    ..
                } => {
                    // A quarter turn swaps the half-extents with the axes, so
                    // the rectangle is spanned from its transformed corners
                    // rather than assumed to keep its authored ones.
                    let (ax, az) = world(center.0 - half_extents.0, center.2 - half_extents.1);
                    let (bx, bz) = world(center.0 + half_extents.0, center.2 + half_extents.1);
                    let _ = writeln!(
                        s,
                        "<rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\" \
                         fill=\"{ROUTE_FILL}\" fill-opacity=\"0.75\" stroke=\"{ROUTE_LINE}\" \
                         stroke-width=\"1.2\"/>",
                        ax.min(bx),
                        az.min(bz),
                        (bx - ax).abs(),
                        (bz - az).abs(),
                    );
                }

                VolumeFeature::Staircase {
                    from,
                    to,
                    width,
                    steps,
                    ..
                } => {
                    let (ax, az) = world(from.0, from.2);
                    let (bx, bz) = world(to.0, to.2);
                    let _ = writeln!(
                        s,
                        "<line x1=\"{ax:.1}\" y1=\"{az:.1}\" x2=\"{bx:.1}\" y2=\"{bz:.1}\" \
                         stroke=\"{ROUTE_FILL}\" stroke-width=\"{:.1}\" opacity=\"0.75\"/>",
                        (width * scale).max(2.0),
                    );
                    // One tick per riser: what distinguishes a flight from a
                    // ramp in the picture is that you can count the steps.
                    let (nx, nz) = ((bz - az), -(bx - ax));
                    let len = (nx * nx + nz * nz).sqrt().max(1e-3);
                    let half = (width * scale).max(2.0) * 0.5;
                    for i in 0..=*steps {
                        let t = i as f32 / (*steps).max(1) as f32;
                        let (cx, cz) = (ax + (bx - ax) * t, az + (bz - az) * t);
                        let _ = writeln!(
                            s,
                            "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" \
                             stroke=\"{ROUTE_LINE}\" stroke-width=\"1\"/>",
                            cx - nx / len * half,
                            cz - nz / len * half,
                            cx + nx / len * half,
                            cz + nz / len * half,
                        );
                    }
                }

                VolumeFeature::Shaft {
                    center,
                    radius,
                    ledge,
                    ..
                } => {
                    let (cx, cz) = world(center.0, center.1);
                    let _ = writeln!(
                        s,
                        "<circle cx=\"{cx:.1}\" cy=\"{cz:.1}\" r=\"{:.1}\" fill=\"#f6f5f2\" \
                         stroke=\"{ROUTE_LINE}\" stroke-width=\"1.2\" stroke-dasharray=\"3 2\"/>",
                        (radius * scale).max(3.0),
                    );
                    if let Some(l) = ledge {
                        let _ = writeln!(
                            s,
                            "<circle cx=\"{cx:.1}\" cy=\"{cz:.1}\" r=\"{:.1}\" fill=\"none\" \
                             stroke=\"{ROUTE_FILL}\" stroke-width=\"{:.1}\" opacity=\"0.75\"/>",
                            ((radius - l.width * 0.5) * scale).max(2.0),
                            (l.width * scale).max(2.0),
                        );
                    }
                }

                _ => {}
            }
        }
    }
}

/// Fill for a walkable surface, and the line that outlines it.
const ROUTE_FILL: &str = "#c9a227";
const ROUTE_LINE: &str = "#6b4f00";

/// Dashed links between connected anchors, labelled with their gap.
///
/// Placement joins and asserted connections are drawn differently, because the
/// difference is the thing a reader most needs: a join determines where an area
/// *is*, an assertion only claims that two areas meet.
fn connection_links(
    s: &mut String,
    level: &Level,
    px: &impl Fn(f32) -> f32,
    pz: &impl Fn(f32) -> f32,
    labels: &mut LabelSpace,
) {
    let joins = level.placements.iter().filter_map(|p| match p {
        Placement::Root { .. } => None,
        Placement::Join {
            segment,
            anchor,
            to,
            gap,
            ..
        } => Some((to.clone(), format!("{segment}.{anchor}"), *gap, true)),
    });
    let asserted = level
        .connections
        .iter()
        .map(|c| (c.from.clone(), c.to.clone(), c.gap, false));

    for (from, to, gap, is_join) in joins.chain(asserted) {
        let (Ok(a), Ok(b)) = (
            world_anchor(level, &level.frames, &from),
            world_anchor(level, &level.frames, &to),
        ) else {
            continue;
        };
        let (colour, dash) = if is_join {
            ("#c1440e", "5 3")
        } else {
            ("#6a3fa0", "2 4")
        };
        let (x0, y0) = (px(a.origin().x), pz(a.origin().z));
        let (x1, y1) = (px(b.origin().x), pz(b.origin().z));
        let _ = writeln!(
            s,
            "<line x1=\"{x0:.1}\" y1=\"{y0:.1}\" x2=\"{x1:.1}\" y2=\"{y1:.1}\" stroke=\"{colour}\" \
             stroke-width=\"1.6\" stroke-dasharray=\"{dash}\"/>",
        );

        let (lx, ly) = ((x0 + x1) / 2.0, (y0 + y1) / 2.0 - 5.0);
        let text = format!("{gap:.1} m");
        if labels.claim_text(lx - text.len() as f32 * TEXT_CHAR_WIDTH * 0.5, ly, &text) {
            let _ = writeln!(
                s,
                "<text class=\"marker\" x=\"{lx:.1}\" y=\"{ly:.1}\" text-anchor=\"middle\" fill=\"{colour}\">{gap:.1} m</text>",
            );
        }
    }
}

/// A small arrow at an anchor, pointing the way the anchor faces.
fn anchor_marker(
    s: &mut String,
    px: &impl Fn(f32) -> f32,
    pz: &impl Fn(f32) -> f32,
    world: &SegmentFrame,
    label: &str,
    scale: f32,
    labels: &mut LabelSpace,
) {
    let o = world.origin();
    let dir = outward(world);
    // A fixed pixel length, so the arrow stays visible at any level size.
    let length = 12.0 / scale;
    let tip = Point3::new(o.x + dir.x * length, o.y, o.z + dir.z * length);

    let _ = writeln!(
        s,
        "<line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#1a3552\" stroke-width=\"1.4\"/>\n\
         <circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"2.6\" fill=\"#1a3552\"/>",
        px(o.x),
        pz(o.z),
        px(tip.x),
        pz(tip.z),
        px(o.x),
        pz(o.z),
    );

    // Put the name on the side the anchor faces. Two anchors mated across a
    // join face each other, so their labels go to opposite sides of the gap and
    // stop competing for the same strip of page — which is the arrangement that
    // used to print `east_ledge` and `west_landing` on top of each other as
    // `east_ledgelanding`, and it recurs at every single join.
    let (ax, az) = (px(o.x), pz(o.z));
    let (tx, tz) = (px(tip.x), pz(tip.z));
    let (out_x, out_z) = (tx - ax, tz - az);
    let anchored_end = if out_x < -0.5 { "end" } else { "start" };
    let dx = if out_x < -0.5 { -7.0 } else { 7.0 };
    // The row is chosen from the facing too, not only the side. Two mated
    // anchors are a few metres apart and their names are sixty pixels long, so
    // putting them on opposite *sides* of a join still overlaps; putting them
    // on opposite sides of the link *line* cannot. The offsets are more than
    // one row apart from each other and from the gap figure at the midpoint, so
    // the three never compete for a cell.
    let faces_negative = out_x < -0.5 || out_z > 0.5;
    let lx = ax + dx;
    let reserve_x = if anchored_end == "end" {
        lx - label.chars().count() as f32 * TEXT_CHAR_WIDTH
    } else {
        lx
    };

    // Preferred row first, then the other side. Two anchors on the same face of
    // one segment are far enough apart to read but not far enough for two
    // names, and falling back beats dropping one of a pair.
    let preferred = if faces_negative { 20.0 } else { -20.0 };
    let ly = [az + preferred, az - preferred]
        .into_iter()
        .find(|&y| labels.claim_text(reserve_x, y, label));
    if let Some(ly) = ly {
        let _ = writeln!(
            s,
            "<text class=\"marker\" x=\"{lx:.1}\" y=\"{ly:.1}\" text-anchor=\"{anchored_end}\" \
             fill=\"#1a3552\">{}</text>",
            escape(label),
        );
    }
}

/// Elevation panel: the terrain's vertical envelope projected along z.
fn elevation_view(
    s: &mut String,
    level: &Level,
    terrain: &TerrainWorld,
    field: &HeightField,
    vertical: &VerticalRange,
    top: f32,
    scale: f32,
) {
    let bounds = terrain.bounds();
    let size = bounds.size();
    let vscale = ELEVATION_HEIGHT_PX / vertical.span();

    let px = |x: f32| MARGIN + (x - bounds.min.x) * scale;
    let py = |y: f32| top + (vertical.hi - y) * vscale;

    let _ = writeln!(
        s,
        "<text class=\"panel\" x=\"{:.0}\" y=\"{:.0}\">Elevation — terrain envelope along z (+x right, +y up), y from {:.0} to {:.0} m, fitted to content</text>\n\
         <rect x=\"{:.1}\" y=\"{:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"#e8e6e1\"/>",
        MARGIN,
        top - 10.0,
        vertical.lo,
        vertical.hi,
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

    // Segment extents as boxes, so a stacked or elevated area reads as one.
    for segment in terrain.segments() {
        let Some(extent) = segment.solid_bounds() else {
            continue;
        };
        let y_top = py(extent.max.y.min(vertical.hi));
        let y_bottom = py(extent.min.y.max(vertical.lo));
        let _ = writeln!(
            s,
            "<rect x=\"{:.1}\" y=\"{y_top:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"none\" \
             stroke=\"#1a3552\" stroke-width=\"1.0\" stroke-dasharray=\"6 3\" stroke-opacity=\"0.7\"/>",
            px(extent.min.x),
            px(extent.max.x) - px(extent.min.x),
            (y_bottom - y_top).max(1.0),
        );
    }

    // Objects at their authored height. An anchored object has no authored
    // height, so it is drawn at the surface it will be dropped onto.
    let mut labels = LabelSpace::new(16.0, 12.0);
    for (index, (_, object)) in level.objects().enumerate() {
        let info = object.describe();
        let (x, y) = match info.placement {
            ObjectPlacement::Free(p) => (p.x, p.y),
            ObjectPlacement::TerrainAnchored { x, z } => {
                (x, terrain.approx_surface_height_at(x, z).unwrap_or(0.0))
            }
        };
        object_marker(s, px(x), py(y), index, &info.placement, &mut labels);
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
        tick_step(size.x),
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
        vertical.lo,
        vertical.hi,
        tick_step(vertical.span()),
        |v| (MARGIN - 14.0, py(v) + 3.0),
        ("y", (MARGIN - 14.0, top - 8.0)),
    );
}

/// A round tick spacing giving no more than a dozen ticks across a span.
fn tick_step(span: f32) -> f32 {
    [
        1.0_f32, 2.0, 5.0, 10.0, 20.0, 25.0, 50.0, 100.0, 200.0, 500.0,
    ]
    .into_iter()
    .find(|step| span / step <= 12.0)
    .unwrap_or(1000.0)
}

/// A numbered dot for one object. Colour distinguishes an authored height from
/// one the spawner will resolve from the terrain.
fn object_marker(
    s: &mut String,
    x: f32,
    y: f32,
    index: usize,
    placement: &ObjectPlacement,
    labels: &mut LabelSpace,
) {
    let fill = match placement {
        ObjectPlacement::Free(_) => "#c1440e",
        ObjectPlacement::TerrainAnchored { .. } => "#1c6ea4",
    };
    let _ = writeln!(
        s,
        "<circle cx=\"{x:.1}\" cy=\"{y:.1}\" r=\"3.2\" fill=\"{fill}\" stroke=\"#fff\" stroke-width=\"0.8\"/>"
    );
    if labels.claim(x + 4.5, y - 3.5) {
        let _ = writeln!(
            s,
            "<text class=\"marker\" x=\"{:.1}\" y=\"{:.1}\">{}</text>",
            x + 4.5,
            y - 3.5,
            index + 1
        );
    }
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
    // Start at the first round multiple inside the range, so ticks read as
    // round numbers rather than as offsets from an arbitrary corner.
    let mut v = (from / step).ceil() * step;
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
         <circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"3.2\" fill=\"#1c6ea4\"/><text class=\"axis\" x=\"{:.1}\" y=\"{:.1}\">anchored</text>\n\
         <line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#c1440e\" stroke-width=\"1.6\" stroke-dasharray=\"5 3\"/><text class=\"axis\" x=\"{:.1}\" y=\"{:.1}\">join</text>\n\
         <line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"#6a3fa0\" stroke-width=\"1.6\" stroke-dasharray=\"2 4\"/><text class=\"axis\" x=\"{:.1}\" y=\"{:.1}\">assertion</text>\n\
         <line x1=\"{:.1}\" y1=\"{:.1}\" x2=\"{:.1}\" y2=\"{:.1}\" stroke=\"{ROUTE_FILL}\" stroke-width=\"6\" opacity=\"0.75\"/><text class=\"axis\" x=\"{:.1}\" y=\"{:.1}\">route</text>",
        x, y, x + 7.0, y + 4.0,
        x + 55.0, y, x + 62.0, y + 4.0,
        x + 130.0, y, x + 150.0, y, x + 154.0, y + 4.0,
        x + 185.0, y, x + 205.0, y, x + 209.0, y + 4.0,
        x + 262.0, y, x + 282.0, y, x + 286.0, y + 4.0,
    );

    // The height ramp itself, as a strip of its own shades.
    let strip_x = x + 470.0;
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

    fn flat_field(min: f32, max: f32) -> HeightField {
        HeightField {
            cell: 1.0,
            nx: 1,
            nz: 1,
            origin_x: 0.0,
            origin_z: 0.0,
            heights: vec![Some(max)],
            min,
            max,
        }
    }

    #[test]
    fn shade_ramp_spans_dark_to_pale() {
        assert_eq!(shade_colour(0), "#2e4a34");
        assert_eq!(shade_colour(SHADE_STEPS - 1), "#dedacd");
    }

    #[test]
    fn text_is_escaped() {
        assert_eq!(escape("a & b <c>"), "a &amp; b &lt;c&gt;");
    }

    /// The elevation panel used to span the whole derived y range, squashing a
    /// 10 m-thick level into a strip of a 96 m box. It must fit the content.
    #[test]
    fn elevation_range_fits_content_not_bounds() {
        let range = VerticalRange::fit(&flat_field(-2.0, 8.0), &[]);
        assert!(range.span() < 20.0, "span {} is not fitted", range.span());
        assert!(range.lo <= -2.0 && range.hi >= 8.0);
    }

    /// A perfectly flat level must not be magnified until its noise looks like
    /// terrain.
    #[test]
    fn a_flat_level_gets_a_minimum_vertical_span() {
        let range = VerticalRange::fit(&flat_field(0.0, 0.0), &[]);
        assert!((range.span() - MIN_ELEVATION_SPAN).abs() < 1e-3);
    }

    /// An object well above the terrain still has to be inside the panel.
    #[test]
    fn elevation_range_includes_authored_object_heights() {
        let range = VerticalRange::fit(&flat_field(0.0, 2.0), &[40.0]);
        assert!(range.hi >= 40.0, "hi {} excludes the object", range.hi);
    }

    /// Two markers in the same spot must not both draw a number.
    #[test]
    fn colliding_labels_are_dropped() {
        let mut space = LabelSpace::new(16.0, 12.0);
        assert!(space.claim(100.0, 100.0));
        assert!(!space.claim(102.0, 103.0));
        assert!(space.claim(200.0, 100.0));
    }

    #[test]
    fn tick_step_keeps_the_axis_readable() {
        for span in [10.0_f32, 64.0, 192.0, 400.0, 2000.0] {
            let step = tick_step(span);
            assert!(
                span / step <= 12.0,
                "span {span} with step {step} gives {} ticks",
                span / step
            );
        }
    }
}
