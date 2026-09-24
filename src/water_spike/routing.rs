//! Spike 0.5b: does routing over the drainage field give usable rivers, and
//! how much hierarchy do the current lakes carry?
//!
//! **Routing.** A channel follows drains from a spring, is smoothed into a
//! centreline, and is cut into cross-sections that carry a design discharge
//! by Manning's equation. Kill criteria (§21): the centreline stays within
//! 0.5 m laterally of the bed's lowest line, top width varies by no more than
//! 20% from cell to cell, and there are at most two real depressions per
//! 100 m of river.
//!
//! **Hierarchy.** Each authored pool's region is flooded at its level, and its
//! depression tree built by merging spans in order of floor: every merge of
//! two real depressions is a saddle the lake splits at when it drains below
//! it. Children per lake bed and splits per full drain are reported.

use std::collections::VecDeque;
use std::fmt::Write as _;
use std::path::Path;

use nalgebra::Point3;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::level::{load_level, Level, WaterBody};
use crate::level_check::build_terrain;
use crate::terrain::TerrainWorld;
use crate::water::geometry::{
    Column, DrainageField, Outlets, SpanGraph, SpanRasteriser, SpanRef, COLUMN_SIZE,
};
use crate::water::network::{Centreline, CrossSection};
use crate::water_viewer::find;

/// Design discharge for the width criterion, m³/s.
const DESIGN_DISCHARGE: f32 = 1.5;
/// A depression no deeper than this and holding no more than
/// `POTHOLE_VOLUME` is a pothole (§7.4).
const POTHOLE_DEPTH: f32 = 0.3;
const POTHOLE_VOLUME: f32 = 1.0;
/// Floors within this of the section's lowest count as its bottom.
const BOTTOM_BAND: f32 = 0.01;
/// Half-width of a cross-section, in metres.
const SECTION_HALF_WIDTH: f32 = 8.0;
/// Distance over which the bed slope is measured, each side, in metres.
const SLOPE_WINDOW: f32 = 4.0;
const CELL_AREA: f32 = COLUMN_SIZE * COLUMN_SIZE;

/// What one routed channel measured.
pub struct RouteFindings {
    pub label: String,
    /// Where the route ended: which kind of drain.
    pub ended: String,
    pub cells: usize,
    pub length: f32,
    /// Lateral distance from the centreline to the bed's lowest line, at each
    /// smoothed point.
    pub deviations: Vec<f32>,
    /// Relative change in top width between consecutive points.
    pub width_steps: Vec<f32>,
    pub widths: Vec<f32>,
    pub potholes: usize,
    pub real_depressions: usize,
    /// Points where the design discharge overtopped the section.
    pub overtopped: usize,
}

/// Route a spring from `start` over a level's terrain.
pub fn route_level(path: &Path, start: Point3<f32>, label: &str) -> Result<RouteFindings, String> {
    let level = load_level(path).map_err(|e| e.to_string())?;
    let terrain = build_terrain(&level);
    Ok(route(&terrain, start, label))
}

/// Route a spring down the water harness's carved river channel: the sloped
/// bed the criteria were re-run on at stage 4b.
pub fn route_river() -> Result<RouteFindings, String> {
    let scenario = find("river").ok_or("the river scenario is missing")?;
    let (_, terrain) = scenario.terrain()?;
    Ok(route(
        &terrain,
        Point3::new(-21.0, 3.5, 0.0),
        "carved river (water_viewer)",
    ))
}

/// Route a spring down the water harness's staircase.
pub fn route_staircase() -> Result<RouteFindings, String> {
    let scenario = find("staircase").ok_or("the staircase scenario is missing")?;
    let (_, terrain) = scenario.terrain()?;
    Ok(route(
        &terrain,
        Point3::new(-26.0, 10.5, 0.0),
        "staircase (water_viewer)",
    ))
}

fn route(terrain: &TerrainWorld, start: Point3<f32>, label: &str) -> RouteFindings {
    let (_, graph) = SpanRasteriser::build(terrain);
    let mut drainage = DrainageField::build(&graph, Outlets::default());
    let column = Column::containing(start.x, start.z);
    let mut findings = RouteFindings {
        label: label.to_string(),
        ended: String::new(),
        cells: 0,
        length: 0.0,
        deviations: Vec::new(),
        width_steps: Vec::new(),
        widths: Vec::new(),
        potholes: 0,
        real_depressions: 0,
        overtopped: 0,
    };
    let Some(first) = graph.span_at(column, start.y) else {
        findings.ended = "no span at the spring".into();
        return findings;
    };

    let mut path: Vec<SpanRef> = vec![first];
    let mut seen: FxHashSet<SpanRef> = FxHashSet::from_iter([first]);
    let mut measured: FxHashSet<SpanRef> = FxHashSet::default();
    let mut span = first;
    loop {
        let fill = drainage.fill(&graph, span);
        if fill > graph.span(span).floor_min + 1e-3 && !measured.contains(&span) {
            let region = depression(&graph, &drainage, span);
            let (depth, volume) = measure(&graph, fill, &region);
            if depth <= POTHOLE_DEPTH && volume <= POTHOLE_VOLUME {
                findings.potholes += 1;
            } else {
                findings.real_depressions += 1;
            }
            measured.extend(region);
        }
        match drainage.downstream_resolved(&graph, span) {
            Some(next) if seen.insert(next) => {
                path.push(next);
                span = next;
            }
            Some(_) => {
                findings.ended = "a cycle".into();
                break;
            }
            None => {
                findings.ended = format!("{:?}", drainage.drain(&graph, span));
                break;
            }
        }
    }
    findings.cells = path.len();

    let points: Vec<Point3<f32>> = path
        .iter()
        .map(|r| {
            let (x, z) = r.column.centre();
            Point3::new(x, graph.span(*r).floor_c, z)
        })
        .collect();
    let line = Centreline::from_path(&points);
    findings.length = line.length();

    let mut previous_width: Option<f32> = None;
    for (i, point) in line.points.iter().enumerate() {
        let section = CrossSection::sample(&graph, *point, line.tangents[i], SECTION_HALF_WIDTH);
        if let Some(offset) = bottom_offset(&section) {
            findings.deviations.push(offset.abs());
            if std::env::var_os("WATER_SPIKE_TRACE").is_some() {
                eprintln!(
                    "{label} {:.1} ({:.1}, {:.1}, {:.1}) offset {offset:.2}",
                    line.distance[i], point.x, point.y, point.z
                );
            }
        }
        let slope = bed_slope(&line, i);
        match section.hydraulics(DESIGN_DISCHARGE, slope) {
            Some(h) => {
                if let Some(w) = previous_width {
                    findings
                        .width_steps
                        .push((h.top_width - w).abs() / w.max(1e-3));
                }
                previous_width = Some(h.top_width);
                findings.widths.push(h.top_width);
            }
            None => {
                findings.overtopped += 1;
                previous_width = None;
            }
        }
    }
    findings
}

/// Distance each side of the centreline the bed's lowest line is looked for,
/// m: a channel's half-width, so a meander's other limb is not mistaken for
/// this one.
const LOWEST_LINE_REACH: f32 = 3.0;

/// Offset of the middle of the section's lowest run from its centre, within
/// `LOWEST_LINE_REACH` of it.
fn bottom_offset(section: &CrossSection) -> Option<f32> {
    let reach = (LOWEST_LINE_REACH / section.spacing).round() as usize;
    let lo = section.centre.saturating_sub(reach);
    let hi = (section.centre + reach).min(section.floors.len() - 1);
    let lowest = section.floors[lo..=hi]
        .iter()
        .flatten()
        .copied()
        .fold(f32::INFINITY, f32::min);
    if !lowest.is_finite() {
        return None;
    }
    let bottom: Vec<usize> = section
        .floors
        .iter()
        .enumerate()
        .filter(|(i, f)| (lo..=hi).contains(i) && f.is_some_and(|f| f <= lowest + BOTTOM_BAND))
        .map(|(i, _)| i)
        .collect();
    // The run of bottom samples nearest the centre.
    let nearest = *bottom
        .iter()
        .min_by_key(|&&i| (i as i64 - section.centre as i64).abs())?;
    let (mut lo, mut hi) = (nearest, nearest);
    while lo > 0 && bottom.contains(&(lo - 1)) {
        lo -= 1;
    }
    while bottom.contains(&(hi + 1)) {
        hi += 1;
    }
    let middle = (lo + hi) as f32 * 0.5;
    Some((middle - section.centre as f32) * section.spacing)
}

/// Bed slope at point `i`, over `SLOPE_WINDOW` each side.
fn bed_slope(line: &Centreline, i: usize) -> f32 {
    let d = line.distance[i];
    let before = line
        .distance
        .iter()
        .position(|&x| x >= d - SLOPE_WINDOW)
        .unwrap_or(0);
    let after = line
        .distance
        .iter()
        .rposition(|&x| x <= d + SLOPE_WINDOW)
        .unwrap_or(i);
    let run = line.distance[after] - line.distance[before];
    if run <= 0.0 {
        return 0.0;
    }
    ((line.points[before].y - line.points[after].y) / run).max(0.0)
}

/// Every span in the filled depression holding `span`: those at its fill,
/// below it, reachable under it.
fn depression(graph: &SpanGraph, drainage: &DrainageField, span: SpanRef) -> Vec<SpanRef> {
    let fill = drainage.fill(graph, span);
    let mut region = vec![span];
    let mut seen = FxHashSet::from_iter([span]);
    let mut queue = VecDeque::from([span]);
    while let Some(s) = queue.pop_front() {
        for n in graph.neighbours(s) {
            if n.saddle < fill
                && drainage.fill(graph, n.span) == fill
                && graph.span(n.span).floor_min < fill
                && seen.insert(n.span)
            {
                region.push(n.span);
                queue.push_back(n.span);
            }
        }
    }
    region
}

/// Depth and volume of still water standing at `level` over `region`.
fn measure(graph: &SpanGraph, level: f32, region: &[SpanRef]) -> (f32, f32) {
    let mut deepest: f32 = 0.0;
    let mut volume = 0.0;
    for r in region {
        let span = graph.span(*r);
        deepest = deepest.max(level - span.floor_min);
        let mean_floor = 0.5 * (span.floor_min + span.floor_max.min(level));
        volume += (level - mean_floor).max(0.0) * CELL_AREA;
    }
    (deepest, volume)
}

/// What one authored pool's depression tree holds.
pub struct HierarchyFindings {
    pub label: String,
    pub level: f32,
    /// The fill level at the seed: below the authored level, the pool would
    /// drain on the first tick.
    pub seed_fill: f32,
    pub region_spans: usize,
    /// Real depressions at the bottom of the tree.
    pub children: usize,
    /// Merges of two real depressions: where the lake splits as it drains.
    pub splits: usize,
    pub potholes: usize,
}

/// The depression tree of every pool in a level.
pub fn hierarchy(path: &Path) -> Result<Vec<HierarchyFindings>, String> {
    let level = load_level(path).map_err(|e| e.to_string())?;
    let terrain = build_terrain(&level);
    let (_, graph) = SpanRasteriser::build(&terrain);
    let drainage = DrainageField::build(&graph, Outlets::default());
    Ok(pools(&level)
        .into_iter()
        .map(|(seed, surface)| pool_tree(&graph, &drainage, path, seed, surface))
        .collect())
}

fn pools(level: &Level) -> Vec<((f32, f32), f32)> {
    level
        .water
        .iter()
        .flat_map(|w| &w.bodies)
        .map(|b| match b {
            WaterBody::Pool {
                seed,
                surface_level,
            } => (*seed, *surface_level),
        })
        .collect()
}

fn pool_tree(
    graph: &SpanGraph,
    drainage: &DrainageField,
    path: &Path,
    seed: (f32, f32),
    surface: f32,
) -> HierarchyFindings {
    let label = format!(
        "{} pool ({:.0}, {:.0}) @ {surface:.1}",
        path.file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default(),
        seed.0,
        seed.1
    );
    let mut findings = HierarchyFindings {
        label,
        level: surface,
        seed_fill: f32::NAN,
        region_spans: 0,
        children: 0,
        splits: 0,
        potholes: 0,
    };
    let Some(start) = graph.span_at(Column::containing(seed.0, seed.1), surface) else {
        return findings;
    };
    findings.seed_fill = drainage.fill(graph, start);

    // The region: spans below the surface, joined under it.
    let mut region = vec![start];
    let mut index: FxHashMap<SpanRef, usize> = FxHashMap::from_iter([(start, 0)]);
    let mut queue = VecDeque::from([start]);
    let mut edges: Vec<(f32, usize, usize)> = Vec::new();
    while let Some(s) = queue.pop_front() {
        let i = index[&s];
        for n in graph.orthogonal_neighbours(s) {
            if n.saddle >= surface || graph.span(n.span).floor_min >= surface {
                continue;
            }
            let j = *index.entry(n.span).or_insert_with(|| {
                region.push(n.span);
                queue.push_back(n.span);
                region.len() - 1
            });
            if i < j {
                edges.push((n.saddle, i, j));
            }
        }
    }
    findings.region_spans = region.len();

    // Kruskal over saddles: each merge is where two parts of the lake join
    // as it fills, and split as it drains.
    edges.sort_by(|a, b| a.0.total_cmp(&b.0));
    let floors: Vec<f32> = region.iter().map(|r| graph.span(*r).floor_min).collect();
    let mut parent: Vec<usize> = (0..region.len()).collect();
    let mut lowest = floors.clone();
    let mut count = vec![1usize; region.len()];
    let mut floor_sum: Vec<f64> = floors.iter().map(|&f| f as f64).collect();
    let mut leaves: FxHashSet<usize> = FxHashSet::default();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let real = |root: usize, at: f32, lowest: &[f32], count: &[usize], floor_sum: &[f64]| {
        let depth = at - lowest[root];
        let volume = (at as f64 * count[root] as f64 - floor_sum[root]) as f32 * CELL_AREA;
        depth > POTHOLE_DEPTH || volume > POTHOLE_VOLUME
    };
    for (saddle, i, j) in edges {
        let (a, b) = (root(&mut parent, i), root(&mut parent, j));
        if a == b {
            continue;
        }
        let a_real = real(a, saddle, &lowest, &count, &floor_sum);
        let b_real = real(b, saddle, &lowest, &count, &floor_sum);
        if a_real && b_real {
            findings.splits += 1;
            leaves.insert(a);
            leaves.insert(b);
        } else if a_real != b_real {
            findings.potholes += 1;
        }
        parent[b] = a;
        lowest[a] = lowest[a].min(lowest[b]);
        count[a] += count[b];
        floor_sum[a] += floor_sum[b];
    }
    findings.children = if findings.splits == 0 {
        1
    } else {
        findings.splits + 1
    };
    findings
}

/// The route findings as a report, with each kill criterion's verdict.
pub fn route_report(f: &RouteFindings) -> String {
    let mut out = String::new();
    let pass = |ok: bool| if ok { "pass" } else { "FAIL" };
    let within = f.deviations.iter().filter(|d| **d <= 0.5).count();
    let worst = f.deviations.iter().copied().fold(0.0f32, f32::max);
    let p95 = percentile(&f.deviations, 0.95);
    let step_p95 = percentile(&f.width_steps, 0.95);
    let step_worst = f.width_steps.iter().copied().fold(0.0f32, f32::max);
    let per_100 = f.real_depressions as f32 / (f.length / 100.0).max(1e-3);
    let _ = writeln!(out, "{}", f.label);
    let _ = writeln!(
        out,
        "  route: {} cells, {:.1} m, ended at {}",
        f.cells, f.length, f.ended
    );
    let _ = writeln!(
        out,
        "  centreline to lowest line: {:.1}% within 0.5 m, p95 {:.2} m, worst {:.2} m   [{}]",
        within as f32 / f.deviations.len().max(1) as f32 * 100.0,
        p95,
        worst,
        pass(worst <= 0.5)
    );
    let _ = writeln!(
        out,
        "  top width at {DESIGN_DISCHARGE} m³/s: {:.1}–{:.1} m; step p95 {:.0}%, worst {:.0}%   [{}]",
        f.widths.iter().copied().fold(f32::INFINITY, f32::min),
        f.widths.iter().copied().fold(0.0f32, f32::max),
        step_p95 * 100.0,
        step_worst * 100.0,
        pass(step_worst <= 0.2)
    );
    let _ = writeln!(
        out,
        "  sections the design discharge overtopped: {}",
        f.overtopped
    );
    let _ = writeln!(
        out,
        "  depressions: {} real ({:.1} per 100 m), {} potholes   [{}]",
        f.real_depressions,
        per_100,
        f.potholes,
        pass(per_100 <= 2.0)
    );
    out
}

pub fn hierarchy_report(f: &HierarchyFindings) -> String {
    let drains = if f.seed_fill < f.level {
        format!(
            "  ** stands above its outlet: fill level at the seed is {:.2} **\n",
            f.seed_fill
        )
    } else {
        String::new()
    };
    format!(
        "{}\n  {} spans; {} children, {} splits per full drain, {} potholes absorbed\n{drains}",
        f.label, f.region_spans, f.children, f.splits, f.potholes
    )
}

fn percentile(values: &[f32], q: f32) -> f32 {
    if values.is_empty() {
        return 0.0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.total_cmp(b));
    sorted[((sorted.len() - 1) as f32 * q).round() as usize]
}
