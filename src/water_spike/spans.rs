//! Spike 0.5a: are rasterised spans right, and cheap enough to rebuild?
//!
//! Three questions, each with a kill criterion (§21):
//!
//! - **Floors.** Ground truth is 8 × 8 vertical rays per column, each floor
//!   hit assigned to a span by the floor-band rule. `floor_min` must never be
//!   above the true band minimum, and within 0.1 m of it in 99% of columns.
//! - **Parity.** Every level must pair with zero repairs.
//! - **Rebuild.** Re-pairing the columns under a blast must take ≤ 1 ms.
//!
//! The ray path is also sampled at every column centre, to count where its
//! structure disagrees with the rasteriser's.

use std::fmt::Write as _;
use std::path::Path;
use std::time::{Duration, Instant};

use nalgebra::Point3;
use rayon::prelude::*;

use crate::level::load_level;
use crate::level_check::build_terrain;
use crate::perf::stats::{max_ms, mean_ms, percentile_ms};
use crate::terrain::{BlastConfig, TerrainWorld};
use crate::water::geometry::{
    Column, Drain, DrainageField, Outlets, Span, SpanGraph, SpanRasteriser, COLUMN_SIZE,
};

/// Grenades set off per level.
const BLASTS: usize = 48;

/// Sub-samples per column side for the ground truth.
const SUBSAMPLES: usize = 8;

/// How far `floor_min` may sit below the sampled minimum and still count as
/// close, in metres.
const CLOSE: f32 = 0.1;

/// A floor above the sampled minimum by more than this is a failure. The rays
/// and the rasteriser interpolate the same triangles differently by rounding.
const ABOVE_TOLERANCE: f32 = 1e-4;

/// What one level's spans measured.
pub struct SpanFindings {
    pub level: String,
    /// Priority-Flood over the whole graph at load.
    pub drainage_build: Duration,
    /// Spans with no path to any outlet.
    pub sealed: usize,
    /// Drainage repair after each blast.
    pub drainage_repairs: Vec<Duration>,
    /// Spans whose repaired fill differs from a full rebuild after the sweep.
    pub drainage_mismatches: usize,
    /// Spans whose drain differs from a full rebuild, once pending flats are
    /// resolved.
    pub drain_mismatches: usize,
    /// Drain walks from every span that cycle or stop short of an outlet.
    pub lost_walks: usize,
    /// Flats left pending by the sweep, and the time to resolve them all.
    pub pending: usize,
    pub pending_resolve: Duration,
    pub build: Duration,
    pub columns: usize,
    pub spans: usize,
    pub multi_span_columns: usize,
    pub parity_repairs: usize,
    /// Spans whose `floor_min` sat above a sampled floor in its band.
    pub floor_above_truth: usize,
    /// The worst of those, in metres.
    pub worst_above: f32,
    /// Spans with sampled floors, and those whose `floor_min` was within
    /// `CLOSE` of the sampled minimum.
    pub compared: usize,
    pub close: usize,
    /// As `close`, against samples that include the square's rim.
    pub close_with_rim: usize,
    /// Columns where the centre ray's structure (span count, or a floor or
    /// ceiling more than 1 mm off) disagreed with the rasteriser.
    pub ray_disagreements: usize,
    pub first_disagreement: Option<String>,
    /// Re-pair times after each blast of the sweep.
    pub rebuilds: Vec<Duration>,
    /// Span chunks re-paired per blast, summed.
    pub rebuilt_chunks: usize,
    pub invariant_violations: usize,
    pub floor_clamps: usize,
    pub worst_clamp: f32,
    /// Columns where the incrementally rebuilt spans differ from a full build
    /// of the blasted terrain.
    pub incremental_mismatches: usize,
    pub first_mismatch: Option<String>,
}

/// Measure one level.
pub fn check_level(path: &Path) -> Result<SpanFindings, String> {
    let level = load_level(path).map_err(|e| e.to_string())?;
    let mut terrain = build_terrain(&level);
    let started = Instant::now();
    let (mut rasteriser, mut graph) = SpanRasteriser::build(&terrain);
    let build = started.elapsed();

    let outlets = Outlets::default();
    let started = Instant::now();
    let mut drainage = DrainageField::build(&graph, outlets.clone());
    let drainage_build = started.elapsed();
    let sealed = graph
        .all_refs()
        .filter(|r| drainage.drain(&graph, *r) == Drain::Sealed)
        .count();

    let columns = occupied_columns(&graph);
    let mut findings = SpanFindings {
        drainage_build,
        sealed,
        drainage_repairs: Vec::new(),
        drainage_mismatches: 0,
        drain_mismatches: 0,
        lost_walks: 0,
        pending: 0,
        pending_resolve: Duration::ZERO,
        level: path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default(),
        build,
        columns: columns.len(),
        spans: graph.span_count(),
        multi_span_columns: columns
            .iter()
            .filter(|c| graph.spans(**c).len() > 1)
            .count(),
        parity_repairs: rasteriser.stats().parity_repairs,
        floor_above_truth: 0,
        worst_above: 0.0,
        compared: 0,
        close: 0,
        close_with_rim: 0,
        ray_disagreements: 0,
        first_disagreement: None,
        rebuilds: Vec::new(),
        rebuilt_chunks: 0,
        invariant_violations: 0,
        floor_clamps: 0,
        worst_clamp: 0.0,
        incremental_mismatches: 0,
        first_mismatch: None,
    };

    let per_column: Vec<ColumnTruth> = columns
        .par_iter()
        .map(|&column| column_truth(&graph, &terrain, column))
        .collect();
    for truth in per_column {
        findings.floor_above_truth += truth.above;
        findings.worst_above = findings.worst_above.max(truth.worst_above);
        findings.compared += truth.compared;
        findings.close += truth.close;
        findings.close_with_rim += truth.close_with_rim;
        if let Some(text) = truth.disagreement {
            findings.ray_disagreements += 1;
            findings.first_disagreement.get_or_insert(text);
        }
    }

    for site in blast_sites(&terrain) {
        terrain.detonate(site, &BlastConfig::default());
        terrain.update();
        let rebuilt = terrain.rebuilt_chunks().to_vec();
        let triangles: usize = rebuilt
            .iter()
            .filter_map(|id| terrain.chunk_geometry(*id))
            .map(|g| g.indices.len() / 3)
            .sum();
        log::info!(
            "{} rebuilt chunks hold {triangles} triangles",
            rebuilt.len()
        );
        let started = Instant::now();
        let changed = terrain.changed_regions().to_vec();
        let remap = rasteriser.rebuild(&terrain, &mut graph, &rebuilt, &changed);
        findings.rebuilds.push(started.elapsed());
        let started = Instant::now();
        let stats = drainage.repair(&graph, &remap);
        let took = started.elapsed();
        findings.drainage_repairs.push(took);
        if took.as_secs_f64() > 0.002 {
            log::info!("slow drainage repair at {site:?}: {took:?}, {stats:?}");
        }
        for r in graph.all_refs() {
            let fill = drainage.fill(&graph, r);
            if fill < graph.span(r).floor_min {
                log::warn!(
                    "after blast at {site:?}: {r:?} fill {fill} below floor {:?}; stale {}; chunks {:?}",
                    graph.span(r),
                    remap.columns.contains(&r.column),
                    remap.chunks
                );
                break;
            }
        }
        findings.rebuilt_chunks += remap.chunks.len();
        let t = rasteriser.last_rebuild();
        log::info!(
            "rebuild {} terrain chunks: rasterise {:?} diff {:?} pair {:?} commit {:?}, {} stale columns",
            rebuilt.len(),
            t.rasterise,
            t.diff,
            t.pair,
            t.commit,
            t.stale_columns
        );
    }
    findings.invariant_violations = rasteriser.stats().invariant_violations;
    findings.floor_clamps = rasteriser.stats().floor_clamps;
    findings.worst_clamp = rasteriser.stats().worst_clamp;

    let pending = drainage.pending_flats();
    let started = Instant::now();
    drainage.resolve_pending(&graph);
    findings.pending_resolve = started.elapsed();
    findings.pending = pending;
    for start in graph.all_refs() {
        let mut span = start;
        let mut steps = 0;
        while let Some(next) = drainage.downstream(&graph, span) {
            span = next;
            steps += 1;
            if steps > 100_000 {
                break;
            }
        }
        if !matches!(drainage.drain(&graph, span), Drain::Outlet | Drain::Sealed) {
            findings.lost_walks += 1;
        }
    }
    let full = DrainageField::build(&graph, outlets.clone());
    findings.drain_mismatches = graph
        .all_refs()
        .filter(|r| full.drain(&graph, *r) != drainage.drain(&graph, *r))
        .count();
    for r in graph.all_refs() {
        if full.fill(&graph, r) != drainage.fill(&graph, r) {
            findings.drainage_mismatches += 1;
            log::warn!(
                "drainage mismatch at {:?}: repaired {} vs full {} ({:?})",
                r,
                drainage.fill(&graph, r),
                full.fill(&graph, r),
                graph.span(r)
            );
        }
    }

    let (_, fresh) = SpanRasteriser::build(&terrain);
    for column in occupied_columns(&fresh)
        .into_iter()
        .chain(occupied_columns(&graph))
    {
        if !same_up_to_clamps(fresh.spans(column), graph.spans(column)) {
            findings.incremental_mismatches += 1;
            findings.first_mismatch.get_or_insert_with(|| {
                format!(
                    "column ({}, {}): incremental {:?} vs full {:?}",
                    column.i,
                    column.k,
                    graph.spans(column),
                    fresh.spans(column)
                )
            });
        }
    }
    Ok(findings)
}

/// Whether incrementally rebuilt spans match a full build, allowing floors the
/// rebuild held below a re-triangulation rise.
fn same_up_to_clamps(full: &[Span], incremental: &[Span]) -> bool {
    full.len() == incremental.len()
        && full.iter().zip(incremental).all(|(f, i)| {
            f.ceiling == i.ceiling
                && f.floor_max == i.floor_max
                && i.floor_c <= f.floor_c
                && i.floor_min <= f.floor_min
                && f.floor_c - i.floor_c <= 0.5
        })
}

/// Columns holding at least one span.
fn occupied_columns(graph: &SpanGraph) -> Vec<Column> {
    let (min, max) = graph.column_bounds();
    (min.k..=max.k)
        .flat_map(|k| (min.i..=max.i).map(move |i| Column::new(i, k)))
        .filter(|c| !graph.spans(*c).is_empty())
        .collect()
}

/// The lowest floor hit in each span's band over a set of sample points.
fn band_minima(
    spans: &[Span],
    terrain: &TerrainWorld,
    points: impl Iterator<Item = (f32, f32)>,
) -> Vec<f32> {
    let mut minima = vec![f32::INFINITY; spans.len()];
    for (x, z) in points {
        for hit in terrain.mesh_hits_at(x, z) {
            if hit.normal.y <= 0.0 {
                continue;
            }
            let y = hit.point.y;
            if let Some(band) = spans.iter().position(|s| s.ceiling > y) {
                minima[band] = minima[band].min(y);
            }
        }
    }
    minima
}

struct ColumnTruth {
    above: usize,
    worst_above: f32,
    compared: usize,
    close: usize,
    close_with_rim: usize,
    disagreement: Option<String>,
}

fn column_truth(graph: &SpanGraph, terrain: &TerrainWorld, column: Column) -> ColumnTruth {
    let spans = graph.spans(column);
    let (x0, z0) = column.min_corner();
    let step = COLUMN_SIZE / SUBSAMPLES as f32;
    // The design's ground truth: sub-sample centres, which never reach the
    // square's rim.
    let interior = (0..SUBSAMPLES).flat_map(|sk| {
        (0..SUBSAMPLES)
            .map(move |si| (x0 + (si as f32 + 0.5) * step, z0 + (sk as f32 + 0.5) * step))
    });
    let sampled_min = band_minima(spans, terrain, interior);
    // The same density with the rim included, where a cliff's lowest floor is.
    let rim = (0..=SUBSAMPLES).flat_map(|sk| {
        (0..=SUBSAMPLES).map(move |si| (x0 + si as f32 * step, z0 + sk as f32 * step))
    });
    let rim_min = band_minima(spans, terrain, rim);

    let mut truth = ColumnTruth {
        above: 0,
        worst_above: 0.0,
        compared: 0,
        close: 0,
        close_with_rim: 0,
        disagreement: None,
    };
    for (span, &min) in spans.iter().zip(&rim_min) {
        if min.is_finite() && min.min(span.floor_c) - span.floor_min <= CLOSE {
            truth.close_with_rim += 1;
        }
    }
    for (span, &min) in spans.iter().zip(&sampled_min) {
        if !min.is_finite() {
            continue;
        }
        truth.compared += 1;
        let above = span.floor_min - min;
        if above > ABOVE_TOLERANCE {
            truth.above += 1;
            truth.worst_above = truth.worst_above.max(above);
        }
        if min - span.floor_min <= CLOSE {
            truth.close += 1;
        }
    }
    truth.disagreement = centre_disagreement(graph, terrain, column);
    truth
}

/// Compare the rasterised spans with a vertical ray through the centre.
fn centre_disagreement(
    graph: &SpanGraph,
    terrain: &TerrainWorld,
    column: Column,
) -> Option<String> {
    let (x, z) = column.centre();
    let mut hits: Vec<(f32, bool)> = terrain
        .mesh_hits_at(x, z)
        .into_iter()
        .filter(|h| h.normal.y.abs() > 1e-6)
        .map(|h| (h.point.y, h.normal.y > 0.0))
        .collect();
    hits.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut ray_spans: Vec<(f32, f32)> = Vec::new();
    let mut floor = None;
    for (y, up) in hits {
        match (up, floor) {
            (true, None) => floor = Some(y),
            (false, Some(f)) => {
                ray_spans.push((f, y));
                floor = None;
            }
            _ => {}
        }
    }
    if let Some(f) = floor {
        ray_spans.push((f, f32::INFINITY));
    }
    let spans = graph.spans(column);
    let agree = ray_spans.len() == spans.len()
        && ray_spans.iter().zip(spans).all(|((f, c), s)| {
            (f - s.floor_c).abs() < 1e-3 && (c == &s.ceiling || (c - s.ceiling).abs() < 1e-3)
        });
    (!agree).then(|| {
        format!(
            "column ({}, {}) at ({x:.2}, {z:.2}): ray {:?} vs spans {:?}",
            column.i,
            column.k,
            ray_spans,
            spans
                .iter()
                .map(|s| (s.floor_c, s.ceiling))
                .collect::<Vec<_>>()
        )
    })
}

/// Grenade sites across the level: a coarse grid of surface points.
fn blast_sites(terrain: &TerrainWorld) -> Vec<Point3<f32>> {
    let bounds = terrain.bounds();
    let margin = 8.0;
    let spacing = 11.0;
    let mut sites = Vec::new();
    let mut z = bounds.min.z + margin;
    while z <= bounds.max.z - margin {
        let mut x = bounds.min.x + margin;
        while x <= bounds.max.x - margin {
            if let Some(y) = terrain.mesh_surface_height_at(x, z) {
                sites.push(Point3::new(x, y, z));
            }
            x += spacing;
        }
        z += spacing;
    }
    sites.truncate(BLASTS);
    sites
}

/// The findings as a report, with each kill criterion's verdict.
pub fn report(f: &SpanFindings) -> String {
    let mut out = String::new();
    let close = if f.compared == 0 {
        1.0
    } else {
        f.close as f64 / f.compared as f64
    };
    let rebuild_p99 = percentile_ms(&f.rebuilds, 0.99);
    let pass = |ok: bool| if ok { "pass" } else { "FAIL" };
    let _ = writeln!(out, "{}", f.level);
    let _ = writeln!(
        out,
        "  build {:.1} ms: {} columns, {} spans, {} columns with more than one",
        f.build.as_secs_f64() * 1000.0,
        f.columns,
        f.spans,
        f.multi_span_columns
    );
    let _ = writeln!(
        out,
        "  floor_min above sampled minimum: {} spans (worst {:.4} m)   [{}]",
        f.floor_above_truth,
        f.worst_above,
        pass(f.floor_above_truth == 0)
    );
    let _ = writeln!(
        out,
        "  floor_min within {CLOSE} m of sampled minimum: {:.2}% of {} spans   [{}]",
        close * 100.0,
        f.compared,
        pass(close >= 0.99)
    );
    let _ = writeln!(
        out,
        "  ... with the square's rim sampled too: {:.2}%",
        f.close_with_rim as f64 / f.compared.max(1) as f64 * 100.0
    );
    let _ = writeln!(
        out,
        "  parity repairs: {}   [{}]",
        f.parity_repairs,
        pass(f.parity_repairs == 0)
    );
    let _ = writeln!(
        out,
        "  centre ray disagrees in {} columns{}",
        f.ray_disagreements,
        f.first_disagreement
            .as_ref()
            .map(|t| format!(", e.g. {t}"))
            .unwrap_or_default()
    );
    let _ = writeln!(
        out,
        "  re-pair after {} grenades: mean {:.3} ms, p99 {:.3}, max {:.3}; {:.1} span chunks each   [{}]",
        f.rebuilds.len(),
        mean_ms(&f.rebuilds),
        rebuild_p99,
        max_ms(&f.rebuilds),
        f.rebuilt_chunks as f64 / f.rebuilds.len().max(1) as f64,
        pass(max_ms(&f.rebuilds) <= 1.0)
    );
    let _ = writeln!(
        out,
        "  floors held against re-triangulation: {} (worst rise {:.2} voxel); violations beyond it: {}   [{}]",
        f.floor_clamps,
        f.worst_clamp,
        f.invariant_violations,
        pass(f.invariant_violations == 0)
    );
    let _ = writeln!(
        out,
        "  drainage: build {:.1} ms, {} sealed spans; repair mean {:.3} ms, max {:.3}; {} fills differ from a full build   [{}]",
        f.drainage_build.as_secs_f64() * 1000.0,
        f.sealed,
        mean_ms(&f.drainage_repairs),
        max_ms(&f.drainage_repairs),
        f.drainage_mismatches,
        pass(f.drainage_mismatches == 0)
    );
    let _ = writeln!(
        out,
        "  drains: {} pending flat levels resolved in {:.1} ms; {} drains differ from a full build; {} walks lost   [{}]",
        f.pending,
        f.pending_resolve.as_secs_f64() * 1000.0,
        f.drain_mismatches,
        f.lost_walks,
        pass(f.lost_walks == 0)
    );
    let _ = writeln!(
        out,
        "  incremental vs full rebuild: {} columns differ{}   [{}]",
        f.incremental_mismatches,
        f.first_mismatch
            .as_ref()
            .map(|t| format!(", e.g. {t}"))
            .unwrap_or_default(),
        pass(f.incremental_mismatches == 0)
    );
    out
}
