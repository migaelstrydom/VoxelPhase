//! The router: lays a channel of reaches from an outlet down the drainage
//! field to the store it reaches (§7.4, §7.5).
//!
//! ```text
//!   outlet crest ──walk drains──▶ cells ──cut 8–16 m──▶ reaches ──▶ target store
//!   or a landing                  │                      rating curve per reach
//!                                 ends at: a basin's water, an existing reach
//!                                 (a junction), the void, a real depression,
//!                                 or a fall step, where the water leaves the bed
//! ```

use nalgebra::Point3;

use crate::water::geometry::{Drain, SpanGraph, SpanRef};
use crate::water::ids::StoreId;
use crate::water::network::{
    is_pothole, Centreline, CrossSection, RatingCurve, Reach, Store, FALL_RUN, FALL_THRESHOLD,
};

/// Reaches are cut at about this length, m.
pub const REACH_LENGTH: f32 = 12.0;

/// A final piece shorter than this joins the reach before it, m.
pub const MIN_REACH_LENGTH: f32 = 8.0;

/// A path shorter than this is no channel: the outflow links straight to its
/// target, m.
pub const MIN_CHANNEL_LENGTH: f32 = 3.0;

/// Half-width of the cross-sections a rating curve is scanned from, m.
pub const SECTION_HALF_WIDTH: f32 = 8.0;

/// Sections are cut this far apart along a reach, m.
pub const SECTION_SPACING: f32 = 1.0;

/// Distance each side over which a section's bed slope is measured, m.
const SLOPE_WINDOW: f32 = 3.0;

/// Steps a walk takes before giving up.
const MAX_WALK: usize = 1 << 16;

/// How a walk down the drainage field ended.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WalkEnd {
    /// Into a store, at this span: a basin's water, or an existing reach.
    Store(StoreId, SpanRef),
    /// Off an edge of the world, or into a sink, at this span.
    Void(SpanRef),
    /// Into a dry depression, at this span.
    Depression(SpanRef),
    /// Over a fall step: the bed drops more than `FALL_THRESHOLD` within
    /// `FALL_RUN` of `lip`, the last cell of the channel.
    Fall {
        lip: SpanRef,
        /// The first cell past the lip, which gives the fall its direction.
        toward: SpanRef,
    },
    /// Nowhere found within the step limit.
    Lost,
}

/// The crest a walk starts over, when it leaves a basin: the lip's inside
/// span and the crest height, so a riser right at the crest is a fall.
#[derive(Debug, Clone, Copy)]
pub struct WalkLip {
    pub inside: SpanRef,
    pub height: f32,
}

/// The cells a channel from `start` runs over, and where it ends. The walk
/// stops at the first span under a basin's water, at a span already carrying
/// a reach, at the void, at a real depression and at a fall step. Whether
/// water falls from its last cell into a basin is its end link's lip's to
/// say (§7.9), not the walk's. `from` is
/// the basin the channel leaves: its own spans do not end it.
pub fn walk(
    graph: &SpanGraph,
    drainage: &mut crate::water::geometry::DrainageField,
    network: &crate::water::network::Network,
    levels: &dyn Fn(StoreId) -> Option<f32>,
    from: Option<StoreId>,
    lip: Option<WalkLip>,
    start: SpanRef,
) -> (Vec<SpanRef>, WalkEnd) {
    let mut cells: Vec<SpanRef> = Vec::new();
    let mut span = start;
    for _ in 0..MAX_WALK {
        if let Some((at, toward)) = fall_step(graph, lip, &cells, span) {
            cells.truncate(at.map_or(0, |i| i + 1));
            let lip = match at {
                Some(i) => cells[i],
                None => {
                    lip.expect("a fall before the first cell is at the lip")
                        .inside
                }
            };
            return (cells, WalkEnd::Fall { lip, toward });
        }
        let owner = graph.owner(span);
        let floor = graph.span(span).floor_min;
        if let Some((reach, _)) = owner.reach {
            if network.store(reach).is_some() {
                return (cells, WalkEnd::Store(reach, span));
            }
        }
        if let Some(body) = owner.body.filter(|b| Some(*b) != from) {
            if levels(body).is_some_and(|level| level > floor) {
                return (cells, WalkEnd::Store(body, span));
            }
        }
        cells.push(span);
        let fill = drainage.fill(graph, span);
        if drainage.drain(graph, span) == Drain::Outlet {
            return (cells, WalkEnd::Void(span));
        }
        if fill > floor + 1e-3 && (!fill.is_finite() || !is_pothole(graph, span, fill)) {
            return (cells, WalkEnd::Depression(span));
        }
        match drainage.downstream_resolved(graph, span) {
            Some(next) => span = next,
            None => return (cells, WalkEnd::Void(span)),
        }
    }
    (cells, WalkEnd::Lost)
}

/// Whether stepping onto `next` goes over a fall: some cell within
/// `FALL_RUN` behind it, or the lip, stands more than `FALL_THRESHOLD` above
/// it. Returns the highest such cell (`None` for the lip) and the cell after
/// it.
fn fall_step(
    graph: &SpanGraph,
    lip: Option<WalkLip>,
    cells: &[SpanRef],
    next: SpanRef,
) -> Option<(Option<usize>, SpanRef)> {
    let floor = graph.span(next).floor_c;
    let (nx, nz) = next.column.centre();
    let within = |column: crate::water::geometry::Column| {
        let (x, z) = column.centre();
        ((x - nx).powi(2) + (z - nz).powi(2)).sqrt() <= FALL_RUN + 1e-3
    };
    let mut best: Option<(Option<usize>, f32)> = None;
    for (i, cell) in cells.iter().enumerate().rev() {
        if !within(cell.column) {
            break;
        }
        let height = graph.span(*cell).floor_c;
        if height - floor > FALL_THRESHOLD && best.is_none_or(|(_, h)| height > h) {
            best = Some((Some(i), height));
        }
    }
    if let Some(lip) = lip {
        let near = cells.iter().all(|c| within(c.column));
        if near && within(lip.inside.column) && lip.height - floor > FALL_THRESHOLD {
            if best.is_none_or(|(_, h)| lip.height > h) {
                best = Some((None, lip.height));
            }
        }
    }
    let (at, _) = best?;
    let toward = match at {
        Some(i) => cells.get(i + 1).copied().unwrap_or(next),
        None => cells.first().copied().unwrap_or(next),
    };
    Some((at, toward))
}

/// Cut a path into reaches and build each: its centreline, sections and
/// rating curve at `q_design`.
pub fn build_reaches(graph: &SpanGraph, cells: &[SpanRef], q_design: f64) -> Vec<Reach> {
    let points: Vec<Point3<f32>> = cells
        .iter()
        .map(|c| {
            let (x, z) = c.column.centre();
            Point3::new(x, graph.span(*c).floor_c, z)
        })
        .collect();
    let mut distance = vec![0.0f32];
    for pair in points.windows(2) {
        let d = ((pair[1].x - pair[0].x).powi(2) + (pair[1].z - pair[0].z).powi(2)).sqrt();
        distance.push(distance.last().unwrap() + d);
    }
    let total = *distance.last().unwrap_or(&0.0);
    if total < MIN_CHANNEL_LENGTH {
        return Vec::new();
    }

    // Cut points: every REACH_LENGTH, with a short tail folded into the
    // reach before it.
    let mut cuts = vec![0usize];
    let mut next = REACH_LENGTH;
    for (i, d) in distance.iter().enumerate() {
        if *d >= next && total - *d >= MIN_REACH_LENGTH {
            cuts.push(i);
            next = *d + REACH_LENGTH;
        }
    }
    cuts.push(points.len() - 1);

    cuts.windows(2)
        .map(|w| {
            let (a, b) = (w[0], w[1]);
            // Each reach includes the first cell of the next, so the
            // channel is continuous.
            let reach_points = &points[a..=b];
            let reach_cells = cells[a..=b].to_vec();
            let line = Centreline::from_path(reach_points);
            let sections = sections_along(graph, &line);
            let rating = RatingCurve::scan(&sections, q_design);
            Reach::new(reach_cells, reach_points, rating, Vec::new())
        })
        .collect()
}

/// One reach over `cells`, however long, with its rating scanned at
/// `q_design`: what is left of a reach cut short. `None` for fewer than two
/// cells, which is no channel.
pub fn one_reach(graph: &SpanGraph, cells: &[SpanRef], q_design: f64) -> Option<Reach> {
    if cells.len() < 2 {
        return None;
    }
    let points: Vec<Point3<f32>> = cells
        .iter()
        .map(|c| {
            let (x, z) = c.column.centre();
            Point3::new(x, graph.span(*c).floor_c, z)
        })
        .collect();
    let line = Centreline::from_path(&points);
    let rating = RatingCurve::scan(&sections_along(graph, &line), q_design);
    Some(Reach::new(cells.to_vec(), &points, rating, Vec::new()))
}

/// A reach's rating scanned again at a new design discharge, once the flow
/// down it outgrows the old one.
pub fn rescan(graph: &SpanGraph, reach: &Reach, q_design: f64) -> RatingCurve {
    RatingCurve::scan(&sections_along(graph, &reach.centreline), q_design)
}

/// Cross-sections every `SECTION_SPACING` along a centreline, each with its
/// bed slope.
fn sections_along(graph: &SpanGraph, line: &Centreline) -> Vec<(CrossSection, f32)> {
    let mut out = Vec::new();
    let mut next = 0.0;
    for (i, point) in line.points.iter().enumerate() {
        if line.distance[i] < next {
            continue;
        }
        next = line.distance[i] + SECTION_SPACING;
        let section = CrossSection::sample(graph, *point, line.tangents[i], SECTION_HALF_WIDTH);
        out.push((section, slope_at(line, i)));
    }
    out
}

/// Bed slope at a point of a centreline, over `SLOPE_WINDOW` each side.
fn slope_at(line: &Centreline, i: usize) -> f32 {
    let d = line.distance[i];
    let before = line.distance.partition_point(|x| *x < d - SLOPE_WINDOW);
    let after = line
        .distance
        .partition_point(|x| *x <= d + SLOPE_WINDOW)
        .saturating_sub(1)
        .max(i);
    let run = line.distance[after] - line.distance[before];
    if run <= 0.0 {
        return 0.0;
    }
    ((line.points[before].y - line.points[after].y) / run).max(0.0)
}

/// The spans each reach cell's water covers at its design discharge: the
/// cross-section's wet samples, keyed by cell index along the reach.
pub fn reach_footprint(graph: &SpanGraph, reach: &Reach) -> Vec<(SpanRef, u16, f32)> {
    let design = reach.rating.at(reach.rating.design());
    let half = (design.top_width * 0.5 + 0.5).min(SECTION_HALF_WIDTH);
    let level_over_bed = design.depth;
    let mut out = Vec::new();
    for (i, point) in reach.centreline.points.iter().enumerate() {
        let tangent = reach.centreline.tangents[i];
        let across = nalgebra::Vector2::new(-tangent.y, tangent.x);
        let cell = reach
            .cell_distance
            .partition_point(|d| *d <= reach.centreline.distance[i])
            .saturating_sub(1) as u16;
        let steps = (half / 0.25).ceil() as i32;
        for s in -steps..=steps {
            let offset = across * (s as f32 * 0.25);
            let (x, z) = (point.x + offset.x, point.z + offset.y);
            let column = crate::water::geometry::Column::containing(x, z);
            let Some(span) = graph.span_at(column, point.y + 0.5) else {
                continue;
            };
            if graph.span(span).floor_min < point.y + level_over_bed {
                out.push((span, cell, offset.norm()));
            }
        }
    }
    out
}

/// The store a reach lets its water into: the down end of its outflow link.
pub fn downstream_of(network: &crate::water::network::Network, reach: StoreId) -> Option<StoreId> {
    network
        .links()
        .find(|(_, l)| {
            l.up == reach
                && network
                    .store(reach)
                    .is_some_and(|s| matches!(s, Store::Reach(_)))
        })
        .map(|(_, l)| l.down)
}
