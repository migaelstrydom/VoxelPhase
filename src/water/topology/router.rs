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

use std::collections::VecDeque;

use nalgebra::Point3;

use crate::water::geometry::{Drain, SpanGraph, SpanRef};
use crate::water::ids::StoreId;
use crate::water::network::{
    downhill, floor_near, is_pothole, Centreline, CrossSection, Network, RatingCurve, Reach, Store,
    FALL_RUN, FALL_THRESHOLD,
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

/// Distance each side over which a channel's move to the middle of its
/// water is averaged, m.
const CENTRE_WINDOW: f32 = 1.5;

/// Distance over which a channel eases from the middle of its water back to
/// the cell it starts or ends on, where it meets a store or a lip, m.
const CENTRE_EASE: f32 = 2.0;

/// The discharge a channel's line is centred on the water of, m³/s: fixed,
/// so that the line depends on the ground alone, not on the flow it was
/// laid for.
pub const CENTRE_DISCHARGE: f64 = 0.5;

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
/// follows the drainage field, and inside a real depression, below where it
/// would spill, runs straight down the pit's dry side instead. It stops at
/// the first span under a basin's water, at a span already carrying a reach,
/// at the void, at the bottom of a dry pit and at a fall step. Whether
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
    // The rest of a way across a flat, still to walk.
    let mut ahead: VecDeque<SpanRef> = VecDeque::new();
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
        // Partway across a flat on a pit's side.
        if let Some(next) = ahead.pop_front() {
            span = next;
            continue;
        }
        if fill > floor + 1e-3 && (!fill.is_finite() || !is_pothole(graph, span, fill)) {
            // In a pit, below where it would spill: the water runs on down
            // the pit's dry side to the water standing in it, or to its
            // bottom, where an empty basin is made.
            let mut path = downhill(graph, span).into_iter();
            match path.next() {
                Some(next) => {
                    ahead.extend(path);
                    span = next;
                    continue;
                }
                None => return (cells, WalkEnd::Depression(span)),
            }
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
pub fn build_reaches(
    graph: &SpanGraph,
    cells: &[SpanRef],
    q_design: f64,
    standing: &dyn Fn(SpanRef) -> bool,
) -> Vec<Reach> {
    // The start eases back to the cell it leaves its store by; the end stays
    // on the water, where the lip it leaves over stands.
    let points = centre_on_water(graph, cells, CENTRE_DISCHARGE, standing, (true, false));
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
            let sections = sections_along(graph, &line, standing);
            let rating = RatingCurve::scan(&sections, q_design);
            Reach::new(reach_cells, reach_points, rating, Vec::new())
        })
        .collect()
}

/// A point on the bed for each of a channel's cells, moved across its
/// water to the middle of the surface carrying `q_design`: where its
/// centreline belongs. The walk follows the drainage field, which runs
/// straight down a flat-bottomed bed from wherever it entered it, along the
/// foot of a bank as readily as down the middle; a centreline left there
/// stands above the water it carries. The move is averaged along the
/// channel, and eased back to the cell at each end flagged in `ease`
/// (start, end), where the channel meets a store or leaves over a lip.
fn centre_on_water(
    graph: &SpanGraph,
    cells: &[SpanRef],
    q_design: f64,
    standing: &dyn Fn(SpanRef) -> bool,
    ease: (bool, bool),
) -> Vec<Point3<f32>> {
    let points: Vec<Point3<f32>> = cells
        .iter()
        .map(|c| {
            let (x, z) = c.column.centre();
            Point3::new(x, graph.span(*c).floor_c, z)
        })
        .collect();
    if points.len() < 2 {
        return points;
    }
    let line = Centreline::from_path(&points);
    // Each cell's nearest point on the smoothed line, which gives it a
    // tangent to cut its section across and a bed slope.
    let mut j = 0;
    let nearest: Vec<usize> = points
        .iter()
        .map(|p| {
            let d = |k: usize| (line.points[k] - p).xz().norm_squared();
            while j + 1 < line.points.len() && d(j + 1) <= d(j) {
                j += 1;
            }
            j
        })
        .collect();
    let across: Vec<nalgebra::Vector2<f32>> = nearest
        .iter()
        .map(|&j| nalgebra::Vector2::new(-line.tangents[j].y, line.tangents[j].x))
        .collect();
    let distance: Vec<f32> = nearest.iter().map(|&j| line.distance[j]).collect();
    // The middle of the water, found every `SECTION_SPACING`.
    let mut middles: Vec<(f32, f32)> = Vec::new();
    for (i, p) in points.iter().enumerate() {
        let last = i + 1 == points.len();
        if middles
            .last()
            .is_some_and(|(d, _)| distance[i] - d < SECTION_SPACING)
            && !last
        {
            continue;
        }
        let j = nearest[i];
        let middle =
            CrossSection::sample(graph, *p, line.tangents[j], SECTION_HALF_WIDTH, standing)
                .water_middle(q_design as f32, slope_at(&line, j))
                .unwrap_or(0.0);
        middles.push((distance[i], middle));
    }
    let total = line.length();
    points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let d = distance[i];
            let near = &middles[middles.partition_point(|(at, _)| *at < d - CENTRE_WINDOW)
                ..middles.partition_point(|(at, _)| *at <= d + CENTRE_WINDOW)];
            let (sum, n) = near.iter().fold((0.0, 0), |(s, n), (_, o)| (s + o, n + 1));
            let mut offset = sum / n.max(1) as f32;
            if ease.0 {
                offset *= (d / CENTRE_EASE).min(1.0);
            }
            if ease.1 {
                offset *= ((total - d) / CENTRE_EASE).clamp(0.0, 1.0);
            }
            let (x, z) = (p.x + across[i].x * offset, p.z + across[i].y * offset);
            let y = floor_near(graph, x, z, p.y).map_or(p.y, |(_, floor)| floor);
            Point3::new(x, y, z)
        })
        .collect()
}

/// A reach's rating scanned again at a new design discharge, once the flow
/// down it outgrows the old one.
pub fn rescan(
    graph: &SpanGraph,
    reach: &Reach,
    q_design: f64,
    standing: &dyn Fn(SpanRef) -> bool,
) -> RatingCurve {
    RatingCurve::scan(
        &sections_along(graph, &reach.centreline, standing),
        q_design,
    )
}

/// Cross-sections every `SECTION_SPACING` along a centreline, each with its
/// bed slope, sampling no span where `standing` says a store's water stands.
fn sections_along(
    graph: &SpanGraph,
    line: &Centreline,
    standing: &dyn Fn(SpanRef) -> bool,
) -> Vec<(CrossSection, f32)> {
    let mut out = Vec::new();
    let mut next = 0.0;
    for (i, point) in line.points.iter().enumerate() {
        if line.distance[i] < next {
            continue;
        }
        next = line.distance[i] + SECTION_SPACING;
        let section = CrossSection::sample(
            graph,
            *point,
            line.tangents[i],
            SECTION_HALF_WIDTH,
            standing,
        );
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

/// The stores a reach's channel drains from: every store above it, up
/// through the reaches feeding it, that is not a reach.
pub fn channel_heads(network: &Network, id: StoreId) -> Vec<StoreId> {
    let mut heads = Vec::new();
    let mut above = vec![id];
    let mut cursor = 0;
    while cursor < above.len() {
        let current = above[cursor];
        cursor += 1;
        for (_, link) in network.links().filter(|(_, l)| l.down == current) {
            let is_reach = network.store(link.up).and_then(Store::as_reach).is_some();
            let seen = if is_reach { &mut above } else { &mut heads };
            if !seen.contains(&link.up) {
                seen.push(link.up);
            }
        }
    }
    heads
}

/// Whether the water of a body other than `heads` stands over a span now:
/// where a channel's cross-section leaves the channel for a lake.
pub fn standing_in<'a>(
    graph: &'a SpanGraph,
    network: &'a Network,
    heads: &'a [StoreId],
) -> impl Fn(SpanRef) -> bool + 'a {
    move |span| {
        graph
            .owner(span)
            .body
            .filter(|b| !heads.contains(b))
            .and_then(|b| network.store(b))
            .and_then(Store::surface)
            .is_some_and(|level| level > graph.span(span).floor_min)
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::geometry::{Column, Span, SpanChunk, SpanChunkCoord, COLUMNS_PER_CHUNK};

    #[test]
    fn a_channel_down_the_foot_of_a_bank_is_centred_on_its_water() {
        // A 5 m flat-bottomed bed along i, from k = 3 to k = 12, falling
        // 0.05 m a column, under banks 0.2 m up and then walls 2 m up. The
        // walk has run down it along the foot of the k = 3 side.
        let coord = SpanChunkCoord { x: 0, z: 0 };
        let mut graph = SpanGraph::new(coord, coord);
        let columns: Vec<Vec<Span>> = (0..COLUMNS_PER_CHUNK)
            .map(|local| {
                let c = coord.column(local);
                let bed = 5.0 - 0.05 * c.i as f32;
                let f = match c.k {
                    3..=12 => bed,
                    2 | 13 => bed + 0.2,
                    _ => bed + 2.0,
                };
                vec![Span {
                    floor_c: f,
                    floor_min: f,
                    floor_max: f,
                    ceiling: f32::INFINITY,
                }]
            })
            .collect();
        graph.replace_chunk(coord, SpanChunk::from_columns(&columns, 1));
        let cells: Vec<SpanRef> = (0..16)
            .map(|i| graph.span_at(Column::new(i, 3), 10.0).unwrap())
            .collect();
        let points = centre_on_water(&graph, &cells, 1.0, &|_| false, (true, true));
        let middle = Column::new(0, 7).centre().1 + 0.25;
        for (i, p) in points.iter().enumerate() {
            let d = i as f32 * 0.5;
            if (CENTRE_EASE..7.5 - CENTRE_EASE).contains(&d) {
                assert!((p.z - middle).abs() < 0.5, "cell {i} at z {}", p.z);
            }
            let bed = 5.0 - 0.05 * i as f32;
            assert!((p.y - bed).abs() < 1e-4, "cell {i} stands on the bed");
        }
        // Each end eases back to its cell.
        assert!((points[0].z - cells[0].column.centre().1).abs() < 1e-4);
        assert!((points[15].z - cells[15].column.centre().1).abs() < 1e-4);
    }
}
