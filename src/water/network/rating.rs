//! Cross-sections and rating curves: how deep, wide and fast a given
//! discharge runs in a channel.
//!
//! A [`CrossSection`] is the channel bed sampled across the smoothed tangent
//! at one point of a centreline. Manning's equation,
//! `Q = (1/n) · A · R^(2/3) · √S`, then gives the depth at which a discharge
//! `Q` runs there, and with it the flow area, top width and velocity. Only
//! the wet run of the section connected to the centre counts: water does not
//! stand in a ditch beyond the bank.

use nalgebra::{Point3, Vector2};

use crate::water::geometry::{Column, SpanGraph, SpanRef};

/// Manning's roughness for a natural earth channel.
pub const MANNING_N: f32 = 0.035;

/// The gentlest bed slope routing will assume. A dead-flat reach still moves
/// its water, slowly, rather than dividing by zero.
pub const MIN_SLOPE: f32 = 1e-3;

/// Spacing of cross-section samples, in metres. Half a column.
pub const SAMPLE_SPACING: f32 = 0.25;

/// The bed sampled across a channel.
#[derive(Debug, Clone)]
pub struct CrossSection {
    /// Floor height at each sample, left bank to right; `None` where the
    /// section leaves the level or meets a wall with no floor at bed height.
    pub floors: Vec<Option<f32>>,
    /// Index of the sample on the centreline.
    pub centre: usize,
    /// Distance between samples.
    pub spacing: f32,
}

/// What a discharge does at one cross-section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hydraulics {
    /// Height of the surface over the bed on the centreline, which is where
    /// a reach's surface is drawn from and sampled at. Measured from the
    /// section's lowest floor instead, a section reaching across a shore
    /// into a lake would read the lake's depth.
    pub depth: f32,
    /// Water surface height.
    pub level: f32,
    /// Flow area, m².
    pub area: f32,
    /// Width of the water surface, m.
    pub top_width: f32,
    /// Mean velocity, m/s.
    pub velocity: f32,
}

impl CrossSection {
    /// Sample the bed across `tangent` at `centre`, out to `half_width` each
    /// side. Each sample reads the floor of the span resting nearest
    /// `centre.y` in its column. A span where `standing` says another
    /// store's water stands is no sample: past a shore with no bank, a
    /// section running on into a lake would count the lake's water as the
    /// channel's.
    pub fn sample(
        graph: &SpanGraph,
        centre: Point3<f32>,
        tangent: Vector2<f32>,
        half_width: f32,
        standing: &dyn Fn(SpanRef) -> bool,
    ) -> Self {
        let across = Vector2::new(-tangent.y, tangent.x);
        let steps = (half_width / SAMPLE_SPACING).round() as i32;
        let floors = (-steps..=steps)
            .map(|i| {
                let offset = across * (i as f32 * SAMPLE_SPACING);
                floor_near(graph, centre.x + offset.x, centre.z + offset.y, centre.y)
                    .filter(|(span, _)| !standing(*span))
                    .map(|(_, floor)| floor)
            })
            .collect();
        Self {
            floors,
            centre: steps as usize,
            spacing: SAMPLE_SPACING,
        }
    }

    /// Area, wetted perimeter and top width with the surface at `level`,
    /// counting only the wet run connected to the centre.
    pub fn geometry_at(&self, level: f32) -> (f32, f32, f32) {
        let (lo, hi) = self.wet_run(level);
        let mut area = 0.0;
        let mut perimeter = 0.0;
        let mut width = 0.0;
        for i in lo..hi {
            let (Some(a), Some(b)) = (self.floors[i], self.floors[i + 1]) else {
                continue;
            };
            let (da, db) = ((level - a).max(0.0), (level - b).max(0.0));
            if da <= 0.0 && db <= 0.0 {
                continue;
            }
            // The bed is linear between samples, so a partly wet interval is
            // wet over the fraction of it below the level.
            let wet = if da > 0.0 && db > 0.0 {
                1.0
            } else {
                (level - a.min(b)) / (a - b).abs().max(1e-6)
            };
            let dx = self.spacing * wet;
            area += 0.5 * (da + db) * dx;
            perimeter += (dx * dx + ((a - b) * wet).powi(2)).sqrt();
            width += dx;
        }
        (area, perimeter, width)
    }

    /// The span of samples, as `[lo, hi]`, wet at `level` and connected to
    /// the centre.
    fn wet_run(&self, level: f32) -> (usize, usize) {
        let wet = |i: usize| self.floors[i].is_some_and(|f| f < level);
        let mut lo = self.centre;
        while lo > 0 && wet(lo - 1) {
            lo -= 1;
        }
        let mut hi = self.centre;
        while hi + 1 < self.floors.len() && wet(hi + 1) {
            hi += 1;
        }
        // Include the bank sample on each side, so the waterline is found
        // between it and the last wet sample.
        (lo.saturating_sub(1), (hi + 1).min(self.floors.len() - 1))
    }

    /// The lowest floor in the section, and its offset from the centre in
    /// metres (positive to the left of travel).
    pub fn lowest(&self) -> Option<(f32, f32)> {
        self.floors
            .iter()
            .enumerate()
            .filter_map(|(i, f)| f.map(|f| (f, i)))
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(f, i)| (f, (i as f32 - self.centre as f32) * self.spacing))
    }

    /// The flow of discharge `q` at bed slope `slope` by Manning's equation,
    /// or `None` if the section cannot hold it before spilling past the
    /// sampled width.
    pub fn hydraulics(&self, q: f32, slope: f32) -> Option<Hydraulics> {
        let bed = self.floors[self.centre]?;
        let slope = slope.max(MIN_SLOPE);
        let conveyance = |level: f32| {
            let (area, perimeter, _) = self.geometry_at(level);
            if area <= 0.0 || perimeter <= 0.0 {
                return 0.0;
            }
            area * (area / perimeter).powf(2.0 / 3.0) * slope.sqrt() / MANNING_N
        };
        // Bisection on the level: conveyance rises with it.
        let (mut lo, mut hi) = (bed, bed + 0.05);
        let mut guard = 0;
        while conveyance(hi) < q {
            hi = bed + (hi - bed) * 2.0;
            guard += 1;
            if guard > 12 {
                return None;
            }
        }
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            if conveyance(mid) < q {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let level = hi;
        let (area, _, top_width) = self.geometry_at(level);
        Some(Hydraulics {
            depth: level - bed,
            level,
            area,
            top_width,
            velocity: if area > 0.0 { q / area } else { 0.0 },
        })
    }
}

/// A reach's hydraulics at one discharge, averaged over its sections.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RatingPoint {
    /// Discharge, m³/s.
    pub q: f64,
    /// Mean flow area, which is storage per metre of wetted reach, m².
    pub area: f64,
    pub top_width: f32,
    pub depth: f32,
    pub velocity: f32,
}

/// How a reach carries discharge: area, width, depth and velocity against
/// `Q`, tabulated at log-spaced discharges between `Q_min` and `Q_design`
/// (§7.5). Below `Q_min` it extrapolates as a power law, depth ∝ Q^0.6; above
/// `Q_design` the reach is re-scanned.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RatingCurve {
    points: Vec<RatingPoint>,
}

/// Discharges a rating curve is tabulated at.
pub const RATING_POINTS: usize = 8;

/// The least discharge a rating curve is tabulated from, m³/s.
pub const RATING_Q_MIN: f64 = 0.01;

/// Exponent of depth (and area) against discharge below the table.
const LOW_FLOW_EXPONENT: f64 = 0.6;

impl RatingCurve {
    /// Scan a reach's sections at each tabulated discharge. Each section
    /// comes with its bed slope. Sections that cannot carry a discharge
    /// within their sampled width are left out of that discharge's mean.
    pub fn scan(sections: &[(CrossSection, f32)], q_design: f64) -> Self {
        let q_design = q_design.max(RATING_Q_MIN * 2.0);
        let ratio = (q_design / RATING_Q_MIN).powf(1.0 / (RATING_POINTS - 1) as f64);
        let mut points = Vec::with_capacity(RATING_POINTS);
        for i in 0..RATING_POINTS {
            let q = RATING_Q_MIN * ratio.powi(i as i32);
            let found: Vec<Hydraulics> = sections
                .iter()
                .filter_map(|(section, slope)| section.hydraulics(q as f32, *slope))
                .collect();
            if found.is_empty() {
                continue;
            }
            let n = found.len() as f64;
            let area = found.iter().map(|h| h.area as f64).sum::<f64>() / n;
            points.push(RatingPoint {
                q,
                area,
                top_width: (found.iter().map(|h| h.top_width as f64).sum::<f64>() / n) as f32,
                depth: (found.iter().map(|h| h.depth as f64).sum::<f64>() / n) as f32,
                velocity: (q / area.max(1e-6)) as f32,
            });
        }
        // Area must rise with discharge for the inverse to exist.
        for i in 1..points.len() {
            if points[i].area <= points[i - 1].area {
                points[i].area = points[i - 1].area * 1.0001;
            }
        }
        Self { points }
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// The tabulated design discharge: the largest the curve holds.
    pub fn design(&self) -> f64 {
        self.points.last().map_or(0.0, |p| p.q)
    }

    /// Hydraulics at discharge `q`.
    pub fn at(&self, q: f64) -> RatingPoint {
        let Some(first) = self.points.first() else {
            return RatingPoint {
                q,
                area: 0.0,
                top_width: 0.0,
                depth: 0.0,
                velocity: 0.0,
            };
        };
        if q <= 0.0 {
            return RatingPoint {
                q: 0.0,
                area: 0.0,
                top_width: first.top_width,
                depth: 0.0,
                velocity: 0.0,
            };
        }
        if q <= first.q || self.points.len() == 1 {
            let scale = (q / first.q).powf(LOW_FLOW_EXPONENT);
            return RatingPoint {
                q,
                area: first.area * scale,
                top_width: first.top_width,
                depth: first.depth * scale as f32,
                velocity: (q / (first.area * scale).max(1e-9)) as f32,
            };
        }
        let k = self
            .points
            .windows(2)
            .position(|w| q <= w[1].q)
            .unwrap_or(self.points.len() - 2);
        let (a, b) = (self.points[k], self.points[k + 1]);
        // Interpolate in log-log, where the rating is nearly straight.
        let t = ((q.ln() - a.q.ln()) / (b.q.ln() - a.q.ln())).max(0.0);
        let lerp = |x: f64, y: f64| (x.ln() + t * (y.ln() - x.ln())).exp();
        let area = lerp(a.area, b.area);
        RatingPoint {
            q,
            area,
            top_width: lerp(a.top_width as f64, b.top_width as f64) as f32,
            depth: lerp(a.depth as f64, b.depth as f64) as f32,
            velocity: (q / area.max(1e-9)) as f32,
        }
    }

    /// The discharge whose mean area is `area`: the inverse of the curve.
    pub fn discharge_for(&self, area: f64) -> f64 {
        if area <= 0.0 || self.points.is_empty() {
            return 0.0;
        }
        let (mut lo, mut hi) = (0.0f64, self.design().max(RATING_Q_MIN));
        while self.at(hi).area < area {
            hi *= 2.0;
            if hi > 1e6 {
                return hi;
            }
        }
        for _ in 0..50 {
            let mid = 0.5 * (lo + hi);
            if self.at(mid).area < area {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// d(area)/d(Q) at `q`.
    pub fn area_slope(&self, q: f64) -> f64 {
        let e = (q * 1e-3).max(1e-6);
        let (lo, hi) = ((q - e).max(0.0), q + e);
        ((self.at(hi).area - self.at(lo).area) / (hi - lo)).max(1e-9)
    }
}

/// The span in the column under (x, z) whose band holds `near`, and its
/// floor.
fn floor_near(graph: &SpanGraph, x: f32, z: f32, near: f32) -> Option<(SpanRef, f32)> {
    let column = Column::containing(x, z);
    let span = graph.span_at(column, near + 0.5)?;
    Some((span, graph.span(span).floor_c))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A rectangular channel `width` wide with vertical banks 5 m high.
    fn rectangle(width: f32) -> CrossSection {
        let half = (width / 2.0 / SAMPLE_SPACING).round() as i32;
        let floors = (-half - 4..=half + 4)
            .map(|i| Some(if i.abs() <= half { 0.0 } else { 5.0 }))
            .collect::<Vec<_>>();
        CrossSection {
            centre: (half + 4) as usize,
            floors,
            spacing: SAMPLE_SPACING,
        }
    }

    #[test]
    fn two_cumecs_in_a_four_metre_bed_runs_as_the_design_says() {
        // Design §7.5: 2 m³/s in a 4 m bed at 1% slope, n = 0.035, runs about
        // 0.37 m deep at 1.33 m/s.
        let h = rectangle(4.0).hydraulics(2.0, 0.01).unwrap();
        assert!((h.depth - 0.37).abs() < 0.04, "depth {}", h.depth);
        assert!((h.velocity - 1.33).abs() < 0.15, "velocity {}", h.velocity);
    }

    #[test]
    fn depth_is_over_the_bed_not_a_lake_beyond_the_bank() {
        // The same channel, its section reaching over the right bank into a
        // lake whose floor is 4 m under the bed.
        let mut section = rectangle(4.0);
        let n = section.floors.len();
        section.floors[n - 2] = Some(-4.0);
        section.floors[n - 1] = Some(-4.0);
        let h = section.hydraulics(2.0, 0.01).unwrap();
        assert!((h.depth - 0.37).abs() < 0.04, "depth {}", h.depth);
        assert!((h.level - h.depth).abs() < 1e-6, "the bed is at 0");
    }

    #[test]
    fn a_section_stops_at_a_lake_with_no_bank_between() {
        use crate::water::geometry::{Span, SpanChunk, SpanChunkCoord, COLUMNS_PER_CHUNK};
        // Across x: a bank at 5 m to the west, a 4 m bed at 0, and past it,
        // with no bank, a lake whose floor is 1 m lower.
        let coord = SpanChunkCoord { x: 0, z: 0 };
        let mut graph = SpanGraph::new(coord, coord);
        let columns: Vec<Vec<Span>> = (0..COLUMNS_PER_CHUNK)
            .map(|local| {
                let f = match coord.column(local).i {
                    ..=3 => 5.0,
                    4..=11 => 0.0,
                    _ => -1.0,
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
        let (x, z) = Column::new(8, 8).centre();
        let centre = Point3::new(x, 0.0, z);
        let along = Vector2::new(0.0, 1.0);
        let lake = |span: SpanRef| span.column.i >= 12;
        let open = CrossSection::sample(&graph, centre, along, 8.0, &|_| false);
        let shored = CrossSection::sample(&graph, centre, along, 8.0, &lake);
        let (wide, narrow) = (
            open.hydraulics(2.0, 0.01).unwrap(),
            shored.hydraulics(2.0, 0.01).unwrap(),
        );
        // Open, the section runs on into the lake, whose water carries the
        // flow: the channel reads as barely wet.
        assert!(wide.depth < 0.05, "{wide:?}");
        // Stopped at the shore, it is the 4 m channel it is.
        assert!((narrow.top_width - 4.0).abs() < 0.3, "{narrow:?}");
        assert!((narrow.depth - 0.37).abs() < 0.05, "{narrow:?}");
    }
}
