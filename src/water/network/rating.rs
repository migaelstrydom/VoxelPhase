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

use crate::water::geometry::{Column, SpanGraph};

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
    /// Water depth over the lowest wet sample.
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
    /// `centre.y` in its column.
    pub fn sample(
        graph: &SpanGraph,
        centre: Point3<f32>,
        tangent: Vector2<f32>,
        half_width: f32,
    ) -> Self {
        let across = Vector2::new(-tangent.y, tangent.x);
        let steps = (half_width / SAMPLE_SPACING).round() as i32;
        let floors = (-steps..=steps)
            .map(|i| {
                let offset = across * (i as f32 * SAMPLE_SPACING);
                floor_near(graph, centre.x + offset.x, centre.z + offset.y, centre.y)
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
        let bottom = self
            .floors
            .iter()
            .flatten()
            .copied()
            .fold(bed, f32::min)
            .min(bed);
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
            depth: level - bottom.min(bed),
            level,
            area,
            top_width,
            velocity: if area > 0.0 { q / area } else { 0.0 },
        })
    }
}

/// The floor of the span in the column under (x, z) whose band holds `near`.
fn floor_near(graph: &SpanGraph, x: f32, z: f32, near: f32) -> Option<f32> {
    let column = Column::containing(x, z);
    let span = graph.span_at(column, near + 0.5)?;
    Some(graph.span(span).floor_c)
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
}
