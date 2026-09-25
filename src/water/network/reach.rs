//! A reach: one 8–16 m stretch of a channel, holding the water running
//! down it.
//!
//! ```text
//!   Advancing   x_f < L_r      S = Ā(Q_in)·ℓ + S_dead      nothing out yet
//!   Flowing     x_f = L_r      Q_out = Ā⁻¹((S − S_dead)/ℓ)
//!   Receding    Q_in = 0       the tail moves down at v(Q_out); Q_out keeps
//!                              the flowing law over the shrinking ℓ
//! ```
//!
//! The ledger sees only the storage `S`. The front `x_f` and tail `x_t` are
//! presentation positions, held fixed through each implicit solve and
//! advanced after it. `Q_out` rises with `S` at fixed wetted length, so the
//! step stays unconditionally stable.

use nalgebra::{Point3, Vector2};

use crate::water::geometry::{SpanRef, COLUMN_SIZE};

use super::centreline::Centreline;
use super::link::FallPath;
use super::lip::{critical_depth, Lip};
use super::rating::{RatingCurve, RatingPoint};

/// Depth at a free brink as a share of the critical depth.
const BRINK_SHARE: f32 = 0.715;

/// An end eases over this many depths of the water in the reach...
const EASE_DEPTHS: f32 = 4.0;

/// ...and never less than this, m.
const MIN_EASE: f32 = 2.0;

/// Where a reach is in its life.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReachState {
    Advancing,
    Flowing,
    Receding,
}

/// Where the last reach of a channel lets its water go. It is kept as a
/// point, which outlives rebuilds of the spans there, so the reach can link
/// again to whatever store stands there if the one it fed is replaced.
#[derive(Debug, Clone, PartialEq)]
pub struct ChannelOutlet {
    /// On the floor or water where the channel's water enters its target.
    pub at: Point3<f32>,
    /// The arc it falls along to get there, if it leaves the bed at a step.
    pub fall: Option<FallPath>,
}

/// One stretch of channel and the water in it.
#[derive(Debug, Clone)]
pub struct Reach {
    /// Water held, m³.
    pub storage: f64,
    /// The D8 path, upstream first.
    pub cells: Vec<SpanRef>,
    /// Each cell's distance along the centreline, m.
    pub cell_distance: Vec<f32>,
    /// The smoothed path.
    pub centreline: Centreline,
    /// Length, m.
    pub length: f32,
    pub rating: RatingCurve,
    /// Pothole storage along the path: distance and volume, ascending.
    pub dead: Vec<(f32, f64)>,
    /// Downstream end of the wetted length, m from the top.
    pub front: f32,
    /// Upstream end of the wetted length.
    pub tail: f32,
    pub state: ReachState,
    /// What flowed in over the last tick, m³/s.
    pub inflow: f64,
    /// What flowed out over the last tick, m³/s.
    pub outflow: f64,
    /// Loses water to the ground and air (§7.7).
    pub minor: bool,
    /// Bumped when the path or rating changes, so its mesh rebuilds.
    pub version: u32,
    /// Columns whose spans this reach claims.
    pub claimed: Vec<crate::water::geometry::Column>,
    /// For the last reach of a channel: where its water goes.
    pub outlet: Option<ChannelOutlet>,
}

impl Reach {
    /// An empty reach along a path, advancing from its top.
    pub fn new(
        cells: Vec<SpanRef>,
        points: &[Point3<f32>],
        rating: RatingCurve,
        dead: Vec<(f32, f64)>,
    ) -> Self {
        let centreline = Centreline::from_path(points);
        let length = centreline.length().max(0.5);
        let cell_distance = points
            .iter()
            .map(|p| nearest_distance(&centreline, *p))
            .collect();
        Self {
            storage: 0.0,
            cells,
            cell_distance,
            centreline,
            length,
            rating,
            dead,
            front: 0.0,
            tail: 0.0,
            state: ReachState::Advancing,
            inflow: 0.0,
            outflow: 0.0,
            minor: false,
            version: 0,
            claimed: Vec::new(),
            outlet: None,
        }
    }

    /// Wetted length, m.
    pub fn wetted(&self) -> f32 {
        (self.front - self.tail).max(0.0)
    }

    /// Pothole storage between two distances along the reach.
    pub fn dead_between(&self, from: f32, to: f32) -> f64 {
        self.dead
            .iter()
            .filter(|(d, _)| *d >= from && *d <= to)
            .map(|(_, v)| v)
            .sum()
    }

    /// Discharge out of the downstream end at `storage`, and its derivative,
    /// with the front and tail where they are.
    pub fn release(&self, storage: f64) -> (f64, f64) {
        if self.state == ReachState::Advancing {
            return (0.0, 0.0);
        }
        let wetted = self.wetted() as f64;
        if wetted <= 1e-3 {
            return (0.0, 0.0);
        }
        let live = storage - self.dead_between(self.tail, self.front);
        if live <= 0.0 {
            return (0.0, 0.0);
        }
        let q = self.rating.discharge_for(live / wetted);
        let dq = 1.0 / (wetted * self.rating.area_slope(q.max(1e-6)));
        (q, dq)
    }

    /// The hydraulics the reach runs at now: its outflow when flowing, else
    /// its inflow.
    pub fn running(&self) -> RatingPoint {
        let q = if self.state == ReachState::Advancing {
            self.inflow
        } else {
            self.outflow.max(self.inflow)
        };
        self.rating.at(q)
    }

    /// The water surface at the top or bottom end.
    pub fn level_at_end(&self, downstream: bool) -> f32 {
        let bed = if downstream {
            self.centreline.points.last()
        } else {
            self.centreline.points.first()
        };
        bed.map_or(0.0, |p| p.y) + self.running().depth
    }

    /// Move the front and tail after a tick of `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        match self.state {
            ReachState::Receding if self.inflow > 0.0 => {
                self.tail = 0.0;
                self.state = if self.front >= self.length {
                    ReachState::Flowing
                } else {
                    ReachState::Advancing
                };
            }
            ReachState::Advancing | ReachState::Flowing
                if self.inflow <= 0.0 && self.storage > 0.0 =>
            {
                self.state = ReachState::Receding;
            }
            _ => {}
        }
        match self.state {
            ReachState::Advancing => {
                let area = self.rating.at(self.inflow).area.max(1e-6);
                // S = Ā·x + S_dead(0, x): rises with x, so bisect.
                let fill = |x: f32| area * x as f64 + self.dead_between(0.0, x);
                let (mut lo, mut hi) = (self.front, self.length);
                if fill(hi) <= self.storage {
                    self.front = self.length;
                    self.state = ReachState::Flowing;
                } else {
                    for _ in 0..40 {
                        let mid = 0.5 * (lo + hi);
                        if fill(mid) < self.storage {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    self.front = self.front.max(lo);
                }
            }
            ReachState::Receding => {
                let v = self.rating.at(self.outflow).velocity;
                self.tail = (self.tail + v * dt).min(self.front);
            }
            ReachState::Flowing => {}
        }
    }

    /// Mean depth of the water held, m.
    pub fn mean_depth(&self) -> f32 {
        let wetted = self.wetted();
        let width = self.running().top_width;
        if wetted <= 0.0 || width <= 0.0 {
            return 0.0;
        }
        (self.storage / (wetted * width) as f64) as f32
    }

    /// Distance along the reach nearest a plan position.
    pub fn distance_at(&self, x: f32, z: f32) -> f32 {
        nearest_distance(&self.centreline, Point3::new(x, 0.0, z))
    }

    /// Unit direction of flow nearest a distance along the reach.
    pub fn direction_at(&self, distance: f32) -> Vector2<f32> {
        let i = self
            .centreline
            .distance
            .partition_point(|d| *d < distance)
            .min(self.centreline.tangents.len().saturating_sub(1));
        self.centreline
            .tangents
            .get(i)
            .copied()
            .unwrap_or_else(Vector2::x)
    }

    /// Where its water leaves it: the edge of its last column, on the bed,
    /// in the direction the channel runs there.
    pub fn end_lip(&self) -> Lip {
        let line = &self.centreline;
        let end = line.points.last().copied().unwrap_or_else(Point3::origin);
        let direction = line.tangents.last().copied().unwrap_or_else(Vector2::x);
        let edge = direction * (0.5 * COLUMN_SIZE);
        Lip {
            at: Point3::new(end.x + edge.x, end.y, end.z + edge.y),
            direction,
        }
    }

    /// Bed height a distance along the reach, between its points.
    pub fn bed_at(&self, distance: f32) -> f32 {
        self.centreline.point_at(distance).y
    }

    /// The surface its rating gives a distance along it, before its ends
    /// meet the stores at its ports.
    pub fn normal_surface(&self, distance: f32) -> f32 {
        self.bed_at(distance) + self.running().depth
    }

    /// Depth at a free brink, where the water leaves its end over a drop: a
    /// little under the critical depth of what it carries now.
    pub fn brink_depth(&self) -> f32 {
        let running = self.running();
        BRINK_SHARE * critical_depth(running.q, running.top_width)
    }

    /// Length over which each end eases to the store at its port, m: never
    /// more than half the reach, so the two ends do not overlap.
    pub fn ease_length(&self) -> f32 {
        (EASE_DEPTHS * self.running().depth)
            .max(MIN_EASE)
            .min(0.5 * self.length)
    }

    /// The surface a distance along the reach, its ends eased by `ends` to
    /// the stores at its ports (§7.9). Mirrored in `shader/river.vert`.
    pub fn surface_at(&self, distance: f32, ends: ReachEnds) -> f32 {
        let ease = self.ease_length();
        let upstream = 1.0 - smoothstep(0.0, ease, distance);
        let downstream = smoothstep(self.length - ease, self.length, distance);
        self.normal_surface(distance) + upstream * ends.upstream + downstream * ends.downstream
    }
}

/// How far a reach's surface is moved at each end to meet the stores at its
/// ports, over its normal surface there, m (§7.9).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ReachEnds {
    pub upstream: f32,
    pub downstream: f32,
}

fn smoothstep(from: f32, to: f32, x: f32) -> f32 {
    if to <= from {
        return if x >= to { 1.0 } else { 0.0 };
    }
    let t = ((x - from) / (to - from)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Distance along a centreline of the point nearest `p` in plan.
fn nearest_distance(line: &Centreline, p: Point3<f32>) -> f32 {
    line.points
        .iter()
        .zip(&line.distance)
        .min_by(|a, b| {
            let da = (a.0.x - p.x).powi(2) + (a.0.z - p.z).powi(2);
            let db = (b.0.x - p.x).powi(2) + (b.0.z - p.z).powi(2);
            da.total_cmp(&db)
        })
        .map_or(0.0, |(_, d)| *d)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A dry reach 20 m long on a bed falling from 5 m to 4 m.
    fn straight() -> Reach {
        let points: Vec<Point3<f32>> = (0..=40)
            .map(|i| Point3::new(i as f32 * 0.5, 5.0 - i as f32 * 0.025, 0.0))
            .collect();
        Reach::new(Vec::new(), &points, RatingCurve::default(), Vec::new())
    }

    #[test]
    fn a_surface_eases_to_its_ports_at_its_ends_only() {
        let reach = straight();
        let ends = ReachEnds {
            upstream: 0.5,
            downstream: -0.3,
        };
        let at = |d: f32| reach.surface_at(d, ends) - reach.normal_surface(d);
        assert!((at(0.0) - 0.5).abs() < 1e-5);
        assert!((at(reach.length) + 0.3).abs() < 1e-5);
        assert_eq!(at(reach.length * 0.5), 0.0);
        // Smoothly: no step between neighbouring points.
        let steps = (0..400).map(|i| {
            let d = i as f32 * reach.length / 400.0;
            (reach.surface_at(d + reach.length / 400.0, ends) - reach.surface_at(d, ends)).abs()
        });
        assert!(steps.fold(0.0f32, f32::max) < 0.03);
    }

    #[test]
    fn the_ends_of_a_short_reach_do_not_overlap() {
        let points = [Point3::new(0.0, 1.0, 0.0), Point3::new(1.5, 1.0, 0.0)];
        let reach = Reach::new(Vec::new(), &points, RatingCurve::default(), Vec::new());
        assert!(reach.ease_length() <= 0.5 * reach.length);
    }

    /// `shader/river.vert` eases the ends the same way.
    #[test]
    fn the_shader_eases_as_surface_at_does() {
        let glsl =
            std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/shader/river.vert"))
                .expect("shader/river.vert");
        assert!(glsl.contains(&format!("EASE_DEPTHS = {EASE_DEPTHS:.1}")));
        assert!(glsl.contains(&format!("MIN_EASE = {MIN_EASE:.1}")));
        assert!(glsl.contains("0.5 * length"));
        assert!(glsl.contains("1.0 - smoothstep(0.0, ease, along)"));
        assert!(glsl.contains("smoothstep(length - ease, length, along)"));
    }
}
