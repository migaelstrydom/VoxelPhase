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

use crate::water::geometry::SpanRef;

use super::centreline::Centreline;
use super::rating::{RatingCurve, RatingPoint};

/// Where a reach is in its life.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReachState {
    Advancing,
    Flowing,
    Receding,
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
    /// For the last reach of a channel ending in a basin's water: that
    /// basin's level when the channel was laid. A channel whose basin has
    /// since risen over it or fallen away from it is re-routed (§8.2).
    pub routed_to_level: Option<f32>,
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
            routed_to_level: None,
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

    /// Bed height nearest a distance along the reach.
    pub fn bed_at(&self, distance: f32) -> f32 {
        let i = self
            .centreline
            .distance
            .partition_point(|d| *d < distance)
            .min(self.centreline.points.len().saturating_sub(1));
        self.centreline.points.get(i).map_or(0.0, |p| p.y)
    }
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
