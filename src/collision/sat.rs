//! Separating Axis Test helpers.
//!
//! Shared SAT projection and overlap utilities used by OBB-OBB and other
//! algebraic (tier 2) pair tests.

use nalgebra::{Point3, Vector3};

/// Result of testing a single SAT axis.
#[derive(Debug, Clone, Copy)]
#[allow(unused)]
pub struct SatAxisResult {
    /// The normalized separating/penetration axis (A→B direction).
    pub axis: Vector3<f32>,
    /// Overlap on this axis. Positive = overlapping, negative = separated.
    pub overlap: f32,
}

/// Minimum axis normalization threshold.
pub const AXIS_EPS: f32 = 1e-6;

/// Tolerance for near-zero overlaps (avoid false contacts from float noise).
pub const OVERLAP_EPS: f32 = 1e-5;

/// Cached separating axis from a previous frame for early-out.
#[derive(Debug, Clone, Copy)]
pub struct SatCache {
    /// Last frame's separating axis (world space). None if the pair was colliding.
    pub separating_axis: Option<Vector3<f32>>,
}

impl SatCache {
    pub fn new() -> Self {
        Self {
            separating_axis: None,
        }
    }
}

impl Default for SatCache {
    fn default() -> Self {
        Self::new()
    }
}

/// Test whether two edges (one from each hull) can form a valid separating axis.
///
/// Uses the Gauss map: each edge's two adjacent face normals define an arc on
/// the unit sphere. Two edges form a Minkowski face (and thus a potential
/// separating axis) only if their Gauss map arcs intersect.
///
/// Parameters `a, b` are face normals adjacent to the edge on hull A.
/// Parameters `c, d` are the **negated** face normals adjacent to the edge on
/// hull B (caller must negate for Minkowski difference).
///
/// The three conditions check: (1) c,d straddle A's great circle,
/// (2) a,b straddle B's great circle, (3) hemisphere consistency.
///
/// Reference: Gregorius, "The Separating Axis Test", GDC 2013.
pub fn is_minkowski_face(
    a: Vector3<f32>,
    b: Vector3<f32>,
    c: Vector3<f32>,
    d: Vector3<f32>,
) -> bool {
    let bxa = b.cross(&a);
    let dxc = d.cross(&c);

    let cba = c.dot(&bxa);
    let dba = d.dot(&bxa);
    let adc = a.dot(&dxc);
    let bdc = b.dot(&dxc);

    cba * dba < 0.0 && adc * bdc < 0.0 && cba * bdc > 0.0
}

/// Compute overlap along `axis` using a specific edge pair.
///
/// For edge pairs that pass the Gauss-map Minkowski-face test, these edges are
/// the supporting features for the edge-edge axis. So we can compute the SAT
/// overlap from just their endpoints in O(1), avoiding global support queries.
pub fn edge_pair_overlap(
    axis: Vector3<f32>,
    a0: Point3<f32>,
    a1: Point3<f32>,
    b0: Point3<f32>,
    b1: Point3<f32>,
    margin: f32,
) -> f32 {
    // For a valid edge-edge SAT axis n = e_a x e_b, n is perpendicular to both
    // edges. So every point on each edge has the same projection on n.
    // Use edge midpoints to reduce endpoint noise.
    let a_proj = 0.5 * (a0.coords.dot(&axis) + a1.coords.dot(&axis));
    let b_proj = 0.5 * (b0.coords.dot(&axis) + b1.coords.dot(&axis));
    a_proj - b_proj + 2.0 * margin
}
