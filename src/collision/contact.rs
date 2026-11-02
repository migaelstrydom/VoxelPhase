//! Contact manifold types for the collision library.
//!
//! Every contact point carries both raw geometric data (from the collision test)
//! and solver-ready values (adjusted for margin and normal smoothing).

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use super::SurfaceId;

/// Identifies the geometric feature pair that generated a contact point.
///
/// Feature IDs enable warm-starting (cached impulse matching), temporal coherence
/// (skip re-clipping when features haven't changed), and separating axis caching.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FeatureId(pub u64);

/// Set on a mesh edge's feature ID, clear on a mesh face's, so that the two
/// cannot be mistaken for each other.
const MESH_EDGE_BIT: u64 = 1 << 63;

/// FNV-1a over 64-bit words.
fn fnv1a(words: &[u64]) -> u64 {
    words.iter().fold(0xcbf29ce484222325, |hash, &word| {
        (hash ^ word).wrapping_mul(0x100000001b3)
    })
}

impl FeatureId {
    /// Constant feature ID for pairs with only one possible contact (e.g. sphere-sphere).
    pub const SINGLE: Self = Self(0);

    /// Construct from a face index on one shape.
    pub fn from_face(face_index: u32) -> Self {
        Self(face_index as u64)
    }

    /// Construct from a face index on each shape (e.g. OBB face-face).
    pub fn from_face_pair(face_a: u32, face_b: u32) -> Self {
        Self(((face_a as u64) << 32) | (face_b as u64))
    }

    /// Construct from an edge index on each shape (e.g. OBB edge-edge).
    pub fn from_edge_pair(edge_a: u32, edge_b: u32) -> Self {
        // Set high bit to distinguish from face-face pairs.
        Self((1u64 << 63) | ((edge_a as u64) << 32) | (edge_b as u64))
    }

    /// A mesh triangle, by a key that names it whatever query it came back
    /// from ([`PatchTriangle::key`](crate::collision::mesh_patch::PatchTriangle::key)).
    pub fn from_mesh_triangle(key: u64) -> Self {
        Self(key & !MESH_EDGE_BIT)
    }

    /// A face merged from two mesh triangles, in either order.
    pub fn from_mesh_triangle_pair(a: u64, b: u64) -> Self {
        Self(fnv1a(&[a.min(b), a.max(b)]) & !MESH_EDGE_BIT)
    }

    /// One edge (0, 1 or 2) of a mesh triangle.
    pub fn from_mesh_edge(triangle: u64, edge: u32) -> Self {
        Self(MESH_EDGE_BIT | (fnv1a(&[triangle, edge as u64]) >> 1))
    }

    /// Combine a base feature ID with a vertex index to produce a unique
    /// per-contact-point ID. Used when multiple contacts share the same
    /// face feature but need distinct IDs for warm-start matching.
    pub fn with_vertex(self, vertex_index: u32) -> Self {
        let mut hash = self.0;
        hash ^= (vertex_index as u64) | (0xA5u64 << 56);
        hash = hash.wrapping_mul(0x100000001b3);
        Self(hash)
    }
}

/// A single contact point between two shapes.
#[derive(Debug, Clone, Copy)]
pub struct ContactPoint {
    /// World-space contact position.
    pub point: Point3<f32>,

    /// Geometric contact normal (directly from collision test, A-to-B).
    pub raw_normal: Vector3<f32>,

    /// Solver-facing normal. Initially equals `raw_normal`, but may be
    /// smoothed by the manifold cache for temporal stability.
    pub normal: Vector3<f32>,

    /// Geometric penetration depth (positive = overlapping, negative = margin-only contact).
    /// This is the unmodified value from the collision test with margin applied.
    pub raw_depth: f32,

    /// Solver-facing depth, clamped to >= 0. Margin-only contacts (raw_depth < 0)
    /// get depth = 0, meaning velocity-only correction with no position push.
    pub depth: f32,

    /// Separation still to close before the shapes touch, for a contact
    /// predicted ahead of the pair's arrival (a speculative contact).
    ///
    /// Zero for a contact where the shapes already meet, which is every contact
    /// the collision tests themselves produce. The solver lets a pair approach
    /// by this much before it arrests them.
    pub gap: f32,

    /// Identifies the geometric feature pair that produced this contact.
    pub feature_id: FeatureId,

    /// Surface of the static triangle this contact was generated against;
    /// `SurfaceId::UNSPECIFIED` for a contact between two shapes.
    pub surface: SurfaceId,
}

impl ContactPoint {
    /// Create a contact point with consistent raw/solver fields.
    ///
    /// `raw_depth` is the geometric depth from the collision test (may be negative
    /// for margin-only contacts). `depth` is clamped to >= 0 automatically.
    /// `normal` is set equal to `raw_normal`; the physics pipeline may smooth it later.
    pub fn new(
        point: Point3<f32>,
        normal: Vector3<f32>,
        raw_depth: f32,
        feature_id: FeatureId,
    ) -> Self {
        Self {
            point,
            raw_normal: normal,
            normal,
            raw_depth,
            depth: raw_depth.max(0.0),
            gap: 0.0,
            feature_id,
            surface: SurfaceId::UNSPECIFIED,
        }
    }

    /// The same contact, generated against a static triangle of `surface`.
    pub fn on(mut self, surface: SurfaceId) -> Self {
        self.surface = surface;
        self
    }
}

/// A contact manifold: up to 4 contact points from a single shape pair.
#[derive(Debug, Clone)]
pub struct ContactManifold {
    pub points: SmallVec<[ContactPoint; 4]>,
}

impl ContactManifold {
    pub fn empty() -> Self {
        Self {
            points: SmallVec::new(),
        }
    }

    pub fn single(point: ContactPoint) -> Self {
        let mut points = SmallVec::new();
        points.push(point);
        Self { points }
    }

    pub fn from_vec(points: SmallVec<[ContactPoint; 4]>) -> Self {
        Self { points }
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feature_id_equality() {
        let a = FeatureId::from_face(3);
        let b = FeatureId::from_face(3);
        let c = FeatureId::from_face(5);
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn feature_id_face_pair_vs_edge_pair() {
        let face = FeatureId::from_face_pair(1, 2);
        let edge = FeatureId::from_edge_pair(1, 2);
        assert_ne!(face, edge);
    }

    #[test]
    fn feature_id_hashing() {
        use std::collections::HashSet;
        let mut set = HashSet::new();
        set.insert(FeatureId::from_face(0));
        set.insert(FeatureId::from_face(1));
        set.insert(FeatureId::from_face(0));
        assert_eq!(set.len(), 2);
    }

    #[test]
    fn a_merged_mesh_face_is_named_the_same_in_either_order() {
        assert_eq!(
            FeatureId::from_mesh_triangle_pair(7, 11),
            FeatureId::from_mesh_triangle_pair(11, 7)
        );
        assert_ne!(
            FeatureId::from_mesh_triangle_pair(7, 11),
            FeatureId::from_mesh_triangle_pair(7, 12)
        );
    }

    #[test]
    fn a_mesh_edge_is_never_a_mesh_face() {
        let key = u64::MAX;
        assert_ne!(
            FeatureId::from_mesh_triangle(key),
            FeatureId::from_mesh_edge(key, 0)
        );
        assert_ne!(
            FeatureId::from_mesh_edge(key, 0),
            FeatureId::from_mesh_edge(key, 1)
        );
    }

    #[test]
    fn contact_point_clamps_depth() {
        let cp = ContactPoint::new(Point3::origin(), Vector3::y(), -0.01, FeatureId::SINGLE);
        assert_eq!(cp.depth, 0.0);
        assert_eq!(cp.raw_depth, -0.01);
    }

    #[test]
    fn contact_point_positive_depth() {
        let cp = ContactPoint::new(Point3::origin(), Vector3::y(), 0.05, FeatureId::SINGLE);
        assert_eq!(cp.depth, 0.05);
        assert_eq!(cp.raw_depth, 0.05);
    }

    #[test]
    fn contact_point_normal_equals_raw() {
        let n = Vector3::new(0.0, 1.0, 0.0);
        let cp = ContactPoint::new(Point3::origin(), n, 0.1, FeatureId::SINGLE);
        assert_eq!(cp.normal, cp.raw_normal);
    }

    #[test]
    fn manifold_empty() {
        let m = ContactManifold::empty();
        assert!(m.is_empty());
        assert_eq!(m.len(), 0);
    }

    #[test]
    fn manifold_single() {
        let cp = ContactPoint::new(Point3::origin(), Vector3::y(), 0.1, FeatureId::SINGLE);
        let m = ContactManifold::single(cp);
        assert_eq!(m.len(), 1);
    }
}
