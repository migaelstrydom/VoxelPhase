//! Contact manifold persistence across frames.
//!
//! The manifold cache matches contacts by `FeatureId` so that impulses from the
//! previous frame can be carried forward (warm-starting). This is the single
//! biggest stability improvement for resting and stacking contacts.
//!
//! When multiple contacts share the same `FeatureId` (e.g. OBB face-face contacts
//! where all points belong to the same face pair), the cache uses proximity-based
//! matching as a tiebreaker to pair each new contact with the closest cached entry.
//!
//! ## Data flow
//!
//! ```text
//! Vec<PairManifold>  ──►  ManifoldCache::merge()  ──►  Vec<SolverManifold>
//!                              │                              │
//!                              │                         solver writes
//!                              │                         accumulated impulses
//!                              │                              │
//!                              ◄── ManifoldCache::write_back()
//! ```

use rustc_hash::FxHashMap;

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::contact::FeatureId;
use crate::physics::drive::plan::TractionRow;
use crate::physics::handle::ColliderHandle;
use crate::physics::pipeline::normal_smoothing::{NormalSmoother, NormalSmoothingConfig};
use crate::physics::pipeline::pair::{PairManifold, SolverContact, SolverManifold};

/// Per-step manifold cache activity counters for diagnostics.
#[derive(Debug, Clone, Copy, Default)]
pub struct ManifoldFrameStats {
    /// Number of manifold points inserted this step.
    pub point_adds: usize,
    /// Number of manifold points replaced this step.
    pub point_replacements: usize,
    /// Number of cached points matched/reused this step.
    pub point_matches: usize,
    /// Number of cached points pruned due to age this step.
    pub point_pruned: usize,
    /// Number of manifolds currently stored after prune.
    pub manifolds: usize,
    /// Number of points currently stored after prune.
    pub points: usize,
}

/// Ordered key for manifold lookup by collider pair.
#[derive(Hash, Eq, PartialEq, Clone, Copy, Debug)]
struct ManifoldKey {
    /// First collider (None for static geometry).
    collider_a: Option<ColliderHandle>,
    /// Second collider.
    collider_b: ColliderHandle,
}

impl ManifoldKey {
    fn new(a: Option<ColliderHandle>, b: ColliderHandle) -> Self {
        // For dynamic-dynamic pairs, order deterministically by raw index
        match a {
            Some(handle_a) => {
                let (idx_a, _) = handle_a.raw_parts();
                let (idx_b, _) = b.raw_parts();
                if idx_a > idx_b {
                    Self {
                        collider_a: Some(b),
                        collider_b: handle_a,
                    }
                } else {
                    Self {
                        collider_a: Some(handle_a),
                        collider_b: b,
                    }
                }
            }
            None => Self {
                collider_a: None,
                collider_b: b,
            },
        }
    }
}

/// A single cached contact point, keyed by feature ID for warm-start matching.
///
/// Stored inside `CachedManifold` and matched against incoming `PairManifold`
/// contacts by `FeatureId`, with proximity as a tiebreaker when multiple contacts
/// share the same feature ID.
#[derive(Debug, Clone)]
pub struct CachedContact {
    /// Feature pair that produced this contact, used as the primary match key.
    pub feature_id: FeatureId,
    /// World-space position from the most recent narrowphase that refreshed this
    /// contact. Used for proximity tiebreaking when multiple contacts share a
    /// feature ID.
    pub point: Point3<f32>,
    /// Accumulated normal impulse from the solver (warm-start value).
    pub normal_impulse: f32,
    /// Accumulated friction impulse in world space from the solver (warm-start value).
    ///
    /// Stored as a world-space vector so that small normal drift between frames
    /// does not rotate the cached friction direction.
    pub friction_impulse_ws: Vector3<f32>,
    /// Contact normal in world space at the time of caching.
    pub normal: Vector3<f32>,
    /// Penetration depth at the time of caching.
    pub depth: f32,
    /// Frames since this contact was last refreshed by the narrowphase.
    pub age: u8,
}

/// A persistent contact manifold keyed by feature ID.
///
/// Stored in `ManifoldCache` and indexed by `ManifoldKey` (collider pair).
#[derive(Debug, Clone)]
pub struct CachedManifold {
    /// Cached contacts, up to 4 points.
    pub points: SmallVec<[CachedContact; 4]>,
}

impl CachedManifold {
    fn new() -> Self {
        Self {
            points: SmallVec::new(),
        }
    }
}

/// Cache of all active contact manifolds, persisted across simulation frames.
pub struct ManifoldCache {
    manifolds: FxHashMap<ManifoldKey, CachedManifold>,
    max_age: u8,
    warm_start_depth_slop: f32,
    normal_smoother: NormalSmoother,
    frame_stats: ManifoldFrameStats,
}

impl ManifoldCache {
    pub fn new(
        max_age: u8,
        warm_start_depth_slop: f32,
        normal_smoothing: NormalSmoothingConfig,
    ) -> Self {
        Self {
            manifolds: FxHashMap::default(),
            max_age,
            warm_start_depth_slop,
            normal_smoother: NormalSmoother::from_config(normal_smoothing),
            frame_stats: ManifoldFrameStats::default(),
        }
    }

    /// Merge raw narrowphase manifolds with cached impulses, returning solver-ready
    /// manifolds with warm-start impulses populated from the cache.
    pub fn merge(
        &mut self,
        raw_manifolds: &[PairManifold],
        deterministic_ordering: bool,
    ) -> Vec<SolverManifold> {
        self.frame_stats = ManifoldFrameStats::default();

        // Age all existing cached points
        for cached in self.manifolds.values_mut() {
            for point in &mut cached.points {
                point.age += 1;
            }
        }

        let mut result = Vec::with_capacity(raw_manifolds.len());

        for pair in raw_manifolds {
            let Some(col_b) = pair.header.collider_b else {
                // Transient contacts (e.g. CCD) without collider info skip the cache
                result.push(pair_to_solver_cold(pair));
                continue;
            };
            let key = ManifoldKey::new(pair.header.collider_a, col_b);
            let cached = self
                .manifolds
                .entry(key)
                .or_insert_with(CachedManifold::new);

            let matches = match_contacts(&pair.manifold.points, &cached.points);
            let mut solver_contacts = SmallVec::with_capacity(pair.manifold.len());

            for (i, contact_point) in pair.manifold.points.iter().enumerate() {
                let feature_id = contact_point.feature_id;

                let (warm_normal, warm_friction_ws, contact_normal) = match matches[i] {
                    Some(idx) => {
                        self.frame_stats.point_matches += 1;
                        let entry = &mut cached.points[idx];

                        let aligned = self
                            .normal_smoother
                            .aligned(entry.normal, contact_point.normal);
                        let contact_normal = if aligned {
                            self.normal_smoother
                                .smooth(entry.normal, contact_point.normal)
                        } else {
                            contact_point.normal
                        };

                        let use_warm =
                            aligned && contact_point.raw_depth >= -self.warm_start_depth_slop;
                        let warm = if use_warm {
                            (entry.normal_impulse, entry.friction_impulse_ws)
                        } else {
                            entry.normal_impulse = 0.0;
                            entry.friction_impulse_ws = Vector3::zeros();
                            (0.0, Vector3::zeros())
                        };

                        entry.point = contact_point.point;
                        entry.feature_id = feature_id;
                        entry.normal = contact_normal;
                        entry.depth = contact_point.depth;
                        entry.age = 0;

                        (warm.0, warm.1, contact_normal)
                    }
                    None => {
                        // New contact: insert into cache
                        let new_entry = CachedContact {
                            feature_id,
                            point: contact_point.point,
                            normal_impulse: 0.0,
                            friction_impulse_ws: Vector3::zeros(),
                            normal: contact_point.normal,
                            depth: contact_point.depth,
                            age: 0,
                        };
                        if cached.points.len() < 4 {
                            cached.points.push(new_entry);
                            self.frame_stats.point_adds += 1;
                        } else if let Some(replace_idx) = find_shallowest_aged(&cached.points) {
                            cached.points[replace_idx] = new_entry;
                            self.frame_stats.point_replacements += 1;
                        }
                        (0.0, Vector3::zeros(), contact_point.normal)
                    }
                };

                solver_contacts.push(SolverContact {
                    point: contact_point.point,
                    normal: contact_normal,
                    raw_normal: contact_point.raw_normal,
                    depth: contact_point.depth,
                    raw_depth: contact_point.raw_depth,
                    feature_id,
                    warm_normal_impulse: warm_normal,
                    warm_friction_impulse_ws: warm_friction_ws,
                    accumulated_normal_impulse: 0.0,
                    accumulated_friction_impulse_ws: Vector3::zeros(),
                    tangential_scale: 1.0,
                    traction: TractionRow::default(),
                });
            }

            result.push(SolverManifold {
                header: pair.header.clone(),
                contacts: solver_contacts,
            });
        }

        if deterministic_ordering {
            result.sort_by(compare_solver_manifolds);
        }

        result
    }

    /// Write solved impulses back into the cache for next frame's warm-start.
    ///
    /// Uses the same feature-ID + proximity matching as [`merge`] to ensure each
    /// solver contact writes to its correct cached entry.
    pub fn write_back(&mut self, solved_manifolds: &[SolverManifold]) {
        for manifold in solved_manifolds {
            let Some(col_b) = manifold.header.collider_b else {
                continue;
            };
            let key = ManifoldKey::new(manifold.header.collider_a, col_b);

            let Some(cached) = self.manifolds.get_mut(&key) else {
                continue;
            };

            let matches = match_solver_contacts(&manifold.contacts, &cached.points);

            for (i, contact) in manifold.contacts.iter().enumerate() {
                if let Some(idx) = matches[i] {
                    let entry = &mut cached.points[idx];
                    entry.normal_impulse = contact.accumulated_normal_impulse;
                    entry.friction_impulse_ws = contact.accumulated_friction_impulse_ws;
                }
            }
        }
    }

    /// Remove stale manifold points that haven't been refreshed within max_age frames.
    /// Remove empty manifolds entirely.
    pub fn prune(&mut self) {
        let before_points: usize = self.manifolds.values().map(|m| m.points.len()).sum();
        self.manifolds.retain(|_, manifold| {
            manifold.points.retain(|p| p.age <= self.max_age);
            !manifold.points.is_empty()
        });
        let after_points: usize = self.manifolds.values().map(|m| m.points.len()).sum();
        self.frame_stats.point_pruned = before_points.saturating_sub(after_points);
        self.frame_stats.manifolds = self.manifolds.len();
        self.frame_stats.points = after_points;
    }

    /// Remove all manifolds involving the given collider.
    pub fn remove_collider(&mut self, handle: ColliderHandle) {
        self.manifolds
            .retain(|key, _| key.collider_a != Some(handle) && key.collider_b != handle);
    }

    /// Returns cache activity and size counters for the most recent step.
    pub fn frame_stats(&self) -> ManifoldFrameStats {
        self.frame_stats
    }
}

// ---------------------------------------------------------------------------
// Contact matching
// ---------------------------------------------------------------------------

/// Match incoming narrowphase contacts to cached entries.
///
/// Feature IDs are the primary key. When multiple contacts share a feature ID,
/// proximity (squared world-space distance) is used as a tiebreaker, with the
/// globally closest pairs matched first to avoid greedy misordering.
fn match_contacts(
    new_points: &SmallVec<[crate::collision::contact::ContactPoint; 4]>,
    cached: &SmallVec<[CachedContact; 4]>,
) -> SmallVec<[Option<usize>; 4]> {
    match_by_feature_and_proximity(new_points, cached, |cp| cp.feature_id, |cp| cp.point)
}

/// Match solver contacts to cached entries for write-back.
///
/// Same algorithm as [`match_contacts`] — feature ID first, proximity tiebreaker.
/// The positions were updated during merge so distances should be near-zero for
/// correctly matched pairs.
fn match_solver_contacts(
    solver_contacts: &SmallVec<[SolverContact; 4]>,
    cached: &SmallVec<[CachedContact; 4]>,
) -> SmallVec<[Option<usize>; 4]> {
    match_by_feature_and_proximity(solver_contacts, cached, |sc| sc.feature_id, |sc| sc.point)
}

/// Generic matcher used by both narrowphase merge and solver write-back.
///
/// Matching policy:
/// 1) unique feature-ID pairs match directly
/// 2) duplicate feature-ID groups match by global nearest-neighbor proximity
fn match_by_feature_and_proximity<T, FFeature, FPoint>(
    new_items: &SmallVec<[T; 4]>,
    cached: &SmallVec<[CachedContact; 4]>,
    feature_of: FFeature,
    point_of: FPoint,
) -> SmallVec<[Option<usize>; 4]>
where
    FFeature: Fn(&T) -> FeatureId,
    FPoint: Fn(&T) -> Point3<f32>,
{
    let new_len = new_items.len();
    let cached_len = cached.len();
    let mut result: SmallVec<[Option<usize>; 4]> = SmallVec::from_elem(None, new_len);

    if cached_len == 0 {
        return result;
    }

    let mut claimed: SmallVec<[bool; 4]> = SmallVec::from_elem(false, cached_len);

    // Pass 1: unique feature-ID matches.
    for (i, item) in new_items.iter().enumerate() {
        let item_feature = feature_of(item);
        let new_count = new_items
            .iter()
            .filter(|c| feature_of(c) == item_feature)
            .count();
        if new_count != 1 {
            continue;
        }

        let mut cached_match = None;
        let mut cached_count = 0u32;
        for (j, ce) in cached.iter().enumerate() {
            if ce.feature_id == item_feature {
                cached_match = Some(j);
                cached_count += 1;
            }
        }

        if cached_count == 1 {
            let j = cached_match.unwrap();
            if !claimed[j] {
                result[i] = Some(j);
                claimed[j] = true;
            }
        }
    }

    // Pass 2: proximity tiebreaker.
    let mut candidates: SmallVec<[(usize, usize, f32); 16]> = SmallVec::new();
    for (i, item) in new_items.iter().enumerate() {
        if result[i].is_some() {
            continue;
        }
        let item_feature = feature_of(item);
        let item_point = point_of(item);
        for (j, ce) in cached.iter().enumerate() {
            if claimed[j] {
                continue;
            }
            if ce.feature_id != item_feature {
                continue;
            }
            let dist_sq = (item_point - ce.point).magnitude_squared();
            candidates.push((i, j, dist_sq));
        }
    }
    candidates.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));

    for (i, j, _) in candidates {
        if result[i].is_some() || claimed[j] {
            continue;
        }
        result[i] = Some(j);
        claimed[j] = true;
    }

    result
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a cold (no warm-start) `SolverManifold` from a `PairManifold`.
fn pair_to_solver_cold(pair: &PairManifold) -> SolverManifold {
    let contacts = pair
        .manifold
        .points
        .iter()
        .map(|cp| SolverContact {
            point: cp.point,
            normal: cp.normal,
            raw_normal: cp.raw_normal,
            depth: cp.depth,
            raw_depth: cp.raw_depth,
            feature_id: cp.feature_id,
            warm_normal_impulse: 0.0,
            warm_friction_impulse_ws: Vector3::zeros(),
            accumulated_normal_impulse: 0.0,
            accumulated_friction_impulse_ws: Vector3::zeros(),
            tangential_scale: 1.0,
            traction: TractionRow::default(),
        })
        .collect();
    SolverManifold {
        header: pair.header.clone(),
        contacts,
    }
}

/// Find the cached point with the highest age and shallowest depth (best candidate
/// for replacement when the cache is full).
fn find_shallowest_aged(points: &SmallVec<[CachedContact; 4]>) -> Option<usize> {
    points
        .iter()
        .enumerate()
        .filter(|(_, p)| p.age > 0)
        .min_by(|(_, a), (_, b)| {
            // Prefer replacing older points first, then shallowest among those
            b.age.cmp(&a.age).then_with(|| {
                a.depth
                    .partial_cmp(&b.depth)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
        })
        .map(|(i, _)| i)
}

/// Deterministic ordering for solver manifolds by collider pair key.
///
/// Manifolds are naturally keyed by collider pair, so this replaces the old
/// per-contact deterministic ordering. Contacts within a manifold are already
/// in deterministic order from the collision library (deepest first).
fn compare_solver_manifolds(a: &SolverManifold, b: &SolverManifold) -> std::cmp::Ordering {
    let coll_a = compare_optional_collider(a.header.collider_a, b.header.collider_a);
    if coll_a != std::cmp::Ordering::Equal {
        return coll_a;
    }
    compare_optional_collider(a.header.collider_b, b.header.collider_b)
}

fn compare_optional_collider(
    a: Option<ColliderHandle>,
    b: Option<ColliderHandle>,
) -> std::cmp::Ordering {
    match (a, b) {
        (None, None) => std::cmp::Ordering::Equal,
        (None, Some(_)) => std::cmp::Ordering::Less,
        (Some(_), None) => std::cmp::Ordering::Greater,
        (Some(a), Some(b)) => {
            let (a_idx, a_gen) = a.raw_parts();
            let (b_idx, b_gen) = b.raw_parts();
            a_idx.cmp(&b_idx).then(a_gen.cmp(&b_gen))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use generational_arena::Index;

    fn test_body(idx: usize) -> crate::physics::handle::RigidBodyHandle {
        crate::physics::handle::RigidBodyHandle(Index::from_raw_parts(idx, 0))
    }

    fn test_collider(idx: usize) -> ColliderHandle {
        ColliderHandle(Index::from_raw_parts(idx, 0))
    }

    fn test_pair_header() -> crate::physics::pipeline::pair::PairHeader {
        crate::physics::pipeline::pair::PairHeader {
            body_a: Some(test_body(1)),
            body_b: test_body(2),
            collider_a: Some(test_collider(10)),
            collider_b: Some(test_collider(20)),
            restitution: 0.0,
            friction: 0.5,
        }
    }

    #[test]
    fn merge_duplicate_feature_ids_matches_warmstart_by_proximity() {
        let feature = FeatureId::from_face_pair(3, 4);
        let mut cache = ManifoldCache::new(4, 0.05, NormalSmoothingConfig::default());

        let key = ManifoldKey::new(Some(test_collider(10)), test_collider(20));
        cache.manifolds.insert(
            key,
            CachedManifold {
                points: SmallVec::from_vec(vec![
                    CachedContact {
                        feature_id: feature,
                        point: Point3::new(-1.0, 0.0, 0.0),
                        normal_impulse: 1.0,
                        friction_impulse_ws: Vector3::new(0.1, 0.0, 0.0),
                        normal: Vector3::x(),
                        depth: 0.2,
                        age: 0,
                    },
                    CachedContact {
                        feature_id: feature,
                        point: Point3::new(1.0, 0.0, 0.0),
                        normal_impulse: 2.0,
                        friction_impulse_ws: Vector3::new(0.2, 0.0, 0.0),
                        normal: Vector3::x(),
                        depth: 0.2,
                        age: 0,
                    },
                ]),
            },
        );

        let raw = PairManifold {
            header: test_pair_header(),
            manifold: crate::collision::contact::ContactManifold::from_vec(SmallVec::from_vec(
                vec![
                    crate::collision::contact::ContactPoint::new(
                        Point3::new(0.95, 0.0, 0.0),
                        Vector3::x(),
                        0.2,
                        feature,
                    ),
                    crate::collision::contact::ContactPoint::new(
                        Point3::new(-0.95, 0.0, 0.0),
                        Vector3::x(),
                        0.2,
                        feature,
                    ),
                ],
            )),
        };

        let merged = cache.merge(&[raw], true);
        assert_eq!(merged.len(), 1);
        assert_eq!(merged[0].contacts.len(), 2);
        assert_eq!(merged[0].contacts[0].warm_normal_impulse, 2.0);
        assert_eq!(merged[0].contacts[1].warm_normal_impulse, 1.0);
    }

    #[test]
    fn write_back_duplicate_feature_ids_uses_proximity_after_shuffle() {
        let feature = FeatureId::from_face_pair(3, 4);
        let mut cache = ManifoldCache::new(4, 0.05, NormalSmoothingConfig::default());

        let key = ManifoldKey::new(Some(test_collider(10)), test_collider(20));
        cache.manifolds.insert(
            key,
            CachedManifold {
                points: SmallVec::from_vec(vec![
                    CachedContact {
                        feature_id: feature,
                        point: Point3::new(-1.0, 0.0, 0.0),
                        normal_impulse: 0.0,
                        friction_impulse_ws: Vector3::zeros(),
                        normal: Vector3::x(),
                        depth: 0.2,
                        age: 0,
                    },
                    CachedContact {
                        feature_id: feature,
                        point: Point3::new(1.0, 0.0, 0.0),
                        normal_impulse: 0.0,
                        friction_impulse_ws: Vector3::zeros(),
                        normal: Vector3::x(),
                        depth: 0.2,
                        age: 0,
                    },
                ]),
            },
        );

        let contacts = SmallVec::from_vec(vec![
            SolverContact {
                point: Point3::new(0.98, 0.0, 0.0),
                normal: Vector3::x(),
                raw_normal: Vector3::x(),
                depth: 0.2,
                raw_depth: 0.2,
                feature_id: feature,
                warm_normal_impulse: 0.0,
                warm_friction_impulse_ws: Vector3::zeros(),
                accumulated_normal_impulse: 7.0,
                accumulated_friction_impulse_ws: Vector3::new(0.7, 0.0, 0.0),
                tangential_scale: 1.0,
                traction: TractionRow::default(),
            },
            SolverContact {
                point: Point3::new(-0.98, 0.0, 0.0),
                normal: Vector3::x(),
                raw_normal: Vector3::x(),
                depth: 0.2,
                raw_depth: 0.2,
                feature_id: feature,
                warm_normal_impulse: 0.0,
                warm_friction_impulse_ws: Vector3::zeros(),
                accumulated_normal_impulse: 9.0,
                accumulated_friction_impulse_ws: Vector3::new(0.9, 0.0, 0.0),
                tangential_scale: 1.0,
                traction: TractionRow::default(),
            },
        ]);
        let solved = SolverManifold {
            header: test_pair_header(),
            contacts,
        };

        cache.write_back(&[solved]);
        let cached = cache.manifolds.get(&key).expect("cached manifold exists");
        assert_eq!(cached.points.len(), 2);

        let left_idx = if cached.points[0].point.x < cached.points[1].point.x {
            0
        } else {
            1
        };
        let right_idx = 1 - left_idx;
        assert_eq!(cached.points[left_idx].normal_impulse, 9.0);
        assert_eq!(cached.points[right_idx].normal_impulse, 7.0);
    }
}
