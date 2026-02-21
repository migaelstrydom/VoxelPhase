//! Contact manifold persistence across frames.
//!
//! The manifold cache matches contacts by `FeatureId` so that impulses from the
//! previous frame can be carried forward (warm-starting). This is the single
//! biggest stability improvement for resting and stacking contacts.
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

use std::collections::HashMap;

use nalgebra::Vector3;
use smallvec::SmallVec;

use crate::collision::contact::FeatureId;
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
/// contacts by `FeatureId`.
#[derive(Debug, Clone)]
pub struct CachedContact {
    /// Feature pair that produced this contact, used as the match key.
    pub feature_id: FeatureId,
    /// Accumulated normal impulse from the solver (warm-start value).
    pub normal_impulse: f32,
    /// Accumulated tangent impulses from the solver (warm-start value).
    pub tangent_impulse: [f32; 2],
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

    /// Find a cached contact by feature ID.
    fn find_by_feature(&self, id: FeatureId) -> Option<usize> {
        self.points.iter().position(|c| c.feature_id == id)
    }
}

/// Cache of all active contact manifolds, persisted across simulation frames.
pub struct ManifoldCache {
    manifolds: HashMap<ManifoldKey, CachedManifold>,
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
            manifolds: HashMap::new(),
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
            let cached = self.manifolds.entry(key).or_insert_with(CachedManifold::new);

            let mut solver_contacts = SmallVec::with_capacity(pair.manifold.len());

            for contact_point in &pair.manifold.points {
                let feature_id = contact_point.feature_id;

                let (warm_normal, warm_tangent, contact_normal) =
                    match cached.find_by_feature(feature_id) {
                        Some(idx) => {
                            self.frame_stats.point_matches += 1;
                            let entry = &mut cached.points[idx];

                            let aligned =
                                self.normal_smoother.aligned(entry.normal, contact_point.normal);
                            let contact_normal = if aligned {
                                self.normal_smoother.smooth(entry.normal, contact_point.normal)
                            } else {
                                contact_point.normal
                            };

                            let use_warm =
                                aligned && contact_point.raw_depth >= -self.warm_start_depth_slop;
                            let warm = if use_warm {
                                (entry.normal_impulse, entry.tangent_impulse)
                            } else {
                                entry.normal_impulse = 0.0;
                                entry.tangent_impulse = [0.0, 0.0];
                                (0.0, [0.0, 0.0])
                            };

                            entry.normal = contact_normal;
                            entry.depth = contact_point.depth;
                            entry.age = 0;

                            (warm.0, warm.1, contact_normal)
                        }
                        None => {
                            // New contact: insert into cache
                            if cached.points.len() < 4 {
                                cached.points.push(CachedContact {
                                    feature_id,
                                    normal_impulse: 0.0,
                                    tangent_impulse: [0.0, 0.0],
                                    normal: contact_point.normal,
                                    depth: contact_point.depth,
                                    age: 0,
                                });
                                self.frame_stats.point_adds += 1;
                            } else if let Some(replace_idx) =
                                find_shallowest_aged(&cached.points)
                            {
                                cached.points[replace_idx] = CachedContact {
                                    feature_id,
                                    normal_impulse: 0.0,
                                    tangent_impulse: [0.0, 0.0],
                                    normal: contact_point.normal,
                                    depth: contact_point.depth,
                                    age: 0,
                                };
                                self.frame_stats.point_replacements += 1;
                            }
                            (0.0, [0.0, 0.0], contact_point.normal)
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
                    warm_tangent_impulse: warm_tangent,
                    accumulated_normal_impulse: 0.0,
                    accumulated_tangent_impulse: [0.0, 0.0],
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
    /// Reads accumulated impulses from each `SolverContact` and stores them in the
    /// corresponding `CachedContact` matched by `FeatureId`.
    pub fn write_back(&mut self, solved_manifolds: &[SolverManifold]) {
        for manifold in solved_manifolds {
            let Some(col_b) = manifold.header.collider_b else {
                continue;
            };
            let key = ManifoldKey::new(manifold.header.collider_a, col_b);

            let Some(cached) = self.manifolds.get_mut(&key) else {
                continue;
            };

            for contact in &manifold.contacts {
                if let Some(idx) = cached.find_by_feature(contact.feature_id) {
                    let entry = &mut cached.points[idx];
                    entry.normal_impulse = contact.accumulated_normal_impulse;
                    entry.tangent_impulse = contact.accumulated_tangent_impulse;
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
            warm_tangent_impulse: [0.0, 0.0],
            accumulated_normal_impulse: 0.0,
            accumulated_tangent_impulse: [0.0, 0.0],
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
            b.age
                .cmp(&a.age)
                .then_with(|| {
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
