//! Contact manifold persistence across frames.
//!
//! The manifold cache stores contact points in body-local space so they can be
//! matched across frames even as bodies move. Matched contacts inherit their
//! accumulated impulses from the previous frame, enabling warm-starting in the
//! solver. This is the single biggest stability improvement for resting and
//! stacking contacts.

use std::collections::{HashMap, HashSet};

use generational_arena::Arena;
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::handle::ColliderHandle;
use crate::physics::pipeline::normal_smoothing::{NormalSmoother, NormalSmoothingConfig};
use crate::physics::pipeline::solver::{ContactConstraint, SolvedImpulses};

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

/// A contact point stored in body-local space for frame-to-frame persistence.
#[derive(Debug, Clone)]
struct ManifoldPoint {
    /// Contact point in body A's local space (world space if A is static).
    local_point_a: Point3<f32>,
    /// Contact point in body B's local space.
    local_point_b: Point3<f32>,
    /// Contact normal in world space (updated each frame).
    normal: Vector3<f32>,
    /// Penetration depth (updated each frame).
    depth: f32,
    /// Accumulated normal impulse from the solver.
    normal_impulse: f32,
    /// Accumulated tangent impulses from the solver.
    tangent_impulse: [f32; 2],
    /// Frames since this point was last refreshed by the narrowphase.
    age: u8,
}

/// A persistent contact manifold between two colliders (or a collider and static geometry).
#[derive(Debug, Clone)]
struct ContactManifold {
    points: Vec<ManifoldPoint>,
}

impl ContactManifold {
    fn new() -> Self {
        Self {
            points: Vec::with_capacity(4),
        }
    }
}

/// Cache of all active contact manifolds, persisted across simulation frames.
pub struct ManifoldCache {
    manifolds: HashMap<ManifoldKey, ContactManifold>,
    match_threshold: f32,
    static_match_multiplier: f32,
    max_age: u8,
    warm_start_depth_slop: f32,
    normal_smoother: NormalSmoother,
    frame_stats: ManifoldFrameStats,
}

impl ManifoldCache {
    pub fn new(
        match_threshold: f32,
        static_match_multiplier: f32,
        max_age: u8,
        warm_start_depth_slop: f32,
        normal_smoothing: NormalSmoothingConfig,
    ) -> Self {
        Self {
            manifolds: HashMap::new(),
            match_threshold,
            static_match_multiplier,
            max_age,
            warm_start_depth_slop,
            normal_smoother: NormalSmoother::from_config(normal_smoothing),
            frame_stats: ManifoldFrameStats::default(),
        }
    }

    /// Merge raw narrowphase contacts with cached manifolds, returning solver-ready
    /// constraints with warm-start impulses populated from the cache.
    pub fn update(
        &mut self,
        raw_contacts: &[ContactConstraint],
        bodies: &Arena<RigidBody>,
    ) -> Vec<ContactConstraint> {
        self.frame_stats = ManifoldFrameStats::default();
        // Age all existing points before processing new contacts
        for manifold in self.manifolds.values_mut() {
            for point in &mut manifold.points {
                point.age += 1;
            }
        }

        let mut result = Vec::with_capacity(raw_contacts.len());
        let mut grouped: HashMap<ManifoldKey, Vec<&ContactConstraint>> = HashMap::new();

        for contact in raw_contacts {
            let Some(col_b) = contact.collider_b else {
                // Transient contacts (e.g. CCD) without collider info skip the cache
                result.push(contact.clone());
                continue;
            };
            let key = ManifoldKey::new(contact.collider_a, col_b);
            grouped.entry(key).or_default().push(contact);
        }

        for (key, contacts) in grouped {
            let manifold = self
                .manifolds
                .entry(key)
                .or_insert_with(ContactManifold::new);

            let contacts = if contacts.len() > 4 {
                reduce_contacts_with_manifold(contacts, manifold, bodies, 4, self.match_threshold)
            } else {
                contacts
            };

            let mut matched_indices: HashSet<usize> = HashSet::new();

            for contact in contacts {
                // Transform contact point to local space
                let (local_a, local_b) = to_local_space(contact, bodies);

                // Find closest existing point in the manifold, excluding already matched
                let threshold = if contact.body_a.is_none() {
                    self.match_threshold * self.static_match_multiplier
                } else {
                    self.match_threshold
                };
                let match_idx = find_closest_point_excluding(
                    manifold,
                    &local_b,
                    threshold,
                    &matched_indices,
                );

                let (warm_normal, warm_tangent, contact_normal) = match match_idx {
                    Some(idx) => {
                        matched_indices.insert(idx);
                        self.frame_stats.point_matches += 1;
                        // Matched: inherit cached impulses, update point
                        let cached = &mut manifold.points[idx];
                        let aligned = self.normal_smoother.aligned(cached.normal, contact.normal);
                        let contact_normal = if aligned {
                            self.normal_smoother.smooth(cached.normal, contact.normal)
                        } else {
                            contact.normal
                        };
                        let use_warm =
                            aligned && contact.raw_depth >= -self.warm_start_depth_slop;
                        let warm = if use_warm {
                            (cached.normal_impulse, cached.tangent_impulse)
                        } else {
                            cached.normal_impulse = 0.0;
                            cached.tangent_impulse = [0.0, 0.0];
                            (0.0, [0.0, 0.0])
                        };
                        cached.local_point_a = local_a;
                        cached.local_point_b = local_b;
                        cached.normal = contact_normal;
                        cached.depth = contact.depth;
                        cached.age = 0;
                        (warm.0, warm.1, contact_normal)
                    }
                    None => {
                        // New point: insert (or replace shallowest if full)
                        let new_point = ManifoldPoint {
                            local_point_a: local_a,
                            local_point_b: local_b,
                            normal: contact.normal,
                            depth: contact.depth,
                            normal_impulse: 0.0,
                            tangent_impulse: [0.0, 0.0],
                            age: 0,
                        };

                        if manifold.points.len() < 4 {
                            manifold.points.push(new_point);
                            matched_indices.insert(manifold.points.len() - 1);
                            self.frame_stats.point_adds += 1;
                        } else if let Some(replace_idx) =
                            find_shallowest_unmatched(manifold, &matched_indices)
                        {
                            manifold.points[replace_idx] = new_point;
                            matched_indices.insert(replace_idx);
                            self.frame_stats.point_replacements += 1;
                        }
                        (0.0, [0.0, 0.0], contact.normal)
                    }
                };

                let mut warm_contact = contact.clone();
                warm_contact.normal = contact_normal;
                warm_contact.warm_normal_impulse = warm_normal;
                warm_contact.warm_tangent_impulse = warm_tangent;
                result.push(warm_contact);
            }
        }

        result
    }

    /// Write solved impulses back into the manifold cache for next frame's warm-start.
    pub fn write_back(
        &mut self,
        constraints: &[ContactConstraint],
        solved: &[SolvedImpulses],
        bodies: &Arena<RigidBody>,
    ) {
        let mut used_indices: HashMap<ManifoldKey, HashSet<usize>> = HashMap::new();

        for (contact, impulse) in constraints.iter().zip(solved.iter()) {
            let Some(col_b) = contact.collider_b else {
                continue;
            };
            let key = ManifoldKey::new(contact.collider_a, col_b);

            let Some(manifold) = self.manifolds.get_mut(&key) else {
                continue;
            };

            let (_, local_b) = to_local_space(contact, bodies);
            let used = used_indices.entry(key).or_default();
            let match_idx =
                find_closest_point_excluding(manifold, &local_b, self.match_threshold, used);

            if let Some(idx) = match_idx {
                used.insert(idx);
                let point = &mut manifold.points[idx];
                point.normal_impulse = impulse.normal;
                point.tangent_impulse = impulse.tangent;
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

/// Transform a contact point to body-local space for both bodies.
fn to_local_space(
    contact: &ContactConstraint,
    bodies: &Arena<RigidBody>,
) -> (Point3<f32>, Point3<f32>) {
    let local_a = match contact.body_a {
        Some(handle) => {
            if let Some(body) = bodies.get(handle.0) {
                world_to_local(contact.point, body.position(), body.rotation())
            } else {
                contact.point
            }
        }
        None => contact.point, // Static: local == world
    };

    let local_b = if let Some(body) = bodies.get(contact.body_b.0) {
        world_to_local(contact.point, body.position(), body.rotation())
    } else {
        contact.point
    };

    (local_a, local_b)
}

fn world_to_local(
    point: Point3<f32>,
    body_pos: Point3<f32>,
    body_rot: UnitQuaternion<f32>,
) -> Point3<f32> {
    Point3::from(body_rot.inverse() * (point - body_pos))
}


fn find_closest_point_excluding(
    manifold: &ContactManifold,
    local_b: &Point3<f32>,
    threshold: f32,
    exclude: &HashSet<usize>,
) -> Option<usize> {
    let threshold_sq = threshold * threshold;
    let mut best_idx = None;
    let mut best_dist_sq = threshold_sq;

    for (i, point) in manifold.points.iter().enumerate() {
        if exclude.contains(&i) {
            continue;
        }
        let dist_sq = (point.local_point_b - local_b).magnitude_squared();
        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_idx = Some(i);
        }
    }

    best_idx
}

fn find_shallowest_unmatched(
    manifold: &ContactManifold,
    exclude: &HashSet<usize>,
) -> Option<usize> {
    manifold
        .points
        .iter()
        .enumerate()
        .filter(|(i, _)| !exclude.contains(i))
        .min_by(|(_, a), (_, b)| {
            a.depth
                .partial_cmp(&b.depth)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(i, _)| i)
}

/// Reduce contacts while biasing toward cached manifold points.
///
/// This preserves temporal stability by keeping points that already exist in the manifold
/// whenever they are within the match threshold. After the history bias is applied, the
/// remaining slots are filled deterministically so the selection does not flicker between
/// frames. The result is a stable, size-limited contact set that still reflects the current
/// narrowphase output.
fn reduce_contacts_with_manifold<'a>(
    contacts: Vec<&'a ContactConstraint>,
    manifold: &ContactManifold,
    bodies: &Arena<RigidBody>,
    max_points: usize,
    match_threshold: f32,
) -> Vec<&'a ContactConstraint> {
    if contacts.len() <= max_points {
        return contacts;
    }

    let locals: Vec<Point3<f32>> = contacts
        .iter()
        .map(|contact| to_local_space(contact, bodies).1)
        .collect();
    let mut selected: Vec<usize> = Vec::new();

    let threshold_sq = match_threshold * match_threshold;
    for point in &manifold.points {
        let mut best_idx = None;
        let mut best_dist_sq = threshold_sq;
        for (idx, local_b) in locals.iter().enumerate() {
            if selected.contains(&idx) {
                continue;
            }
            let dist_sq = (point.local_point_b - *local_b).magnitude_squared();
            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                best_idx = Some(idx);
            }
        }
        if let Some(idx) = best_idx {
            selected.push(idx);
        }
    }

    if selected.len() > max_points {
        selected.sort_by(|a, b| compare_contacts(contacts[*a], contacts[*b]));
        selected.truncate(max_points);
    }

    if selected.is_empty() {
        let mut indices: Vec<usize> = (0..contacts.len()).collect();
        indices.sort_by(|a, b| compare_contacts(contacts[*a], contacts[*b]));
        if let Some(idx) = indices.first() {
            selected.push(*idx);
        }
    }

    let mut remaining: Vec<usize> = (0..contacts.len())
        .filter(|idx| !selected.contains(idx))
        .collect();
    remaining.sort_by(|a, b| compare_contacts(contacts[*a], contacts[*b]));

    for idx in remaining {
        if selected.len() >= max_points {
            break;
        }
        selected.push(idx);
    }

    selected.into_iter().map(|idx| contacts[idx]).collect()
}

/// Deterministic ordering for contact reduction.
///
/// This orders by deeper contacts first, then by raw depth, then by position and normal
/// components. The goal is stability rather than geometric optimality, so ties are broken
/// consistently across frames. This helps prevent oscillation caused by nondeterministic
/// contact selection.
fn compare_contacts(a: &ContactConstraint, b: &ContactConstraint) -> std::cmp::Ordering {
    let depth = b
        .depth
        .partial_cmp(&a.depth)
        .unwrap_or(std::cmp::Ordering::Equal);
    if depth != std::cmp::Ordering::Equal {
        return depth;
    }
    let raw = b
        .raw_depth
        .partial_cmp(&a.raw_depth)
        .unwrap_or(std::cmp::Ordering::Equal);
    if raw != std::cmp::Ordering::Equal {
        return raw;
    }
    let point_x = a
        .point
        .x
        .partial_cmp(&b.point.x)
        .unwrap_or(std::cmp::Ordering::Equal);
    if point_x != std::cmp::Ordering::Equal {
        return point_x;
    }
    let point_y = a
        .point
        .y
        .partial_cmp(&b.point.y)
        .unwrap_or(std::cmp::Ordering::Equal);
    if point_y != std::cmp::Ordering::Equal {
        return point_y;
    }
    let point_z = a
        .point
        .z
        .partial_cmp(&b.point.z)
        .unwrap_or(std::cmp::Ordering::Equal);
    if point_z != std::cmp::Ordering::Equal {
        return point_z;
    }
    let normal_x = a
        .normal
        .x
        .partial_cmp(&b.normal.x)
        .unwrap_or(std::cmp::Ordering::Equal);
    if normal_x != std::cmp::Ordering::Equal {
        return normal_x;
    }
    let normal_y = a
        .normal
        .y
        .partial_cmp(&b.normal.y)
        .unwrap_or(std::cmp::Ordering::Equal);
    if normal_y != std::cmp::Ordering::Equal {
        return normal_y;
    }
    a.normal
        .z
        .partial_cmp(&b.normal.z)
        .unwrap_or(std::cmp::Ordering::Equal)
}
