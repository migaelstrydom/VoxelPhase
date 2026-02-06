//! Contact manifold persistence across frames.
//!
//! The manifold cache stores contact points in body-local space so they can be
//! matched across frames even as bodies move. Matched contacts inherit their
//! accumulated impulses from the previous frame, enabling warm-starting in the
//! solver. This is the single biggest stability improvement for resting and
//! stacking contacts.

use std::collections::HashMap;

use generational_arena::Arena;
use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::handle::ColliderHandle;
use crate::physics::pipeline::solver::{ContactConstraint, SolvedImpulses};

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
    max_age: u8,
}

impl ManifoldCache {
    pub fn new(match_threshold: f32, max_age: u8) -> Self {
        Self {
            manifolds: HashMap::new(),
            match_threshold,
            max_age,
        }
    }

    /// Merge raw narrowphase contacts with cached manifolds, returning solver-ready
    /// constraints with warm-start impulses populated from the cache.
    pub fn update(
        &mut self,
        raw_contacts: &[ContactConstraint],
        bodies: &Arena<RigidBody>,
    ) -> Vec<ContactConstraint> {
        // Age all existing points before processing new contacts
        for manifold in self.manifolds.values_mut() {
            for point in &mut manifold.points {
                point.age += 1;
            }
        }

        let mut result = Vec::with_capacity(raw_contacts.len());

        for contact in raw_contacts {
            let Some(col_b) = contact.collider_b else {
                // Transient contacts (e.g. CCD) without collider info skip the cache
                result.push(contact.clone());
                continue;
            };

            let key = ManifoldKey::new(contact.collider_a, col_b);

            // Transform contact point to local space
            let (local_a, local_b) = to_local_space(contact, bodies);

            let manifold = self
                .manifolds
                .entry(key)
                .or_insert_with(ContactManifold::new);

            // Find closest existing point in the manifold
            let match_idx = find_closest_point(manifold, &local_b, self.match_threshold);

            let (warm_normal, warm_tangent) = match match_idx {
                Some(idx) => {
                    // Matched: inherit cached impulses, update point
                    let cached = &mut manifold.points[idx];
                    let warm = (cached.normal_impulse, cached.tangent_impulse);
                    cached.local_point_a = local_a;
                    cached.local_point_b = local_b;
                    cached.normal = contact.normal;
                    cached.depth = contact.depth;
                    cached.age = 0;
                    warm
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
                    } else {
                        // Replace the shallowest (least important) point
                        if let Some(replace_idx) = manifold
                            .points
                            .iter()
                            .enumerate()
                            .min_by(|(_, a), (_, b)| {
                                a.depth
                                    .partial_cmp(&b.depth)
                                    .unwrap_or(std::cmp::Ordering::Equal)
                            })
                            .map(|(i, _)| i)
                        {
                            manifold.points[replace_idx] = new_point;
                        }
                    }
                    (0.0, [0.0, 0.0])
                }
            };

            let mut warm_contact = contact.clone();
            warm_contact.warm_normal_impulse = warm_normal;
            warm_contact.warm_tangent_impulse = warm_tangent;
            result.push(warm_contact);
        }

        result
    }

    /// Write solved impulses back into the manifold cache for next frame's warm-start.
    pub fn write_back(&mut self, constraints: &[ContactConstraint], solved: &[SolvedImpulses]) {
        for (contact, impulse) in constraints.iter().zip(solved.iter()) {
            let Some(col_b) = contact.collider_b else {
                continue;
            };
            let key = ManifoldKey::new(contact.collider_a, col_b);

            let Some(manifold) = self.manifolds.get_mut(&key) else {
                continue;
            };

            // Find the fresh point (age == 0) that corresponds to this contact.
            // Since we process one raw contact per manifold key per frame, the
            // most recently updated point is the right one.
            if let Some(point) = manifold.points.iter_mut().find(|p| p.age == 0) {
                point.normal_impulse = impulse.normal;
                point.tangent_impulse = impulse.tangent;
            }
        }
    }

    /// Remove stale manifold points that haven't been refreshed within max_age frames.
    /// Remove empty manifolds entirely.
    pub fn prune(&mut self) {
        self.manifolds.retain(|_, manifold| {
            manifold.points.retain(|p| p.age <= self.max_age);
            !manifold.points.is_empty()
        });
    }

    /// Remove all manifolds involving the given collider.
    pub fn remove_collider(&mut self, handle: ColliderHandle) {
        self.manifolds
            .retain(|key, _| key.collider_a != Some(handle) && key.collider_b != handle);
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

/// Find the closest existing manifold point to a new contact (in body-B local space).
fn find_closest_point(
    manifold: &ContactManifold,
    local_b: &Point3<f32>,
    threshold: f32,
) -> Option<usize> {
    let threshold_sq = threshold * threshold;
    let mut best_idx = None;
    let mut best_dist_sq = threshold_sq;

    for (i, point) in manifold.points.iter().enumerate() {
        let dist_sq = (point.local_point_b - local_b).magnitude_squared();
        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_idx = Some(i);
        }
    }

    best_idx
}
