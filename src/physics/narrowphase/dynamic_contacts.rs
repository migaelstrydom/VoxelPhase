//! Unified narrowphase contact generation for all dynamic-vs-dynamic collider pairs.

use rustc_hash::{FxHashMap, FxHashSet};

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::collision::contact::{ContactManifold, ContactPoint, FeatureId};
use crate::collision::continuous::swept_sphere_sphere;
use crate::collision::discrete::gjk::GjkCache;
use crate::collision::dispatch;
use crate::collision::sat::SatCache;
use crate::collision::AABB;
use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderMaterial, ColliderShape};
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::pipeline::pair::{PairHeader, PairManifold};

use super::collider_state::{collect_collider_states_into, ColliderState};
use super::config::NarrowphaseConfig;
use super::work_buffer::NarrowphaseWorkBuffer;

/// Pair key for SAT axis caching. Collider handles are stored in canonical order.
#[derive(Hash, Eq, PartialEq, Clone, Copy)]
struct SatPairKey(ColliderHandle, ColliderHandle);

impl SatPairKey {
    fn new(a: ColliderHandle, b: ColliderHandle) -> Self {
        let (idx_a, _) = a.raw_parts();
        let (idx_b, _) = b.raw_parts();
        if idx_a <= idx_b {
            Self(a, b)
        } else {
            Self(b, a)
        }
    }
}

/// Per-frame SAT axis cache for OBB-OBB pairs.
///
/// Stores the last separating axis found for each pair. When the cached axis
/// still separates the pair next frame, the full 15-axis SAT test is skipped.
pub struct SatCacheMap {
    caches: FxHashMap<SatPairKey, SatCache>,
}

impl SatCacheMap {
    pub fn new() -> Self {
        Self {
            caches: FxHashMap::default(),
        }
    }

    /// Remove entries for pairs that were not tested this frame.
    pub fn prune(&mut self, active_pairs: &FxHashSet<(ColliderHandle, ColliderHandle)>) {
        self.caches.retain(|key, _| {
            active_pairs.contains(&(key.0, key.1)) || active_pairs.contains(&(key.1, key.0))
        });
    }
}

/// Pair key for GJK warm-start direction caching.
#[derive(Hash, Eq, PartialEq, Clone, Copy)]
struct GjkPairKey(ColliderHandle, ColliderHandle);

impl GjkPairKey {
    fn new(a: ColliderHandle, b: ColliderHandle) -> Self {
        let (idx_a, _) = a.raw_parts();
        let (idx_b, _) = b.raw_parts();
        if idx_a <= idx_b {
            Self(a, b)
        } else {
            Self(b, a)
        }
    }
}

/// Per-frame GJK warm-start cache map for wildcard dispatch pairs.
pub struct GjkCacheMap {
    caches: FxHashMap<GjkPairKey, GjkCache>,
}

impl GjkCacheMap {
    pub fn new() -> Self {
        Self {
            caches: FxHashMap::default(),
        }
    }

    /// Remove entries for pairs that were not tested this frame.
    pub fn prune(&mut self, active_pairs: &FxHashSet<(ColliderHandle, ColliderHandle)>) {
        self.caches.retain(|key, _| {
            active_pairs.contains(&(key.0, key.1)) || active_pairs.contains(&(key.1, key.0))
        });
    }
}

/// Generate contacts between all pairs of non-static colliders.
///
/// Uses sort-and-sweep broadphase on the axis of greatest positional spread
/// to prune pairs before narrowphase dispatch. Manifolds are appended to `buf`,
/// after any the static pass has already contributed, and can be read via
/// [`NarrowphaseWorkBuffer::manifolds`].
///
/// The work buffer's internal `Vec`s are reused across frames — only cleared,
/// never deallocated — eliminating per-frame allocation overhead.
pub fn generate_dynamic_contacts(
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    config: &NarrowphaseConfig,
    dt: f32,
    sleeping: Option<&FxHashSet<RigidBodyHandle>>,
    sat_cache_map: &mut SatCacheMap,
    gjk_cache_map: &mut GjkCacheMap,
    buf: &mut NarrowphaseWorkBuffer,
) {
    let contact_margin = config.contact_margin;
    buf.clear_pair_scratch();

    collect_collider_states_into(&mut buf.states, bodies, colliders, sleeping);
    if buf.states.len() < 2 {
        return;
    }

    sweep_and_prune_into(
        &buf.states,
        contact_margin,
        &mut buf.bounds,
        &mut buf.sorted_indices,
        &mut buf.pairs,
    );

    for pair_idx in 0..buf.pairs.len() {
        let (i, j) = buf.pairs[pair_idx];
        let si = &buf.states[i];
        let sj = &buf.states[j];

        if si.body_handle == sj.body_handle {
            continue;
        }

        let view_a = si.view();
        let view_b = sj.view();

        // SAT cache: look up for Box-Box, ConvexHull-ConvexHull, and Box-ConvexHull pairs.
        let mut sat_cache_opt = match (&si.shape, &sj.shape) {
            (ColliderShape::Box { .. }, ColliderShape::Box { .. })
            | (ColliderShape::ConvexHull { .. }, ColliderShape::ConvexHull { .. })
            | (ColliderShape::Box { .. }, ColliderShape::ConvexHull { .. })
            | (ColliderShape::ConvexHull { .. }, ColliderShape::Box { .. }) => {
                buf.active_sat_pairs
                    .insert((si.collider_handle, sj.collider_handle));
                let key = SatPairKey::new(si.collider_handle, sj.collider_handle);
                Some(sat_cache_map.caches.entry(key).or_default())
            }
            _ => None,
        };

        let mut gjk_cache_opt = if requires_gjk_fallback(&si.shape, &sj.shape) {
            buf.active_gjk_pairs
                .insert((si.collider_handle, sj.collider_handle));
            let key = GjkPairKey::new(si.collider_handle, sj.collider_handle);
            Some(gjk_cache_map.caches.entry(key).or_default())
        } else {
            None
        };

        let manifold = dispatch::generate_manifold(
            &view_a,
            &view_b,
            contact_margin,
            sat_cache_opt.as_deref_mut(),
            gjk_cache_opt.as_deref_mut(),
        );

        if !manifold.is_empty() {
            // The dispatch routes mixed pairs to a canonical shape order
            // (e.g. obb_capsule_manifold always has OBB first), so the contact
            // normal always points from the "larger" shape toward the "smaller".
            // The solver convention is normal from A→B, so body_a must be the
            // larger shape type to match.
            let rep_normal = manifold_representative_normal(&manifold);
            let header = if shape_type_rank(&si.shape) >= shape_type_rank(&sj.shape) {
                make_pair_header(si, sj, &rep_normal)
            } else {
                make_pair_header(sj, si, &rep_normal)
            };
            push_if_nonempty(&mut buf.manifolds, header, manifold);
        } else {
            // Speculative CCD for sphere-sphere pairs.
            if let (ColliderShape::Sphere { radius: ra }, ColliderShape::Sphere { radius: rb }) =
                (&si.shape, &sj.shape)
            {
                sphere_sphere_speculative(si, *ra, sj, *rb, config, dt, &mut buf.manifolds);
            }
        }
    }

    sat_cache_map.prune(&buf.active_sat_pairs);
    gjk_cache_map.prune(&buf.active_gjk_pairs);
}

/// Rank shape types for deterministic body_a/body_b ordering.
///
/// The dispatch module routes mixed-type pairs to manifold functions that
/// always take the "larger" shape first (e.g. `obb_capsule_manifold(obb, capsule)`).
/// The resulting contact normal points from the larger shape toward the smaller.
/// The solver convention is normal from body_a → body_b, so body_a must be the
/// higher-ranked shape type.
fn shape_type_rank(shape: &ColliderShape) -> u8 {
    match shape {
        ColliderShape::ConvexHull { .. } => 3,
        ColliderShape::Box { .. } => 2,
        ColliderShape::Capsule { .. } => 1,
        ColliderShape::Sphere { .. } => 0,
    }
}

fn requires_gjk_fallback(a: &ColliderShape, b: &ColliderShape) -> bool {
    !matches!(
        (a, b),
        (ColliderShape::Sphere { .. }, ColliderShape::Sphere { .. })
            | (ColliderShape::Sphere { .. }, ColliderShape::Box { .. })
            | (ColliderShape::Box { .. }, ColliderShape::Sphere { .. })
            | (ColliderShape::Box { .. }, ColliderShape::Box { .. })
            | (ColliderShape::Sphere { .. }, ColliderShape::Capsule { .. })
            | (ColliderShape::Capsule { .. }, ColliderShape::Sphere { .. })
            | (ColliderShape::Box { .. }, ColliderShape::Capsule { .. })
            | (ColliderShape::Capsule { .. }, ColliderShape::Box { .. })
            | (ColliderShape::Capsule { .. }, ColliderShape::Capsule { .. })
    )
}

/// Sort-and-sweep broadphase, writing candidate pairs into pre-allocated buffers.
fn sweep_and_prune_into(
    states: &[ColliderState],
    margin: f32,
    bounds: &mut Vec<AABB>,
    sorted: &mut Vec<usize>,
    pairs: &mut Vec<(usize, usize)>,
) {
    let sweep_axis = pick_sweep_axis(states);

    bounds.reserve(states.len());
    bounds.extend(states.iter().map(|s| s.bounds(margin)));

    sorted.extend(0..states.len());
    sorted
        .sort_unstable_by(|&a, &b| bounds[a].min[sweep_axis].total_cmp(&bounds[b].min[sweep_axis]));

    for ii in 0..sorted.len() {
        let i = sorted[ii];
        let i_max = bounds[i].max[sweep_axis];

        for jj in (ii + 1)..sorted.len() {
            let j = sorted[jj];

            if bounds[j].min[sweep_axis] > i_max {
                break;
            }

            if states[i].is_sleeping && states[j].is_sleeping {
                continue;
            }

            if bounds[i].intersects(&bounds[j]) {
                pairs.push((i, j));
            }
        }
    }
}

/// Pick the axis (0=x, 1=y, 2=z) with the greatest positional spread.
fn pick_sweep_axis(states: &[ColliderState]) -> usize {
    let mut min = [f32::MAX; 3];
    let mut max = [f32::MIN; 3];
    for s in states {
        let c = [s.center.x, s.center.y, s.center.z];
        for axis in 0..3 {
            min[axis] = min[axis].min(c[axis]);
            max[axis] = max[axis].max(c[axis]);
        }
    }
    let spread = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
    if spread[0] >= spread[1] && spread[0] >= spread[2] {
        0
    } else if spread[1] >= spread[2] {
        1
    } else {
        2
    }
}

/// Build a `PairHeader` from two collider states with combined material.
///
/// **A/B ordering invariant:** The solver convention is that contact normals
/// point from body_a toward body_b. The dispatch module's manifold functions
/// produce normals pointing from the first shape argument toward the second
/// (e.g. `sphere_obb_manifold(obb, sphere_center, ..)` → normal points OBB→sphere).
/// Callers must ensure that `a` corresponds to the shape that dispatch treats
/// as "first" — use `shape_type_rank` to enforce this for mixed-type pairs.
/// Getting this wrong inverts the impulse direction and causes penetration.
fn make_pair_header(
    a: &ColliderState,
    b: &ColliderState,
    contact_normal_world: &Vector3<f32>,
) -> PairHeader {
    let (restitution, friction) = ColliderMaterial::combine_at(
        &a.material,
        &b.material,
        contact_normal_world,
        &a.rotation,
        &b.rotation,
    );
    PairHeader {
        body_a: Some(a.body_handle),
        body_b: b.body_handle,
        collider_a: Some(a.collider_handle),
        collider_b: Some(b.collider_handle),
        restitution,
        friction,
    }
}

/// Pick a representative world-space normal for a manifold (deepest contact).
fn manifold_representative_normal(manifold: &ContactManifold) -> Vector3<f32> {
    manifold
        .points
        .iter()
        .max_by(|a, b| {
            a.depth
                .partial_cmp(&b.depth)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|cp| cp.normal)
        .unwrap_or_else(Vector3::y)
}

/// Push a `PairManifold` into the output buffer if the manifold is non-empty.
fn push_if_nonempty(out: &mut Vec<PairManifold>, header: PairHeader, manifold: ContactManifold) {
    if !manifold.is_empty() {
        out.push(PairManifold { header, manifold });
    }
}

/// Sphere-sphere speculative CCD (special case that uses swept_sphere_sphere).
fn sphere_sphere_speculative(
    a: &ColliderState,
    radius_a: f32,
    b: &ColliderState,
    radius_b: f32,
    config: &NarrowphaseConfig,
    dt: f32,
    out: &mut Vec<PairManifold>,
) {
    if !config.admits_speculative_pair(
        a.velocity.magnitude(),
        radius_a,
        b.velocity.magnitude(),
        radius_b,
        dt,
    ) {
        return;
    }
    let contact_margin = config.contact_margin;

    let end_a = a.center + a.velocity * dt;
    let end_b = b.center + b.velocity * dt;
    if let Some(t) = swept_sphere_sphere(
        a.center,
        end_a,
        radius_a + contact_margin,
        b.center,
        end_b,
        radius_b + contact_margin,
    ) {
        let pos_a = a.center + (end_a - a.center) * t;
        let pos_b = b.center + (end_b - b.center) * t;
        let delta = pos_b - pos_a;
        let dist = delta.magnitude();
        let normal = if dist < 1e-6 {
            Vector3::y()
        } else {
            delta / dist
        };
        let actual_depth = (radius_a + radius_b) - dist;
        let solver_depth = actual_depth.max(0.0);
        let point = pos_a + normal * (radius_a - actual_depth * 0.5);

        let cp = ContactPoint::new(point, normal, actual_depth, FeatureId::SINGLE);
        let mut speculative = ContactManifold::single(cp);
        speculative.points[0].depth = solver_depth;

        out.push(PairManifold {
            header: make_pair_header(a, b, &normal),
            manifold: speculative,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::dispatch;
    use crate::collision::shape_view::ShapeView;
    use nalgebra::{Point3, UnitQuaternion, Vector3};

    /// Verify that shape_type_rank produces the expected hierarchy:
    /// Box > Capsule > Sphere, matching the dispatch normal convention.
    #[test]
    fn shape_type_rank_hierarchy() {
        let sphere = ColliderShape::Sphere { radius: 1.0 };
        let capsule = ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        };
        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };

        assert!(shape_type_rank(&box_shape) > shape_type_rank(&capsule));
        assert!(shape_type_rank(&capsule) > shape_type_rank(&sphere));
        assert!(shape_type_rank(&box_shape) > shape_type_rank(&sphere));
    }

    /// The critical invariant: when a capsule (lower broadphase index) collides
    /// with a box, the contact normal must point from body_a toward body_b.
    ///
    /// This test simulates the scenario that caused penetration: the capsule
    /// happened to be `si` and the box `sj` due to broadphase ordering. Without
    /// the shape_type_rank fix, body_a would be the capsule, but the normal
    /// from `obb_capsule_manifold` points OBB→capsule, meaning B→A — inverted.
    #[test]
    fn capsule_box_pair_normal_points_a_to_b() {
        // Capsule at origin, box to the right — overlapping.
        let capsule_shape = ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.5,
        };
        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let rot = UnitQuaternion::identity();

        let capsule_center = Point3::new(0.0, 0.0, 0.0);
        let box_center = Point3::new(0.8, 0.0, 0.0);

        // Simulate broadphase putting capsule first (lower index).
        let view_capsule = ShapeView {
            center: capsule_center,
            rotation: rot,
            shape: &capsule_shape,
        };
        let view_box = ShapeView {
            center: box_center,
            rotation: rot,
            shape: &box_shape,
        };

        let manifold = dispatch::generate_manifold(&view_capsule, &view_box, 0.02, None, None);
        assert!(!manifold.is_empty(), "Should have contacts");

        // Apply the same ordering logic used in generate_dynamic_contacts.
        let capsule_is_a = shape_type_rank(&capsule_shape) >= shape_type_rank(&box_shape);

        for cp in &manifold.points {
            let (pos_a, pos_b) = if capsule_is_a {
                (capsule_center, box_center)
            } else {
                (box_center, capsule_center)
            };
            let a_to_b = pos_b - pos_a;
            let dot = cp.normal.dot(&a_to_b);
            assert!(
                dot > 0.0,
                "Normal should point from body_a toward body_b. \
                 normal={:?}, a_to_b={:?}, dot={:.3}. \
                 capsule_is_a={}, which means body_a is {}.",
                cp.normal,
                a_to_b,
                dot,
                capsule_is_a,
                if capsule_is_a { "capsule" } else { "box" },
            );
        }
    }

    /// Same test for sphere-box pair.
    #[test]
    fn sphere_box_pair_normal_points_a_to_b() {
        let sphere_shape = ColliderShape::Sphere { radius: 0.5 };
        let box_shape = ColliderShape::Box {
            half_extents: Vector3::new(0.5, 0.5, 0.5),
        };
        let rot = UnitQuaternion::identity();

        let sphere_center = Point3::new(0.0, 0.0, 0.0);
        let box_center = Point3::new(0.8, 0.0, 0.0);

        let view_sphere = ShapeView {
            center: sphere_center,
            rotation: rot,
            shape: &sphere_shape,
        };
        let view_box = ShapeView {
            center: box_center,
            rotation: rot,
            shape: &box_shape,
        };

        let manifold = dispatch::generate_manifold(&view_sphere, &view_box, 0.02, None, None);
        assert!(!manifold.is_empty(), "Should have contacts");

        let sphere_is_a = shape_type_rank(&sphere_shape) >= shape_type_rank(&box_shape);

        for cp in &manifold.points {
            let (pos_a, pos_b) = if sphere_is_a {
                (sphere_center, box_center)
            } else {
                (box_center, sphere_center)
            };
            let dot = cp.normal.dot(&(pos_b - pos_a));
            assert!(
                dot > 0.0,
                "Normal should point from body_a toward body_b (dot={:.3})",
                dot,
            );
        }
    }
}
