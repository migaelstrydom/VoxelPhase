//! Unified narrowphase contact generation for all body-vs-body collider pairs.

use rustc_hash::{FxHashMap, FxHashSet};

use generational_arena::Arena;

use crate::collision::contact::ContactManifold;
use crate::collision::discrete::gjk::GjkCache;
use crate::collision::dispatch;
use crate::collision::sat::SatCache;
use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderMaterial, ColliderShape};
use crate::physics::handle::ColliderHandle;
use crate::physics::pipeline::pair::{PairHeader, PairManifold};

use super::collider_state::{collect_collider_states_into, ColliderState};
use super::config::{ContactHorizon, NarrowphaseConfig};
use super::scope::ContactScope;
use super::speculative::{rewind_to_now, time_of_impact};
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

/// Generate contacts between all pairs of body colliders.
///
/// Covers dynamic, kinematic and static bodies alike; a static body is an
/// ordinary collider of infinite mass, and only pairs where *neither* side can
/// move are skipped. Static level *geometry* is a separate concern, handled by
/// [`generate_static_contacts`](super::generate_static_contacts).
///
/// Uses sort-and-sweep broadphase on the axis of greatest positional spread
/// to prune pairs before narrowphase dispatch. Manifolds are appended to `buf`,
/// after any the static pass has already contributed, and can be read via
/// [`NarrowphaseWorkBuffer::manifolds`]. Only pairs some body in `scope`
/// starts are generated.
///
/// The work buffer's internal `Vec`s are reused across frames — only cleared,
/// never deallocated — eliminating per-frame allocation overhead.
pub fn generate_dynamic_contacts(
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    config: &NarrowphaseConfig,
    horizon: ContactHorizon,
    scope: ContactScope<'_>,
    sat_cache_map: &mut SatCacheMap,
    gjk_cache_map: &mut GjkCacheMap,
    buf: &mut NarrowphaseWorkBuffer,
) {
    let contact_margin = config.contact_margin;
    buf.clear_pair_scratch();

    collect_collider_states_into(&mut buf.states, bodies, colliders, scope);
    if buf.states.len() < 2 {
        return;
    }

    // A collider in the speculative band is bounded over the whole frame's
    // travel, not where it stands: a pair that will meet this frame has to be
    // paired now, or there is nothing to predict for.
    let frame_dt = horizon.frame_dt();
    buf.speculative.reserve(buf.states.len());
    buf.speculative.extend(buf.states.iter().map(|s| {
        s.is_mobile()
            && config.admits_speculative(s.velocity.magnitude(), horizon, s.shape.bounding_radius())
    }));
    buf.bounds.reserve(buf.states.len());
    buf.bounds.extend(
        buf.states
            .iter()
            .zip(&buf.speculative)
            .map(|(s, &speculative)| {
                if speculative {
                    s.swept_bounds(contact_margin, s.velocity * frame_dt)
                } else {
                    s.bounds(contact_margin)
                }
            }),
    );
    let states = &buf.states;
    buf.broadphase
        .pairs_into(&buf.bounds, |i| states[i].is_mobile(), &mut buf.pairs);

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

        // The dispatch routes mixed pairs to a canonical shape order
        // (e.g. obb_capsule_manifold always has OBB first), so the contact
        // normal always points from the "larger" shape toward the "smaller".
        // The solver convention is normal from A→B, so body_a must be the
        // larger shape type to match.
        let (first, second) = if shape_type_rank(&si.shape) >= shape_type_rank(&sj.shape) {
            (si, sj)
        } else {
            (sj, si)
        };

        let predict = manifold.is_empty()
            && (buf.speculative[i] || buf.speculative[j])
            && config.admits_speculative_pair((si.velocity - sj.velocity).magnitude(), horizon);
        let manifold = if predict {
            predict_pair_manifold(first, second, contact_margin, frame_dt)
        } else {
            debug_assert!(
                normals_point_from_a_to_b(first, second, &manifold),
                "Contact normal appears to point from B toward A for {:?} vs {:?}. \
                 This usually means the body_a/body_b ordering in PairHeader \
                 doesn't match the manifold's normal convention.",
                first.shape,
                second.shape,
            );
            manifold
        };
        push_if_nonempty(
            &mut buf.manifolds,
            make_pair_header(first, second),
            manifold,
        );
    }
}

/// Drop the SAT and GJK caches of pairs no pass tested this frame.
pub fn prune_pair_caches(
    sat_cache_map: &mut SatCacheMap,
    gjk_cache_map: &mut GjkCacheMap,
    buf: &NarrowphaseWorkBuffer,
) {
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

/// Sanity check on the solver convention: the contact normal should roughly
/// point from collider `a` toward collider `b`. A flipped normal inverts the
/// impulse direction, causing penetration instead of separation.
///
/// Judged between the *collider* centres, not the bodies': a compound body's
/// origin can be metres from the child that is actually touching, and a
/// normal that is perfectly right for the child can point anywhere at all
/// relative to the body. The threshold is generous (-0.5 ≈ 120°) to allow
/// edge and corner contacts where the normal is perpendicular to the
/// centre-to-centre axis.
#[allow(dead_code)]
fn normals_point_from_a_to_b(
    a: &ColliderState,
    b: &ColliderState,
    manifold: &ContactManifold,
) -> bool {
    let ab = b.center - a.center;
    let length = ab.magnitude();
    if length < 1e-3 {
        return true;
    }
    manifold
        .points
        .iter()
        .all(|contact| contact.normal.dot(&ab) / length > -0.5)
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
fn make_pair_header(a: &ColliderState, b: &ColliderState) -> PairHeader {
    let (restitution, friction) = ColliderMaterial::combine(&a.material, &b.material);
    PairHeader {
        body_a: Some(a.body_handle),
        body_b: b.body_handle,
        collider_a: Some(a.collider_handle),
        collider_b: Some(b.collider_handle),
        restitution,
        friction,
    }
}

/// Push a `PairManifold` into the output buffer if the manifold is non-empty.
fn push_if_nonempty(out: &mut Vec<PairManifold>, header: PairHeader, manifold: ContactManifold) {
    if !manifold.is_empty() {
        out.push(PairManifold { header, manifold });
    }
}

/// Speculative contacts for a pair that is apart now but meets within
/// `frame_dt`: the manifold where it meets, rewound to the present with the gap
/// each point has still to close. Empty if the pair does not meet.
///
/// `first` and `second` are in the solver's A/B order. The manifold at the
/// meeting pose comes from the same dispatch as any other contact, so it has as
/// many points, and the same normal convention, as the contact the pair will
/// have once it arrives.
fn predict_pair_manifold(
    first: &ColliderState,
    second: &ColliderState,
    contact_margin: f32,
    frame_dt: f32,
) -> ContactManifold {
    let Some(t) = time_of_impact(first, second, frame_dt) else {
        return ContactManifold::empty();
    };
    let travel_first = first.velocity * (frame_dt * t);
    let travel_second = second.velocity * (frame_dt * t);
    let mut manifold = dispatch::generate_manifold(
        &first.view_moved(travel_first),
        &second.view_moved(travel_second),
        contact_margin,
        None,
        None,
    );
    rewind_to_now(&mut manifold, Some(travel_first), travel_second);
    manifold
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
