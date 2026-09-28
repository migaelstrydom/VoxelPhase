//! Unified narrowphase contact generation for all collider shapes vs static geometry.
//!
//! Uses the manifold-first mesh pipeline: seam filter merges coplanar triangles
//! into polygonal contact faces, then shape-specific patch routines generate
//! manifolds directly from the merged geometry.

use rayon::prelude::*;
use rustc_hash::FxHashSet;

use generational_arena::Arena;
use nalgebra::Point3;

use crate::collision::contact::ContactManifold;
use crate::collision::dispatch;
use crate::collision::mesh::seam_filter::{filter_patch, COPLANAR_DOT};
use crate::collision::shape_view::ShapeView;
use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderMaterial};
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::pipeline::pair::{PairHeader, PairManifold};
use crate::physics::static_geometry::StaticGeometry;

use super::config::{ContactHorizon, NarrowphaseConfig};
use super::speculative::rewind_to_now;
use super::work_buffer::NarrowphaseWorkBuffer;

/// Generate contacts between static geometry and every collider of an awake,
/// non-static body that does not ignore it.
///
/// Appends one `PairManifold` per collider that has contacts (or speculative
/// contacts) with static geometry. Each manifold carries the collision library's
/// `ContactManifold` with `FeatureId` per point, wrapped with body/collider/
/// material metadata.
///
/// Output is appended to `buf`, which the caller resets once per frame via
/// [`NarrowphaseWorkBuffer::begin_frame`]. Static contacts are generated before
/// dynamic ones, so they lead the buffer.
pub fn generate_static_contacts(
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    static_geometry: &dyn StaticGeometry,
    config: &NarrowphaseConfig,
    horizon: ContactHorizon,
    sleeping: Option<&FxHashSet<RigidBodyHandle>>,
    buf: &mut NarrowphaseWorkBuffer,
) {
    let awake_colliders: Vec<(RigidBodyHandle, &RigidBody, ColliderHandle)> = bodies
        .iter()
        .filter(|(idx, body)| {
            !body.is_static()
                && !body.ignores_static()
                && !sleeping.is_some_and(|sleeping| sleeping.contains(&RigidBodyHandle(*idx)))
        })
        .flat_map(|(idx, body)| {
            body.colliders()
                .iter()
                .map(move |&collider| (RigidBodyHandle(idx), body, collider))
        })
        .collect();

    // Each collider's query is independent; collecting keeps collider order,
    // so the buffer is the same whichever thread ran which query.
    let manifolds: Vec<Option<PairManifold>> = awake_colliders
        .par_iter()
        .map(|&(body_handle, body, collider_handle)| {
            let collider = colliders.get(collider_handle.0)?;
            collider_vs_static(
                body_handle,
                body,
                collider_handle,
                collider,
                static_geometry,
                config,
                horizon,
            )
        })
        .collect();
    buf.manifolds.extend(manifolds.into_iter().flatten());
}

/// The manifold of one collider against static geometry: its contacts now, or
/// if it has none and is in the speculative band, speculative contacts where it
/// will be at the end of `horizon`.
fn collider_vs_static(
    body_handle: RigidBodyHandle,
    body: &RigidBody,
    collider_handle: ColliderHandle,
    collider: &Collider,
    static_geometry: &dyn StaticGeometry,
    config: &NarrowphaseConfig,
    horizon: ContactHorizon,
) -> Option<PairManifold> {
    let contact_margin = config.contact_margin;
    let world_tf = collider.world_transform(body.position(), body.rotation());
    let center = Point3::from(world_tf.translation.vector);
    let rotation = world_tf.rotation;
    let linear_velocity = body.linear_velocity();
    let speed = linear_velocity.magnitude();

    let view = ShapeView {
        center,
        rotation,
        shape: collider.shape(),
    };
    let manifold = shape_vs_static(&view, static_geometry, contact_margin);

    let manifold = if manifold.is_empty()
        && config.admits_speculative(speed, horizon, collider.shape().bounding_radius())
    {
        let travel = linear_velocity * horizon.frame_dt();
        let predicted_view = ShapeView {
            center: center + travel,
            rotation,
            shape: collider.shape(),
        };
        let mut m = shape_vs_static(&predicted_view, static_geometry, contact_margin);
        rewind_to_now(&mut m, None, travel);
        m
    } else {
        manifold
    };

    if manifold.is_empty() {
        return None;
    }
    let (restitution, friction) =
        static_coefficients(collider.material(), &manifold, static_geometry);
    Some(PairManifold {
        header: PairHeader {
            body_a: None,
            body_b: body_handle,
            collider_a: None,
            collider_b: Some(collider_handle),
            restitution,
            friction,
        },
        manifold,
    })
}

/// Restitution and friction of a manifold against static geometry: what the
/// collider meets at each contact's surface, averaged over the contacts.
///
/// Per manifold rather than per contact because the solver holds one pair of
/// coefficients per manifold. The two differ only for a body straddling two
/// materials, where the mean is what its contacts share between them.
fn static_coefficients(
    material: &ColliderMaterial,
    manifold: &ContactManifold,
    static_geometry: &dyn StaticGeometry,
) -> (f32, f32) {
    let own = (material.restitution, material.friction());
    let (restitution, friction) = manifold
        .points
        .iter()
        .map(|cp| {
            static_geometry
                .surface(cp.surface)
                .map_or(own, |surface| surface.meet(material))
        })
        .fold((0.0, 0.0), |sum, (r, f)| (sum.0 + r, sum.1 + f));
    let count = manifold.points.len() as f32;
    (restitution / count, friction / count)
}

/// Generate contacts between a convex shape and static geometry via the mesh pipeline.
fn shape_vs_static(
    view: &ShapeView,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
) -> ContactManifold {
    let query = view.query_aabb(contact_margin);
    let patch = static_geometry.query_region(&query);
    let filtered = filter_patch(&patch, COPLANAR_DOT);
    dispatch::generate_mesh_manifold(view, &filtered, contact_margin)
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::collision::contact::{ContactPoint, FeatureId};
    use crate::collision::{MeshPatch, SurfaceId, AABB};
    use crate::physics::collider::FrictionModel;
    use crate::physics::static_surface::StaticSurface;

    /// Geometry whose only material is grass, under id 1.
    struct Lawn;

    impl StaticGeometry for Lawn {
        fn query_region(&self, _aabb: &AABB) -> MeshPatch {
            MeshPatch {
                triangles: Vec::new(),
            }
        }

        fn surface(&self, id: SurfaceId) -> Option<StaticSurface> {
            (id == SurfaceId(1)).then_some(StaticSurface::yielding(0.45, 0.2))
        }
    }

    fn manifold_on(surfaces: &[SurfaceId]) -> ContactManifold {
        ContactManifold {
            points: surfaces
                .iter()
                .map(|&s| {
                    ContactPoint::new(Point3::origin(), Vector3::y(), 0.0, FeatureId::SINGLE).on(s)
                })
                .collect(),
        }
    }

    const ICE: ColliderMaterial = ColliderMaterial {
        restitution: 0.15,
        friction: FrictionModel::Isotropic(0.06),
    };

    #[test]
    fn a_manifold_takes_the_surface_under_it() {
        let (_, friction) = static_coefficients(&ICE, &manifold_on(&[SurfaceId(1); 2]), &Lawn);
        assert_eq!(friction, 0.45);
    }

    #[test]
    fn unspecified_ground_leaves_the_collider_as_it_is() {
        let manifold = manifold_on(&[SurfaceId::UNSPECIFIED]);
        assert_eq!(static_coefficients(&ICE, &manifold, &Lawn), (0.15, 0.06));
    }

    #[test]
    fn a_manifold_straddling_two_surfaces_takes_their_mean() {
        let manifold = manifold_on(&[SurfaceId(1), SurfaceId::UNSPECIFIED]);
        let (_, friction) = static_coefficients(&ICE, &manifold, &Lawn);
        assert!((friction - (0.45 + 0.06) / 2.0).abs() < 1e-6);
    }
}
