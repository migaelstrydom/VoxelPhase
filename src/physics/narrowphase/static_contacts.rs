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
use crate::physics::collider::Collider;
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};
use crate::physics::pipeline::pair::{PairHeader, PairManifold};
use crate::physics::static_geometry::StaticGeometry;

use super::config::NarrowphaseConfig;
use super::work_buffer::NarrowphaseWorkBuffer;

/// Generate contacts between all non-static colliders and static geometry.
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
    dt: f32,
    sleeping: Option<&FxHashSet<RigidBodyHandle>>,
    buf: &mut NarrowphaseWorkBuffer,
) {
    let awake_colliders: Vec<(RigidBodyHandle, &RigidBody, ColliderHandle)> = bodies
        .iter()
        .filter(|(idx, body)| {
            !body.is_static()
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
                dt,
            )
        })
        .collect();
    buf.manifolds.extend(manifolds.into_iter().flatten());
}

/// The manifold of one collider against static geometry: its contacts now, or
/// if it has none and is fast enough, speculative contacts where it will be
/// after `dt`.
fn collider_vs_static(
    body_handle: RigidBodyHandle,
    body: &RigidBody,
    collider_handle: ColliderHandle,
    collider: &Collider,
    static_geometry: &dyn StaticGeometry,
    config: &NarrowphaseConfig,
    dt: f32,
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
        && config.admits_speculative(speed, dt, collider.shape().bounding_radius())
    {
        let predicted_center = center + linear_velocity * dt;
        let predicted_view = ShapeView {
            center: predicted_center,
            rotation: body.rotation(),
            shape: collider.shape(),
        };
        let mut m = shape_vs_static(&predicted_view, static_geometry, contact_margin);
        make_speculative(&mut m, contact_margin);
        m
    } else {
        manifold
    };

    if manifold.is_empty() {
        return None;
    }
    Some(PairManifold {
        header: PairHeader {
            body_a: None,
            body_b: body_handle,
            collider_a: None,
            collider_b: Some(collider_handle),
            restitution: collider.material().restitution,
            friction: collider.material().friction(),
        },
        manifold,
    })
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

/// Convert a manifold to speculative: depth=0, raw_depth=-margin.
fn make_speculative(manifold: &mut ContactManifold, contact_margin: f32) {
    for cp in &mut manifold.points {
        cp.depth = 0.0;
        cp.raw_depth = -contact_margin;
    }
}
