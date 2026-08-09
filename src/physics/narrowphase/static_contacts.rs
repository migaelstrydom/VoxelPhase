//! Unified narrowphase contact generation for all collider shapes vs static geometry.
//!
//! Uses the manifold-first mesh pipeline: seam filter merges coplanar triangles
//! into polygonal contact faces, then shape-specific patch routines generate
//! manifolds directly from the merged geometry.

use rustc_hash::FxHashSet;

use generational_arena::Arena;
use nalgebra::{Point3, Vector3};

use crate::collision::contact::ContactManifold;
use crate::collision::dispatch;
use crate::collision::mesh::seam_filter::{filter_patch, COPLANAR_DOT};
use crate::collision::shape_view::ShapeView;
use crate::physics::body::RigidBody;
use crate::physics::collider::Collider;
use crate::physics::handle::RigidBodyHandle;
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
    let contact_margin = config.contact_margin;

    for (idx, body) in bodies.iter() {
        if body.is_static() {
            continue;
        }
        let body_handle = RigidBodyHandle(idx);
        if let Some(sleeping) = sleeping {
            if sleeping.contains(&body_handle) {
                continue;
            }
        }

        for collider_handle in body.colliders() {
            let Some(collider) = colliders.get(collider_handle.0) else {
                continue;
            };

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

            if !manifold.is_empty() {
                let rep_normal = representative_normal(&manifold);
                let friction = collider.material().friction_at(&rep_normal, &rotation);
                buf.manifolds.push(PairManifold {
                    header: PairHeader {
                        body_a: None,
                        body_b: body_handle,
                        collider_a: None,
                        collider_b: Some(*collider_handle),
                        restitution: collider.material().restitution,
                        friction,
                    },
                    manifold,
                });
            }
        }
    }
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

/// Pick a representative world-space normal for a manifold. Uses the deepest
/// contact — most faithful on mixed-normal manifolds (e.g. capsule on a step)
/// where the dominant interaction should drive friction selection.
fn representative_normal(manifold: &ContactManifold) -> Vector3<f32> {
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
