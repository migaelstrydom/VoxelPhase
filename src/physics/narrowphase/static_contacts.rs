//! Unified narrowphase contact generation for all collider shapes vs static geometry.
//!
//! Uses the manifold-first mesh pipeline: seam filter merges coplanar triangles
//! into polygonal contact faces, then shape-specific patch routines generate
//! manifolds directly from the merged geometry.

use std::collections::HashSet;

use generational_arena::Arena;
use nalgebra::{Point3, Vector3};

use crate::collision::capsule::Capsule;
use crate::collision::contact::ContactManifold;
use crate::collision::mesh::capsule_patch::capsule_patch_manifold;
use crate::collision::mesh::obb_patch::obb_patch_manifold;
use crate::collision::mesh::seam_filter::filter_patch;
use crate::collision::mesh::sphere_patch::sphere_patch_manifold;
use crate::collision::obb::Obb;
use crate::collision::AABB;
use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderShape};
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::{PairHeader, PairManifold};
use crate::physics::static_geometry::StaticGeometry;

/// Default cosine threshold for seam filter coplanar merging.
const SEAM_FILTER_COPLANAR_DOT: f32 = 0.98;

/// Generate contacts between all non-static colliders and static geometry.
///
/// Returns one `PairManifold` per collider that has contacts (or speculative contacts)
/// with static geometry. Each manifold carries the collision library's `ContactManifold`
/// with `FeatureId` per point, wrapped with body/collider/material metadata.
pub fn generate_static_contacts(
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
    dt: f32,
    ccd_threshold: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
    sleeping: Option<&HashSet<RigidBodyHandle>>,
) -> Vec<PairManifold> {
    let mut contacts = Vec::new();

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

            let manifold = match collider.shape() {
                ColliderShape::Sphere { radius } => {
                    sphere_vs_static(center, *radius, static_geometry, contact_margin)
                }
                ColliderShape::Box { half_extents } => box_vs_static(
                    center,
                    rotation,
                    *half_extents,
                    static_geometry,
                    contact_margin,
                ),
                ColliderShape::Capsule {
                    half_height,
                    radius,
                } => capsule_vs_static(
                    center,
                    rotation,
                    *half_height,
                    *radius,
                    static_geometry,
                    contact_margin,
                ),
            };

            let manifold = if manifold.is_empty()
                && is_speculative_candidate(
                    speed,
                    dt,
                    collider.shape().bounding_radius(),
                    ccd_threshold,
                    contact_margin,
                    enable_speculative_contacts,
                    speculative_min_speed,
                    speculative_margin_multiplier,
                ) {
                let predicted_center = center + linear_velocity * dt;
                let mut m = speculative_static_manifold(
                    collider,
                    predicted_center,
                    body.rotation(),
                    static_geometry,
                    contact_margin,
                );
                make_speculative(&mut m, contact_margin);
                m
            } else {
                manifold
            };

            if !manifold.is_empty() {
                contacts.push(PairManifold {
                    header: PairHeader {
                        body_a: None,
                        body_b: body_handle,
                        collider_a: None,
                        collider_b: Some(*collider_handle),
                        restitution: collider.material().restitution,
                        friction: collider.material().friction,
                    },
                    manifold,
                });
            }
        }
    }

    contacts
}

/// Generate sphere-static contacts via the mesh pipeline.
fn sphere_vs_static(
    center: Point3<f32>,
    radius: f32,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
) -> ContactManifold {
    let query_radius = radius + contact_margin;
    let query = AABB::new(
        Point3::new(
            center.x - query_radius,
            center.y - query_radius,
            center.z - query_radius,
        ),
        Point3::new(
            center.x + query_radius,
            center.y + query_radius,
            center.z + query_radius,
        ),
    );
    let patch = static_geometry.query_region(&query);
    let filtered = filter_patch(&patch, SEAM_FILTER_COPLANAR_DOT);
    sphere_patch_manifold(center, radius, &filtered, contact_margin)
}

/// Generate box-static contacts via the mesh pipeline.
fn box_vs_static(
    center: Point3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    half_extents: Vector3<f32>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
) -> ContactManifold {
    let obb = Obb::new(center, rotation, half_extents);
    let (aabb_min, aabb_max) = obb.enclosing_aabb();
    let margin_vec = Vector3::new(contact_margin, contact_margin, contact_margin);
    let query = AABB::new(aabb_min - margin_vec, aabb_max + margin_vec);

    let patch = static_geometry.query_region(&query);
    let filtered = filter_patch(&patch, SEAM_FILTER_COPLANAR_DOT);
    obb_patch_manifold(&obb, &filtered, contact_margin)
}

/// Generate capsule-static contacts via the mesh pipeline.
fn capsule_vs_static(
    center: Point3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    half_height: f32,
    radius: f32,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
) -> ContactManifold {
    let capsule = Capsule::new(center, rotation, half_height, radius);
    let (aabb_min, aabb_max) = capsule.enclosing_aabb();
    let margin_vec = Vector3::new(contact_margin, contact_margin, contact_margin);
    let query = AABB::new(aabb_min - margin_vec, aabb_max + margin_vec);

    let patch = static_geometry.query_region(&query);
    let filtered = filter_patch(&patch, SEAM_FILTER_COPLANAR_DOT);
    let (seg_a, seg_b) = capsule.segment_endpoints();
    capsule_patch_manifold(seg_a, seg_b, radius, &filtered, contact_margin)
}

/// Generate a static contact manifold at a predicted pose for any shape.
fn speculative_static_manifold(
    collider: &Collider,
    predicted_center: Point3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    static_geometry: &dyn StaticGeometry,
    contact_margin: f32,
) -> ContactManifold {
    match collider.shape() {
        ColliderShape::Sphere { radius } => {
            sphere_vs_static(predicted_center, *radius, static_geometry, contact_margin)
        }
        ColliderShape::Box { half_extents } => box_vs_static(
            predicted_center,
            rotation,
            *half_extents,
            static_geometry,
            contact_margin,
        ),
        ColliderShape::Capsule {
            half_height,
            radius,
        } => capsule_vs_static(
            predicted_center,
            rotation,
            *half_height,
            *radius,
            static_geometry,
            contact_margin,
        ),
    }
}

/// Convert a manifold to speculative: depth=0, raw_depth=-margin.
fn make_speculative(manifold: &mut ContactManifold, contact_margin: f32) {
    for cp in &mut manifold.points {
        cp.depth = 0.0;
        cp.raw_depth = -contact_margin;
    }
}

/// Gate speculative contacts by speed and CCD travel window.
fn is_speculative_candidate(
    speed: f32,
    dt: f32,
    radius: f32,
    ccd_threshold: f32,
    contact_margin: f32,
    enable_speculative_contacts: bool,
    speculative_min_speed: f32,
    speculative_margin_multiplier: f32,
) -> bool {
    if !enable_speculative_contacts {
        return false;
    }
    if dt <= 0.0 || speed < speculative_min_speed {
        return false;
    }
    let travel = speed * dt;
    let margin_gate = contact_margin * speculative_margin_multiplier;
    travel > margin_gate && travel <= radius * ccd_threshold
}
