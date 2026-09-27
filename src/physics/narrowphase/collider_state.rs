//! World-space collider snapshots feeding broadphase and pair dispatch.

use generational_arena::Arena;
use nalgebra::{Point3, UnitQuaternion, Vector3};
use rustc_hash::FxHashSet;

use crate::collision::shape_view::ShapeView;
use crate::collision::AABB;
use crate::physics::body::RigidBody;
use crate::physics::collider::{Collider, ColliderMaterial, ColliderShape};
use crate::physics::handle::{ColliderHandle, RigidBodyHandle};

/// Shape-agnostic snapshot of a collider's world-space state for pair dispatch.
pub(super) struct ColliderState {
    pub body_handle: RigidBodyHandle,
    pub collider_handle: ColliderHandle,
    pub center: Point3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub shape: ColliderShape,
    pub velocity: Vector3<f32>,
    pub material: ColliderMaterial,
    pub is_sleeping: bool,
    /// Whether the owning body is static. Static bodies take part in contacts
    /// but never initiate them.
    pub is_static: bool,
}

impl ColliderState {
    /// Whether this collider can move during the coming step.
    ///
    /// Static bodies never move; sleeping bodies will not until something wakes
    /// them, and are not integrated meanwhile. A pair in which neither side can
    /// move cannot produce a contact that did not already exist, so the
    /// broadphase skips it — the same reasoning by which the static-geometry
    /// pass skips sleeping bodies outright.
    pub fn is_mobile(&self) -> bool {
        !self.is_static && !self.is_sleeping
    }

    /// World-space AABB of this collider, expanded by `margin`.
    ///
    /// Delegates to [`ShapeView::query_aabb`], which bounds boxes and capsules
    /// by their actual oriented extents rather than by `bounding_radius`. The
    /// distinction matters for elongated shapes: a 2.0 x 0.12 x 0.12 slab has a
    /// bounding radius of ~1.0, so a radius-derived box is some two orders of
    /// magnitude larger in volume than the shape it stands for, and the
    /// broadphase pays for the difference in false pairs.
    pub fn bounds(&self, margin: f32) -> AABB {
        self.view().query_aabb(margin)
    }

    /// World-space AABB enclosing this collider both where it is and after
    /// moving by `travel`, expanded by `margin`.
    ///
    /// Encloses the whole straight path between the two, since an AABB swept
    /// along a line is contained in the box around its two ends.
    pub fn swept_bounds(&self, margin: f32, travel: Vector3<f32>) -> AABB {
        let here = self.bounds(margin);
        let there = AABB::new(here.min + travel, here.max + travel);
        here.merged(&there)
    }

    /// Borrow this state as a dispatch-facing shape view.
    pub fn view(&self) -> ShapeView<'_> {
        ShapeView {
            center: self.center,
            rotation: self.rotation,
            shape: &self.shape,
        }
    }

    /// This state's shape view after translating by `travel`, orientation held.
    pub fn view_moved(&self, travel: Vector3<f32>) -> ShapeView<'_> {
        ShapeView {
            center: self.center + travel,
            ..self.view()
        }
    }
}

/// Snapshot every collider eligible for pair contacts.
///
/// Static bodies are included. They are distinct from the static *geometry*
/// that `StaticGeometry` supplies — a static body is an ordinary collider that
/// happens to have infinite mass, and dynamic bodies must be able to rest on
/// one. Being immobile, they can only ever be the passive side of a pair.
pub(super) fn collect_collider_states_into(
    states: &mut Vec<ColliderState>,
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    sleeping: Option<&FxHashSet<RigidBodyHandle>>,
) {
    for (idx, body) in bodies.iter() {
        let body_handle = RigidBodyHandle(idx);
        let is_sleeping = sleeping.map(|s| s.contains(&body_handle)).unwrap_or(false);

        for collider_handle in body.colliders() {
            let Some(collider) = colliders.get(collider_handle.0) else {
                continue;
            };
            let world_tf = collider.world_transform(body.position(), body.rotation());
            states.push(ColliderState {
                body_handle,
                collider_handle: *collider_handle,
                center: Point3::from(world_tf.translation.vector),
                rotation: world_tf.rotation,
                shape: collider.shape().clone(),
                velocity: body.linear_velocity(),
                material: *collider.material(),
                is_sleeping,
                is_static: body.is_static(),
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn slab() -> ColliderState {
        ColliderState {
            body_handle: RigidBodyHandle(generational_arena::Index::from_raw_parts(0, 0)),
            collider_handle: ColliderHandle(generational_arena::Index::from_raw_parts(0, 0)),
            center: Point3::origin(),
            rotation: UnitQuaternion::identity(),
            shape: ColliderShape::Box {
                half_extents: Vector3::new(1.0, 0.06, 0.06),
            },
            velocity: Vector3::zeros(),
            material: ColliderMaterial::default(),
            is_sleeping: false,
            is_static: false,
        }
    }

    #[test]
    fn static_and_sleeping_colliders_are_immobile() {
        let mut state = slab();
        assert!(state.is_mobile());

        state.is_static = true;
        assert!(!state.is_mobile());

        state.is_static = false;
        state.is_sleeping = true;
        assert!(!state.is_mobile());
    }

    /// The pendulum-arm case: an axis-aligned slab must not be bounded by its
    /// bounding sphere, or the broadphase sees a cube ~140x its true volume.
    #[test]
    fn slab_bounds_are_oriented_not_spherical() {
        let state = slab();
        let bounds = state.bounds(0.0);
        assert!((bounds.max.x - 1.0).abs() < 1e-5);
        assert!((bounds.max.y - 0.06).abs() < 1e-5);
        assert!((bounds.max.z - 0.06).abs() < 1e-5);
        assert!(bounds.max.y < state.shape.bounding_radius());
    }

    #[test]
    fn bounds_include_margin() {
        let bounds = slab().bounds(0.02);
        assert!((bounds.max.y - 0.08).abs() < 1e-5);
        assert!((bounds.min.y + 0.08).abs() < 1e-5);
    }

    /// Rotating a slab 90 degrees about Z swaps which axis is long.
    #[test]
    fn bounds_follow_rotation() {
        let mut state = slab();
        state.rotation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2);
        let bounds = state.bounds(0.0);
        assert!((bounds.max.x - 0.06).abs() < 1e-5);
        assert!((bounds.max.y - 1.0).abs() < 1e-5);
    }
}
