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
}

impl ColliderState {
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

    /// Borrow this state as a dispatch-facing shape view.
    pub fn view(&self) -> ShapeView<'_> {
        ShapeView {
            center: self.center,
            rotation: self.rotation,
            shape: &self.shape,
        }
    }
}

/// Snapshot every collider eligible for dynamic pair contacts.
pub(super) fn collect_collider_states_into(
    states: &mut Vec<ColliderState>,
    bodies: &Arena<RigidBody>,
    colliders: &Arena<Collider>,
    sleeping: Option<&FxHashSet<RigidBodyHandle>>,
) {
    for (idx, body) in bodies.iter() {
        if body.is_static() {
            continue;
        }
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
        }
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
