use nalgebra::Point3;

use crate::physics::{CastObstacles, PhysicsWorld, SphereCast, StaticGeometry};

/// Keeps the camera on the near side of the scenery between it and what it
/// looks at.
///
/// ```text
///   pivot ──── sphere cast ────► camera
///                  │ hits terrain or a static body
///                  ▼
///   camera pulled in to where the sphere touched
/// ```
///
/// Only the scenery counts: terrain and static bodies. Crates, creatures and
/// rubble passing between are seen through rather than chased, which would
/// make the camera lurch every time something flew past.
///
/// The pull-in is immediate, so the lens never enters the scenery. Easing
/// back out is left to the camera's own follow smoothing, which starts each
/// frame from where the clearance left it.
#[derive(Debug, Clone)]
pub struct CameraClearance {
    /// Radius of the sphere swept from the pivot to the camera, in metres.
    /// It must exceed the near plane's half-extent so the near plane stays
    /// out of the surface the sphere touches.
    pub radius: f32,
}

impl Default for CameraClearance {
    fn default() -> Self {
        Self { radius: 0.3 }
    }
}

impl CameraClearance {
    /// Where the camera may stand on the segment from `pivot` toward
    /// `position`: `position` itself if the way is clear, otherwise the
    /// centre of the swept sphere where it first touches the scenery.
    pub fn clear_position(
        &self,
        physics: &PhysicsWorld,
        scenery: &dyn StaticGeometry,
        pivot: Point3<f32>,
        position: Point3<f32>,
    ) -> Point3<f32> {
        let offset = position - pivot;
        let length = offset.magnitude();
        if length <= f32::EPSILON {
            return position;
        }
        let cast = SphereCast {
            origin: pivot,
            direction: offset / length,
            length,
            radius: self.radius,
            exclude: &[],
            solid_bodies: false,
            obstacles: CastObstacles::Scenery,
        };
        match physics.cast_sphere(scenery, &cast) {
            Some(hit) => cast.centre_at(hit.distance),
            None => position,
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;
    use crate::physics::bench_harness::geometry::{EmptyGeometry, WallAndFloorGeometry};
    use crate::physics::{ColliderDesc, RigidBodyDesc};

    const PIVOT: Point3<f32> = Point3::new(2.0, 1.0, 0.0);

    fn world_with_box(desc: RigidBodyDesc) -> PhysicsWorld {
        let mut world = PhysicsWorld::default();
        let body = world.create_body(desc.position(Point3::new(0.0, 1.0, 0.0)));
        world.attach_collider(body, ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 0.5)));
        world
    }

    #[test]
    fn a_clear_line_leaves_the_camera_where_it_is() {
        let world = PhysicsWorld::default();
        let wall = WallAndFloorGeometry::new(10.0, 4.0);
        let position = Point3::new(6.0, 1.5, 1.0);
        let cleared = CameraClearance::default().clear_position(&world, &wall, PIVOT, position);
        assert_eq!(cleared, position);
    }

    #[test]
    fn terrain_between_pulls_the_camera_in_front_of_it() {
        let world = PhysicsWorld::default();
        let wall = WallAndFloorGeometry::new(10.0, 4.0);
        let clearance = CameraClearance::default();
        let cleared = clearance.clear_position(&world, &wall, PIVOT, Point3::new(-3.0, 1.0, 0.0));
        assert!(
            (cleared.x - clearance.radius).abs() < 1.0e-3,
            "camera at {cleared:?}, expected x = {}",
            clearance.radius
        );
        assert!((cleared.y - 1.0).abs() < 1.0e-4 && cleared.z.abs() < 1.0e-4);
    }

    #[test]
    fn a_static_body_between_pulls_the_camera_in() {
        let world = world_with_box(RigidBodyDesc::static_body());
        let cleared = CameraClearance::default().clear_position(
            &world,
            &EmptyGeometry,
            PIVOT,
            Point3::new(-3.0, 1.0, 0.0),
        );
        assert!(
            cleared.x > 0.5,
            "camera at {cleared:?} is inside or behind the box"
        );
    }

    #[test]
    fn a_dynamic_body_between_is_seen_through() {
        let world = world_with_box(RigidBodyDesc::dynamic());
        let position = Point3::new(-3.0, 1.0, 0.0);
        let cleared =
            CameraClearance::default().clear_position(&world, &EmptyGeometry, PIVOT, position);
        assert_eq!(cleared, position);
    }
}
