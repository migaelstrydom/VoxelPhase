//! Moving a set of bodies along a straight line until they first touch
//! something, without simulating anything.
//!
//! ```text
//!   movers' colliders ──translate along `travel`──▶ first contact
//!                              │                      ▲
//!                              ├── other bodies ──────┤ (GJK raycast per pair)
//!                              └── static geometry ───┘ (front faces only)
//! ```
//!
//! The movers keep their orientation and travel as one rigid group, so a
//! structure of many bodies is swept as the structure: it stops where its
//! first part touches, and every part moves by the same distance. Nothing in
//! the world is changed; the caller decides what to do with the answer.
//!
//! This is a placement query, not a CCD pass. It asks every collider in the
//! world rather than a broadphase's candidates, which is the right trade for
//! something run a few hundred times while a level loads and never per frame.

use nalgebra::{Point3, Vector3};

use crate::collision::continuous::{gjk_raycast, GjkRaycastHit};
use crate::collision::shape_view::ShapeView;
use crate::collision::AABB;

use super::collider::ColliderShape;
use super::handle::RigidBodyHandle;
use super::static_geometry::StaticGeometry;
use super::world::PhysicsWorld;

/// Margin added to every bound the sweep filters with, so a pair that touches
/// exactly is never filtered out by rounding.
const BOUNDS_MARGIN: f32 = 0.01;

/// What a sweep ran into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepObstacle {
    /// The world's static geometry: terrain.
    Static,
    /// Another body.
    Body(RigidBodyHandle),
}

/// Where a sweep first touched something.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweepHit {
    /// Fraction of the travel covered before the touch, in `[0, 1]`. Zero
    /// means the movers already touch or overlap the obstacle where they are.
    pub fraction: f32,
    /// How far the movers travel before the touch.
    pub distance: f32,
    /// Approximate contact point on the obstacle's surface.
    pub point: Point3<f32>,
    /// Contact normal, pointing from the obstacle toward the movers.
    pub normal: Vector3<f32>,
    /// What was touched.
    pub obstacle: SweepObstacle,
}

/// One mover collider, as the sweep sees it at its starting pose.
struct MoverShape {
    center: Point3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    shape: ColliderShape,
}

impl MoverShape {
    fn view(&self) -> ShapeView<'_> {
        ShapeView {
            center: self.center,
            rotation: self.rotation,
            shape: &self.shape,
        }
    }

    /// Bounds of the whole straight path the collider takes.
    fn swept_bounds(&self, travel: Vector3<f32>) -> AABB {
        let here = self.view().query_aabb(BOUNDS_MARGIN);
        let there = AABB::new(here.min + travel, here.max + travel);
        here.merged(&there)
    }
}

/// A straight-line sweep of some bodies against the rest of the world.
///
/// Built over a world and, optionally, its static geometry; without the
/// latter only bodies are obstacles.
pub struct BodySweep<'a> {
    world: &'a PhysicsWorld,
    static_geometry: Option<&'a dyn StaticGeometry>,
}

impl<'a> BodySweep<'a> {
    pub fn new(world: &'a PhysicsWorld) -> Self {
        Self {
            world,
            static_geometry: None,
        }
    }

    /// Include `geometry` among the obstacles.
    pub fn against_static(mut self, geometry: &'a dyn StaticGeometry) -> Self {
        self.static_geometry = Some(geometry);
        self
    }

    /// The first thing `movers` touch when translated together by `travel`,
    /// or `None` if they travel the whole way without touching anything.
    ///
    /// Movers are never obstacles to each other.
    pub fn cast(&self, movers: &[RigidBodyHandle], travel: Vector3<f32>) -> Option<SweepHit> {
        let shapes = self.mover_shapes(movers);
        let length = travel.magnitude();

        let mut earliest: Option<SweepHit> = None;
        let mut keep = |hit: GjkRaycastHit, obstacle: SweepObstacle| {
            if earliest.is_none_or(|e| hit.t < e.fraction) {
                earliest = Some(SweepHit {
                    fraction: hit.t,
                    distance: hit.t * length,
                    point: hit.point,
                    normal: hit.normal,
                    obstacle,
                });
            }
        };

        for mover in &shapes {
            let bounds = mover.swept_bounds(travel);
            for (obstacle, view) in self.body_obstacles(movers, &bounds) {
                if let Some(hit) = gjk_raycast(&mover.view(), &view, travel, Vector3::zeros()) {
                    keep(hit, SweepObstacle::Body(obstacle));
                }
            }
            if let Some(geometry) = self.static_geometry {
                for triangle in geometry.query_region_triangles(&bounds) {
                    // Only a face the mover approaches from in front can stop
                    // it: one met from behind is the far side of a surface the
                    // mover is already inside, and stopping there would hide
                    // the burial rather than report it.
                    if triangle.normal_unnormalized().dot(&travel) >= 0.0 {
                        continue;
                    }
                    if let Some(hit) =
                        gjk_raycast(&mover.view(), &triangle, travel, Vector3::zeros())
                    {
                        keep(hit, SweepObstacle::Static);
                    }
                }
            }
        }
        earliest
    }

    fn mover_shapes(&self, movers: &[RigidBodyHandle]) -> Vec<MoverShape> {
        let mut shapes = Vec::new();
        for &handle in movers {
            let Some(body) = self.world.body(handle) else {
                continue;
            };
            for collider_handle in body.colliders() {
                let Some(collider) = self.world.collider(*collider_handle) else {
                    continue;
                };
                let transform = collider.world_transform(body.position(), body.rotation());
                shapes.push(MoverShape {
                    center: Point3::from(transform.translation.vector),
                    rotation: transform.rotation,
                    shape: collider.shape().clone(),
                });
            }
        }
        shapes
    }

    /// Every collider of every non-mover body whose bounds meet `bounds`.
    fn body_obstacles<'s>(
        &'s self,
        movers: &'s [RigidBodyHandle],
        bounds: &'s AABB,
    ) -> impl Iterator<Item = (RigidBodyHandle, ShapeView<'s>)> + 's {
        self.world
            .bodies()
            .iter()
            .map(|(index, body)| (RigidBodyHandle(index), body))
            .filter(|(handle, _)| !movers.contains(handle))
            .flat_map(move |(handle, body)| {
                body.colliders().iter().filter_map(move |collider_handle| {
                    let collider = self.world.collider(*collider_handle)?;
                    let transform = collider.world_transform(body.position(), body.rotation());
                    let view = ShapeView {
                        center: Point3::from(transform.translation.vector),
                        rotation: transform.rotation,
                        shape: collider.shape(),
                    };
                    view.query_aabb(BOUNDS_MARGIN)
                        .intersects(bounds)
                        .then_some((handle, view))
                })
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    fn world() -> PhysicsWorld {
        PhysicsWorld::new(PhysicsConfig::default())
    }

    fn cube(world: &mut PhysicsWorld, at: Point3<f32>, half: f32) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic().position(at));
        world.attach_collider(
            body,
            ColliderDesc::box_shape(Vector3::new(half, half, half)),
        );
        body
    }

    fn down(distance: f32) -> Vector3<f32> {
        Vector3::new(0.0, -distance, 0.0)
    }

    /// A half-metre cube three metres up lands on the ground after 2.5 m.
    #[test]
    fn a_cube_drops_onto_static_ground() {
        let mut world = world();
        let cube = cube(&mut world, Point3::new(0.0, 3.0, 0.0), 0.5);
        let ground = FlatQuadGeometry::new(10.0);

        let hit = BodySweep::new(&world)
            .against_static(&ground)
            .cast(&[cube], down(10.0))
            .expect("the cube is over the ground");

        assert_eq!(hit.obstacle, SweepObstacle::Static);
        assert!(
            (hit.distance - 2.5).abs() < 0.01,
            "dropped {}",
            hit.distance
        );
        assert!(hit.normal.y > 0.99, "normal {:?}", hit.normal);
    }

    /// A cube over another cube stops on its top face, not on the ground.
    #[test]
    fn a_cube_lands_on_the_body_below_it() {
        let mut world = world();
        let base = cube(&mut world, Point3::new(0.0, 0.5, 0.0), 0.5);
        let top = cube(&mut world, Point3::new(0.2, 4.0, 0.0), 0.25);
        let ground = FlatQuadGeometry::new(10.0);

        let hit = BodySweep::new(&world)
            .against_static(&ground)
            .cast(&[top], down(10.0))
            .expect("the base is below");

        assert_eq!(hit.obstacle, SweepObstacle::Body(base));
        // Top's underside 3.75 m up, the base's top 1.0 m up.
        assert!(
            (hit.distance - 2.75).abs() < 0.01,
            "dropped {}",
            hit.distance
        );
    }

    /// Two bodies swept together stop when the lower one lands, and never
    /// count each other as obstacles even though they touch.
    #[test]
    fn movers_travel_as_one_group() {
        let mut world = world();
        let lower = cube(&mut world, Point3::new(0.0, 2.5, 0.0), 0.5);
        let upper = cube(&mut world, Point3::new(0.0, 3.5, 0.0), 0.5);
        let ground = FlatQuadGeometry::new(10.0);

        let hit = BodySweep::new(&world)
            .against_static(&ground)
            .cast(&[lower, upper], down(10.0))
            .expect("the ground is below");

        assert_eq!(hit.obstacle, SweepObstacle::Static);
        assert!(
            (hit.distance - 2.0).abs() < 0.01,
            "dropped {}",
            hit.distance
        );
    }

    #[test]
    fn nothing_below_is_a_miss() {
        let mut world = world();
        let cube = cube(&mut world, Point3::new(0.0, 3.0, 0.0), 0.5);
        assert!(BodySweep::new(&world).cast(&[cube], down(10.0)).is_none());
    }

    /// A cube already resting on another reports a zero-length sweep.
    #[test]
    fn touching_at_the_start_is_a_zero_fraction() {
        let mut world = world();
        let base = cube(&mut world, Point3::new(0.0, 0.5, 0.0), 0.5);
        let top = cube(&mut world, Point3::new(0.0, 1.4, 0.0), 0.5);

        let hit = BodySweep::new(&world)
            .cast(&[top], down(10.0))
            .expect("they overlap");

        assert_eq!(hit.obstacle, SweepObstacle::Body(base));
        assert_eq!(hit.fraction, 0.0);
    }

    /// Ground the mover is already below is the far side of a surface it is
    /// inside, and is not where it lands.
    #[test]
    fn ground_met_from_behind_is_not_a_landing() {
        let mut world = world();
        let cube = cube(&mut world, Point3::new(0.0, -2.0, 0.0), 0.5);
        let ceiling = FlatQuadGeometry::new(10.0);

        let hit = BodySweep::new(&world)
            .against_static(&ceiling)
            .cast(&[cube], Vector3::new(0.0, 5.0, 0.0));
        assert!(hit.is_none(), "hit {hit:?}");
    }
}
