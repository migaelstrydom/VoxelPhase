//! Moving shapes along a straight line until they first touch something,
//! without simulating anything.
//!
//! ```text
//!   movers' colliders ─┐
//!   or one cast shape ─┴─translate along `travel`──▶ first contact
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
//! The bodies it tests are those whose bounds meet the sweep's, found by the
//! world's collider index (`collider_bounds.rs`) rather than by testing every
//! collider: a scene query may be cast every frame (the camera's, for one),
//! and a level holds hundreds of bodies.
//!
//! A sphere that arrives on an edge or a corner touches it with a rounded
//! normal, leaning between the faces that meet there. The hit also names the
//! face it arrived on (`face_normal`): of the faces touched first, the one
//! most squarely in the sphere's way.

use nalgebra::{Point3, Vector3};
use smallvec::SmallVec;

use crate::collision::continuous::{
    gjk_raycast, swept_sphere_sphere, swept_sphere_triangle, GjkRaycastHit,
};
use crate::collision::shape_view::ShapeView;
use crate::collision::{SurfaceId, Triangle, AABB};

use super::collider::ColliderShape;
use super::collider_bounds::BOUNDS_MARGIN;
use super::handle::{ColliderHandle, RigidBodyHandle};
use super::static_geometry::StaticGeometry;
use super::world::PhysicsWorld;

/// How much further than the first touch, in metres, another touch still
/// counts as the same one: a sphere arriving on an edge meets both faces
/// beside it at once, give or take rounding.
const SAME_TOUCH: f32 = 1.0e-4;

/// What a sweep ran into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepObstacle {
    /// The world's static geometry: terrain.
    Static,
    /// Another body.
    Body(RigidBodyHandle),
}

/// The surface a sweep touched, for what it is made of.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweptSurface {
    /// A static triangle, by the surface the geometry tags it with.
    Static(SurfaceId),
    /// A body's collider.
    Collider(ColliderHandle),
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
    /// Contact normal, pointing from the obstacle toward the movers. On an
    /// edge or a corner it is rounded, leaning between the faces there.
    pub normal: Vector3<f32>,
    /// The normal of the face the movers arrived on: `normal` on a face, and
    /// on an edge or a corner the face beside it most squarely in the
    /// travel's way.
    pub face_normal: Vector3<f32>,
    /// What was touched.
    pub obstacle: SweepObstacle,
    /// The surface touched.
    pub surface: SweptSurface,
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
}

/// Bounds of the whole straight path `view` takes along `travel`.
fn swept_bounds(view: &ShapeView, travel: Vector3<f32>) -> AABB {
    let here = view.query_aabb(BOUNDS_MARGIN);
    let there = AABB::new(here.min + travel, here.max + travel);
    here.merged(&there)
}

/// A touch, and the face it was made on.
struct FaceHit {
    hit: GjkRaycastHit,
    /// Outward normal of the face touched; the touch's own normal where the
    /// obstacle has no faces to name (a sphere, a capsule's round).
    face_normal: Vector3<f32>,
}

impl FaceHit {
    /// A touch whose own normal is the face's.
    fn on_face(hit: GjkRaycastHit) -> Self {
        let face_normal = hit.normal;
        Self { hit, face_normal }
    }
}

/// Where `view`, translated by `travel`, first touches `triangle`.
///
/// A sphere is cast in closed form: against the triangle's face, edges and
/// corners. GJK's witness for a sphere arriving on an edge shared by two
/// coplanar triangles is unreliable — its normal can lean tens of degrees off
/// the face — and a caller resting something on the hit reads the plane that
/// normal names.
fn cast_against_triangle(
    view: &ShapeView,
    triangle: &Triangle,
    travel: Vector3<f32>,
) -> Option<FaceHit> {
    match view.shape {
        ColliderShape::Sphere { radius } => {
            swept_sphere_triangle(view.center, view.center + travel, *radius, triangle).map(|hit| {
                FaceHit {
                    hit: GjkRaycastHit {
                        t: hit.t,
                        normal: hit.normal,
                        point: hit.point,
                    },
                    face_normal: triangle.normal(),
                }
            })
        }
        _ => gjk_raycast(view, triangle, travel, Vector3::zeros()).map(FaceHit::on_face),
    }
}

/// Where `view`, translated by `travel`, first touches the collider
/// `obstacle`.
///
/// A sphere is cast in closed form against a sphere, and against the faces
/// of a box or a hull (their edges and corners with them), for the reason it
/// is against a triangle: GJK's normal for a sphere arriving on a body's edge
/// leans off the true one, by over ten degrees on a box's top edge, and can
/// point into the body.
fn cast_against_collider(
    view: &ShapeView,
    obstacle: &ShapeView,
    travel: Vector3<f32>,
) -> Option<FaceHit> {
    let ColliderShape::Sphere { radius } = view.shape else {
        return gjk_raycast(view, obstacle, travel, Vector3::zeros()).map(FaceHit::on_face);
    };
    let start = view.center;
    let end = start + travel;
    match obstacle.shape {
        ColliderShape::Sphere { radius: other } => {
            let t = swept_sphere_sphere(
                start,
                end,
                *radius,
                obstacle.center,
                obstacle.center,
                *other,
            )?;
            let normal = (start + travel * t - obstacle.center)
                .try_normalize(1.0e-9)
                .unwrap_or_else(|| -travel.normalize());
            Some(FaceHit::on_face(GjkRaycastHit {
                t,
                normal,
                point: obstacle.center + normal * *other,
            }))
        }
        ColliderShape::Box { half_extents } => {
            let axes = [Vector3::x(), Vector3::y(), Vector3::z()].map(|a| obstacle.rotation * a);
            let faces = (0..3).flat_map(|i| {
                let (u, v) = (axes[(i + 1) % 3], axes[(i + 2) % 3]);
                let (hu, hv) = (half_extents[(i + 1) % 3], half_extents[(i + 2) % 3]);
                [1.0f32, -1.0].map(|sign| {
                    let normal = axes[i] * sign;
                    let centre = obstacle.center + normal * half_extents[i];
                    // Counter-clockwise seen from outside: u × v is the axis.
                    let (u, v) = (u * hu * sign, v * hv);
                    (
                        normal,
                        SmallVec::from_buf([
                            centre - u - v,
                            centre + u - v,
                            centre + u + v,
                            centre - u + v,
                        ]),
                    )
                })
            });
            sphere_against_faces(start, travel, *radius, faces)
        }
        ColliderShape::ConvexHull { hull } => {
            let faces = hull.faces.iter().map(|face| {
                let corners: SmallVec<[Point3<f32>; 4]> = face
                    .vertex_indices
                    .iter()
                    .map(|&i| obstacle.center + obstacle.rotation * hull.vertices[i as usize])
                    .collect();
                (obstacle.rotation * face.normal, corners)
            });
            sphere_against_faces(start, travel, *radius, faces)
        }
        ColliderShape::Capsule { .. } => {
            gjk_raycast(view, obstacle, travel, Vector3::zeros()).map(FaceHit::on_face)
        }
    }
}

/// A touch at the start, for a sphere whose centre is inside a box or a hull:
/// on the face nearest the centre.
fn sphere_inside(view: &ShapeView, obstacle: &ShapeView) -> Option<FaceHit> {
    let ColliderShape::Sphere { .. } = view.shape else {
        return None;
    };
    let local = obstacle.rotation.inverse() * (view.center - obstacle.center);
    let (depth, local_normal) = match obstacle.shape {
        ColliderShape::Box { half_extents } => (0..3)
            .map(|i| {
                let sign = if local[i] >= 0.0 { 1.0 } else { -1.0 };
                (half_extents[i] - local[i].abs(), Vector3::ith(i, sign))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))?,
        ColliderShape::ConvexHull { hull } => hull
            .faces
            .iter()
            .map(|face| {
                let on_face = hull.vertices[face.vertex_indices[0] as usize];
                ((on_face - local).dot(&face.normal), face.normal)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))?,
        _ => return None,
    };
    if depth <= 0.0 {
        return None;
    }
    let normal = obstacle.rotation * local_normal;
    Some(FaceHit::on_face(GjkRaycastHit {
        t: 0.0,
        normal,
        point: view.center + normal * depth,
    }))
}

/// Where a sphere of `radius` cast from `start` along `travel` first touches
/// a convex body given as its faces (outward normal, corners counter-clockwise
/// from outside). Faces turned away from the travel cannot stop it.
fn sphere_against_faces(
    start: Point3<f32>,
    travel: Vector3<f32>,
    radius: f32,
    faces: impl Iterator<Item = (Vector3<f32>, SmallVec<[Point3<f32>; 4]>)>,
) -> Option<FaceHit> {
    let end = start + travel;
    let mut first = FirstTouch::new(travel);
    faces
        .filter(|(normal, _)| normal.dot(&travel) < 0.0)
        .flat_map(|(normal, corners)| {
            (1..corners.len() - 1)
                .map(move |i| {
                    (
                        normal,
                        Triangle::new(corners[0], corners[i], corners[i + 1]),
                    )
                })
                .collect::<SmallVec<[(Vector3<f32>, Triangle); 4]>>()
        })
        .filter_map(|(normal, triangle)| {
            swept_sphere_triangle(start, end, radius, &triangle).map(|hit| FaceHit {
                hit: GjkRaycastHit {
                    t: hit.t,
                    normal: hit.normal,
                    point: hit.point,
                },
                face_normal: normal,
            })
        })
        .for_each(|hit| first.offer(hit));
    first.into_hit()
}

/// The first of several touches along one travel, and the face it was made
/// on.
///
/// The touch itself is the earliest offered, the first offered of equals.
/// The face is chosen among every touch within [`SAME_TOUCH`] of it: the one
/// facing the travel most squarely. A sphere arriving on the edge between a
/// plank's top and its side meets both faces' triangles at the same
/// distance, and it is standing on the top.
struct FirstTouch {
    /// The travel's direction.
    heading: Vector3<f32>,
    /// The travel's length, to turn [`SAME_TOUCH`] into a fraction.
    length: f32,
    best: Option<FaceHit>,
}

impl FirstTouch {
    fn new(travel: Vector3<f32>) -> Self {
        let length = travel.magnitude();
        Self {
            heading: travel / length.max(1.0e-9),
            length,
            best: None,
        }
    }

    fn offer(&mut self, candidate: FaceHit) {
        let same = SAME_TOUCH / self.length.max(1.0e-9);
        let Some(best) = &mut self.best else {
            self.best = Some(candidate);
            return;
        };
        let squarer =
            |a: &Vector3<f32>, b: &Vector3<f32>| -a.dot(&self.heading) > -b.dot(&self.heading);
        if candidate.hit.t < best.hit.t - same {
            *best = candidate;
        } else if candidate.hit.t <= best.hit.t + same {
            let face_normal = if squarer(&candidate.face_normal, &best.face_normal) {
                candidate.face_normal
            } else {
                best.face_normal
            };
            if candidate.hit.t < best.hit.t {
                best.hit = candidate.hit;
            }
            best.face_normal = face_normal;
        }
    }

    fn into_hit(self) -> Option<FaceHit> {
        self.best
    }
}

/// A straight-line sweep of some bodies, or of a shape, against the rest of
/// the world.
///
/// Built over a world and, optionally, its static geometry; without the
/// latter only bodies are obstacles.
pub struct BodySweep<'a> {
    world: &'a PhysicsWorld,
    static_geometry: Option<&'a dyn StaticGeometry>,
    /// Whether a sphere that starts inside a box or a hull touches it at
    /// once. Otherwise it passes out through the far side unseen, as a
    /// sweep from inside a surface does.
    solid_bodies: bool,
    /// Whether only static bodies are obstacles, so anything that moves is
    /// passed through.
    scenery_only: bool,
}

impl<'a> BodySweep<'a> {
    pub fn new(world: &'a PhysicsWorld) -> Self {
        Self {
            world,
            static_geometry: None,
            solid_bodies: false,
            scenery_only: false,
        }
    }

    /// Only static bodies are obstacles; dynamic and kinematic ones are
    /// passed through.
    pub fn scenery_only(mut self) -> Self {
        self.scenery_only = true;
        self
    }

    /// A sphere starting inside a box or a hull touches it where it starts:
    /// for a cast asking what is there, not where a mover would stop.
    pub fn solid_bodies(mut self) -> Self {
        self.solid_bodies = true;
        self
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
        let mut earliest: Option<SweepHit> = None;
        for mover in &self.mover_shapes(movers) {
            if let Some(hit) = self.cast_shape(&mover.view(), travel, movers) {
                if earliest.is_none_or(|e| hit.fraction < e.fraction) {
                    earliest = Some(hit);
                }
            }
        }
        earliest
    }

    /// The first thing `view` touches when translated by `travel`, ignoring
    /// the bodies in `exclude`, or `None` if it travels the whole way.
    pub fn cast_shape(
        &self,
        view: &ShapeView,
        travel: Vector3<f32>,
        exclude: &[RigidBodyHandle],
    ) -> Option<SweepHit> {
        let length = travel.magnitude();
        let mut first = FirstTouch::new(travel);
        let mut earliest: Option<(SweepObstacle, SweptSurface)> = None;
        let mut keep = |hit: FaceHit, obstacle: SweepObstacle, surface: SweptSurface| {
            let t = hit.hit.t;
            if first.best.as_ref().is_none_or(|e| t < e.hit.t) {
                earliest = Some((obstacle, surface));
            }
            first.offer(hit);
        };

        let bounds = swept_bounds(view, travel);
        for (obstacle, collider) in self.body_obstacles(exclude, &bounds) {
            let Some(obstacle_view) = self.collider_view(obstacle, collider) else {
                continue;
            };
            let inside = if self.solid_bodies {
                sphere_inside(view, &obstacle_view)
            } else {
                None
            };
            if let Some(hit) =
                inside.or_else(|| cast_against_collider(view, &obstacle_view, travel))
            {
                keep(
                    hit,
                    SweepObstacle::Body(obstacle),
                    SweptSurface::Collider(collider),
                );
            }
        }
        if let Some(geometry) = self.static_geometry {
            for (triangle, surface) in geometry.query_region_surfaces(&bounds) {
                let triangle = &triangle;
                // Only a face the mover approaches from in front can stop
                // it: one met from behind is the far side of a surface the
                // mover is already inside, and stopping there would hide
                // the burial rather than report it.
                if triangle.normal_unnormalized().dot(&travel) >= 0.0 {
                    continue;
                }
                if let Some(hit) = cast_against_triangle(view, triangle, travel) {
                    keep(hit, SweepObstacle::Static, SweptSurface::Static(surface));
                }
            }
        }
        let (obstacle, surface) = earliest?;
        let FaceHit { hit, face_normal } = first.into_hit()?;
        Some(SweepHit {
            fraction: hit.t,
            distance: hit.t * length,
            point: hit.point,
            normal: hit.normal,
            face_normal,
            obstacle,
            surface,
        })
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

    /// Every collider of every body not in `exclude` whose bounds meet
    /// `bounds`, in arena order.
    fn body_obstacles(
        &self,
        exclude: &[RigidBodyHandle],
        bounds: &AABB,
    ) -> Vec<(RigidBodyHandle, ColliderHandle)> {
        let mut near = Vec::new();
        self.world.colliders_meeting(bounds, &mut near);
        let world = self.world;
        near.retain(|(body, _)| {
            !exclude.contains(body)
                && (!self.scenery_only || world.body(*body).is_some_and(|b| b.is_static()))
        });
        near
    }

    /// Where `collider` of `body` stands now.
    fn collider_view(
        &self,
        body: RigidBodyHandle,
        collider: ColliderHandle,
    ) -> Option<ShapeView<'a>> {
        let world = self.world;
        let body = world.body(body)?;
        let collider = world.collider(collider)?;
        let transform = collider.world_transform(body.position(), body.rotation());
        Some(ShapeView {
            center: Point3::from(transform.translation.vector),
            rotation: transform.rotation,
            shape: collider.shape(),
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

    /// A sphere cast down past a box's top edge meets the edge, and the
    /// normal there points from the edge to the sphere's centre, on a box
    /// turned about the vertical as on one square to the world.
    #[test]
    fn a_sphere_cast_onto_a_box_edge_has_the_edge_normal() {
        let radius = 0.1;
        for turn in [0.0f32, 0.7] {
            let mut world = world();
            let rotation = nalgebra::UnitQuaternion::from_euler_angles(0.0, turn, 0.0);
            let base = world.create_body(RigidBodyDesc::dynamic().rotation(rotation));
            world.attach_collider(base, ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 0.5)));
            let sphere = ColliderShape::Sphere { radius };
            let outward = rotation * Vector3::x();
            let view = ShapeView {
                center: Point3::from(outward * 0.55) + Vector3::y() * 2.0,
                rotation: nalgebra::UnitQuaternion::identity(),
                shape: &sphere,
            };

            let hit = BodySweep::new(&world)
                .cast_shape(&view, down(3.0), &[])
                .expect("the sphere passes over the edge");

            let expected = (outward * 0.5 + Vector3::y() * 0.75f32.sqrt()).normalize();
            assert!(
                hit.normal.angle(&expected).to_degrees() < 0.1,
                "turned {turn}: normal {:?}, expected {expected:?}",
                hit.normal
            );
            assert!(
                (hit.distance - (2.0 - 0.5 - radius * 0.75f32.sqrt())).abs() < 1.0e-4,
                "turned {turn}: fell {}",
                hit.distance
            );
        }
    }

    /// A sphere cast onto a box's face, across the diagonal that splits it
    /// into triangles, meets it square on.
    #[test]
    fn a_sphere_cast_onto_a_box_face_has_the_face_normal() {
        let mut world = world();
        let base = cube(&mut world, Point3::new(0.0, 0.0, 0.0), 0.5);
        let sphere = ColliderShape::Sphere { radius: 0.1 };
        for x in [0.0, 0.2, -0.3] {
            let view = ShapeView {
                center: Point3::new(x, 2.0, x),
                rotation: nalgebra::UnitQuaternion::identity(),
                shape: &sphere,
            };
            let hit = BodySweep::new(&world)
                .cast_shape(&view, down(3.0), &[])
                .expect("the sphere is over the box");
            assert_eq!(hit.obstacle, SweepObstacle::Body(base));
            assert!(hit.normal.y > 0.99999, "at {x}: normal {:?}", hit.normal);
            assert!(
                (hit.distance - 1.4).abs() < 1.0e-4,
                "at {x}: fell {}",
                hit.distance
            );
        }
    }
}
