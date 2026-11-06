//! The firm heart of a loose clump: what physics sees of it.
//!
//! A ball of leaves is mostly air. What stops when it lands is the dense
//! middle of each lobe, so the body is a solid piece inside each lobe, a
//! share of its size; the leaves round it are drawn (`skin`) and drag what passes
//! through them (`drag`), but nothing collides with them. A cluster of
//! spheres, not one ball: a single sphere has nothing to stop it rolling, and
//! the engine's friction has no rolling resistance, while a lumpy cluster
//! tumbles like a sack and stops. Its bulk, what the air and water see, is
//! its whole lobes, each core piece carrying the same piece at full size,
//! dragging as a porous bush rather than smooth balls: a
//! ball of leaves is mostly air, light for its size, so the world's air slows
//! it to a floaty fall, and a blast, which strikes it by its size, throws it
//! a few dozen metres rather than hundreds.
//!
//! A lobe's core follows its shape (`CorePiece`): a sphere when it is round,
//! a capsule along its length when it is tall (a column), two crossed
//! capsules when it is flat (a mound), so a column falls over and lies on
//! its side, and a mound sits flat rather than rolling on a rod.

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use smallvec::SmallVec;

use super::shape::{FoliageShape, Lobe};
use crate::collision_layers;
use crate::physics::{
    BulkShape, ColliderDesc, ColliderShape, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, Volume,
};

/// How a clump's core is built.
#[derive(Debug, Clone, Copy)]
pub struct LeafCore {
    /// Size of each lobe's core pieces as a share of the lobe's own. Large,
    /// so a resting clump's skin, which never bends, sinks only a little
    /// into what it rests on; not so large that the cluster rolls like a
    /// ball.
    pub core_fraction: f32,
    /// Mass per cubic metre of leaves, kg/m³: the clump's whole mass over the
    /// space its lobes fill. Light, so the player shoves it rather than
    /// stands on it.
    pub bulk_density: f32,
    /// Drag coefficient of the clump's lobes in air and water: a porous
    /// bush's, about twice a smooth ball's, since the flow is torn through
    /// the leaves rather than parted round a surface.
    pub drag_coefficient: f32,
    /// The share of a blast's loading the clump's lobes catch: leaves let the
    /// shock front through, and only the wind behind it pushes
    /// (`BulkShape::porous`).
    pub solidity: f32,
    /// Grip of the core on whatever it lands on. High: leaves snag.
    pub friction: f32,
    /// Share of its spin the clump loses per second: leaves brushing the
    /// ground and the air as it tumbles, which is what brings a rolling clump
    /// to rest on a gentle slope.
    pub angular_damping: f32,
}

impl Default for LeafCore {
    fn default() -> Self {
        Self {
            core_fraction: 0.8,
            bulk_density: 30.0,
            drag_coefficient: 1.0,
            solidity: 0.3,
            friction: 0.9,
            angular_damping: 0.6,
        }
    }
}

impl LeafCore {
    /// The clump's mass, kg.
    pub fn mass(&self, shape: &FoliageShape) -> f32 {
        self.bulk_density * shape.leaf_volume()
    }

    /// The core pieces of each lobe, a share of its size. Each lobe's
    /// pieces weigh its share of the clump by volume, so the core's centre
    /// of mass is the lobes' volume-weighted centre, which is where
    /// [`FoliageShape::centred`] put the origin. Each piece's bulk, what
    /// fluids and blasts see of it, is the same piece at full size, porous
    /// ([`Self::piece_bulk`]).
    ///
    /// They are placed with the clump's frame at `at` in the body's.
    pub fn colliders(&self, shape: &FoliageShape, at: Isometry3<f32>) -> Vec<ColliderDesc> {
        let total: f32 = shape.lobes.iter().map(Lobe::volume).sum();
        let mass = self.mass(shape);
        let mut colliders = Vec::new();
        for lobe in &shape.lobes {
            let pieces = CorePiece::filling(lobe, self.core_fraction);
            let whole = CorePiece::filling(lobe, 1.0);
            let piece_volume: f32 = pieces.iter().map(CorePiece::volume).sum();
            let density = mass * lobe.volume() / total / piece_volume;
            colliders.extend(pieces.iter().zip(&whole).map(|(piece, whole)| {
                let mut collider = piece
                    .collider()
                    .density(density)
                    .restitution(0.0)
                    .friction(self.friction)
                    .bulk(self.piece_bulk(whole));
                collider.offset = at * collider.offset;
                collider
            }));
        }
        colliders
    }

    /// What fluids and blasts see of one core piece: `whole`, the same piece
    /// at full size, in the piece's frame, porous. It weighs what the piece
    /// does.
    fn piece_bulk(&self, whole: &CorePiece) -> BulkShape {
        BulkShape::displacing(vec![Volume::centred(whole.shape())])
            .dragging_as(self.drag_coefficient)
            .porous(self.solidity)
    }

    /// How the core rests on level ground: the direction, in the clump's
    /// frame, that points down when it does, and how high its origin then
    /// sits above the ground.
    ///
    /// The origin's height over a plane the pieces rest on, with `d` the
    /// way down, is the furthest any piece reaches along `d`
    /// ([`CorePiece::reach`]). The clump rests where that is least, on the
    /// pieces it settles onto, found by sampling directions and refining
    /// round the best.
    pub fn resting_down(&self, shape: &FoliageShape) -> (Vector3<f32>, f32) {
        let pieces: Vec<CorePiece> = shape
            .lobes
            .iter()
            .flat_map(|lobe| CorePiece::filling(lobe, self.core_fraction))
            .collect();
        let height = |d: &Vector3<f32>| {
            pieces
                .iter()
                .map(|piece| piece.reach(d))
                .fold(f32::MIN, f32::max)
        };
        let golden = std::f32::consts::PI * (3.0 - 5.0f32.sqrt());
        const SAMPLES: usize = 2000;
        let mut best = (-Vector3::y(), height(&-Vector3::y()));
        for i in 0..SAMPLES {
            let y = 1.0 - 2.0 * (i as f32 + 0.5) / SAMPLES as f32;
            let ring = (1.0 - y * y).sqrt();
            let azimuth = golden * i as f32;
            let d = Vector3::new(ring * azimuth.cos(), y, ring * azimuth.sin());
            let h = height(&d);
            if h < best.1 {
                best = (d, h);
            }
        }
        // Refine: walk downhill in ever smaller steps.
        let mut step = 0.05;
        while step > 1e-4 {
            let (d, h) = best;
            let a = d
                .cross(&Vector3::x())
                .try_normalize(1e-6)
                .unwrap_or_else(|| d.cross(&Vector3::z()).normalize());
            let b = d.cross(&a);
            let improved = [a, -a, b, -b]
                .iter()
                .map(|t| (d + t * step).normalize())
                .map(|n| (n, height(&n)))
                .filter(|(_, hn)| *hn < h)
                .min_by(|x, y| x.1.total_cmp(&y.1));
            match improved {
                Some(found) => best = found,
                None => step *= 0.5,
            }
        }
        best
    }

    /// The pose a loose clump rests in on level ground at `ground`, turned
    /// `yaw` about the vertical.
    pub fn resting_pose(
        &self,
        shape: &FoliageShape,
        ground: Point3<f32>,
        yaw: f32,
    ) -> Isometry3<f32> {
        let (down, height) = self.resting_down(shape);
        let settle = UnitQuaternion::rotation_between(&down, &-Vector3::y()).unwrap_or_else(|| {
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f32::consts::PI)
        });
        let turn = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), yaw);
        Isometry3::from_parts(
            Translation3::from(ground.coords + Vector3::y() * height),
            turn * settle,
        )
    }

    /// The clump as a dynamic body at `pose`, its origin at the shape's.
    pub fn spawn(
        &self,
        world: &mut PhysicsWorld,
        shape: &FoliageShape,
        pose: Isometry3<f32>,
    ) -> RigidBodyHandle {
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::from(pose.translation.vector))
                .rotation(pose.rotation)
                .angular_damping(self.angular_damping)
                .filter(collision_layers::FOLIAGE_CORE),
        );
        for collider in self.colliders(shape, Isometry3::identity()) {
            world.attach_collider(body, collider);
        }
        body
    }
}

/// How much longer than its shortest radius a lobe's axis must be to be
/// given a capsule along it; below it the lobe is near enough a ball.
const ELONGATED: f32 = 1.2;

/// One solid piece filling part of a lobe, in the clump's frame: a capsule
/// along an axis, or a sphere when it has no length.
#[derive(Debug, Clone, Copy)]
struct CorePiece {
    centre: Vector3<f32>,
    /// Turns local Y, a capsule's axis, onto the piece's axis.
    rotation: UnitQuaternion<f32>,
    /// From the centre to the end of the piece along its axis, caps
    /// included: the radius when it is a sphere.
    half_height: f32,
    radius: f32,
}

impl CorePiece {
    /// The pieces filling `lobe` shrunk by `scale`: a capsule as thick as
    /// its least radius along each axis notably longer than that, crossed
    /// when there are two, or one sphere when there are none.
    fn filling(lobe: &Lobe, scale: f32) -> SmallVec<[Self; 2]> {
        let least = lobe.least_radius();
        let radius = least * scale;
        let mut pieces: SmallVec<[Self; 2]> = (0..3)
            .filter(|&axis| lobe.radii[axis] > ELONGATED * least)
            .map(|axis| {
                let along = Vector3::ith(axis, 1.0);
                let onto = UnitQuaternion::rotation_between(&Vector3::y(), &along)
                    .unwrap_or_else(UnitQuaternion::identity);
                Self {
                    centre: lobe.centre.coords,
                    rotation: lobe.rotation * onto,
                    half_height: lobe.radii[axis] * scale,
                    radius,
                }
            })
            .collect();
        if pieces.is_empty() {
            pieces.push(Self {
                centre: lobe.centre.coords,
                rotation: UnitQuaternion::identity(),
                half_height: radius,
                radius,
            });
        }
        pieces
    }

    fn is_sphere(&self) -> bool {
        self.half_height <= self.radius
    }

    fn shape(&self) -> ColliderShape {
        if self.is_sphere() {
            ColliderShape::Sphere {
                radius: self.radius,
            }
        } else {
            ColliderShape::Capsule {
                half_height: self.half_height,
                radius: self.radius,
            }
        }
    }

    fn collider(&self) -> ColliderDesc {
        let collider = if self.is_sphere() {
            ColliderDesc::sphere(self.radius)
        } else {
            ColliderDesc::capsule(self.half_height, self.radius)
        };
        collider
            .offset_translation(self.centre)
            .offset_rotation(self.rotation)
    }

    /// Its volume, m³.
    fn volume(&self) -> f32 {
        let r = self.radius;
        let cylinder = 2.0 * (self.half_height - r).max(0.0);
        std::f32::consts::PI * r * r * (cylinder + 4.0 / 3.0 * r)
    }

    /// How far it reaches from the clump's origin along the unit `d`: its
    /// centre, half its cylinder along its axis, and its radius.
    fn reach(&self, d: &Vector3<f32>) -> f32 {
        let axis = self.rotation * Vector3::y();
        let half_cylinder = (self.half_height - self.radius).max(0.0);
        self.centre.dot(d) + half_cylinder * axis.dot(d).abs() + self.radius
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foliage::shape::{Lobe, ShrubSpec};

    fn column() -> FoliageShape {
        ShrubSpec::with_radius(0.45)
            .rooted()
            .with_height(2.8)
            .grow(5)
    }

    /// A core balances on the origin its shape was centred on, so the body
    /// turns about where its leaves are, however unlike its lobes are.
    #[test]
    fn a_core_is_centred_on_its_leaves() {
        let shape = FoliageShape {
            lobes: vec![
                Lobe::round(Point3::new(-0.5, 0.0, 0.0), 0.3),
                Lobe {
                    radii: Vector3::new(0.3, 1.2, 0.3),
                    ..Lobe::round(Point3::new(0.5, 0.0, 0.0), 0.3)
                },
            ],
        }
        .centred();
        let mut world = PhysicsWorld::default();
        let core = LeafCore::default();
        let body = core.spawn(&mut world, &shape, Isometry3::identity());
        let shift = world.recenter_on_colliders(body);
        assert!(shift.norm() < 1e-3, "its centre of mass is {shift:?} off");
        let mass = world.body(body).expect("spawned").mass();
        let expected = core.mass(&shape);
        assert!(
            (mass - expected).abs() < 1e-3 * expected,
            "{mass} kg, not {expected}"
        );
    }

    /// What the air and blasts see of a column is about its leaves' volume,
    /// not a ball of its width per lobe.
    #[test]
    fn a_columns_bulk_fills_its_lobes() {
        let shape = column();
        let lobes: f32 = shape.lobes.iter().map(Lobe::volume).sum();
        let bulk: f32 = shape
            .lobes
            .iter()
            .flat_map(|lobe| CorePiece::filling(lobe, 1.0))
            .map(|piece| piece.volume())
            .sum();
        assert!(
            (0.8..1.4).contains(&(bulk / lobes)),
            "bulk {bulk} m³ for {lobes} m³ of lobes"
        );
    }

    /// A column torn loose lies down: it rests far lower than it stands.
    #[test]
    fn a_column_rests_on_its_side() {
        let shape = column();
        let (_, height) = LeafCore::default().resting_down(&shape);
        let (low, high) = shape.bounds();
        assert!(height < 0.25 * (high.y - low.y), "it rests {height} m up");
    }

    /// At its resting pose the lowest core sphere touches the ground, and it
    /// sits no higher than it would stood upright.
    #[test]
    fn a_resting_clump_sits_on_its_spheres() {
        let core = LeafCore::default();
        let shape = ShrubSpec::with_radius(0.6).grow(5);
        let pose = core.resting_pose(&shape, Point3::origin(), 0.3);
        let lowest = shape
            .lobes
            .iter()
            .map(|l| (pose * l.centre).y - l.least_radius() * core.core_fraction)
            .fold(f32::MAX, f32::min);
        assert!(lowest.abs() < 1e-4, "lowest sphere at {lowest}");
        let upright = shape
            .lobes
            .iter()
            .map(|l| l.least_radius() * core.core_fraction - l.centre.y)
            .fold(f32::MIN, f32::max);
        let (_, height) = core.resting_down(&shape);
        assert!(
            height <= upright + 1e-5,
            "it rests higher than it stands upright"
        );
    }
}
