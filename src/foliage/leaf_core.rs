//! The firm heart of a loose clump: what physics sees of it.
//!
//! A ball of leaves is mostly air. What stops when it lands is the dense
//! middle of each lobe, so the body is one sphere inside each lobe, a share
//! of its radius; the leaves round it are drawn (`skin`) and drag what passes
//! through them (`drag`), but nothing collides with them. A cluster of
//! spheres, not one ball: a single sphere has nothing to stop it rolling, and
//! the engine's friction has no rolling resistance, while a lumpy cluster
//! tumbles like a sack and stops. Its bulk, what the air and water see, is
//! its whole lobes, dragging as a porous bush rather than smooth balls: a
//! ball of leaves is mostly air, light for its size, so the world's air slows
//! it to a floaty fall, and a blast, which strikes it by its size, throws it
//! a few dozen metres rather than hundreds.
//!
//! Only round shrubs come loose, so the core reads each lobe as a ball: of
//! its least radius, which is the lobe's own radius when it is round.

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

use super::shape::FoliageShape;
use crate::collision_layers;
use crate::physics::{
    BulkShape, ColliderDesc, ColliderShape, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, Volume,
};

/// How a clump's core is built.
#[derive(Debug, Clone, Copy)]
pub struct LeafCore {
    /// Radius of each lobe's core sphere as a share of the lobe's own. Large,
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

    /// One sphere per lobe, at its centre. Every sphere has the same density,
    /// so the core's centre of mass is the lobes' cube-weighted centre, which
    /// is where [`FoliageShape::centred`] put the origin.
    pub fn colliders(&self, shape: &FoliageShape) -> Vec<ColliderDesc> {
        let radii: Vec<f32> = shape
            .lobes
            .iter()
            .map(|l| l.least_radius() * self.core_fraction)
            .collect();
        let core_volume: f32 = radii
            .iter()
            .map(|r| 4.0 / 3.0 * std::f32::consts::PI * r.powi(3))
            .sum();
        let density = self.mass(shape) / core_volume;
        shape
            .lobes
            .iter()
            .zip(radii)
            .map(|(lobe, radius)| {
                ColliderDesc::sphere(radius)
                    .density(density)
                    .restitution(0.0)
                    .friction(self.friction)
                    .offset_translation(lobe.centre.coords)
            })
            .collect()
    }

    /// What fluids and blasts see of the clump: every lobe whole, porous. It
    /// weighs what its core spheres weigh.
    pub fn bulk(&self, shape: &FoliageShape) -> BulkShape {
        BulkShape::displacing(
            shape
                .lobes
                .iter()
                .map(|lobe| {
                    Volume::at(
                        ColliderShape::Sphere {
                            radius: lobe.least_radius(),
                        },
                        lobe.centre.coords,
                    )
                })
                .collect(),
        )
        .dragging_as(self.drag_coefficient)
        .porous(self.solidity)
    }

    /// How the core rests on level ground: the direction, in the clump's
    /// frame, that points down when it does, and how high its origin then
    /// sits above the ground.
    ///
    /// The origin's height over a plane the spheres rest on, with `d` the
    /// way down, is the furthest any sphere reaches along `d`:
    /// `max(c·d + r)`. The clump rests where that is least, on the three
    /// spheres (or two, or one) it settles onto, found by sampling
    /// directions and refining round the best.
    pub fn resting_down(&self, shape: &FoliageShape) -> (Vector3<f32>, f32) {
        let spheres: Vec<(Vector3<f32>, f32)> = shape
            .lobes
            .iter()
            .map(|l| (l.centre.coords, l.least_radius() * self.core_fraction))
            .collect();
        let height = |d: &Vector3<f32>| {
            spheres
                .iter()
                .map(|(c, r)| c.dot(d) + r)
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
                .bulk(self.bulk(shape))
                .filter(collision_layers::FOLIAGE_CORE),
        );
        for collider in self.colliders(shape) {
            world.attach_collider(body, collider);
        }
        body
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foliage::shape::ShrubSpec;

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
