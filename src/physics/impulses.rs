use nalgebra::{Isometry3, Point3, Vector3};
use smallvec::SmallVec;

use super::bulk::EnvelopePart;
use super::collider::ColliderShape;

/// Where across its bounds a part is sampled, in units of its half extent:
/// the two-point Gauss rule, which weighs a loading that changes linearly
/// across the part exactly.
const GAUSS_POINT: f32 = 0.577_350_3;

// === One-shot impulses (explosions, jump pads, etc.) ===

/// A one-shot impulse event applied to nearby bodies and then discarded.
#[derive(Debug, Clone, Copy)]
pub enum PhysicsImpulse {
    /// A blast wave expanding outward from a point with linear falloff. It
    /// pushes a body by the area the body shows it: a leaf and a boulder of
    /// one size are struck alike, and the leaf flies further for being
    /// lighter.
    Radial {
        /// Center of the impulse in world space.
        center: Point3<f32>,
        /// Radius of effect in meters.
        radius: f32,
        /// Impulse per area struck (N·s/m², Pa·s) at the center, falling off
        /// linearly to zero at the radius.
        strength: f32,
    },
}

impl PhysicsImpulse {
    /// Create a radial impulse (e.g. explosion).
    pub fn radial(center: Point3<f32>, radius: f32, strength: f32) -> Self {
        Self::Radial {
            center,
            radius,
            strength,
        }
    }

    /// The impulse on a body at `body_pos` whose silhouette seen along a
    /// direction is `exposed(direction)`, m². `None` outside the area of
    /// effect.
    pub fn impulse_on(
        &self,
        body_pos: Point3<f32>,
        exposed: impl FnOnce(Vector3<f32>) -> f32,
    ) -> Option<Vector3<f32>> {
        let loading = self.loading_at(body_pos)?;
        Some(loading.per_area * exposed(loading.travel))
    }

    /// Where the blast's push on a body acts: the body posed at `pose`,
    /// with its envelope `parts`. Each part is weighed by the area it shows
    /// the blast and sampled across its bounds, each sample by how hard the
    /// blast strikes there: the near side of a body is struck harder than
    /// the far, so a blast at one end of a plank spins it, and one at a
    /// standing body's feet sweeps them. A part shielded by another still
    /// draws the point to itself. `None` outside the area of effect.
    pub fn centre_of_pressure<'a>(
        &self,
        parts: impl IntoIterator<Item = EnvelopePart<'a>>,
        pose: &Isometry3<f32>,
    ) -> Option<Point3<f32>> {
        let mut weighed = Vector3::zeros();
        let mut weight = 0.0;
        for part in parts {
            let placed = pose * part.offset;
            let Some(loading) = self.loading_at(placed * Point3::origin()) else {
                continue;
            };
            let area = part.solidity
                * part
                    .shape
                    .projected_area(placed.rotation.inverse() * loading.travel);
            let samples = pressure_samples(part.shape);
            let share = area / samples.len() as f32;
            for sample in samples {
                let at = placed * Point3::from(sample);
                let Some(there) = self.loading_at(at) else {
                    continue;
                };
                let w = share * there.per_area.norm();
                weighed += at.coords * w;
                weight += w;
            }
        }
        (weight > 0.0).then(|| Point3::from(weighed / weight))
    }

    /// How hard the blast strikes at `point`: what each square metre facing
    /// it is given. `None` outside the area of effect.
    pub fn loading_at(&self, point: Point3<f32>) -> Option<Loading> {
        match *self {
            Self::Radial {
                center,
                radius,
                strength,
            } => {
                if radius <= 0.0 || strength.abs() < 1e-6 {
                    return None;
                }
                let delta = point - center;
                let distance = delta.magnitude();
                if distance >= radius {
                    return None;
                }
                let (direction, falloff) = if distance < 1e-4 {
                    (Vector3::y(), 1.0)
                } else {
                    (delta / distance, 1.0 - (distance / radius))
                };
                Some(Loading {
                    travel: direction,
                    per_area: direction * strength * falloff,
                })
            }
        }
    }
}

/// Where `shape` is sampled for the pressure on it, in its own frame: a
/// ball at its centre, since a blast's pressure on it all points through
/// its centre; a capsule along its axis; a box or hull at the Gauss points
/// of its bounds.
fn pressure_samples(shape: &ColliderShape) -> SmallVec<[Vector3<f32>; 8]> {
    let corners = |centre: Vector3<f32>, half: Vector3<f32>| {
        let half = half * GAUSS_POINT;
        (0..8)
            .map(|corner| {
                let sign = |bit: u32| if corner & (1 << bit) == 0 { -1.0 } else { 1.0 };
                centre + Vector3::new(sign(0) * half.x, sign(1) * half.y, sign(2) * half.z)
            })
            .collect()
    };
    match shape {
        ColliderShape::Sphere { .. } => SmallVec::from_slice(&[Vector3::zeros()]),
        ColliderShape::Capsule { half_height, .. } => {
            let along = Vector3::y() * (*half_height * GAUSS_POINT);
            SmallVec::from_slice(&[along, -along])
        }
        ColliderShape::Box { half_extents } => corners(Vector3::zeros(), *half_extents),
        ColliderShape::ConvexHull { hull } => corners(hull.box_center, hull.box_half_extents),
    }
}

/// A blast's loading at a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Loading {
    /// The way the wave travels there, unit length.
    pub travel: Vector3<f32>,
    /// The impulse it gives each square metre facing it, N·s/m².
    pub per_area: Vector3<f32>,
}

/// Queue of one-shot impulse events consumed each physics step.
///
/// After draining, a snapshot of the drained impulses is kept in
/// `last_impulses` so that downstream systems (e.g. fracture) can
/// query what was applied this frame.
#[derive(Default)]
pub struct PhysicsImpulseQueue {
    events: Vec<PhysicsImpulse>,
    last_impulses: Vec<PhysicsImpulse>,
}

impl PhysicsImpulseQueue {
    pub fn push(&mut self, event: PhysicsImpulse) {
        self.events.push(event);
    }

    pub fn drain(&mut self) -> impl Iterator<Item = PhysicsImpulse> + '_ {
        self.last_impulses.clear();
        self.last_impulses.extend(self.events.iter().copied());
        self.events.drain(..)
    }

    /// Impulses that were applied during the most recent physics step.
    pub fn last_impulses(&self) -> &[PhysicsImpulse] {
        &self.last_impulses
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::physics::{
        BulkShape, ColliderDesc, ColliderShape, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
        Volume,
    };

    /// A blast 3 m along -x from the origin, reaching 6 m.
    fn blast() -> PhysicsImpulse {
        PhysicsImpulse::radial(Point3::new(-3.0, 0.0, 0.0), 6.0, 100.0)
    }

    fn body_of(world: &mut PhysicsWorld, colliders: Vec<ColliderDesc>) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic());
        for collider in colliders {
            world.attach_collider(body, collider);
        }
        body
    }

    fn struck(colliders: Vec<ColliderDesc>) -> f32 {
        let mut world = PhysicsWorld::default();
        let body = body_of(&mut world, colliders);
        world.impulse_on(body, &blast()).unwrap().x
    }

    /// Two balls of a size are struck alike, whatever they weigh: the
    /// light one flies the faster.
    #[test]
    fn a_blast_strikes_by_size_not_weight() {
        let light = struck(vec![ColliderDesc::sphere(0.5).density(50.0)]);
        let heavy = struck(vec![ColliderDesc::sphere(0.5).density(2500.0)]);
        assert_eq!(light, heavy);
        let half_strength = 100.0 * 0.5;
        let exact = half_strength * std::f32::consts::PI * 0.25;
        assert!(
            (light - exact).abs() < 1e-3 * exact,
            "{light} against {exact}"
        );
    }

    /// A plate facing the blast is struck by its face, and edge-on by its
    /// edge.
    #[test]
    fn a_plate_is_struck_by_the_side_it_shows() {
        let face_on = struck(vec![ColliderDesc::box_shape(Vector3::new(0.025, 0.5, 0.5))]);
        let edge_on = struck(vec![ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 0.025))]);
        assert!(
            (face_on - 20.0 * edge_on).abs() < 1e-3 * face_on,
            "face on {face_on}, edge on {edge_on}"
        );
    }

    /// Two blocks side by side across the blast are struck twice as hard as
    /// one; one behind the other, the front one shields the back.
    #[test]
    fn a_body_in_parts_is_struck_by_its_silhouette() {
        let block = |x: f32, z: f32| {
            ColliderDesc::box_shape(Vector3::repeat(0.25))
                .offset_translation(Vector3::new(x, 0.0, z))
        };
        let one = struck(vec![block(0.0, 0.0)]);
        let abreast = struck(vec![block(0.0, -0.25), block(0.0, 0.25)]);
        let in_line = struck(vec![block(-0.25, 0.0), block(0.25, 0.0)]);
        assert!(
            (abreast - 2.0 * one).abs() < 1e-3 * one,
            "abreast {abreast}, one {one}"
        );
        assert!(
            (in_line - one).abs() < 1e-3 * one,
            "in line {in_line}, one {one}"
        );
    }

    /// A porous body catches its declared share of the blast on its
    /// silhouette; a solid one of the same shape catches all of it.
    #[test]
    fn a_porous_body_catches_its_share_of_a_blast() {
        let solid = struck(vec![ColliderDesc::sphere(0.5)]);
        let mut world = PhysicsWorld::default();
        let bush = world.create_body(RigidBodyDesc::dynamic());
        world.attach_collider(
            bush,
            ColliderDesc::sphere(0.3).bulk(
                BulkShape::displacing(vec![Volume::centred(ColliderShape::Sphere { radius: 0.5 })])
                    .porous(0.3),
            ),
        );
        let porous = world.impulse_on(bush, &blast()).unwrap().x;
        assert!(
            (porous - 0.3 * solid).abs() < 1e-3 * solid,
            "porous {porous}, solid {solid}"
        );
    }

    /// A body at `at` wearing `collider`, shoved by `blast` as a step would.
    fn blasted(
        collider: ColliderDesc,
        at: Point3<f32>,
        blast: &PhysicsImpulse,
    ) -> (PhysicsWorld, RigidBodyHandle) {
        let mut world = PhysicsWorld::default();
        let body = world.create_body(RigidBodyDesc::dynamic().position(at));
        world.attach_collider(body, collider);
        assert!(world.apply_blast(body, blast));
        (world, body)
    }

    /// A blast's pressure on a ball all points through its centre: struck
    /// off to one side, it flies without spinning.
    #[test]
    fn a_blast_does_not_spin_a_ball() {
        let blast = PhysicsImpulse::radial(Point3::new(-1.0, -0.8, 0.3), 4.0, 2000.0);
        let (world, ball) = blasted(ColliderDesc::sphere(0.4), Point3::origin(), &blast);
        let ball = world.body(ball).unwrap();
        assert!(ball.linear_velocity().norm() > 1.0);
        assert!(
            ball.angular_velocity().norm() < 1e-4,
            "{}",
            ball.angular_velocity()
        );
    }

    /// A plank lying across a blast's path is struck harder at the end
    /// nearer the blast, which is thrown the faster.
    #[test]
    fn a_blast_spins_a_plank_by_its_nearer_end() {
        let blast = PhysicsImpulse::radial(Point3::new(1.5, 0.0, -1.0), 4.0, 2000.0);
        let (world, plank) = blasted(
            ColliderDesc::box_shape(Vector3::new(1.5, 0.05, 0.2)),
            Point3::origin(),
            &blast,
        );
        let plank = world.body(plank).unwrap();
        let near = plank.velocity_at(Point3::new(1.5, 0.0, 0.0)).z;
        let far = plank.velocity_at(Point3::new(-1.5, 0.0, 0.0)).z;
        assert!(far > 0.0, "far end {far}");
        assert!(near > 1.5 * far, "near end {near}, far end {far}");
    }

    /// A blast on the ground beside a standing body sweeps its feet out
    /// from under it: they are thrown faster than its head.
    #[test]
    fn a_blast_at_the_feet_sweeps_them() {
        let blast = PhysicsImpulse::radial(Point3::new(-1.5, 0.0, 0.0), 5.0, 2000.0);
        let (world, body) = blasted(
            ColliderDesc::capsule(0.9, 0.25),
            Point3::new(0.0, 0.9, 0.0),
            &blast,
        );
        let body = world.body(body).unwrap();
        let feet = body.velocity_at(Point3::new(0.0, 0.1, 0.0)).x;
        let head = body.velocity_at(Point3::new(0.0, 1.7, 0.0)).x;
        assert!(head > 0.0, "head {head}");
        assert!(feet > head, "feet {feet}, head {head}");
    }
}
