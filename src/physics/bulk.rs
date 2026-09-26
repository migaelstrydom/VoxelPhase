//! What a body is to the media it moves through, as opposed to what it touches.
//!
//! A body's colliders are its contact envelope: what the world may not pass
//! through. For most bodies that envelope is also the body — a crate is as
//! solid as its box — and nothing here applies. For some it is not. A
//! character's capsule bounds a figure that fills half of it; a barrel's hull
//! bounds air. Water, wind, snow and quicksand should see the figure and the
//! air, and contacts should see the capsule and the hull.
//!
//! ```text
//!   RigidBody ─┬─ colliders ─────────────────▶ contacts, CCD, raycasts, fracture
//!              │
//!              └─ bulk: Option<BulkShape>
//!                    │  None: the colliders, as they are
//!                    ▼
//!              mass_parts() ─────────────────▶ mass, centre of mass, inertia
//!              envelope() ───────────────────▶ buoyancy, drag, splash (any medium)
//! ```
//!
//! The choice between the override and the colliders is made once, inside the
//! two iterators; nothing that reads a body's bulk asks which it has.

use generational_arena::Arena;
use nalgebra::{Isometry3, Matrix3, Point3, UnitQuaternion, Vector3};

use super::collider::{Collider, ColliderShape};
use super::handle::ColliderHandle;

/// A shape placed in a body's frame.
#[derive(Debug, Clone)]
pub struct Volume {
    pub shape: ColliderShape,
    /// Placement in the body frame, measured from the body's origin.
    pub offset: Isometry3<f32>,
}

impl Volume {
    /// `shape` centred on the body's origin.
    pub fn centred(shape: ColliderShape) -> Self {
        Self {
            shape,
            offset: Isometry3::identity(),
        }
    }

    /// `shape` with its centre at `translation` in the body frame.
    pub fn at(shape: ColliderShape, translation: Vector3<f32>) -> Self {
        Self {
            shape,
            offset: Isometry3::translation(translation.x, translation.y, translation.z),
        }
    }
}

/// A volume that carries mass.
#[derive(Debug, Clone)]
struct MassPart {
    volume: Volume,
    /// Signed: a negative part takes mass away from where a positive one put
    /// it, which is how a shell is its outer solid less its inner one.
    mass: f32,
    /// Inertia about the part's own centre, in the part's frame. Signed with
    /// `mass`.
    local_inertia: Matrix3<f32>,
}

impl MassPart {
    fn new(volume: Volume, density: f32) -> Self {
        let mass = volume.shape.compute_mass(density);
        let local_inertia = volume.shape.compute_inertia(mass);
        Self {
            volume,
            mass,
            local_inertia,
        }
    }
}

/// A body's bulk, declared apart from its colliders. See the module docs.
///
/// Each of its two roles is declared or left to the colliders on its own: a
/// creature whose capsule already weighs what it should can declare only what
/// it displaces.
#[derive(Debug, Clone)]
pub struct BulkShape {
    /// Where the body's mass is. The parts' masses and inertias add, and
    /// their centre of mass belongs on the body's origin, as a collider's
    /// does. `None` leaves it to the colliders.
    mass: Option<Vec<MassPart>>,
    /// What the body displaces from a fluid and presents to its flow. `None`
    /// leaves it to the colliders.
    envelope: Option<Vec<Volume>>,
}

impl BulkShape {
    /// A solid body filling `volume` at `density`: the same shape weighs it
    /// and displaces for it.
    pub fn solid(volume: Volume, density: f32) -> Self {
        Self {
            mass: Some(vec![MassPart::new(volume.clone(), density)]),
            envelope: Some(vec![volume]),
        }
    }

    /// A sealed hollow body: a shell of `density` between `outer` and
    /// `inner`, displacing everything `outer` encloses.
    ///
    /// `inner` must lie inside `outer`; the shell's mass and inertia are
    /// exactly the outer solid's less the inner one's.
    pub fn hollow(outer: Volume, inner: Volume, density: f32) -> Self {
        Self {
            mass: Some(vec![
                MassPart::new(outer.clone(), density),
                MassPart::new(inner, -density),
            ]),
            envelope: Some(vec![outer]),
        }
    }

    /// A body that weighs what its colliders weigh and displaces only
    /// `volumes`: a creature whose capsule is mostly the air between its legs.
    pub fn displacing(volumes: Vec<Volume>) -> Self {
        Self {
            mass: None,
            envelope: Some(volumes),
        }
    }

    /// Total mass of the declared parts; `None` where the colliders weigh the
    /// body.
    pub fn mass(&self) -> Option<f32> {
        self.mass
            .as_ref()
            .map(|parts| parts.iter().map(|p| p.mass).sum())
    }

    /// Move every part by `delta` in the body frame, as a body re-centring on
    /// its mass does to everything attached to it.
    pub(crate) fn shift(&mut self, delta: Vector3<f32>) {
        for part in self.mass.iter_mut().flatten() {
            part.volume.offset.translation.vector += delta;
        }
        for volume in self.envelope.iter_mut().flatten() {
            volume.offset.translation.vector += delta;
        }
    }
}

/// One piece of a body's envelope, whichever of the two it came from.
#[derive(Clone, Copy)]
pub struct EnvelopePart<'a> {
    pub shape: &'a ColliderShape,
    /// Placement in the body frame.
    pub offset: &'a Isometry3<f32>,
}

impl EnvelopePart<'_> {
    /// Where this part is in the world, for a body at `position`, `rotation`.
    pub fn world_transform(
        &self,
        position: Point3<f32>,
        rotation: UnitQuaternion<f32>,
    ) -> Isometry3<f32> {
        Isometry3::from_parts(position.coords.into(), rotation) * self.offset
    }
}

/// One piece of a body's mass, whichever of the two it came from.
#[derive(Clone, Copy)]
pub struct MassPartRef<'a> {
    /// Placement in the body frame.
    pub offset: &'a Isometry3<f32>,
    pub mass: f32,
    /// Inertia about the part's own centre, in the part's frame.
    pub local_inertia: &'a Matrix3<f32>,
}

/// The two places a body's bulk can come from.
enum Source<'a, T> {
    Declared(std::slice::Iter<'a, T>),
    Colliders {
        handles: std::slice::Iter<'a, ColliderHandle>,
        colliders: &'a Arena<Collider>,
    },
}

impl<'a, T> Source<'a, T> {
    fn new(
        declared: Option<&'a [T]>,
        handles: &'a [ColliderHandle],
        colliders: &'a Arena<Collider>,
    ) -> Self {
        match declared {
            Some(parts) => Source::Declared(parts.iter()),
            None => Source::Colliders {
                handles: handles.iter(),
                colliders,
            },
        }
    }

    fn next_with<R>(
        &mut self,
        declared: impl Fn(&'a T) -> R,
        collider: impl Fn(&'a Collider) -> R,
    ) -> Option<R> {
        match self {
            Source::Declared(parts) => parts.next().map(declared),
            Source::Colliders { handles, colliders } => {
                handles.find_map(|h| colliders.get(h.0)).map(collider)
            }
        }
    }
}

/// A body's envelope: its declared one, or its colliders.
pub struct Envelope<'a>(Source<'a, Volume>);

impl<'a> Iterator for Envelope<'a> {
    type Item = EnvelopePart<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next_with(
            |v| EnvelopePart {
                shape: &v.shape,
                offset: &v.offset,
            },
            |c| EnvelopePart {
                shape: c.shape(),
                offset: c.offset(),
            },
        )
    }
}

/// A body's mass distribution: its declared one, or its colliders.
pub struct MassParts<'a>(Source<'a, MassPart>);

impl<'a> Iterator for MassParts<'a> {
    type Item = MassPartRef<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next_with(
            |p| MassPartRef {
                offset: &p.volume.offset,
                mass: p.mass,
                local_inertia: &p.local_inertia,
            },
            |c| MassPartRef {
                offset: c.offset(),
                mass: c.mass(),
                local_inertia: c.local_inertia(),
            },
        )
    }
}

/// The envelope of a body with `bulk` and `handles`.
pub(crate) fn envelope<'a>(
    bulk: Option<&'a BulkShape>,
    handles: &'a [ColliderHandle],
    colliders: &'a Arena<Collider>,
) -> Envelope<'a> {
    Envelope(Source::new(
        bulk.and_then(|b| b.envelope.as_deref()),
        handles,
        colliders,
    ))
}

/// The mass parts of a body with `bulk` and `handles`.
pub(crate) fn mass_parts<'a>(
    bulk: Option<&'a BulkShape>,
    handles: &'a [ColliderHandle],
    colliders: &'a Arena<Collider>,
) -> MassParts<'a> {
    MassParts(Source::new(
        bulk.and_then(|b| b.mass.as_deref()),
        handles,
        colliders,
    ))
}

#[cfg(test)]
mod tests {
    use std::f32::consts::PI;

    use super::*;
    use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc};

    fn capsule(half_height: f32, radius: f32) -> ColliderShape {
        ColliderShape::Capsule {
            half_height,
            radius,
        }
    }

    #[test]
    fn a_body_without_bulk_is_its_colliders() {
        let mut world = PhysicsWorld::default();
        let body = world.create_body(RigidBodyDesc::dynamic());
        world.attach_collider(body, ColliderDesc::capsule(0.5, 0.25).density(400.0));

        let expected = capsule(0.5, 0.25).compute_mass(400.0);
        assert!((world.body(body).unwrap().mass() - expected).abs() < 1e-4);
        let parts: Vec<_> = world.envelope(body).collect();
        assert_eq!(parts.len(), 1);
        assert!(matches!(parts[0].shape, ColliderShape::Capsule { radius, .. } if *radius == 0.25));
    }

    #[test]
    fn declared_bulk_sets_mass_and_envelope_and_leaves_the_colliders() {
        let mut world = PhysicsWorld::default();
        let bulk = BulkShape::solid(Volume::centred(capsule(0.5, 0.15)), 900.0);
        let body = world.create_body(RigidBodyDesc::dynamic().bulk(bulk));
        world.attach_collider(body, ColliderDesc::capsule(0.5, 0.25).density(400.0));

        let expected = capsule(0.5, 0.15).compute_mass(900.0);
        assert!((world.body(body).unwrap().mass() - expected).abs() < 1e-4);
        let parts: Vec<_> = world.envelope(body).collect();
        assert!(matches!(parts[0].shape, ColliderShape::Capsule { radius, .. } if *radius == 0.15));
        let handle = world.body(body).unwrap().colliders()[0];
        assert!(matches!(
            world.collider(handle).unwrap().shape(),
            ColliderShape::Capsule { radius, .. } if *radius == 0.25
        ));
    }

    #[test]
    fn a_hollow_sphere_weighs_its_shell_and_has_a_shells_inertia() {
        let (outer, inner, density) = (0.5f32, 0.45f32, 800.0f32);
        let shell = BulkShape::hollow(
            Volume::centred(ColliderShape::Sphere { radius: outer }),
            Volume::centred(ColliderShape::Sphere { radius: inner }),
            density,
        );
        let mut world = PhysicsWorld::default();
        let body = world.create_body(RigidBodyDesc::dynamic().bulk(shell));
        world.attach_collider(body, ColliderDesc::sphere(outer));

        let mass = density * 4.0 / 3.0 * PI * (outer.powi(3) - inner.powi(3));
        let body = world.body(body).unwrap();
        assert!((body.mass() - mass).abs() / mass < 1e-4);
        // I = (2/5)·m·(R⁵ − r⁵)/(R³ − r³), between a solid's 2/5 and a thin
        // shell's 2/3 of m·R².
        let inertia =
            0.4 * mass * (outer.powi(5) - inner.powi(5)) / (outer.powi(3) - inner.powi(3));
        let world_inertia = body.world_inv_inertia().try_inverse().unwrap();
        assert!((world_inertia[(0, 0)] - inertia).abs() / inertia < 1e-3);
    }

    #[test]
    fn a_displacing_bulk_keeps_the_colliders_mass() {
        let mut world = PhysicsWorld::default();
        let eye = Volume::at(ColliderShape::Sphere { radius: 0.2 }, Vector3::y() * 0.6);
        let body =
            world.create_body(RigidBodyDesc::dynamic().bulk(BulkShape::displacing(vec![eye])));
        world.attach_collider(body, ColliderDesc::capsule(0.8, 0.2).density(300.0));

        let expected = capsule(0.8, 0.2).compute_mass(300.0);
        assert!((world.body(body).unwrap().mass() - expected).abs() < 1e-3);
        let parts: Vec<_> = world.envelope(body).collect();
        assert_eq!(parts.len(), 1);
        assert!(matches!(parts[0].shape, ColliderShape::Sphere { .. }));
        assert!((parts[0].offset.translation.vector.y - 0.6).abs() < 1e-6);
    }

    #[test]
    fn re_centring_on_a_declared_mass_moves_the_bulk_with_the_colliders() {
        let mut world = PhysicsWorld::default();
        let lump = Volume::at(ColliderShape::Sphere { radius: 0.2 }, Vector3::x() * 0.5);
        let bulk = BulkShape::solid(lump, 1000.0);
        let body = world.create_body(RigidBodyDesc::dynamic().bulk(bulk));
        world.attach_collider(body, ColliderDesc::sphere(0.6));

        let moved = world.recenter_on_colliders(body);
        assert!((moved - Vector3::x() * 0.5).norm() < 1e-5);
        let part = world.envelope(body).next().unwrap();
        assert!(part.offset.translation.vector.norm() < 1e-5);
        let handle = world.body(body).unwrap().colliders()[0];
        let collider = world.collider(handle).unwrap();
        assert!((collider.offset().translation.vector + Vector3::x() * 0.5).norm() < 1e-5);
    }

    #[test]
    fn detaching_a_collider_drops_the_declared_bulk() {
        let mut world = PhysicsWorld::default();
        let bulk = BulkShape::solid(
            Volume::centred(ColliderShape::Sphere { radius: 0.1 }),
            500.0,
        );
        let body = world.create_body(RigidBodyDesc::dynamic().bulk(bulk));
        let a = world
            .attach_collider(
                body,
                ColliderDesc::sphere(0.3).offset_translation(Vector3::x()),
            )
            .unwrap();
        world.attach_collider(
            body,
            ColliderDesc::sphere(0.3).offset_translation(-Vector3::x()),
        );

        world.detach_collider(body, a);
        let expected = ColliderShape::Sphere { radius: 0.3 }.compute_mass(1000.0);
        assert!((world.body(body).unwrap().mass() - expected).abs() < 1e-3);
        assert!(world.body(body).unwrap().bulk().is_none());
    }
}
