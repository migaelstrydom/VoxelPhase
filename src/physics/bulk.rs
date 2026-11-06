//! What a collider is to the media it moves through, as opposed to what it
//! touches.
//!
//! A collider is a contact envelope: what the world may not pass through.
//! For most colliders that envelope is also what it stands for — a crate is
//! as solid as its box — and nothing here applies. For some it is not. A
//! character's capsule bounds a figure that fills half of it; a barrel's hull
//! bounds air; a clump's core sits inside leaves that drag far wider. Water,
//! wind and blasts should see the figure, the air and the leaves, and
//! contacts should see the capsule, the hull and the core.
//!
//! ```text
//!   RigidBody ── colliders ──────────────────▶ contacts, CCD, raycasts, fracture
//!                  │
//!                  Collider ── bulk: Option<BulkShape>, in its own frame
//!                                │  None: the collider, as it is
//!                                ▼
//!   body's mass_parts(): each collider's, composed with its offset ──▶ mass,
//!                                                     centre of mass, inertia
//!   body's envelope(): each collider's, composed with its offset ──▶ buoyancy,
//!                                         drag, splash (any medium), blasts
//! ```
//!
//! A body sums its colliders. A collider's bulk travels with it, so a body
//! that loses a collider loses exactly that collider's share. The choice
//! between a collider's declaration and its shape is made once, inside the
//! two iterators; nothing that reads a body's bulk asks which it has.

use generational_arena::Arena;
use nalgebra::{Isometry3, Matrix3, Point3, UnitQuaternion, Vector3};

use super::collider::{Collider, ColliderShape};
use super::fluid_drag::ShapeDrag;
use super::handle::ColliderHandle;
use super::impulses::PhysicsImpulse;
use super::silhouette;

/// A shape placed in a collider's frame.
#[derive(Debug, Clone)]
pub struct Volume {
    pub shape: ColliderShape,
    /// Placement in the collider's frame, measured from its origin.
    pub offset: Isometry3<f32>,
}

impl Volume {
    /// `shape` centred on the collider's origin.
    pub fn centred(shape: ColliderShape) -> Self {
        Self {
            shape,
            offset: Isometry3::identity(),
        }
    }

    /// `shape` with its centre at `translation` in the collider's frame.
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

    /// This part as one piece of a mass distribution, placed by `frame`
    /// composed with its own offset.
    fn placed(&self, frame: &Isometry3<f32>) -> MassPartRef<'_> {
        MassPartRef {
            offset: frame * self.volume.offset,
            mass: self.mass,
            local_inertia: &self.local_inertia,
        }
    }
}

/// A collider's bulk, declared apart from its shape. See the module docs.
///
/// Each of its two roles is declared or left to the shape on its own: a
/// creature whose capsule already weighs what it should can declare only what
/// it displaces.
#[derive(Debug, Clone)]
pub struct BulkShape {
    /// Where the collider's mass is. The parts' masses and inertias add.
    /// `None` leaves it to the shape at its density.
    mass: Option<Vec<MassPart>>,
    /// What the collider displaces from a fluid and presents to its flow and
    /// to blasts. `None` leaves it to the shape; empty, it shows nothing: a
    /// tree's wood among its leaves.
    envelope: Option<Vec<Volume>>,
    /// The drag coefficient every fluid sees the envelope with, where its
    /// shapes' own would be wrong: a clump of leaves bounded by spheres
    /// drags as a porous bush, not as smooth balls. `None` leaves each part
    /// its shape's.
    drag_coefficient: Option<f32>,
    /// The share of a blast's loading on its silhouette the envelope
    /// catches, 0 to 1. A blast strikes a solid body twice: its shock
    /// front's pressure jump, then the wind behind it. Through a porous one,
    /// a bush, the pressure evens out at once between the leaves and only the
    /// wind pushes. `None` is solid: all of it.
    solidity: Option<f32>,
}

impl BulkShape {
    /// A solid filling `volume` at `density`: the same shape weighs it and
    /// displaces for it.
    pub fn solid(volume: Volume, density: f32) -> Self {
        Self {
            mass: Some(vec![MassPart::new(volume.clone(), density)]),
            envelope: Some(vec![volume]),
            drag_coefficient: None,
            solidity: None,
        }
    }

    /// A sealed hollow: a shell of `density` between `outer` and `inner`,
    /// displacing everything `outer` encloses.
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
            drag_coefficient: None,
            solidity: None,
        }
    }

    /// Weighing what the collider's shape weighs and displacing only
    /// `volumes`: a creature whose capsule is mostly the air between its
    /// legs. With no volumes, it displaces nothing and catches no blast.
    pub fn displacing(volumes: Vec<Volume>) -> Self {
        Self {
            mass: None,
            envelope: Some(volumes),
            drag_coefficient: None,
            solidity: None,
        }
    }

    /// The same bulk, dragging as `coefficient` in every fluid whatever
    /// its envelope's shapes.
    pub fn dragging_as(mut self, coefficient: f32) -> Self {
        self.drag_coefficient = Some(coefficient);
        self
    }

    /// The same bulk, porous: catching `solidity` (0 to 1) of a blast's
    /// loading on its silhouette.
    pub fn porous(mut self, solidity: f32) -> Self {
        self.solidity = Some(solidity.clamp(0.0, 1.0));
        self
    }

    /// The share of a blast's loading the envelope catches: 1 unless it is
    /// declared porous.
    pub fn solidity(&self) -> f32 {
        self.solidity.unwrap_or(1.0)
    }

    /// Total mass of the declared parts; `None` where the shape weighs the
    /// collider.
    pub fn mass(&self) -> Option<f32> {
        self.mass
            .as_ref()
            .map(|parts| parts.iter().map(|p| p.mass).sum())
    }

    /// The declared parts' inertia about this bulk's origin, in its frame;
    /// `None` where the shape weighs the collider.
    pub(crate) fn inertia(&self) -> Option<Matrix3<f32>> {
        self.mass.as_ref().map(|parts| {
            parts
                .iter()
                .map(|part| part.placed(&Isometry3::identity()).inertia_in_frame())
                .sum()
        })
    }

    /// The impulse `impulse` gives this bulk with its frame at `pose`, as it
    /// gives a collider declaring it: by the area its envelope shows the
    /// blast, times its solidity, struck at its frame's origin. For a part
    /// of a body to judge what a blast does to it alone. `None` out of
    /// reach, or with no envelope declared.
    pub fn blast(&self, pose: &Isometry3<f32>, impulse: &PhysicsImpulse) -> Option<Vector3<f32>> {
        let volumes = self.envelope.as_deref()?;
        let parts = volumes
            .iter()
            .map(|volume| self.envelope_part(volume, &Isometry3::identity()));
        impulse.impulse_on(Point3::from(pose.translation.vector), |travel| {
            silhouette::struck_area(parts, pose.rotation.inverse() * travel)
        })
    }

    /// One of the declared envelope's volumes, placed by `frame` composed
    /// with its own offset.
    fn envelope_part<'a>(&self, volume: &'a Volume, frame: &Isometry3<f32>) -> EnvelopePart<'a> {
        EnvelopePart {
            shape: &volume.shape,
            offset: frame * volume.offset,
            drag_coefficient: self.drag_coefficient,
            solidity: self.solidity(),
        }
    }
}

/// One piece of a body's envelope, whichever collider it came from and
/// whether that collider declared it or is it.
#[derive(Clone, Copy)]
pub struct EnvelopePart<'a> {
    pub shape: &'a ColliderShape,
    /// Placement in the body frame.
    pub offset: Isometry3<f32>,
    /// The drag coefficient its collider's bulk declares, if it does. See
    /// [`BulkShape::dragging_as`].
    pub drag_coefficient: Option<f32>,
    /// The share of a blast's loading it catches, 0 to 1. See
    /// [`BulkShape::porous`].
    pub solidity: f32,
}

impl EnvelopePart<'_> {
    /// What this part presents to a fluid: its shape's areas, with its
    /// declared drag coefficient where it has one.
    pub fn drag(&self) -> ShapeDrag {
        let drag = ShapeDrag::of(self.shape);
        ShapeDrag {
            coefficient: self.drag_coefficient.unwrap_or(drag.coefficient),
            ..drag
        }
    }

    /// Where this part is in the world, for a body at `position`, `rotation`.
    pub fn world_transform(
        &self,
        position: Point3<f32>,
        rotation: UnitQuaternion<f32>,
    ) -> Isometry3<f32> {
        Isometry3::from_parts(position.coords.into(), rotation) * self.offset
    }
}

/// One piece of a body's mass, whichever collider it came from and whether
/// that collider declared it or is it.
#[derive(Clone, Copy)]
pub struct MassPartRef<'a> {
    /// Placement in the body frame.
    pub offset: Isometry3<f32>,
    pub mass: f32,
    /// Inertia about the part's own centre, in the part's frame.
    pub local_inertia: &'a Matrix3<f32>,
}

impl MassPartRef<'_> {
    /// This part's inertia about the origin of the frame it is placed in,
    /// in that frame: its own turned by its offset's rotation, plus the
    /// parallel-axis term for its offset's translation.
    pub fn inertia_in_frame(&self) -> Matrix3<f32> {
        let r = self.offset.rotation.to_rotation_matrix();
        let rotated = r * self.local_inertia * r.transpose();
        let d = self.offset.translation.vector;
        let steiner = self.mass * (d.dot(&d) * Matrix3::identity() - d * d.transpose());
        rotated + steiner
    }
}

/// What one collider gives a body's mass or envelope: a part its bulk
/// declares, or the collider itself.
enum ColliderPart<'a, T> {
    Declared(&'a Collider, &'a T),
    Own(&'a Collider),
}

/// Walks a body's colliders, giving each collider's declared parts of one
/// kind or, where it declares none, the collider itself.
struct PerCollider<'a, T> {
    handles: std::slice::Iter<'a, ColliderHandle>,
    colliders: &'a Arena<Collider>,
    /// Which parts of a collider's bulk this walk gives.
    declared_of: fn(&'a BulkShape) -> Option<&'a [T]>,
    /// The collider whose declared parts are being given, and those left.
    declared: Option<(&'a Collider, std::slice::Iter<'a, T>)>,
}

impl<'a, T> PerCollider<'a, T> {
    fn new(
        handles: &'a [ColliderHandle],
        colliders: &'a Arena<Collider>,
        declared_of: fn(&'a BulkShape) -> Option<&'a [T]>,
    ) -> Self {
        Self {
            handles: handles.iter(),
            colliders,
            declared_of,
            declared: None,
        }
    }
}

impl<'a, T> Iterator for PerCollider<'a, T> {
    type Item = ColliderPart<'a, T>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some((collider, parts)) = &mut self.declared {
                if let Some(part) = parts.next() {
                    return Some(ColliderPart::Declared(collider, part));
                }
                self.declared = None;
            }
            let colliders = self.colliders;
            let collider = self.handles.find_map(|h| colliders.get(h.0))?;
            match collider.bulk().and_then(self.declared_of) {
                Some(parts) => self.declared = Some((collider, parts.iter())),
                None => return Some(ColliderPart::Own(collider)),
            }
        }
    }
}

/// A body's envelope: each collider's declared one, or the collider.
pub struct Envelope<'a>(PerCollider<'a, Volume>);

impl<'a> Iterator for Envelope<'a> {
    type Item = EnvelopePart<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next().map(|part| match part {
            ColliderPart::Declared(collider, volume) => collider
                .bulk()
                .expect("a declared part has a bulk")
                .envelope_part(volume, collider.offset()),
            ColliderPart::Own(collider) => EnvelopePart {
                shape: collider.shape(),
                offset: *collider.offset(),
                drag_coefficient: collider.bulk().and_then(|b| b.drag_coefficient),
                solidity: collider.bulk().map_or(1.0, BulkShape::solidity),
            },
        })
    }
}

/// A body's mass distribution: each collider's declared one, or the
/// collider.
pub struct MassParts<'a>(PerCollider<'a, MassPart>);

impl<'a> Iterator for MassParts<'a> {
    type Item = MassPartRef<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next().map(|part| match part {
            ColliderPart::Declared(collider, part) => part.placed(collider.offset()),
            ColliderPart::Own(collider) => MassPartRef {
                offset: *collider.offset(),
                mass: collider.mass(),
                local_inertia: collider.local_inertia(),
            },
        })
    }
}

/// The envelope of a body with colliders `handles`.
pub(crate) fn envelope<'a>(
    handles: &'a [ColliderHandle],
    colliders: &'a Arena<Collider>,
) -> Envelope<'a> {
    Envelope(PerCollider::new(handles, colliders, |bulk| {
        bulk.envelope.as_deref()
    }))
}

/// The mass parts of a body with colliders `handles`.
pub(crate) fn mass_parts<'a>(
    handles: &'a [ColliderHandle],
    colliders: &'a Arena<Collider>,
) -> MassParts<'a> {
    MassParts(PerCollider::new(handles, colliders, |bulk| {
        bulk.mass.as_deref()
    }))
}

#[cfg(test)]
mod tests {
    use std::f32::consts::{FRAC_PI_2, PI};

    use super::*;
    use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc, RigidBodyHandle};
    use crate::water::buoyancy::{lift, StillWater};

    fn capsule(half_height: f32, radius: f32) -> ColliderShape {
        ColliderShape::Capsule {
            half_height,
            radius,
        }
    }

    fn ball(radius: f32) -> ColliderShape {
        ColliderShape::Sphere { radius }
    }

    /// A blast from 3 m along +Z of the origin, travelling down -Z there.
    fn blast() -> PhysicsImpulse {
        PhysicsImpulse::radial(Point3::new(0.0, 0.0, 3.0), 10.0, 100.0, 0.0)
    }

    /// A body at the origin holding `colliders`.
    fn body_of(world: &mut PhysicsWorld, colliders: Vec<ColliderDesc>) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic());
        for collider in colliders {
            world.attach_collider(body, collider);
        }
        body
    }

    #[test]
    fn a_collider_without_bulk_is_itself() {
        let mut world = PhysicsWorld::default();
        let body = body_of(
            &mut world,
            vec![ColliderDesc::capsule(0.5, 0.25).density(400.0)],
        );

        let expected = capsule(0.5, 0.25).compute_mass(400.0);
        assert!((world.body(body).unwrap().mass() - expected).abs() < 1e-4);
        let parts: Vec<_> = world.envelope(body).collect();
        assert_eq!(parts.len(), 1);
        assert!(matches!(parts[0].shape, ColliderShape::Capsule { radius, .. } if *radius == 0.25));
    }

    #[test]
    fn declared_bulk_sets_mass_and_envelope_and_leaves_the_shape() {
        let mut world = PhysicsWorld::default();
        let bulk = BulkShape::solid(Volume::centred(capsule(0.5, 0.15)), 900.0);
        let body = body_of(
            &mut world,
            vec![ColliderDesc::capsule(0.5, 0.25).density(400.0).bulk(bulk)],
        );

        let expected = capsule(0.5, 0.15).compute_mass(900.0);
        assert!((world.body(body).unwrap().mass() - expected).abs() < 1e-4);
        let parts: Vec<_> = world.envelope(body).collect();
        assert!(matches!(parts[0].shape, ColliderShape::Capsule { radius, .. } if *radius == 0.15));
        let handle = world.body(body).unwrap().colliders()[0];
        let collider = world.collider(handle).unwrap();
        assert!((collider.mass() - expected).abs() < 1e-4);
        assert!(matches!(
            collider.shape(),
            ColliderShape::Capsule { radius, .. } if *radius == 0.25
        ));
    }

    #[test]
    fn a_hollow_sphere_weighs_its_shell_and_has_a_shells_inertia() {
        let (outer, inner, density) = (0.5f32, 0.45f32, 800.0f32);
        let shell = BulkShape::hollow(
            Volume::centred(ball(outer)),
            Volume::centred(ball(inner)),
            density,
        );
        let mut world = PhysicsWorld::default();
        let body = body_of(&mut world, vec![ColliderDesc::sphere(outer).bulk(shell)]);

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
    fn a_displacing_bulk_keeps_the_shapes_mass() {
        let mut world = PhysicsWorld::default();
        let eye = Volume::at(ball(0.2), Vector3::y() * 0.6);
        let body = body_of(
            &mut world,
            vec![ColliderDesc::capsule(0.8, 0.2)
                .density(300.0)
                .bulk(BulkShape::displacing(vec![eye]))],
        );

        let expected = capsule(0.8, 0.2).compute_mass(300.0);
        assert!((world.body(body).unwrap().mass() - expected).abs() < 1e-3);
        let parts: Vec<_> = world.envelope(body).collect();
        assert_eq!(parts.len(), 1);
        assert!(matches!(parts[0].shape, ColliderShape::Sphere { .. }));
        assert!((parts[0].offset.translation.vector.y - 0.6).abs() < 1e-6);
    }

    /// The body weighs and shows each collider's bulk where it declares one
    /// and the collider where it does not.
    #[test]
    fn a_body_sums_its_colliders_bulk() {
        let mut world = PhysicsWorld::default();
        let lump = BulkShape::solid(Volume::centred(ball(0.2)), 500.0);
        let body = body_of(
            &mut world,
            vec![
                ColliderDesc::sphere(0.3)
                    .offset_translation(Vector3::x())
                    .bulk(lump),
                ColliderDesc::sphere(0.3).offset_translation(-Vector3::x()),
            ],
        );

        let expected = ball(0.2).compute_mass(500.0) + ball(0.3).compute_mass(1000.0);
        assert!((world.body(body).unwrap().mass() - expected).abs() < 1e-3);
        let radii: Vec<(f32, f32)> = world
            .envelope(body)
            .map(|p| (p.shape.bounding_radius(), p.offset.translation.vector.x))
            .collect();
        assert_eq!(radii, vec![(0.2, 1.0), (0.3, -1.0)]);
    }

    /// A collider's bulk is in its frame: turning and moving the collider
    /// turns and moves the bulk, for its mass and its envelope alike.
    #[test]
    fn a_rotated_collider_offset_rotates_its_bulk() {
        let lump = Volume::at(ball(0.2), Vector3::y() * 0.5);
        let turn = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        let mut world = PhysicsWorld::default();
        let body = body_of(
            &mut world,
            vec![ColliderDesc::capsule(0.6, 0.2)
                .offset_translation(Vector3::new(0.0, 0.0, 2.0))
                .offset_rotation(turn)
                .bulk(BulkShape::solid(lump, 1000.0))],
        );

        // Local +Y turned a quarter about Z points along -X.
        let expected = Vector3::new(-0.5, 0.0, 2.0);
        let part = world.envelope(body).next().unwrap();
        assert!((part.offset.translation.vector - expected).norm() < 1e-5);
        assert!(part.offset.rotation.angle_to(&turn) < 1e-5);
        let moved = world.recenter_on_colliders(body);
        assert!(
            (moved - expected).norm() < 1e-5,
            "mass centred at {moved:?}"
        );
    }

    #[test]
    fn re_centring_moves_a_bulk_with_its_collider() {
        let mut world = PhysicsWorld::default();
        let lump = Volume::at(ball(0.2), Vector3::x() * 0.5);
        let body = body_of(
            &mut world,
            vec![ColliderDesc::sphere(0.6).bulk(BulkShape::solid(lump, 1000.0))],
        );

        let moved = world.recenter_on_colliders(body);
        assert!((moved - Vector3::x() * 0.5).norm() < 1e-5);
        let part = world.envelope(body).next().unwrap();
        assert!(part.offset.translation.vector.norm() < 1e-5);
        let handle = world.body(body).unwrap().colliders()[0];
        let collider = world.collider(handle).unwrap();
        assert!((collider.offset().translation.vector + Vector3::x() * 0.5).norm() < 1e-5);
    }

    /// Detaching a collider takes its bulk with it and leaves the rest's.
    #[test]
    fn a_colliders_bulk_travels_with_it_when_detached() {
        let mut world = PhysicsWorld::default();
        let small = BulkShape::solid(Volume::centred(ball(0.1)), 500.0);
        let large = BulkShape::solid(Volume::centred(ball(0.2)), 500.0);
        let body = world.create_body(RigidBodyDesc::dynamic());
        let a = world
            .attach_collider(
                body,
                ColliderDesc::sphere(0.3)
                    .offset_translation(Vector3::x())
                    .bulk(small),
            )
            .unwrap();
        world.attach_collider(
            body,
            ColliderDesc::sphere(0.3)
                .offset_translation(-Vector3::x())
                .bulk(large),
        );

        let detached = world.detach_collider(body, a).unwrap();
        let carried = detached.bulk().and_then(BulkShape::mass).unwrap();
        assert!((carried - ball(0.1).compute_mass(500.0)).abs() < 1e-4);
        assert!((detached.mass() - carried).abs() < 1e-6);
        let left = ball(0.2).compute_mass(500.0);
        assert!((world.body(body).unwrap().mass() - left).abs() < 1e-4);
        let parts: Vec<_> = world.envelope(body).collect();
        assert_eq!(parts.len(), 1);
        assert_eq!(parts[0].shape.bounding_radius(), 0.2);
    }

    /// A collider declaring an empty envelope weighs what its shape does
    /// and shows nothing: no lift, no blast.
    #[test]
    fn an_empty_envelope_displaces_nothing() {
        let mut world = PhysicsWorld::default();
        let body = body_of(
            &mut world,
            vec![ColliderDesc::sphere(0.3).bulk(BulkShape::displacing(Vec::new()))],
        );

        let expected = ball(0.3).compute_mass(1000.0);
        assert!((world.body(body).unwrap().mass() - expected).abs() < 1e-3);
        assert_eq!(world.envelope(body).count(), 0);
        let drowned = StillWater {
            surface: 10.0,
            floor: -10.0,
        };
        assert_eq!(lift(&world, body, &drowned), 0.0);
        let struck = world.impulse_on(body, &blast()).unwrap();
        assert_eq!(struck, Vector3::zeros());
    }

    /// Colliders of one solidity catch that share of their union, as a body
    /// of one porous bulk always did.
    #[test]
    fn uniformly_porous_colliders_catch_their_share_of_the_union() {
        let lobes = |solidity: f32| {
            [-0.2f32, 0.2]
                .iter()
                .map(|&x| {
                    ColliderDesc::sphere(0.1)
                        .offset_translation(Vector3::x() * x)
                        .bulk(
                            BulkShape::displacing(vec![Volume::centred(ball(0.3))])
                                .porous(solidity),
                        )
                })
                .collect()
        };
        let mut world = PhysicsWorld::default();
        let porous = body_of(&mut world, lobes(0.3));
        let solid = body_of(&mut world, lobes(1.0));

        let porous = world.impulse_on(porous, &blast()).unwrap();
        let solid = world.impulse_on(solid, &blast()).unwrap();
        assert!((porous - 0.3 * solid).norm() < 1e-5 * solid.norm());
    }

    /// A solid box and a porous one side by side catch the solid's face and
    /// the porous share of the other's; laid over each other, no more than
    /// the one face they show.
    #[test]
    fn mixed_solidity_sums_each_solidity_up_to_the_union() {
        let block = |x: f32, solidity: f32| {
            let half = Vector3::repeat(0.25);
            ColliderDesc::box_shape(half)
                .offset_translation(Vector3::x() * x)
                .bulk(
                    BulkShape::displacing(vec![Volume::centred(ColliderShape::Box {
                        half_extents: half,
                    })])
                    .porous(solidity),
                )
        };
        let mut world = PhysicsWorld::default();
        let one = body_of(&mut world, vec![block(0.0, 1.0)]);
        let apart = body_of(&mut world, vec![block(-1.0, 1.0), block(1.0, 0.3)]);
        let together = body_of(&mut world, vec![block(0.0, 1.0), block(0.0, 0.3)]);

        let one = world.impulse_on(one, &blast()).unwrap().z;
        let apart = world.impulse_on(apart, &blast()).unwrap().z;
        let together = world.impulse_on(together, &blast()).unwrap().z;
        assert!(
            (apart - 1.3 * one).abs() < 1e-4 * one.abs(),
            "{apart} vs {one}"
        );
        assert!(
            (together - one).abs() < 1e-4 * one.abs(),
            "{together} vs {one}"
        );
    }
}
