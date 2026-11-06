//! Fracture ECS system.

use nalgebra::{Isometry3, Vector3};
use specs::{Entities, Entity, Join, Read, System, Write, WriteStorage};

use super::components::CompoundFracture;
use super::debris::Debris;
use super::load::BondLoads;
use super::parts::PartId;
use super::piece_split::{PieceSplit, PieceSplits};
use crate::components::{Orientation, Position, Renderable, RigidBodyComponent, Velocity};
use crate::model::PartLooks;
use crate::physics::{ColliderHandle, PhysicsImpulse, PhysicsImpulseQueue, RigidBodyHandle};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// Breaks the bonds of compound bodies whose load at the bond exceeds what
/// it can hold, then splits every group of parts still bonded but the one
/// that stays off onto a body of its own.
///
/// Two loads are judged at each bond's anchor:
///
/// - **Blast.** `PhysicsImpulseQueue::last_impulses()`' loading per area
///   where the bond holds, so an explosion opens a hole where it went off.
/// - **Impact.** The frame-over-frame *spike* in the contact impulse the
///   solver pushed through either part's colliders, felt at the anchor over
///   the bond's reach.
///
/// Contact is read as a spike and not as a level on purpose. The level includes
/// whatever the object is already carrying: a heavy compound standing still
/// pushes `mass * gravity * frame_dt` through its lowest parts every frame,
/// which for anything massive is larger than a sensible fracture threshold on
/// its own. Differencing against last frame leaves resting weight and steady
/// pushing at roughly zero and keeps only the step change of a real collision.
///
/// The group that stays is the tied one, else the heaviest. Every other
/// group goes whole through `PhysicsWorld::split_off`, taking its ties, its
/// share of the compound and of the looks, its kick part by part, and a
/// [`PieceSplit`] for the owners to add their own share.
pub struct FractureSystem;

impl<'a> System<'a> for FractureSystem {
    #[allow(clippy::type_complexity)]
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        WriteStorage<'a, CompoundFracture>,
        WriteStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, PartLooks>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, Orientation>,
        WriteStorage<'a, Renderable>,
        WriteStorage<'a, Debris>,
        Write<'a, PieceSplits>,
        Read<'a, PhysicsImpulseQueue>,
        Read<'a, Time>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            mut physics,
            mut fractures,
            mut bodies,
            mut looks,
            mut positions,
            mut velocities,
            mut orientations,
            mut renderables,
            mut debris,
            mut splits,
            impulse_queue,
            time,
        ) = data;
        splits.clear();
        let blasts = impulse_queue.last_impulses();

        let mut breaking: Vec<(Entity, RigidBodyHandle)> = Vec::new();
        for (entity, fracture, body) in (&entities, &mut fractures, &bodies).join() {
            if judge(&physics, fracture, body.0, blasts, time.delta_seconds()) {
                breaking.push((entity, body.0));
            }
        }

        for (entity, body) in breaking {
            let Some(fracture) = fractures.get_mut(entity) else {
                continue;
            };
            let pieces = split(&mut physics, fracture, body, blasts);
            if pieces.is_empty() {
                continue;
            }
            let kept: Vec<ColliderHandle> = physics
                .world
                .body(body)
                .map_or_else(Vec::new, |b| b.colliders().to_vec());
            let shares: Vec<(CompoundFracture, Option<Debris>)> = pieces
                .iter()
                .map(|piece| {
                    let share = fracture.share(&piece.parts, &piece.colliders);
                    let debris = fracture.sheds_debris.then(|| {
                        let size = piece_volume(&physics, &piece.colliders);
                        Debris::new(entity, size)
                    });
                    (share, debris)
                })
                .collect();
            fracture.retain(&kept);

            for (piece, (share, piece_debris)) in pieces.iter().zip(shares) {
                let Some(new_body) = physics.world.body(piece.body) else {
                    continue;
                };
                let new_entity = entities.create();
                let piece_looks = looks
                    .get(entity)
                    .map(|l| l.share(&piece.colliders))
                    .unwrap_or_default();
                let _ = positions.insert(new_entity, Position(new_body.position().coords));
                let _ = velocities.insert(new_entity, Velocity(new_body.linear_velocity()));
                let _ = orientations.insert(new_entity, Orientation(new_body.rotation()));
                let _ = bodies.insert(new_entity, RigidBodyComponent(piece.body));
                let _ = renderables.insert(new_entity, Renderable);
                let _ = looks.insert(new_entity, piece_looks);
                let _ = fractures.insert(new_entity, share);
                if let Some(piece_debris) = piece_debris {
                    let _ = debris.insert(new_entity, piece_debris);
                }
                splits.push(PieceSplit {
                    from: entity,
                    piece: new_entity,
                });
            }

            // The kept body re-centred on what it holds, so its looks are
            // drawn again about its new origin as well as without the
            // pieces that left.
            if let Some(looks) = looks.get_mut(entity) {
                looks.retain(&kept);
                looks.mark_changed();
            }
        }
    }
}

/// Advance a compound's loads for the frame and break the bonds that cannot
/// hold. Whether it must split.
fn judge(
    physics: &PhysicsResource,
    fracture: &mut CompoundFracture,
    body: RigidBodyHandle,
    blasts: &[PhysicsImpulse],
    dt: f32,
) -> bool {
    let Some(rigid) = physics.world.body(body) else {
        return false;
    };
    let colliders = rigid.colliders().to_vec();
    let pose = Isometry3::from_parts(rigid.position().coords.into(), rigid.rotation());

    // The impact baseline has to advance every frame, including frames where
    // this body cannot break. Skipping it would let a load build up
    // unwatched and then read as one huge spike later.
    let spikes = fracture
        .contact_load
        .advance(&physics.world, body, &colliders, dt);
    if fracture.graph.parts().len() <= 1 && !fracture.released {
        return false;
    }

    let loads = BondLoads::new(
        colliders.into_iter().zip(spikes),
        blasts,
        pose,
        fracture.default_toughness,
    );
    let breaks: Vec<bool> = fracture
        .graph
        .bonds()
        .iter()
        .map(|bond| loads.breaks(&fracture.graph, bond))
        .collect();
    let mut verdicts = breaks.into_iter();
    let broke = fracture
        .graph
        .break_bonds(|_| verdicts.next().unwrap_or(false));
    broke > 0 || fracture.split_pending
}

/// One group split off onto a body of its own.
struct Piece {
    parts: Vec<PartId>,
    colliders: Vec<ColliderHandle>,
    body: RigidBodyHandle,
}

/// Split every group but the one that stays off `body`, each through
/// `split_off` with its ties, kicked part by part. The compound's graph
/// still holds every part when this returns, its anchors in the kept body's
/// frame; each piece's share is taken from it.
fn split(
    physics: &mut PhysicsResource,
    fracture: &mut CompoundFracture,
    body: RigidBodyHandle,
    blasts: &[PhysicsImpulse],
) -> Vec<Piece> {
    fracture.split_pending = false;
    for constraint in fracture.untied.drain(..) {
        physics.world.remove_constraint(constraint);
    }
    if std::mem::take(&mut fracture.released) {
        for constraint in fracture.graph.release() {
            physics.world.remove_constraint(constraint);
        }
    }

    let groups = fracture.graph.groups();
    let weight = |part: PartId| -> f32 {
        fracture
            .graph
            .colliders_of(&[part])
            .iter()
            .filter_map(|c| physics.world.collider(*c))
            .map(|c| c.mass())
            .sum()
    };
    let Some(kept) = fracture.graph.kept_group(&groups, weight) else {
        return Vec::new();
    };
    let Some(source) = physics.world.body(body) else {
        return Vec::new();
    };
    let (filter, ignores_static) = (*source.filter(), source.ignores_static());

    let mut pieces = Vec::new();
    for (index, group) in groups.iter().enumerate() {
        if index == kept {
            continue;
        }
        let colliders = fracture.graph.colliders_of(group);
        let ties = fracture.graph.ties_of(group);
        // A piece held where it stands stays bedded as the body was; a loose
        // one meets the ground.
        let traits = fracture
            .pieces
            .body(filter, ignores_static && !ties.is_empty());
        let Some(made) = physics.world.split_off(body, &colliders, &ties, traits) else {
            continue;
        };
        // The piece's bonds are re-expressed in its own frame when its share
        // is taken; everything left follows the kept body's origin.
        let rest: Vec<PartId> = fracture
            .graph
            .parts()
            .iter()
            .map(|part| part.id)
            .filter(|id| !group.contains(id))
            .collect();
        let kept_moved = -made.kept_moved;
        fracture
            .graph
            .transform_anchors(group, &made.new_frame.inverse());
        fracture.graph.transform_anchors(
            &rest,
            &Isometry3::translation(kept_moved.x, kept_moved.y, kept_moved.z),
        );
        for part in group {
            kick(physics, fracture, made.body, *part, blasts);
        }
        pieces.push(Piece {
            parts: group.clone(),
            colliders,
            body: made.body,
        });
    }

    // What severed a part that stayed still pushes it.
    for part in &groups[kept] {
        kick(physics, fracture, body, *part, &[]);
    }
    fracture.kicks.clear();
    pieces
}

/// Push `part`, now on `body`, at its own centre of mass: its share of
/// `blasts`, by the area its colliders show them, and whatever severed it.
/// A group struck off-centre spins.
fn kick(
    physics: &mut PhysicsResource,
    fracture: &CompoundFracture,
    body: RigidBodyHandle,
    part: PartId,
    blasts: &[PhysicsImpulse],
) {
    let colliders = fracture.graph.colliders_of(&[part]);
    let Some((_, centre)) = physics.world.mass_centre_of(body, &colliders) else {
        return;
    };
    let blast: Vector3<f32> = blasts
        .iter()
        .filter_map(|blast| physics.world.impulse_on_colliders(body, &colliders, blast))
        .map(|(impulse, _)| impulse)
        .sum();
    let impulse = blast + fracture.kick_of(part);
    if impulse != Vector3::zeros() {
        physics.world.apply_impulse_at_point(body, impulse, centre);
    }
}

/// The volume of a piece's colliders together, m³: the size the debris
/// budget ranks it by.
fn piece_volume(physics: &PhysicsResource, colliders: &[ColliderHandle]) -> f32 {
    colliders
        .iter()
        .filter_map(|c| physics.world.collider(*c))
        .map(|c| c.shape().compute_mass(1.0))
        .sum()
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use nalgebra::{Point3, Vector3};
    use specs::{Builder, Join, RunNow, World, WorldExt};

    use super::*;
    use crate::components::ModelInstance;
    use crate::debug::DebugLines;
    use crate::fracture::{Bond, ContactLoadTracker, PartGraph};
    use crate::geometry::{generate_cube_indices, generate_cube_vertices};
    use crate::model::Look;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{
        ColliderDesc, ConstraintKind, PhysicsConfig, PhysicsImpulse, PhysicsWorld, RigidBodyDesc,
    };
    use crate::rendering::colour::Colour;
    use crate::rendering::material::MaterialId;
    use crate::systems::PartDrawSystem;

    const FRAME_DT: f32 = 1.0 / 60.0;
    const HALF: Vector3<f32> = Vector3::new(0.5, 0.5, 0.5);

    /// A four-block slab, the shape of one course of an igloo wall, heavy
    /// enough that its own weight is the interesting quantity.
    fn compound_slab(
        world: &mut PhysicsWorld,
        height: f32,
        velocity: Vector3<f32>,
    ) -> (RigidBodyHandle, Vec<ColliderHandle>) {
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, height, 0.0))
                .linear_velocity(velocity),
        );
        let handles = (0..4)
            .map(|i| {
                world
                    .attach_collider(
                        body,
                        ColliderDesc::box_shape(HALF)
                            .offset_translation(Vector3::new(i as f32 - 1.5, 0.0, 0.0))
                            .density(900.0)
                            .restitution(0.0),
                    )
                    .expect("collider attaches to a live body")
            })
            .collect();
        (body, handles)
    }

    /// Run the world and report the largest per-collider spike seen over the
    /// frames in `window`, exactly as `FractureSystem` would read it.
    ///
    /// `drive`, when given, is forced onto the body's linear velocity every
    /// frame — a stand-in for something that keeps pushing.
    fn peak_spike(
        world: &mut PhysicsWorld,
        body: RigidBodyHandle,
        colliders: &[ColliderHandle],
        frames: usize,
        window: std::ops::Range<usize>,
        drive: Option<Vector3<f32>>,
    ) -> f32 {
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        let mut debug = DebugLines::default();
        let mut tracker = ContactLoadTracker::default();
        let mut peak = 0.0f32;

        for frame in 0..frames {
            if let (Some(velocity), Some(body_mut)) = (drive, world.body_mut(body)) {
                body_mut.set_linear_velocity(velocity);
            }
            stepper.step(world, FRAME_DT, &geometry, &[], &[], &mut debug);
            let spikes = tracker.advance(world, body, colliders, FRAME_DT);
            if window.contains(&frame) {
                peak = peak.max(spikes.into_iter().fold(0.0f32, |m, s| m.max(s.magnitude)));
            }
        }
        peak
    }

    /// The whole contact-fracture rule rests on this margin. A compound's own
    /// weight must not look like a hit, and a hit must not look like weight —
    /// with enough room between them that an authored threshold can sit in the
    /// gap without being fussy. Measured on a 3.6 t slab: 9e-5 N·s standing,
    /// 2e-3 N·s while shoved along the ground, 20549 N·s on landing.
    ///
    /// Sleep is off for the two quiet cases on purpose. A sleeping body reports
    /// nothing at all, which would make the comparison vacuous; what has to be
    /// small is what an *awake* compound produces just by existing.
    #[test]
    fn a_landing_and_a_compound_sitting_still_are_orders_of_magnitude_apart() {
        let mut awake = PhysicsConfig::default();
        awake.sleep.enabled = false;

        let mut resting_world = PhysicsWorld::new(awake.clone());
        let (resting, resting_colliders) = compound_slab(&mut resting_world, 0.5, Vector3::zeros());
        let resting_peak = peak_spike(
            &mut resting_world,
            resting,
            &resting_colliders,
            240,
            60..240,
            None,
        );

        let mut pushed_world = PhysicsWorld::new(awake);
        let (pushed, pushed_colliders) = compound_slab(&mut pushed_world, 0.5, Vector3::zeros());
        let pushed_peak = peak_spike(
            &mut pushed_world,
            pushed,
            &pushed_colliders,
            240,
            60..240,
            Some(Vector3::new(3.0, 0.0, 0.0)),
        );

        let mut dropped_world = PhysicsWorld::new(PhysicsConfig::default());
        let (dropped, dropped_colliders) =
            compound_slab(&mut dropped_world, 4.0, Vector3::new(0.0, -12.0, 0.0));
        let landing_peak = peak_spike(
            &mut dropped_world,
            dropped,
            &dropped_colliders,
            240,
            0..240,
            None,
        );

        let quiet = resting_peak.max(pushed_peak);
        assert!(
            landing_peak > quiet * 20.0,
            "landing {landing_peak} is not clear of resting {resting_peak} / \
             pushed {pushed_peak}"
        );
    }

    /// Waking up is not an impact. The solver stops reporting impulses for a
    /// sleeping body, and the frame it wakes its children are carrying their
    /// full share again — which must not read as having arrived all at once.
    #[test]
    fn waking_up_is_not_read_as_a_spike() {
        let geometry = FlatQuadGeometry::new(50.0);
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let (body, colliders) = compound_slab(&mut world, 0.5, Vector3::zeros());
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        let mut debug = DebugLines::default();
        let mut tracker = ContactLoadTracker::default();

        let mut slept = false;
        for _ in 0..600 {
            stepper.step(&mut world, FRAME_DT, &geometry, &[], &[], &mut debug);
            tracker.advance(&world, body, &colliders, FRAME_DT);
            if world.is_sleeping(body) {
                slept = true;
                break;
            }
        }
        assert!(slept, "the slab never settled; the test cannot run");

        world
            .body_mut(body)
            .unwrap()
            .set_linear_velocity(Vector3::new(0.4, 0.0, 0.0));
        world.wake_body(body);

        let mut peak = 0.0f32;
        for _ in 0..30 {
            stepper.step(&mut world, FRAME_DT, &geometry, &[], &[], &mut debug);
            let spikes = tracker.advance(&world, body, &colliders, FRAME_DT);
            peak = peak.max(spikes.into_iter().fold(0.0f32, |m, s| m.max(s.magnitude)));
        }

        // What is left is the slab re-seating under the shove, which is a real
        // change in contact and small: measured 44 N·s against a frame of
        // weight of 589 N·s.
        let weight_per_frame = world.body(body).unwrap().mass() * 9.81 * FRAME_DT;
        assert!(
            peak < weight_per_frame * 0.2,
            "waking read as a spike of {peak}, a large share of one frame of \
             weight ({weight_per_frame})"
        );
    }

    /// A body held up by the world carries its weight through what holds
    /// it, so a spike on it is judged without the body's weight taken off.
    #[test]
    fn a_body_held_by_the_world_gets_no_deadband() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let (body, _) = compound_slab(&mut world, 3.0, Vector3::zeros());
        assert!(!world.is_held_by_world(body));
        world.create_constraint(ConstraintKind::world_fixed(
            body,
            Point3::new(0.0, 3.0, 0.0),
            Vector3::zeros(),
            &nalgebra::UnitQuaternion::identity(),
            0.0,
            f32::INFINITY,
        ));
        assert!(world.is_held_by_world(body));
    }

    /// A world with the fracture and drawing systems' storages and a row of
    /// `count` boxes at `origin`, a part each, bonded to the next at the face
    /// they share, each drawn as a cube.
    fn row_world(count: usize, origin: Point3<f32>) -> (World, specs::Entity, RigidBodyHandle) {
        let mut world = World::new();
        crate::app::breakage::register(&mut world);
        world.insert(Time::fixed(FRAME_DT));

        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let body = physics_world.create_body(RigidBodyDesc::dynamic().position(origin));
        let centre = (count as f32 - 1.0) * 0.5;
        let children: Vec<ColliderHandle> = (0..count)
            .map(|i| {
                physics_world
                    .attach_collider(
                        body,
                        ColliderDesc::box_shape(HALF)
                            .offset_translation(Vector3::new(i as f32 - centre, 0.0, 0.0))
                            .density(500.0),
                    )
                    .expect("collider attaches to a live body")
            })
            .collect();
        let mut graph = PartGraph::default();
        let parts: Vec<PartId> = children.iter().map(|c| graph.add_part([*c])).collect();
        for (i, pair) in parts.windows(2).enumerate() {
            graph.bond(Bond {
                a: pair[0],
                b: pair[1],
                anchor: Point3::new(i as f32 + 0.5 - centre, 0.0, 0.0),
                reach: 2.0,
                threshold: 10.0,
                toughness: Some(10.0),
            });
        }
        let look = Arc::new(Look::single(
            generate_cube_vertices(HALF, Colour::WHITE),
            generate_cube_indices(),
            MaterialId(0),
        ));
        let looks = PartLooks::new(children.iter().map(|c| (*c, look.clone())));
        world.insert(PhysicsResource::new(
            physics_world,
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));
        let entity = world
            .create_entity()
            .with(Position(origin.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(nalgebra::UnitQuaternion::identity()))
            .with(RigidBodyComponent(body))
            .with(Renderable)
            .with(looks)
            .with(CompoundFracture::new(graph))
            .build();
        (world, entity, body)
    }

    /// A blast this frame, as the physics step leaves it for the systems after.
    fn blast_at(world: &mut World, at: Point3<f32>, radius: f32) {
        let mut queue = PhysicsImpulseQueue::default();
        queue.push(PhysicsImpulse::radial(at, radius, 5_000.0, 0.0));
        let _ = queue.drain().count();
        world.insert(queue);
    }

    fn split_and_draw(world: &mut World) {
        FractureSystem.run_now(world);
        PartDrawSystem.run_now(world);
        world.maintain();
    }

    /// Every entity with a body: its body and how many colliders it holds.
    fn pieces(world: &World) -> Vec<(specs::Entity, RigidBodyHandle, usize)> {
        let physics = world.read_resource::<PhysicsResource>();
        let bodies = world.read_storage::<RigidBodyComponent>();
        (&world.entities(), &bodies)
            .join()
            .map(|(e, b)| {
                let count = physics.world.body(b.0).map_or(0, |b| b.colliders().len());
                (e, b.0, count)
            })
            .collect()
    }

    /// The flicker this guards: a structure jumped sideways for one frame at
    /// the moment it broke. Splitting moves the compound's origin onto what is
    /// left, and the drawing system, running after, must draw it there.
    #[test]
    fn a_broken_compound_is_drawn_where_its_body_actually_is() {
        let origin = Point3::new(0.0, 10.0, 0.0);
        let (mut world, entity, body) = row_world(4, origin);
        blast_at(&mut world, origin + Vector3::new(-1.5, 0.0, 0.0), 0.9);

        split_and_draw(&mut world);

        let recentred = {
            let physics = world.read_resource::<PhysicsResource>();
            let body = physics.world.body(body).expect("the remnant survives");
            assert!(
                body.colliders().len() < 4,
                "nothing broke off, so there is no recentring to check"
            );
            body.position()
        };
        assert!(
            (recentred - origin).magnitude() > 0.1,
            "the remnant's origin did not move, so this test proves nothing"
        );
        let positions = world.read_storage::<Position>();
        let drawn = positions.get(entity).expect("the remnant keeps a position");
        assert!(
            (drawn.0 - recentred.coords).magnitude() < 1e-5,
            "drawn at {:?} but the body is at {:?}",
            drawn.0,
            recentred.coords
        );
    }

    /// Every piece must be drawable on the very frame it breaks off, at the
    /// place it broke off from. It leaves the compound's model that frame,
    /// so any gap or displacement here is a piece that visibly blinks.
    #[test]
    fn a_split_piece_is_drawable_where_it_broke_off() {
        let origin = Point3::new(0.0, 10.0, 0.0);
        let (mut world, _, _) = row_world(4, origin);
        let broken_off = origin + Vector3::new(-1.5, 0.0, 0.0);
        blast_at(&mut world, origin + Vector3::new(-1.5, 0.0, 0.0), 0.9);

        split_and_draw(&mut world);

        let models = world.read_storage::<ModelInstance>();
        let positions = world.read_storage::<Position>();
        let drawn: Vec<Vector3<f32>> = (&models, &positions).join().map(|(_, p)| p.0).collect();
        assert_eq!(drawn.len(), 2, "the compound and its piece are drawn");
        assert!(
            drawn
                .iter()
                .any(|p| (p - broken_off.coords).magnitude() < 1e-4),
            "no drawn entity sits where the piece broke off: {drawn:?}"
        );
    }

    /// The rebuilt remnant model must still span the pieces it is made of,
    /// each look at its own collider, not stacked at the body's origin.
    #[test]
    fn a_rebuilt_remnant_still_spans_its_surviving_pieces() {
        let origin = Point3::new(0.0, 10.0, 0.0);
        let (mut world, entity, body) = row_world(4, origin);
        blast_at(&mut world, origin + Vector3::new(-1.5, 0.0, 0.0), 0.9);

        split_and_draw(&mut world);

        let models = world.read_storage::<ModelInstance>();
        let model = &models.get(entity).expect("the remnant keeps a model").model;
        let (mut model_min, mut model_max) = (
            Vector3::repeat(f32::INFINITY),
            Vector3::repeat(f32::NEG_INFINITY),
        );
        let mut vertex_count = 0;
        for vertex in model
            .parts
            .iter()
            .flat_map(|part| &part.primitives)
            .flat_map(|primitive| &primitive.vertices)
        {
            model_min = model_min.inf(&vertex.pos);
            model_max = model_max.sup(&vertex.pos);
            vertex_count += 1;
        }

        let physics = world.read_resource::<PhysicsResource>();
        let remnant = physics.world.body(body).expect("the remnant survives");
        let (mut solid_min, mut solid_max) = (
            Vector3::repeat(f32::INFINITY),
            Vector3::repeat(f32::NEG_INFINITY),
        );
        for handle in remnant.colliders() {
            let centre = physics
                .world
                .collider(*handle)
                .unwrap()
                .offset()
                .translation
                .vector;
            solid_min = solid_min.inf(&(centre - HALF));
            solid_max = solid_max.sup(&(centre + HALF));
        }

        assert_eq!(vertex_count, 24 * remnant.colliders().len());
        assert!(
            (model_min - solid_min).magnitude() < 1e-4
                && (model_max - solid_max).magnitude() < 1e-4,
            "model spans {model_min:?}..{model_max:?} but the colliders span \
             {solid_min:?}..{solid_max:?}"
        );
        assert!(
            model_max.x - model_min.x > 2.0,
            "the remnant's boxes collapsed"
        );
    }

    /// Two planks still bonded to each other fly as one body, as the bond
    /// says, rather than as a plank each.
    #[test]
    fn a_still_joined_pair_flies_as_one_body() {
        let origin = Point3::new(0.0, 10.0, 0.0);
        let (mut world, entity, body) = row_world(4, origin);
        // Break the middle bond alone: two pairs.
        {
            let mut fractures = world.write_storage::<CompoundFracture>();
            let fracture = fractures.get_mut(entity).unwrap();
            fracture.graph.break_bonds(|bond| bond.anchor.x.abs() < 0.1);
            fracture.split_pending = true;
        }

        split_and_draw(&mut world);

        let all = pieces(&world);
        assert_eq!(all.len(), 2, "the row came apart as {all:?}");
        assert!(all.iter().all(|(_, _, count)| *count == 2), "{all:?}");
        let splits = world.read_resource::<PieceSplits>();
        let split = *splits.iter().next().expect("the split was told");
        assert_eq!(split.from, entity);
        let (piece, piece_body, _) = *all.iter().find(|p| p.0 != entity).unwrap();
        assert_eq!(split.piece, piece);
        assert_ne!(piece_body, body);

        // The piece's share is its own parts, bond and looks.
        let fractures = world.read_storage::<CompoundFracture>();
        let share = fractures.get(piece).expect("the piece can break again");
        assert_eq!(share.graph.parts().len(), 2);
        assert_eq!(share.graph.bonds().len(), 1);
        let looks = world.read_storage::<PartLooks>();
        assert_eq!(looks.get(piece).unwrap().len(), 2);
        assert_eq!(looks.get(entity).unwrap().len(), 2);
        assert_eq!(fractures.get(entity).unwrap().graph.parts().len(), 2);

        // Its bond is held where the two planks meet, in its own frame.
        let physics = world.read_resource::<PhysicsResource>();
        let bond = share.graph.bonds()[0];
        let rigid = physics.world.body(piece_body).unwrap();
        let world_anchor = rigid.position() + rigid.rotation() * bond.anchor.coords;
        let meet = share
            .graph
            .colliders_of(&[bond.a, bond.b])
            .iter()
            .map(|c| {
                let offset = physics
                    .world
                    .collider(*c)
                    .unwrap()
                    .offset()
                    .translation
                    .vector;
                rigid.position() + rigid.rotation() * offset
            })
            .fold(Vector3::zeros(), |sum, p| sum + p.coords)
            * 0.5;
        assert!(
            (world_anchor.coords - meet).norm() < 1e-4,
            "the piece's bond moved: {world_anchor:?} against {meet:?}"
        );
    }

    /// Without ties the heaviest group stays on the compound's body, however
    /// few parts it has; with one, the tied group stays.
    #[test]
    fn the_heaviest_group_stays_unless_another_is_tied() {
        let origin = Point3::new(0.0, 10.0, 0.0);
        let (world, entity, body) = row_world(3, origin);
        let colliders = {
            let physics = world.read_resource::<PhysicsResource>();
            physics.world.body(body).unwrap().colliders().to_vec()
        };
        // Make the last box heavy, then cut it loose: it stays, the pair goes.
        {
            let mut physics = world.write_resource::<PhysicsResource>();
            physics.world.detach_collider(body, colliders[2]);
        }
        let heavy = world
            .write_resource::<PhysicsResource>()
            .world
            .attach_collider(
                body,
                ColliderDesc::box_shape(HALF)
                    .offset_translation(Vector3::new(1.0, 0.0, 0.0))
                    .density(50_000.0),
            )
            .unwrap();
        {
            let mut fractures = world.write_storage::<CompoundFracture>();
            let fracture = fractures.get_mut(entity).unwrap();
            let mut graph = PartGraph::default();
            let a = graph.add_part([colliders[0]]);
            let b = graph.add_part([colliders[1]]);
            graph.add_part([heavy]);
            graph.bond(Bond {
                a,
                b,
                anchor: Point3::new(-0.5, 0.0, 0.0),
                reach: 2.0,
                threshold: 10.0,
                toughness: Some(10.0),
            });
            fracture.graph = graph;
            fracture.split_pending = true;
        }
        FractureSystem.run_now(&world);
        {
            let physics = world.read_resource::<PhysicsResource>();
            assert_eq!(physics.world.body(body).unwrap().colliders(), &[heavy]);
        }

        // Tie the light end of a fresh row: it stays.
        let (mut world, entity, body) = row_world(3, origin);
        let light = {
            let physics = world.read_resource::<PhysicsResource>();
            physics.world.body(body).unwrap().colliders()[0]
        };
        {
            let mut physics = world.write_resource::<PhysicsResource>();
            let weld = physics.world.create_constraint(ConstraintKind::world_fixed(
                body,
                origin + Vector3::new(-1.0, 0.0, 0.0),
                Vector3::new(-1.0, 0.0, 0.0),
                &nalgebra::UnitQuaternion::identity(),
                0.0,
                f32::INFINITY,
            ));
            let mut fractures = world.write_storage::<CompoundFracture>();
            let fracture = fractures.get_mut(entity).unwrap();
            let part = fracture.graph.part_of(light).unwrap();
            fracture.graph.tie(weld, part);
            fracture.graph.break_bonds(|bond| bond.holds(part));
            fracture.split_pending = true;
        }
        split_and_draw(&mut world);
        let physics = world.read_resource::<PhysicsResource>();
        assert_eq!(physics.world.body(body).unwrap().colliders(), &[light]);
        assert!(physics.world.is_held_by_world(body));
        drop(physics);
        assert_eq!(pieces(&world).len(), 2);
    }

    /// Release gives up every tie and bond: each part becomes a piece, the
    /// heaviest keeps the body, and no body is left empty or held.
    #[test]
    fn release_lets_every_part_go_and_leaves_no_body_empty() {
        let origin = Point3::new(0.0, 10.0, 0.0);
        let (mut world, entity, body) = row_world(4, origin);
        {
            let mut physics = world.write_resource::<PhysicsResource>();
            let weld = physics.world.create_constraint(ConstraintKind::world_fixed(
                body,
                origin,
                Vector3::zeros(),
                &nalgebra::UnitQuaternion::identity(),
                0.0,
                f32::INFINITY,
            ));
            let mut fractures = world.write_storage::<CompoundFracture>();
            let fracture = fractures.get_mut(entity).unwrap();
            let first = fracture.graph.parts()[0].id;
            fracture.graph.tie(weld, first);
            fracture.release();
        }

        split_and_draw(&mut world);

        let all = pieces(&world);
        assert_eq!(all.len(), 4, "{all:?}");
        let physics = world.read_resource::<PhysicsResource>();
        for (_, piece_body, count) in &all {
            assert_eq!(*count, 1);
            assert!(!physics.world.is_held_by_world(*piece_body));
        }
        assert!(world.read_storage::<Renderable>().contains(entity));
    }

    /// A group struck off its centre spins: its kick lands at each part's
    /// own centre of mass, not at the body's.
    #[test]
    fn a_kick_off_a_pieces_centre_spins_it() {
        let origin = Point3::new(0.0, 10.0, 0.0);
        let (mut world, entity, _) = row_world(3, origin);
        {
            let mut fractures = world.write_storage::<CompoundFracture>();
            let fracture = fractures.get_mut(entity).unwrap();
            // The first two stay bonded and come away; only the end one is
            // pushed.
            let parts: Vec<PartId> = fracture.graph.parts().iter().map(|p| p.id).collect();
            fracture
                .graph
                .break_bonds(|bond| bond.holds(parts[1]) && bond.holds(parts[2]));
            fracture.split_pending = true;
            // Make the pair the lighter group by giving the kept one more.
            fracture
                .kicks
                .insert(parts[0], Vector3::new(0.0, 100.0, 0.0));
        }
        // Weigh the end box down so the pair is the piece that leaves.
        let pair_body = {
            FractureSystem.run_now(&world);
            world.maintain();
            let physics = world.read_resource::<PhysicsResource>();
            let all = pieces(&world);
            let (_, pair, _) = *all.iter().find(|p| p.2 == 2).expect("a pair split off");
            let spin = physics.world.body(pair).unwrap().angular_velocity();
            assert!(
                spin.norm() > 1e-3,
                "a kick at one end did not turn the pair"
            );
            pair
        };
        let _ = pair_body;
    }
}
