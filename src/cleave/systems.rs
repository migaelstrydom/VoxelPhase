//! The systems that turn blows on a brittle solid into cleaved wedges, and
//! give a piece split off a solid its share of it.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Translation3, Vector3};
use specs::{Join, Read, ReadStorage, System, WriteStorage};

use super::components::{BlowMeasure, BrittleSolid};
use crate::components::RigidBodyComponent;
use crate::debug::DebugLog;
use crate::fracture::{replace_part, Bond, CompoundFracture, PieceSplits};
use crate::model::PartLooks;
use crate::physics::{ColliderHandle, PhysicsImpulse, PhysicsImpulseQueue, RigidBodyHandle};
use crate::systems::PhysicsResource;
use crate::time::Time;

/// Fastest a freed wedge leaves, in m/s.
///
/// The spike a fixed block records is the whole momentum of whatever hit it,
/// and handing all of that to a wedge would fire it like a bullet.
const MAX_PIECE_SPEED: f32 = 6.0;

/// How much each wedge is shrunk about its own centre, as a fraction, so
/// that it stands back from the surfaces it was cut along.
///
/// Two pieces that share a face exactly are two colliders in contact from the
/// moment one comes free, and a body that begins a sweep already touching its
/// neighbour is clamped at time zero by continuous collision detection: it
/// hangs in the air with its velocity climbing. A hairline of clearance means
/// no wedge ever starts out touching another, and reads as the crack.
///
/// Shrunk rather than moved apart: a wedge pushed off its own centre of
/// volume would take the body's centre of mass with it.
const CLEAVE_INSET: f32 = 0.004;

/// Cleaves brittle blocks where they are struck, and hands the results to the
/// fracture system.
///
/// Runs after the physics step and *before* `FractureSystem`, so that a blow
/// on a block becomes wedges before the bonds are judged: the alternative is
/// a whole block leaving the wall it was part of, intact.
pub struct SolidCleaveSystem;

impl<'a> System<'a> for SolidCleaveSystem {
    #[allow(clippy::type_complexity)]
    type SystemData = (
        specs::Write<'a, PhysicsResource>,
        WriteStorage<'a, BrittleSolid>,
        WriteStorage<'a, CompoundFracture>,
        WriteStorage<'a, PartLooks>,
        ReadStorage<'a, RigidBodyComponent>,
        Read<'a, PhysicsImpulseQueue>,
        Read<'a, Time>,
        specs::Write<'a, DebugLog>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut physics,
            mut solids,
            mut fractures,
            mut looks,
            bodies,
            impulse_queue,
            time,
            mut debug_log,
        ) = data;
        let dt = time.delta_seconds();
        let blasts = impulse_queue.last_impulses();
        let mut loudest = 0.0f32;

        for (solid, fracture, looks, body_comp) in
            (&mut solids, &mut fractures, &mut looks, &bodies).join()
        {
            let body_handle = body_comp.0;
            let hits = gather_hits(&physics, solid, body_handle, blasts, dt, &mut loudest);
            for hit in hits {
                cleave(&mut physics, solid, fracture, looks, body_handle, &hit);
            }
        }

        debug_log.add("Cleave/MaxSpike", format!("{loudest:.1} N·s"));
    }
}

/// Gives a piece split off a brittle solid its share of it, so its wedges
/// can cleave and its blocks break as the whole did; and leaves the solid the
/// records of what it kept.
///
/// Runs after `FractureSystem`, which sends the splits.
pub struct SolidShareSystem;

impl<'a> System<'a> for SolidShareSystem {
    type SystemData = (
        Read<'a, PieceSplits>,
        WriteStorage<'a, BrittleSolid>,
        ReadStorage<'a, RigidBodyComponent>,
        Read<'a, PhysicsResource>,
    );

    fn run(&mut self, (splits, mut solids, bodies, physics): Self::SystemData) {
        if splits.is_empty() {
            return;
        }
        let children = |entity| -> Option<Vec<ColliderHandle>> {
            let body = bodies.get(entity)?.0;
            Some(physics.world.body(body)?.colliders().to_vec())
        };

        let mut sources = Vec::new();
        for split in splits.iter() {
            let Some(solid) = solids.get(split.from) else {
                continue;
            };
            if !sources.contains(&split.from) {
                sources.push(split.from);
            }
            let Some(children) = children(split.piece) else {
                continue;
            };
            if solid.worth_sharing(&children) {
                let share = solid.share(&children);
                let _ = solids.insert(split.piece, share);
            }
        }
        for source in sources {
            if let (Some(solid), Some(children)) = (solids.get_mut(source), children(source)) {
                solid.retain(&children);
            }
        }
    }
}

/// One blow on one block.
struct Hit {
    child: ColliderHandle,
    /// Where it landed, in world space.
    point: Point3<f32>,
    /// The impulse the freed wedge leaves with, in world space.
    kick: Vector3<f32>,
}

/// Read this frame's loads and pick out the blows that break a block.
///
/// Reads every tracker every frame whether or not anything happens: a
/// baseline that stops advancing turns a quiet load into a spike later.
fn gather_hits(
    physics: &PhysicsResource,
    solid: &mut BrittleSolid,
    body_handle: RigidBodyHandle,
    blasts: &[PhysicsImpulse],
    dt: f32,
    loudest: &mut f32,
) -> Vec<Hit> {
    let world = &physics.world;
    let Some(body) = world.body(body_handle) else {
        return Vec::new();
    };
    let body_pos = body.position();
    let body_rot = body.rotation();
    let handles: Vec<ColliderHandle> = body.colliders().to_vec();
    let spikes = solid.contact_load.advance(world, body_handle, &handles, dt);
    let threshold = solid.cleaving.threshold;

    // Both measures advance every frame, whichever is judged by, so that
    // neither's baseline goes stale.
    let gravity = world.config().gravity * body.gravity_scale() * dt;
    let arrest = if world.is_sleeping(body_handle) {
        solid.arrest(Vector3::zeros(), Vector3::zeros(), body.mass())
    } else {
        solid.arrest(body.linear_velocity(), gravity, body.mass())
    };
    // An arrest is the whole body's; it lands on the child that took the
    // biggest share of the contact.
    let struck = handles
        .iter()
        .zip(&spikes)
        .max_by(|a, b| a.1.magnitude.total_cmp(&b.1.magnitude))
        .map(|(handle, _)| *handle);

    let mut hits = Vec::new();
    for (handle, spike) in handles.iter().zip(&spikes) {
        if !solid.is_brittle(*handle) {
            continue;
        }
        let blow = match solid.measure {
            BlowMeasure::ContactSpike => spike.magnitude,
            BlowMeasure::Arrest if struck == Some(*handle) => arrest,
            BlowMeasure::Arrest => 0.0,
        };
        *loudest = loudest.max(blow);

        if blow > threshold {
            let impact = world.impacts().for_collider(*handle);
            hits.push(Hit {
                child: *handle,
                point: impact.map_or(body_pos, |i| i.centre),
                kick: impact.map_or(Vector3::zeros(), |i| i.normal) * blow,
            });
            continue;
        }

        // A blast breaks a block wherever it reaches it. The fracture system,
        // reading the same blasts this frame, decides which of the new wedges
        // it blows out.
        let Some(collider) = world.collider(*handle) else {
            continue;
        };
        let centre = Point3::from(
            collider
                .world_transform(body_pos, body_rot)
                .translation
                .vector,
        );
        let strongest = blasts
            .iter()
            .filter_map(|blast| {
                blast
                    .loading_at(centre)
                    .map(|loading| (loading.per_area.magnitude(), blast))
            })
            .max_by(|a, b| a.0.total_cmp(&b.0));
        if let Some((magnitude, PhysicsImpulse::Radial { center, .. })) = strongest {
            *loudest = loudest.max(magnitude);
            if magnitude > threshold {
                hits.push(Hit {
                    child: *handle,
                    point: *center,
                    kick: Vector3::zeros(),
                });
            }
        }
    }
    hits
}

/// Replace the struck block with the wedges it cleaves into, or knock it out
/// whole if it is too small or already as broken as this object goes.
///
/// The wedges are bonded to each other; each of the block's own bonds goes
/// to the wedge nearest it.
fn cleave(
    physics: &mut PhysicsResource,
    solid: &mut BrittleSolid,
    fracture: &mut CompoundFracture,
    looks: &mut PartLooks,
    body_handle: RigidBodyHandle,
    hit: &Hit,
) {
    let world = &physics.world;
    let Some(body) = world.body(body_handle) else {
        return;
    };
    let Some(part) = fracture.graph.part_of(hit.child) else {
        return;
    };
    // Its place in the body's list, which seeds the cut and nothing else.
    let Some(place) = body.colliders().iter().position(|h| *h == hit.child) else {
        // Already replaced by an earlier hit this frame.
        return;
    };
    let Some(collider) = world.collider(hit.child) else {
        return;
    };
    let depth = solid.depths.of(hit.child);
    let offset = *collider.offset();

    // The blow, in the block's own frame: through the body's rotation, then
    // through the block's own placement within it. Igloo blocks are laid at
    // an angle to the dome they are part of, and a cut planned in the body's
    // frame would run across them all the same way.
    let in_body = body
        .rotation()
        .inverse_transform_vector(&(hit.point - body.position()));
    let in_block = offset
        .rotation
        .inverse_transform_vector(&(in_body - offset.translation.vector));
    let salt = in_block.x.to_bits() ^ in_block.y.to_bits().rotate_left(11) ^ (place as u32);

    let pieces = solid
        .may_cleave(hit.child)
        .then(|| solid.cleaving.cleave(collider.shape(), in_block, salt))
        .flatten();
    let Some(pieces) = pieces else {
        // A wedge, not a block. It comes out whole.
        fracture.sever(part, capped_kick(hit.kick, collider.mass(), 1));
        return;
    };

    // The wedge the blow lands on comes away at once, so it inherits none of
    // what the block held: the wedges that stay hold it.
    let struck_index = nearest_piece(&pieces, in_block);
    let mut wedges = solid.blocks.pieces(
        hit.child,
        &offset,
        pieces.iter().map(|piece| {
            let at = offset.translation.vector + offset.rotation * piece.centre;
            (
                Arc::new(piece.hull.scaled(1.0 - CLEAVE_INSET)),
                Isometry3::from_parts(Translation3::from(at), offset.rotation),
            )
        }),
    );
    if let Some(wedge) = struck_index.and_then(|index| wedges.get_mut(index)) {
        wedge.inherits = false;
    }
    let Some(made) = replace_part(physics, fracture, looks, body_handle, part, wedges) else {
        return;
    };
    solid.blocks.record(hit.child, &made);
    for wedge in &made {
        solid.depths.set(wedge.collider, depth + 1);
    }

    // The wedges are keyed to each other where they meet.
    for (k, first) in made.iter().enumerate() {
        for second in &made[k + 1..] {
            let (a, b) = (&pieces[first.index], &pieces[second.index]);
            let middle = (a.centre + b.centre) * 0.5;
            fracture.graph.bond(Bond {
                a: first.part,
                b: second.part,
                anchor: Point3::from(offset.translation.vector + offset.rotation * middle),
                reach: 2.0 * a.hull.bounding_radius.min(b.hull.bounding_radius),
                threshold: solid.joint_threshold,
                toughness: Some(solid.joint_toughness),
            });
        }
    }

    // The wedge the blow landed on comes away; the rest hold until their own
    // bonds give. Without this a blow that breaks a block leaves every piece
    // of it exactly where it was, and nothing looks broken.
    if let Some(struck) = made.iter().find(|wedge| Some(wedge.index) == struck_index) {
        let mass = physics
            .world
            .collider(struck.collider)
            .map_or(0.0, |c| c.mass());
        fracture.sever(struck.part, capped_kick(hit.kick, mass, 1));
    }
}

/// Which of `pieces` has its centre nearest the blow.
fn nearest_piece(pieces: &[super::plan::CleavePiece], hit: Vector3<f32>) -> Option<usize> {
    (0..pieces.len()).min_by(|a, b| {
        let distance = |index: &usize| (pieces[*index].centre - hit).magnitude_squared();
        distance(a).total_cmp(&distance(b))
    })
}

/// A wedge's share of the blow, held to `MAX_PIECE_SPEED`.
fn capped_kick(kick: Vector3<f32>, mass: f32, shares: usize) -> Vector3<f32> {
    let share = kick / shares.max(1) as f32;
    let limit = mass * MAX_PIECE_SPEED;
    let magnitude = share.magnitude();
    if magnitude > limit && magnitude > 0.0 {
        share * (limit / magnitude)
    } else {
        share
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;
    use specs::{Entity, RunNow, World, WorldExt};

    use super::super::plan::CleaveRule;
    use crate::app::breakage;
    use crate::app::spawnables::shared::models::texture_seam;
    use crate::app::spawnables::{ice_cleaving, ice_texture_spread, ice_uvs, IceBlock};
    use crate::debug::DebugLines;
    use crate::fracture::{ContactLoadTracker, PartGraph};
    use crate::fracture::{FractureSystem, PieceSplits};
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{
        ColliderDesc, ConstraintKind, PhysicsConfig, PhysicsWorld, RigidBodyDesc,
    };
    use crate::rendering::material::MaterialId;
    use crate::systems::PartDrawSystem;

    const FRAME_DT: f32 = 1.0 / 60.0;
    const BLOCK_HEIGHT: f32 = 1.0;
    const HALF: Vector3<f32> = Vector3::new(0.3, 0.15, 0.2);

    /// A world with the break pipeline and no sleep: a body set down at rest
    /// is put to sleep on its first frame, and a sleeping body loads nothing.
    fn pipeline_world() -> World {
        let mut world = World::new();
        breakage::register(&mut world);
        world.insert(Time::fixed(FRAME_DT));
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        world.insert(PhysicsResource::new(
            PhysicsWorld::new(config),
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));
        world
    }

    /// One block of ice at `BLOCK_HEIGHT`, as a compound of one, welded to
    /// the world where it stands, with a floor far below.
    fn ice_block(threshold: f32) -> (World, Entity, RigidBodyHandle) {
        let mut world = pipeline_world();
        let mut block = IceBlock::new(
            Point3::new(0.0, BLOCK_HEIGHT, 0.0),
            HALF,
            MaterialId(0),
            ice_texture_spread(HALF),
        );
        block.cleaving = CleaveRule {
            threshold,
            ..CleaveRule::default()
        };
        let entity = block.spawn(&mut world);
        let body = world
            .read_storage::<RigidBodyComponent>()
            .get(entity)
            .unwrap()
            .0;
        let weld = world
            .write_resource::<PhysicsResource>()
            .world
            .create_constraint(ConstraintKind::world_fixed(
                body,
                Point3::new(0.0, BLOCK_HEIGHT, 0.0),
                Vector3::zeros(),
                &UnitQuaternion::identity(),
                0.0,
                f32::INFINITY,
            ));
        {
            let mut fractures = world.write_storage::<CompoundFracture>();
            let fracture = fractures.get_mut(entity).unwrap();
            let block = fracture.graph.parts()[0].id;
            fracture.graph.tie(weld, block);
        }
        (world, entity, body)
    }

    /// A box of `mass` kg dropped from `drop` metres onto the block.
    fn drop_box(world: &mut World, x: f32, drop: f32, mass: f32) -> RigidBodyHandle {
        let half = 0.1;
        let density = mass / (8.0 * half * half * half);
        let mut physics = world.write_resource::<PhysicsResource>();
        let body = physics
            .world
            .create_body(RigidBodyDesc::dynamic().position(Point3::new(
                x,
                BLOCK_HEIGHT + HALF.y + half + drop,
                0.0,
            )));
        physics.world.attach_collider(
            body,
            ColliderDesc::box_shape(Vector3::repeat(half))
                .density(density)
                .restitution(0.0),
        );
        body
    }

    fn step_physics(
        world: &mut World,
        stepper: &mut SequentialStepper,
        geometry: &FlatQuadGeometry,
    ) {
        let mut physics = world.write_resource::<PhysicsResource>();
        let mut debug = DebugLines::default();
        stepper.step(&mut physics.world, FRAME_DT, geometry, &[], &[], &mut debug);
    }

    /// One whole game frame: physics, then the break pipeline.
    fn frame(world: &mut World, stepper: &mut SequentialStepper, geometry: &FlatQuadGeometry) {
        step_physics(world, stepper, geometry);
        breakage::run(world);
    }

    /// A frame without the fracture system, which splits the struck wedge off
    /// the same frame the block breaks. Tests that ask what the *break* did
    /// stop here; tests that ask what the object looks like afterwards do not.
    fn step(world: &mut World, stepper: &mut SequentialStepper, geometry: &FlatQuadGeometry) {
        step_physics(world, stepper, geometry);
        SolidCleaveSystem.run_now(world);
        world.maintain();
    }

    fn child_count(world: &World, body: RigidBodyHandle) -> usize {
        let physics = world.read_resource::<PhysicsResource>();
        physics.world.body(body).map_or(0, |b| b.colliders().len())
    }

    /// Three blocks in a row, bonded end to end, as a course of a wall or a
    /// dome is. The middle one is the one that gets hit.
    fn row_of_three() -> (World, Entity, RigidBodyHandle) {
        let (world, entity, body) = ice_block(20.0);
        let (middle, sides) = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let middle = physics.world.body(body).expect("the block").colliders()[0];
            let sides: Vec<(ColliderHandle, f32)> = [-1.0, 1.0]
                .into_iter()
                .filter_map(|side| {
                    let collider = physics.world.attach_collider(
                        body,
                        ColliderDesc::box_shape(HALF)
                            .density(900.0)
                            .friction(0.05)
                            .offset_translation(Vector3::new(
                                side * (HALF.x * 2.0 + 0.01),
                                0.0,
                                0.0,
                            )),
                    )?;
                    Some((collider, side))
                })
                .collect();
            (middle, sides)
        };
        let mut fractures = world.write_storage::<CompoundFracture>();
        let fracture = fractures.get_mut(entity).expect("the row");
        let ties = fracture.graph.ties().to_vec();
        let mut graph = PartGraph::default();
        let middle_part = graph.add_part([middle]);
        for tie in ties {
            graph.tie(tie.constraint, middle_part);
        }
        for (side, sign) in sides {
            let part = graph.add_part([side]);
            graph.bond(Bond {
                a: middle_part,
                b: part,
                anchor: Point3::new(sign * (HALF.x + 0.005), 0.0, 0.0),
                reach: 2.0 * HALF.norm(),
                threshold: 1.0e9,
                toughness: Some(1.0e9),
            });
        }
        fracture.graph = graph;
        drop(fractures);
        let mut solids = world.write_storage::<BrittleSolid>();
        let solid = solids.get_mut(entity).expect("the row");
        solid.contact_load = ContactLoadTracker::default();
        drop(solids);
        (world, entity, body)
    }

    /// A block that cleaves in the middle of a structure must not take the
    /// structure apart with it: each of its bonds goes to the wedge nearest
    /// it, or everything the block was carrying is suddenly connected to
    /// nothing and the fracture system drops it.
    #[test]
    fn a_cleaved_block_still_holds_what_it_was_holding() {
        let (mut world, entity, body) = row_of_three();
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);

        let _hammer = drop_box(&mut world, 0.0, 1.0, 40.0);
        for _ in 0..30 {
            step(&mut world, &mut stepper, &geometry);
            if child_count(&world, body) > 3 {
                break;
            }
        }
        assert!(
            child_count(&world, body) > 3,
            "the middle block never broke"
        );

        // Both sides are still bonded to a wedge, and the wedges to each
        // other bar the one the blow cut loose.
        let fractures = world.read_storage::<CompoundFracture>();
        let fracture = fractures.get(entity).expect("the row");
        let groups = fracture.graph.groups();
        let biggest = groups.iter().map(Vec::len).max().expect("a structure");
        let parts = fracture.graph.parts().len();
        assert!(
            biggest >= parts - 1,
            "the row fell into {groups:?} of {parts} parts"
        );
        let sides_held = fracture
            .graph
            .bonds()
            .iter()
            .filter(|bond| bond.threshold >= 1.0e9)
            .count();
        assert_eq!(sides_held, 2, "a side lost its hold on the cleaved block");
    }

    /// A block that has already cracked must not hold like one that has not.
    #[test]
    fn a_cracked_block_does_not_hold_like_a_whole_one() {
        let (mut world, entity, block) = ice_block(20.0);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);

        let _hammer = drop_box(&mut world, 0.15, 1.0, 40.0);
        for _ in 0..90 {
            step(&mut world, &mut stepper, &geometry);
            if child_count(&world, block) > 1 {
                break;
            }
        }
        assert!(child_count(&world, block) > 1, "the block never cracked");

        // What holds the wedges to each other must be well under what it took
        // to crack the block. Held at the block's own strength, a struck block
        // sheds the one wedge that was hit and then stands there cracked and
        // rigid, which is what a player sees as a bug.
        let fractures = world.read_storage::<CompoundFracture>();
        let fracture = fractures.get(entity).expect("the block");
        let parts = fracture.graph.parts().len();
        assert!(parts > 1);
        for bond in fracture.graph.bonds() {
            assert!(
                bond.threshold < 20.0,
                "a wedge is held at {}, as strongly as the whole block was",
                bond.threshold
            );
        }
    }

    /// The bond is weak, not absent: a cracked block must not shed its wedges
    /// the instant it cracks, or every break becomes a burst.
    #[test]
    fn a_cracked_block_left_alone_keeps_its_wedges() {
        // At the real ice threshold, not the harness's low one: the bond is a
        // fraction of it, and a fraction of a small number is jitter.
        let (mut world, _, block) = ice_block(ice_cleaving().threshold);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);

        let hammer = drop_box(&mut world, 0.15, 3.0, 400.0);
        for _ in 0..90 {
            frame(&mut world, &mut stepper, &geometry);
            if child_count(&world, block) > 1 {
                break;
            }
        }
        let cracked = child_count(&world, block);
        assert!(cracked > 1, "the block never cracked");

        // Take the hammer away: it is still lying on the block, and a cracked
        // block is *supposed* to shed wedges under a load.
        world
            .write_resource::<PhysicsResource>()
            .world
            .remove_body(hammer);

        for _ in 0..180 {
            frame(&mut world, &mut stepper, &geometry);
        }
        assert_eq!(
            child_count(&world, block),
            cracked,
            "the block shed wedges with nothing touching it"
        );
    }

    /// The whole point: a struck block is a few wedges afterwards, and they
    /// are wedges rather than boxes, each drawn.
    #[test]
    fn a_struck_block_is_replaced_by_the_wedges_it_cleaved_into() {
        let (mut world, entity, block) = ice_block(20.0);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        assert_eq!(child_count(&world, block), 1, "one whole block to start");

        let _hammer = drop_box(&mut world, 0.15, 1.0, 40.0);
        for _ in 0..90 {
            step(&mut world, &mut stepper, &geometry);
            if child_count(&world, block) > 1 {
                break;
            }
        }

        let physics = world.read_resource::<PhysicsResource>();
        let body = physics.world.body(block).expect("the block is still there");
        assert!(
            (2..=3).contains(&body.colliders().len()),
            "a block should break into two or three pieces, not {}",
            body.colliders().len()
        );
        let looks = world.read_storage::<PartLooks>();
        let looks = looks.get(entity).unwrap();
        for handle in body.colliders() {
            let collider = physics.world.collider(*handle).expect("live collider");
            assert!(
                matches!(
                    collider.shape(),
                    crate::physics::ColliderShape::ConvexHull { .. }
                ),
                "a cleaved piece should be a hull, not a box"
            );
            assert!(looks.get(*handle).is_some(), "a wedge has no look");
        }
    }

    /// A block left alone is a block. The cleave system reads every tracker
    /// every frame, and a resting load must never read as a blow.
    #[test]
    fn a_block_nobody_touches_stays_whole() {
        let (mut world, _, block) = ice_block(20.0);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        for _ in 0..120 {
            frame(&mut world, &mut stepper, &geometry);
        }
        assert_eq!(child_count(&world, block), 1);
    }

    /// Cleaving must not move the object: the wedges fill the volume the
    /// block did, so the body's mass and centre of mass are what they were.
    #[test]
    fn the_wedges_weigh_what_the_block_weighed() {
        let (mut world, _, block) = ice_block(20.0);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        let mass_of = |world: &World| -> f32 {
            let physics = world.read_resource::<PhysicsResource>();
            let body = physics.world.body(block).unwrap();
            body.colliders()
                .iter()
                .filter_map(|h| physics.world.collider(*h))
                .map(|c| c.mass())
                .sum()
        };
        let whole = mass_of(&world);

        let _hammer = drop_box(&mut world, 0.0, 1.0, 40.0);
        // The fracture system is left out: it splits the struck wedge off
        // the same frame the block breaks, and the question here is what the
        // cut itself did to the body, not what left it afterwards.
        let mut after = whole;
        for _ in 0..90 {
            step(&mut world, &mut stepper, &geometry);
            if child_count(&world, block) > 1 {
                after = mass_of(&world);
                break;
            }
        }
        assert!(after != whole, "the block never broke");
        assert!(
            (after - whole).abs() < whole * 0.05,
            "the pieces weigh {after} where the block weighed {whole}"
        );
    }

    /// Half-extents of a slab big enough that its texture holds more than one
    /// tile, which is the case where the block's mapping and a mapping fitted
    /// to a piece disagree.
    const SLAB: Vector3<f32> = Vector3::new(2.5, 0.5, 2.5);

    /// A real ice slab, spawned the way a level spawns one, resting on the
    /// ground.
    fn ice_slab() -> (World, Entity) {
        let mut world = pipeline_world();
        let block = IceBlock::new(
            Point3::new(0.0, SLAB.y, 0.0),
            SLAB,
            MaterialId(0),
            ice_texture_spread(SLAB),
        )
        .spawn(&mut world);
        (world, block)
    }

    /// A box of `mass` kg dropped from `drop` metres onto the slab.
    ///
    /// Its own, rather than `drop_box`: a slab five metres across weighs more
    /// than twenty tonnes, and the blow that cracks a block the size of a
    /// crate does not mark it.
    fn slab_hammer(world: &mut World, x: f32, drop: f32, mass: f32) -> RigidBodyHandle {
        let half = 0.25;
        let density = mass / (8.0 * half * half * half);
        let mut physics = world.write_resource::<PhysicsResource>();
        let body = physics
            .world
            .create_body(RigidBodyDesc::dynamic().position(Point3::new(
                x,
                SLAB.y * 2.0 + half + drop,
                0.0,
            )));
        physics.world.attach_collider(
            body,
            ColliderDesc::box_shape(Vector3::repeat(half))
                .density(density)
                .restitution(0.0),
        );
        body
    }

    /// The seam of every piece of ice in the world: each look against where
    /// its collider sits in the slab's texture frame.
    fn seam(world: &World) -> Option<f32> {
        let physics = world.read_resource::<PhysicsResource>();
        let looks = world.read_storage::<PartLooks>();
        let solids = world.read_storage::<BrittleSolid>();
        let bodies = world.read_storage::<RigidBodyComponent>();
        let mut pieces = Vec::new();
        for (looks, solid, body) in (&looks, &solids, &bodies).join() {
            for collider in physics
                .world
                .body(body.0)
                .map_or(&[][..], |b| b.colliders())
            {
                if let (Some(look), Some(at)) =
                    (looks.get(*collider), solid.blocks.texture_at(*collider))
                {
                    pieces.push((look.clone(), at));
                }
            }
        }
        texture_seam(
            pieces.iter().map(|(look, at)| (&**look, *at)),
            ice_uvs(ice_texture_spread(SLAB)),
        )
    }

    /// The whole of the complaint, end to end and through the real objects: a
    /// block that cracks must go on looking like the block it was.
    ///
    /// Not a check that the pattern is at the right scale, which a mesh test
    /// can make on its own — a check that every piece reads the *same* sheet
    /// of ice, at one scale and in one place. A piece textured in its own
    /// frame passes the first and fails this: the frost is the right size and
    /// jumps sideways at the crack, which is what a player sees as the block
    /// changing texture.
    ///
    /// And a piece that comes away looks exactly as it did on the block: it
    /// takes the very look it had, not one redrawn from its shape.
    #[test]
    fn a_cracked_slab_is_still_one_sheet_of_ice() {
        let (mut world, block) = ice_slab();
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        assert_eq!(
            seam(&world).map(|seam| seam < 1e-4),
            Some(true),
            "the whole slab is not one sheet to begin with"
        );

        let _hammer = slab_hammer(&mut world, 0.8, 4.0, 3000.0);
        let body = world
            .read_storage::<RigidBodyComponent>()
            .get(block)
            .expect("the slab has a body")
            .0;
        let spawned_at = world
            .read_resource::<PhysicsResource>()
            .world
            .body(body)
            .unwrap()
            .position();

        // The *whole* break, fracture system included, so that a wedge comes
        // free and the remnant is re-centred on what is left. Re-centring is
        // the step that moves the ground the pattern is projected from, and a
        // test that stops at the cleave never reaches it.
        let mut looks_before_split = Vec::new();
        let mut split = false;
        for _ in 0..240 {
            step(&mut world, &mut stepper, &geometry);
            {
                let looks = world.read_storage::<PartLooks>();
                let physics = world.read_resource::<PhysicsResource>();
                let colliders = physics.world.body(body).unwrap().colliders().to_vec();
                let mine = looks.get(block).unwrap();
                looks_before_split = colliders
                    .iter()
                    .filter_map(|c| Some((*c, mine.get(*c)?.clone())))
                    .collect();
            }
            FractureSystem.run_now(&world);
            SolidShareSystem.run_now(&world);
            PartDrawSystem.run_now(&world);
            world.maintain();
            if !world.read_resource::<PieceSplits>().is_empty() {
                split = true;
                break;
            }
        }
        assert!(split, "nothing ever came off the slab");
        let moved = world
            .read_resource::<PhysicsResource>()
            .world
            .body(body)
            .unwrap()
            .position();
        assert!(
            (moved - spawned_at).norm() > 1e-3,
            "the remnant was never re-centred, so this proves nothing"
        );

        let looks = world.read_storage::<PartLooks>();
        for (collider, before) in &looks_before_split {
            let now = (&looks)
                .join()
                .find_map(|looks| looks.get(*collider))
                .expect("a piece of the slab lost its look");
            assert!(
                Arc::ptr_eq(now, before),
                "a piece was drawn afresh as it came away"
            );
        }
        drop(looks);
        let seam = seam(&world).expect("ice has a projection");
        assert!(
            seam < 1e-4,
            "a piece reads the pattern {seam} from where the block wore it"
        );
    }
}
