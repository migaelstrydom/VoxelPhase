//! The system that turns blows on a brittle solid into cleaved wedges.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use specs::{Join, Read, ReadStorage, System, WriteStorage};

use super::components::BrittleSolid;
use crate::components::{ModelInstance, RigidBodyComponent};
use crate::debug::DebugLog;
use crate::fracture::systems::compound_model_of;
use crate::fracture::{split_child, ChildSubstance, CompoundFracture, FractureJoint};
use crate::physics::{
    ColliderDesc, ColliderHandle, PhysicsImpulse, PhysicsImpulseQueue, RigidBodyHandle,
};
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
/// on a block becomes wedges before the joints are judged: the alternative is
/// a whole block leaving the wall it was part of, intact.
pub struct SolidCleaveSystem;

impl<'a> System<'a> for SolidCleaveSystem {
    type SystemData = (
        specs::Write<'a, PhysicsResource>,
        WriteStorage<'a, BrittleSolid>,
        WriteStorage<'a, CompoundFracture>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, ModelInstance>,
        Read<'a, PhysicsImpulseQueue>,
        Read<'a, Time>,
        specs::Write<'a, DebugLog>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut physics,
            mut solids,
            mut fractures,
            bodies,
            mut models,
            impulse_queue,
            time,
            mut debug_log,
        ) = data;
        let dt = time.delta_seconds();
        let blasts = impulse_queue.last_impulses();
        let mut loudest = 0.0f32;

        for (solid, fracture, body_comp, model) in
            (&mut solids, &mut fractures, &bodies, &mut models).join()
        {
            let body_handle = body_comp.0;
            let hits = gather_hits(
                &physics,
                solid,
                fracture,
                body_handle,
                blasts,
                dt,
                &mut loudest,
            );
            let mut broke = false;
            for hit in hits {
                broke |= cleave(&mut physics, solid, fracture, body_handle, &hit);
            }
            if broke {
                if let Some(rebuilt) =
                    compound_model_of(&physics, body_handle, &fracture.materials, fracture.style)
                {
                    model.model = rebuilt;
                }
            }
        }

        debug_log.add("Cleave/MaxSpike", format!("{loudest:.1} N·s"));
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
    fracture: &CompoundFracture,
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

    let mut hits = Vec::new();
    for (index, handle) in handles.iter().enumerate() {
        if !solid.is_brittle(fracture.material_of(index)) {
            continue;
        }
        let spike = spikes[index];
        *loudest = loudest.max(spike.magnitude);

        if spike.magnitude > threshold {
            let impact = world.impacts().for_collider(*handle);
            hits.push(Hit {
                child: *handle,
                point: impact.map_or(body_pos, |i| i.centre),
                kick: impact.map_or(Vector3::zeros(), |i| i.normal) * spike.magnitude,
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
            .filter_map(|blast| blast.impulse_at(centre).map(|v| (v.magnitude(), blast)))
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
/// Returns whether the body's colliders changed.
fn cleave(
    physics: &mut PhysicsResource,
    solid: &mut BrittleSolid,
    fracture: &mut CompoundFracture,
    body_handle: RigidBodyHandle,
    hit: &Hit,
) -> bool {
    let world = &physics.world;
    let Some(body) = world.body(body_handle) else {
        return false;
    };
    let Some(child) = body.colliders().iter().position(|h| *h == hit.child) else {
        // Already replaced by an earlier hit this frame.
        return false;
    };
    let Some(collider) = world.collider(hit.child) else {
        return false;
    };
    let depth = solid.depths.of(child);
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
    let salt = in_block.x.to_bits() ^ in_block.y.to_bits().rotate_left(11) ^ (child as u32);

    let pieces = solid
        .may_cleave(child)
        .then(|| solid.cleaving.cleave(collider.shape(), in_block, salt))
        .flatten();
    let Some(pieces) = pieces else {
        // A wedge, not a block. It comes out whole.
        fracture.sever(child, capped_kick(hit.kick, collider.mass(), 1));
        return false;
    };

    let Some(substance) = ChildSubstance::of(physics, hit.child) else {
        return false;
    };
    // Whatever held the block holds each of its wedges: working out afresh
    // which neighbour each wedge touches would need the proximity of two
    // rotated boxes, and a wedge that lost its grip on the wall mid-dome
    // would drop the courses above it.
    let inherited: Vec<(usize, f32)> = fracture
        .joints
        .iter()
        .filter_map(|joint| match (joint.child_a, joint.child_b) {
            (a, b) if a == child => Some((b, joint.threshold)),
            (a, b) if b == child => Some((a, joint.threshold)),
            _ => None,
        })
        .collect();

    let descs = pieces.iter().map(|piece| {
        substance.clothe(
            ColliderDesc::convex_hull(Arc::new(piece.hull.scaled(1.0 - CLEAVE_INSET)))
                .offset_translation(offset.translation.vector + offset.rotation * piece.centre)
                .offset_rotation(offset.rotation),
        )
    });
    let Some(split) = split_child(physics, fracture, body_handle, hit.child, descs) else {
        return false;
    };

    solid.contact_load.reindex(&split.order);
    solid.depths.reindex(&split.order);
    for index in &split.pieces {
        fracture.materials[*index] = solid.material;
        solid.depths.set(*index, depth + 1);
    }

    // Every wedge keeps the block's old joints, and is keyed to its fellows.
    let now_at = |old: usize| split.order.iter().position(|o| *o == Some(old));
    for (position, index) in split.pieces.iter().enumerate() {
        for (other, threshold) in &inherited {
            if let Some(other) = now_at(*other) {
                fracture.joints.push(FractureJoint {
                    child_a: *index,
                    child_b: other,
                    threshold: *threshold,
                });
            }
        }
        for fellow in &split.pieces[position + 1..] {
            fracture.joints.push(FractureJoint {
                child_a: *index,
                child_b: *fellow,
                threshold: solid.joint_threshold,
            });
        }
    }

    // The wedge the blow landed on comes away; the rest hold until their own
    // joints give. Without this a blow that breaks a block leaves every piece
    // of it exactly where it was, and nothing looks broken.
    if let Some(struck) = nearest_piece(&pieces, in_block, &split.pieces) {
        let mass = physics
            .world
            .body(body_handle)
            .and_then(|b| b.colliders().get(struck).copied())
            .and_then(|h| physics.world.collider(h))
            .map_or(0.0, |c| c.mass());
        fracture.sever(struck, capped_kick(hit.kick, mass, 1));
    }

    true
}

/// The child index of the wedge whose centre is nearest the blow.
fn nearest_piece(
    pieces: &[super::plan::CleavePiece],
    hit: Vector3<f32>,
    indices: &[usize],
) -> Option<usize> {
    pieces
        .iter()
        .zip(indices)
        .min_by(|a, b| {
            (a.0.centre - hit)
                .magnitude_squared()
                .total_cmp(&(b.0.centre - hit).magnitude_squared())
        })
        .map(|(_, index)| *index)
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
    use specs::{Builder, RunNow, World, WorldExt};

    use super::super::plan::CleaveRule;
    use crate::components::{Orientation, Position, Renderable, Velocity};
    use crate::debug::DebugLines;
    use crate::fracture::FractureSystem;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{PhysicsConfig, PhysicsWorld, RigidBodyDesc};
    use crate::rendering::material::MaterialId;

    const FRAME_DT: f32 = 1.0 / 60.0;
    const BLOCK_HEIGHT: f32 = 1.0;
    const HALF: Vector3<f32> = Vector3::new(0.3, 0.15, 0.2);

    /// One fixed block of ice at `BLOCK_HEIGHT`, as a compound of one, with a
    /// floor far below.
    fn ice_block(threshold: f32) -> (World, RigidBodyHandle) {
        let mut world = World::new();
        world.register::<Position>();
        world.register::<Velocity>();
        world.register::<Orientation>();
        world.register::<RigidBodyComponent>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<CompoundFracture>();
        world.register::<BrittleSolid>();
        world.insert(Time::fixed(FRAME_DT));
        world.insert(PhysicsImpulseQueue::default());
        world.insert(DebugLog::default());

        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        let mut physics_world = PhysicsWorld::new(config);
        let body = physics_world.create_body(RigidBodyDesc::static_body().position(Point3::new(
            0.0,
            BLOCK_HEIGHT,
            0.0,
        )));
        physics_world.attach_collider(
            body,
            ColliderDesc::box_shape(HALF).density(900.0).friction(0.05),
        );
        world.insert(PhysicsResource::new(
            physics_world,
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));

        let model = crate::app::spawnables::shared::models::cuboid_model(HALF, MaterialId(0));
        world
            .create_entity()
            .with(Position(Vector3::new(0.0, BLOCK_HEIGHT, 0.0)))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(UnitQuaternion::identity()))
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(CompoundFracture::boxes(Vec::new(), 1, MaterialId(0)))
            .with(BrittleSolid::new(
                CleaveRule {
                    threshold,
                    ..CleaveRule::default()
                },
                MaterialId(0),
                1,
            ))
            .build();
        (world, body)
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

    fn frame(world: &mut World, stepper: &mut SequentialStepper, geometry: &FlatQuadGeometry) {
        step(world, stepper, geometry);
        FractureSystem.run_now(world);
        world.maintain();
    }

    /// A frame without the fracture system, which frees the struck wedge the
    /// same frame the block breaks. Tests that ask what the *break* did stop
    /// here; tests that ask what the object looks like afterwards do not.
    fn step(world: &mut World, stepper: &mut SequentialStepper, geometry: &FlatQuadGeometry) {
        {
            let mut physics = world.write_resource::<PhysicsResource>();
            let mut debug = DebugLines::default();
            stepper.step(&mut physics.world, FRAME_DT, geometry, &[], &[], &mut debug);
        }
        SolidCleaveSystem.run_now(world);
        world.maintain();
    }

    fn child_count(world: &World, body: RigidBodyHandle) -> usize {
        let physics = world.read_resource::<PhysicsResource>();
        physics.world.body(body).map_or(0, |b| b.colliders().len())
    }

    /// Three blocks in a row, joined end to end, as a course of a wall or a
    /// dome is. The middle one is the one that gets hit.
    fn row_of_three() -> (World, RigidBodyHandle) {
        let (world, body) = ice_block(20.0);
        {
            let mut physics = world.write_resource::<PhysicsResource>();
            for side in [-1.0, 1.0] {
                physics.world.attach_collider(
                    body,
                    ColliderDesc::box_shape(HALF)
                        .density(900.0)
                        .friction(0.05)
                        .offset_translation(Vector3::new(side * (HALF.x * 2.0 + 0.01), 0.0, 0.0)),
                );
            }
        }
        // Child 0 is the middle block; the two it holds are 1 and 2.
        let mut fractures = world.write_storage::<CompoundFracture>();
        let fracture = (&mut fractures).join().next().expect("the row");
        *fracture = CompoundFracture::boxes(
            vec![
                FractureJoint {
                    child_a: 0,
                    child_b: 1,
                    threshold: 1.0e9,
                },
                FractureJoint {
                    child_a: 0,
                    child_b: 2,
                    threshold: 1.0e9,
                },
            ],
            3,
            MaterialId(0),
        );
        drop(fractures);
        let mut solids = world.write_storage::<BrittleSolid>();
        let solid = (&mut solids).join().next().expect("the row");
        solid.contact_load = crate::fracture::ContactLoadTracker::new(3);
        solid.depths = crate::fracture::BreakDepths::new(3);
        drop(solids);
        (world, body)
    }

    /// A block that cleaves in the middle of a structure must not take the
    /// structure apart with it: its wedges inherit what it was holding, or
    /// everything the block was carrying is suddenly connected to nothing
    /// and the fracture system drops it.
    #[test]
    fn a_cleaved_block_still_holds_what_it_was_holding() {
        let (mut world, body) = row_of_three();
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);

        let _hammer = drop_box(&mut world, 0.0, 1.0, 40.0);
        for _ in 0..30 {
            {
                let mut physics = world.write_resource::<PhysicsResource>();
                let mut debug = DebugLines::default();
                stepper.step(
                    &mut physics.world,
                    FRAME_DT,
                    &geometry,
                    &[],
                    &[],
                    &mut debug,
                );
            }
            SolidCleaveSystem.run_now(&world);
            world.maintain();
            if child_count(&world, body) > 3 {
                break;
            }
        }
        assert!(
            child_count(&world, body) > 3,
            "the middle block never broke"
        );

        let fractures = world.read_storage::<CompoundFracture>();
        let fracture = (&fractures).join().next().expect("the row");
        let biggest = fracture
            .connected_components()
            .into_iter()
            .max_by_key(Vec::len)
            .expect("a structure");
        assert!(
            biggest.len() >= 3,
            "the row fell into {:?}",
            fracture.connected_components()
        );
    }

    /// The whole point: a struck block is a few wedges afterwards, and they
    /// are wedges rather than boxes.
    #[test]
    fn a_struck_block_is_replaced_by_the_wedges_it_cleaved_into() {
        let (mut world, block) = ice_block(20.0);
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
        for handle in body.colliders() {
            let collider = physics.world.collider(*handle).expect("live collider");
            assert!(
                matches!(
                    collider.shape(),
                    crate::physics::ColliderShape::ConvexHull { .. }
                ),
                "a cleaved piece should be a hull, not a box"
            );
        }
    }

    /// A block left alone is a block. The cleave system reads every tracker
    /// every frame, and a resting load must never read as a blow.
    #[test]
    fn a_block_nobody_touches_stays_whole() {
        let (mut world, block) = ice_block(20.0);
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
        let (mut world, block) = ice_block(20.0);
        let geometry = FlatQuadGeometry::new(50.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        let whole: f32 = {
            let physics = world.read_resource::<PhysicsResource>();
            let body = physics.world.body(block).unwrap();
            body.colliders()
                .iter()
                .filter_map(|h| physics.world.collider(*h))
                .map(|c| c.mass())
                .sum()
        };

        let _hammer = drop_box(&mut world, 0.0, 1.0, 40.0);
        // The fracture system is left out: it frees the struck wedge the
        // same frame the block breaks, and the question here is what the
        // split itself did to the body, not what left it afterwards.
        let mut after = whole;
        for _ in 0..90 {
            {
                let mut physics = world.write_resource::<PhysicsResource>();
                let mut debug = DebugLines::default();
                stepper.step(
                    &mut physics.world,
                    FRAME_DT,
                    &geometry,
                    &[],
                    &[],
                    &mut debug,
                );
            }
            SolidCleaveSystem.run_now(&world);
            world.maintain();

            let physics = world.read_resource::<PhysicsResource>();
            let Some(body) = physics.world.body(block) else {
                break;
            };
            if body.colliders().len() > 1 {
                after = body
                    .colliders()
                    .iter()
                    .filter_map(|h| physics.world.collider(*h))
                    .map(|c| c.mass())
                    .sum();
                break;
            }
        }
        assert!(after != whole, "the block never broke");
        assert!(
            (after - whole).abs() < whole * 0.05,
            "the pieces weigh {after} where the block weighed {whole}"
        );
    }
}
