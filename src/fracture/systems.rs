//! Fracture ECS system.

use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use rayon::prelude::*;
use specs::{Builder, Entities, Join, Read, ReadStorage, System, WriteStorage};

use super::components::CompoundFracture;
use super::debris::Debris;
use super::load::{ChildLoad, ChildLoads};
use crate::app::spawnables::shared::models::{
    assemble_by_material, piece_model, PieceHull, PiecePlacement, PieceStyle, PlacedMesh,
};
use crate::collision::convex_hull::ConvexHull;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::model::Model;
use crate::physics::{
    ColliderHandle, ColliderShape, FrictionModel, PhysicsImpulseQueue, RigidBodyHandle,
};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;
use crate::time::Time;

/// Breaks joints on compound bodies when the load on a child exceeds what its
/// joints can hold. After breaking, splits disconnected children into
/// independent bodies.
///
/// Two sources add into that load, per child:
///
/// - **Blast.** `PhysicsImpulseQueue::last_impulses()` evaluated at the child's
///   world position, so an explosion's falloff opens a hole where it went off.
/// - **Impact.** The frame-over-frame *spike* in the contact impulse the solver
///   pushed through that child, read from `ImpactLedger::for_collider`.
///
/// Contact is read as a spike and not as a level on purpose. The level includes
/// whatever the object is already carrying: a heavy compound standing still
/// pushes `mass * gravity * frame_dt` through its lowest children every frame,
/// which for anything massive is larger than a sensible fracture threshold on
/// its own. Differencing against last frame leaves resting weight and steady
/// pushing at roughly zero and keeps only the step change of a real collision.
pub struct FractureSystem;

impl<'a> System<'a> for FractureSystem {
    type SystemData = (
        Entities<'a>,
        specs::Write<'a, PhysicsResource>,
        WriteStorage<'a, CompoundFracture>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, ModelInstance>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Velocity>,
        Read<'a, specs::LazyUpdate>,
        Read<'a, PhysicsImpulseQueue>,
        Read<'a, Time>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            mut physics,
            mut fractures,
            bodies,
            mut models,
            mut positions,
            mut velocities,
            lazy,
            impulse_queue,
            time,
        ) = data;

        let last_impulses = impulse_queue.last_impulses();

        // Collect fracture triggers.
        let mut triggers: Vec<FractureTrigger> = Vec::new();

        for (entity, fracture, body_comp) in (&entities, &mut fractures, &bodies).join() {
            let body_handle = body_comp.0;
            let Some(body) = physics.world.body(body_handle) else {
                continue;
            };

            let body_pos = body.position();
            let body_rot = body.rotation();
            let collider_handles: Vec<_> = body.colliders().to_vec();

            // The impact baseline has to advance every frame, including frames
            // where this body cannot break. Skipping it would let a load build
            // up unwatched and then read as one huge spike later.
            let contact_spikes = fracture.contact_load.advance(
                &physics.world,
                body_handle,
                &collider_handles,
                time.delta_seconds(),
            );

            if collider_handles.len() <= 1 {
                continue;
            }

            // Per-child load. The blast impulse is evaluated at the child's
            // own world position so distance falloff is respected per piece;
            // the contact spike is carried with the point it landed at, so a
            // long child does not report a hit at one end to a joint at the
            // other.
            let children: Vec<ChildLoad> = collider_handles
                .iter()
                .zip(contact_spikes)
                .map(|(handle, spike)| {
                    let collider = physics.world.collider(*handle);
                    let centre = collider
                        .map(|c| {
                            Point3::from(c.world_transform(body_pos, body_rot).translation.vector)
                        })
                        .unwrap_or(body_pos);
                    let blast = last_impulses
                        .iter()
                        .filter_map(|imp| imp.impulse_at(centre))
                        .map(|v| v.magnitude())
                        .sum();
                    ChildLoad {
                        blast,
                        contact: spike.magnitude,
                        contact_point: spike.point,
                        centre,
                        radius: collider.map(|c| c.shape().bounding_radius()).unwrap_or(0.0),
                    }
                })
                .collect();

            let loads = ChildLoads::new(children, fracture.contact_threshold);
            if !fracture.split_pending && !fracture.joints.iter().any(|joint| loads.breaks(joint)) {
                continue;
            }

            triggers.push(FractureTrigger {
                entity,
                body_handle,
                collider_handles,
                loads,
            });
        }

        // Execute fracture operations.
        for trigger in triggers {
            let Some(fracture) = fractures.get_mut(trigger.entity) else {
                continue;
            };
            let materials = fracture.materials.clone();
            let style = fracture.style;
            let anchor = fracture.texture_anchor;
            let whole = fracture.whole_shape.clone();
            let debris_of = fracture.sheds_debris.then_some(trigger.entity);

            // Break the overloaded joints; the rest of the structure survives.
            fracture.joints.retain(|joint| !trigger.loads.breaks(joint));
            fracture.split_pending = false;

            // Compute connected components from surviving joints.
            let components = fracture.connected_components();
            let released = fracture.released;

            if components.len() <= 1 && !released {
                continue;
            }

            // Keep the largest component on the original body, unless the
            // body is giving everything up.
            let largest_idx = if released {
                None
            } else {
                components
                    .iter()
                    .enumerate()
                    .max_by_key(|(_, c)| c.len())
                    .map(|(i, _)| i)
            };

            // Snapshot body state.
            let Some(body) = physics.world.body(trigger.body_handle) else {
                continue;
            };
            let body_pos = body.position();
            let body_rot = body.rotation();
            let body_lin_vel = body.linear_velocity();
            let body_ang_vel = body.angular_velocity();

            // Snapshot all children that will be split off.
            let mut split_groups: Vec<Vec<ChildSnapshot>> = Vec::new();
            for (comp_idx, component) in components.iter().enumerate() {
                if Some(comp_idx) == largest_idx {
                    continue;
                }
                let mut group = Vec::new();
                for &child_idx in component {
                    if child_idx < trigger.collider_handles.len() {
                        if let Some(mut info) = snapshot_child(
                            &physics,
                            trigger.collider_handles[child_idx],
                            body_pos,
                            body_rot,
                            body_lin_vel,
                            body_ang_vel,
                        ) {
                            info.material = materials
                                .get(child_idx)
                                .or_else(|| materials.last())
                                .copied()
                                .unwrap_or(MaterialId(0));
                            info.kick = fracture.kick_of(child_idx);
                            group.push(info);
                        }
                    }
                }
                split_groups.push(group);
            }

            // Detach the split-off children from the compound body.
            let mut handles_to_detach: Vec<ColliderHandle> = Vec::new();
            for (comp_idx, component) in components.iter().enumerate() {
                if Some(comp_idx) == largest_idx {
                    continue;
                }
                for &child_idx in component {
                    if child_idx < trigger.collider_handles.len() {
                        handles_to_detach.push(trigger.collider_handles[child_idx]);
                    }
                }
            }
            for ch in &handles_to_detach {
                physics.world.detach_collider(trigger.body_handle, *ch);
            }

            // Spawn each split-off child as an independent body. Every piece's
            // mesh is its own business, and a blast frees them by the hundred,
            // so they are built side by side before the spawning, which has
            // to take the physics world one piece at a time.
            let freed: Vec<&ChildSnapshot> = split_groups.iter().flatten().collect();
            let freed_models: Vec<Option<Arc<Model>>> = freed
                .par_iter()
                .map(|info| freed_piece_model(info, style, anchor, whole.as_deref()))
                .collect();
            for (info, model) in freed.into_iter().zip(freed_models) {
                spawn_freed_piece(
                    &mut physics,
                    &entities,
                    &lazy,
                    info,
                    model,
                    body_ang_vel,
                    last_impulses,
                    debris_of.map(|origin| Debris::new(origin, info.shape.compute_mass(1.0))),
                );
            }

            // A released body has nothing left; it and its entity are done.
            if released {
                physics.world.remove_body(trigger.body_handle);
                let _ = entities.delete(trigger.entity);
                continue;
            }

            // The survivors are no longer laid out around the body origin, so
            // move the origin onto them. Without this the remnant spins about
            // the vanished compound's centre — a plank pivoting on a phantom
            // axle metres away, too sluggish to push straight.
            // The origin moves onto the survivors, so the object's texture
            // origin has to follow it: everything below draws from the new
            // frame, and the pieces that came free a moment ago were drawn
            // from the old one.
            let moved = physics.world.recenter_on_colliders(trigger.body_handle);
            if let Some(fracture) = fractures.get_mut(trigger.entity) {
                fracture.texture_anchor += moved;
            }

            // Recentering moves the body's origin and shifts every surviving
            // collider's offset to match, so nothing moves in the world — but
            // `PhysicsSyncSystem` has already run this frame, so the entity's
            // own copy of the transform is now a frame out of date. Drawing
            // the rebuilt model, whose offsets are relative to the new origin,
            // against the old origin displaces the whole structure by the
            // centre-of-mass shift for exactly one frame: the object blinks
            // sideways at the moment it breaks.
            refresh_transform(
                &physics,
                trigger.entity,
                trigger.body_handle,
                &mut positions,
                &mut velocities,
            );

            // Update the fracture component to the body's collider list as it
            // now stands. Detaching swaps the last collider into the hole it
            // leaves, so the survivors are *not* in the order the component
            // listed them; the order has to be read back off the body, or a
            // surviving child inherits its neighbour's material and load.
            let kept: Vec<usize> = physics
                .world
                .body(trigger.body_handle)
                .map(|body| {
                    body.colliders()
                        .iter()
                        .filter_map(|handle| {
                            trigger.collider_handles.iter().position(|h| h == handle)
                        })
                        .collect()
                })
                .unwrap_or_default();
            let new_count = kept.len();
            let Some(fracture) = fractures.get_mut(trigger.entity) else {
                continue;
            };
            fracture.remap_children(&kept, new_count);
            fracture.model_stale = true;
        }

        rebuild_stale_models(&physics, &mut fractures, &bodies, &mut models);
    }
}

/// Redraw every compound whose children changed this frame, once each.
///
/// The last thing the system does, so that a body reshaped by a cleave or a
/// craze and then split here is drawn as it finally stands, and only then.
fn rebuild_stale_models(
    physics: &PhysicsResource,
    fractures: &mut WriteStorage<CompoundFracture>,
    bodies: &ReadStorage<RigidBodyComponent>,
    models: &mut WriteStorage<ModelInstance>,
) {
    for (fracture, body, instance) in (fractures, bodies, models).join() {
        if !std::mem::take(&mut fracture.model_stale) {
            continue;
        }
        if let Some(model) = compound_model_of(
            physics,
            body.0,
            &fracture.materials,
            fracture.style,
            fracture.texture_anchor,
            fracture.whole_shape.as_deref(),
        ) {
            instance.model = model;
        }
    }
}

/// Copy a body's transform onto its entity, for a system that moved the body
/// after this frame's physics sync had already run.
fn refresh_transform(
    physics: &PhysicsResource,
    entity: specs::Entity,
    body_handle: RigidBodyHandle,
    positions: &mut WriteStorage<Position>,
    velocities: &mut WriteStorage<Velocity>,
) {
    let Some(body) = physics.world.body(body_handle) else {
        return;
    };
    if let Some(position) = positions.get_mut(entity) {
        position.0 = body.position().coords;
    }
    if let Some(velocity) = velocities.get_mut(entity) {
        velocity.0 = body.linear_velocity();
    }
}

/// Trigger data collected from the join pass.
struct FractureTrigger {
    entity: specs::Entity,
    body_handle: RigidBodyHandle,
    collider_handles: Vec<ColliderHandle>,
    /// What each child is carrying, indexed by child position in the collider
    /// list.
    loads: ChildLoads,
}

/// Snapshot of a child collider's state, captured before detachment.
struct ChildSnapshot {
    /// Material this child is drawn with, copied from the compound's per-child
    /// list so a freed piece keeps the look it had while attached.
    material: MaterialId,
    /// Impulse handed to the piece as it leaves, from whoever severed it.
    kick: Vector3<f32>,
    shape: ColliderShape,
    mass: f32,
    restitution: f32,
    friction: FrictionModel,
    world_pos: Point3<f32>,
    world_rot: nalgebra::UnitQuaternion<f32>,
    lin_vel: Vector3<f32>,
    /// Where the child sat in the compound's frame, kept only so its mesh can
    /// be drawn with the markings it wore while attached.
    local_offset: Vector3<f32>,
}

fn snapshot_child(
    physics: &PhysicsResource,
    collider_handle: ColliderHandle,
    body_pos: Point3<f32>,
    body_rot: nalgebra::UnitQuaternion<f32>,
    body_lin_vel: Vector3<f32>,
    body_ang_vel: Vector3<f32>,
) -> Option<ChildSnapshot> {
    let collider = physics.world.collider(collider_handle)?;
    let world_xform = collider.world_transform(body_pos, body_rot);
    let world_pos = Point3::from(world_xform.translation.vector);
    let r = world_pos - body_pos;

    Some(ChildSnapshot {
        // Both overwritten by the caller, which knows this child's index.
        material: MaterialId(0),
        kick: Vector3::zeros(),
        shape: collider.shape().clone(),
        mass: collider.mass(),
        restitution: collider.material().restitution,
        friction: collider.material().friction,
        world_pos,
        world_rot: world_xform.rotation,
        lin_vel: body_lin_vel + body_ang_vel.cross(&r),
        local_offset: collider.offset().translation.vector,
    })
}

/// The model a freed piece is drawn with, standing on its own.
///
/// `None` for a shape no style draws. Pure, so a blast's worth of pieces can
/// be built side by side.
fn freed_piece_model(
    info: &ChildSnapshot,
    style: PieceStyle,
    anchor: Vector3<f32>,
    whole: Option<&ConvexHull>,
) -> Option<Arc<Model>> {
    match &info.shape {
        ColliderShape::Box { half_extents } => Some(piece_model(
            &PiecePlacement::new(*half_extents, info.local_offset + anchor),
            style.boxes,
            style.uvs,
            info.material,
        )),
        ColliderShape::ConvexHull { hull } => {
            let (vertices, indices) = (style.hulls)(
                &PieceHull::new(hull, info.local_offset + anchor).within(whole),
                style.uvs,
            );
            Some(assemble_by_material(vec![PlacedMesh {
                vertices,
                indices,
                offset: Vector3::zeros(),
                rotation: nalgebra::UnitQuaternion::identity(),
                material: info.material,
            }]))
        }
        _ => None,
    }
}

#[allow(clippy::too_many_arguments)]
fn spawn_freed_piece(
    physics: &mut PhysicsResource,
    entities: &Entities,
    lazy: &specs::LazyUpdate,
    info: &ChildSnapshot,
    model: Option<Arc<Model>>,
    body_ang_vel: Vector3<f32>,
    impulse_sources: &[crate::physics::PhysicsImpulse],
    debris: Option<Debris>,
) {
    // Apply explosion impulse directly to this piece's mass so light
    // pieces fly off faster than heavy ones.
    let explosion_kick: Vector3<f32> = impulse_sources
        .iter()
        .filter_map(|imp| imp.impulse_at(info.world_pos))
        .sum();
    let piece_vel = info.lin_vel + (explosion_kick + info.kick) / info.mass;

    let new_body_desc = crate::physics::RigidBodyDesc::dynamic()
        .position(info.world_pos)
        .rotation(info.world_rot)
        .linear_velocity(piece_vel)
        .angular_velocity(body_ang_vel)
        .linear_damping(0.01)
        .angular_damping(0.005);

    let new_body_handle = physics.world.create_body(new_body_desc);

    let density = info.mass / info.shape.compute_mass(1.0);
    let new_collider_desc =
        collider_desc_from_shape(&info.shape, density, info.restitution, info.friction);
    physics
        .world
        .attach_collider(new_body_handle, new_collider_desc);

    let Some(piece_model) = model else {
        return;
    };

    let mut piece = lazy
        .create_entity(entities)
        .with(Position(info.world_pos.coords))
        .with(Velocity(piece_vel))
        .with(Orientation(info.world_rot))
        .with(RigidBodyComponent(new_body_handle))
        .with(ModelInstance::new(piece_model))
        .with(Renderable);
    if let Some(debris) = debris {
        piece = piece.with(debris);
    }
    piece.build();
}

/// The model of a compound body as its colliders stand right now: box
/// children and hull children each drawn by `style`, each wearing its own
/// material.
///
/// `None` if the body has no drawable child, which is not the same as an
/// empty model — a caller that has nothing to draw should keep what it had
/// rather than blank the object.
pub(crate) fn compound_model_of(
    physics: &PhysicsResource,
    body_handle: RigidBodyHandle,
    materials: &[MaterialId],
    style: PieceStyle,
    anchor: Vector3<f32>,
    whole: Option<&ConvexHull>,
) -> Option<Arc<Model>> {
    let body = physics.world.body(body_handle)?;
    let fallback = materials.last().copied().unwrap_or(MaterialId(0));

    // Read off serially, meshed in parallel: a freshly cleaved dome is
    // hundreds of chamfered wedges, and each one's mesh is its own business.
    let children: Vec<(ColliderShape, Vector3<f32>, UnitQuaternion<f32>, MaterialId)> = body
        .colliders()
        .iter()
        .enumerate()
        .filter_map(|(child, ch)| {
            let c = physics.world.collider(*ch)?;
            Some((
                c.shape().clone(),
                c.offset().translation.vector,
                c.offset().rotation,
                materials.get(child).copied().unwrap_or(fallback),
            ))
        })
        .collect();

    let pieces: Vec<PlacedMesh> = children
        .par_iter()
        .filter_map(|(shape, offset, rotation, material)| {
            let (offset, rotation) = (*offset, *rotation);
            if !offset.iter().all(|v| v.is_finite()) {
                log::error!("Fracture: NaN/Inf in remaining collider offset {offset:?}");
                return None;
            }
            let (vertices, indices) = match shape {
                ColliderShape::Box { half_extents } => {
                    if !half_extents.iter().all(|v| v.is_finite()) {
                        log::error!(
                            "Fracture: NaN/Inf in remaining collider extents {half_extents:?}"
                        );
                        return None;
                    }
                    // Rotation carried through: a child that was laid at an
                    // angle must still be at that angle after the break, or an
                    // object made of tilted pieces straightens itself out the
                    // moment it loses one.
                    (style.boxes)(
                        &PiecePlacement::new(*half_extents, offset + anchor).rotated(rotation),
                        style.uvs,
                    )
                }
                ColliderShape::ConvexHull { hull } => (style.hulls)(
                    &PieceHull::new(hull, offset + anchor).within(whole),
                    style.uvs,
                ),
                _ => return None,
            };
            Some(PlacedMesh {
                vertices,
                indices,
                offset,
                rotation,
                material: *material,
            })
        })
        .collect();

    if pieces.is_empty() {
        return None;
    }
    Some(assemble_by_material(pieces))
}

/// Create a `ColliderDesc` with identity offset from a detached shape.
fn collider_desc_from_shape(
    shape: &ColliderShape,
    density: f32,
    restitution: f32,
    friction: FrictionModel,
) -> crate::physics::ColliderDesc {
    let desc = match shape {
        ColliderShape::Box { half_extents } => {
            crate::physics::ColliderDesc::box_shape(*half_extents)
        }
        ColliderShape::Sphere { radius } => crate::physics::ColliderDesc::sphere(*radius),
        ColliderShape::Capsule {
            half_height,
            radius,
        } => crate::physics::ColliderDesc::capsule(*half_height, *radius),
        ColliderShape::ConvexHull { hull } => {
            crate::physics::ColliderDesc::convex_hull(hull.clone())
        }
    };
    desc.density(density)
        .restitution(restitution)
        .friction_model(friction)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::debug::DebugLines;
    use crate::fracture::{ContactLoadTracker, FractureJoint};
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc};

    const FRAME_DT: f32 = 1.0 / 60.0;

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
                        ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 0.5))
                            .offset_translation(Vector3::new(i as f32 - 1.5, 0.0, 0.0))
                            .density(900.0)
                            .restitution(0.0),
                    )
                    .expect("collider attaches to a live body")
            })
            .collect();
        (body, handles)
    }

    /// Run the world and report the largest per-child spike seen over the
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
        let mut tracker = ContactLoadTracker::new(colliders.len());
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
        // Skip the frames where it is still settling onto the quad; what is
        // measured is a slab that is simply standing there.
        let resting_peak = peak_spike(
            &mut resting_world,
            resting,
            &resting_colliders,
            240,
            60..240,
            None,
        );

        // The same slab with something shoving it along the ground — the case
        // the old system excluded contact entirely to avoid.
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

    /// A freed piece is drawn by joining over storages, and `LazyUpdate` does
    /// not put it into them until `world.maintain()`. The frame loop must
    /// therefore maintain *before* the thread-local render pass: the compound's
    /// model drops a piece the moment it breaks off, so if the piece's own
    /// entity is not there yet, the structure blinks out as it comes apart.
    #[test]
    fn a_lazily_created_piece_is_invisible_until_the_world_is_maintained() {
        use specs::{Builder, Join, World, WorldExt};

        let mut world = World::new();
        world.register::<Renderable>();
        let lazy_created = {
            let entities = world.entities();
            let lazy = world.read_resource::<specs::LazyUpdate>();
            lazy.create_entity(&entities).with(Renderable).build()
        };

        let visible = |world: &World| {
            let renderables = world.read_storage::<Renderable>();
            let entities = world.entities();
            (&entities, &renderables).join().count()
        };

        assert_eq!(
            visible(&world),
            0,
            "a lazily created entity was joinable before maintain; \
             the render-order hazard this guards has changed"
        );

        world.maintain();

        assert_eq!(visible(&world), 1, "maintain did not apply the creation");
        assert!(world.is_alive(lazy_created));
    }

    /// The flicker this guards: a structure jumped sideways for one frame at
    /// the moment it broke.
    ///
    /// Detaching pieces moves the compound's origin onto what is left, and
    /// every surviving collider's offset shifts to match, so nothing actually
    /// moves. But this system runs after the frame's physics sync, so unless it
    /// refreshes the entity itself, the rebuilt model — whose offsets are
    /// relative to the new origin — is drawn against the old one.
    #[test]
    fn a_broken_compound_is_drawn_where_its_body_actually_is() {
        use crate::physics::{PhysicsImpulse, PhysicsImpulseQueue};
        use specs::{Builder, RunNow, World, WorldExt};

        let mut world = World::new();
        world.register::<Position>();
        world.register::<Velocity>();
        world.register::<Orientation>();
        world.register::<RigidBodyComponent>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<CompoundFracture>();
        world.insert(crate::time::Time::default());

        // A row of four boxes, joined in a chain, so that blasting one end off
        // moves the centre of mass a long way along the row.
        let origin = Point3::new(0.0, 10.0, 0.0);
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let body = physics_world.create_body(RigidBodyDesc::dynamic().position(origin));
        for i in 0..4 {
            physics_world
                .attach_collider(
                    body,
                    ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 0.5))
                        .offset_translation(Vector3::new(i as f32 - 1.5, 0.0, 0.0))
                        .density(500.0),
                )
                .expect("collider attaches to a live body");
        }
        world.insert(PhysicsResource::new(
            physics_world,
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));

        let joints = (0..3)
            .map(|i| FractureJoint {
                child_a: i,
                child_b: i + 1,
                threshold: 10.0,
            })
            .collect();
        let entity = world
            .create_entity()
            .with(Position(origin.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(nalgebra::UnitQuaternion::identity()))
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(piece_model(
                &PiecePlacement::new(Vector3::new(0.5, 0.5, 0.5), Vector3::zeros()),
                crate::app::spawnables::shared::models::cuboid_mesh,
                crate::app::spawnables::shared::models::SurfaceUvs::Fitted,
                crate::rendering::material::MaterialId(0),
            )))
            .with(Renderable)
            .with(CompoundFracture::boxes(
                joints,
                4,
                crate::rendering::material::MaterialId(0),
            ))
            .build();

        // A blast at the far end of the row, tight enough to reach only the
        // outermost box.
        let mut queue = PhysicsImpulseQueue::default();
        queue.push(PhysicsImpulse::radial(
            origin + Vector3::new(-1.5, 0.0, 0.0),
            0.9,
            5_000.0,
            0.0,
        ));
        let _ = queue.drain().count();
        world.insert(queue);

        FractureSystem.run_now(&world);

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

    /// Every freed piece must be drawable on the very frame it breaks off, at
    /// the place it broke off from. It leaves the compound's model that frame,
    /// so any gap or displacement here is a piece that visibly blinks.
    #[test]
    fn a_freed_piece_is_drawable_where_it_broke_off() {
        use crate::physics::{PhysicsImpulse, PhysicsImpulseQueue};
        use specs::{Builder, Join, RunNow, World, WorldExt};

        let mut world = World::new();
        world.register::<Position>();
        world.register::<Velocity>();
        world.register::<Orientation>();
        world.register::<RigidBodyComponent>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<CompoundFracture>();
        world.insert(crate::time::Time::default());

        let origin = Point3::new(0.0, 10.0, 0.0);
        let mut physics_world = PhysicsWorld::new(PhysicsConfig::default());
        let body = physics_world.create_body(RigidBodyDesc::dynamic().position(origin));
        for i in 0..4 {
            physics_world
                .attach_collider(
                    body,
                    ColliderDesc::box_shape(Vector3::new(0.5, 0.5, 0.5))
                        .offset_translation(Vector3::new(i as f32 - 1.5, 0.0, 0.0))
                        .density(500.0),
                )
                .expect("collider attaches to a live body");
        }
        world.insert(PhysicsResource::new(
            physics_world,
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));

        let joints = (0..3)
            .map(|i| FractureJoint {
                child_a: i,
                child_b: i + 1,
                threshold: 10.0,
            })
            .collect();
        world
            .create_entity()
            .with(Position(origin.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(nalgebra::UnitQuaternion::identity()))
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(piece_model(
                &PiecePlacement::new(Vector3::new(0.5, 0.5, 0.5), Vector3::zeros()),
                crate::app::spawnables::shared::models::cuboid_mesh,
                crate::app::spawnables::shared::models::SurfaceUvs::Fitted,
                crate::rendering::material::MaterialId(0),
            )))
            .with(Renderable)
            .with(CompoundFracture::boxes(
                joints,
                4,
                crate::rendering::material::MaterialId(0),
            ))
            .build();
        world.maintain();

        // The outermost box sat at x = -1.5 relative to the origin.
        let broken_off = origin + Vector3::new(-1.5, 0.0, 0.0);
        let mut queue = PhysicsImpulseQueue::default();
        queue.push(PhysicsImpulse::radial(broken_off, 0.9, 5_000.0, 0.0));
        let _ = queue.drain().count();
        world.insert(queue);

        let drawn = |world: &World| -> Vec<Vector3<f32>> {
            let models = world.read_storage::<ModelInstance>();
            let positions = world.read_storage::<Position>();
            let renderables = world.read_storage::<Renderable>();
            let entities = world.entities();
            (&entities, &models, &positions, &renderables)
                .join()
                .map(|(_, _, p, _)| p.0)
                .collect()
        };

        assert_eq!(drawn(&world).len(), 1, "one compound before the break");

        FractureSystem.run_now(&world);
        // Exactly what the frame loop does before the render pass runs.
        world.maintain();

        let after = drawn(&world);
        assert_eq!(
            after.len(),
            2,
            "the compound and its freed piece should both be drawable on the \
             break frame, found {} drawable entities",
            after.len()
        );
        assert!(
            after
                .iter()
                .any(|p| (p - broken_off.coords).magnitude() < 1e-4),
            "no drawable entity sits where the piece broke off ({:?}); found {:?}",
            broken_off.coords,
            after
        );
        for p in &after {
            assert!(p.iter().all(|c| c.is_finite()), "non-finite position {p:?}");
        }
    }

    /// The rebuilt remnant model must still span the pieces it is made of.
    ///
    /// The trace can say an entity was drawn and where its origin was, but not
    /// whether its geometry is right: a model whose per-piece offsets were lost
    /// draws every box on top of the others, at the body's origin, which is
    /// what "collapsed to the centre of mass" would look like.
    #[test]
    fn a_rebuilt_remnant_still_spans_its_surviving_pieces() {
        use crate::physics::{PhysicsImpulse, PhysicsImpulseQueue};
        use specs::{Builder, RunNow, World, WorldExt};

        let mut world = World::new();
        world.register::<Position>();
        world.register::<Velocity>();
        world.register::<Orientation>();
        world.register::<RigidBodyComponent>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<CompoundFracture>();
        world.insert(crate::time::Time::default());

        let origin = Point3::new(0.0, 10.0, 0.0);
        let half = Vector3::new(0.5, 0.5, 0.5);
        let mut pw = PhysicsWorld::new(PhysicsConfig::default());
        let body = pw.create_body(RigidBodyDesc::dynamic().position(origin));
        for i in 0..4 {
            pw.attach_collider(
                body,
                ColliderDesc::box_shape(half)
                    .offset_translation(Vector3::new(i as f32 - 1.5, 0.0, 0.0))
                    .density(500.0),
            )
            .expect("collider attaches to a live body");
        }
        world.insert(PhysicsResource::new(
            pw,
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));

        let joints = (0..3)
            .map(|i| FractureJoint {
                child_a: i,
                child_b: i + 1,
                threshold: 10.0,
            })
            .collect();
        let entity = world
            .create_entity()
            .with(Position(origin.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(nalgebra::UnitQuaternion::identity()))
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(piece_model(
                &PiecePlacement::new(half, Vector3::zeros()),
                crate::app::spawnables::shared::models::cuboid_mesh,
                crate::app::spawnables::shared::models::SurfaceUvs::Fitted,
                crate::rendering::material::MaterialId(0),
            )))
            .with(Renderable)
            .with(CompoundFracture::boxes(
                joints,
                4,
                crate::rendering::material::MaterialId(0),
            ))
            .build();
        world.maintain();

        let mut queue = PhysicsImpulseQueue::default();
        queue.push(PhysicsImpulse::radial(
            origin + Vector3::new(-1.5, 0.0, 0.0),
            0.9,
            5_000.0,
            0.0,
        ));
        let _ = queue.drain().count();
        world.insert(queue);

        FractureSystem.run_now(&world);
        world.maintain();

        // Where the model says its geometry is, in the body's own frame.
        let models = world.read_storage::<ModelInstance>();
        let model = &models.get(entity).expect("the remnant keeps a model").model;
        let (mut model_min, mut model_max) = (
            Vector3::repeat(f32::INFINITY),
            Vector3::repeat(f32::NEG_INFINITY),
        );
        let mut vertex_count = 0;
        for part in &model.parts {
            for primitive in &part.primitives {
                for vertex in &primitive.vertices {
                    model_min = model_min.inf(&vertex.pos);
                    model_max = model_max.sup(&vertex.pos);
                    vertex_count += 1;
                }
            }
        }

        // Where the physics says it is, in the same frame.
        let physics = world.read_resource::<PhysicsResource>();
        let remnant = physics.world.body(body).expect("the remnant survives");
        let (mut solid_min, mut solid_max) = (
            Vector3::repeat(f32::INFINITY),
            Vector3::repeat(f32::NEG_INFINITY),
        );
        for handle in remnant.colliders() {
            let collider = physics.world.collider(*handle).unwrap();
            let centre = collider.offset().translation.vector;
            solid_min = solid_min.inf(&(centre - half));
            solid_max = solid_max.sup(&(centre + half));
        }

        assert_eq!(vertex_count, 24 * remnant.colliders().len());
        assert!(
            (model_min - solid_min).magnitude() < 1e-4
                && (model_max - solid_max).magnitude() < 1e-4,
            "model spans {model_min:?}..{model_max:?} but the colliders span \
             {solid_min:?}..{solid_max:?}"
        );
        // And it must actually be spread out, not stacked at the origin.
        assert!(
            (model_max.x - model_min.x) > 2.0,
            "the remnant's boxes collapsed onto each other: x span {}",
            model_max.x - model_min.x
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
        let mut tracker = ContactLoadTracker::new(colliders.len());

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

        // Nudge it awake without hitting it: a velocity a shove would impart.
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
}
