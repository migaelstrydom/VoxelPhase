//! Fracture ECS system.

use nalgebra::{Point3, Vector3};
use specs::{Builder, Entities, Join, Read, System, WriteStorage};

use super::components::CompoundFracture;
use crate::app::spawnables::shared::models::{
    compound_model, piece_model, PieceMesh, PiecePlacement,
};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::physics::{ColliderHandle, ColliderShape, FrictionModel, PhysicsImpulseQueue};
use crate::systems::PhysicsResource;

/// Breaks joints on compound bodies when explicit impulse sources (explosions,
/// etc.) deliver enough energy. After breaking, splits disconnected children
/// into independent bodies.
///
/// Uses `PhysicsImpulseQueue::last_impulses()` rather than contact solver
/// impulses, so sustained contact forces (player pushing, resting on ground)
/// never cause fracture.
pub struct FractureSystem;

impl<'a> System<'a> for FractureSystem {
    type SystemData = (
        Entities<'a>,
        specs::Write<'a, PhysicsResource>,
        WriteStorage<'a, CompoundFracture>,
        WriteStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, ModelInstance>,
        Read<'a, specs::LazyUpdate>,
        Read<'a, PhysicsImpulseQueue>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, mut physics, mut fractures, bodies, mut models, lazy, impulse_queue) = data;

        let last_impulses = impulse_queue.last_impulses();
        if last_impulses.is_empty() {
            return;
        }

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
            if collider_handles.len() <= 1 {
                continue;
            }

            // Compute per-child impulse magnitudes at each collider's world
            // position so that distance falloff is respected per piece.
            let child_impulses: Vec<f32> = collider_handles
                .iter()
                .map(|ch| {
                    let child_pos = physics
                        .world
                        .collider(*ch)
                        .map(|c| {
                            Point3::from(c.world_transform(body_pos, body_rot).translation.vector)
                        })
                        .unwrap_or(body_pos);
                    last_impulses
                        .iter()
                        .filter_map(|imp| imp.impulse_at(child_pos))
                        .map(|v| v.magnitude())
                        .sum()
                })
                .collect();

            // Check if any joint should break (impulse at either endpoint
            // exceeds that joint's threshold).
            let any_broken = fracture.joints.iter().any(|joint| {
                let imp_a = child_impulses.get(joint.child_a).copied().unwrap_or(0.0);
                let imp_b = child_impulses.get(joint.child_b).copied().unwrap_or(0.0);
                imp_a.max(imp_b) > joint.threshold
            });

            if !any_broken {
                continue;
            }

            triggers.push(FractureTrigger {
                entity,
                body_handle,
                collider_handles,
                child_impulses,
            });
        }

        // Execute fracture operations.
        for trigger in triggers {
            let Some(fracture) = fractures.get_mut(trigger.entity) else {
                continue;
            };
            let material = fracture.material;
            let piece_mesh = fracture.piece_mesh;

            // Break joints where the impulse at either endpoint exceeds the
            // joint's threshold — joints far from the blast survive.
            fracture.joints.retain(|joint| {
                let imp_a = trigger
                    .child_impulses
                    .get(joint.child_a)
                    .copied()
                    .unwrap_or(0.0);
                let imp_b = trigger
                    .child_impulses
                    .get(joint.child_b)
                    .copied()
                    .unwrap_or(0.0);
                imp_a.max(imp_b) <= joint.threshold
            });

            // Compute connected components from surviving joints.
            let components = fracture.connected_components();

            if components.len() <= 1 {
                continue;
            }

            // Keep the largest component on the original body.
            let largest_idx = components
                .iter()
                .enumerate()
                .max_by_key(|(_, c)| c.len())
                .map(|(i, _)| i)
                .unwrap_or(0);

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
                if comp_idx == largest_idx {
                    continue;
                }
                let mut group = Vec::new();
                for &child_idx in component {
                    if child_idx < trigger.collider_handles.len() {
                        if let Some(info) = snapshot_child(
                            &physics,
                            trigger.collider_handles[child_idx],
                            body_pos,
                            body_rot,
                            body_lin_vel,
                            body_ang_vel,
                        ) {
                            group.push(info);
                        }
                    }
                }
                split_groups.push(group);
            }

            // Detach the split-off children from the compound body.
            let mut handles_to_detach: Vec<ColliderHandle> = Vec::new();
            for (comp_idx, component) in components.iter().enumerate() {
                if comp_idx == largest_idx {
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

            // The survivors are no longer laid out around the body origin, so
            // move the origin onto them. Without this the remnant spins about
            // the vanished compound's centre — a plank pivoting on a phantom
            // axle metres away, too sluggish to push straight.
            physics.world.recenter_on_colliders(trigger.body_handle);

            // Spawn each split-off child as an independent body.
            for group in &split_groups {
                for info in group {
                    spawn_freed_piece(
                        &mut physics,
                        &entities,
                        &lazy,
                        info,
                        material,
                        piece_mesh,
                        body_ang_vel,
                        last_impulses,
                    );
                }
            }

            // Update the fracture component: keep only the largest component,
            // remap child indices.
            let kept = &components[largest_idx];
            let new_count = kept.len();
            let Some(fracture) = fractures.get_mut(trigger.entity) else {
                continue;
            };
            fracture.remap_children(kept, new_count);

            // Rebuild the model for the remaining compound body.
            rebuild_compound_model(
                &physics,
                trigger.body_handle,
                trigger.entity,
                material,
                piece_mesh,
                &mut models,
            );
        }
    }
}

/// Trigger data collected from the join pass.
struct FractureTrigger {
    entity: specs::Entity,
    body_handle: crate::physics::RigidBodyHandle,
    collider_handles: Vec<ColliderHandle>,
    /// Per-child impulse magnitudes, indexed by child position in collider list.
    child_impulses: Vec<f32>,
}

/// Snapshot of a child collider's state, captured before detachment.
struct ChildSnapshot {
    shape: ColliderShape,
    mass: f32,
    restitution: f32,
    friction: FrictionModel,
    world_pos: Point3<f32>,
    world_rot: nalgebra::UnitQuaternion<f32>,
    lin_vel: Vector3<f32>,
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
        shape: collider.shape().clone(),
        mass: collider.mass(),
        restitution: collider.material().restitution,
        friction: collider.material().friction,
        world_pos,
        world_rot: world_xform.rotation,
        lin_vel: body_lin_vel + body_ang_vel.cross(&r),
    })
}

#[allow(clippy::too_many_arguments)]
fn spawn_freed_piece(
    physics: &mut PhysicsResource,
    entities: &Entities,
    lazy: &specs::LazyUpdate,
    info: &ChildSnapshot,
    material: crate::rendering::material::MaterialId,
    piece_mesh: PieceMesh,
    body_ang_vel: Vector3<f32>,
    impulse_sources: &[crate::physics::PhysicsImpulse],
) {
    // Apply explosion impulse directly to this piece's mass so light
    // pieces fly off faster than heavy ones.
    let explosion_kick: Vector3<f32> = impulse_sources
        .iter()
        .filter_map(|imp| imp.impulse_at(info.world_pos))
        .sum();
    let piece_vel = info.lin_vel + explosion_kick / info.mass;

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

    let piece_model = match &info.shape {
        ColliderShape::Box { half_extents } => piece_model(*half_extents, piece_mesh, material),
        _ => return,
    };

    lazy.create_entity(entities)
        .with(Position(info.world_pos.coords))
        .with(Velocity(piece_vel))
        .with(Orientation(info.world_rot))
        .with(RigidBodyComponent(new_body_handle))
        .with(ModelInstance::new(piece_model))
        .with(Renderable)
        .build();
}

fn rebuild_compound_model(
    physics: &PhysicsResource,
    body_handle: crate::physics::RigidBodyHandle,
    entity: specs::Entity,
    material: crate::rendering::material::MaterialId,
    piece_mesh: PieceMesh,
    models: &mut WriteStorage<ModelInstance>,
) {
    let Some(body) = physics.world.body(body_handle) else {
        return;
    };
    let remaining: Vec<_> = body
        .colliders()
        .iter()
        .filter_map(|ch| {
            let c = physics.world.collider(*ch)?;
            let he = match c.shape() {
                ColliderShape::Box { half_extents } => *half_extents,
                _ => return None,
            };
            let offset = c.offset().translation.vector;
            let rotation = c.offset().rotation;
            if !he.x.is_finite()
                || !he.y.is_finite()
                || !he.z.is_finite()
                || !offset.x.is_finite()
                || !offset.y.is_finite()
                || !offset.z.is_finite()
            {
                log::error!(
                    "Fracture: NaN/Inf in remaining collider: he={:?} offset={:?}",
                    he,
                    offset
                );
                return None;
            }
            // Rotation carried through: a child that was laid at an angle
            // must still be at that angle after the break, or an object made
            // of tilted pieces straightens itself out the moment it loses one.
            Some(PiecePlacement::new(he, offset).rotated(rotation))
        })
        .collect();

    if remaining.is_empty() {
        return;
    }
    let new_model = compound_model(&remaining, piece_mesh, material);

    if let Some(model_inst) = models.get_mut(entity) {
        model_inst.model = new_model;
    }
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
