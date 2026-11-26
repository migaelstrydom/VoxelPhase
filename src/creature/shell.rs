//! A brittle shell a creature wears, broken off in shards by the blow that
//! kills it.
//!
//! ```text
//!   Harm::dying (a deadly blow) ──▶ ShellSystem, the frame it lands:
//!     each brittle part of its PosedModel, where it is drawn
//!        ──▶ triangles dealt to the nearest of a few seeds ──▶ shards,
//!              each a body moving as the shell moved where it was, a box
//!              fitted to it, drawn as its own triangles (PartLooks), Debris
//!     its model drawn without them, and it collides as the thinner capsule
//!     of what is left inside (`bare_radius`)
//! ```
//!
//! A shard carries the motion the shell had where it was, and nothing more:
//! it flies on with a body the blow threw, and drops where a body that was
//! stopped dead stands.

use std::sync::Arc;

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use specs::{
    Component, DenseVecStorage, Entities, Entity, Join, ReadStorage, System, Write, WriteStorage,
};

use crate::components::{
    Orientation, PoseFrame, PosedModel, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::fracture::Debris;
use crate::harm::Harm;
use crate::model::{Look, MeshPrimitive, PartLooks, Transform};
use crate::physics::{ColliderDesc, ColliderShape, PhysicsWorld, RigidBodyDesc, RigidBodyHandle};
use crate::systems::PhysicsResource;

/// How far a shard's box reaches across its face, as a share of the shard:
/// a box fitted to a curved shard stands proud of it at the corners, and
/// would overlap its neighbours'.
const BOX_INSET: f32 = 0.75;

/// The thinnest a shard's box is, m.
const THINNEST_BOX: f32 = 0.008;

/// A brittle shell a creature wears: the parts of its look that are shell,
/// which a deadly blow breaks into shards, and what is left of it once they
/// are gone. Taken off the creature as it breaks.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Shell {
    /// The parts of its posed model that are shell.
    pub parts: Vec<usize>,
    /// How many shards the shell breaks into.
    pub shards: usize,
    /// Density of a shard's box, kg/m³: light, as the box holds more air
    /// than shell.
    pub density: f32,
    /// Radius of the capsule it collides as once its shell is gone, m: what
    /// is left inside. Its shards lie outside it.
    pub bare_radius: f32,
}

/// One shard of a shell, in the frame of the part it came from.
struct Shard {
    /// Where its box sits and how it is turned: across its face along x
    /// and z, out of the shell along y.
    frame: Isometry3<f32>,
    /// Its box's half extents.
    half_extents: Vector3<f32>,
    /// Its triangles, in its box's frame.
    look: Look,
}

/// Breaks the shell of each creature struck a deadly blow this frame.
///
/// Runs after harm, which marks the blow, and after the creatures are
/// posed, so the shards leave from where the shell is drawn.
pub struct ShellSystem;

impl<'a> System<'a> for ShellSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        WriteStorage<'a, Shell>,
        ReadStorage<'a, Harm>,
        WriteStorage<'a, PosedModel>,
        WriteStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, Position>,
        WriteStorage<'a, Orientation>,
        WriteStorage<'a, Velocity>,
        WriteStorage<'a, Renderable>,
        WriteStorage<'a, PartLooks>,
        WriteStorage<'a, Debris>,
    );

    fn run(
        &mut self,
        (
            entities,
            mut physics,
            mut shells,
            harms,
            mut posed,
            mut bodies,
            mut positions,
            mut orientations,
            mut velocities,
            mut renderables,
            mut looks,
            mut debris,
        ): Self::SystemData,
    ) {
        let struck: Vec<Entity> = (&entities, &shells, &harms)
            .join()
            .filter(|(_, _, harm)| harm.dying.is_some())
            .map(|(entity, _, _)| entity)
            .collect();
        for creature in struck {
            let Some(shell) = shells.remove(creature) else {
                continue;
            };
            let (Some(body), Some(model)) =
                (bodies.get(creature).map(|b| b.0), posed.get_mut(creature))
            else {
                continue;
            };
            let Some(rigid) = physics.world.body(body) else {
                continue;
            };
            let (centre, linear, angular) = (
                rigid.position(),
                rigid.linear_velocity(),
                rigid.angular_velocity(),
            );
            let frame = match model.frame {
                PoseFrame::Position => Isometry3::translation(centre.x, centre.y, centre.z),
                PoseFrame::Entity => {
                    Isometry3::from_parts(Translation3::from(centre.coords), rigid.rotation())
                }
            };

            let mut bare = (*model.model).clone();
            for &part in &shell.parts {
                let Some(worn) = bare.parts.get_mut(part) else {
                    continue;
                };
                let pose = model.poses.get(part).cloned().unwrap_or_default();
                let placed = frame
                    * Isometry3::from_parts(Translation3::from(pose.translation), pose.rotation);
                let primitives = scaled(&worn.primitives, &pose);
                for shard in shatter(&primitives, shell.shards) {
                    let at = placed * shard.frame;
                    let moving = linear + angular.cross(&(at.translation.vector - centre.coords));
                    let piece = physics.world.create_body(
                        RigidBodyDesc::dynamic()
                            .position(Point3::from(at.translation.vector))
                            .rotation(at.rotation)
                            .linear_velocity(moving)
                            .angular_velocity(angular),
                    );
                    let Some(collider) = physics.world.attach_collider(
                        piece,
                        ColliderDesc::box_shape(shard.half_extents).density(shell.density),
                    ) else {
                        continue;
                    };
                    physics.world.wake_body(piece);
                    let size = shard.half_extents.iter().product::<f32>() * 8.0;
                    let entity = entities.create();
                    let _ = positions.insert(entity, Position(at.translation.vector));
                    let _ = orientations.insert(entity, Orientation(at.rotation));
                    let _ = velocities.insert(entity, Velocity(moving));
                    let _ = bodies.insert(entity, RigidBodyComponent(piece));
                    let _ = renderables.insert(entity, Renderable);
                    let _ =
                        looks.insert(entity, PartLooks::new([(collider, Arc::new(shard.look))]));
                    let _ = debris.insert(entity, Debris::new(creature, size));
                }
                worn.primitives.clear();
            }
            model.model = Arc::new(bare);
            thin(&mut physics.world, body, shell.bare_radius);
        }
    }
}

/// `primitives` as `pose` scales them.
fn scaled(primitives: &[MeshPrimitive], pose: &Transform) -> Vec<MeshPrimitive> {
    let mut scaled = primitives.to_vec();
    for vertex in scaled
        .iter_mut()
        .flat_map(|primitive| &mut primitive.vertices)
    {
        vertex.pos = vertex.pos.component_mul(&pose.scale);
    }
    scaled
}

/// The shell `primitives`, modelled about its middle, broken into `count`
/// shards: each triangle dealt to the nearest of `count` seeds spread over
/// it as far apart as they will go, and each shard boxed in its own frame,
/// across its face and out from the middle.
fn shatter(primitives: &[MeshPrimitive], count: usize) -> Vec<Shard> {
    let triangles: Vec<(usize, usize, Vector3<f32>)> = primitives
        .iter()
        .enumerate()
        .flat_map(|(p, primitive)| {
            primitive
                .indices
                .chunks_exact(3)
                .enumerate()
                .map(move |(t, corners)| {
                    let centre = corners
                        .iter()
                        .map(|&i| primitive.vertices[i as usize].pos)
                        .sum::<Vector3<f32>>()
                        / 3.0;
                    (p, t, centre)
                })
        })
        .collect();
    if triangles.is_empty() || count == 0 {
        return Vec::new();
    }
    let nearest = deal(&triangles, count);

    (0..count)
        .filter_map(|seed| {
            let mine = triangles
                .iter()
                .zip(&nearest)
                .filter(|(_, &near)| near == seed)
                .map(|(&(p, t, _), _)| (p, t));
            boxed(primitives, mine)
        })
        .collect()
}

/// Which of `count` seeds each of `triangles` is nearest: the seeds their
/// own centres, the first triangle's and then each the farthest from those
/// already chosen.
fn deal(triangles: &[(usize, usize, Vector3<f32>)], count: usize) -> Vec<usize> {
    let mut nearest = vec![0; triangles.len()];
    let mut distance = vec![f32::MAX; triangles.len()];
    let mut seed = triangles[0].2;
    for index in 0..count.min(triangles.len()) {
        for ((_, _, centre), (near, far)) in
            triangles.iter().zip(nearest.iter_mut().zip(&mut distance))
        {
            let d = (centre - seed).norm_squared();
            if d < *far {
                (*near, *far) = (index, d);
            }
        }
        let farthest = distance
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.total_cmp(b.1));
        seed = triangles[farthest.map_or(0, |(at, _)| at)].2;
    }
    nearest
}

/// A shard of the triangles `mine` of `primitives` (each a primitive and a
/// triangle in it), boxed; `None` for none.
fn boxed(
    primitives: &[MeshPrimitive],
    mine: impl Iterator<Item = (usize, usize)>,
) -> Option<Shard> {
    let mut pieces: Vec<MeshPrimitive> = primitives
        .iter()
        .map(|primitive| MeshPrimitive {
            vertices: Vec::new(),
            indices: Vec::new(),
            material: primitive.material,
        })
        .collect();
    let mut renumbered: Vec<Vec<u32>> = primitives
        .iter()
        .map(|p| vec![u32::MAX; p.vertices.len()])
        .collect();
    for (p, t) in mine {
        for &corner in &primitives[p].indices[3 * t..3 * t + 3] {
            let slot = &mut renumbered[p][corner as usize];
            if *slot == u32::MAX {
                *slot = pieces[p].vertices.len() as u32;
                pieces[p]
                    .vertices
                    .push(primitives[p].vertices[corner as usize]);
            }
            pieces[p].indices.push(*slot);
        }
    }
    pieces.retain(|piece| !piece.indices.is_empty());
    let points: Vec<Vector3<f32>> = pieces
        .iter()
        .flat_map(|p| p.vertices.iter().map(|v| v.pos))
        .collect();
    if points.is_empty() {
        return None;
    }
    let middle = points.iter().sum::<Vector3<f32>>() / points.len() as f32;
    let out = middle.try_normalize(1e-6).unwrap_or_else(Vector3::y);
    let across = points
        .iter()
        .map(|p| (p - middle) - out * out.dot(&(p - middle)))
        .max_by(|a, b| a.norm_squared().total_cmp(&b.norm_squared()))
        .and_then(|reach| reach.try_normalize(1e-6))
        .unwrap_or_else(|| {
            out.cross(&Vector3::x())
                .try_normalize(1e-6)
                .unwrap_or_else(Vector3::z)
        });
    let axes = Matrix3::from_columns(&[across, out, across.cross(&out)]);
    let turn = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(axes));
    let back = turn.inverse();

    let mut reach = Vector3::<f32>::zeros();
    for vertex in pieces.iter_mut().flat_map(|piece| &mut piece.vertices) {
        vertex.pos = back * (vertex.pos - middle);
        vertex.normal = back * vertex.normal;
        reach = reach.sup(&vertex.pos.abs());
    }
    let half_extents = Vector3::new(
        (reach.x * BOX_INSET).max(THINNEST_BOX),
        reach.y.max(THINNEST_BOX),
        (reach.z * BOX_INSET).max(THINNEST_BOX),
    );
    Some(Shard {
        frame: Isometry3::from_parts(Translation3::from(middle), turn),
        half_extents,
        look: Look { primitives: pieces },
    })
}

/// `body`'s capsule thinned to `radius` about the same axis: the same
/// length between its caps' centres, and the same weight.
fn thin(physics: &mut PhysicsWorld, body: RigidBodyHandle, radius: f32) {
    let Some(rigid) = physics.body(body) else {
        return;
    };
    let capsule = rigid.colliders().iter().copied().find_map(|handle| {
        let collider = physics.collider(handle)?;
        match *collider.shape() {
            ColliderShape::Capsule {
                half_height,
                radius: was,
            } if was > radius => Some((handle, half_height, was)),
            _ => None,
        }
    });
    let Some((handle, half_height, was)) = capsule else {
        return;
    };
    let Some(old) = physics.detach_collider(body, handle) else {
        return;
    };
    let shape = ColliderShape::Capsule {
        half_height: half_height - (was - radius),
        radius,
    };
    let volume = shape.compute_mass(1.0);
    let thinned = ColliderDesc {
        shape,
        offset: *old.offset(),
        density: old.mass() / volume.max(1e-6),
        material: *old.material(),
        bulk: old.bulk().cloned(),
        bluntness: None,
    };
    physics.attach_collider(body, thinned);
    physics.wake_body(body);
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;
    use crate::app::sculpt::{Clay, Lump};
    use crate::model::{Model, ModelPart};
    use crate::rendering::material::MaterialId;

    /// A hollow ball of shell, 0.25 m to its outside, about its middle.
    fn shell() -> Vec<MeshPrimitive> {
        let ball = Clay::new(0.0).add(Lump::ball(0.25, (0.0, 0.0, 0.0)), (1.0, 1.0, 1.0));
        vec![ball.hollow(0.02).skin(0.02, MaterialId(0))]
    }

    fn triangles(primitives: &[MeshPrimitive]) -> usize {
        primitives.iter().map(|p| p.indices.len() / 3).sum()
    }

    /// Every triangle goes to one shard, and each shard is boxed across its
    /// face and out of the shell: thinner out than across.
    #[test]
    fn a_shell_breaks_into_the_shards_asked_for_and_loses_nothing() {
        let shell = shell();
        let shards = shatter(&shell, 8);
        assert_eq!(shards.len(), 8);
        let dealt: usize = shards.iter().map(|s| triangles(&s.look.primitives)).sum();
        assert_eq!(dealt, triangles(&shell));
        for shard in &shards {
            let out = shard.frame.rotation * Vector3::y();
            let middle = shard.frame.translation.vector.normalize();
            assert!(out.dot(&middle) > 0.99, "{out:?} against {middle:?}");
            let across = shard.half_extents.x.min(shard.half_extents.z);
            assert!(shard.half_extents.y < across, "{:?}", shard.half_extents);
        }
    }

    /// Struck a deadly blow, a creature's shell comes off in shards, each a
    /// body of its own; it is drawn without it, and collides as the thinner
    /// capsule of what is left, about the same axis and as heavy.
    #[test]
    fn a_deadly_blow_breaks_the_shell_off_in_shards() {
        let mut world = World::new();
        System::setup(&mut ShellSystem, &mut world);
        let mut physics = PhysicsResource::default();
        let body = physics
            .world
            .create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 1.0, 0.0)));
        physics
            .world
            .attach_collider(body, ColliderDesc::capsule(1.0, 0.3));
        let mass = physics.world.body(body).unwrap().mass();
        world.insert(physics);
        let creature = world
            .create_entity()
            .with(Shell {
                parts: vec![0],
                shards: 6,
                density: 300.0,
                bare_radius: 0.1,
            })
            .with(Harm {
                dying: Some(1.0),
                ..Harm::default()
            })
            .with(PosedModel {
                model: Arc::new(Model::flat(vec![ModelPart::new(shell())])),
                poses: vec![Transform {
                    translation: Vector3::new(0.0, 0.6, 0.0),
                    ..Transform::default()
                }],
                frame: PoseFrame::Position,
            })
            .with(RigidBodyComponent(body))
            .build();

        ShellSystem.run_now(&world);
        world.maintain();

        assert!(world.read_storage::<Shell>().get(creature).is_none());
        assert_eq!((&world.read_storage::<Debris>()).join().count(), 6);
        let posed = world.read_storage::<PosedModel>();
        assert!(posed.get(creature).unwrap().model.parts[0]
            .primitives
            .is_empty());
        let physics = world.read_resource::<PhysicsResource>();
        let rigid = physics.world.body(body).unwrap();
        let shape = rigid
            .colliders()
            .iter()
            .map(|&c| physics.world.collider(c).unwrap().shape().clone())
            .collect::<Vec<_>>();
        assert!(
            matches!(shape[..], [ColliderShape::Capsule { half_height, radius }]
                if (half_height - 0.8).abs() < 1e-5 && radius == 0.1),
            "{shape:?}"
        );
        assert!((rigid.mass() - mass).abs() < 1e-3 * mass);
    }
}
