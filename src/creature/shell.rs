//! A brittle shell a creature wears, broken by the blow that kills it.
//!
//! ```text
//!   Harm::dying (a deadly blow) ──▶ ShellSystem, the frame it lands:
//!     each brittle part of its look, where it is drawn (PosedModel or
//!     ModelInstance)
//!        ──▶ triangles dealt to pieces (Cracks): to the nearest of a few
//!            seeds spread over it, each boxed; or to either side of a seam
//!            through its axis, each half a dome
//!        ──▶ each piece a body moving as the shell moved where it was,
//!            drawn as its own triangles (PartLooks), Debris
//!     what remains: drawn without its shell, colliding as the thinner
//!     capsule of what was inside; or nothing, its body gone into the pieces
//! ```
//!
//! A piece carries the motion the shell had where it was, and nothing more:
//! it flies on with a body the blow threw, and drops where a body that was
//! stopped dead stands.

use std::f32::consts::{FRAC_PI_2, TAU};
use std::sync::Arc;

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};
use smallvec::SmallVec;
use specs::{
    Component, DenseVecStorage, Entities, Entity, Join, ReadStorage, System, Write, WriteStorage,
};

use crate::collision::convex_hull::{ConvexHull, HullFace};
use crate::components::{
    ModelInstance, Orientation, PoseFrame, PosedModel, Position, Renderable, RigidBodyComponent,
    Velocity,
};
use crate::fracture::Debris;
use crate::harm::Harm;
use crate::model::{Look, MeshPrimitive, Model, PartLooks, Transform};
use crate::physics::{ColliderDesc, ColliderShape, PhysicsWorld, RigidBodyDesc, RigidBodyHandle};
use crate::systems::PhysicsResource;

/// How far a shard's box reaches across its face, as a share of the shard:
/// a box fitted to a curved shard stands proud of it at the corners, and
/// would overlap its neighbours'.
const BOX_INSET: f32 = 0.75;

/// How far each face of a piece's collider is drawn in, m: a hairline
/// between neighbours, which would otherwise start out flush or
/// overlapping.
const GAP: f32 = 0.005;

/// The thinnest a shard's box is, m.
const THINNEST_BOX: f32 = 0.008;

/// The corners round a dome, and its rings from the rim up to its top.
const DOME_SIDES: usize = 12;
const DOME_RINGS: usize = 4;

/// A brittle shell a creature wears: the parts of its look that are shell,
/// which a deadly blow breaks, and what is left of it once they are gone.
/// Taken off the creature as it breaks.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Shell {
    /// The parts of its look that are shell.
    pub parts: Vec<usize>,
    /// How the shell comes apart.
    pub cracks: Cracks,
    /// Density of a piece, kg/m³, where the creature lives on without its
    /// shell: light, as a shard's box holds more air than shell. Where
    /// nothing remains, the pieces share its whole weight instead.
    pub density: f32,
    /// What is left of it once its shell is gone: the radius of the capsule
    /// it then collides as, m, which the pieces lie outside; `None` if
    /// nothing is, the shell being all of it.
    pub remains: Option<f32>,
}

/// How a shell comes apart.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Cracks {
    /// Into `shards` pieces about the same size, anywhere.
    Anywhere { shards: usize },
    /// In two along a seam through its part's y axis, `seam` radians round
    /// it from +z toward +x, and straight across from it: each half a dome
    /// of the ball of `radius` it was, which falls open as a dome does.
    Halves { seam: f32, radius: f32 },
}

/// One piece of a shell, in the frame of the part it came from.
struct Shard {
    /// Where its collider sits and how it is turned: out of the shell along
    /// y.
    frame: Isometry3<f32>,
    /// What it collides as, in that frame.
    shape: ColliderShape,
    /// Its triangles, in that frame.
    look: Look,
}

impl Shard {
    fn volume(&self) -> f32 {
        self.shape.compute_mass(1.0)
    }
}

/// Breaks the shell of each creature struck a deadly blow this frame.
///
/// Runs after harm, which marks the blow, and after the creatures are
/// posed, so the pieces leave from where the shell is drawn.
pub struct ShellSystem;

impl<'a> System<'a> for ShellSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        WriteStorage<'a, Shell>,
        ReadStorage<'a, Harm>,
        WriteStorage<'a, PosedModel>,
        WriteStorage<'a, ModelInstance>,
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
            mut instances,
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
            let Some(body) = bodies.get(creature).map(|b| b.0) else {
                continue;
            };
            let Some(rigid) = physics.world.body(body) else {
                continue;
            };
            let (centre, linear, angular, mass) = (
                rigid.position(),
                rigid.linear_velocity(),
                rigid.angular_velocity(),
                rigid.mass(),
            );
            let itself = Isometry3::from_parts(Translation3::from(centre.coords), rigid.rotation());
            let Some((model, placed)) = drawn(posed.get(creature), instances.get(creature), itself)
            else {
                continue;
            };

            let mut bare = (*model).clone();
            let mut pieces = Vec::new();
            for &part in &shell.parts {
                let (Some(worn), Some(&(at, scale))) = (bare.parts.get_mut(part), placed.get(part))
                else {
                    continue;
                };
                let primitives = scaled(&worn.primitives, scale);
                pieces.extend(
                    shatter(&primitives, &shell.cracks)
                        .into_iter()
                        .map(|s| (at, s)),
                );
                worn.primitives.clear();
            }
            let density = match shell.remains {
                Some(_) => shell.density,
                None => {
                    mass / pieces
                        .iter()
                        .map(|(_, s)| s.volume())
                        .sum::<f32>()
                        .max(1e-6)
                }
            };

            for (at, shard) in pieces {
                let at = at * shard.frame;
                let moving = linear + angular.cross(&(at.translation.vector - centre.coords));
                let piece = physics.world.create_body(
                    RigidBodyDesc::dynamic()
                        .position(Point3::from(at.translation.vector))
                        .rotation(at.rotation)
                        .linear_velocity(moving)
                        .angular_velocity(angular),
                );
                let size = shard.volume();
                let desc = ColliderDesc {
                    shape: shard.shape,
                    ..ColliderDesc::sphere(1.0)
                };
                let Some(collider) = physics.world.attach_collider(piece, desc.density(density))
                else {
                    continue;
                };
                physics.world.wake_body(piece);
                let entity = entities.create();
                let _ = positions.insert(entity, Position(at.translation.vector));
                let _ = orientations.insert(entity, Orientation(at.rotation));
                let _ = velocities.insert(entity, Velocity(moving));
                let _ = bodies.insert(entity, RigidBodyComponent(piece));
                let _ = renderables.insert(entity, Renderable);
                let _ = looks.insert(entity, PartLooks::new([(collider, Arc::new(shard.look))]));
                let _ = debris.insert(entity, Debris::new(creature, size));
            }

            match shell.remains {
                Some(radius) => {
                    let bare = Arc::new(bare);
                    if let Some(posed) = posed.get_mut(creature) {
                        posed.model = bare;
                    } else if let Some(instance) = instances.get_mut(creature) {
                        instance.model = bare;
                    }
                    thin(&mut physics.world, body, radius);
                }
                None => {
                    physics.world.remove_body(body);
                    let _ = entities.delete(creature);
                }
            }
        }
    }
}

/// The model a creature is drawn as, and where each of its parts is drawn
/// and how it is scaled, for a creature whose body stands at `itself`:
/// posed part by part, or one model in its own frame.
fn drawn(
    posed: Option<&PosedModel>,
    instance: Option<&ModelInstance>,
    itself: Isometry3<f32>,
) -> Option<(Arc<Model>, Vec<(Isometry3<f32>, Vector3<f32>)>)> {
    let place = |frame: Isometry3<f32>, pose: &Transform| {
        let at = frame * Isometry3::from_parts(Translation3::from(pose.translation), pose.rotation);
        (at, pose.scale)
    };
    if let Some(posed) = posed {
        let frame = match posed.frame {
            PoseFrame::Position => {
                Isometry3::from_parts(itself.translation, UnitQuaternion::identity())
            }
            PoseFrame::Entity => itself,
        };
        let placed = (0..posed.model.parts.len())
            .map(|part| place(frame, &posed.poses.get(part).cloned().unwrap_or_default()))
            .collect();
        return Some((posed.model.clone(), placed));
    }
    let model = instance?.model.clone();
    let placed = model
        .parts
        .iter()
        .map(|part| place(itself, &part.local_transform))
        .collect();
    Some((model, placed))
}

/// `primitives` scaled by `scale`.
fn scaled(primitives: &[MeshPrimitive], scale: Vector3<f32>) -> Vec<MeshPrimitive> {
    let mut scaled = primitives.to_vec();
    for vertex in scaled
        .iter_mut()
        .flat_map(|primitive| &mut primitive.vertices)
    {
        vertex.pos = vertex.pos.component_mul(&scale);
    }
    scaled
}

/// The shell `primitives`, modelled about its middle, broken as `cracks`
/// says.
fn shatter(primitives: &[MeshPrimitive], cracks: &Cracks) -> Vec<Shard> {
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
    if triangles.is_empty() {
        return Vec::new();
    }
    let mine = |dealt: &[usize], piece: usize| {
        let dealt = dealt.to_vec();
        let triangles = triangles.iter().zip(dealt);
        triangles
            .filter(move |(_, dealt)| *dealt == piece)
            .map(|(&(p, t, _), _)| (p, t))
    };
    match *cracks {
        Cracks::Anywhere { shards } => {
            let dealt = spread(&triangles, shards);
            (0..shards)
                .filter_map(|piece| boxed(primitives, mine(&dealt, piece)))
                .collect()
        }
        Cracks::Halves { seam, radius } => {
            let across = Vector3::new(seam.cos(), 0.0, -seam.sin());
            let dealt: Vec<usize> = triangles
                .iter()
                .map(|(_, _, centre)| usize::from(centre.dot(&across) < 0.0))
                .collect();
            [across, -across]
                .into_iter()
                .enumerate()
                .filter_map(|(piece, out)| half(primitives, mine(&dealt, piece), out, radius))
                .collect()
        }
    }
}

/// Which of `count` seeds each of `triangles` is nearest: the seeds their
/// own centres, the first triangle's and then each the farthest from those
/// already chosen.
fn spread(triangles: &[(usize, usize, Vector3<f32>)], count: usize) -> Vec<usize> {
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

/// The triangles `mine` of `primitives` (each a primitive and a triangle in
/// it), as primitives of their own; `None` for none.
fn gathered(
    primitives: &[MeshPrimitive],
    mine: impl Iterator<Item = (usize, usize)>,
) -> Option<Vec<MeshPrimitive>> {
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
    (!pieces.is_empty()).then_some(pieces)
}

/// `pieces` as a shard in the frame at `origin` with its y along `out` and
/// its x along `across`, colliding as `shape` makes of its points there.
fn piece(
    mut pieces: Vec<MeshPrimitive>,
    origin: Vector3<f32>,
    out: Vector3<f32>,
    across: Vector3<f32>,
    shape: impl FnOnce(&[Vector3<f32>]) -> ColliderShape,
) -> Shard {
    let axes = Matrix3::from_columns(&[across, out, across.cross(&out)]);
    let turn = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(axes));
    let back = turn.inverse();
    for vertex in pieces.iter_mut().flat_map(|piece| &mut piece.vertices) {
        vertex.pos = back * (vertex.pos - origin);
        vertex.normal = back * vertex.normal;
    }
    let points: Vec<Vector3<f32>> = pieces
        .iter()
        .flat_map(|p| p.vertices.iter().map(|v| v.pos))
        .collect();
    Shard {
        frame: Isometry3::from_parts(Translation3::from(origin), turn),
        shape: shape(&points),
        look: Look { primitives: pieces },
    }
}

/// The half of a ball of `radius` the triangles `mine` of `primitives` lie
/// in, on the side of its middle `out` points to: a dome on the seam,
/// standing out along y, about its own centre of mass.
fn half(
    primitives: &[MeshPrimitive],
    mine: impl Iterator<Item = (usize, usize)>,
    out: Vector3<f32>,
    radius: f32,
) -> Option<Shard> {
    let pieces = gathered(primitives, mine)?;
    let dome = dome(radius - GAP, GAP);
    let centre = dome.centroid();
    let hull = Arc::new(dome.translated(-centre));
    let shape = |_: &[Vector3<f32>]| ColliderShape::ConvexHull { hull };
    Some(piece(pieces, out * centre.y, out, Vector3::y(), shape))
}

/// A shard of the triangles `mine` of `primitives` (each a primitive and a
/// triangle in it), boxed across its face and out of the shell from its
/// middle; `None` for none.
fn boxed(
    primitives: &[MeshPrimitive],
    mine: impl Iterator<Item = (usize, usize)>,
) -> Option<Shard> {
    let pieces = gathered(primitives, mine)?;
    let points: Vec<Vector3<f32>> = pieces
        .iter()
        .flat_map(|p| p.vertices.iter().map(|v| v.pos))
        .collect();
    let middle = points.iter().sum::<Vector3<f32>>() / points.len() as f32;
    let out = middle.try_normalize(1e-6).unwrap_or_else(Vector3::y);
    let across = points
        .iter()
        .map(|p| (p - middle) - out * out.dot(&(p - middle)))
        .max_by(|a, b| a.norm_squared().total_cmp(&b.norm_squared()))
        .and_then(|reach| reach.try_normalize(1e-6))
        .unwrap_or_else(|| {
            let side = out.cross(&Vector3::x()).try_normalize(1e-6);
            side.unwrap_or_else(Vector3::z)
        });
    let mut shard = piece(pieces, middle, out, across, |points| {
        let (mut low, mut high) = (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN));
        for point in points {
            low = low.inf(point);
            high = high.sup(point);
        }
        let reach = 0.5 * (high - low);
        let half_extents = Vector3::new(reach.x * BOX_INSET, reach.y, reach.z * BOX_INSET)
            .map(|half| (half - GAP).max(THINNEST_BOX));
        ColliderShape::Box { half_extents }
    });

    // The box sits about the middle of what it boxes.
    let (mut low, mut high) = (Vector3::repeat(f32::MAX), Vector3::repeat(f32::MIN));
    for vertex in shard.look.primitives.iter().flat_map(|p| &p.vertices) {
        low = low.inf(&vertex.pos);
        high = high.sup(&vertex.pos);
    }
    let centre = 0.5 * (low + high);
    for vertex in shard
        .look
        .primitives
        .iter_mut()
        .flat_map(|p| &mut p.vertices)
    {
        vertex.pos -= centre;
    }
    shard.frame *= Translation3::from(centre);
    Some(shard)
}

/// The hull of a dome of `radius` standing up y from a flat rim `base` m
/// above the origin.
fn dome(radius: f32, base: f32) -> ConvexHull {
    let corner = |ring: usize, side: usize| ring * DOME_SIDES + side % DOME_SIDES;
    let mut vertices: Vec<Vector3<f32>> = (0..DOME_RINGS)
        .flat_map(|ring| {
            let up = FRAC_PI_2 * ring as f32 / DOME_RINGS as f32;
            (0..DOME_SIDES).map(move |side| {
                let round = TAU * side as f32 / DOME_SIDES as f32;
                let (rise, reach) = (radius * up.sin(), radius * up.cos());
                Vector3::new(reach * round.cos(), base + rise, reach * round.sin())
            })
        })
        .collect();
    let top = vertices.len();
    vertices.push(Vector3::new(0.0, base + radius, 0.0));
    let inside = Vector3::new(0.0, base + 0.3 * radius, 0.0);

    let mut faces = vec![face(&vertices, (0..DOME_SIDES).collect(), inside)];
    for side in 0..DOME_SIDES {
        for ring in 0..DOME_RINGS - 1 {
            let quad = vec![
                corner(ring, side),
                corner(ring, side + 1),
                corner(ring + 1, side + 1),
                corner(ring + 1, side),
            ];
            faces.push(face(&vertices, quad, inside));
        }
        let last = DOME_RINGS - 1;
        let cap = vec![corner(last, side), corner(last, side + 1), top];
        faces.push(face(&vertices, cap, inside));
    }
    ConvexHull::new(vertices, faces)
}

/// The face through the corners `corners` of `vertices`, wound and facing
/// away from `inside`.
fn face(vertices: &[Vector3<f32>], mut corners: Vec<usize>, inside: Vector3<f32>) -> HullFace {
    let [a, b, c] = [0, 1, 2].map(|i| vertices[corners[i]]);
    let mut normal = (b - a).cross(&(c - a)).normalize();
    if normal.dot(&(a - inside)) < 0.0 {
        normal = -normal;
        corners[1..].reverse();
    }
    HullFace {
        vertex_indices: corners
            .iter()
            .map(|&i| i as u16)
            .collect::<SmallVec<[u16; 6]>>(),
        normal,
    }
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
    use crate::model::ModelPart;
    use crate::rendering::material::MaterialId;

    /// A hollow ball of shell, 0.25 m to its outside, about its middle.
    fn shell() -> Vec<MeshPrimitive> {
        let ball = Clay::new(0.0).add(Lump::ball(0.25, (0.0, 0.0, 0.0)), (1.0, 1.0, 1.0));
        vec![ball.hollow(0.02).skin(0.02, MaterialId(0))]
    }

    fn triangles(primitives: &[MeshPrimitive]) -> usize {
        primitives.iter().map(|p| p.indices.len() / 3).sum()
    }

    /// A world holding `shell` on a body of `collider`, struck a deadly
    /// blow, drawn as `drawn` gives it: after the shell system has run on
    /// it, with the creature and its body.
    fn broken(
        shell: Shell,
        collider: ColliderDesc,
        drawn: impl FnOnce(specs::EntityBuilder) -> specs::EntityBuilder,
    ) -> (World, Entity, RigidBodyHandle, f32) {
        let mut world = World::new();
        System::setup(&mut ShellSystem, &mut world);
        let mut physics = PhysicsResource::default();
        let body = physics
            .world
            .create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 1.0, 0.0)));
        physics.world.attach_collider(body, collider);
        let mass = physics.world.body(body).unwrap().mass();
        world.insert(physics);
        let struck = Harm {
            dying: Some(1.0),
            ..Harm::default()
        };
        let creature = world
            .create_entity()
            .with(shell)
            .with(struck)
            .with(RigidBodyComponent(body));
        let creature = drawn(creature).build();
        ShellSystem.run_now(&world);
        world.maintain();
        (world, creature, body, mass)
    }

    /// Every triangle goes to one shard, and each shard is boxed across its
    /// face and out of the shell: thinner out than across.
    #[test]
    fn a_shell_breaks_into_the_shards_asked_for_and_loses_nothing() {
        let shell = shell();
        let shards = shatter(&shell, &Cracks::Anywhere { shards: 8 });
        assert_eq!(shards.len(), 8);
        let dealt: usize = shards.iter().map(|s| triangles(&s.look.primitives)).sum();
        assert_eq!(dealt, triangles(&shell));
        for shard in &shards {
            let out = shard.frame.rotation * Vector3::y();
            let middle = shard.frame.translation.vector.normalize();
            assert!(out.dot(&middle) > 0.95, "{out:?} against {middle:?}");
            let ColliderShape::Box { half_extents } = shard.shape else {
                panic!("a shard is boxed");
            };
            assert!(
                half_extents.y < half_extents.x.min(half_extents.z),
                "{half_extents:?}"
            );
        }
    }

    /// Split along a seam, a shell comes apart in two halves, one each side
    /// of it, each a dome standing clear of the seam on its own side, and
    /// drawn there.
    #[test]
    fn a_shell_split_along_a_seam_comes_apart_in_two_domes() {
        let shell = shell();
        let halves = shatter(
            &shell,
            &Cracks::Halves {
                seam: 0.0,
                radius: 0.25,
            },
        );
        assert_eq!(halves.len(), 2);
        let dealt: usize = halves.iter().map(|h| triangles(&h.look.primitives)).sum();
        assert_eq!(dealt, triangles(&shell));
        for (half, side) in halves.iter().zip([1.0, -1.0]) {
            let out = half.frame.rotation * Vector3::y();
            assert!((out - Vector3::x() * side).norm() < 1e-5, "{out:?}");
            let ColliderShape::ConvexHull { hull } = &half.shape else {
                panic!("a half is a dome");
            };
            assert!(hull.centroid().norm() < 1e-4, "about its centre of mass");
            let across_seam = |v: &Vector3<f32>| (half.frame * Point3::from(*v)).coords.dot(&out);
            let rim = hull
                .vertices
                .iter()
                .map(across_seam)
                .fold(f32::MAX, f32::min);
            assert!(rim > 0.0, "the dome stands clear of the seam: {rim}");
            for vertex in half.look.primitives.iter().flat_map(|p| &p.vertices) {
                let drawn = across_seam(&vertex.pos);
                assert!(drawn > -0.05, "drawn on its own side: {drawn}");
            }
        }
    }

    /// Struck a deadly blow, a creature's shell comes off in shards, each a
    /// body of its own; it is drawn without it, and collides as the thinner
    /// capsule of what is left, about the same axis and as heavy.
    #[test]
    fn a_deadly_blow_breaks_the_shell_off_in_shards() {
        let shell = Shell {
            parts: vec![0],
            cracks: Cracks::Anywhere { shards: 6 },
            density: 300.0,
            remains: Some(0.1),
        };
        let posed = PosedModel {
            model: Arc::new(Model::flat(vec![ModelPart::new(self::shell())])),
            poses: vec![Transform {
                translation: Vector3::new(0.0, 0.6, 0.0),
                ..Transform::default()
            }],
            frame: PoseFrame::Position,
        };
        let capsule = ColliderDesc::capsule(1.0, 0.3);
        let (world, creature, body, mass) = broken(shell, capsule, |c| c.with(posed));

        assert!(world.read_storage::<Shell>().get(creature).is_none());
        assert_eq!((&world.read_storage::<Debris>()).join().count(), 6);
        let posed = world.read_storage::<PosedModel>();
        let worn = &posed.get(creature).unwrap().model.parts[0];
        assert!(worn.primitives.is_empty());
        let physics = world.read_resource::<PhysicsResource>();
        let rigid = physics.world.body(body).unwrap();
        let shape: Vec<ColliderShape> = rigid
            .colliders()
            .iter()
            .map(|&c| physics.world.collider(c).unwrap().shape().clone())
            .collect();
        assert!(
            matches!(shape[..], [ColliderShape::Capsule { half_height, radius }]
                if (half_height - 0.8).abs() < 1e-5 && radius == 0.1),
            "{shape:?}"
        );
        assert!((rigid.mass() - mass).abs() < 1e-3 * mass);
    }

    /// A creature that is all shell is gone once it breaks, body and all,
    /// its whole weight in its two halves.
    #[test]
    fn a_creature_that_is_all_shell_goes_into_its_halves() {
        let shell = Shell {
            parts: vec![0],
            cracks: Cracks::Halves {
                seam: 0.0,
                radius: 0.25,
            },
            density: 2400.0,
            remains: None,
        };
        let model = Arc::new(Model::flat(vec![ModelPart::new(self::shell())]));
        let ball = ColliderDesc::sphere(0.25).density(2400.0);
        let (world, creature, body, mass) =
            broken(shell, ball, |c| c.with(ModelInstance::new(model)));

        assert!(!world.is_alive(creature));
        let physics = world.read_resource::<PhysicsResource>();
        assert!(physics.world.body(body).is_none());
        let bodies = world.read_storage::<RigidBodyComponent>();
        let debris = world.read_storage::<Debris>();
        let halves: Vec<f32> = (&debris, &bodies)
            .join()
            .map(|(_, b)| physics.world.body(b.0).unwrap().mass())
            .collect();
        assert_eq!(halves.len(), 2);
        let whole: f32 = halves.iter().sum();
        assert!((whole - mass).abs() < 1e-3 * mass, "{halves:?} of {mass}");
    }
}
