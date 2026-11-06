//! Terrain anchor systems: releasing anchored bodies when their terrain is
//! destroyed, and sharing an anchoring out when a compound splits.

use nalgebra::{Isometry3, Point3};
use specs::{Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::components::{ModelInstance, RigidBodyComponent, TerrainAnchored};
use crate::fracture::{CompoundFracture, PieceSplits};
use crate::physics::{ConstraintHandle, PhysicsWorld, RigidBodyHandle};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Checks terrain solidity under each anchored body, after an update that
/// changed the terrain around it, and releases the body when its ground is
/// gone: its constraints are removed, a compound forgets the ties they
/// were, and it meets static geometry again.
pub struct TerrainAnchorSystem;

impl<'a> System<'a> for TerrainAnchorSystem {
    type SystemData = (
        Entities<'a>,
        WriteStorage<'a, TerrainAnchored>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, ModelInstance>,
        WriteStorage<'a, CompoundFracture>,
        Write<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
    );

    fn run(
        &mut self,
        (entities, mut anchored, bodies, mut models, mut fractures, mut physics, terrain_opt): Self::SystemData,
    ) {
        let Some(terrain) = terrain_opt else {
            return;
        };
        // Ground only goes where the terrain changed.
        let changed = terrain.changed_regions();
        if changed.is_empty() {
            return;
        }

        let mut released = Vec::new();

        for (entity, anchor, body) in (&entities, &mut anchored, &bodies).join() {
            let terrain_gone = anchor
                .anchor_points
                .iter()
                .filter(|p| changed.iter().any(|region| region.contains_point(**p)))
                .any(|p| {
                    terrain
                        .mesh_surface_height_at(p.x, p.z)
                        .is_none_or(|surface_y| surface_y < p.y)
                });
            if terrain_gone {
                let mut fracture = fractures.get_mut(entity);
                for &constraint in &anchor.constraints {
                    physics.world.remove_constraint(constraint);
                    if let Some(fracture) = fracture.as_mut() {
                        fracture.untie(constraint);
                    }
                }
                physics.world.set_ignores_static(body.0, false);

                if let Some(released_model) = anchor.released_model.take() {
                    if let Some(model_instance) = models.get_mut(entity) {
                        model_instance.model = released_model;
                    }
                }

                released.push(entity);
            }
        }

        for entity in released {
            anchored.remove(entity);
        }
    }
}

/// Gives a piece split off an anchored compound its share of the anchoring:
/// the constraints that moved with it, and the anchor points nearer those
/// than any that stayed. The piece's body already keeps the compound's
/// `ignores_static`, since it took ties.
pub struct AnchorShareSystem;

impl<'a> System<'a> for AnchorShareSystem {
    type SystemData = (
        Read<'a, PieceSplits>,
        WriteStorage<'a, TerrainAnchored>,
        ReadStorage<'a, RigidBodyComponent>,
        Read<'a, PhysicsResource>,
    );

    fn run(&mut self, (splits, mut anchored, bodies, physics): Self::SystemData) {
        for split in splits.iter() {
            let (Some(piece_body), Some(from_body)) =
                (bodies.get(split.piece), bodies.get(split.from))
            else {
                continue;
            };
            let Some(anchor) = anchored.get_mut(split.from) else {
                continue;
            };
            let Some(share) = anchor_share(&physics.world, anchor, from_body.0, piece_body.0)
            else {
                continue;
            };
            let _ = anchored.insert(split.piece, share);
        }
    }
}

/// Take from `anchor`, on `from`, what now holds `piece`: `None` if nothing
/// moved.
fn anchor_share(
    world: &PhysicsWorld,
    anchor: &mut TerrainAnchored,
    from: RigidBodyHandle,
    piece: RigidBodyHandle,
) -> Option<TerrainAnchored> {
    let holds = |constraint: &ConstraintHandle, body| {
        world
            .constraint(*constraint)
            .is_some_and(|c| c.kind.references_body(body))
    };
    let (moved, stayed): (Vec<ConstraintHandle>, Vec<ConstraintHandle>) = anchor
        .constraints
        .iter()
        .partition(|constraint| holds(constraint, piece));
    if moved.is_empty() {
        return None;
    }

    let held_at = |constraints: &[ConstraintHandle], body| -> Vec<Point3<f32>> {
        let Some(rigid) = world.body(body) else {
            return Vec::new();
        };
        let pose = Isometry3::from_parts(rigid.position().coords.into(), rigid.rotation());
        constraints
            .iter()
            .filter_map(|c| world.constraint(*c)?.kind.anchor_on(body))
            .map(|local| pose * Point3::from(local))
            .collect()
    };
    let (moved_at, stayed_at) = (held_at(&moved, piece), held_at(&stayed, from));
    let nearest = |point: &Point3<f32>, places: &[Point3<f32>]| {
        places
            .iter()
            .map(|p| (p - point).norm())
            .fold(f32::INFINITY, f32::min)
    };
    let (piece_points, kept_points): (Vec<Point3<f32>>, Vec<Point3<f32>>) = anchor
        .anchor_points
        .iter()
        .partition(|point| nearest(point, &moved_at) < nearest(point, &stayed_at));

    anchor.constraints = stayed;
    anchor.anchor_points = kept_points;
    Some(TerrainAnchored {
        constraints: moved,
        anchor_points: piece_points,
        released_model: None,
    })
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;
    use specs::{Entity, RunNow, World, WorldExt};

    use super::*;
    use crate::app::world_builder::WorldBuilder;
    use crate::level::{resolve_placements, Level};
    use crate::level_check::build_terrain;
    use crate::rendering::material::MaterialId;
    use crate::terrain::BlastConfig;

    /// Where the menhir stands, on flat ground at y = 0.
    const MENHIR: (f32, f32) = (4.0, 4.0);

    /// A world with flat ground and one menhir bedded in it.
    fn menhir_world() -> (World, Entity) {
        let ron = format!(
            r#"
            Level(
                name: "Anchor",
                segments: [(
                    name: "main",
                    terrain: Terrain(
                        voxel_size: 1.0,
                        bounds: (min: (-32.0, -32.0, -32.0), max: (32.0, 32.0, 32.0)),
                        base_height: 0.0,
                        features: [],
                    ),
                    objects: [Menhir(pos: ({}, {}))],
                )],
                placements: [Root(segment: "main")],
                player_spawn: (0.0, 2.0, 0.0),
            )
            "#,
            MENHIR.0, MENHIR.1
        );
        let mut level: Level = ron::from_str(&ron).expect("anchor level should parse");
        level.frames = resolve_placements(&level).expect("anchor placement should resolve");

        let mut world = WorldBuilder::new()
            .with_default_resources()
            .build()
            .expect("a world with no device attached builds infallibly");
        world.insert(build_terrain(&level));
        let (_, object) = level.objects().next().expect("one object");
        let spawnable = object.to_spawnable();
        let materials = vec![MaterialId(0); spawnable.material_count()];
        let menhir = spawnable.spawn(&mut world, &materials)[0];
        (world, menhir)
    }

    /// Blow the ground at `at` away and publish the change, as a frame does.
    fn blast(world: &mut World, at: Point3<f32>) {
        let mut terrain = world.write_resource::<TerrainWorld>();
        terrain.detonate(at, &BlastConfig::default());
        terrain.update();
    }

    fn is_anchored(world: &World, menhir: Entity) -> bool {
        world.read_storage::<TerrainAnchored>().contains(menhir)
    }

    fn ignores_static(world: &World, menhir: Entity) -> bool {
        let body = world
            .read_storage::<RigidBodyComponent>()
            .get(menhir)
            .unwrap()
            .0;
        let physics = world.read_resource::<PhysicsResource>();
        physics.world.body(body).unwrap().ignores_static()
    }

    /// A bedded menhir passes through the ground it is welded into, and is
    /// left alone until the ground under it goes: by a blast somewhere else,
    /// or by none at all.
    #[test]
    fn a_menhir_stays_bedded_while_its_ground_stands() {
        let (mut world, menhir) = menhir_world();
        assert!(
            ignores_static(&world, menhir),
            "a bedded menhir should pass through the ground"
        );

        TerrainAnchorSystem.run_now(&world);
        blast(&mut world, Point3::new(-20.0, 0.0, -20.0));
        TerrainAnchorSystem.run_now(&world);

        assert!(
            is_anchored(&world, menhir),
            "a blast 30 m away released the menhir"
        );
        assert!(ignores_static(&world, menhir));
    }

    /// Once a blast takes the ground from under a menhir, it is released: its
    /// weld goes, and it meets the ground again.
    #[test]
    fn a_blast_under_a_menhir_releases_it() {
        let (mut world, menhir) = menhir_world();
        let weld = world
            .read_storage::<TerrainAnchored>()
            .get(menhir)
            .unwrap()
            .constraints[0];
        blast(&mut world, Point3::new(MENHIR.0, 0.0, MENHIR.1));
        TerrainAnchorSystem.run_now(&world);

        assert!(
            !is_anchored(&world, menhir),
            "the menhir stayed anchored over a crater"
        );
        assert!(
            !ignores_static(&world, menhir),
            "the released menhir still passes through the ground"
        );
        let physics = world.read_resource::<PhysicsResource>();
        assert!(
            physics.world.constraint(weld).is_none(),
            "the menhir's weld survived release"
        );
    }

    /// A compound bedded in the ground forgets its tie to the weld a blast
    /// under it removes, so a later split does not try to move it.
    #[test]
    fn a_released_compound_forgets_its_tie() {
        use crate::fracture::PartGraph;

        let (mut world, menhir) = menhir_world();
        let weld = world
            .read_storage::<TerrainAnchored>()
            .get(menhir)
            .unwrap()
            .constraints[0];
        let collider = {
            let body = world
                .read_storage::<RigidBodyComponent>()
                .get(menhir)
                .unwrap()
                .0;
            let physics = world.read_resource::<PhysicsResource>();
            physics.world.body(body).unwrap().colliders()[0]
        };
        let mut graph = PartGraph::default();
        let part = graph.add_part([collider]);
        graph.tie(weld, part);
        world
            .write_storage::<CompoundFracture>()
            .insert(menhir, CompoundFracture::new(graph))
            .unwrap();

        blast(&mut world, Point3::new(MENHIR.0, 0.0, MENHIR.1));
        TerrainAnchorSystem.run_now(&world);

        let fractures = world.read_storage::<CompoundFracture>();
        assert!(
            fractures.get(menhir).unwrap().graph.ties().is_empty(),
            "the compound still names the removed weld as a tie"
        );
    }

    /// A welded compound split in two: the piece takes the weld that moved
    /// with it and the anchor point beside it; the rest stays.
    #[test]
    fn a_split_piece_takes_its_share_of_the_anchoring() {
        use crate::physics::{ColliderDesc, ConstraintKind, PhysicsConfig, RigidBodyDesc};
        use nalgebra::{UnitQuaternion, Vector3};

        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = world.create_body(RigidBodyDesc::dynamic());
        let [left, right] = [-1.0f32, 1.0].map(|x| {
            world
                .attach_collider(
                    body,
                    ColliderDesc::box_shape(Vector3::repeat(0.4))
                        .offset_translation(Vector3::new(x, 0.0, 0.0)),
                )
                .unwrap()
        });
        let weld = |world: &mut PhysicsWorld, x: f32| {
            world.create_constraint(ConstraintKind::world_fixed(
                body,
                Point3::new(x, -0.4, 0.0),
                Vector3::new(x, -0.4, 0.0),
                &UnitQuaternion::identity(),
                0.0,
                f32::INFINITY,
            ))
        };
        let (left_weld, right_weld) = (weld(&mut world, -1.0), weld(&mut world, 1.0));
        let mut anchor = TerrainAnchored {
            constraints: vec![left_weld, right_weld],
            anchor_points: vec![Point3::new(-1.0, -0.5, 0.0), Point3::new(1.0, -0.5, 0.0)],
            released_model: None,
        };
        let traits = crate::fracture::PieceTraits::default().body(Default::default(), true);
        let piece = world
            .split_off(body, &[right], &[right_weld], traits)
            .unwrap()
            .body;

        let share = anchor_share(&world, &mut anchor, body, piece).expect("a weld moved");

        assert_eq!(share.constraints, vec![right_weld]);
        assert_eq!(share.anchor_points, vec![Point3::new(1.0, -0.5, 0.0)]);
        assert_eq!(anchor.constraints, vec![left_weld]);
        assert_eq!(anchor.anchor_points, vec![Point3::new(-1.0, -0.5, 0.0)]);
        let _ = left;
    }
}
