//! Terrain anchor system: releases anchored bodies when their terrain is destroyed.

use specs::{Entities, Join, Read, ReadStorage, System, Write, WriteStorage};

use crate::components::{ModelInstance, RigidBodyComponent, TerrainAnchored};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

/// Checks terrain solidity under each anchored body, after an update that
/// changed the terrain around it, and releases the body when its ground is
/// gone: its constraints are removed and it meets static geometry again.
pub struct TerrainAnchorSystem;

impl<'a> System<'a> for TerrainAnchorSystem {
    type SystemData = (
        Entities<'a>,
        WriteStorage<'a, TerrainAnchored>,
        ReadStorage<'a, RigidBodyComponent>,
        WriteStorage<'a, ModelInstance>,
        Write<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
    );

    fn run(
        &mut self,
        (entities, mut anchored, bodies, mut models, mut physics, terrain_opt): Self::SystemData,
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
                        .map_or(true, |surface_y| surface_y < p.y)
                });
            if terrain_gone {
                physics.world.remove_constraint(anchor.anchor_handle);
                physics.world.remove_constraint(anchor.upright_handle);
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
            .anchor_handle;
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
}
