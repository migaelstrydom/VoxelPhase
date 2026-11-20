//! The bottom of the world. Below a level's lowest terrain nothing can catch
//! a body, so whatever falls past it is gone:
//!
//! ```text
//!   lowest terrain ─────────────────────────── TerrainWorld::bounds().min.y
//!                      FALL_MARGIN
//!   world floor ─────────────────────────────── past it: the player ─▶ LevelRestart
//!                                                        any other ─▶ removed
//! ```
//!
//! The player falling past it asks for the level to begin again, as R does;
//! the frame loop reads [`LevelRestart`] after the frame. Any other dynamic
//! body leaves the physics world, and its entity the ECS, rather than fall
//! for ever awake.

use specs::{Entities, Join, Read, ReadStorage, System, Write};

use super::PhysicsResource;
use crate::components::RigidBodyComponent;
use crate::player::Player;
use crate::terrain::TerrainWorld;

/// How far below the level's lowest terrain the world's floor lies, m.
pub const FALL_MARGIN: f32 = 4.0;

/// The height below which nothing over `terrain` can be caught.
pub fn world_floor(terrain: &TerrainWorld) -> f32 {
    terrain.bounds().min.y - FALL_MARGIN
}

/// Whether the level asks to begin again as it began.
#[derive(Debug, Default)]
pub struct LevelRestart {
    pub requested: bool,
}

/// Takes what has fallen past the world's floor out of the world.
pub struct OutOfWorldSystem;

impl<'a> System<'a> for OutOfWorldSystem {
    type SystemData = (
        Entities<'a>,
        Write<'a, PhysicsResource>,
        Option<Read<'a, TerrainWorld>>,
        Write<'a, LevelRestart>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Player>,
    );

    fn run(
        &mut self,
        (entities, mut physics, terrain, mut restart, bodies, players): Self::SystemData,
    ) {
        let Some(terrain) = terrain else {
            return;
        };
        let floor = world_floor(&terrain);
        for (entity, body) in (&entities, &bodies).join() {
            let fallen = physics
                .world
                .body(body.0)
                .is_some_and(|rigid| rigid.is_dynamic() && rigid.position().y < floor);
            if !fallen {
                continue;
            }
            if players.contains(entity) {
                restart.requested = true;
            } else {
                physics.world.remove_body(body.0);
                let _ = entities.delete(entity);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;
    use crate::level::{build_segments, resolve_placements, Level};
    use crate::physics::RigidBodyDesc;

    /// Ground a few metres thick, from -8 m to 0 m, hung on nothing.
    fn terrain() -> TerrainWorld {
        let mut level: Level = ron::from_str(
            r#"Level(
                name: "Floor fixture",
                segments: [(
                    name: "main",
                    terrain: Terrain(
                        voxel_size: 1.0,
                        bounds: (min: (0.0, -8.0, 0.0), max: (16.0, 4.0, 16.0)),
                        base_height: 0.0,
                        bedrock_thickness: 0.0,
                        features: [],
                    ),
                )],
                placements: [Root(segment: "main")],
                player_spawn: (8.0, 2.0, 8.0),
            )"#,
        )
        .expect("fixture level should parse");
        level.frames = resolve_placements(&level).expect("fixture placement should resolve");
        TerrainWorld::from_segments_headless(build_segments(&level).expect("fixture should build"))
    }

    fn world() -> World {
        let mut world = World::new();
        world.register::<RigidBodyComponent>();
        world.register::<Player>();
        world.insert(PhysicsResource::default());
        world.insert(LevelRestart::default());
        world.insert(terrain());
        world
    }

    fn spawn(world: &mut World, y: f32, desc: fn() -> RigidBodyDesc) -> specs::Entity {
        let handle = world
            .write_resource::<PhysicsResource>()
            .world
            .create_body(desc().position(Point3::new(8.0, y, 8.0)));
        world
            .create_entity()
            .with(RigidBodyComponent(handle))
            .build()
    }

    #[test]
    fn a_body_past_the_floor_is_taken_out_of_the_world() {
        let mut world = world();
        let floor = world_floor(&world.read_resource::<TerrainWorld>());
        let fallen = spawn(&mut world, floor - 0.5, RigidBodyDesc::dynamic);
        let above = spawn(&mut world, floor + 0.5, RigidBodyDesc::dynamic);
        let anchored = spawn(&mut world, floor - 0.5, RigidBodyDesc::static_body);
        let fallen_body = world
            .read_storage::<RigidBodyComponent>()
            .get(fallen)
            .unwrap()
            .0;

        OutOfWorldSystem.run_now(&world);
        world.maintain();

        assert!(!world.is_alive(fallen));
        assert!(world
            .read_resource::<PhysicsResource>()
            .world
            .body(fallen_body)
            .is_none());
        assert!(world.is_alive(above), "above the floor it may still land");
        assert!(world.is_alive(anchored), "what does not fall stays");
        assert!(!world.read_resource::<LevelRestart>().requested);
    }

    #[test]
    fn the_player_past_the_floor_restarts_the_level() {
        let mut world = world();
        let floor = world_floor(&world.read_resource::<TerrainWorld>());
        let player = spawn(&mut world, floor + 0.5, RigidBodyDesc::dynamic);
        world
            .write_storage::<Player>()
            .insert(player, Player)
            .unwrap();

        OutOfWorldSystem.run_now(&world);
        assert!(!world.read_resource::<LevelRestart>().requested);

        let body = world
            .read_storage::<RigidBodyComponent>()
            .get(player)
            .unwrap()
            .0;
        world
            .write_resource::<PhysicsResource>()
            .world
            .body_mut(body)
            .unwrap()
            .set_position(Point3::new(8.0, floor - 0.5, 8.0));
        OutOfWorldSystem.run_now(&world);
        world.maintain();

        assert!(world.read_resource::<LevelRestart>().requested);
        assert!(world.is_alive(player), "the restart takes the player back");
    }
}
