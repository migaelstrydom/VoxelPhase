//! The place a level ends.

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, Join, ReadStorage, System, Write, WriteStorage};

use crate::components::Position;
use crate::player::Player;

use super::progress::LevelProgress;

/// The place that finishes the level when the player reaches it with every
/// solid the level holds.
///
/// Carried by the thing the player is looking at (a brazier), so what
/// decides the level is over is what answers it. A goal can be ruined (a
/// brazier knocked over): then it can never be reached, and a level whose
/// goals are all ruined is lost.
#[derive(Component, Clone, Copy, Debug)]
#[storage(DenseVecStorage)]
pub struct Goal {
    /// Horizontal distance from the goal that counts as arriving.
    pub radius: f32,

    /// Whether the player has arrived here with every solid.
    pub reached: bool,

    /// Whether it has been ruined and can no longer be reached.
    pub ruined: bool,
}

impl Goal {
    /// A goal not yet reached, arrived at within `radius`.
    pub fn new(radius: f32) -> Self {
        Self {
            radius,
            reached: false,
            ruined: false,
        }
    }

    /// Whether a player at `player` is standing in the goal at `goal`.
    ///
    /// Horizontal distance against the radius, with a separate and generous
    /// vertical band: the player may be a step up or mid-jump, but the floor
    /// below must not count.
    pub fn contains(&self, goal: Vector3<f32>, player: Vector3<f32>) -> bool {
        let dx = player.x - goal.x;
        let dz = player.z - goal.z;
        let horizontal = (dx * dx + dz * dz).sqrt();

        horizontal <= self.radius && (player.y - goal.y).abs() <= VERTICAL_REACH
    }
}

/// How far above or below the goal's own point the player may be. Wide
/// enough to cover a jump and a plinth, narrow enough that a walkway overhead
/// is not the goal.
const VERTICAL_REACH: f32 = 3.0;

/// Finishes the level when the player reaches a goal with every solid, and
/// loses it when every goal is ruined.
///
/// Arriving early does nothing: the goal is the level's score, not a
/// mechanism (`docs/WORLD_AND_STORY.md`).
pub struct GoalSystem;

impl<'a> System<'a> for GoalSystem {
    type SystemData = (
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, Goal>,
        Write<'a, LevelProgress>,
    );

    fn run(&mut self, (players, positions, mut goals, mut progress): Self::SystemData) {
        let mut any = false;
        let mut all_ruined = true;
        for goal in (&goals).join() {
            any = true;
            all_ruined &= goal.ruined;
        }
        if any && all_ruined {
            progress.forfeit();
        }

        if progress.solids_outstanding() > 0 || progress.lost {
            return;
        }
        let Some(player) = (&players, &positions)
            .join()
            .next()
            .map(|(_, position)| position.0)
        else {
            return;
        };

        for (goal, position) in (&mut goals, &positions).join() {
            if goal.ruined || goal.reached || !goal.contains(position.0, player) {
                continue;
            }
            goal.reached = true;
            progress.complete();
        }
    }
}

#[cfg(test)]
mod tests {
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;

    fn goal() -> Goal {
        Goal::new(2.0)
    }

    #[test]
    fn the_goal_is_a_column_not_a_sphere() {
        let at = Vector3::new(10.0, 4.0, 10.0);

        assert!(goal().contains(at, Vector3::new(11.9, 6.0, 10.0)));
        assert!(!goal().contains(at, Vector3::new(12.5, 4.0, 10.0)));
        // Directly below, but a storey down: a different floor, not the goal.
        assert!(!goal().contains(at, Vector3::new(10.0, -0.5, 10.0)));
    }

    /// A world with the player standing in one goal, and a level of
    /// `solids` solids with `taken` taken.
    fn scene(solids: u32, taken: u32, ruined: bool) -> (World, specs::Entity) {
        let mut world = World::new();
        world.register::<Player>();
        world.register::<Position>();
        world.register::<Goal>();
        let mut progress = LevelProgress::default();
        (0..solids).for_each(|_| progress.add_solid());
        (0..taken).for_each(|_| progress.take_solid());
        world.insert(progress);
        world
            .create_entity()
            .with(Player)
            .with(Position(Vector3::new(1.0, 0.0, 0.0)))
            .build();
        let mut placed = goal();
        placed.ruined = ruined;
        let entity = world
            .create_entity()
            .with(Position(Vector3::zeros()))
            .with(placed)
            .build();
        (world, entity)
    }

    #[test]
    fn arriving_without_every_solid_does_nothing() {
        let (world, entity) = scene(5, 4, false);
        GoalSystem.run_now(&world);

        assert!(!world.read_resource::<LevelProgress>().is_complete());
        assert!(!world.read_storage::<Goal>().get(entity).unwrap().reached);
    }

    #[test]
    fn arriving_with_every_solid_finishes_the_level_at_that_goal() {
        let (world, entity) = scene(5, 5, false);
        GoalSystem.run_now(&world);

        assert!(world.read_resource::<LevelProgress>().is_complete());
        assert!(world.read_storage::<Goal>().get(entity).unwrap().reached);
    }

    #[test]
    fn a_level_whose_only_goal_is_ruined_is_lost() {
        let (world, entity) = scene(0, 0, true);
        GoalSystem.run_now(&world);

        let progress = world.read_resource::<LevelProgress>();
        assert!(progress.lost);
        assert!(!progress.is_complete());
        assert!(!world.read_storage::<Goal>().get(entity).unwrap().reached);
    }
}
