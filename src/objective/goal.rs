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

    /// Where the ground it stands on is, from its position, m: what the
    /// player must be standing level with to arrive.
    pub ground: Vector3<f32>,
}

impl Goal {
    /// A goal not yet reached, arrived at within `radius` of a position
    /// that is its ground.
    pub fn new(radius: f32) -> Self {
        Self {
            radius,
            reached: false,
            ruined: false,
            ground: Vector3::zeros(),
        }
    }

    /// The same goal, standing on ground `ground` from its position.
    pub fn standing_on(self, ground: Vector3<f32>) -> Self {
        Self { ground, ..self }
    }

    /// Whether a player whose middle is at `player` is standing in the goal
    /// at `goal`.
    ///
    /// Horizontal distance against the radius, and the player's middle in a
    /// band over the goal's ground: on it, a step up or mid-jump counts; a
    /// floor below it does not, nor a walkway overhead.
    pub fn contains(&self, goal: Vector3<f32>, player: Vector3<f32>) -> bool {
        let ground = goal + self.ground;
        let dx = player.x - ground.x;
        let dz = player.z - ground.z;
        let horizontal = (dx * dx + dz * dz).sqrt();
        let rise = player.y - ground.y;

        horizontal <= self.radius && (-BELOW_GROUND..=ABOVE_GROUND).contains(&rise)
    }
}

/// How far below the goal's ground the player's middle may be, m: standing
/// in a shallow dip beside it, but not on a floor a storey down.
const BELOW_GROUND: f32 = 0.5;
/// How far above it, m: a step up beside it, or the top of a jump.
const ABOVE_GROUND: f32 = 2.5;

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

        assert!(goal().contains(at, Vector3::new(11.9, 4.8, 10.0)));
        assert!(goal().contains(at, Vector3::new(10.0, 6.0, 10.0)));
        assert!(!goal().contains(at, Vector3::new(12.5, 4.8, 10.0)));
        // Directly below, but on a floor two metres down: not the goal.
        assert!(!goal().contains(at, Vector3::new(10.0, 2.8, 10.0)));
        // Nor a walkway overhead.
        assert!(!goal().contains(at, Vector3::new(10.0, 7.5, 10.0)));
    }

    /// A brazier's position is its middle; its goal is judged from the
    /// ground it stands on.
    #[test]
    fn the_goal_is_judged_from_its_ground() {
        let middle = Vector3::new(0.0, 0.5, 0.0);
        let goal = goal().standing_on(Vector3::new(0.0, -0.5, 0.0));

        assert!(goal.contains(middle, Vector3::new(1.0, 0.8, 0.0)));
        assert!(!goal.contains(middle, Vector3::new(1.0, -1.2, 0.0)));
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
