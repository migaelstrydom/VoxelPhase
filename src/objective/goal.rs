//! The place a level ends.

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, Join, ReadStorage, System, Write, WriteStorage};

use crate::components::{MaterialModulation, Position};
use crate::debug::DebugLines;
use crate::player::Player;
use crate::rendering::material::SurfaceModulation;

use super::progress::LevelProgress;

/// The beacon that finishes the level when the player reaches it.
///
/// Carried by the beacon entity itself, so the thing that decides the level is
/// over is the same thing the player is looking at.
#[derive(Component, Clone, Copy, Debug)]
#[storage(DenseVecStorage)]
pub struct Goal {
    /// Horizontal distance from the beacon that counts as arriving.
    pub radius: f32,

    /// Gems that must be caught first. Zero makes the goal a plain finish line.
    pub required_gems: u32,
}

impl Goal {
    /// Whether a player at `player` is standing in the goal at `goal`.
    ///
    /// Horizontal distance against the radius, with a separate and generous
    /// vertical band: the beacon is metres tall and the player may be a step up
    /// or mid-jump, but the floor below must not count.
    pub fn contains(&self, goal: Vector3<f32>, player: Vector3<f32>) -> bool {
        let dx = player.x - goal.x;
        let dz = player.z - goal.z;
        let horizontal = (dx * dx + dz * dz).sqrt();

        horizontal <= self.radius && (player.y - goal.y).abs() <= VERTICAL_REACH
    }
}

/// How far above or below the beacon's own point the player may be. Wide
/// enough to cover a jump and a plinth, narrow enough that a walkway overhead
/// is not the goal.
const VERTICAL_REACH: f32 = 3.0;

/// Emissive multiplier applied to a beacon that has been reached.
const COMPLETED_GLOW_SCALE: f32 = 2.2;

/// Finishes the level when the player reaches a goal with the gems it asks for.
pub struct GoalSystem;

impl<'a> System<'a> for GoalSystem {
    type SystemData = (
        ReadStorage<'a, Player>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Goal>,
        WriteStorage<'a, MaterialModulation>,
        Write<'a, LevelProgress>,
        Write<'a, DebugLines>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (players, positions, goals, mut modulations, mut progress, mut debug) = data;

        let Some(player_position) = (&players, &positions)
            .join()
            .next()
            .map(|(_, position)| position.0)
        else {
            return;
        };

        let mut reached = false;
        let mut shortfall: Option<u32> = None;

        for (goal, position) in (&goals, &positions).join() {
            if !goal.contains(position.0, player_position) {
                continue;
            }

            let outstanding = progress.gems_outstanding(goal.required_gems);
            if outstanding > 0 {
                shortfall = Some(shortfall.map_or(outstanding, |n: u32| n.min(outstanding)));
                continue;
            }
            reached = true;
        }

        if reached {
            progress.complete();
        } else if let Some(missing) = shortfall {
            if !progress.is_complete() {
                let plural = if missing == 1 { "gem" } else { "gems" };
                debug.add(HUD_GOAL_KEY, format!("need {missing} more {plural}"));
            }
        }

        if progress.is_complete() {
            for (_, modulation) in (&goals, &mut modulations).join() {
                modulation.0 = SurfaceModulation::emissive(COMPLETED_GLOW_SCALE);
            }
        }
    }
}

/// The hint shown while the player stands in a goal they cannot yet open.
pub const HUD_GOAL_KEY: &str = "!! Goal";

#[cfg(test)]
mod tests {
    use super::*;

    fn goal() -> Goal {
        Goal {
            radius: 2.0,
            required_gems: 3,
        }
    }

    #[test]
    fn the_goal_is_a_column_not_a_sphere() {
        let at = Vector3::new(10.0, 4.0, 10.0);

        assert!(goal().contains(at, Vector3::new(11.9, 6.0, 10.0)));
        assert!(!goal().contains(at, Vector3::new(12.5, 4.0, 10.0)));
        // Directly below, but a storey down: a different floor, not the goal.
        assert!(!goal().contains(at, Vector3::new(10.0, -0.5, 10.0)));
    }

    #[test]
    fn arriving_without_the_gems_does_not_finish_the_level() {
        let mut progress = LevelProgress::default();
        progress.collect_gem();

        assert_eq!(progress.gems_outstanding(goal().required_gems), 2);
        assert!(!progress.is_complete());
    }

    #[test]
    fn the_last_gem_opens_the_goal() {
        let mut progress = LevelProgress::default();
        for _ in 0..3 {
            progress.collect_gem();
        }
        progress.elapsed = 7.5;

        assert_eq!(progress.gems_outstanding(goal().required_gems), 0);
        assert!(progress.complete());
        assert_eq!(progress.completed, Some(7.5));
    }
}
