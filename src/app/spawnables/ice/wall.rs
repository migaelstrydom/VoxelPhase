//! Ice wall — a course-by-course wall of ice bricks laid in a running bond.
//!
//! ```text
//!        ┌────┬────────┬────────┬────┐   odd course: half brick, full
//!        │    │        │        │    │   bricks, half brick
//!        ├────┴───┬────┴───┬────┴────┤
//!        │        │        │         │   even course: `columns` full bricks
//!        └────────┴────────┴─────────┘
//!        ◀──────── columns × brick ───▶
//! ```
//!
//! The half bricks closing the odd courses are the difference between a wall
//! and a stack of offset rows: they square the ends off, so the wall's
//! footprint is the rectangle its `columns` say it is however tall it gets.
//!
//! The bricks are loose bodies. A wall of them stands because each course rests
//! flat on the one below, which needs no friction — and ice has almost none, so
//! anything that leans on it or hits it from the side finds that out.

use nalgebra::Vector3;
use serde::Deserialize;
use specs::{Entity, World};

use super::super::shared::orientation::Yaw;
use super::super::shared::textures::hash_pair;
use super::super::{MaterialCtx, Spawnable};
use super::block::{ice_materials, texture_spread, IceBlock};
use crate::core::error::EngineResult;
use crate::rendering::material::MaterialId;

/// How many distinct ice textures a wall's bricks are drawn from.
///
/// Enough that no two neighbours are obviously twins; few enough that a wall
/// costs a handful of textures rather than one per brick.
const TEXTURE_VARIANTS: usize = 4;

#[derive(Deserialize)]
pub struct IceWallDef {
    /// Centre of the wall's bottom edge.
    pub base: (f32, f32, f32),

    /// Half-extents of one brick: length along the wall, height, thickness.
    #[serde(default = "IceWallDef::default_brick")]
    pub brick_half_extents: (f32, f32, f32),

    /// Bricks per course, counted in whole bricks.
    pub columns: u32,

    /// Courses stacked upward.
    pub rows: u32,

    /// Rotation about `+Y`, in degrees. The wall runs along its own `+X`, so
    /// this is the one field that decides which way it faces.
    #[serde(default)]
    pub yaw: f32,

    /// Offset alternate courses by half a brick. Off gives stack bond: every
    /// joint lines up, and the wall comes apart in columns.
    #[serde(default = "IceWallDef::default_stagger")]
    pub stagger: bool,
}

impl IceWallDef {
    pub fn default_brick() -> (f32, f32, f32) {
        (0.45, 0.2, 0.22)
    }

    pub fn default_stagger() -> bool {
        true
    }

    fn brick(&self) -> Vector3<f32> {
        Vector3::new(
            self.brick_half_extents.0,
            self.brick_half_extents.1,
            self.brick_half_extents.2,
        )
    }

    /// Half the wall's length, which is what the ends of every course line up
    /// against.
    fn half_length(&self) -> f32 {
        self.columns as f32 * self.brick_half_extents.0
    }

    /// The bricks of one course as `(centre x, half length)` pairs, in the
    /// wall's own axes.
    fn course(&self, row: u32) -> Vec<(f32, f32)> {
        let half_brick = self.brick_half_extents.0;
        let end = self.half_length();
        let offset = self.stagger && row % 2 == 1;

        if !offset {
            return (0..self.columns)
                .map(|column| (-end + half_brick * (2 * column + 1) as f32, half_brick))
                .collect();
        }

        // Half a brick at each end, whole bricks between them.
        let mut bricks = vec![(-end + half_brick * 0.5, half_brick * 0.5)];
        bricks.extend(
            (0..self.columns.saturating_sub(1))
                .map(|column| (-end + half_brick * (2 * column + 2) as f32, half_brick)),
        );
        if self.columns > 0 {
            bricks.push((end - half_brick * 0.5, half_brick * 0.5));
        }
        bricks
    }
}

impl Spawnable for IceWallDef {
    fn material_count(&self) -> usize {
        TEXTURE_VARIANTS
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        ice_materials(
            ctx,
            self.base,
            TEXTURE_VARIANTS,
            texture_spread(self.brick()),
        )
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let brick = self.brick();
        let yaw = Yaw::degrees(self.yaw);
        let rotation = yaw.rotation();

        // One spread for the whole wall, from the full brick: the end bricks
        // are half-length and wear the same textures, so they have to address
        // them the same way.
        let spread = texture_spread(brick);

        let mut entities = Vec::new();
        for row in 0..self.rows {
            let y = brick.y * (2 * row + 1) as f32;

            for (index, (x, half_length)) in self.course(row).into_iter().enumerate() {
                let centre = yaw.place(self.base, Vector3::new(x, y, 0.0));
                let half_extents = Vector3::new(half_length, brick.y, brick.z);
                let material =
                    materials[hash_pair(row as i32, index as i32) as usize % materials.len()];

                entities.push(
                    IceBlock::new(centre, half_extents, material, spread)
                        .rotated(rotation)
                        .spawn(world),
                );
            }
        }

        entities
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wall(columns: u32, stagger: bool) -> IceWallDef {
        IceWallDef {
            base: (0.0, 0.0, 0.0),
            brick_half_extents: (0.5, 0.2, 0.2),
            columns,
            rows: 2,
            yaw: 0.0,
            stagger,
        }
    }

    /// Every course, offset or not, must span exactly the same length — a
    /// running bond that overhangs is a wall whose footprint is a lie.
    #[test]
    fn every_course_spans_the_same_wall() {
        let wall = wall(4, true);
        for row in 0..4 {
            let course = wall.course(row);
            let left = course
                .iter()
                .fold(f32::MAX, |acc, (x, half)| acc.min(x - half));
            let right = course
                .iter()
                .fold(f32::MIN, |acc, (x, half)| acc.max(x + half));
            assert!(
                (left + wall.half_length()).abs() < 1e-5,
                "row {row}: {left}"
            );
            assert!(
                (right - wall.half_length()).abs() < 1e-5,
                "row {row}: {right}"
            );
        }
    }

    /// And the bricks within a course must meet without gaps or overlaps.
    #[test]
    fn bricks_in_a_course_are_laid_end_to_end() {
        for stagger in [false, true] {
            let wall = wall(5, stagger);
            for row in 0..2 {
                let course = wall.course(row);
                for pair in course.windows(2) {
                    let (left, left_half) = pair[0];
                    let (right, right_half) = pair[1];
                    assert!(
                        ((right - right_half) - (left + left_half)).abs() < 1e-5,
                        "bricks at {left} and {right} do not meet"
                    );
                }
            }
        }
    }

    /// The joints of one course must fall in the middle of the bricks of the
    /// next, which is the whole point of a running bond.
    #[test]
    fn a_staggered_course_breaks_the_joints_below_it() {
        let wall = wall(4, true);
        let joints: Vec<f32> = wall.course(0).iter().map(|(x, half)| x + half).collect();
        let centres: Vec<f32> = wall.course(1).iter().map(|(x, _)| *x).collect();
        for joint in joints.iter().take(wall.columns as usize - 1) {
            assert!(
                centres.iter().any(|centre| (centre - joint).abs() < 1e-5),
                "joint at {joint} is not covered"
            );
        }
    }
}
