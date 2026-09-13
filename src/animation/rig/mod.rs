//! Rig-building parts shared by every procedural character.

mod droop;
mod leg;
mod mesh;

pub use droop::{DroopConfig, DroopPair, DroopSide};
pub use leg::{solve_knee, within_reach, Leg};
pub use mesh::{project_to_horizontal, right_vector, FootShape, Frame, RigMesh};
