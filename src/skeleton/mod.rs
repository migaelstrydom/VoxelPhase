//! Procedural skeletal animation system.
//!
//! This module provides a complete procedural animation system for humanoid characters:
//!
//! - **Verlet physics** (`verlet.rs`): Position-based particle simulation for natural movement
//! - **FABRIK IK** (`fabrik.rs`): Fast inverse kinematics for limb positioning
//! - **Humanoid skeleton** (`humanoid.rs`): Pre-built skeleton topology for platformer characters
//! - **Procedural mesh** (`mesh.rs`): Generate visual geometry from skeleton state
//! - **Locomotion** (`locomotion.rs`): Procedural walking and terrain adaptation
//! - **Components** (`components.rs`): ECS integration for specs
//!
//! ## Usage
//!
//! ```ignore
//! // Create a skeleton
//! let skeleton = HumanoidSkeleton::new(HumanoidConfig::default());
//!
//! // Create locomotion controller
//! let locomotion = LocomotionController::new(LocomotionConfig::default());
//!
//! // Each frame:
//! locomotion.update(&mut skeleton, root_position, ground_height, dt);
//! skeleton.update(dt);
//!
//! // Generate mesh for rendering
//! let mesh_gen = HumanoidMeshGenerator::default();
//! let (vertices, indices) = mesh_gen.generate(&skeleton);
//! ```

mod components;
mod fabrik;
mod humanoid;
mod locomotion;
mod mesh;
mod verlet;

// Re-export main types used by the ECS
pub use components::{ProceduralCharacter, ProceduralCharacterConfig};
