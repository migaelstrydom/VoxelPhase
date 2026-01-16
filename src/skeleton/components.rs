//! ECS components for the procedural animation system.
//!
//! These components integrate the skeleton system with the specs ECS.

use nalgebra::Point3;
use specs::{Component, VecStorage};

use super::humanoid::{HumanoidConfig, HumanoidSkeleton};
use super::locomotion::{LocomotionConfig, LocomotionController};
use super::mesh::{HumanoidMeshConfig, HumanoidMeshGenerator};
use crate::rendering::vertex::Vertex;

/// Configuration for a procedural character.
#[derive(Clone, Debug)]
pub struct ProceduralCharacterConfig {
    pub humanoid: HumanoidConfig,
    pub locomotion: LocomotionConfig,
    pub mesh: HumanoidMeshConfig,
}

impl Default for ProceduralCharacterConfig {
    fn default() -> Self {
        Self {
            humanoid: HumanoidConfig::default(),
            locomotion: LocomotionConfig::default(),
            mesh: HumanoidMeshConfig::default(),
        }
    }
}

/// A procedural character with skeleton, locomotion, and mesh generation.
///
/// This component owns all the animation state for a humanoid character.
#[derive(Component)]
#[storage(VecStorage)]
pub struct ProceduralCharacter {
    /// The skeletal physics simulation.
    pub skeleton: HumanoidSkeleton,
    /// The locomotion controller.
    pub locomotion: LocomotionController,
    /// Mesh generator for rendering.
    mesh_generator: HumanoidMeshGenerator,
    /// Cached mesh vertices (regenerated each frame).
    cached_vertices: Vec<Vertex>,
    /// Cached mesh indices (regenerated each frame).
    cached_indices: Vec<u32>,
    /// Whether the mesh needs regeneration.
    mesh_dirty: bool,
    /// Current ground height (from terrain).
    ground_height: f32,
    /// Is the character grounded?
    is_grounded: bool,
}

impl ProceduralCharacter {
    /// Create a new procedural character with the given config.
    pub fn new(config: ProceduralCharacterConfig) -> Self {
        let skeleton = HumanoidSkeleton::new(config.humanoid);
        let locomotion = LocomotionController::new(config.locomotion, &skeleton);

        Self {
            skeleton,
            locomotion,
            mesh_generator: HumanoidMeshGenerator::new(config.mesh),
            cached_vertices: Vec::new(),
            cached_indices: Vec::new(),
            mesh_dirty: true,
            ground_height: 0.0,
            is_grounded: true,
        }
    }

    /// Update the character animation.
    ///
    /// `root_position` - The world position of the character's hips.
    /// `ground_height` - The height of the ground at the character's position.
    /// `is_grounded` - Whether the character is touching ground.
    /// `dt` - Delta time in seconds.
    pub fn update(
        &mut self,
        root_position: Point3<f32>,
        ground_height: f32,
        is_grounded: bool,
        dt: f32,
    ) {
        self.ground_height = ground_height;
        self.is_grounded = is_grounded;
        self.locomotion.is_grounded = is_grounded;

        // Update skeleton root position
        self.skeleton.set_root_position(root_position);

        // Update locomotion (procedural walking/airborne pose)
        self.locomotion
            .update(&mut self.skeleton, root_position, ground_height, dt);

        // Run physics simulation
        self.skeleton.update(dt);

        // Mark mesh as needing regeneration
        self.mesh_dirty = true;
    }

    /// Get the mesh vertices and indices for rendering.
    ///
    /// This regenerates the mesh from skeleton state if needed.
    pub fn mesh(&mut self) -> (&[Vertex], &[u32]) {
        if self.mesh_dirty {
            let (vertices, indices) = self.mesh_generator.generate(&self.skeleton);
            self.cached_vertices = vertices;
            self.cached_indices = indices;
            self.mesh_dirty = false;
        }

        (&self.cached_vertices, &self.cached_indices)
    }
}

impl Default for ProceduralCharacter {
    fn default() -> Self {
        Self::new(ProceduralCharacterConfig::default())
    }
}
