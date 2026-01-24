//! ECS components for the procedural animation system.
//!
//! These components integrate the skeleton system with the specs ECS.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

use super::biped::{SpringBipedConfig, SpringBipedSkeleton};
use super::mesh::SpringBipedMeshConfig;
use super::mesh::SpringBipedMeshGenerator;
use crate::rendering::vertex::Vertex;

/// Configuration for a spring biped character.
#[derive(Clone, Debug)]
pub struct SpringBipedCharacterConfig {
    pub skeleton: SpringBipedConfig,
    pub mesh: SpringBipedMeshConfig,
}

impl Default for SpringBipedCharacterConfig {
    fn default() -> Self {
        Self {
            skeleton: SpringBipedConfig::default(),
            mesh: SpringBipedMeshConfig::default(),
        }
    }
}

/// A spring-leg biped character with physics-based locomotion.
///
/// Uses spring forces for natural bouncy walking:
/// - Legs act as springs pushing the pelvis
/// - FABRIK IK positions knees cosmetically
/// - Sphere collision on joints
/// - Gait-phase driven foot targeting
#[derive(Component)]
#[storage(VecStorage)]
pub struct SpringBipedCharacter {
    /// The spring biped skeleton.
    pub skeleton: SpringBipedSkeleton,
    /// Whether the character is currently grounded.
    pub grounded: bool,
    /// Normal of the ground surface (if grounded).
    pub ground_normal: Option<Vector3<f32>>,
    /// Mesh generator for rendering.
    mesh_generator: SpringBipedMeshGenerator,
    /// Cached mesh vertices.
    cached_vertices: Vec<Vertex>,
    /// Cached mesh indices.
    cached_indices: Vec<u32>,
    /// Whether the mesh needs regeneration.
    mesh_dirty: bool,
}

impl SpringBipedCharacter {
    /// Create a new spring biped character at the given position.
    pub fn new(config: SpringBipedCharacterConfig, initial_position: Point3<f32>) -> Self {
        let skeleton = SpringBipedSkeleton::new(config.skeleton, initial_position);

        Self {
            skeleton,
            grounded: true,
            ground_normal: Some(Vector3::new(0.0, 1.0, 0.0)),
            mesh_generator: SpringBipedMeshGenerator::new(config.mesh),
            cached_vertices: Vec::new(),
            cached_indices: Vec::new(),
            mesh_dirty: true,
        }
    }

    /// Resolve collisions with terrain triangles.
    ///
    /// Call this after update() with triangles from terrain query.
    pub fn resolve_collisions(&mut self, triangles: &[crate::collision::Triangle]) -> bool {
        self.skeleton.resolve_collisions(triangles)
    }

    /// Get the AABB for terrain collision queries.
    pub fn get_collision_aabb(&self) -> crate::collision::AABB {
        self.skeleton.get_collision_aabb()
    }

    /// Set the pelvis position directly.
    pub fn set_pelvis_position(&mut self, position: Point3<f32>) {
        self.skeleton.set_pelvis_position(position);
    }

    /// Get the mesh vertices and indices for rendering.
    pub fn mesh(&mut self) -> (&[Vertex], &[u32]) {
        if self.mesh_dirty {
            let (vertices, indices) = self.mesh_generator.generate(&self.skeleton);
            self.cached_vertices = vertices;
            self.cached_indices = indices;
            self.mesh_dirty = false;
        }

        (&self.cached_vertices, &self.cached_indices)
    }

    /// Mark mesh as dirty after manual skeleton edits.
    pub fn mark_dirty(&mut self) {
        self.mesh_dirty = true;
    }
}
