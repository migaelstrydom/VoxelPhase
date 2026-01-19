//! ECS components for the procedural animation system.
//!
//! These components integrate the skeleton system with the specs ECS.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

use super::biped::{BipedConfig, GaitConfig, GaitController, SimpleBipedSkeleton};
use super::humanoid::{HumanoidConfig, HumanoidSkeleton};
use super::locomotion::{LocomotionConfig, LocomotionController};
use super::mesh::{
    BipedMeshConfig, BipedMeshGenerator, HumanoidMeshConfig, HumanoidMeshGenerator, PogoMeshConfig,
    PogoMeshGenerator,
};
use super::pogo::{PogoConfig, PogoStickSkeleton};
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

// ============================================================================
// Stage 1: Pogo Stick Character
// ============================================================================

/// Configuration for a pogo stick character.
#[derive(Clone, Debug)]
pub struct PogoCharacterConfig {
    pub pogo: PogoConfig,
    pub mesh: PogoMeshConfig,
}

impl Default for PogoCharacterConfig {
    fn default() -> Self {
        Self {
            pogo: PogoConfig::default(),
            mesh: PogoMeshConfig::default(),
        }
    }
}

/// A simple pogo stick character for Stage 1 skeleton development.
///
/// This is the simplest possible skeleton:
/// - 2 particles (foot + head)
/// - 1 distance constraint (the pole)
/// - No IK, no locomotion controller
///
/// Use this to validate Verlet physics and constraint solving work correctly.
#[derive(Component)]
#[storage(VecStorage)]
pub struct PogoCharacter {
    /// The pogo stick skeleton.
    pub skeleton: PogoStickSkeleton,
    /// Mesh generator for rendering.
    mesh_generator: PogoMeshGenerator,
    /// Cached mesh vertices.
    cached_vertices: Vec<Vertex>,
    /// Cached mesh indices.
    cached_indices: Vec<u32>,
    /// Whether the mesh needs regeneration.
    mesh_dirty: bool,
    /// Previous root position (for velocity calculation).
    prev_root_position: Point3<f32>,
    /// Was grounded last frame?
    was_grounded: bool,
}

impl PogoCharacter {
    /// Create a new pogo character with the given config.
    pub fn new(config: PogoCharacterConfig) -> Self {
        Self {
            skeleton: PogoStickSkeleton::new(config.pogo),
            mesh_generator: PogoMeshGenerator::new(config.mesh),
            cached_vertices: Vec::new(),
            cached_indices: Vec::new(),
            mesh_dirty: true,
            prev_root_position: Point3::origin(),
            was_grounded: true,
        }
    }

    /// Update the pogo character animation.
    ///
    /// `root_position` - The world position of the character's feet.
    /// `velocity` - The character's current velocity.
    /// `is_grounded` - Whether the character is touching ground.
    /// `dt` - Delta time in seconds.
    pub fn update(
        &mut self,
        root_position: Point3<f32>,
        velocity: Vector3<f32>,
        is_grounded: bool,
        dt: f32,
    ) {
        // Detect landing (transition from airborne to grounded)
        let just_landed = is_grounded && !self.was_grounded;

        // Update root position
        self.skeleton.set_root_position(root_position);

        // Apply velocity-based impulse to the head for natural bobbing
        // When moving, the head should lag slightly behind the foot
        if is_grounded {
            // Calculate movement delta
            let movement = root_position - self.prev_root_position;

            // Apply a gentle impulse opposite to movement direction
            // This creates natural inertia - head lags when accelerating
            let inertia_factor = 0.3;
            let inertia_impulse = -movement * inertia_factor;
            self.skeleton.apply_head_impulse(inertia_impulse);

            // On landing, apply a downward bob
            if just_landed {
                let landing_impulse = Vector3::new(0.0, -0.1, 0.0);
                self.skeleton.apply_head_impulse(landing_impulse);
            }
        }

        // Pin/unpin foot based on ground state
        // When grounded, foot is fixed at root position
        // When airborne, foot can swing freely
        if is_grounded {
            self.skeleton.pin_foot();
        } else {
            // When airborne, unpin foot so the whole pogo moves as a unit
            self.skeleton.unpin_foot();
        }

        // Run physics simulation
        self.skeleton.update(dt);

        // Update state for next frame
        self.prev_root_position = root_position;
        self.was_grounded = is_grounded;
        self.mesh_dirty = true;
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
}

impl Default for PogoCharacter {
    fn default() -> Self {
        Self::new(PogoCharacterConfig::default())
    }
}

// ============================================================================
// Stage 2: Biped Character (Pelvis + Two Legs)
// ============================================================================

/// Configuration for a biped character.
#[derive(Clone, Debug)]
pub struct BipedCharacterConfig {
    pub biped: BipedConfig,
    pub gait: GaitConfig,
    pub mesh: BipedMeshConfig,
}

impl Default for BipedCharacterConfig {
    fn default() -> Self {
        Self {
            biped: BipedConfig::default(),
            gait: GaitConfig::default(),
            mesh: BipedMeshConfig::default(),
        }
    }
}

/// A biped character with pelvis and two feet (Stage 2).
///
/// Uses phase-based gait controller for natural walking animation:
/// - Pelvis position set directly from physics body
/// - Feet positions driven by gait phase (not reactive)
/// - Feet hang below pelvis when airborne
/// - Later stages will add FABRIK IK for knee positioning
#[derive(Component)]
#[storage(VecStorage)]
pub struct BipedCharacter {
    /// The biped skeleton (just stores positions, no physics).
    pub skeleton: SimpleBipedSkeleton,
    /// Phase-based gait controller for foot placement.
    pub gait_controller: GaitController,
    /// Mesh generator for rendering.
    mesh_generator: BipedMeshGenerator,
    /// Cached mesh vertices.
    cached_vertices: Vec<Vertex>,
    /// Cached mesh indices.
    cached_indices: Vec<u32>,
    /// Whether the mesh needs regeneration.
    mesh_dirty: bool,
}

impl BipedCharacter {
    /// Create a new biped character at the given position.
    pub fn new(
        config: BipedCharacterConfig,
        initial_position: Point3<f32>,
        _ground_height: f32,
    ) -> Self {
        let skeleton = SimpleBipedSkeleton::new(config.biped, initial_position);
        let gait_controller = GaitController::new(config.gait);

        Self {
            skeleton,
            gait_controller,
            mesh_generator: BipedMeshGenerator::new(config.mesh),
            cached_vertices: Vec::new(),
            cached_indices: Vec::new(),
            mesh_dirty: true,
        }
    }

    /// Update the biped animation.
    ///
    /// `pelvis_position` - Where the pelvis should be (from physics body).
    /// `ground_height` - Height of the ground at the character's position.
    /// `velocity` - Current movement velocity.
    /// `facing` - Unit vector indicating which way the character is facing.
    /// `is_grounded` - Whether the character is on the ground.
    /// `dt` - Delta time in seconds.
    pub fn update(
        &mut self,
        pelvis_position: Point3<f32>,
        ground_height: f32,
        velocity: Vector3<f32>,
        facing: Vector3<f32>,
        is_grounded: bool,
        dt: f32,
    ) {
        self.skeleton.set_pelvis_position(pelvis_position);

        if is_grounded {
            // Grounded: gait controller drives foot placement from phase
            self.gait_controller
                .update(pelvis_position, ground_height, velocity, facing, dt);
            self.skeleton
                .set_left_foot_position(self.gait_controller.left_foot());
            self.skeleton
                .set_right_foot_position(self.gait_controller.right_foot());
        } else {
            // Airborne: feet hang below pelvis at leg length
            let leg_length =
                self.skeleton.config.upper_leg_length + self.skeleton.config.lower_leg_length;
            let (left_foot, right_foot) =
                self.gait_controller
                    .airborne_feet(pelvis_position, leg_length, facing);

            self.skeleton.set_left_foot_position(left_foot);
            self.skeleton.set_right_foot_position(right_foot);
        }

        self.mesh_dirty = true;
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
}
