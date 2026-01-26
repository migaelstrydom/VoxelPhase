//! Biped skeleton - joint positions and IK.
//!
//! Pure geometry and math, no animation state.

use nalgebra::{Point3, Vector3, Vector4};

use super::config::BipedConfig;
use super::state::BipedState;
use crate::{
    geometry::{generate_cylinder, generate_sphere_indices, generate_sphere_vertices},
    rendering::{colour::Colour, vertex::Vertex},
    skeleton::fabrik::{FABRIKSolver, IKChain, IKTarget},
};

/// Joint positions for a biped skeleton.
///
/// This is the output of animation - just joint positions ready for rendering
/// and collision detection. No animation state here.
pub struct BipedSkeleton {
    // Joint positions
    pub pelvis: Point3<f32>,
    pub left_hip: Point3<f32>,
    pub right_hip: Point3<f32>,
    pub left_knee: Point3<f32>,
    pub right_knee: Point3<f32>,
    pub left_foot: Point3<f32>,
    pub right_foot: Point3<f32>,

    // IK solver
    ik_solver: FABRIKSolver,
}

impl BipedSkeleton {
    /// Create a skeleton at the given pelvis position.
    pub fn new(config: &BipedConfig, pelvis_position: Point3<f32>, facing: Vector3<f32>) -> Self {
        let right = right_vector(facing);
        let leg_length = config.leg_length();

        let left_hip = pelvis_position + Vector3::new(-right.x, 0.0, -right.z) * config.hip_width;
        let right_hip = pelvis_position + right * config.hip_width;

        let left_foot = Point3::new(left_hip.x, pelvis_position.y - leg_length, left_hip.z);
        let right_foot = Point3::new(right_hip.x, pelvis_position.y - leg_length, right_hip.z);

        let left_knee = Point3::new(
            left_hip.x,
            pelvis_position.y - config.upper_leg_length,
            left_hip.z + 0.05,
        );
        let right_knee = Point3::new(
            right_hip.x,
            pelvis_position.y - config.upper_leg_length,
            right_hip.z + 0.05,
        );

        Self {
            pelvis: pelvis_position,
            left_hip,
            right_hip,
            left_knee,
            right_knee,
            left_foot,
            right_foot,
            ik_solver: FABRIKSolver::new(),
        }
    }

    /// Update skeleton from animation state.
    ///
    /// Takes current state and config, computes all joint positions.
    pub fn update_from_state(&mut self, state: &BipedState, config: &BipedConfig) {
        // Update pelvis
        self.pelvis = state.pelvis_position;

        // Compute hip positions from pelvis and facing
        let right = right_vector(state.facing);
        let left = Vector3::new(-right.x, 0.0, -right.z);
        self.left_hip = self.pelvis + left * config.hip_width;
        self.right_hip = self.pelvis + right * config.hip_width;

        // Copy foot positions from state
        self.left_foot = state.left.position;
        self.right_foot = state.right.position;

        // Solve IK for knees
        self.solve_knee_ik(state.facing, config);
    }

    /// Solve IK to position knees based on hip and foot positions.
    fn solve_knee_ik(&mut self, facing: Vector3<f32>, config: &BipedConfig) {
        let right = right_vector(facing);

        // Knee bend directions (slightly outward from facing)
        let left_bend = (facing - right * 0.2).normalize();
        let right_bend = (facing + right * 0.2).normalize();

        // Left leg IK
        self.left_knee = solve_leg_ik(
            &mut self.ik_solver,
            self.left_hip,
            self.left_foot,
            self.left_knee,
            config.upper_leg_length,
            config.lower_leg_length,
            left_bend,
        );

        // Right leg IK
        self.right_knee = solve_leg_ik(
            &mut self.ik_solver,
            self.right_hip,
            self.right_foot,
            self.right_knee,
            config.upper_leg_length,
            config.lower_leg_length,
            right_bend,
        );
    }
}

/// Solve IK for a single leg and return the knee position.
fn solve_leg_ik(
    solver: &mut FABRIKSolver,
    hip: Point3<f32>,
    foot: Point3<f32>,
    current_knee: Point3<f32>,
    upper_length: f32,
    lower_length: f32,
    bend_dir: Vector3<f32>,
) -> Point3<f32> {
    // Create temporary position array for FABRIK
    let mut positions = vec![hip, current_knee, foot];
    let chain = IKChain::new(vec![0, 1, 2], &positions);
    let target = IKTarget::new(foot);

    solver.solve(&mut positions, &chain, &target, true);

    // Apply knee bend bias and constrain
    let mut knee = positions[1] + bend_dir * 0.05;
    constrain_knee(&mut knee, &hip, &foot, upper_length, lower_length, bend_dir);

    knee
}

/// Constrain knee position to maintain bone lengths.
fn constrain_knee(
    knee: &mut Point3<f32>,
    hip: &Point3<f32>,
    foot: &Point3<f32>,
    upper: f32,
    lower: f32,
    bend_dir: Vector3<f32>,
) {
    let hip_to_foot = foot - hip;
    let dist = hip_to_foot.magnitude();

    if dist < 0.001 {
        // Foot at hip - put knee forward
        *knee = *hip + bend_dir * upper;
        return;
    }

    // Clamp distance to valid range
    let dist = dist.clamp(0.01, upper + lower - 0.01);

    // Law of cosines to find knee angle
    let cos_angle =
        ((upper * upper + dist * dist - lower * lower) / (2.0 * upper * dist)).clamp(-1.0, 1.0);
    let angle = cos_angle.acos();

    // Direction from hip to foot
    let forward = hip_to_foot.normalize();

    // Compute orthogonal bend direction
    let bend_axis = forward.cross(&bend_dir);
    let bend_dir_orth = if bend_axis.magnitude() > 0.01 {
        bend_axis.cross(&forward).normalize()
    } else {
        Vector3::y()
    };

    // Position knee using law of cosines result
    let knee_offset = forward * (angle.cos() * upper) + bend_dir_orth * (angle.sin() * upper);
    *knee = hip + knee_offset;
}

#[inline]
fn right_vector(facing: Vector3<f32>) -> Vector3<f32> {
    facing.cross(&Vector3::y()).normalize()
}

/// Generate a simple mesh for the biped skeleton.
pub fn generate_biped_mesh(
    skeleton: &BipedSkeleton,
    config: &BipedConfig,
) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    let segments = config.mesh_segments;
    let rod_radius = config.knee_radius * 0.5;

    // Add cylinders (rods connecting joints)
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.pelvis,
        skeleton.left_knee,
        rod_radius,
        segments,
        config.leg_colour,
    );
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.pelvis,
        skeleton.right_knee,
        rod_radius,
        segments,
        config.leg_colour,
    );
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_knee,
        skeleton.left_foot,
        rod_radius,
        segments,
        config.leg_colour,
    );
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_knee,
        skeleton.right_foot,
        rod_radius,
        segments,
        config.leg_colour,
    );

    // Add spheres (joints)
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.pelvis,
        config.pelvis_radius,
        segments,
        config.body_colour,
    );
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_knee,
        config.knee_radius,
        segments,
        config.leg_colour,
    );
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_knee,
        config.knee_radius,
        segments,
        config.leg_colour,
    );
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_foot,
        config.foot_radius,
        segments,
        config.foot_colour,
    );
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_foot,
        config.foot_radius,
        segments,
        config.foot_colour,
    );

    (vertices, indices)
}

fn add_sphere_to_mesh(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    center: Point3<f32>,
    radius: f32,
    segments: u32,
    colour: crate::rendering::Colour,
) {
    let base_index = vertices.len() as u32;
    let sphere_verts = generate_sphere_vertices(radius, segments, segments, colour);
    let sphere_indices = generate_sphere_indices(segments, segments);

    for mut vert in sphere_verts {
        vert.pos = Vector4::new(
            vert.pos.x + center.x,
            vert.pos.y + center.y,
            vert.pos.z + center.z,
            1.0,
        );
        vertices.push(vert);
    }
    indices.extend(sphere_indices.iter().map(|i| i + base_index));
}

fn add_cylinder_to_mesh(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    start: Point3<f32>,
    end: Point3<f32>,
    radius: f32,
    segments: u32,
    colour: Colour,
) {
    let base_index = vertices.len() as u32;
    let (cylinder_verts, cylinder_indices) =
        generate_cylinder(start, end, radius, segments, colour);

    vertices.extend(cylinder_verts);
    indices.extend(cylinder_indices.iter().map(|i| i + base_index));
}
