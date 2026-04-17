//! Humanoid skeleton - joint positions and IK.
//!
//! Pure geometry and math, no animation state.

use nalgebra::{Point3, Vector3, Vector4};

use crate::animation::config::CharacterRigConfig;
use crate::animation::state::AnimationState;
use crate::{
    geometry::{generate_cylinder, generate_sphere_indices, generate_sphere_vertices},
    rendering::{colour::Colour, vertex::Vertex},
    skeleton::fabrik::{FABRIKSolver, IKChain, IKTarget},
};

/// Joint positions for a humanoid skeleton.
///
/// This is the output of animation - just joint positions ready for rendering
/// and collision detection. No animation state here.
pub struct Skeleton {
    // Lower body joints
    pub pelvis: Point3<f32>,
    pub left_hip: Point3<f32>,
    pub right_hip: Point3<f32>,
    pub left_knee: Point3<f32>,
    pub right_knee: Point3<f32>,
    pub left_foot: Point3<f32>,
    pub right_foot: Point3<f32>,

    // Upper body joints
    pub chest: Point3<f32>,
    pub left_shoulder: Point3<f32>,
    pub right_shoulder: Point3<f32>,
    pub left_elbow: Point3<f32>,
    pub right_elbow: Point3<f32>,
    pub left_hand: Point3<f32>,
    pub right_hand: Point3<f32>,
    pub neck: Point3<f32>,
    pub head: Point3<f32>,

    // IK solver
    ik_solver: FABRIKSolver,
}

impl Skeleton {
    /// Create a skeleton at the given pelvis position.
    pub fn new(
        config: &CharacterRigConfig,
        pelvis_position: Point3<f32>,
        facing: Vector3<f32>,
    ) -> Self {
        let right = right_vector(facing);
        let left = Vector3::new(-right.x, 0.0, -right.z);
        let leg_length = config.leg_length();

        // Lower body
        let left_hip = pelvis_position + left * config.hip_width;
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

        // Upper body
        let chest = pelvis_position + Vector3::y() * config.torso_height;
        let left_shoulder = chest + left * config.shoulder_width;
        let right_shoulder = chest + right * config.shoulder_width;

        // Arms hang down by default
        let left_elbow = left_shoulder - Vector3::y() * config.upper_arm_length;
        let right_elbow = right_shoulder - Vector3::y() * config.upper_arm_length;
        let left_hand = left_elbow - Vector3::y() * config.lower_arm_length;
        let right_hand = right_elbow - Vector3::y() * config.lower_arm_length;

        // Head
        let neck = chest + Vector3::y() * config.neck_length;
        let head = neck + Vector3::y() * config.head_radius;

        Self {
            pelvis: pelvis_position,
            left_hip,
            right_hip,
            left_knee,
            right_knee,
            left_foot,
            right_foot,
            chest,
            left_shoulder,
            right_shoulder,
            left_elbow,
            right_elbow,
            left_hand,
            right_hand,
            neck,
            head,
            ik_solver: FABRIKSolver::new(),
        }
    }

    /// Update skeleton from animation state.
    ///
    /// Takes current state and config, computes all joint positions.
    pub fn update_from_state(&mut self, state: &AnimationState, config: &CharacterRigConfig) {
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

        // Update upper body
        self.update_upper_body(state, config);
    }

    /// Update upper body joint positions.
    fn update_upper_body(&mut self, state: &AnimationState, config: &CharacterRigConfig) {
        let facing = state.facing;
        let right = right_vector(facing);
        let left = Vector3::new(-right.x, 0.0, -right.z);

        // Chest is directly above pelvis
        self.chest = self.pelvis + Vector3::y() * config.torso_height;

        // Apply shoulder twist rotation
        let twist = state.shoulder_twist;
        let cos_twist = twist.cos();
        let sin_twist = twist.sin();

        // Rotate shoulder positions around vertical axis through chest
        // Left shoulder goes forward when twist is positive
        let left_offset = left * cos_twist + facing * sin_twist;
        let right_offset = right * cos_twist - facing * sin_twist;

        self.left_shoulder = self.chest + left_offset * config.shoulder_width;
        self.right_shoulder = self.chest + right_offset * config.shoulder_width;

        // Copy hand positions from state
        self.left_hand = state.left_hand.position;
        self.right_hand = state.right_hand.position;

        // Solve IK for elbows
        self.solve_elbow_ik(facing, config);

        // Neck and head
        self.neck = self.chest + Vector3::y() * config.neck_length;

        // Apply head tilt and bob
        let tilt_forward = state.head_tilt.x;
        let tilt_lateral = state.head_tilt.y;
        let bob = state.head_bob;

        // Head offset: forward tilt moves head forward, lateral tilt moves it sideways
        let head_offset = Vector3::y() * (config.head_radius + bob)
            + facing * tilt_forward
            + right * tilt_lateral;

        self.head = self.neck + head_offset;
    }

    /// Solve IK to position knees based on hip and foot positions.
    fn solve_knee_ik(&mut self, facing: Vector3<f32>, config: &CharacterRigConfig) {
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

    /// Solve IK to position elbows based on shoulder and hand positions.
    fn solve_elbow_ik(&mut self, facing: Vector3<f32>, config: &CharacterRigConfig) {
        // Elbow bend directions (backward, slightly outward)
        let right = right_vector(facing);
        let backward = -facing;
        let left_bend = (backward - right * 0.3).normalize();
        let right_bend = (backward + right * 0.3).normalize();

        // Left arm IK
        self.left_elbow = solve_arm_ik(
            self.left_shoulder,
            self.left_hand,
            self.left_elbow,
            config.upper_arm_length,
            config.lower_arm_length,
            left_bend,
        );

        // Right arm IK
        self.right_elbow = solve_arm_ik(
            self.right_shoulder,
            self.right_hand,
            self.right_elbow,
            config.upper_arm_length,
            config.lower_arm_length,
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

/// Solve IK for an arm and return the elbow position.
fn solve_arm_ik(
    shoulder: Point3<f32>,
    hand: Point3<f32>,
    _current_elbow: Point3<f32>,
    upper_length: f32,
    lower_length: f32,
    bend_dir: Vector3<f32>,
) -> Point3<f32> {
    let shoulder_to_hand = hand - shoulder;
    let dist = shoulder_to_hand.magnitude();

    if dist < 0.001 {
        // Hand at shoulder - put elbow in bend direction
        return shoulder + bend_dir * upper_length;
    }

    // Clamp distance to valid range
    let dist = dist.clamp(0.01, upper_length + lower_length - 0.01);

    // Law of cosines to find elbow angle
    let cos_angle = ((upper_length * upper_length + dist * dist - lower_length * lower_length)
        / (2.0 * upper_length * dist))
        .clamp(-1.0, 1.0);
    let angle = cos_angle.acos();

    // Direction from shoulder to hand
    let forward = shoulder_to_hand.normalize();

    // Compute orthogonal bend direction
    let bend_axis = forward.cross(&bend_dir);
    let bend_dir_orth = if bend_axis.magnitude() > 0.01 {
        bend_axis.cross(&forward).normalize()
    } else {
        -Vector3::y() // Default to downward if vectors are parallel
    };

    // Position elbow using law of cosines result
    let elbow_offset =
        forward * (angle.cos() * upper_length) + bend_dir_orth * (angle.sin() * upper_length);
    shoulder + elbow_offset
}

#[inline]
fn right_vector(facing: Vector3<f32>) -> Vector3<f32> {
    facing.cross(&Vector3::y()).normalize()
}

/// Generate a simple mesh for the humanoid skeleton.
pub fn generate_character_mesh(
    skeleton: &Skeleton,
    config: &CharacterRigConfig,
) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    let segments = config.mesh_segments;
    let leg_rod_radius = config.knee_radius * 0.5;
    let arm_rod_radius = config.elbow_radius * 0.5;

    // === Lower body cylinders ===
    // Upper legs (hip to knee)
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_hip,
        skeleton.left_knee,
        leg_rod_radius,
        segments,
        config.leg_colour,
    );
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_hip,
        skeleton.right_knee,
        leg_rod_radius,
        segments,
        config.leg_colour,
    );
    // Lower legs (knee to foot)
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_knee,
        skeleton.left_foot,
        leg_rod_radius,
        segments,
        config.leg_colour,
    );
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_knee,
        skeleton.right_foot,
        leg_rod_radius,
        segments,
        config.leg_colour,
    );

    // === Upper body cylinders ===
    // Torso (pelvis to chest)
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.pelvis,
        skeleton.chest,
        config.torso_radius,
        segments,
        config.body_colour,
    );
    // Neck (chest to neck)
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.chest,
        skeleton.neck,
        config.torso_radius * 0.5,
        segments,
        config.body_colour,
    );
    // Upper arms (shoulder to elbow)
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_shoulder,
        skeleton.left_elbow,
        arm_rod_radius,
        segments,
        config.leg_colour,
    );
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_shoulder,
        skeleton.right_elbow,
        arm_rod_radius,
        segments,
        config.leg_colour,
    );
    // Lower arms (elbow to hand)
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_elbow,
        skeleton.left_hand,
        arm_rod_radius,
        segments,
        config.leg_colour,
    );
    add_cylinder_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_elbow,
        skeleton.right_hand,
        arm_rod_radius,
        segments,
        config.leg_colour,
    );

    // === Lower body spheres ===
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
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_hip,
        config.hip_radius,
        segments,
        config.hip_colour,
    );
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_hip,
        config.hip_radius,
        segments,
        config.hip_colour,
    );

    // === Upper body spheres ===
    // Chest
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.chest,
        config.torso_radius,
        segments,
        config.body_colour,
    );
    // Shoulders
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_shoulder,
        config.shoulder_radius,
        segments,
        config.leg_colour,
    );
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_shoulder,
        config.shoulder_radius,
        segments,
        config.leg_colour,
    );
    // Elbows
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_elbow,
        config.elbow_radius,
        segments,
        config.leg_colour,
    );
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_elbow,
        config.elbow_radius,
        segments,
        config.leg_colour,
    );
    // Hands
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.left_hand,
        config.hand_radius,
        segments,
        config.foot_colour,
    );
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.right_hand,
        config.hand_radius,
        segments,
        config.foot_colour,
    );
    // Head
    add_sphere_to_mesh(
        &mut vertices,
        &mut indices,
        skeleton.head,
        config.head_radius,
        segments,
        config.head_colour,
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
