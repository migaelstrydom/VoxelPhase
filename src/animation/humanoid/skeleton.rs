//! Humanoid skeleton - joint positions and IK.
//!
//! Pure geometry and math, no animation state.

use nalgebra::{Point3, Vector2, Vector3};

use crate::animation::config::CharacterRigConfig;
use crate::animation::pose::PoseFragment;
use crate::animation::rig::{right_vector, solve_knee, within_reach, FootShape, RigMesh};
use crate::animation::state::AnimationState;
use crate::rendering::vertex::Vertex;
use crate::skeleton::fabrik::FABRIKSolver;

/// Visible foot dimensions. Independent from `foot_radius` (which keeps
/// its physics / gait meaning).
pub const FOOT_SHAPE: FootShape = FootShape {
    radius: 0.035,
    half_length: 0.075,
    ankle_forward_offset: 0.02,
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
    /// Horizontal forward direction each foot's toe points. Updated per
    /// frame by `apply_fragment` from the foot state. Both feet share the
    /// body facing today.
    pub left_foot_forward: Vector3<f32>,
    pub right_foot_forward: Vector3<f32>,
    /// Foot up-axis per side (perpendicular to the sole). Drives the
    /// ankle tilt of the rendered capsule so feet hug slopes/stairs.
    pub left_foot_up: Vector3<f32>,
    pub right_foot_up: Vector3<f32>,

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
            left_foot_forward: facing,
            right_foot_forward: facing,
            left_foot_up: Vector3::y(),
            right_foot_up: Vector3::y(),
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

    /// Update skeleton from animation state. Convenience wrapper for
    /// `apply_fragment` with an empty fragment — the skeleton reads every
    /// channel from `state`.
    pub fn update_from_state(&mut self, state: &AnimationState, config: &CharacterRigConfig) {
        self.apply_fragment(&PoseFragment::default(), state, config);
    }

    /// Update skeleton from a pose fragment, falling back to `state` for any
    /// channel the fragment leaves `None`. This is the single render-path
    /// entry point: all pose data flows through a `PoseFragment`.
    pub fn apply_fragment(
        &mut self,
        fragment: &PoseFragment,
        state: &AnimationState,
        config: &CharacterRigConfig,
    ) {
        let pelvis_base = state.pelvis_position;
        let pelvis_offset = fragment.pelvis_offset.unwrap_or_else(Vector3::zeros);
        self.pelvis = pelvis_base + pelvis_offset;

        let right = right_vector(state.facing);
        let left = Vector3::new(-right.x, 0.0, -right.z);
        self.left_hip = self.pelvis + left * config.hip_width;
        self.right_hip = self.pelvis + right * config.hip_width;

        let (left_foot, right_foot) = match &fragment.feet {
            Some(feet) => (feet.left, feet.right),
            None => (state.left.position, state.right.position),
        };
        // Whatever animation asks for, a leg is as long as it is. Nothing
        // upstream refuses an unreachable foot — the placer's overstretch
        // release is a request to step, not a veto — so without this the
        // skeleton draws the leg to wherever the foot was asked for and the
        // limb visibly separates. It happens for a handful of frames at every
        // edge case that outruns the gait: walking off a ledge with a foot
        // mid-swing, a landing slide, ground dropping away underfoot.
        // Clamping turns those into a leg trailing at full stretch, which is
        // what a leg does.
        self.left_foot = within_reach(self.left_hip, left_foot, config.leg_length());
        self.right_foot = within_reach(self.right_hip, right_foot, config.leg_length());
        self.left_foot_forward = state.left.forward;
        self.right_foot_forward = state.right.forward;
        self.left_foot_up = state.left.up;
        self.right_foot_up = state.right.up;

        self.solve_knee_ik(state.facing, config);

        let shoulder_twist = fragment.shoulder_twist.unwrap_or(state.shoulder_twist);
        let head_tilt = fragment.head_tilt.unwrap_or(state.head_tilt);
        let head_bob = fragment.head_bob.unwrap_or(state.head_bob);
        let torso_pitch = fragment.torso_pitch.unwrap_or(0.0);
        let (left_hand, right_hand) = match &fragment.hands {
            Some(hands) => (hands.left, hands.right),
            None => (state.left_hand.position, state.right_hand.position),
        };

        self.update_upper_body(
            state.facing,
            config,
            shoulder_twist,
            head_tilt,
            head_bob,
            torso_pitch,
            left_hand,
            right_hand,
        );
    }

    /// Update upper body joint positions from resolved pose channels.
    fn update_upper_body(
        &mut self,
        facing: Vector3<f32>,
        config: &CharacterRigConfig,
        shoulder_twist: f32,
        head_tilt: Vector2<f32>,
        head_bob: f32,
        torso_pitch: f32,
        left_hand: Point3<f32>,
        right_hand: Point3<f32>,
    ) {
        let right = right_vector(facing);
        let left = Vector3::new(-right.x, 0.0, -right.z);

        // Torso-local up axis, pitched forward by `torso_pitch` around the
        // lateral axis. Chest, neck, and head ride on this axis.
        let torso_up = Vector3::y() * torso_pitch.cos() + facing * torso_pitch.sin();

        self.chest = self.pelvis + torso_up * config.torso_height;

        let cos_twist = shoulder_twist.cos();
        let sin_twist = shoulder_twist.sin();

        // Left shoulder goes forward when twist is positive.
        let left_offset = left * cos_twist + facing * sin_twist;
        let right_offset = right * cos_twist - facing * sin_twist;

        self.left_shoulder = self.chest + left_offset * config.shoulder_width;
        self.right_shoulder = self.chest + right_offset * config.shoulder_width;

        self.left_hand = left_hand;
        self.right_hand = right_hand;

        self.solve_elbow_ik(facing, config);

        self.neck = self.chest + torso_up * config.neck_length;

        let tilt_forward = head_tilt.x;
        let tilt_lateral = head_tilt.y;

        let head_offset = torso_up * (config.head_radius + head_bob)
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
        self.left_knee = solve_knee(
            &mut self.ik_solver,
            self.left_hip,
            self.left_foot,
            self.left_knee,
            config.upper_leg_length,
            config.lower_leg_length,
            left_bend,
        );

        // Right leg IK
        self.right_knee = solve_knee(
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

/// Generate a simple mesh for the humanoid skeleton.
pub fn generate_character_mesh(
    skeleton: &Skeleton,
    config: &CharacterRigConfig,
) -> (Vec<Vertex>, Vec<u32>) {
    let mut mesh = RigMesh::new(config.mesh_segments);

    let leg_rod_radius = config.knee_radius * 0.5;
    let arm_rod_radius = config.elbow_radius * 0.5;

    // === Bones ===
    // Both sides of each segment before moving down the limb, so the
    // mesh is laid out the way the rig reads.
    for (upper, lower) in [
        (skeleton.left_hip, skeleton.left_knee),
        (skeleton.right_hip, skeleton.right_knee),
        (skeleton.left_knee, skeleton.left_foot),
        (skeleton.right_knee, skeleton.right_foot),
    ] {
        mesh.rod(upper, lower, leg_rod_radius, config.leg_colour);
    }
    mesh.rod(
        skeleton.pelvis,
        skeleton.chest,
        config.torso_radius,
        config.body_colour,
    );
    mesh.rod(
        skeleton.chest,
        skeleton.neck,
        config.torso_radius * 0.5,
        config.body_colour,
    );
    for (upper, lower) in [
        (skeleton.left_shoulder, skeleton.left_elbow),
        (skeleton.right_shoulder, skeleton.right_elbow),
        (skeleton.left_elbow, skeleton.left_hand),
        (skeleton.right_elbow, skeleton.right_hand),
    ] {
        mesh.rod(upper, lower, arm_rod_radius, config.leg_colour);
    }

    // === Lower body joints ===
    mesh.sphere(skeleton.pelvis, config.pelvis_radius, config.body_colour);
    mesh.sphere(skeleton.left_knee, config.knee_radius, config.leg_colour);
    mesh.sphere(skeleton.right_knee, config.knee_radius, config.leg_colour);
    mesh.foot(
        FOOT_SHAPE,
        skeleton.left_foot,
        skeleton.left_foot_forward,
        skeleton.left_foot_up,
        config.foot_colour,
    );
    mesh.foot(
        FOOT_SHAPE,
        skeleton.right_foot,
        skeleton.right_foot_forward,
        skeleton.right_foot_up,
        config.foot_colour,
    );
    mesh.sphere(skeleton.left_hip, config.hip_radius, config.hip_colour);
    mesh.sphere(skeleton.right_hip, config.hip_radius, config.hip_colour);

    // === Upper body joints ===
    mesh.sphere(skeleton.chest, config.torso_radius, config.body_colour);
    mesh.sphere(
        skeleton.left_shoulder,
        config.shoulder_radius,
        config.leg_colour,
    );
    mesh.sphere(
        skeleton.right_shoulder,
        config.shoulder_radius,
        config.leg_colour,
    );
    mesh.sphere(skeleton.left_elbow, config.elbow_radius, config.leg_colour);
    mesh.sphere(skeleton.right_elbow, config.elbow_radius, config.leg_colour);
    mesh.sphere(skeleton.left_hand, config.hand_radius, config.foot_colour);
    mesh.sphere(skeleton.right_hand, config.hand_radius, config.foot_colour);
    mesh.sphere(skeleton.head, config.head_radius, config.head_colour);

    mesh.into_parts()
}
