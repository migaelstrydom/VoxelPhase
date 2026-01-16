//! Procedural mesh generation from skeleton.
//!
//! Generates renderable geometry (capsules/spheres) from skeleton joint positions.
//! This gives the character a "bean" or "noodle" look typical of platformer characters.

use nalgebra::{Point3, Vector2, Vector3, Vector4};

use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

use super::humanoid::HumanoidSkeleton;

/// Configuration for the visual appearance of the humanoid mesh.
#[derive(Clone, Debug)]
pub struct HumanoidMeshConfig {
    /// Color of the body.
    pub body_colour: Colour,
    /// Color of the head.
    pub head_colour: Colour,
    /// Color of hands/feet.
    pub extremity_colour: Colour,

    /// Radius of limb capsules (relative to height).
    pub limb_radius: f32,
    /// Radius of the head sphere (relative to configured head size).
    pub head_radius_scale: f32,
    /// Segments around capsules (higher = smoother).
    pub radial_segments: u32,
    /// Segments along capsule length.
    pub length_segments: u32,
}

impl Default for HumanoidMeshConfig {
    fn default() -> Self {
        Self {
            body_colour: Colour::new(0.9, 0.6, 0.4, 1.0), // Peachy skin tone
            head_colour: Colour::new(0.95, 0.65, 0.45, 1.0), // Slightly lighter head
            extremity_colour: Colour::new(0.3, 0.3, 0.35, 1.0), // Dark hands/feet

            limb_radius: 0.03,
            head_radius_scale: 1.0,
            radial_segments: 8,
            length_segments: 4,
        }
    }
}

/// A bone segment to render as a capsule.
#[derive(Clone, Debug)]
struct BoneSegment {
    start: Point3<f32>,
    end: Point3<f32>,
    radius: f32,
    colour: Colour,
}

/// Generate mesh data for a humanoid skeleton.
pub struct HumanoidMeshGenerator {
    pub mesh_config: HumanoidMeshConfig,
}

impl HumanoidMeshGenerator {
    /// Create a new mesh generator with the given config.
    pub fn new(config: HumanoidMeshConfig) -> Self {
        Self {
            mesh_config: config,
        }
    }

    /// Generate mesh vertices and indices from a skeleton.
    ///
    /// Returns (vertices, indices) suitable for rendering.
    pub fn generate(&self, skeleton: &HumanoidSkeleton) -> (Vec<Vertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        let joints = &skeleton.joints;
        let config = &skeleton.config;

        // Calculate actual limb radius
        let limb_radius = self.mesh_config.limb_radius * config.height;
        let thick_radius = limb_radius * 1.5; // Thicker for torso

        // Generate bones as capsules
        let bones = self.collect_bones(skeleton, limb_radius, thick_radius);

        for bone in &bones {
            self.add_capsule(bone, &mut vertices, &mut indices);
        }

        // Generate head as a sphere
        let head_radius =
            config.head_size * config.height * 0.5 * self.mesh_config.head_radius_scale;
        let head_pos = skeleton.joint_position(joints.head);
        self.add_sphere(
            head_pos,
            head_radius,
            self.mesh_config.head_colour,
            &mut vertices,
            &mut indices,
        );

        // Generate eyes
        let eye_radius = head_radius * 0.2;
        let eye_offset_y = head_radius * 0.2;
        let eye_offset_x = head_radius * 0.35;
        let eye_offset_z = head_radius * 0.85;

        // Calculate facing direction for eye placement
        let facing = skeleton.facing_direction();
        let right = facing.cross(&Vector3::y()).normalize();
        let up = Vector3::y();

        let left_eye_pos =
            head_pos + facing * eye_offset_z + up * eye_offset_y - right * eye_offset_x;
        let right_eye_pos =
            head_pos + facing * eye_offset_z + up * eye_offset_y + right * eye_offset_x;

        self.add_sphere(
            left_eye_pos,
            eye_radius,
            Colour::WHITE,
            &mut vertices,
            &mut indices,
        );
        self.add_sphere(
            right_eye_pos,
            eye_radius,
            Colour::WHITE,
            &mut vertices,
            &mut indices,
        );

        // Pupils
        let pupil_radius = eye_radius * 0.5;
        let pupil_offset = eye_radius * 0.6;
        self.add_sphere(
            left_eye_pos + facing * pupil_offset,
            pupil_radius,
            Colour::BLACK,
            &mut vertices,
            &mut indices,
        );
        self.add_sphere(
            right_eye_pos + facing * pupil_offset,
            pupil_radius,
            Colour::BLACK,
            &mut vertices,
            &mut indices,
        );

        // Hands and feet as spheres
        let hand_radius = limb_radius * 2.0;
        let foot_radius = limb_radius * 2.5;

        self.add_sphere(
            skeleton.joint_position(joints.left_hand),
            hand_radius,
            self.mesh_config.extremity_colour,
            &mut vertices,
            &mut indices,
        );
        self.add_sphere(
            skeleton.joint_position(joints.right_hand),
            hand_radius,
            self.mesh_config.extremity_colour,
            &mut vertices,
            &mut indices,
        );
        self.add_sphere(
            skeleton.joint_position(joints.left_foot),
            foot_radius,
            self.mesh_config.extremity_colour,
            &mut vertices,
            &mut indices,
        );
        self.add_sphere(
            skeleton.joint_position(joints.right_foot),
            foot_radius,
            self.mesh_config.extremity_colour,
            &mut vertices,
            &mut indices,
        );

        (vertices, indices)
    }

    /// Collect all bone segments from the skeleton.
    fn collect_bones(
        &self,
        skeleton: &HumanoidSkeleton,
        limb_radius: f32,
        thick_radius: f32,
    ) -> Vec<BoneSegment> {
        let joints = &skeleton.joints;
        let body = self.mesh_config.body_colour;

        vec![
            // Spine
            BoneSegment {
                start: skeleton.joint_position(joints.hips),
                end: skeleton.joint_position(joints.spine),
                radius: thick_radius,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.spine),
                end: skeleton.joint_position(joints.chest),
                radius: thick_radius,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.chest),
                end: skeleton.joint_position(joints.neck),
                radius: thick_radius * 0.8,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.neck),
                end: skeleton.joint_position(joints.head),
                radius: thick_radius * 0.5,
                colour: body,
            },
            // Left arm
            BoneSegment {
                start: skeleton.joint_position(joints.chest),
                end: skeleton.joint_position(joints.left_shoulder),
                radius: limb_radius * 1.2,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.left_shoulder),
                end: skeleton.joint_position(joints.left_elbow),
                radius: limb_radius,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.left_elbow),
                end: skeleton.joint_position(joints.left_hand),
                radius: limb_radius * 0.9,
                colour: body,
            },
            // Right arm
            BoneSegment {
                start: skeleton.joint_position(joints.chest),
                end: skeleton.joint_position(joints.right_shoulder),
                radius: limb_radius * 1.2,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.right_shoulder),
                end: skeleton.joint_position(joints.right_elbow),
                radius: limb_radius,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.right_elbow),
                end: skeleton.joint_position(joints.right_hand),
                radius: limb_radius * 0.9,
                colour: body,
            },
            // Left leg
            BoneSegment {
                start: skeleton.joint_position(joints.hips),
                end: skeleton.joint_position(joints.left_hip),
                radius: thick_radius * 0.9,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.left_hip),
                end: skeleton.joint_position(joints.left_knee),
                radius: limb_radius * 1.3,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.left_knee),
                end: skeleton.joint_position(joints.left_foot),
                radius: limb_radius * 1.1,
                colour: body,
            },
            // Right leg
            BoneSegment {
                start: skeleton.joint_position(joints.hips),
                end: skeleton.joint_position(joints.right_hip),
                radius: thick_radius * 0.9,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.right_hip),
                end: skeleton.joint_position(joints.right_knee),
                radius: limb_radius * 1.3,
                colour: body,
            },
            BoneSegment {
                start: skeleton.joint_position(joints.right_knee),
                end: skeleton.joint_position(joints.right_foot),
                radius: limb_radius * 1.1,
                colour: body,
            },
        ]
    }

    /// Add a capsule (cylinder with hemispherical caps) to the mesh.
    fn add_capsule(&self, bone: &BoneSegment, vertices: &mut Vec<Vertex>, indices: &mut Vec<u32>) {
        let start_idx = vertices.len() as u32;

        let direction = bone.end - bone.start;
        let length = direction.magnitude();

        if length < 0.0001 {
            // Degenerate - just add a sphere
            self.add_sphere(bone.start, bone.radius, bone.colour, vertices, indices);
            return;
        }

        let axis = direction.normalize();

        // Find perpendicular vectors
        let up = if axis.y.abs() < 0.99 {
            Vector3::y()
        } else {
            Vector3::x()
        };
        let right = axis.cross(&up).normalize();
        let forward = right.cross(&axis).normalize();

        let radial = self.mesh_config.radial_segments;
        let length_segs = self.mesh_config.length_segments;

        let colour = Vector4::new(bone.colour.r, bone.colour.g, bone.colour.b, bone.colour.a);

        // Generate vertices along the capsule
        // Start cap
        for i in 0..=radial {
            let angle = (i as f32 / radial as f32) * std::f32::consts::TAU;
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            // Start hemisphere
            for j in 0..=length_segs / 2 {
                let phi = (j as f32 / (length_segs / 2) as f32) * std::f32::consts::FRAC_PI_2;
                let y_offset = -phi.cos() * bone.radius;
                let r = phi.sin() * bone.radius;

                let pos =
                    bone.start + axis * y_offset + right * (cos_a * r) + forward * (sin_a * r);
                let normal = (pos - (bone.start + axis * y_offset)).normalize();

                vertices.push(Vertex {
                    pos: Vector4::new(pos.x, pos.y, pos.z, 1.0),
                    color: colour,
                    tex_coords: Vector2::new(
                        i as f32 / radial as f32,
                        j as f32 / length_segs as f32,
                    ),
                    normal,
                });
            }

            // Cylinder body
            for j in 1..length_segs {
                let t = j as f32 / length_segs as f32;
                let pos = bone.start
                    + axis * (t * length)
                    + right * (cos_a * bone.radius)
                    + forward * (sin_a * bone.radius);
                let normal = (right * cos_a + forward * sin_a).normalize();

                vertices.push(Vertex {
                    pos: Vector4::new(pos.x, pos.y, pos.z, 1.0),
                    color: colour,
                    tex_coords: Vector2::new(i as f32 / radial as f32, 0.25 + t * 0.5),
                    normal,
                });
            }

            // End hemisphere
            for j in 0..=length_segs / 2 {
                let phi = (j as f32 / (length_segs / 2) as f32) * std::f32::consts::FRAC_PI_2;
                let y_offset = phi.sin() * bone.radius;
                let r = phi.cos() * bone.radius;

                let pos = bone.end + axis * y_offset + right * (cos_a * r) + forward * (sin_a * r);
                let normal = (pos - (bone.end + axis * y_offset)).normalize();

                vertices.push(Vertex {
                    pos: Vector4::new(pos.x, pos.y, pos.z, 1.0),
                    color: colour,
                    tex_coords: Vector2::new(
                        i as f32 / radial as f32,
                        0.75 + j as f32 / length_segs as f32 * 0.25,
                    ),
                    normal,
                });
            }
        }

        // Generate indices (counter-clockwise winding for outward-facing normals)
        let verts_per_ring = (length_segs / 2 + 1) + (length_segs - 1) + (length_segs / 2 + 1);

        for i in 0..radial {
            for j in 0..verts_per_ring - 1 {
                let a = start_idx + i * verts_per_ring as u32 + j as u32;
                let b = start_idx + (i + 1) * verts_per_ring as u32 + j as u32;
                let c = start_idx + (i + 1) * verts_per_ring as u32 + (j + 1) as u32;
                let d = start_idx + i * verts_per_ring as u32 + (j + 1) as u32;

                // Two triangles per quad, counter-clockwise winding
                indices.extend_from_slice(&[a, d, b]);
                indices.extend_from_slice(&[d, c, b]);
            }
        }
    }

    /// Add a sphere to the mesh.
    fn add_sphere(
        &self,
        center: Point3<f32>,
        radius: f32,
        colour: Colour,
        vertices: &mut Vec<Vertex>,
        indices: &mut Vec<u32>,
    ) {
        let start_idx = vertices.len() as u32;

        let stacks = 8u32;
        let slices = 12u32;

        let colour_vec = Vector4::new(colour.r, colour.g, colour.b, colour.a);

        // Generate vertices
        for i in 0..=stacks {
            let phi = (i as f32 / stacks as f32) * std::f32::consts::PI;
            let y = phi.cos() * radius;
            let r = phi.sin() * radius;

            for j in 0..=slices {
                let theta = (j as f32 / slices as f32) * std::f32::consts::TAU;
                let x = theta.cos() * r;
                let z = theta.sin() * r;

                let pos = center + Vector3::new(x, y, z);
                let normal = Vector3::new(x, y, z).normalize();

                vertices.push(Vertex {
                    pos: Vector4::new(pos.x, pos.y, pos.z, 1.0),
                    color: colour_vec,
                    tex_coords: Vector2::new(j as f32 / slices as f32, i as f32 / stacks as f32),
                    normal,
                });
            }
        }

        // Generate indices (matching geometry/sphere.rs winding order)
        for i in 0..stacks {
            for j in 0..slices {
                let a = start_idx + i * (slices + 1) + j; // current stack, current slice
                let b = start_idx + (i + 1) * (slices + 1) + j; // next stack, current slice
                let c = start_idx + (i + 1) * (slices + 1) + (j + 1); // next stack, next slice
                let d = start_idx + i * (slices + 1) + (j + 1); // current stack, next slice

                // First triangle: a, d, b (counter-clockwise when viewed from outside)
                indices.extend_from_slice(&[a, d, b]);

                // Second triangle: d, c, b
                indices.extend_from_slice(&[d, c, b]);
            }
        }
    }
}

impl Default for HumanoidMeshGenerator {
    fn default() -> Self {
        Self::new(HumanoidMeshConfig::default())
    }
}
