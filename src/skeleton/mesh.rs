//! Procedural mesh generation from skeleton.
//!
//! Generates renderable geometry (capsules/spheres) from skeleton joint positions.
//! This gives the character a "bean" or "noodle" look typical of platformer characters.

use nalgebra::{Point3, Vector2, Vector3, Vector4};

use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

use super::biped::SpringBipedSkeleton;

// ============================================================================
// Spring Biped Mesh Generator (Stage 3 - with legs and knees)
// ============================================================================

/// Configuration for the spring biped mesh.
///
/// Full biped with pelvis, legs (with knees), and feet.
#[derive(Clone, Debug)]
pub struct SpringBipedMeshConfig {
    /// Color of the pelvis/body.
    pub body_colour: Colour,
    /// Color of the legs.
    pub leg_colour: Colour,
    /// Color of the feet.
    pub foot_colour: Colour,
    /// Radius of the pelvis sphere.
    pub pelvis_radius: f32,
    /// Radius of the knee spheres.
    pub knee_radius: f32,
    /// Radius of the foot spheres.
    pub foot_radius: f32,
    /// Radius of the leg capsules.
    pub leg_radius: f32,
    /// Segments around capsules (higher = smoother).
    pub radial_segments: u32,
    /// Segments along capsule length.
    pub length_segments: u32,
}

impl Default for SpringBipedMeshConfig {
    fn default() -> Self {
        Self {
            body_colour: Colour::new(0.8, 0.5, 0.3, 1.0), // Orange-ish body
            leg_colour: Colour::new(0.6, 0.4, 0.25, 1.0), // Slightly darker legs
            foot_colour: Colour::new(0.3, 0.3, 0.35, 1.0), // Dark feet
            pelvis_radius: 0.1,
            knee_radius: 0.04,
            foot_radius: 0.06,
            leg_radius: 0.03,
            radial_segments: 8,
            length_segments: 4,
        }
    }
}

/// Generate mesh data for a spring biped skeleton.
///
/// Renders pelvis, legs (upper + lower with knees), and feet.
pub struct SpringBipedMeshGenerator {
    config: SpringBipedMeshConfig,
}

impl SpringBipedMeshGenerator {
    pub fn new(config: SpringBipedMeshConfig) -> Self {
        Self { config }
    }

    /// Generate mesh vertices and indices from a spring biped skeleton.
    pub fn generate(&self, skeleton: &SpringBipedSkeleton) -> (Vec<Vertex>, Vec<u32>) {
        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        let pelvis = skeleton.pelvis_position();
        let left_hip = skeleton.left_hip();
        let right_hip = skeleton.right_hip();
        let left_knee = skeleton.left_knee_position();
        let right_knee = skeleton.right_knee_position();
        let left_foot = skeleton.left_foot_position();
        let right_foot = skeleton.right_foot_position();

        // Pelvis sphere
        self.add_sphere(
            pelvis,
            self.config.pelvis_radius,
            self.config.body_colour,
            &mut vertices,
            &mut indices,
        );

        // Left leg: hip -> knee -> foot
        self.add_capsule(
            left_hip,
            left_knee,
            self.config.leg_radius,
            self.config.leg_colour,
            &mut vertices,
            &mut indices,
        );
        self.add_capsule(
            left_knee,
            left_foot,
            self.config.leg_radius * 0.9,
            self.config.leg_colour,
            &mut vertices,
            &mut indices,
        );

        // Right leg: hip -> knee -> foot
        self.add_capsule(
            right_hip,
            right_knee,
            self.config.leg_radius,
            self.config.leg_colour,
            &mut vertices,
            &mut indices,
        );
        self.add_capsule(
            right_knee,
            right_foot,
            self.config.leg_radius * 0.9,
            self.config.leg_colour,
            &mut vertices,
            &mut indices,
        );

        // Knee spheres
        self.add_sphere(
            left_knee,
            self.config.knee_radius,
            self.config.leg_colour,
            &mut vertices,
            &mut indices,
        );
        self.add_sphere(
            right_knee,
            self.config.knee_radius,
            self.config.leg_colour,
            &mut vertices,
            &mut indices,
        );

        // Foot spheres
        self.add_sphere(
            left_foot,
            self.config.foot_radius,
            self.config.foot_colour,
            &mut vertices,
            &mut indices,
        );
        self.add_sphere(
            right_foot,
            self.config.foot_radius,
            self.config.foot_colour,
            &mut vertices,
            &mut indices,
        );

        // Add eyes on pelvis (character face)
        let facing = skeleton.facing_direction();
        let up = Vector3::y();
        let right = up.cross(&facing).normalize();

        let eye_radius = self.config.pelvis_radius * 0.2;
        let eye_offset_forward = self.config.pelvis_radius * 0.85;
        let eye_offset_up = self.config.pelvis_radius * 0.3;
        let eye_offset_side = self.config.pelvis_radius * 0.35;

        let left_eye_pos =
            pelvis + facing * eye_offset_forward + up * eye_offset_up - right * eye_offset_side;
        let right_eye_pos =
            pelvis + facing * eye_offset_forward + up * eye_offset_up + right * eye_offset_side;

        // White eye balls
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

        // Black pupils
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

        (vertices, indices)
    }

    /// Add a capsule (cylinder with hemispherical caps) to the mesh.
    fn add_capsule(
        &self,
        start: Point3<f32>,
        end: Point3<f32>,
        radius: f32,
        colour: Colour,
        vertices: &mut Vec<Vertex>,
        indices: &mut Vec<u32>,
    ) {
        let start_idx = vertices.len() as u32;

        let direction = end - start;
        let length = direction.magnitude();

        if length < 0.0001 {
            self.add_sphere(start, radius, colour, vertices, indices);
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

        let radial = self.config.radial_segments;
        let length_segs = self.config.length_segments;

        let colour_vec = Vector4::new(colour.r, colour.g, colour.b, colour.a);

        // Generate vertices along the capsule
        for i in 0..=radial {
            let angle = (i as f32 / radial as f32) * std::f32::consts::TAU;
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            // Start hemisphere
            for j in 0..=length_segs / 2 {
                let phi = (j as f32 / (length_segs / 2) as f32) * std::f32::consts::FRAC_PI_2;
                let y_offset = -phi.cos() * radius;
                let r = phi.sin() * radius;

                let pos = start + axis * y_offset + right * (cos_a * r) + forward * (sin_a * r);
                let normal = (pos - (start + axis * y_offset)).normalize();

                vertices.push(Vertex {
                    pos: Vector4::new(pos.x, pos.y, pos.z, 1.0),
                    color: colour_vec,
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
                let pos = start
                    + axis * (t * length)
                    + right * (cos_a * radius)
                    + forward * (sin_a * radius);
                let normal = (right * cos_a + forward * sin_a).normalize();

                vertices.push(Vertex {
                    pos: Vector4::new(pos.x, pos.y, pos.z, 1.0),
                    color: colour_vec,
                    tex_coords: Vector2::new(i as f32 / radial as f32, 0.25 + t * 0.5),
                    normal,
                });
            }

            // End hemisphere
            for j in 0..=length_segs / 2 {
                let phi = (j as f32 / (length_segs / 2) as f32) * std::f32::consts::FRAC_PI_2;
                let y_offset = phi.sin() * radius;
                let r = phi.cos() * radius;

                let pos = end + axis * y_offset + right * (cos_a * r) + forward * (sin_a * r);
                let normal = (pos - (end + axis * y_offset)).normalize();

                vertices.push(Vertex {
                    pos: Vector4::new(pos.x, pos.y, pos.z, 1.0),
                    color: colour_vec,
                    tex_coords: Vector2::new(
                        i as f32 / radial as f32,
                        0.75 + j as f32 / length_segs as f32 * 0.25,
                    ),
                    normal,
                });
            }
        }

        // Generate indices
        let verts_per_ring = (length_segs / 2 + 1) + (length_segs - 1) + (length_segs / 2 + 1);

        for i in 0..radial {
            for j in 0..verts_per_ring - 1 {
                let a = start_idx + i * verts_per_ring as u32 + j as u32;
                let b = start_idx + (i + 1) * verts_per_ring as u32 + j as u32;
                let c = start_idx + (i + 1) * verts_per_ring as u32 + (j + 1) as u32;
                let d = start_idx + i * verts_per_ring as u32 + (j + 1) as u32;

                indices.extend_from_slice(&[a, d, b]);
                indices.extend_from_slice(&[d, c, b]);
            }
        }
    }

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

        for i in 0..stacks {
            for j in 0..slices {
                let a = start_idx + i * (slices + 1) + j;
                let b = start_idx + (i + 1) * (slices + 1) + j;
                let c = start_idx + (i + 1) * (slices + 1) + (j + 1);
                let d = start_idx + i * (slices + 1) + (j + 1);

                indices.extend_from_slice(&[a, d, b]);
                indices.extend_from_slice(&[d, c, b]);
            }
        }
    }
}

impl Default for SpringBipedMeshGenerator {
    fn default() -> Self {
        Self::new(SpringBipedMeshConfig::default())
    }
}
