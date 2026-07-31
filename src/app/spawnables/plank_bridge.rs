//! Plank bridge spawnable — rustic destructible bridge.
//!
//! Two long beams (stringers) run end to end. Cross-planks are laid across
//! them haphazardly: slightly rotated, varying widths, with small gaps.
//! The whole structure is a single compound body with fracture joints so
//! it shatters on explosion.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::multi_material_rotated_compound_cuboid_model;
use super::shared::orientation::Yaw;
use super::shared::textures::*;
use super::spawnable::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::{CompoundFracture, FractureJoint};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct PlankBridgeDef {
    /// World position of the bridge center (bottom of the beams).
    pub pos: (f32, f32, f32),
    /// Total length of the bridge along the Z axis.
    #[serde(default = "PlankBridgeDef::default_length")]
    pub length: f32,
    /// Lateral spacing between the two beam centers.
    #[serde(default = "PlankBridgeDef::default_beam_spacing")]
    pub beam_spacing: f32,
    /// Number of cross-planks.
    #[serde(default = "PlankBridgeDef::default_plank_count")]
    pub plank_count: u32,
    /// Half-extents of each stringer beam (x, y, z).
    #[serde(default = "PlankBridgeDef::default_beam_half_extents")]
    pub beam_half_extents: (f32, f32, f32),
    /// Half-extents of each cross-plank (x, y, z). X spans the bridge width.
    #[serde(default = "PlankBridgeDef::default_plank_half_extents")]
    pub plank_half_extents: (f32, f32, f32),
    #[serde(default = "PlankBridgeDef::default_density")]
    pub density: f32,
    /// Rotation about `+Y`, in degrees. 0 = the bridge runs along Z.
    ///
    /// Degrees rather than radians so it matches every other yaw in the level
    /// format — anchors, placements and the other oriented spawnables.
    #[serde(default)]
    pub yaw: f32,
    /// Impulse threshold for fracture joints.
    #[serde(default = "PlankBridgeDef::default_fracture_threshold")]
    pub fracture_threshold: f32,
}

impl PlankBridgeDef {
    pub fn default_length() -> f32 {
        8.0
    }
    pub fn default_beam_spacing() -> f32 {
        1.6
    }
    pub fn default_plank_count() -> u32 {
        8
    }
    pub fn default_beam_half_extents() -> (f32, f32, f32) {
        (0.18, 0.16, 4.0)
    }
    pub fn default_plank_half_extents() -> (f32, f32, f32) {
        (1.0, 0.12, 0.38)
    }
    pub fn default_density() -> f32 {
        500.0
    }
    pub fn default_fracture_threshold() -> f32 {
        8.0
    }

    fn beam_he(&self) -> Vector3<f32> {
        // Override beam Z to half the bridge length.
        Vector3::new(
            self.beam_half_extents.0,
            self.beam_half_extents.1,
            self.length / 2.0,
        )
    }

    fn plank_he(&self) -> Vector3<f32> {
        // Override plank X to span beam spacing + overhang.
        let span_x = self.beam_spacing / 2.0 + self.beam_half_extents.0 + 0.25;
        Vector3::new(span_x, self.plank_half_extents.1, self.plank_half_extents.2)
    }

    fn child_count(&self) -> usize {
        2 + self.plank_count as usize
    }
}

impl Spawnable for PlankBridgeDef {
    fn material_count(&self) -> usize {
        2
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let beam_pixels = generate_beam_wood();
        let beam_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &beam_pixels, true)?;
        let beam_mat = ctx.materials.register(Material::textured(beam_tex));

        let plank_pixels = generate_plank_wood();
        let plank_tex =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &plank_pixels, true)?;
        let plank_mat = ctx.materials.register(Material::textured(plank_tex));

        Ok(vec![beam_mat, plank_mat])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let center = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let beam_he = self.beam_he();
        let plank_he = self.plank_he();
        let beam_mat = materials[0];
        let plank_mat = materials[1];

        let beam_y = beam_he.y;
        let plank_y = beam_he.y * 2.0 + plank_he.y;
        let half_spacing = self.beam_spacing / 2.0;

        // Child 0 = left beam, child 1 = right beam.
        let beam_offsets = [
            Vector3::new(-half_spacing, beam_y, 0.0),
            Vector3::new(half_spacing, beam_y, 0.0),
        ];

        // Generate haphazard plank placements.
        let usable_length = self.length - plank_he.z * 2.0;
        let base_spacing = usable_length / (self.plank_count as f32);
        let start_z = -usable_length / 2.0;

        let seed = rand_u32();
        let min_gap = 0.02;

        // Pre-compute per-plank randomised properties.
        struct PlankLayout {
            z: f32,
            half_w: f32,
            yaw: f32,
            /// Conservative Z half-footprint accounting for yaw rotation.
            z_footprint: f32,
            hash: u32,
        }

        let mut raw_planks: Vec<PlankLayout> = (0..self.plank_count)
            .map(|i| {
                let h = hash_pair(seed as i32, i as i32);
                let jitter_z = pseudo_range(h, 0) * base_spacing * 0.25;
                let width_factor = 0.75 + pseudo_range(h, 2).abs() * 0.50;
                let z = start_z + (i as f32 + 0.5) * base_spacing + jitter_z;
                let half_w = plank_he.z * width_factor;
                let yaw = pseudo_range(h, 3) * 0.18;
                let z_footprint = plank_he.x * yaw.abs().sin() + half_w * yaw.abs().cos();
                PlankLayout {
                    z,
                    half_w,
                    yaw,
                    z_footprint,
                    hash: h,
                }
            })
            .collect();

        // Push planks apart so no two overlap (using rotated footprints).
        for i in 1..raw_planks.len() {
            let required_z = raw_planks[i - 1].z
                + raw_planks[i - 1].z_footprint
                + raw_planks[i].z_footprint
                + min_gap;
            if raw_planks[i].z < required_z {
                raw_planks[i].z = required_z;
            }
        }

        let planks: Vec<(Vector3<f32>, Vector3<f32>, UnitQuaternion<f32>)> = raw_planks
            .iter()
            .map(|p| {
                let jitter_x = pseudo_range(p.hash, 1) * 0.12;
                let offset = Vector3::new(jitter_x, plank_y, p.z);
                let varied_he = Vector3::new(plank_he.x, plank_he.y, p.half_w);
                let rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), p.yaw);
                (offset, varied_he, rotation)
            })
            .collect();

        // Build compound model.
        let identity_rot = UnitQuaternion::identity();
        let mut boxes: Vec<(Vector3<f32>, Vector3<f32>, UnitQuaternion<f32>, MaterialId)> =
            Vec::new();
        for &offset in &beam_offsets {
            boxes.push((beam_he, offset, identity_rot, beam_mat));
        }
        for &(offset, varied_he, rotation) in &planks {
            boxes.push((varied_he, offset, rotation, plank_mat));
        }
        let model = multi_material_rotated_compound_cuboid_model(&boxes);

        let orientation = Yaw::degrees(self.yaw).rotation();

        // Physics compound body.
        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(center)
                .rotation(orientation)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.005);

            let body_handle = physics.world.create_body(body_desc);

            // Beams.
            for &offset in &beam_offsets {
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(beam_he)
                        .offset_translation(offset)
                        .density(self.density)
                        .restitution(0.05)
                        .friction(0.7),
                );
            }

            // Planks — match the varied half-extents and rotation from the model.
            for &(offset, varied_he, rotation) in &planks {
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(varied_he)
                        .offset_translation(offset)
                        .offset_rotation(rotation)
                        .density(self.density)
                        .restitution(0.05)
                        .friction(0.7),
                );
            }

            body_handle
        };

        // Fracture joints: each plank connects to both beams.
        let threshold = self.fracture_threshold;
        let mut joints = Vec::with_capacity(self.plank_count as usize * 2);
        for i in 0..self.plank_count as usize {
            let plank_child = 2 + i;
            joints.push(FractureJoint {
                child_a: 0,
                child_b: plank_child,
                threshold,
            });
            joints.push(FractureJoint {
                child_a: 1,
                child_b: plank_child,
                threshold,
            });
        }

        let fracture = CompoundFracture {
            joints,
            child_count: self.child_count(),
            material: plank_mat,
        };

        vec![world
            .create_entity()
            .with(Position(Vector3::new(center.x, center.y, center.z)))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(orientation))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(fracture)
            .build()]
    }
}

/// Deterministic float in [-1, 1] from a hash and channel index.
fn pseudo_range(hash: u32, channel: u32) -> f32 {
    let h = hash
        .wrapping_mul(2654435761)
        .wrapping_add(channel.wrapping_mul(374761393));
    let h = (h ^ (h >> 16)).wrapping_mul(0x45d9f3b);
    ((h & 0xFFFF) as f32 / 32768.0) - 1.0
}

// ---------------------------------------------------------------------------
// Procedural rustic wood textures
// ---------------------------------------------------------------------------

/// Weathered beam wood — dark, knotty, with prominent grain.
fn generate_beam_wood() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let base = Rgb::new(0.35, 0.24, 0.14);
    let seed = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Heavy longitudinal grain.
            let grain = fbm_2d_periodic(u * 4.0, v * 24.0, 4, 0.55, 2.0, seed, Some(4));
            let grain_factor = 0.75 + grain * 0.25;

            // Knots — dark patches.
            let knot = fbm_2d_periodic(u * 3.0, v * 3.0, 3, 0.6, 2.0, seed + 3, Some(3));
            let knot_dark = if knot > 0.65 {
                (knot - 0.65) / 0.35 * 0.25
            } else {
                0.0
            };

            // Weathering cracks.
            let crack = crack_pattern(u, v, seed + 7);

            let mut c = base.scale(grain_factor);
            c = c.scale(1.0 - knot_dark);
            c = c.scale(1.0 - crack * 0.15);
            c = c.scale(edge_vignette(u, v));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Lighter plank wood — varied, with saw marks and nail holes.
fn generate_plank_wood() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    // Randomly pick between lighter wood tones for variety.
    let warmth = rand_range(-0.03, 0.03);
    let base = Rgb::new(0.52 + warmth, 0.38 + warmth * 0.7, 0.22 + warmth * 0.4);
    let seed = rand_u32();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Grain running along the plank length (U direction for cross-planks).
            let grain = fbm_2d_periodic(u * 20.0, v * 5.0, 3, 0.5, 2.0, seed, Some(20));
            let grain_factor = 0.82 + grain * 0.18;

            // Saw marks — faint horizontal lines.
            let saw = ((v * size as f32 * 1.5).sin() * 0.5 + 0.5).powf(8.0);
            let saw_factor = 1.0 - saw * 0.06;

            // Nail holes — two dark dots near the ends.
            let nail = nail_holes(u, v);

            // Subtle weathering.
            let weather = fbm_2d_periodic(u * 8.0, v * 8.0, 2, 0.4, 2.0, seed + 5, Some(8));
            let weather_factor = 0.92 + weather * 0.08;

            let mut c = base.scale(grain_factor * saw_factor * weather_factor);
            c = c.scale(1.0 - nail * 0.5);

            // Darken edges for that individual-plank look.
            let edge = border_band(u, v, 0.05);
            c = c.scale(1.0 - edge * 0.30);
            c = c.scale(edge_vignette(u, v));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Two rustic nail-head dots near the left and right ends of a plank.
fn nail_holes(u: f32, v: f32) -> f32 {
    let nail_radius = 0.03;
    let positions = [(0.12, 0.5), (0.88, 0.5)];

    let mut best = 0.0f32;
    for &(nu, nv) in &positions {
        let du = u - nu;
        let dv = v - nv;
        let dist = (du * du + dv * dv).sqrt();
        if dist < nail_radius {
            best = best.max(1.0 - dist / nail_radius);
        }
    }
    best
}
