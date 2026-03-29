//! Domino spawnable — a row of tall, thin blocks spaced for chain toppling.
//!
//! Each domino gets a unique procedural texture with a coloured face and
//! pip dots, like a real domino tile.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::Entity;

use super::shared::models::cuboid_model;
use super::shared::textures::*;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId, MaterialManagerBuilder};
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

use specs::{Builder, WorldExt};

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct DominoDef {
    /// Position of the first domino's base centre.
    pub base: (f32, f32, f32),
    /// Direction the row extends in (normalized internally).
    pub direction: (f32, f32),
    #[serde(default = "DominoDef::default_count")]
    pub count: u32,
    #[serde(default = "DominoDef::default_spacing")]
    pub spacing: f32,
    #[serde(default = "DominoDef::default_half_extents")]
    pub half_extents: (f32, f32, f32),
    #[serde(default = "DominoDef::default_density")]
    pub density: f32,
}

impl DominoDef {
    pub fn default_count() -> u32 {
        8
    }
    pub fn default_spacing() -> f32 {
        0.70
    }
    pub fn default_density() -> f32 {
        120.0
    }
    pub fn default_half_extents() -> (f32, f32, f32) {
        (0.30, 0.60, 0.08)
    }
}

impl Spawnable for DominoDef {
    fn material_count(&self) -> usize {
        self.count as usize
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let mut mats = Vec::with_capacity(self.count as usize);
        for i in 0..self.count {
            mats.push(create_domino_material(
                i,
                self.count,
                ctx.textures,
                ctx.materials,
            )?);
        }
        Ok(mats)
    }

    fn spawn(&self, world: &mut specs::World, materials: &[MaterialId]) -> Vec<Entity> {
        let he = Vector3::new(self.half_extents.0, self.half_extents.1, self.half_extents.2);

        let dir_len = (self.direction.0 * self.direction.0 + self.direction.1 * self.direction.1)
            .sqrt()
            .max(1e-6);
        let dir_x = self.direction.0 / dir_len;
        let dir_z = self.direction.1 / dir_len;

        // Orient dominoes so their thin axis (Z) aligns with the row direction.
        let facing_angle = dir_x.atan2(dir_z);
        let orientation =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), facing_angle);

        let mut entities = Vec::with_capacity(self.count as usize);

        for i in 0..self.count as usize {
            let offset = i as f32 * self.spacing;
            let pos = Point3::new(
                self.base.0 + dir_x * offset,
                self.base.1 + he.y,
                self.base.2 + dir_z * offset,
            );
            let model = cuboid_model(he, materials[i]);

            let body_handle = {
                let mut physics = world.write_resource::<PhysicsResource>();
                let body_desc = RigidBodyDesc::dynamic()
                    .position(pos)
                    .rotation(orientation)
                    .gravity_scale(1.0)
                    .linear_damping(0.01)
                    .angular_damping(0.005);
                let body_handle = physics.world.create_body(body_desc);
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::box_shape(he)
                        .density(self.density)
                        .restitution(0.1)
                        .friction(0.5),
                );
                body_handle
            };

            entities.push(
                world
                    .create_entity()
                    .with(Position(Vector3::new(pos.x, pos.y, pos.z)))
                    .with(Velocity(Vector3::zeros()))
                    .with(Orientation(orientation))
                    .with(RigidBodyComponent(body_handle))
                    .with(ModelInstance::new(model))
                    .with(Renderable)
                    .build(),
            );
        }

        entities
    }
}

// ---------------------------------------------------------------------------
// Domino texture generation
// ---------------------------------------------------------------------------

fn create_domino_material(
    index: u32,
    total: u32,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId> {
    let pixels = generate_domino_tile(index, total);
    let texture = texture_manager.create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
    let material = Material::textured(texture);
    Ok(material_builder.register(material))
}

/// Procedural domino tile: ivory/cream body with a coloured centre stripe
/// and pip dots. Each domino in the row gets a different pip count and hue.
fn generate_domino_tile(index: u32, total: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let seed = rand_u32();

    // Ivory base with subtle per-tile warmth variation
    let warmth = rand_range(-0.02, 0.02);
    let base = Rgb::new(0.92 + warmth, 0.90 + warmth, 0.85 + warmth);

    // Each domino gets a unique accent hue spread evenly around the colour wheel
    let hue = (index as f32 / total.max(1) as f32) * 6.0 + rand_range(0.0, 0.5);
    let accent = hue_to_rgb(hue, 0.55, 0.75);

    // Pip count: 1–6 based on position in the row
    let pips = (index % 6) + 1;

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            // Subtle surface noise
            let noise = fbm_2d_periodic(u * 10.0, v * 10.0, 2, 0.3, 2.0, seed, Some(10));
            let noise_factor = 0.95 + noise * 0.05;

            let mut c = base.scale(noise_factor);

            // Centre divider line
            let divider_dist = (v - 0.5).abs();
            if divider_dist < 0.015 {
                c = accent.scale(0.6);
            }

            // Coloured accent border (inset frame)
            let inset = 0.08;
            let border_width = 0.03;
            let du = u.min(1.0 - u);
            let dv = v.min(1.0 - v);
            let in_border = (du > inset && du < inset + border_width)
                || (dv > inset && dv < inset + border_width);
            let in_inset = du > inset && dv > inset;
            if in_border && in_inset {
                c = c.lerp(accent, 0.6);
            }

            // Pip dots — placed in the top half (v < 0.5)
            let pip_intensity = pip_pattern(u, v, pips);
            if pip_intensity > 0.0 {
                c = c.lerp(accent, pip_intensity * 0.85);
            }

            // Rounded edge darkening
            c = c.scale(edge_vignette(u, v));

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}

/// Returns intensity (0–1) for pip dots at the given UV coordinate.
/// Pips are placed in a standard domino layout within the top half (v 0.15–0.45).
fn pip_pattern(u: f32, v: f32, pips: u32) -> f32 {
    let pip_radius = 0.04;

    let positions: &[(f32, f32)] = match pips {
        1 => &[(0.5, 0.25)],
        2 => &[(0.35, 0.20), (0.65, 0.30)],
        3 => &[(0.35, 0.18), (0.5, 0.25), (0.65, 0.32)],
        4 => &[(0.35, 0.18), (0.65, 0.18), (0.35, 0.32), (0.65, 0.32)],
        5 => &[
            (0.35, 0.18),
            (0.65, 0.18),
            (0.5, 0.25),
            (0.35, 0.32),
            (0.65, 0.32),
        ],
        _ => &[
            (0.35, 0.18),
            (0.65, 0.18),
            (0.35, 0.25),
            (0.65, 0.25),
            (0.35, 0.32),
            (0.65, 0.32),
        ],
    };

    let mut best = 0.0f32;
    for &(pu, pv) in positions {
        let du = u - pu;
        let dv = v - pv;
        let dist = (du * du + dv * dv).sqrt();
        if dist < pip_radius {
            let intensity = 1.0 - (dist / pip_radius);
            best = best.max(intensity);
        }
    }

    // Mirror pips into the bottom half
    if best == 0.0 {
        let mirrored_v = 1.0 - v;
        for &(pu, pv) in positions {
            let du = u - pu;
            let dv = mirrored_v - pv;
            let dist = (du * du + dv * dv).sqrt();
            if dist < pip_radius {
                let intensity = 1.0 - (dist / pip_radius);
                best = best.max(intensity);
            }
        }
    }

    best
}
