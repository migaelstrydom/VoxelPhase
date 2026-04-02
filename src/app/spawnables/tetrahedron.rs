//! Tetrahedron spawnable — regular tetrahedron with ConvexHull collider.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{build_convex_hull, convex_solid_model, SolidFace};
use super::shared::textures::hue_to_rgb;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct TetrahedronDef {
    pub pos: (f32, f32, f32),
    /// Edge length of the regular tetrahedron.
    #[serde(default = "TetrahedronDef::default_size")]
    pub size: f32,
    #[serde(default = "TetrahedronDef::default_density")]
    pub density: f32,
    #[serde(default = "TetrahedronDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "TetrahedronDef::default_friction")]
    pub friction: f32,
}

impl TetrahedronDef {
    pub fn default_size() -> f32 {
        1.0
    }
    pub fn default_density() -> f32 {
        500.0
    }
    pub fn default_restitution() -> f32 {
        0.2
    }
    pub fn default_friction() -> f32 {
        0.6
    }
}

impl Spawnable for TetrahedronDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let pixels = generate_faceted_texture();
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        let material = Material::textured(texture);
        Ok(vec![ctx.materials.register(material)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let material = materials[0];

        let (vertices, faces) = tetrahedron_geometry(self.size);
        let hull = Arc::new(build_convex_hull(&vertices, &faces));
        let model = convex_solid_model(&vertices, &faces, material);

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.005);

            let body_handle = physics.world.create_body(body_desc);

            let collider_desc = ColliderDesc::convex_hull(hull)
                .density(self.density)
                .restitution(self.restitution)
                .friction(self.friction);

            physics.world.attach_collider(body_handle, collider_desc);

            body_handle
        };

        vec![world
            .create_entity()
            .with(Position(Vector3::new(
                initial_pos.x,
                initial_pos.y,
                initial_pos.z,
            )))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build()]
    }
}

/// Vertices and faces of a regular tetrahedron with the given edge length.
fn tetrahedron_geometry(edge: f32) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let r = edge * (6.0f32).sqrt() / 4.0;
    let top = Vector3::new(0.0, r, 0.0);

    let y_base = -r / 3.0;
    let base_r = (r * r - y_base * y_base).sqrt();

    let v0 = Vector3::new(0.0, y_base, base_r);
    let v1 = Vector3::new(
        base_r * (2.0 * std::f32::consts::PI / 3.0).sin(),
        y_base,
        base_r * (2.0 * std::f32::consts::PI / 3.0).cos(),
    );
    let v2 = Vector3::new(
        base_r * (4.0 * std::f32::consts::PI / 3.0).sin(),
        y_base,
        base_r * (4.0 * std::f32::consts::PI / 3.0).cos(),
    );

    let vertices = vec![top, v0, v1, v2];
    let faces = vec![
        SolidFace {
            vertex_indices: vec![1, 2, 3],
            opposite_vertex: 0,
        },
        SolidFace {
            vertex_indices: vec![0, 2, 1],
            opposite_vertex: 3,
        },
        SolidFace {
            vertex_indices: vec![0, 3, 2],
            opposite_vertex: 1,
        },
        SolidFace {
            vertex_indices: vec![0, 1, 3],
            opposite_vertex: 2,
        },
    ];

    (vertices, faces)
}

/// Procedural texture with a crystalline/gemstone look.
fn generate_faceted_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let hue = rand::random::<f32>() * 6.0;
    let base = hue_to_rgb(hue, 0.50, 0.75);
    let seed = rand::random::<u32>();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let crystal = fbm_2d_periodic(u * 6.0, v * 6.0, 4, 0.5, 2.0, seed, Some(6));
            let factor = 0.80 + crystal * 0.20;
            let c = base.scale(factor);

            let edge = ((u - 0.5).abs().max((v - 0.5).abs()) * 4.0).min(1.0);
            let c = c.scale(0.92 + edge * 0.08);

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
