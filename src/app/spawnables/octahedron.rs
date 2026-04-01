//! Octahedron spawnable — regular octahedron with ConvexHull collider.

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
pub struct OctahedronDef {
    pub pos: (f32, f32, f32),
    /// Edge length.
    #[serde(default = "OctahedronDef::default_size")]
    pub size: f32,
    #[serde(default = "OctahedronDef::default_density")]
    pub density: f32,
    #[serde(default = "OctahedronDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "OctahedronDef::default_friction")]
    pub friction: f32,
}

impl OctahedronDef {
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

impl Spawnable for OctahedronDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let pixels = generate_metallic_texture();
        let texture =
            ctx.textures
                .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        let material = Material::textured(texture);
        Ok(vec![ctx.materials.register(material)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let material = materials[0];

        let (vertices, faces) = octahedron_geometry(self.size);
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

/// Vertices and faces of a regular octahedron with the given edge length.
///
/// 6 vertices (±a on each axis), 8 triangular faces.
fn octahedron_geometry(edge: f32) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    // For a regular octahedron with edge length e,
    // the circumradius (vertex distance from origin) is e / sqrt(2).
    let a = edge / (2.0f32).sqrt();

    let vertices = vec![
        Vector3::new(0.0, a, 0.0),  // 0: +Y (top)
        Vector3::new(0.0, -a, 0.0), // 1: -Y (bottom)
        Vector3::new(a, 0.0, 0.0),  // 2: +X
        Vector3::new(-a, 0.0, 0.0), // 3: -X
        Vector3::new(0.0, 0.0, a),  // 4: +Z
        Vector3::new(0.0, 0.0, -a), // 5: -Z
    ];

    // 8 triangular faces. Each face's opposite_vertex is the vertex on the
    // opposite side of the origin from the face.
    let faces = vec![
        // Upper four faces (contain vertex 0 = +Y), opposite is 1 = -Y.
        SolidFace { vertex_indices: vec![0, 4, 2], opposite_vertex: 1 },
        SolidFace { vertex_indices: vec![0, 2, 5], opposite_vertex: 1 },
        SolidFace { vertex_indices: vec![0, 5, 3], opposite_vertex: 1 },
        SolidFace { vertex_indices: vec![0, 3, 4], opposite_vertex: 1 },
        // Lower four faces (contain vertex 1 = -Y), opposite is 0 = +Y.
        SolidFace { vertex_indices: vec![1, 2, 4], opposite_vertex: 0 },
        SolidFace { vertex_indices: vec![1, 5, 2], opposite_vertex: 0 },
        SolidFace { vertex_indices: vec![1, 3, 5], opposite_vertex: 0 },
        SolidFace { vertex_indices: vec![1, 4, 3], opposite_vertex: 0 },
    ];

    (vertices, faces)
}

/// Procedural metallic/brushed texture.
fn generate_metallic_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let tint = rand::random::<f32>() * 6.0;
    let base = hue_to_rgb(tint, 0.20, 0.70);
    let seed = rand::random::<u32>();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let brush = fbm_2d_periodic(u * 6.0, v * 24.0, 3, 0.5, 2.0, seed, Some(6));
            let speck = fbm_2d_periodic(u * 16.0, v * 16.0, 2, 0.3, 2.0, seed + 7, Some(16));
            let factor = 0.82 + brush * 0.12 + speck * 0.06;
            let c = base.scale(factor);

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
