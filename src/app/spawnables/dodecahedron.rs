//! Dodecahedron spawnable — regular dodecahedron with ConvexHull collider.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::models::{build_convex_hull, convex_solid_model, SolidFace};
use super::shared::textures::hue_to_rgb;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct DodecahedronDef {
    pub pos: (f32, f32, f32),
    /// Edge length.
    #[serde(default = "DodecahedronDef::default_size")]
    pub size: f32,
    #[serde(default = "DodecahedronDef::default_density")]
    pub density: f32,
    #[serde(default = "DodecahedronDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "DodecahedronDef::default_friction")]
    pub friction: f32,
}

impl DodecahedronDef {
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

    /// The one declaration of this object's physics. The collider takes the
    /// coefficients and the material takes the finish they imply, so the two
    /// cannot drift apart.
    fn surface(&self) -> PhysicalSurface {
        PhysicalSurface {
            restitution: self.restitution,
            friction: self.friction,
            density: self.density,
        }
    }
}

impl Spawnable for DodecahedronDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let pixels = generate_marble_texture();
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        let material = Material::textured(texture).with_derived_finish(self.surface());
        Ok(vec![ctx.materials.register(material)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let material = materials[0];

        let (vertices, faces) = dodecahedron_geometry(self.size);
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

            let collider_desc =
                ColliderDesc::convex_hull(hull).with_physical_surface(self.surface());

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

/// Vertices and faces of a regular dodecahedron with the given edge length.
///
/// 20 vertices, 12 pentagonal faces. Coordinates derived from the golden ratio.
fn dodecahedron_geometry(edge: f32) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let phi: f32 = (1.0 + 5.0f32.sqrt()) / 2.0;
    let inv_phi = 1.0 / phi;

    // Raw vertices of a dodecahedron with circumradius sqrt(3).
    // Three groups:
    //   8 cube vertices:      (±1, ±1, ±1)
    //   4 on XY plane:        (0, ±1/φ, ±φ)
    //   4 on YZ plane:        (±1/φ, ±φ, 0)
    //   4 on XZ plane:        (±φ, 0, ±1/φ)
    let raw = [
        // Cube vertices (0–7)
        Vector3::new(1.0, 1.0, 1.0),
        Vector3::new(1.0, 1.0, -1.0),
        Vector3::new(1.0, -1.0, 1.0),
        Vector3::new(1.0, -1.0, -1.0),
        Vector3::new(-1.0, 1.0, 1.0),
        Vector3::new(-1.0, 1.0, -1.0),
        Vector3::new(-1.0, -1.0, 1.0),
        Vector3::new(-1.0, -1.0, -1.0),
        // XZ rectangle (8–11)
        Vector3::new(0.0, inv_phi, phi),
        Vector3::new(0.0, inv_phi, -phi),
        Vector3::new(0.0, -inv_phi, phi),
        Vector3::new(0.0, -inv_phi, -phi),
        // YZ rectangle (12–15)
        Vector3::new(inv_phi, phi, 0.0),
        Vector3::new(inv_phi, -phi, 0.0),
        Vector3::new(-inv_phi, phi, 0.0),
        Vector3::new(-inv_phi, -phi, 0.0),
        // XY rectangle (16–19)
        Vector3::new(phi, 0.0, inv_phi),
        Vector3::new(phi, 0.0, -inv_phi),
        Vector3::new(-phi, 0.0, inv_phi),
        Vector3::new(-phi, 0.0, -inv_phi),
    ];

    // Scale so edge length matches requested size.
    // Raw edge length = 2/φ, so scale = edge / (2/φ) = edge * φ/2.
    let scale = edge * phi / 2.0;
    let vertices: Vec<Vector3<f32>> = raw.iter().map(|v| v * scale).collect();

    // The 12 pentagonal faces of a dodecahedron.
    // Each face lists 5 vertex indices in order. opposite_vertex is any vertex
    // on the far side (centroid-opposing vertex works for normal correction).
    let faces = vec![
        SolidFace {
            vertex_indices: vec![0, 8, 10, 2, 16],
            opposite_vertex: 7,
        },
        SolidFace {
            vertex_indices: vec![0, 16, 17, 1, 12],
            opposite_vertex: 7,
        },
        SolidFace {
            vertex_indices: vec![0, 12, 14, 4, 8],
            opposite_vertex: 3,
        },
        SolidFace {
            vertex_indices: vec![1, 17, 3, 11, 9],
            opposite_vertex: 4,
        },
        SolidFace {
            vertex_indices: vec![1, 9, 5, 14, 12],
            opposite_vertex: 2,
        },
        SolidFace {
            vertex_indices: vec![2, 10, 6, 15, 13],
            opposite_vertex: 5,
        },
        SolidFace {
            vertex_indices: vec![2, 13, 3, 17, 16],
            opposite_vertex: 4,
        },
        SolidFace {
            vertex_indices: vec![4, 14, 5, 19, 18],
            opposite_vertex: 3,
        },
        SolidFace {
            vertex_indices: vec![4, 18, 6, 10, 8],
            opposite_vertex: 1,
        },
        SolidFace {
            vertex_indices: vec![5, 9, 11, 7, 19],
            opposite_vertex: 0,
        },
        SolidFace {
            vertex_indices: vec![3, 13, 15, 7, 11],
            opposite_vertex: 0,
        },
        SolidFace {
            vertex_indices: vec![6, 18, 19, 7, 15],
            opposite_vertex: 0,
        },
    ];

    (vertices, faces)
}

/// Procedural marble texture.
fn generate_marble_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let warmth = (rand::random::<f32>() - 0.5) * 0.1;
    let base = hue_to_rgb(rand::random::<f32>() * 6.0, 0.15, 0.82 + warmth);
    let seed = rand::random::<u32>();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let vein = fbm_2d_periodic(u * 4.0, v * 4.0, 5, 0.6, 2.0, seed, Some(4));
            let detail = fbm_2d_periodic(u * 12.0, v * 12.0, 3, 0.4, 2.0, seed + 3, Some(12));
            let factor = 0.75 + vein * 0.18 + detail * 0.07;
            let c = base.scale(factor);

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
