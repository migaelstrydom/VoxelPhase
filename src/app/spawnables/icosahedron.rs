//! Icosahedron spawnable — regular icosahedron with ConvexHull collider.

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
pub struct IcosahedronDef {
    pub pos: (f32, f32, f32),
    /// Edge length.
    #[serde(default = "IcosahedronDef::default_size")]
    pub size: f32,
    #[serde(default = "IcosahedronDef::default_density")]
    pub density: f32,
    #[serde(default = "IcosahedronDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "IcosahedronDef::default_friction")]
    pub friction: f32,
}

impl IcosahedronDef {
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

impl Spawnable for IcosahedronDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let pixels = generate_jade_texture();
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        let material = Material::textured(texture).with_derived_finish(self.surface());
        Ok(vec![ctx.materials.register(material)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let material = materials[0];

        let (vertices, faces) = icosahedron_geometry(self.size);
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

/// Vertices and faces of a regular icosahedron with the given edge length.
///
/// 12 vertices, 20 triangular faces. Coordinates derived from the golden ratio.
fn icosahedron_geometry(edge: f32) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let phi: f32 = (1.0 + 5.0f32.sqrt()) / 2.0;

    // Raw vertices at circumradius sqrt(1 + φ²) ≈ 1.902.
    // Three orthogonal golden rectangles: (0, ±1, ±φ), (±1, ±φ, 0), (±φ, 0, ±1).
    let raw = [
        Vector3::new(0.0, 1.0, phi),   // 0
        Vector3::new(0.0, 1.0, -phi),  // 1
        Vector3::new(0.0, -1.0, phi),  // 2
        Vector3::new(0.0, -1.0, -phi), // 3
        Vector3::new(1.0, phi, 0.0),   // 4
        Vector3::new(1.0, -phi, 0.0),  // 5
        Vector3::new(-1.0, phi, 0.0),  // 6
        Vector3::new(-1.0, -phi, 0.0), // 7
        Vector3::new(phi, 0.0, 1.0),   // 8
        Vector3::new(phi, 0.0, -1.0),  // 9
        Vector3::new(-phi, 0.0, 1.0),  // 10
        Vector3::new(-phi, 0.0, -1.0), // 11
    ];

    // Scale so edge length = requested size. Raw edge length = 2.
    let scale = edge / 2.0;
    let vertices: Vec<Vector3<f32>> = raw.iter().map(|v| v * scale).collect();

    // 20 triangular faces. For opposite_vertex, pick any vertex not on the face
    // that is roughly opposite — the centroid (origin) works as a reference,
    // so any vertex across the origin from the face triangle suffices.
    let faces = vec![
        // Top cap (around vertex 0)
        SolidFace {
            vertex_indices: vec![0, 2, 8],
            opposite_vertex: 3,
        },
        SolidFace {
            vertex_indices: vec![0, 8, 4],
            opposite_vertex: 7,
        },
        SolidFace {
            vertex_indices: vec![0, 4, 6],
            opposite_vertex: 5,
        },
        SolidFace {
            vertex_indices: vec![0, 6, 10],
            opposite_vertex: 9,
        },
        SolidFace {
            vertex_indices: vec![0, 10, 2],
            opposite_vertex: 3,
        },
        // Middle band
        SolidFace {
            vertex_indices: vec![2, 5, 8],
            opposite_vertex: 6,
        },
        SolidFace {
            vertex_indices: vec![8, 5, 9],
            opposite_vertex: 10,
        },
        SolidFace {
            vertex_indices: vec![8, 9, 4],
            opposite_vertex: 7,
        },
        SolidFace {
            vertex_indices: vec![4, 9, 1],
            opposite_vertex: 2,
        },
        SolidFace {
            vertex_indices: vec![4, 1, 6],
            opposite_vertex: 5,
        },
        SolidFace {
            vertex_indices: vec![6, 1, 11],
            opposite_vertex: 8,
        },
        SolidFace {
            vertex_indices: vec![6, 11, 10],
            opposite_vertex: 9,
        },
        SolidFace {
            vertex_indices: vec![10, 11, 7],
            opposite_vertex: 4,
        },
        SolidFace {
            vertex_indices: vec![10, 7, 2],
            opposite_vertex: 1,
        },
        SolidFace {
            vertex_indices: vec![2, 7, 5],
            opposite_vertex: 6,
        },
        // Bottom cap (around vertex 3)
        SolidFace {
            vertex_indices: vec![3, 9, 5],
            opposite_vertex: 10,
        },
        SolidFace {
            vertex_indices: vec![3, 1, 9],
            opposite_vertex: 2,
        },
        SolidFace {
            vertex_indices: vec![3, 11, 1],
            opposite_vertex: 8,
        },
        SolidFace {
            vertex_indices: vec![3, 7, 11],
            opposite_vertex: 4,
        },
        SolidFace {
            vertex_indices: vec![3, 5, 7],
            opposite_vertex: 6,
        },
    ];

    (vertices, faces)
}

/// Procedural jade/stone texture.
fn generate_jade_texture() -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let hue = 1.5 + (rand::random::<f32>() - 0.5) * 1.0; // green-ish range
    let base = hue_to_rgb(hue, 0.35, 0.65);
    let seed = rand::random::<u32>();

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let swirl =
                fbm_2d_periodic(u * 3.0 + v * 2.0, v * 5.0 - u, 5, 0.55, 2.0, seed, Some(5));
            let fleck = fbm_2d_periodic(u * 20.0, v * 20.0, 2, 0.3, 2.0, seed + 11, Some(20));
            let factor = 0.72 + swirl * 0.22 + fleck * 0.06;
            let c = base.scale(factor);

            c.write_rgba(&mut pixels);
        }
    }
    pixels
}
