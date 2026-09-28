//! Dodecahedron spawnable — regular dodecahedron with ConvexHull collider.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::models::{convex_solid_model, SolidFace};
use super::shared::textures::hue_to_rgb;
use super::shared::textures::seed_from_position;
use super::shared::textures::TextureRng;
use super::{MaterialCtx, Spawnable};
use crate::collision::convex_hull::{dodecahedron_hull, ConvexHull};
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
        let pixels = generate_marble_texture(seed_from_position(self.pos, 0));
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        let material = Material::textured(texture).with_derived_finish(self.surface());
        Ok(vec![ctx.materials.register(material)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let material = materials[0];

        let hull = Arc::new(dodecahedron_hull(self.size));
        let model = convex_solid_model(&hull.vertices, &solid_faces(&hull), material);

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

/// The faces of `hull` as the model builder takes them.
fn solid_faces(hull: &ConvexHull) -> Vec<SolidFace> {
    hull.faces
        .iter()
        .map(|face| SolidFace {
            vertex_indices: face.vertex_indices.iter().map(|&i| i as usize).collect(),
            opposite_vertex: (0..hull.vertices.len())
                .min_by(|&a, &b| {
                    let along = |i: usize| hull.vertices[i].dot(&face.normal);
                    along(a).total_cmp(&along(b))
                })
                .expect("a hull has vertices"),
        })
        .collect()
}

/// Procedural marble texture.
fn generate_marble_texture(seed: u32) -> Vec<u8> {
    let rng = &mut TextureRng::new(seed);
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let warmth = (rng.unit() - 0.5) * 0.1;
    let base = hue_to_rgb(rng.unit() * 6.0, 0.15, 0.82 + warmth);
    let seed = rng.u32();

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
