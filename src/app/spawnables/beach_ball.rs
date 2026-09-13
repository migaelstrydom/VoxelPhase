//! Beach ball spawnable — bouncy sphere with randomised colour spiral.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::geometry::{generate_magic_sphere_vertices, generate_sphere_indices, MagicSphereConfig};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::systems::PhysicsResource;

use super::shared::finish::{ColliderSurface, MaterialSurface};
use super::shared::textures::seed_from_position;
use super::shared::textures::TextureRng;

/// The ball's one dimension. Public because a beach ball is a stack item, and
/// working out how much ground a stack covers means knowing how wide its
/// widest item is.
pub const RADIUS: f32 = 0.5;
const SEGMENTS: u32 = 32;
const RINGS: u32 = 24;

/// Light and lively. Declared once so the collider and the material cannot
/// disagree about how bouncy the ball is.
const SURFACE: PhysicalSurface = PhysicalSurface {
    restitution: 0.6,
    friction: 0.5,
    density: 100.0,
};

#[derive(Deserialize)]
pub struct BeachBallDef {
    pub pos: (f32, f32, f32),
}

impl Spawnable for BeachBallDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let texture = ctx
            .textures
            .create_solid_colour(crate::rendering::colour::Colour::new(1.0, 1.0, 1.0, 1.0))?;
        let material = Material::textured(texture).with_derived_finish(SURFACE);
        Ok(vec![ctx.materials.register(material)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let material = materials[0];

        let rng = &mut TextureRng::new(seed_from_position(self.pos, 0));
        let config = MagicSphereConfig {
            base_hue: rng.unit(),
            spiral_frequency: rng.range(2.0, 5.0),
            spiral_tightness: rng.range(1.5, 4.0),
            accent_hue_offset: rng.range(0.2, 0.45),
            color_variation: rng.range(0.1, 0.35),
            glow_intensity: rng.range(0.85, 1.15),
        };

        let parts = vec![ModelPart::new(vec![MeshPrimitive {
            vertices: generate_magic_sphere_vertices(RADIUS, SEGMENTS, RINGS, &config),
            indices: generate_sphere_indices(SEGMENTS, RINGS),
            material,
        }])];

        let model = Arc::new(Model::flat(parts));

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_desc = RigidBodyDesc::dynamic()
                .position(initial_pos)
                .gravity_scale(1.0)
                .linear_damping(0.5)
                .angular_damping(0.002);

            let body_handle = physics.world.create_body(body_desc);

            let collider_desc = ColliderDesc::sphere(RADIUS).with_physical_surface(SURFACE);

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
