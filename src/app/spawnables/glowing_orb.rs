//! Glowing orb spawnable — a polished, self-illuminated sphere.
//!
//! Physically identical to [`super::beach_ball`]; it exists purely as a subject
//! for lighting and material work (see docs/LIGHTING_PLAN.md). Keep the physics
//! parameters below in sync with the beach ball so that any difference on
//! screen is attributable to shading alone.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Emission, Material, MaterialId, SurfaceFinish};
use crate::systems::PhysicsResource;

const RADIUS: f32 = 0.5;
const SEGMENTS: u32 = 48;
const RINGS: u32 = 32;

/// Default orb colour: a cool arcane cyan.
const DEFAULT_COLOUR: Colour = Colour {
    r: 0.35,
    g: 0.75,
    b: 1.0,
    a: 1.0,
};

/// Base emissive strength. Deliberately above 1.0: the scene renders to an HDR
/// target, so this is the headroom the bloom bright-pass keys off. Below the
/// post-processing bloom threshold the orb is merely bright, not luminous.
const DEFAULT_GLOW: f32 = 2.5;

/// Silhouette brightening. The dominant "magical volume" cue at this stage.
const RIM_STRENGTH: f32 = 1.6;
const RIM_POWER: f32 = 2.5;

#[derive(Deserialize)]
pub struct GlowingOrbDef {
    pub pos: (f32, f32, f32),

    /// Orb colour, tinting both the surface and its glow. Defaults to cyan.
    #[serde(default)]
    pub colour: Option<(f32, f32, f32)>,

    /// Emissive strength multiplier. Defaults to `DEFAULT_GLOW`.
    #[serde(default)]
    pub glow: Option<f32>,
}

impl GlowingOrbDef {
    fn colour(&self) -> Colour {
        match self.colour {
            Some((r, g, b)) => Colour::new(r, g, b, 1.0),
            None => DEFAULT_COLOUR,
        }
    }

    fn glow(&self) -> f32 {
        self.glow.unwrap_or(DEFAULT_GLOW)
    }
}

impl Spawnable for GlowingOrbDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let texture = ctx.textures.create_solid_colour(Colour::WHITE)?;

        let material = Material::textured(texture)
            .with_finish(SurfaceFinish::POLISHED)
            .with_emission(Emission {
                colour: self.colour(),
                strength: self.glow(),
                rim_strength: RIM_STRENGTH,
                rim_power: RIM_POWER,
            });

        Ok(vec![ctx.materials.register(material)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);

        // The lit surface is dark relative to the glow, so the emissive term and
        // the specular highlight dominate rather than fighting a bright albedo.
        let base = self.colour();
        let surface_colour = Colour::new(base.r * 0.25, base.g * 0.25, base.b * 0.25, 1.0);

        let parts = vec![ModelPart::new(vec![MeshPrimitive {
            vertices: generate_sphere_vertices(RADIUS, SEGMENTS, RINGS, surface_colour),
            indices: generate_sphere_indices(SEGMENTS, RINGS),
            material: materials[0],
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

            let collider_desc = ColliderDesc::sphere(RADIUS)
                .density(100.0)
                .restitution(0.6)
                .friction(0.5);

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
