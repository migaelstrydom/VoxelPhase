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
use crate::lighting::PointLight;
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

/// Base emissive luminance. Deliberately above the bloom threshold (1.3): the
/// scene renders to an HDR target, so this is the headroom the bright pass keys
/// off. Below the threshold the orb is merely bright, not luminous.
///
/// Because `Emission::strength` is luminance rather than a colour multiplier,
/// this number means the same brightness whatever `colour` is set to.
const DEFAULT_GLOW: f32 = 1.71;

/// Silhouette brightening. The dominant "magical volume" cue at this stage.
///
/// Re-derived now that the rim term reads normalised emissive colour
/// (luminance-consistent across hues) instead of raw emissive.rgb: per-orb
/// preservation of the old look would need 0.64 / 0.40 / 0.43 across the
/// three orb colours in use, and no single value preserves all three — this
/// change cannot preserve appearance, since the old per-hue inconsistency was
/// the defect being fixed. 0.5 is the middle of that range; needs a human eye
/// afterwards.
const RIM_STRENGTH: f32 = 0.5;
const RIM_POWER: f32 = 2.5;

/// How far the orb's light reaches. Beyond this its contribution is exactly
/// zero, and the collector stops considering it relevant.
const LIGHT_RANGE: f32 = 8.0;

/// Brightness of the light the orb casts, as luminance. Larger than
/// `sun_intensity` because inverse-square falloff has already more than
/// halved it a metre out; the sun has no such divisor.
///
/// Re-derived for the luminance convention from the old multiplier (1.5): the
/// three orb colours in use have luminance 0.683 / 0.545 / 0.457, so
/// preserving each individually would need 1.025 / 0.818 / 0.686. No single
/// value preserves all three — that inconsistency was the bug — so this is
/// the middle of that range, 0.85, as a starting point pending a look.
const LIGHT_INTENSITY: f32 = 0.85;

#[derive(Deserialize)]
pub struct GlowingOrbDef {
    pub pos: (f32, f32, f32),

    /// Orb colour, tinting both the surface and its glow. Defaults to cyan.
    #[serde(default)]
    pub colour: Option<(f32, f32, f32)>,

    /// Emitted luminance, independent of hue — above the bloom threshold the
    /// orb glows, below it merely looks bright. Defaults to `DEFAULT_GLOW`.
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
            // The light sits at the orb's centre, with no offset: for a sphere
            // that is where the glow physically originates, and it means the
            // orb cannot light its own surface (the light direction is the
            // exact opposite of every surface normal, so n·l is zero).
            .with(PointLight::new(base, LIGHT_INTENSITY, LIGHT_RANGE))
            .build()]
    }
}
