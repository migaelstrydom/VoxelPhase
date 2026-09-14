//! Gem spawnable — the thing a level asks you to collect.
//!
//! An octahedron with no rigid body. It hangs where it was authored, hovers,
//! turns, and is caught by walking into it; nothing can knock it down a slope
//! or into a lake, which is the whole reason it has no physics at all.

use nalgebra::Vector3;
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{convex_solid_model, SolidFace};
use super::{MaterialCtx, Spawnable};
use crate::components::{MaterialModulation, ModelInstance, Orientation, Position, Renderable};
use crate::core::error::EngineResult;
use crate::creature::Collectable;
use crate::lighting::PointLight;
use crate::objective::{GemMotion, LevelProgress};
use crate::rendering::colour::Colour;
use crate::rendering::material::{
    Emission, Material, MaterialId, SurfaceFinish, SurfaceModulation,
};

/// Half-height of the gem, from centre to point.
const HALF_EXTENT: f32 = 0.35;

/// How close the player has to get. Comfortably wider than the gem itself:
/// brushing past one and not collecting it reads as a bug.
const REACH: f32 = 1.2;

/// Warm gold. Reads as treasure against terrain and foliage alike.
const DEFAULT_COLOUR: Colour = Colour {
    r: 1.0,
    g: 0.78,
    b: 0.25,
    a: 1.0,
};

/// Emitted luminance, above the bloom threshold so the gem carries a halo and
/// can be picked out at distance. Kept close to the threshold: further up, the
/// facets clip to white and the gem loses the colour that identifies it.
const GLOW: f32 = 1.5;

/// How much of the gem's colour survives into its albedo. The glow washes out
/// a bright surface, so the body is deliberately dark and the hue is carried
/// by the emission and the highlight.
const BODY_TINT: f32 = 0.45;

/// Silhouette brightening, matching the glowing orb's convention.
const RIM_STRENGTH: f32 = 0.5;
const RIM_POWER: f32 = 2.5;

/// The gem lights its own immediate surroundings, which is what makes one
/// tucked into a corner findable.
const LIGHT_INTENSITY: f32 = 0.6;
const LIGHT_RANGE: f32 = 5.0;

#[derive(Deserialize)]
pub struct GemDef {
    pub pos: (f32, f32, f32),

    /// Body and glow tint. Defaults to warm gold.
    #[serde(default)]
    pub colour: Option<(f32, f32, f32)>,
}

impl GemDef {
    fn colour(&self) -> Colour {
        match self.colour {
            Some((r, g, b)) => Colour::new(r, g, b, 1.0),
            None => DEFAULT_COLOUR,
        }
    }

    /// Phase offset, so that two gems authored at different places do not bob
    /// in lockstep. Derived from the position rather than a counter: the same
    /// level always looks the same.
    fn phase(&self) -> f32 {
        let (x, y, z) = self.pos;
        (x * 0.37 + y * 0.61 + z * 0.23).rem_euclid(3.0)
    }
}

impl Spawnable for GemDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let base = self.colour();
        let body = Colour::new(
            base.r * BODY_TINT,
            base.g * BODY_TINT,
            base.b * BODY_TINT,
            1.0,
        );
        let texture = ctx.textures.create_solid_colour(body)?;

        let material = Material::textured(texture)
            .with_finish(SurfaceFinish::POLISHED)
            .with_emission(Emission {
                colour: base,
                strength: GLOW,
                rim_strength: RIM_STRENGTH,
                rim_power: RIM_POWER,
            });

        Ok(vec![ctx.materials.register(material)])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let anchor = Vector3::new(self.pos.0, self.pos.1, self.pos.2);

        let base = self.colour();

        let (vertices, faces) = gem_geometry();
        let model = convex_solid_model(&vertices, &faces, materials[0]);

        world
            .entry::<LevelProgress>()
            .or_insert_with(LevelProgress::default)
            .add_gem();

        let motion = GemMotion::new(anchor, self.phase());
        let (position, orientation) = motion.pose();

        vec![world
            .create_entity()
            .with(Position(position))
            .with(Orientation(orientation))
            .with(motion)
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(MaterialModulation(SurfaceModulation::IDENTITY))
            .with(Collectable::gem(REACH))
            .with(PointLight::new(base, LIGHT_INTENSITY, LIGHT_RANGE))
            .build()]
    }
}

/// Vertices and faces of the gem: an octahedron stretched along `+Y`, so it
/// reads as a cut stone rather than as a die.
fn gem_geometry() -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let waist = HALF_EXTENT * 0.62;

    let vertices = vec![
        Vector3::new(0.0, HALF_EXTENT, 0.0),  // 0: top point
        Vector3::new(0.0, -HALF_EXTENT, 0.0), // 1: bottom point
        Vector3::new(waist, 0.0, 0.0),        // 2: +X
        Vector3::new(-waist, 0.0, 0.0),       // 3: -X
        Vector3::new(0.0, 0.0, waist),        // 4: +Z
        Vector3::new(0.0, 0.0, -waist),       // 5: -Z
    ];

    let faces = vec![
        SolidFace {
            vertex_indices: vec![0, 4, 2],
            opposite_vertex: 1,
        },
        SolidFace {
            vertex_indices: vec![0, 2, 5],
            opposite_vertex: 1,
        },
        SolidFace {
            vertex_indices: vec![0, 5, 3],
            opposite_vertex: 1,
        },
        SolidFace {
            vertex_indices: vec![0, 3, 4],
            opposite_vertex: 1,
        },
        SolidFace {
            vertex_indices: vec![1, 2, 4],
            opposite_vertex: 0,
        },
        SolidFace {
            vertex_indices: vec![1, 5, 2],
            opposite_vertex: 0,
        },
        SolidFace {
            vertex_indices: vec![1, 3, 5],
            opposite_vertex: 0,
        },
        SolidFace {
            vertex_indices: vec![1, 4, 3],
            opposite_vertex: 0,
        },
    ];

    (vertices, faces)
}
