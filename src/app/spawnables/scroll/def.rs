//! The scroll's level-file definition and what it spawns.

use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::figure::Proposition;
use super::mesh::{figure_mesh, knobs_mesh, recto_mesh, verso_mesh, Face};
use super::texture::{recto_texture, verso_texture, RECTO_HEIGHT, RECTO_WIDTH, VERSO_SIZE};
use crate::app::spawnables::shared::gilt::{gilt, GILT_FINISH};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::components::{MaterialModulation, ModelInstance, Orientation, Position, Renderable};
use crate::core::error::EngineResult;
use crate::creature::Collectable;
use crate::lighting::PointLight;
use crate::model::{Model, ModelPart, Transform};
use crate::objective::{Hover, LevelProgress};
use crate::particles::{Palette, ParticleEffectType, ParticleEmitter};
use crate::rendering::colour::Colour;
use crate::rendering::material::{
    Emission, Material, MaterialId, SurfaceFinish, SurfaceModulation,
};
use crate::rendering::reflection::Reflects;
use crate::resources::textures::TextureManager;

/// The name the scrolls' materials are shared under in a level: every
/// scroll is the same papyrus and the same gold.
const PALETTE: &str = "euclid_scroll";

/// Slots in the palette.
const RECTO: usize = 0;
const VERSO: usize = 1;
const GILT: usize = 2;
const FIGURE: usize = 3;
const FIGURE_THROUGH: usize = 4;

/// The papyrus is the same on every scroll, as one book's pages are.
const PAPYRUS_SEED: u32 = 0x0e0c_1d00;

/// Papyrus: dry and fibrous, a dull sheen at a grazing angle.
const PAPYRUS_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.82,
    metallic: 0.0,
};

/// The gold's light caught in the papyrus, faint and warm, a little more
/// at its edges: enough that a sheet turned from the sun stays papyrus and
/// does not go grey.
const PAPYRUS_GLOW: Emission = Emission {
    colour: Colour::rgb(1.0, 0.8, 0.55),
    strength: 0.04,
    rim_strength: 0.2,
    rim_power: 3.0,
};

/// The figure is in gold ink, as an illuminated book's are, and it shines:
/// what tells a scroll from across a level, and what is taken. Its glow,
/// as luminance, over the bloom threshold so its lines bloom, and brighter
/// at its edges.
const FIGURE_GLOW: Colour = Colour::rgb(1.0, 0.68, 0.3);
const FIGURE_GLOW_STRENGTH: f32 = 1.35;
const FIGURE_RIM_STRENGTH: f32 = 1.0;
const FIGURE_RIM_POWER: f32 = 2.0;

/// The figure seen through the verso: the same light, dimmed by the sheet.
const THROUGH_GLOW_STRENGTH: f32 = 0.55;

/// A slower turn than a solid's, so the figure is seen for a while each
/// time it comes round.
const SPIN_RATE: f32 = 0.16;

/// How close the player has to get, m: wider than the scroll itself.
const REACH: f32 = 1.2;

/// It lights its surroundings a little, warm as the gold, so one in a
/// corner is findable; less than a solid, which it must not be taken for.
const LIGHT: Colour = Colour::rgb(1.0, 0.8, 0.52);
const LIGHT_INTENSITY: f32 = 1.0;
const LIGHT_RANGE: f32 = 4.5;

/// Motes of gold light about it each second: fewer than a solid's.
const MOTES_PER_SECOND: f32 = 9.0;
const MOTE_COLOUR: Vector3<f32> = Vector3::new(1.0, 0.82, 0.5);

/// A page of Euclid's *Elements*: a papyrus roll held open on its figure.
/// It hangs where it is authored, turning, and is taken by walking into
/// it. Taking one is never needed to finish a level.
#[derive(Deserialize)]
pub struct ScrollDef {
    /// Where the middle of its open sheet hangs.
    pub pos: (f32, f32, f32),

    /// Whose figure it carries.
    pub proposition: Proposition,
}

impl ScrollDef {
    /// The scrolls' materials, one to a slot: the recto, the verso and
    /// rolls, the knobs' gilt, the gold figure and its light through the
    /// verso.
    pub fn materials(textures: &TextureManager) -> EngineResult<Vec<Material>> {
        let recto = textures.create_from_rgba(
            RECTO_WIDTH,
            RECTO_HEIGHT,
            &recto_texture(PAPYRUS_SEED),
            true,
        )?;
        let verso = textures.create_from_rgba(
            VERSO_SIZE,
            VERSO_SIZE,
            &verso_texture(PAPYRUS_SEED),
            true,
        )?;
        let papyrus = |texture| {
            Material::textured(texture)
                .with_finish(PAPYRUS_FINISH)
                .with_emission(PAPYRUS_GLOW)
        };
        Ok(vec![
            papyrus(recto),
            papyrus(verso),
            gilt(),
            Material::coloured(Colour::WHITE)
                .with_finish(GILT_FINISH)
                .with_reflections(Reflects::Surroundings)
                .with_emission(Emission {
                    colour: FIGURE_GLOW,
                    strength: FIGURE_GLOW_STRENGTH,
                    rim_strength: FIGURE_RIM_STRENGTH,
                    rim_power: FIGURE_RIM_POWER,
                }),
            Material::coloured(Colour::WHITE)
                .with_finish(PAPYRUS_FINISH)
                .with_emission(Emission::glow(FIGURE_GLOW, THROUGH_GLOW_STRENGTH)),
        ])
    }

    /// The model of a scroll open on `proposition`, about the middle of its
    /// sheet, in the palette whose ids are `ids`.
    pub fn model(proposition: Proposition, ids: &[MaterialId]) -> Model {
        let figure = proposition.figure();
        Model::flat(vec![ModelPart {
            local_transform: Transform::default(),
            primitives: vec![
                recto_mesh(ids[RECTO]),
                verso_mesh(ids[VERSO]),
                knobs_mesh(ids[GILT]),
                figure_mesh(&figure, Face::Recto, ids[FIGURE]),
                figure_mesh(&figure, Face::Verso, ids[FIGURE_THROUGH]),
            ],
        }])
    }

    /// Phase offset, from the position, so two scrolls do not bob and turn
    /// as one, and a level always looks the same.
    fn phase(&self) -> f32 {
        let (x, y, z) = self.pos;
        (x * 0.41 + y * 0.53 + z * 0.29).rem_euclid(10.0)
    }
}

impl Spawnable for ScrollDef {
    fn material_count(&self) -> usize {
        5
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        ctx.try_palette(PALETTE, Self::materials)
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let anchor = Vector3::new(self.pos.0, self.pos.1, self.pos.2);
        let model = Arc::new(Self::model(self.proposition, materials));

        world
            .entry::<LevelProgress>()
            .or_insert_with(LevelProgress::default)
            .add_scroll();

        let hover = Hover {
            spin_rate: SPIN_RATE,
            ..Hover::new(anchor, UnitQuaternion::identity(), self.phase())
        };
        let (position, orientation) = hover.pose();

        vec![world
            .create_entity()
            .with(Position(position))
            .with(Orientation(orientation))
            .with(hover)
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(MaterialModulation(SurfaceModulation::emissive(
                hover.glow(),
            )))
            .with(Collectable::scroll(REACH))
            .with(PointLight::new(LIGHT, LIGHT_INTENSITY, LIGHT_RANGE))
            .with(
                ParticleEmitter::new(ParticleEffectType::Motes)
                    .with_spawn_rate(MOTES_PER_SECOND)
                    .with_palette(Palette::single(MOTE_COLOUR)),
            )
            .build()]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::spawnables::scroll::mesh::{SHEET_HEIGHT, SHEET_WIDTH};

    /// Every figure's model is whole, and none reaches past the player's
    /// reach of its middle.
    #[test]
    fn every_model_is_whole_and_within_reach() {
        let ids: Vec<MaterialId> = (0..5).map(MaterialId).collect();
        for proposition in Proposition::ALL {
            let model = ScrollDef::model(proposition, &ids);
            for mesh in model.parts.iter().flat_map(|part| &part.primitives) {
                assert!(!mesh.indices.is_empty(), "{proposition:?}");
                let count = mesh.vertices.len() as u32;
                assert!(mesh.indices.iter().all(|&i| i < count), "{proposition:?}");
                for vertex in &mesh.vertices {
                    assert!(vertex.pos.norm() < REACH * 0.5, "{proposition:?}");
                    assert!(vertex.pos.x.abs() < SHEET_WIDTH, "{proposition:?}");
                    assert!(vertex.pos.y.abs() < SHEET_HEIGHT, "{proposition:?}");
                }
            }
        }
    }
}
