//! What the creatures that are one sculpted model share: the material they
//! are painted on, how they perceive, and the ground they are stood on.

use std::sync::Arc;

use specs::{World, WorldExt};

use crate::model::{Model, Transform};

use crate::creature::Perception;
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, SurfaceFinish};
use crate::terrain::TerrainWorld;

/// A creature that is one model, set out in one of its states for a tool
/// to draw.
pub struct Exhibit {
    /// What the state is called.
    pub name: &'static str,
    /// The model, against the palette's slots.
    pub model: Arc<Model>,
    /// Each part's pose in this state, in part order.
    pub poses: Vec<Transform>,
    /// How high its origin is held off the ground, m.
    pub lift: f32,
    /// How big it is across, m: what a camera frames.
    pub reach: f32,
}

impl Exhibit {
    /// `model` at rest.
    pub fn at_rest(name: &'static str, model: Arc<Model>, lift: f32, reach: f32) -> Self {
        Self {
            name,
            poses: vec![Transform::default(); model.parts.len()],
            model,
            lift,
            reach,
        }
    }
}

/// Paint: matte, with a breath of sheen.
const PAINT: SurfaceFinish = SurfaceFinish {
    roughness: 0.7,
    metallic: 0.0,
};

/// What a sculpted creature is drawn in: white, its colours riding on its
/// vertices.
pub fn sculpt_material() -> Material {
    Material::coloured(Colour::WHITE).with_finish(PAINT)
}

/// Perception for a creature with no front: it notices anything within
/// `range`, whichever way that lies, and forgets it quickly.
pub fn all_round(range: f32) -> Perception {
    Perception {
        sight_range: range,
        sight_cone_cos: -1.0,
        hearing_range: range,
        memory_duration: 1.5,
        target: None,
    }
}

/// The height of the ground at `at` (x, z), or `None` where there is none:
/// the level moved, and the creature's spot went with it.
pub fn ground_under(world: &World, at: (f32, f32)) -> Option<f32> {
    world
        .read_resource::<TerrainWorld>()
        .mesh_surface_height_at(at.0, at.1)
}

#[cfg(test)]
mod tests {
    use crate::app::creatures::{FlyingFishDef, MusselDef, PiperDef, RollerDef};
    use crate::rendering::material::MaterialId;

    /// A model that panics while it is sculpted, or comes out empty or with
    /// a part flung off to nowhere, would only show when its creature was
    /// first met in a level.
    #[test]
    fn every_sculpted_creature_is_whole_and_no_bigger_than_it_should_be() {
        let paint = MaterialId(0);
        let models = [
            ("roller", RollerDef::model(0.6, paint), 0.8),
            ("piper", PiperDef::model(paint), 1.6),
            ("flying fish", FlyingFishDef::model(paint), 1.2),
            ("open mussel", MusselDef::model(paint, 1.0), 1.6),
            ("shut mussel", MusselDef::model(paint, 0.0), 1.2),
        ];
        for (name, model, reach) in models {
            let meshes: Vec<_> = model
                .parts
                .iter()
                .flat_map(|part| &part.primitives)
                .collect();
            assert!(!meshes.is_empty(), "the {name} has nothing to draw");
            for mesh in meshes {
                let count = mesh.vertices.len() as u32;
                assert!(mesh.indices.iter().all(|index| *index < count));
                for vertex in &mesh.vertices {
                    assert!(vertex.pos.iter().all(|x| x.is_finite()));
                    assert!(
                        vertex.pos.norm() < reach,
                        "part of the {name} is {} m from its middle",
                        vertex.pos.norm()
                    );
                }
            }
        }
    }
}
