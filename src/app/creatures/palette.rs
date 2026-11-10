//! The materials every creature is painted in.
//!
//! ```text
//!   Paint::Skin ─┐
//!   Paint::Wet ──┼─ id() ──▶ the slot a sculpted model names
//!   …            │
//!   materials() ─┴─▶ one Material to a slot ──ctx.palette──▶ the level's ids
//! ```
//!
//! A creature is sculpted once, against the slots, and its colours ride on
//! its vertices; what a slot gives is the *finish*: how wet, how rough, how
//! much a mirror. One palette for the whole bestiary keeps the creatures
//! of a kind and of a piece, and lets every creature of one kind in a level
//! share one model.

use crate::rendering::colour::Colour;
use crate::rendering::grain::GrainSpec;
use crate::rendering::material::{Emission, Material, MaterialId, SurfaceFinish};
use crate::rendering::reflection::Reflects;

/// The name the bestiary's materials are shared under in a level.
pub const BESTIARY: &str = "bestiary";

/// What a part of a creature is made of.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Paint {
    /// Living skin: soft, with a breath of sheen and pores.
    Skin,
    /// Anything wet: an eye, the inside of a mouth, raw flesh, slime.
    Wet,
    /// Horn: beaks, claws, nails, teeth, thorns.
    Horn,
    /// Eggshell and bone: chalky, faintly pitted.
    Shell,
    /// Woven stuff: robes and rags.
    Cloth,
    /// Feathers and fur.
    Feather,
    /// Turned, oiled wood.
    Wood,
    /// Cast bronze, bright where it is rubbed.
    Bronze,
    /// Beaten tin: a duller mirror.
    Tin,
    /// Mother-of-pearl.
    Nacre,
    /// A husk or a rind: dry, leathery.
    Husk,
    /// Something that gives its own light: an eye in the dark.
    Glow,
}

impl Paint {
    /// Every paint, in slot order.
    pub const ALL: [Paint; 12] = [
        Paint::Skin,
        Paint::Wet,
        Paint::Horn,
        Paint::Shell,
        Paint::Cloth,
        Paint::Feather,
        Paint::Wood,
        Paint::Bronze,
        Paint::Tin,
        Paint::Nacre,
        Paint::Husk,
        Paint::Glow,
    ];

    /// The slot a model sculpted in this paint names.
    pub fn id(self) -> MaterialId {
        MaterialId(self as u32)
    }

    fn finish(self) -> SurfaceFinish {
        let (roughness, metallic) = match self {
            Paint::Skin => (0.5, 0.0),
            Paint::Wet => (0.08, 0.0),
            Paint::Horn => (0.28, 0.0),
            Paint::Shell => (0.42, 0.0),
            Paint::Cloth => (0.95, 0.0),
            Paint::Feather => (0.6, 0.0),
            Paint::Wood => (0.45, 0.0),
            Paint::Bronze => (0.26, 1.0),
            Paint::Tin => (0.42, 1.0),
            Paint::Nacre => (0.16, 0.25),
            Paint::Husk => (0.8, 0.0),
            Paint::Glow => (0.3, 0.0),
        };
        SurfaceFinish {
            roughness,
            metallic,
        }
    }

    /// The fine structure the finish breaks its highlight on.
    fn grain(self) -> GrainSpec {
        match self {
            Paint::Skin => GrainSpec::STONE.with_scale(1.6).with_strength(0.16),
            Paint::Shell => GrainSpec::STONE.with_scale(2.2).with_strength(0.1),
            Paint::Cloth => GrainSpec::WOOD.with_scale(2.0).with_strength(0.3),
            Paint::Feather => GrainSpec::WOOD.with_scale(1.4).with_strength(0.28),
            Paint::Wood => GrainSpec::WOOD.with_strength(0.2),
            Paint::Bronze => GrainSpec::DRESSED_STONE.with_strength(0.12),
            Paint::Tin => GrainSpec::DRESSED_STONE.with_strength(0.22),
            Paint::Husk => GrainSpec::WEATHERED_STONE.with_strength(0.28),
            Paint::Horn | Paint::Wet | Paint::Nacre | Paint::Glow => GrainSpec::NONE,
        }
    }

    fn material(self) -> Material {
        let material = Material::coloured(Colour::WHITE)
            .with_finish(self.finish())
            .with_grain(self.grain());
        match self {
            // A metal shows no colour of its own but what it reflects.
            Paint::Bronze | Paint::Tin | Paint::Nacre => {
                material.with_reflections(Reflects::Surroundings)
            }
            Paint::Glow => material.with_emission(Emission {
                colour: Colour::new(1.0, 0.75, 0.3, 1.0),
                strength: 2.5,
                ..Emission::default()
            }),
            _ => material,
        }
    }
}

/// The bestiary's materials, one to a slot.
pub fn materials() -> Vec<Material> {
    Paint::ALL.iter().map(|paint| paint.material()).collect()
}

/// For a model sculpted against the slots: the level's material for each
/// slot, given the ids the level registered [`materials`] under.
pub fn in_level(ids: &[MaterialId]) -> impl Fn(MaterialId) -> MaterialId + '_ {
    |slot| ids[slot.0 as usize]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_paint_has_its_own_slot_and_a_material_in_it() {
        for (slot, paint) in Paint::ALL.iter().enumerate() {
            assert_eq!(paint.id(), MaterialId(slot as u32));
        }
        assert_eq!(materials().len(), Paint::ALL.len());
    }
}
