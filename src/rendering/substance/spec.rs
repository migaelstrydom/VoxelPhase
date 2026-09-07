//! What a thing is made of, declared once.
//!
//! A spawnable builds its collider in `spawn` and its material in
//! `create_materials` — different methods, called at different times, with
//! nothing holding them together. `shared/finish.rs` addressed half of that by
//! deriving appearance from the collider's coefficients. This addresses the
//! other half: the derivation was a lossy projection of many physics parameters
//! onto two visual ones, and it failed exactly where it mattered. Polished
//! granite and weathered granite have identical friction and completely
//! different finishes; a grippy floor was forced to look like chalk.
//!
//! So a substance carries its finish rather than inferring it, and fans out to
//! everything that needs to agree about the material:
//!
//! ```text
//!                     ┌──▶ physics ──▶ ColliderDesc  (density, friction, restitution)
//!   Substance ────────┼──▶ finish  ──┐
//!   (GRANITE, OAK…)   ├──▶ grain   ──┼▶ Material
//!                     └──▶ palette ──┘  (and the procedural texture it draws)
//! ```
//!
//! [`Substance::from_physics`] keeps the old derivation available for a
//! spawnable that genuinely has nothing to say about how it should look. What
//! changed is that it is now the fallback rather than the only voice.

use crate::physics::ColliderDesc;
use crate::rendering::grain::GrainSpec;
use crate::rendering::material::{Material, SurfaceFinish};
use crate::rendering::physical_finish::PhysicalSurface;
use crate::rendering::substance::palette::Palette;
use crate::resources::textures::TextureHandle;

/// A named material: how it behaves, how it looks, and what it is made of.
#[derive(Clone, Copy, Debug)]
pub struct Substance {
    /// For diagnostics and bench sheets. Not used for lookup — substances are
    /// constants, not strings.
    pub name: &'static str,

    /// Density, friction and restitution, as the collider takes them.
    pub physics: PhysicalSurface,

    /// Roughness and metallic, authored rather than derived. A polished and a
    /// weathered block of the same rock differ here and nowhere else.
    pub finish: SurfaceFinish,

    /// The microstructure the surface shows under a highlight.
    pub grain: GrainSpec,

    /// Whether the grain is addressed by the mesh's texture coordinates rather
    /// than projected from its own frame.
    ///
    /// True for substances whose grain has a direction the mesh already knows —
    /// timber, whose fibre runs along the plank. A projection would run it
    /// along whichever axis a face happens to point at.
    pub grain_by_uv: bool,

    /// The colours a procedural texture for this substance draws from.
    pub palette: Palette,
}

impl Substance {
    /// A substance whose appearance is derived from its physics.
    ///
    /// The old behaviour, kept for spawnables that declare coefficients and
    /// nothing else. Grainless and unpalletted: a derivation can guess at
    /// roughness, but nothing about friction implies what a surface is *made*
    /// of, so it does not pretend to.
    pub fn from_physics(name: &'static str, physics: PhysicalSurface, palette: Palette) -> Self {
        Self {
            name,
            physics,
            finish: physics.finish(),
            grain: GrainSpec::NONE,
            grain_by_uv: false,
            palette,
        }
    }

    /// The same substance at a different density.
    ///
    /// For a spawnable whose density is authored per instance in level data
    /// while everything else about the material stays put. Deliberately does
    /// *not* re-derive the finish: a heavier block of granite is still granite.
    pub fn with_density(mut self, density: f32) -> Self {
        self.physics.density = density;
        self
    }

    /// The same substance with different coefficients throughout.
    ///
    /// For a spawnable whose physics is authored per instance in level data —
    /// a dolos whose whole point is that a level tunes how it grips and
    /// bounces — while its appearance stays the library's. The finish is not
    /// re-derived, which is exactly the drift the old derivation could not
    /// avoid: making a block bouncier should not repaint it.
    pub fn with_physics(mut self, physics: PhysicalSurface) -> Self {
        self.physics = physics;
        self
    }

    /// The same substance with all three coefficients authored, usable in a
    /// constant.
    ///
    /// For a spawnable whose physics is a gameplay decision rather than a
    /// material fact — the temple's stylobate grips far harder than real
    /// masonry because a stack of loose steps has to stand up. Its appearance
    /// still comes from the library, which is the separation that lets a
    /// surface be grippy without being forced to look like chalk.
    pub const fn with_coefficients(
        mut self,
        restitution: f32,
        friction: f32,
        density: f32,
    ) -> Self {
        self.physics = PhysicalSurface {
            restitution,
            friction,
            density,
        };
        self
    }

    /// The same substance at a different friction, for a level that tunes how
    /// an object grips without changing what it is made of.
    pub fn with_friction(mut self, friction: f32) -> Self {
        self.physics.friction = friction;
        self
    }

    /// The same substance with a different finish, for a variant a level wants
    /// that the library does not name — a wet stone, a waxed plank.
    pub fn with_finish(mut self, finish: SurfaceFinish) -> Self {
        self.finish = finish;
        self
    }

    /// The same substance showing more or less of its microstructure.
    pub fn with_grain(mut self, grain: GrainSpec) -> Self {
        self.grain = grain;
        self
    }

    /// A collider carrying this substance's coefficients.
    pub fn collider(&self, desc: ColliderDesc) -> ColliderDesc {
        desc.density(self.physics.density)
            .restitution(self.physics.restitution)
            .friction(self.physics.friction)
    }

    /// A material carrying this substance's finish and grain.
    pub fn material(&self, texture: TextureHandle) -> Material {
        self.shade(Material::textured(texture))
    }

    /// Apply this substance's finish and grain to an existing material.
    ///
    /// The one place the choice between the two grain addressing modes is made,
    /// so a spawnable never has to remember that timber is the odd one.
    pub fn shade(&self, material: Material) -> Material {
        let material = material.with_finish(self.finish);
        if self.grain_by_uv {
            material.with_uv_grain(self.grain)
        } else {
            material.with_grain(self.grain)
        }
    }
}

/// Configures a collider from a declared physical surface.
pub trait ColliderSubstance {
    /// Set density, restitution and (isotropic) friction from a substance.
    fn of(self, substance: &Substance) -> Self;
}

impl ColliderSubstance for ColliderDesc {
    fn of(self, substance: &Substance) -> Self {
        substance.collider(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::FrictionModel;
    use crate::rendering::colour::Colour;
    use crate::rendering::substance::library;

    fn palette() -> Palette {
        Palette::from_base(Colour::new(0.5, 0.5, 0.5, 1.0), 0.2)
    }

    #[test]
    fn a_collider_takes_every_coefficient() {
        let granite = library::GRANITE;
        let desc = ColliderDesc::sphere(0.5).of(&granite);

        assert_eq!(desc.density, granite.physics.density);
        assert_eq!(desc.material.restitution, granite.physics.restitution);
        assert_eq!(desc.material.friction(), granite.physics.friction);
        assert!(matches!(
            desc.material.friction,
            FrictionModel::Isotropic(_)
        ));
    }

    /// The whole reason the library exists. Under the old derivation these two
    /// were the same surface, because friction is all it had to go on.
    #[test]
    fn two_stones_that_behave_alike_can_still_look_different() {
        let rough = library::GRANITE;
        let polished = library::MARBLE;

        assert!((rough.physics.friction - polished.physics.friction).abs() < 0.25);
        assert!(
            rough.finish.roughness - polished.finish.roughness > 0.2,
            "granite {} vs marble {}",
            rough.finish.roughness,
            polished.finish.roughness
        );
    }

    /// A level authoring a heavier stone is not authoring a different material.
    #[test]
    fn changing_density_leaves_the_look_alone() {
        let heavy = library::GRANITE.with_density(4000.0);

        assert_eq!(heavy.physics.density, 4000.0);
        assert_eq!(heavy.finish.roughness, library::GRANITE.finish.roughness);
        assert_eq!(heavy.grain.scale, library::GRANITE.grain.scale);
    }

    /// Timber is the one substance whose grain has a direction, and forgetting
    /// that is the kind of thing every call site would get wrong once.
    #[test]
    fn timber_shades_its_grain_by_uv_and_stone_does_not() {
        use crate::rendering::surface_source::SurfaceSource;

        let plank = library::OAK.shade(Material::coloured(Colour::WHITE));
        assert!(plank.source.contains(SurfaceSource::GRAIN_BY_UV));
        assert!(!plank.source.contains(SurfaceSource::GRAIN_OBJECT_SPACE));

        let block = library::GRANITE.shade(Material::coloured(Colour::WHITE));
        assert!(block.source.contains(SurfaceSource::GRAIN_OBJECT_SPACE));
        assert!(!block.source.contains(SurfaceSource::GRAIN_BY_UV));
    }

    /// The fallback still works, and still refuses to invent a microstructure
    /// it has no basis for.
    /// Authoring physics per instance must not repaint the object. This is the
    /// failure mode the old friction-to-roughness derivation had by
    /// construction — a crate made bouncier came back looking different.
    #[test]
    fn authoring_physics_leaves_the_look_alone() {
        let tuned = library::CONCRETE.with_physics(PhysicalSurface::new(0.7, 0.3, 1000.0));

        assert_eq!(tuned.physics.restitution, 0.7);
        assert_eq!(tuned.finish.roughness, library::CONCRETE.finish.roughness);
        assert_eq!(tuned.grain.scale, library::CONCRETE.grain.scale);
    }

    #[test]
    fn a_derived_substance_takes_the_finish_its_physics_implies() {
        let physics = PhysicalSurface::new(0.6, 0.5, 100.0);
        let derived = Substance::from_physics("test", physics, palette());

        assert_eq!(derived.finish.roughness, physics.finish().roughness);
        assert!(!derived.grain.is_enabled());
    }
}
