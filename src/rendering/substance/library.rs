//! The substances the game is built from.
//!
//! One declaration per material, so that two things made of granite are made of
//! the same granite. Before this, every stone spawnable invented its own grey,
//! its own friction and its own roughness, and the differences between them
//! were accidents rather than decisions.
//!
//! # How the numbers were chosen
//!
//! Densities are real, in kg/m³, because destruction and buoyancy read them and
//! a lie there is a lie about gameplay. Friction and restitution are plausible
//! rather than measured — they are tuned for how an object *plays*, and the
//! engine exists to show off its physics rather than to reproduce it.
//!
//! Finishes are authored. The pairing that matters is roughness against grain:
//! a tight specular lobe on a smooth surface reads as plastic, and relief under
//! a fully diffuse lobe is invisible. Every entry here sets both together, which
//! is the thing a derivation from friction could never do.

use crate::rendering::colour::Colour;
use crate::rendering::grain::GrainSpec;
use crate::rendering::material::SurfaceFinish;
use crate::rendering::physical_finish::PhysicalSurface;
use crate::rendering::substance::palette::Palette;
use crate::rendering::substance::spec::Substance;

/// Build a stone-family substance. They differ only in the four numbers that
/// follow, and spelling each one out in full would bury that.
const fn stone(
    name: &'static str,
    density: f32,
    friction: f32,
    roughness: f32,
    grain: GrainSpec,
    base: Colour,
    spread: f32,
) -> Substance {
    Substance {
        name,
        physics: PhysicalSurface {
            restitution: 0.1,
            friction,
            density,
        },
        finish: SurfaceFinish {
            roughness,
            metallic: 0.0,
        },
        grain,
        grain_by_uv: false,
        palette: Palette::from_base_const(base, spread),
    }
}

/// Coarse-grained igneous rock. The default standing stone: heavy, grippy, and
/// rough enough that its crystal structure catches the light.
pub const GRANITE: Substance = stone(
    "granite",
    2700.0,
    0.8,
    0.55,
    GrainSpec::STONE,
    Colour::new(0.58, 0.56, 0.54, 1.0),
    0.22,
);

/// Softer sedimentary rock, the stone of a dressed arch. Paler and finer than
/// granite, and worked rather than broken, so it shows less relief.
pub const LIMESTONE: Substance = stone(
    "limestone",
    2400.0,
    0.75,
    0.62,
    GrainSpec::DRESSED_STONE,
    Colour::new(0.72, 0.69, 0.61, 1.0),
    0.16,
);

/// Polished stone. The one material that proves the point of this library:
/// marble and granite have near-identical friction and could never have been
/// told apart by a derivation from it.
pub const MARBLE: Substance = stone(
    "marble",
    2700.0,
    0.6,
    0.22,
    GrainSpec::DRESSED_STONE.with_strength(0.25),
    Colour::new(0.86, 0.85, 0.82, 1.0),
    0.12,
);

/// Weathered sedimentary rock: warm, soft, and the roughest of the stones.
pub const SANDSTONE: Substance = stone(
    "sandstone",
    2200.0,
    0.85,
    0.72,
    GrainSpec::STONE.with_strength(0.85),
    Colour::new(0.74, 0.62, 0.45, 1.0),
    0.18,
);

/// Cast aggregate. Coarser microstructure than any natural stone, because the
/// aggregate is bigger than the crystal.
pub const CONCRETE: Substance = stone(
    "concrete",
    2400.0,
    0.9,
    0.7,
    GrainSpec::CONCRETE,
    Colour::new(0.66, 0.65, 0.63, 1.0),
    0.14,
);

/// Fired clay. A brick wall's character is its mortar, so the accent carries it
/// rather than being a darker brick.
pub const BRICK: Substance = Substance {
    name: "brick",
    physics: PhysicalSurface {
        restitution: 0.1,
        friction: 0.85,
        density: 1900.0,
    },
    finish: SurfaceFinish {
        roughness: 0.68,
        metallic: 0.0,
    },
    grain: GrainSpec::DRESSED_STONE,
    grain_by_uv: false,
    palette: Palette::from_base_const(Colour::new(0.55, 0.27, 0.20, 1.0), 0.2)
        .with_accent(Colour::new(0.72, 0.70, 0.66, 1.0)),
};

/// Split roofing stone: dark, dense, and the smoothest of the stones because a
/// cleaved face is genuinely flat.
pub const SLATE: Substance = stone(
    "slate",
    2800.0,
    0.7,
    0.4,
    GrainSpec::DRESSED_STONE.with_strength(0.4),
    Colour::new(0.32, 0.34, 0.37, 1.0),
    0.2,
);

/// Build a timber substance. Grain runs along the mesh's v axis, which is what
/// separates wood from every other material in the library.
const fn timber(
    name: &'static str,
    density: f32,
    roughness: f32,
    base: Colour,
    accent: Colour,
) -> Substance {
    Substance {
        name,
        physics: PhysicalSurface {
            restitution: 0.25,
            friction: 0.6,
            density,
        },
        finish: SurfaceFinish {
            roughness,
            metallic: 0.0,
        },
        grain: GrainSpec::WOOD,
        grain_by_uv: true,
        palette: Palette::from_base_const(base, 0.22).with_accent(accent),
    }
}

/// Dense hardwood: furniture, a heavy crate, a plank bridge.
pub const OAK: Substance = timber(
    "oak",
    750.0,
    0.6,
    Colour::new(0.55, 0.40, 0.26, 1.0),
    Colour::new(0.30, 0.20, 0.12, 1.0),
);

/// Light softwood: a fence post, a knocked-together box. Knots are its accent.
pub const PINE: Substance = timber(
    "pine",
    500.0,
    0.68,
    Colour::new(0.74, 0.60, 0.40, 1.0),
    Colour::new(0.42, 0.30, 0.17, 1.0),
);

/// Structural steel. The only substance here that reads as metal, and the
/// density is what says so — see `physical_finish`, which draws the same line.
pub const STEEL: Substance = Substance {
    name: "steel",
    physics: PhysicalSurface {
        restitution: 0.4,
        friction: 0.35,
        density: 7800.0,
    },
    finish: SurfaceFinish {
        roughness: 0.28,
        metallic: 1.0,
    },
    // A brushed rather than a cast finish: metal is the one material where a
    // perfectly flat normal is genuinely wrong, and the one that wants least of
    // the correction.
    grain: GrainSpec::DRESSED_STONE.with_strength(0.2),
    grain_by_uv: false,
    palette: Palette::from_base_const(Colour::new(0.62, 0.64, 0.67, 1.0), 0.18),
};

/// Bouncy and grippy, and deliberately smooth: bounce reads as a coated,
/// rubbery surface, and grain on it would read as perished.
pub const RUBBER: Substance = Substance {
    name: "rubber",
    physics: PhysicalSurface {
        restitution: 0.85,
        friction: 0.9,
        density: 1100.0,
    },
    finish: SurfaceFinish {
        roughness: 0.35,
        metallic: 0.0,
    },
    grain: GrainSpec::NONE,
    grain_by_uv: false,
    palette: Palette::from_base_const(Colour::new(0.22, 0.22, 0.24, 1.0), 0.25),
};

/// Moulded plastic: the bright toy props. Smooth on purpose — the visual
/// direction is crisp plastic, and these are the surfaces it is crisp against.
pub const PLASTIC: Substance = Substance {
    name: "plastic",
    physics: PhysicalSurface {
        restitution: 0.45,
        friction: 0.5,
        density: 950.0,
    },
    finish: SurfaceFinish {
        roughness: 0.2,
        metallic: 0.0,
    },
    grain: GrainSpec::NONE,
    grain_by_uv: false,
    palette: Palette::from_base_const(Colour::new(0.8, 0.8, 0.82, 1.0), 0.2),
};

#[cfg(test)]
mod tests {
    use super::*;

    /// Every substance in the library, for the invariants that must hold across
    /// all of them.
    const ALL: &[Substance] = &[
        GRANITE, LIMESTONE, MARBLE, SANDSTONE, CONCRETE, BRICK, SLATE, OAK, PINE, STEEL, RUBBER,
        PLASTIC,
    ];

    #[test]
    fn every_substance_has_a_plausible_density() {
        for substance in ALL {
            assert!(
                (100.0..20000.0).contains(&substance.physics.density),
                "{} has a density of {}",
                substance.name,
                substance.physics.density
            );
        }
    }

    #[test]
    fn every_finish_is_in_range() {
        for substance in ALL {
            assert!(
                (0.0..=1.0).contains(&substance.finish.roughness),
                "{} roughness {}",
                substance.name,
                substance.finish.roughness
            );
            assert!((0.0..=1.0).contains(&substance.finish.metallic));
        }
    }

    /// Only genuinely metal-dense things read as metal. Heavy rock should read
    /// heavy, not metallic — the same line `physical_finish` draws.
    #[test]
    fn nothing_but_steel_reads_as_metal() {
        for substance in ALL {
            if substance.name == "steel" {
                assert!(substance.finish.metallic > 0.9);
            } else {
                assert_eq!(
                    substance.finish.metallic, 0.0,
                    "{} reads as metal",
                    substance.name
                );
            }
        }
    }

    /// The pairing the whole visual model rests on: a surface glossy enough to
    /// show a tight highlight needs microstructure for that highlight to break
    /// up against, or it reads as plastic. Plastic and rubber are the exception
    /// because plastic is what they are.
    #[test]
    fn a_glossy_substance_has_grain_to_break_its_highlight_up() {
        for substance in ALL {
            let moulded = matches!(substance.name, "plastic" | "rubber");
            if substance.finish.roughness < 0.45 && !moulded {
                assert!(
                    substance.grain.is_enabled(),
                    "{} is glossy ({}) with no microstructure — it will read as plastic",
                    substance.name,
                    substance.finish.roughness
                );
            }
        }
    }

    /// Timber is the only family whose grain has a direction.
    #[test]
    fn only_timber_addresses_its_grain_by_uv() {
        for substance in ALL {
            let is_timber = matches!(substance.name, "oak" | "pine");
            assert_eq!(
                substance.grain_by_uv, is_timber,
                "{} disagrees about how its grain is addressed",
                substance.name
            );
        }
    }

    /// Substances exist to be told apart. Two that behave alike must still look
    /// different, and two that look alike must behave differently — otherwise
    /// one of them is redundant.
    #[test]
    fn no_two_substances_are_the_same_material_twice() {
        for (i, a) in ALL.iter().enumerate() {
            for b in ALL.iter().skip(i + 1) {
                let visual = (a.finish.roughness - b.finish.roughness).abs()
                    + (a.finish.metallic - b.finish.metallic).abs()
                    + (a.palette.base.r - b.palette.base.r).abs()
                    + (a.palette.base.g - b.palette.base.g).abs()
                    + (a.palette.base.b - b.palette.base.b).abs();
                let physical = (a.physics.density - b.physics.density).abs() / 1000.0
                    + (a.physics.friction - b.physics.friction).abs()
                    + (a.physics.restitution - b.physics.restitution).abs();

                assert!(
                    visual + physical > 0.15,
                    "{} and {} are the same material twice",
                    a.name,
                    b.name
                );
            }
        }
    }

    /// The library's own claim about itself: marble is polished granite. If a
    /// tuning pass ever brings their roughness together, the point of authoring
    /// finishes instead of deriving them has been lost.
    #[test]
    fn marble_is_the_polished_stone_and_sandstone_the_rough_one() {
        assert!(MARBLE.finish.roughness < GRANITE.finish.roughness);
        assert!(GRANITE.finish.roughness < SANDSTONE.finish.roughness);
    }
}
