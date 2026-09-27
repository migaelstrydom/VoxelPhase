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
use crate::rendering::reflection::Reflects;
use crate::rendering::substance::palette::Palette;
use crate::rendering::substance::spec::Substance;
use crate::rendering::transparency::Transparency;

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
        transparency: Transparency::OPAQUE,
        reflects: Reflects::Sky,
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

/// Softer sedimentary rock, the stone of an old arch: warm buff, worked once
/// and weathered since, so its grain is soft and its stains are umber and
/// olive rather than grey. The base is held below the palest a stone could
/// be, so that grime and lichen have somewhere to go and the block does not
/// read as plaster in full sun.
pub const LIMESTONE: Substance = Substance {
    palette: Palette::from_base_const(Colour::new(0.64, 0.60, 0.50, 1.0), 0.16)
        .with_dark(Colour::new(0.44, 0.39, 0.31, 1.0))
        .with_accent(Colour::new(0.34, 0.29, 0.22, 1.0)),
    ..stone(
        "limestone",
        2400.0,
        0.75,
        0.62,
        GrainSpec::WEATHERED_STONE,
        Colour::new(0.64, 0.60, 0.50, 1.0),
        0.16,
    )
};

/// Hard silcrete, the stone of standing stones: as heavy and grippy as
/// granite, but a warm grey-brown, weathered for millennia rather than
/// dressed. Its darks are brown rather than blue-grey, which in a cool sky
/// keeps a shaded face reading as stone and not as slate.
pub const SARSEN: Substance = Substance {
    palette: Palette::from_base_const(Colour::new(0.62, 0.60, 0.55, 1.0), 0.16)
        .with_dark(Colour::new(0.43, 0.40, 0.35, 1.0))
        .with_accent(Colour::new(0.33, 0.30, 0.25, 1.0)),
    ..stone(
        "sarsen",
        2650.0,
        0.8,
        0.6,
        GrainSpec::WEATHERED_STONE,
        Colour::new(0.62, 0.60, 0.55, 1.0),
        0.16,
    )
};

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
    transparency: Transparency::OPAQUE,
    reflects: Reflects::Sky,
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
        transparency: Transparency::OPAQUE,
        reflects: Reflects::Sky,
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

/// Structural steel. The one metal that is worked rather than polished, and
/// the density is what says it is metal — see `physical_finish`, which draws the same line.
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
    transparency: Transparency::OPAQUE,
    reflects: Reflects::Sky,
};

/// Build a pure-metal substance. Fully metallic, so its colour is all in the
/// reflection, and with no grain: a finished face is optically flat, and the
/// reflected horizon running straight across it is what says metal. Any grain
/// at all crinkles that line into a hammered finish. How polished it is, is
/// the roughness alone.
const fn pure_metal(
    name: &'static str,
    density: f32,
    friction: f32,
    restitution: f32,
    roughness: f32,
    palette: Palette,
) -> Substance {
    Substance {
        name,
        physics: PhysicalSurface {
            restitution,
            friction,
            density,
        },
        finish: SurfaceFinish {
            roughness,
            metallic: 1.0,
        },
        grain: GrainSpec::NONE,
        grain_by_uv: false,
        palette,
        transparency: Transparency::OPAQUE,
        // A metal shows no colour of its own but what it reflects; the sky
        // alone leaves its downward half a flat grey.
        reflects: Reflects::Surroundings,
    }
}

/// Ground tungsten: the heaviest thing in the library, and grey in a way steel
/// is not — darker, flatter and a shade warm. A hand-sized block of it weighs
/// what a person does, which is the whole joke of owning one.
pub const TUNGSTEN: Substance = pure_metal(
    "tungsten",
    19250.0,
    0.4,
    0.3,
    0.22,
    Palette::from_base_const(Colour::new(0.66, 0.65, 0.62, 1.0), 0.08),
);

/// Polished gold: as dense as tungsten, and soft enough that it grips a little
/// more and bounces less. The only warm-white metal here.
pub const GOLD: Substance = pure_metal(
    "gold",
    19300.0,
    0.5,
    0.15,
    0.14,
    Palette::from_base_const(Colour::new(1.0, 0.74, 0.26, 1.0), 0.08),
);

/// Polished copper, starting to tarnish: the accent is the brown oxide that
/// gathers where it has been handled.
pub const COPPER: Substance = pure_metal(
    "copper",
    8960.0,
    0.45,
    0.25,
    0.18,
    Palette::from_base_const(Colour::new(0.96, 0.52, 0.34, 1.0), 0.08)
        .with_accent(Colour::new(0.48, 0.27, 0.18, 1.0)),
);

/// Machined aluminium: the brightest, whitest metal here, and the odd one out
/// for weight — barely heavier than stone, so a cube of it is the one that
/// can actually be shoved about. A shade rougher than the rest, because it is
/// turned rather than lapped.
pub const ALUMINIUM: Substance = pure_metal(
    "aluminium",
    2700.0,
    0.45,
    0.3,
    0.26,
    Palette::from_base_const(Colour::new(0.9, 0.91, 0.92, 1.0), 0.08),
);

/// Bismuth: a pinkish silver, and brittle enough that it barely bounces. The
/// accent is the blue-violet of the oxide film it grows, which is what makes
/// a bismuth crystal iridescent; on a polished cube it shows only where the
/// surface has been handled.
pub const BISMUTH: Substance = pure_metal(
    "bismuth",
    9780.0,
    0.4,
    0.1,
    0.2,
    Palette::from_base_const(Colour::new(0.8, 0.72, 0.73, 1.0), 0.08)
        .with_accent(Colour::new(0.42, 0.4, 0.78, 1.0)),
);

/// Lithium: the lightest metal, and the only one here that floats — at about
/// half the density of water. Soft, and never polished for long: the tarnish
/// it takes on in air is the dark accent, and the finish is the dullest of
/// the metals.
pub const LITHIUM: Substance = pure_metal(
    "lithium",
    534.0,
    0.6,
    0.1,
    0.42,
    Palette::from_base_const(Colour::new(0.7, 0.7, 0.68, 1.0), 0.1)
        .with_accent(Colour::new(0.3, 0.3, 0.3, 1.0)),
);

/// Osmium: the densest element there is, heavier even than tungsten, and the
/// one metal with a blue cast of its own.
pub const OSMIUM: Substance = pure_metal(
    "osmium",
    22590.0,
    0.35,
    0.3,
    0.16,
    Palette::from_base_const(Colour::new(0.6, 0.64, 0.7, 1.0), 0.08),
);

/// Titanium: a warm mid-grey, half the density of steel. Where it is heated
/// it grows an oxide film thin enough to colour by interference, which is
/// the anodised blue its markings take on.
pub const TITANIUM: Substance = pure_metal(
    "titanium",
    4506.0,
    0.45,
    0.35,
    0.22,
    Palette::from_base_const(Colour::new(0.62, 0.6, 0.57, 1.0), 0.08),
);

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
    transparency: Transparency::OPAQUE,
    reflects: Reflects::Sky,
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
    transparency: Transparency::OPAQUE,
    reflects: Reflects::Sky,
};

/// Frozen water. The library's first transmissive substance, and the one that
/// motivated the blended pass.
///
/// The physics is as unusual as the optics. Ice is barely denser than water
/// and less than half as dense as the lightest stone, so a block of it is
/// startlingly light for its size; and its friction is the lowest number in
/// the library by a wide margin, which is the entire reason to want one. The
/// restitution is deliberately not bouncy — ice is brittle, not springy.
///
/// The finish is the polished end of the library, paired with a faint stone
/// grain. Deliberately not *the* smoothest: at a roughness below about 0.1 the
/// sun's highlight narrows to a speck barely a degree wide, and on a block
/// whose shine is supposed to come from a dozen narrow bevels that means no
/// highlight lands on any of them. Ice is glassy, not a mirror. The grain is
/// fine and weak for the same reason it is there at all — frost on the
/// surface, not the crystal structure of a rock.
pub const ICE: Substance = Substance {
    name: "ice",
    physics: PhysicalSurface {
        restitution: 0.15,
        friction: 0.06,
        density: 917.0,
    },
    finish: SurfaceFinish {
        roughness: 0.14,
        metallic: 0.0,
    },
    // Directional, and addressed by UV so that the relief runs the same way as
    // the fracture planes the pattern draws. Object-space projection would put
    // the streaks along whichever world axis a face happened to point at, and
    // change which one as the cube tumbles.
    grain: GrainSpec::ICE,
    grain_by_uv: true,
    // Pale blue, with two slots doing jobs the base cannot. The light is white
    // rather than a brighter blue, because that slot carries the frost and
    // frost is the one part of a block of ice with no colour in it at all. The
    // accent goes *deeper* blue rather than darker grey: a thick part of a
    // block looks like more of the same colour, not like a shadow.
    palette: Palette::from_base_const(Colour::new(0.58, 0.78, 0.91, 1.0), 0.16)
        .with_light(Colour::new(0.95, 0.98, 1.0, 1.0))
        .with_accent(Colour::new(0.26, 0.55, 0.78, 1.0)),
    transparency: Transparency::ICE,
    reflects: Reflects::Sky,
};

/// Window glass: the one substance in the library you are meant to look
/// through rather than at.
///
/// Denser than any stone but marble and with none of the give of plastic —
/// glass does not bounce, it breaks — and the finish is the smoothest here by
/// a margin: a flat pane is a mirror at a grazing angle, and the speck-narrow
/// highlight ice avoids is exactly what a window shows the sun as.
///
/// The palette barely matters. At a window's opacity the base colour is the
/// faint green tint a thick edge of float glass shows; the light slot is the
/// white of a reflection, and the accent the deeper green of a shard seen
/// edge-on. The transparency does the rest.
pub const GLASS: Substance = Substance {
    name: "glass",
    physics: PhysicalSurface {
        restitution: 0.1,
        friction: 0.4,
        density: 2500.0,
    },
    finish: SurfaceFinish {
        roughness: 0.06,
        metallic: 0.0,
    },
    grain: GrainSpec::NONE,
    grain_by_uv: false,
    palette: Palette::from_base_const(Colour::new(0.82, 0.93, 0.90, 1.0), 0.08)
        .with_light(Colour::new(0.98, 1.0, 1.0, 1.0))
        .with_accent(Colour::new(0.50, 0.76, 0.72, 1.0)),
    transparency: Transparency::GLASS,
    reflects: Reflects::Sky,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rendering::grain::GrainLayer;

    /// Every substance in the library, for the invariants that must hold across
    /// all of them.
    const ALL: &[Substance] = &[
        GRANITE, LIMESTONE, SARSEN, MARBLE, SANDSTONE, CONCRETE, BRICK, SLATE, OAK, PINE, STEEL,
        TUNGSTEN, GOLD, COPPER, ALUMINIUM, BISMUTH, LITHIUM, OSMIUM, TITANIUM, RUBBER, PLASTIC,
        ICE, GLASS,
    ];

    /// The metals, which are the only things that read as metal.
    const METALS: &[&str] = &[
        "steel",
        "tungsten",
        "gold",
        "copper",
        "aluminium",
        "bismuth",
        "lithium",
        "osmium",
        "titanium",
    ];

    /// Nothing is lighter than balsa or denser than osmium, the densest
    /// element there is.
    #[test]
    fn every_substance_has_a_plausible_density() {
        for substance in ALL {
            assert!(
                (100.0..=22590.0).contains(&substance.physics.density),
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
    fn nothing_but_the_metals_reads_as_metal() {
        for substance in ALL {
            if METALS.contains(&substance.name) {
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
    /// because plastic is what they are; glass and polished metal are the
    /// other, because they are optically flat and the tight, unbroken
    /// reflection *is* the look.
    #[test]
    fn a_glossy_substance_has_grain_to_break_its_highlight_up() {
        for substance in ALL {
            let flat_by_nature = matches!(substance.name, "plastic" | "rubber" | "glass")
                || METALS.contains(&substance.name);
            if substance.finish.roughness < 0.45 && !flat_by_nature {
                assert!(
                    substance.grain.is_enabled(),
                    "{} is glossy ({}) with no microstructure — it will read as plastic",
                    substance.name,
                    substance.finish.roughness
                );
            }
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

    /// Ice and glass are the substances you can see through, and the check
    /// that nothing else has quietly acquired transparency it did not mean to
    /// — a stray opacity would move that material into the sorted pass and
    /// change how every object made of it is drawn. Of the two, glass is the
    /// one you can read through.
    #[test]
    fn ice_and_glass_are_the_only_things_light_gets_through() {
        assert!(GLASS.transparency.opacity < ICE.transparency.opacity);
        for substance in ALL {
            assert_eq!(
                substance.transparency.is_blended(),
                matches!(substance.name, "ice" | "glass"),
                "{} disagrees about whether light passes through it",
                substance.name
            );
        }
    }

    /// A grain with a direction has to be addressed by the mesh's own texture
    /// coordinates, because the direction is a property of the object and an
    /// object-space projection would run it along whichever world axis a face
    /// happens to point at — and change which one as the object tumbles. The
    /// two settings are declared independently, so nothing but this stops a
    /// substance from pairing them wrongly.
    #[test]
    fn a_directional_grain_is_addressed_by_uv() {
        for substance in ALL {
            let directional = substance.grain.layer == GrainLayer::Fibre;
            if !substance.grain.is_enabled() {
                continue;
            }
            assert_eq!(
                substance.grain_by_uv, directional,
                "{} disagrees about how its grain is addressed",
                substance.name
            );
        }
    }

    /// The property that makes an ice cube worth spawning: it is the
    /// slipperiest thing in the game, by a margin nothing can close by
    /// accident.
    #[test]
    fn nothing_grips_less_than_ice() {
        for substance in ALL {
            if substance.name == "ice" {
                continue;
            }
            assert!(
                substance.physics.friction > ICE.physics.friction * 3.0,
                "{} ({}) is within reach of ice ({})",
                substance.name,
                substance.physics.friction,
                ICE.physics.friction
            );
        }
    }
}
