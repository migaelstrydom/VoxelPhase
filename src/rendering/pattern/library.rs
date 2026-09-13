//! The patterns the game's surfaces are drawn with.
//!
//! Each is the recipe that was duplicated across the spawnables, written once.
//! What changed is that they name their colours by palette slot, so a pattern
//! and a substance compose: [`STONE`] over granite's palette is granite, and
//! over sandstone's palette is sandstone, with no second generator.
//!
//! Frequencies are in features per tile, and a tile is one texture repeat. The
//! ordering within a pattern matters — broad variation first, so later layers
//! modulate a surface that already has large-scale structure rather than a flat
//! one.

use crate::rendering::pattern::layer::{Layer, Pattern, Slot};

/// Broken igneous rock: blotchy, veined, and speckled with mineral grain.
///
/// The recipe menhir and temple-stone had independently arrived at, which is
/// the strongest evidence it was the right one.
pub const STONE: Pattern = Pattern {
    name: "stone",
    layers: &[
        // Large blotchy regions — what the surface reads as from a distance.
        Layer::Wash {
            scale: 3.0,
            octaves: 3,
            towards: Slot::Light,
            amount: 1.0,
        },
        // Weathering: bumpy mid-frequency undulation.
        Layer::Shade {
            scale: 8.0,
            octaves: 3,
            amount: 0.15,
        },
        // Lichen and mineral deposits.
        Layer::Patch {
            scale: 4.0,
            octaves: 2,
            threshold: 0.58,
            span: 0.18,
            amount: 0.5,
            towards: Slot::Dark,
        },
        // The odd hairline crack. Thin and faint on purpose: at any width
        // worth noticing these stop reading as cracks in a rock and start
        // reading as a net thrown over it.
        Layer::Vein {
            scale: 9.0,
            octaves: 2,
            sharpness: 18.0,
            amount: 0.18,
            towards: Slot::Dark,
        },
        // Mineral speckle. Last, so it sits on everything above it.
        Layer::Shade {
            scale: 24.0,
            octaves: 2,
            amount: 0.1,
        },
    ],
};

/// Dressed stone: the same rock with the broken face worked off. Keeps the
/// blotching and the grain, loses the cracks.
pub const DRESSED_STONE: Pattern = Pattern {
    name: "dressed_stone",
    layers: &[
        Layer::Wash {
            scale: 2.0,
            octaves: 2,
            towards: Slot::Light,
            amount: 0.8,
        },
        Layer::Shade {
            scale: 8.0,
            octaves: 3,
            amount: 0.12,
        },
        Layer::Patch {
            scale: 5.0,
            octaves: 2,
            threshold: 0.62,
            span: 0.2,
            amount: 0.3,
            towards: Slot::Dark,
        },
        Layer::Shade {
            scale: 28.0,
            octaves: 2,
            amount: 0.08,
        },
    ],
};

/// Dressed stone that has stood outside for a few centuries.
///
/// [`DRESSED_STONE`] is what the mason handed over; this is what the weather
/// gave back. The difference is structure at every scale — staining in broad
/// discrete regions, cracks running across the face, and chipped pitting —
/// because a surface whose only variation is a fine speckle reads as uniform
/// roughness however much of it there is.
pub const WEATHERED_STONE: Pattern = Pattern {
    name: "weathered_stone",
    layers: &[
        // Broad tonal variation, stronger than dressed stone's: sunlight and
        // rain do not fade a wall evenly.
        Layer::Wash {
            scale: 2.0,
            octaves: 3,
            towards: Slot::Light,
            amount: 0.9,
        },
        // Erosion relief — where the face has worn hollow.
        Layer::Shade {
            scale: 6.0,
            octaves: 3,
            amount: 0.16,
        },
        // Damp staining, in regions large enough to see across a courtyard.
        Layer::Patch {
            scale: 2.5,
            octaves: 3,
            threshold: 0.52,
            span: 0.24,
            amount: 0.3,
            towards: Slot::Dark,
        },
        // The cracks. Few per face and thin, so they read as fractures in the
        // block rather than as a pattern printed on it.
        Layer::Vein {
            scale: 6.0,
            octaves: 3,
            sharpness: 45.0,
            amount: 0.6,
            towards: Slot::Accent,
        },
        // A second, finer set at another frequency, so the cracks branch and
        // cross instead of running as one family of parallel lines.
        Layer::Vein {
            scale: 11.0,
            octaves: 2,
            sharpness: 60.0,
            amount: 0.35,
            towards: Slot::Dark,
        },
        // Pitting: small, scattered, and dark. A high threshold over a narrow
        // span is what makes these discrete chips rather than more mottling.
        Layer::Patch {
            scale: 30.0,
            octaves: 2,
            threshold: 0.72,
            span: 0.05,
            amount: 0.45,
            towards: Slot::Dark,
        },
        Layer::Shade {
            scale: 26.0,
            octaves: 2,
            amount: 0.06,
        },
    ],
};

/// Polished stone with fine veining.
///
/// The veins are marble's whole character, so they are the thinnest in the
/// library — but restraint matters more here than anywhere else: veining broad
/// enough to read as mottling turns polished marble into weathered concrete,
/// which is what the first pass at these numbers produced.
pub const MARBLE: Pattern = Pattern {
    name: "marble",
    layers: &[
        Layer::Wash {
            scale: 2.0,
            octaves: 2,
            towards: Slot::Light,
            amount: 0.6,
        },
        Layer::Vein {
            scale: 6.0,
            octaves: 4,
            sharpness: 9.0,
            amount: 0.45,
            towards: Slot::Accent,
        },
        // A second vein set at a different frequency, so the marbling branches
        // instead of running as one family of parallel lines.
        Layer::Vein {
            scale: 11.0,
            octaves: 3,
            sharpness: 13.0,
            amount: 0.25,
            towards: Slot::Dark,
        },
        Layer::Shade {
            scale: 30.0,
            octaves: 2,
            amount: 0.05,
        },
    ],
};

/// Cast aggregate: broad staining over a coarse speckle, and no veins — poured
/// concrete has no crystal structure to crack along.
pub const CONCRETE: Pattern = Pattern {
    name: "concrete",
    layers: &[
        Layer::Wash {
            scale: 4.0,
            octaves: 3,
            towards: Slot::Light,
            amount: 0.4,
        },
        // Damp staining, in broad discrete regions rather than as mottling.
        Layer::Patch {
            scale: 2.5,
            octaves: 2,
            threshold: 0.55,
            span: 0.22,
            amount: 0.6,
            towards: Slot::Dark,
        },
        Layer::Shade {
            scale: 32.0,
            octaves: 2,
            amount: 0.1,
        },
    ],
};

/// Cleaved roofing stone: banded along one axis, smooth, and nearly veinless.
pub const SLATE: Pattern = Pattern {
    name: "slate",
    layers: &[
        Layer::Wash {
            scale: 3.0,
            octaves: 2,
            towards: Slot::Light,
            amount: 0.5,
        },
        // The cleavage planes: thin, and the only thing that says this rock
        // splits into sheets.
        Layer::Vein {
            scale: 9.0,
            octaves: 2,
            sharpness: 6.0,
            amount: 0.35,
            towards: Slot::Dark,
        },
        Layer::Shade {
            scale: 26.0,
            octaves: 2,
            amount: 0.07,
        },
    ],
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rendering::colour::Colour;
    use crate::rendering::substance::Palette;

    const ALL: &[Pattern] = &[
        STONE,
        DRESSED_STONE,
        WEATHERED_STONE,
        MARBLE,
        CONCRETE,
        SLATE,
    ];

    /// Names key the texture cache. Two patterns sharing one would have the
    /// cache hand out the wrong texture, silently and only sometimes.
    #[test]
    fn every_pattern_has_a_distinct_name() {
        for (i, a) in ALL.iter().enumerate() {
            for b in ALL.iter().skip(i + 1) {
                assert_ne!(a.name, b.name, "two patterns are called {}", a.name);
            }
        }
    }

    #[test]
    fn every_pattern_does_something() {
        assert!(ALL.iter().all(|p| !p.layers.is_empty()));
    }

    /// Patterns exist to be told apart. Two that render alike over the same
    /// palette are one pattern with two names.
    #[test]
    fn no_two_patterns_render_alike() {
        let palette = Palette::from_base(Colour::new(0.6, 0.6, 0.6, 1.0), 0.22);

        for (i, a) in ALL.iter().enumerate() {
            for b in ALL.iter().skip(i + 1) {
                let mut difference = 0.0;
                for step in 0..64 {
                    let u = (step % 8) as f32 / 8.0;
                    let v = (step / 8) as f32 / 8.0;
                    let ca = a.sample(u, v, &palette, 1);
                    let cb = b.sample(u, v, &palette, 1);
                    difference += (ca.r - cb.r).abs();
                }

                assert!(
                    difference / 64.0 > 0.01,
                    "{} and {} render alike",
                    a.name,
                    b.name
                );
            }
        }
    }

    /// The arch asked for weathered stone because dressed stone had no
    /// structure to see. Measured as the fraction of the tile the cracks and
    /// pits darken well below its own average — a dark tail a fine speckle
    /// never reaches, however much of it there is.
    #[test]
    fn weathered_stone_is_more_broken_than_dressed() {
        let palette = Palette::from_base(Colour::new(0.72, 0.69, 0.61, 1.0), 0.16);

        let broken = |pattern: &Pattern| {
            let samples: Vec<f32> = (0..4096)
                .map(|step| {
                    let u = (step % 64) as f32 / 64.0;
                    let v = (step / 64) as f32 / 64.0;
                    pattern.sample(u, v, &palette, 77).r
                })
                .collect();

            let mean = samples.iter().sum::<f32>() / samples.len() as f32;
            let dark = samples.iter().filter(|&&r| r < mean - 0.1).count();
            dark as f32 / samples.len() as f32
        };

        // Cracks are thin on purpose, so the fraction is small in absolute
        // terms; what matters is that dressed stone's is near zero.
        assert!(
            broken(&WEATHERED_STONE) > 0.01
                && broken(&WEATHERED_STONE) > broken(&DRESSED_STONE) * 5.0,
            "weathered {} vs dressed {}",
            broken(&WEATHERED_STONE),
            broken(&DRESSED_STONE)
        );
    }

    /// Marble's character is its veining, and a veinless marble is just a pale
    /// stone. Measured as how much of the tile the accent reaches.
    #[test]
    fn marble_is_more_veined_than_dressed_stone() {
        let palette = Palette::from_base(Colour::new(0.85, 0.84, 0.8, 1.0), 0.2)
            .with_accent(Colour::new(0.3, 0.3, 0.32, 1.0));

        let veining = |pattern: &Pattern| {
            let mut darkest: f32 = 1.0;
            for step in 0..400 {
                let u = (step % 20) as f32 / 20.0;
                let v = (step / 20) as f32 / 20.0;
                darkest = darkest.min(pattern.sample(u, v, &palette, 4).r);
            }
            darkest
        };

        assert!(
            veining(&MARBLE) < veining(&DRESSED_STONE),
            "marble {} vs dressed {}",
            veining(&MARBLE),
            veining(&DRESSED_STONE)
        );
    }
}
