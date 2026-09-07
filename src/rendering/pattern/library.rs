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
        // Thin cracks and veins.
        Layer::Vein {
            scale: 14.0,
            octaves: 3,
            sharpness: 6.0,
            amount: 0.6,
            towards: Slot::Accent,
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

    const ALL: &[Pattern] = &[STONE, DRESSED_STONE, MARBLE, CONCRETE, SLATE];

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
