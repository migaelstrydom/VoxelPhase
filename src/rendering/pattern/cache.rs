//! One texture per distinct surface, not one per object.
//!
//! Nothing shared textures. A seven-column temple baked and uploaded seven
//! identical 256×256 marble tiles, a level with twenty menhirs uploaded twenty
//! stone tiles, and the only reason it was not worse is that levels are small.
//! Each is a quarter-megabyte of noise evaluation and a GPU upload for a result
//! already sitting in memory.
//!
//! The cache is keyed by everything that goes into the bake, so a hit is
//! byte-identical by construction rather than by assumption:
//!
//! ```text
//!   (pattern name, palette, seed, size) ──▶ TextureHandle
//! ```
//!
//! Objects that *want* to differ still do — they pass different seeds, which is
//! exactly what a menhir does so two standing stones are not the same rock
//! twice. The cache does not take that away; it removes the case where the
//! inputs were identical and the work was repeated anyway.

use std::collections::HashMap;

use crate::core::error::EngineResult;
use crate::rendering::pattern::layer::Pattern;
use crate::rendering::substance::Palette;
use crate::resources::textures::{TextureHandle, TextureManager};

/// Everything a baked pattern depends on.
///
/// Floats are keyed by their bits rather than their value, because a cache key
/// needs `Eq` and float equality does not provide it. Two palettes that differ
/// in the last bit are a miss, which costs a bake and is always correct — the
/// opposite mistake would hand out the wrong texture.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct TextureKey {
    pattern: &'static str,
    palette: [u32; 16],
    seed: u32,
    size: u32,
}

impl TextureKey {
    /// The key for baking `pattern` over `palette`.
    pub fn new(pattern: &Pattern, palette: &Palette, seed: u32, size: u32) -> Self {
        let mut bits = [0u32; 16];
        for (slot, colour) in [palette.base, palette.light, palette.dark, palette.accent]
            .iter()
            .enumerate()
        {
            bits[slot * 4] = colour.r.to_bits();
            bits[slot * 4 + 1] = colour.g.to_bits();
            bits[slot * 4 + 2] = colour.b.to_bits();
            bits[slot * 4 + 3] = colour.a.to_bits();
        }

        Self {
            pattern: pattern.name,
            palette: bits,
            seed,
            size,
        }
    }
}

/// Baked pattern textures, shared for the lifetime of a level load.
///
/// Holds a `TextureHandle` per entry, which is what keeps the texture alive:
/// dropping the last handle frees the GPU resources and its descriptor set, so
/// a cache that stored ids rather than handles would hand out dead textures.
#[derive(Default)]
pub struct TextureCache {
    entries: HashMap<TextureKey, TextureHandle>,
    hits: u32,
    misses: u32,
}

impl TextureCache {
    pub fn new() -> Self {
        Self::default()
    }

    /// The texture for this pattern, baking and uploading it only if no
    /// identical one has been made yet.
    pub fn get_or_bake(
        &mut self,
        textures: &TextureManager,
        pattern: &Pattern,
        palette: &Palette,
        seed: u32,
        size: u32,
    ) -> EngineResult<TextureHandle> {
        let key = TextureKey::new(pattern, palette, seed, size);

        if let Some(existing) = self.entries.get(&key) {
            self.hits += 1;
            return Ok(existing.clone());
        }

        self.misses += 1;
        let pixels = pattern.bake(size, palette, seed);
        let handle = textures.create_from_rgba(size, size, &pixels, true)?;
        self.entries.insert(key, handle.clone());

        Ok(handle)
    }

    /// Textures baked, and requests served from one already baked.
    pub fn stats(&self) -> (u32, u32) {
        (self.misses, self.hits)
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rendering::colour::Colour;
    use crate::rendering::pattern::library;

    fn palette(base: f32) -> Palette {
        Palette::from_base(Colour::new(base, base, base, 1.0), 0.2)
    }

    #[test]
    fn the_same_inputs_give_the_same_key() {
        let a = TextureKey::new(&library::STONE, &palette(0.6), 1, 256);
        let b = TextureKey::new(&library::STONE, &palette(0.6), 1, 256);

        assert_eq!(a, b);
    }

    /// Each input has to actually participate. A key that ignored one would
    /// serve a marble tile where granite was asked for, and only sometimes.
    #[test]
    fn every_input_changes_the_key() {
        let base = TextureKey::new(&library::STONE, &palette(0.6), 1, 256);

        assert_ne!(
            base,
            TextureKey::new(&library::MARBLE, &palette(0.6), 1, 256)
        );
        assert_ne!(
            base,
            TextureKey::new(&library::STONE, &palette(0.7), 1, 256)
        );
        assert_ne!(
            base,
            TextureKey::new(&library::STONE, &palette(0.6), 2, 256)
        );
        assert_ne!(
            base,
            TextureKey::new(&library::STONE, &palette(0.6), 1, 128)
        );
    }

    /// The accent is the slot a pattern uses least, so it is the one a key is
    /// most likely to forget.
    #[test]
    fn a_difference_in_any_palette_slot_changes_the_key() {
        let plain = palette(0.6);
        let accented = plain.with_accent(Colour::new(0.9, 0.1, 0.1, 1.0));

        assert_ne!(
            TextureKey::new(&library::STONE, &plain, 1, 256),
            TextureKey::new(&library::STONE, &accented, 1, 256)
        );
    }

    #[test]
    fn a_fresh_cache_is_empty() {
        let cache = TextureCache::new();
        assert!(cache.is_empty());
        assert_eq!(cache.stats(), (0, 0));
    }
}
