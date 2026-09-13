//! Shared procedural texture helpers used by multiple spawnables.
//!
//! Extracted from duplicated code across box, table, and capsule spawners.

use crate::utils::noise::fbm_2d_periodic;

/// RGB colour used during texture generation.
#[derive(Clone, Copy)]
pub struct Rgb {
    pub r: f32,
    pub g: f32,
    pub b: f32,
}

impl Rgb {
    pub fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b }
    }

    pub fn scale(self, factor: f32) -> Self {
        Self::new(self.r * factor, self.g * factor, self.b * factor)
    }

    pub fn lerp(self, other: Self, t: f32) -> Self {
        Self::new(
            self.r + (other.r - self.r) * t,
            self.g + (other.g - self.g) * t,
            self.b + (other.b - self.b) * t,
        )
    }

    pub fn write_rgba(self, pixels: &mut Vec<u8>) {
        pixels.push((self.r.clamp(0.0, 1.0) * 255.0) as u8);
        pixels.push((self.g.clamp(0.0, 1.0) * 255.0) as u8);
        pixels.push((self.b.clamp(0.0, 1.0) * 255.0) as u8);
        pixels.push(255);
    }
}

/// A texture's own source of variation, drawn from a seed rather than from the
/// process.
///
/// Procedural textures want variety — a wall of identical crates reads as a
/// tiling error — but the variety has to be a property of the *object*, not of
/// how many other objects happened to be built first. Drawing from the global
/// `rand` means every texture in a level shifts when an unrelated object is
/// added to the file, which is a level that cannot be authored: the platform
/// you tuned to look like a hazard stripe comes back blue.
///
/// So the caller supplies a seed it can derive from something stable — a spawn
/// position, an index within a stack, a constant for equipment that should all
/// match — and the whole texture follows from it.
///
/// The generator is a 32-bit xorshift. It is not statistically remarkable and
/// does not need to be: it decides a hue nudge and a noise seed.
pub struct TextureRng {
    state: u32,
}

impl TextureRng {
    /// A generator seeded by `seed`. Any seed is usable; zero is folded away
    /// rather than left to produce an all-zero stream.
    pub fn new(seed: u32) -> Self {
        Self {
            state: seed ^ 0x9e37_79b9,
        }
    }

    /// The next value in the stream.
    pub fn u32(&mut self) -> u32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        self.state
    }

    /// A value in `[0, 1)`.
    pub fn unit(&mut self) -> f32 {
        self.u32() as f32 / u32::MAX as f32
    }

    /// A value in `[lo, hi)`.
    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + self.unit() * (hi - lo)
    }

    /// One of `n` choices.
    pub fn pick(&mut self, n: u32) -> u32 {
        self.u32() % n.max(1)
    }

    /// A coin flip.
    pub fn flip(&mut self) -> bool {
        self.u32() & 1 == 1
    }
}

/// A [`TextureRng`] seed derived from a world position.
///
/// The natural stable identity for a level object: two crates in different
/// places differ, the same crate looks the same every time the level is
/// loaded, and adding an object elsewhere in the file changes neither. Mix in
/// an `index` to separate several textures baked at one position, such as the
/// blocks of a wall.
pub fn seed_from_position(position: (f32, f32, f32), index: u32) -> u32 {
    let (x, y, z) = position;
    let mut seed = index.wrapping_mul(0x27d4_eb2d);
    for bits in [x.to_bits(), y.to_bits(), z.to_bits()] {
        seed = (seed ^ bits).wrapping_mul(0x85eb_ca6b);
        seed ^= seed >> 13;
    }
    seed
}

/// [`seed_from_position`] for the spawnables authored as a ground position,
/// whose height the terrain decides.
pub fn seed_from_ground(position: (f32, f32), index: u32) -> u32 {
    seed_from_position((position.0, 0.0, position.1), index)
}

/// Convert hue (0–6), saturation, value to [`Rgb`].
pub fn hue_to_rgb(h: f32, s: f32, v: f32) -> Rgb {
    let h = h % 6.0;
    let i = h.floor() as i32;
    let f = h - h.floor();
    let p = v * (1.0 - s);
    let q = v * (1.0 - f * s);
    let t = v * (1.0 - (1.0 - f) * s);
    match i % 6 {
        0 => Rgb::new(v, t, p),
        1 => Rgb::new(q, v, p),
        2 => Rgb::new(p, v, t),
        3 => Rgb::new(p, q, v),
        4 => Rgb::new(t, p, v),
        _ => Rgb::new(v, p, q),
    }
}

/// Subtle vignette darkening at texture edges.
pub fn edge_vignette(u: f32, v: f32) -> f32 {
    let eu = (u - 0.5).abs() * 2.0;
    let ev = (v - 0.5).abs() * 2.0;
    let edge = ((eu.max(ev) - 0.85) / 0.15).clamp(0.0, 1.0);
    1.0 - edge * 0.25
}

/// Smooth border at the midpoint cross (plank divider).
pub fn plank_border(u: f32, v: f32, width: f32) -> f32 {
    let hx = (u - 0.5).abs();
    let hy = (v - 0.5).abs();
    let h = (1.0 - (hx / width).min(1.0)).powi(2);
    let vb = (1.0 - (hy / width).min(1.0)).powi(2);
    (h + vb).min(1.0)
}

/// L-shaped corner bracket intensity.
pub fn corner_bracket(u: f32, v: f32) -> f32 {
    let cu = if u < 0.5 { u } else { 1.0 - u };
    let cv = if v < 0.5 { v } else { 1.0 - v };

    let h_bar = cu < 0.22 && (cv - 0.06).abs() < 0.025;
    let v_bar = cv < 0.22 && (cu - 0.06).abs() < 0.025;

    if h_bar || v_bar {
        1.0
    } else {
        0.0
    }
}

/// Fold line at a given v-coordinate.
pub fn fold_line(v: f32, centre: f32, width: f32) -> f32 {
    let d = (v - centre).abs();
    (1.0 - (d / width).min(1.0)).powi(2)
}

/// Rivet dots near the edges of a panel.
pub fn rivet_pattern(u: f32, v: f32, size: u32) -> f32 {
    let margin = 0.08;
    let near_edge_u = u < margin || u > (1.0 - margin);
    let near_edge_v = v < margin || v > (1.0 - margin);
    if !near_edge_u && !near_edge_v {
        return 0.0;
    }

    let spacing = 0.12;
    let ru = ((u / spacing) + 0.5).fract() - 0.5;
    let rv = ((v / spacing) + 0.5).fract() - 0.5;
    let dist = (ru * ru + rv * rv).sqrt();
    let rivet_radius = 1.5 / size as f32;
    (1.0 - (dist / rivet_radius).min(1.0)).powi(2)
}

/// Sharp crack-like features from thresholded noise.
pub fn crack_pattern(u: f32, v: f32, seed: u32) -> f32 {
    let n = fbm_2d_periodic(u * 12.0, v * 12.0, 4, 0.7, 2.0, seed, Some(12));
    let edge = ((n - 0.48).abs()).min(0.02) / 0.02;
    (1.0 - edge).powi(3)
}

/// Solid border band at a given inset distance.
pub fn border_band(u: f32, v: f32, width: f32) -> f32 {
    let eu = u.min(1.0 - u);
    let ev = v.min(1.0 - v);
    let d = eu.min(ev);
    (1.0 - (d / width).min(1.0)).powi(2)
}

/// Simple integer hash for deterministic per-element variation.
pub fn hash_pair(a: i32, b: i32) -> u32 {
    let mut n = (a as u32)
        .wrapping_mul(374761393)
        .wrapping_add((b as u32).wrapping_mul(668265263));
    n = (n ^ (n >> 13)).wrapping_mul(1274126177);
    n ^ (n >> 16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::Path;

    #[test]
    fn a_seed_is_all_the_stream_depends_on() {
        let draw = |seed| {
            let mut rng = TextureRng::new(seed);
            (0..8).map(|_| rng.u32()).collect::<Vec<_>>()
        };
        assert_eq!(draw(7), draw(7));
        assert_ne!(draw(7), draw(8));
        assert_ne!(draw(0), draw(1), "a zero seed must not collapse the stream");
    }

    #[test]
    fn a_position_is_a_stable_identity() {
        assert_eq!(
            seed_from_position((3.0, 1.5, -2.0), 0),
            seed_from_position((3.0, 1.5, -2.0), 0)
        );
        assert_ne!(
            seed_from_position((3.0, 1.5, -2.0), 0),
            seed_from_position((3.0, 1.5, -2.1), 0)
        );
        assert_ne!(
            seed_from_position((3.0, 1.5, -2.0), 0),
            seed_from_position((3.0, 1.5, -2.0), 1)
        );
        assert_eq!(
            seed_from_ground((3.0, -2.0), 4),
            seed_from_position((3.0, 0.0, -2.0), 4)
        );
    }

    /// The invariant the whole module exists to hold, guarded at the source.
    ///
    /// A procedural texture that draws from the global `rand` is not a function
    /// of the object it belongs to — it is a function of how many textures were
    /// baked before it. The symptom is a level that repaints itself when an
    /// unrelated object is added to the file, which is invisible to every other
    /// test here because nothing else asserts on a rendered colour.
    ///
    /// One global draw put back into any spawnable brings the whole class of
    /// bug back, so the check is on the sources rather than on any one texture.
    #[test]
    fn no_spawnable_draws_from_the_global_rand() {
        let mut offenders = Vec::new();
        for dir in ["src/app/spawnables", "src/app/creatures"] {
            visit(Path::new(dir), &mut offenders);
        }
        assert!(
            offenders.is_empty(),
            "these draw from the global rand instead of a seeded TextureRng: {offenders:?}"
        );
    }

    fn visit(dir: &Path, offenders: &mut Vec<String>) {
        let entries =
            fs::read_dir(dir).expect("spawnable sources are readable from the crate root");
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                visit(&path, offenders);
            } else if path.extension().is_some_and(|e| e == "rs") {
                let source = fs::read_to_string(&path).expect("source file is readable");
                // Spelled in two halves so this file is not its own offender.
                let needle = ["rand", "::random"].concat();
                if source.contains(&needle) {
                    offenders.push(path.display().to_string());
                }
            }
        }
    }
}
