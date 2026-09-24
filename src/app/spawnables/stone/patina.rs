//! The colour centuries leave on a stone, painted per vertex.
//!
//! A stone's texture is what the stone *is* — its grain, its pores, the
//! hairline cracks in it — and it is the same on every face. What makes a
//! block look old is what happened to each face, and that depends on which
//! way the face points and where on the block it is: rain runs down the sides
//! and leaves streaks, soot and damp collect underneath, lichen takes hold on
//! top, and the arrises the weather scours are paler than the hollows it
//! fills. None of that can live in a tiled texture; all of it can be read off
//! the position and normal of a vertex.
//!
//! The colour is a multiplier on the texture, and a break is painted clean:
//! the inside of a limestone block is the colour the whole block was when it
//! was cut, which is exactly what makes a fresh break read as one.
//!
//! "Up" is the stone frame's +y — the block as it was set. A block that has
//! fallen over carries its streaks sideways, as a real one would.

use nalgebra::{Vector3, Vector4};

use super::surface::Wear;
use crate::utils::noise::fbm_perlin_3d;

/// Warm and cool ends of the stone-to-stone tone variation. Blocks from one
/// quarry still differ, and an arch of identical stones reads as cast.
const WARM: Vector3<f32> = Vector3::new(0.98, 0.89, 0.74);
const COOL: Vector3<f32> = Vector3::new(0.84, 0.83, 0.80);

/// Tone variations per thickness: about one change of tone per block.
const TONE_FREQUENCY: f32 = 0.45;

/// Soot and damp under an overhang.
const GRIME: Vector3<f32> = Vector3::new(0.40, 0.38, 0.35);
/// How far the underside of a block goes towards [`GRIME`].
const UNDERSIDE_GRIME: f32 = 0.5;

/// Rain streaks: fine across, long down the face.
const STREAK_ACROSS: f32 = 7.0;
const STREAK_ALONG: f32 = 0.6;
/// How far a streak goes towards [`GRIME`].
const STREAK_STRENGTH: f32 = 0.45;

/// Crustose lichen: pale grey-green, and the rarer orange.
const LICHEN: Vector3<f32> = Vector3::new(0.86, 0.88, 0.66);
const LICHEN_ORANGE: Vector3<f32> = Vector3::new(1.05, 0.72, 0.38);
/// Lichen patches per thickness.
const LICHEN_FREQUENCY: f32 = 4.0;

/// A worn arris is scoured paler than the face it bounds.
const ARRIS_BLEACH: f32 = 0.06;

/// How much darker the bottom of a hollow is than its rim.
const HOLLOW_SHADE: f32 = 0.3;

/// The inside of the block, as the quarry cut it.
const FRESH: Vector3<f32> = Vector3::new(1.06, 1.02, 0.94);

/// Seeds, apart from each other and from the shape's fields.
const TONE_SEED: u32 = 2029;
const STREAK_SEED: u32 = 2273;
const LICHEN_SEED: u32 = 2551;
const ORANGE_SEED: u32 = 2749;

/// The colour multiplier and sky occlusion of the stone at `p`, whose
/// outward normal is `normal`, worn as `wear` says, in a block `thickness`
/// thick.
pub fn patina(
    p: &Vector3<f32>,
    normal: &Vector3<f32>,
    wear: &Wear,
    thickness: f32,
) -> (Vector4<f32>, f32) {
    let old = weathered(p, normal, wear, thickness);
    let colour = old.lerp(&FRESH, wear.fresh);

    // The bottom of a hollow sees less of the sky than its rim; a break is
    // fresh and flat and sees all of it.
    let hollow_occlusion = 1.0 - 0.3 * smoothstep(0.55, 0.95, wear.hollow);
    let occlusion = hollow_occlusion + (1.0 - hollow_occlusion) * wear.fresh;

    (Vector4::new(colour.x, colour.y, colour.z, 1.0), occlusion)
}

/// The colour of old surface.
fn weathered(p: &Vector3<f32>, normal: &Vector3<f32>, wear: &Wear, thickness: f32) -> Vector3<f32> {
    let at = |frequency: f32| p / thickness * frequency;

    let t = at(TONE_FREQUENCY);
    let tone = 0.5 + 0.5 * fbm_perlin_3d(t.x, t.y, t.z, 2, 0.5, TONE_SEED) * 1.6;
    let mut colour = WARM.lerp(&COOL, tone.clamp(0.0, 1.0));

    // Underneath: soot and damp.
    let underside = (-normal.y).clamp(0.0, 1.0);
    colour = colour.lerp(&colour.component_mul(&GRIME), underside * UNDERSIDE_GRIME);

    // Down the sides: rain streaks, sampled on a lattice stretched along y so
    // each feature is a long drip rather than a blot.
    let side = 1.0 - normal.y.abs();
    let s = p / thickness;
    let streak = fbm_perlin_3d(
        s.x * STREAK_ACROSS,
        s.y * STREAK_ALONG,
        s.z * STREAK_ACROSS,
        3,
        0.5,
        STREAK_SEED,
    );
    let streak = smoothstep(0.0, 0.45, streak) * side;
    colour = colour.lerp(&colour.component_mul(&GRIME), streak * STREAK_STRENGTH);

    // On top, and a little way down the sides: lichen.
    let exposure = (normal.y + 0.3).clamp(0.0, 1.0);
    let l = at(LICHEN_FREQUENCY);
    let lichen = smoothstep(
        0.18,
        0.38,
        fbm_perlin_3d(l.x, l.y, l.z, 3, 0.55, LICHEN_SEED),
    );
    colour = colour.lerp(&colour.component_mul(&LICHEN), lichen * exposure * 0.9);
    let o = at(LICHEN_FREQUENCY * 1.7);
    let orange = smoothstep(
        0.34,
        0.46,
        fbm_perlin_3d(o.x, o.y, o.z, 2, 0.5, ORANGE_SEED),
    );
    colour = colour.lerp(
        &colour.component_mul(&LICHEN_ORANGE),
        orange * exposure * 0.8,
    );

    // Scoured arrises, filled hollows.
    colour *= 1.0 + ARRIS_BLEACH * wear.arris - HOLLOW_SHADE * (wear.hollow - 0.5);
    colour
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn luminance(c: &Vector4<f32>) -> f32 {
        0.2126 * c.x + 0.7152 * c.y + 0.0722 * c.z
    }

    /// Averaged over many points, the underside of a block is darker than
    /// its top: that is where soot and damp collect.
    #[test]
    fn undersides_are_grimier_than_tops() {
        let (mut top, mut bottom) = (0.0, 0.0);
        for i in 0..200 {
            let p = Vector3::new(i as f32 * 0.037, 0.2, i as f32 * 0.051);
            let wear = Wear::default();
            top += luminance(&patina(&p, &Vector3::y(), &wear, 0.5).0);
            bottom += luminance(&patina(&p, &-Vector3::y(), &wear, 0.5).0);
        }
        assert!(bottom < top * 0.85, "top {top} bottom {bottom}");
    }

    /// A break is painted clean, and is paler than the weathered stone
    /// around it — the look that says it just happened.
    #[test]
    fn a_fresh_break_is_paler_than_old_surface() {
        let (mut old, mut fresh) = (0.0, 0.0);
        for i in 0..200 {
            let p = Vector3::new(i as f32 * 0.043, i as f32 * 0.029, 0.1);
            let normal = Vector3::x();
            old += luminance(&patina(&p, &normal, &Wear::default(), 0.5).0);
            let broken = Wear {
                fresh: 1.0,
                ..Wear::default()
            };
            fresh += luminance(&patina(&p, &normal, &broken, 0.5).0);
        }
        assert!(fresh > old * 1.05, "fresh {fresh} old {old}");
    }
}
