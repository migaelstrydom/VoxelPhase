//! The colours a substance is made of.
//!
//! Every stone spawnable in the game currently invents its own grey. The temple
//! has one, the menhir has another, the trilithon a third, and they are near
//! enough to look like a mistake and far enough apart to look like an accident.
//! A palette is the one place a substance's colour is decided, so that two
//! things made of granite are made of the *same* granite.
//!
//! Four colours, because that is what the procedural patterns actually consume:
//! a base to sit at, a light and a dark to vary towards, and an accent for the
//! feature that gives the material its character — a vein in marble, a mortar
//! line in brick, a knot in pine.

use crate::rendering::colour::Colour;

/// The colours a procedural texture for this substance draws from.
#[derive(Clone, Copy, Debug)]
pub struct Palette {
    /// The colour the surface reads as overall.
    pub base: Colour,

    /// Where broad variation lightens towards.
    pub light: Colour,

    /// Where broad variation darkens towards — weathering, damp, shadowed pits.
    pub dark: Colour,

    /// The feature colour: veins, mortar, knots, oxide.
    pub accent: Colour,
}

impl Palette {
    /// A palette built from a base colour by lightening and darkening it.
    ///
    /// For substances whose character is in their structure rather than in
    /// their colour. `spread` is the fraction lightened and darkened by; the
    /// accent lands at twice the darkening, which is where a vein or a mortar
    /// line has to sit to read as a separate material rather than as shading.
    pub fn from_base(base: Colour, spread: f32) -> Self {
        Self::from_base_const(base, spread)
    }

    /// [`from_base`](Self::from_base) usable in a constant.
    ///
    /// The library is a table of constants, so the derivation has to be
    /// available at compile time; this is the same function, and `from_base`
    /// exists only so call sites are not littered with the word `const`.
    pub const fn from_base_const(base: Colour, spread: f32) -> Self {
        Self {
            base,
            light: scale(base, 1.0 + spread),
            dark: scale(base, 1.0 - spread),
            accent: scale(base, 1.0 - spread * 2.0),
        }
    }

    /// The same palette with a different accent.
    pub const fn with_accent(mut self, accent: Colour) -> Self {
        self.accent = accent;
        self
    }

    /// The same palette with a different light.
    ///
    /// For the surfaces whose bright slot is not their base turned up: frost
    /// on ice is white, not pale blue, and a spread derived from the base can
    /// never get there without washing the base out with it.
    pub const fn with_light(mut self, light: Colour) -> Self {
        self.light = light;
        self
    }
}

/// Multiply a colour's channels, leaving alpha alone.
const fn scale(colour: Colour, factor: f32) -> Colour {
    Colour {
        r: clamp_unit(colour.r * factor),
        g: clamp_unit(colour.g * factor),
        b: clamp_unit(colour.b * factor),
        a: colour.a,
    }
}

const fn clamp_unit(value: f32) -> f32 {
    if value < 0.0 {
        0.0
    } else if value > 1.0 {
        1.0
    } else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_derived_palette_brackets_its_base() {
        let palette = Palette::from_base(Colour::new(0.5, 0.5, 0.5, 1.0), 0.2);

        assert!(palette.light.r > palette.base.r);
        assert!(palette.dark.r < palette.base.r);
        assert!(palette.accent.r < palette.dark.r);
    }

    /// A palette derived from a near-white base must not produce a light colour
    /// that has silently clipped to the base — that would leave the pattern with
    /// no variation in one direction and no sign of why.
    #[test]
    fn a_bright_base_clamps_rather_than_wrapping() {
        let palette = Palette::from_base(Colour::new(0.95, 0.95, 0.95, 1.0), 0.3);

        assert!(palette.light.r <= 1.0);
        assert!(palette.dark.r < palette.base.r);
    }

    #[test]
    fn an_accent_can_be_something_other_than_a_darker_base() {
        let rust = Colour::new(0.55, 0.25, 0.1, 1.0);
        let palette = Palette::from_base(Colour::new(0.6, 0.6, 0.6, 1.0), 0.15).with_accent(rust);

        assert_eq!(palette.accent.r, rust.r);
        assert_eq!(palette.base.r, 0.6);
    }
}
