use nalgebra::Vector4;

/// RGBA colour with values in [0, 1].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Colour {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Colour {
    pub const WHITE: Self = Self {
        r: 1.0,
        g: 1.0,
        b: 1.0,
        a: 1.0,
    };
    pub const BLACK: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    };
    pub const RED: Self = Self {
        r: 1.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    };
    pub const GREEN: Self = Self {
        r: 0.0,
        g: 1.0,
        b: 0.0,
        a: 1.0,
    };
    pub const BLUE: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 1.0,
        a: 1.0,
    };
    pub const YELLOW: Self = Self {
        r: 1.0,
        g: 1.0,
        b: 0.0,
        a: 1.0,
    };

    /// Create a new colour with the given RGBA values.
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Create a new opaque colour with the given RGB values.
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1.0 }
    }

    /// Return a copy with a different alpha value.
    pub const fn with_alpha(self, a: f32) -> Self {
        Self {
            r: self.r,
            g: self.g,
            b: self.b,
            a,
        }
    }

    /// Blend towards `other`, component-wise. `t` is clamped to [0, 1], so
    /// callers can hand over an unbounded ramp without banding past the ends.
    pub fn lerp(self, other: Self, t: f32) -> Self {
        let t = t.clamp(0.0, 1.0);
        Self {
            r: self.r + (other.r - self.r) * t,
            g: self.g + (other.g - self.g) * t,
            b: self.b + (other.b - self.b) * t,
            a: self.a + (other.a - self.a) * t,
        }
    }

    /// Perceptual brightness of this colour (Rec. 709 primaries).
    ///
    /// Matches `luminance()` in shader/tonemap.glsl, which is what the bloom
    /// bright pass thresholds on — so a radiance whose luminance exceeds the
    /// threshold is exactly the radiance that blooms.
    pub fn luminance(self) -> f32 {
        0.2126 * self.r + 0.7152 * self.g + 0.0722 * self.b
    }

    /// Convert to a Vector4 for shader upload.
    pub fn to_vec4(self) -> Vector4<f32> {
        Vector4::new(self.r, self.g, self.b, self.a)
    }
}

impl Default for Colour {
    fn default() -> Self {
        Self::WHITE
    }
}

impl From<Colour> for Vector4<f32> {
    fn from(colour: Colour) -> Self {
        colour.to_vec4()
    }
}

impl From<Vector4<f32>> for Colour {
    fn from(v: Vector4<f32>) -> Self {
        Self::new(v.x, v.y, v.z, v.w)
    }
}
