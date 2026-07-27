use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

use crate::rendering::colour::Colour;

/// A local light source attached to an entity.
///
/// The light sits at the entity's `Position` plus `offset`, so a lamp model can
/// carry its light above its own origin. Contribution falls off with distance
/// and reaches exactly zero at `range`, which is what lets the collector cull
/// and the shader loop bail out without a visible seam.
#[derive(Component, Debug, Clone, Copy)]
#[storage(DenseVecStorage)]
pub struct PointLight {
    /// Linear colour of the emitted light. Alpha is ignored.
    pub colour: Colour,

    /// Brightness the light casts, as luminance.
    ///
    /// Deliberately not a plain multiplier on `colour`, for the same reason as
    /// `Emission::strength` (src/rendering/material.rs): a multiplier makes the
    /// same number mean different brightness at every hue, so retinting a light
    /// toward a dark colour would silently dim what it casts. Expressed as
    /// luminance, `intensity` means the same thing regardless of `colour`.
    ///
    /// The scene target is floating point, so values above 1.0 are preserved
    /// and drive bloom rather than clipping.
    pub intensity: f32,

    /// Distance at which the light's contribution reaches exactly zero.
    /// A hard cutoff, not a falloff scale: the falloff curve itself lives in
    /// `pointAttenuation` in shader/lighting.glsl, since only the GPU evaluates
    /// per-fragment lighting. Note that [`LightCollector`](super::LightCollector)
    /// culls and scores on camera distance and does *not* use that curve.
    pub range: f32,

    /// Light position relative to the entity's `Position`, in world axes.
    pub offset: Vector3<f32>,
}

impl PointLight {
    /// A warm, room-filling lamp.
    ///
    /// `intensity` re-derived for the luminance convention: the old value (4.0)
    /// was a multiplier on `colour` (luminance 0.864), so `4.0 * 0.864 = 3.455`
    /// keeps the same cast light.
    pub const LAMP: Self = Self {
        colour: Colour::rgb(1.0, 0.85, 0.6),
        intensity: 3.455,
        range: 12.0,
        offset: Vector3::new(0.0, 0.0, 0.0),
    };

    /// A small, flickery-scale source — torches, embers, muzzle glow.
    ///
    /// `intensity` re-derived for the luminance convention: the old value (2.0)
    /// was a multiplier on `colour` (luminance 0.545), so `2.0 * 0.545 = 1.091`
    /// keeps the same cast light.
    #[allow(dead_code)]
    pub const EMBER: Self = Self {
        colour: Colour::rgb(1.0, 0.45, 0.15),
        intensity: 1.091,
        range: 6.0,
        offset: Vector3::new(0.0, 0.0, 0.0),
    };

    /// A light of the given colour, intensity and range, at the entity origin.
    pub const fn new(colour: Colour, intensity: f32, range: f32) -> Self {
        Self {
            colour,
            intensity,
            range,
            offset: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    /// Move the light relative to its entity's origin.
    pub const fn with_offset(mut self, offset: Vector3<f32>) -> Self {
        self.offset = offset;
        self
    }

    /// World position of the light for an entity at `entity_position`.
    pub fn world_position(&self, entity_position: Vector3<f32>) -> Vector3<f32> {
        entity_position + self.offset
    }

    /// Scalar the shader multiplies `colour` by to reach `intensity` luminance.
    ///
    /// Mirrors `Emission::radiance_scale` (src/rendering/material.rs): folding
    /// the hue normalisation into a scale rather than into the packed colour
    /// keeps `colour` available at its authored magnitude for anything else
    /// that reads it.
    ///
    /// A colour with no luminance (black) cannot be scaled to any brightness,
    /// so it casts nothing.
    pub fn radiance_scale(&self) -> f32 {
        let luminance = self.colour.luminance();
        if luminance <= f32::EPSILON {
            0.0
        } else {
            self.intensity / luminance
        }
    }
}

impl Default for PointLight {
    fn default() -> Self {
        Self::LAMP
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offset_shifts_world_position() {
        let light =
            PointLight::new(Colour::WHITE, 1.0, 10.0).with_offset(Vector3::new(0.0, 2.0, 0.0));
        let world = light.world_position(Vector3::new(1.0, 0.0, -3.0));
        assert_eq!(world, Vector3::new(1.0, 2.0, -3.0));
    }
}
