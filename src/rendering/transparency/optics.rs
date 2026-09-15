//! How much light a surface lets through, and how it bends at the interface.
//!
//! The dial a material sets to stop being opaque. It is deliberately two
//! numbers rather than one: an opacity alone produces the grey ghost that every
//! naive transparency looks like, because the thing that reads as *glass* is
//! that a sheet you can see straight through turns into a mirror when you look
//! along it. That second half is the refractive index, and it is the only
//! parameter here that a physicist would recognise.
//!
//! ```text
//!   Transparency ──▶ opacity   ──┐
//!   (ICE, GLASS)  ──▶ ior      ──┴─▶ GpuSurface.optics ──▶ shader coverage
//! ```
//!
//! The shader turns the pair into a per-fragment *coverage*: how much of the
//! pixel this surface claims, rising from `opacity` head-on to fully opaque at
//! the silhouette. See `shader/transparency.glsl`, which is the only place the
//! curve is evaluated.

/// Refractive index of the air a surface is assumed to sit in.
const AIR_IOR: f32 = 1.0;

/// Head-on opacity at which a surface starts casting a shadow.
///
/// A shadow map stores one depth per texel, so a caster's shadow is all or
/// nothing — there is no way to write "this much light got through". The
/// threshold picks which of the two errors a material would rather make.
/// Below it the surface is closer to a pane you look straight through, and a
/// hard black bite behind it reads as a bug; above it the surface is closer
/// to a solid block, and *no* shadow reads as the object floating.
const SHADOW_OPACITY_THRESHOLD: f32 = 0.5;

/// How a surface transmits light.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transparency {
    /// Fraction of the pixel the surface claims when viewed head-on, before
    /// the Fresnel gain at grazing angles. 1.0 is fully opaque and takes the
    /// surface out of the blended pass entirely.
    pub opacity: f32,

    /// Refractive index of the material. Drives how strongly the surface
    /// turns reflective away from head-on — 1.31 for ice, 1.5 for glass,
    /// 2.42 for diamond. Only meaningful when `opacity` is below 1.
    pub ior: f32,
}

impl Transparency {
    /// A surface that lets nothing through. The default, and the only value
    /// that keeps a material in the opaque pass.
    pub const OPAQUE: Self = Self {
        opacity: 1.0,
        ior: 1.5,
    };

    /// Frozen water: you can see through it, but not read through it.
    ///
    /// Well above glass, and deliberately: the two materials differ far more
    /// in how much they let past than in how they bend it, so an ice block at
    /// a window's opacity is a glass block whatever its index says. What the
    /// number stands for is the light scattered by the trapped air and the
    /// fracture planes that [`pattern::ICE`](crate::rendering::pattern::ICE)
    /// draws — the block is cloudy, not tinted.
    pub const ICE: Self = Self {
        opacity: 0.62,
        ior: 1.31,
    };

    /// Window glass.
    pub const GLASS: Self = Self {
        opacity: 0.12,
        ior: 1.5,
    };

    /// A surface at the given head-on opacity and refractive index.
    #[allow(dead_code)]
    pub const fn new(opacity: f32, ior: f32) -> Self {
        Self { opacity, ior }
    }

    /// Whether this surface has to go through the sorted blended pass.
    ///
    /// The question every draw asks, and the reason it is phrased as "is
    /// blended" rather than "is transparent": a surface at opacity 1 is
    /// perfectly well described by this type and still belongs in the opaque
    /// pass, where it is cheaper and needs no sorting.
    pub fn is_blended(&self) -> bool {
        self.opacity < 1.0
    }

    /// Whether this surface is solid enough to be worth a shadow.
    ///
    /// Independent of [`is_blended`](Self::is_blended): a surface can go
    /// through the sorted pass and still occlude the sun. Ice does — it is
    /// cloudy rather than clear, and a block of it standing on the ground
    /// with no shadow under it does not look like it is resting there.
    /// Window glass does not, being nearly all transmission.
    pub fn casts_shadow(&self) -> bool {
        self.opacity >= SHADOW_OPACITY_THRESHOLD
    }

    /// Reflectance at normal incidence, from the Fresnel equations at a
    /// flat air-to-material interface.
    ///
    /// `((n1 - n2) / (n1 + n2))^2` — the same F0 the BRDF's Schlick term
    /// takes, derived here rather than authored so that a material declaring
    /// an index cannot also declare an inconsistent reflectance.
    pub fn reflectance(&self) -> f32 {
        let delta = (AIR_IOR - self.ior) / (AIR_IOR + self.ior);
        delta * delta
    }
}

impl Default for Transparency {
    fn default() -> Self {
        Self::OPAQUE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_opaque_surface_stays_out_of_the_blended_pass() {
        assert!(!Transparency::OPAQUE.is_blended());
        assert!(Transparency::ICE.is_blended());
    }

    /// The textbook figure for a glass window, and the check that the
    /// derivation is the Fresnel one rather than a guess. 1.5 gives 0.04,
    /// which is also where the renderer's `DIELECTRIC_F0` came from.
    #[test]
    fn glass_reflects_four_percent_head_on() {
        assert!((Transparency::GLASS.reflectance() - 0.04).abs() < 0.002);
    }

    /// The pair of materials the threshold has to separate: a cloudy block
    /// throws a shadow, a window does not. If a tuning pass ever moves ice's
    /// opacity below the threshold this fires rather than silently dropping
    /// every ice shadow in the game.
    #[test]
    fn ice_shadows_and_glass_does_not() {
        assert!(Transparency::OPAQUE.casts_shadow());
        assert!(Transparency::ICE.casts_shadow());
        assert!(!Transparency::GLASS.casts_shadow());
    }

    /// Ice bends light less than glass, so it must also reflect less of it.
    /// If these ever came out equal the index would not be reaching the
    /// shader at all.
    #[test]
    fn a_lower_index_reflects_less() {
        assert!(Transparency::ICE.reflectance() < Transparency::GLASS.reflectance());
    }
}
