//! Deriving how a surface *looks* from how it *behaves*.
//!
//! The engine exists to show off its physics, so appearance is most useful when
//! it predicts the simulation: a player should be able to see that a thing is
//! slippery, bouncy or heavy before touching it. This maps the three parameters
//! that decide a collision — friction, restitution and density — onto the two
//! the shader understands, roughness and metallic.
//!
//! It is a *default*, not a law. The mapping is a lossy projection of many
//! physics parameters onto few visual ones, and a level is free to author a
//! [`SurfaceFinish`] that disagrees with it. What it buys is that a spawnable
//! which says nothing about its finish still gets one that agrees with its
//! collider, and that the two can never silently drift apart.
//!
//! ```text
//!   friction  ──▶ base roughness      (slippery = sharp, chalky = diffuse)
//!   restitution ─▶ pull towards rubber (bouncy is never a mirror)
//!   density   ──▶ metallic + tighten  (only genuinely metal-dense reads metal)
//! ```

use crate::rendering::material::SurfaceFinish;

/// Friction at which a surface reads as fully diffuse. Above this it cannot get
/// any chalkier on screen, so the extra grip is invisible — chosen at the top of
/// the range the engine's materials actually use rather than at the top of the
/// physically possible range, which would waste most of the visual scale.
const FULLY_DIFFUSE_FRICTION: f32 = 1.2;

/// Roughness of a frictionless surface. Not zero: below about this the specular
/// lobe narrows past a pixel and a mirror ends up reading flatter than a satin
/// finish (see `SUN_SPECULAR_ROUGHNESS_FLOOR` for the same problem on the sun).
const MIRROR_ROUGHNESS: f32 = 0.05;

/// Roughness a perfectly elastic surface is pulled towards, whatever its
/// friction. Bounce reads as a coated, rubbery surface, and with only roughness
/// and metallic to work with, gloss is the only cue available for that — the
/// broad *soft* highlight the direction doc describes is clearcoat's job (§1.3),
/// which does not exist yet.
///
/// It has to be glossy enough to separate a bouncy ball from plain satin wood,
/// which lands around 0.4: anything above ~0.25 here puts the two within a
/// difference no one can see on the bench sheet.
const ELASTIC_GLOSS_ROUGHNESS: f32 = 0.18;

/// Density below which nothing reads as metal, in kg/m³. Stone sits at ~2600,
/// so the threshold is above it: heavy rock should read heavy, not metallic.
const METALLIC_ONSET_DENSITY: f32 = 3000.0;

/// Density at which a surface reads as fully metal, in kg/m³. Steel is ~7800,
/// aluminium ~2700 — the band is deliberately narrow so few materials land in
/// the middle of it, where a half-metal looks like neither.
const METALLIC_FULL_DENSITY: f32 = 7000.0;

/// How much of its roughness a fully metallic surface keeps. Dense materials
/// take a tighter highlight; a metal that stayed as rough as its friction
/// suggests would read as unfinished casting rather than as a dense object.
const METALLIC_ROUGHNESS_RETENTION: f32 = 0.7;

/// The physical parameters an appearance is derived from.
///
/// Deliberately plain scalars rather than a `ColliderMaterial`: friction there
/// can be direction-dependent, and rendering has no business knowing that.
/// Callers resolve a representative coefficient and hand it over.
#[derive(Clone, Copy, Debug)]
pub struct PhysicalSurface {
    /// Coefficient of restitution. 0 is dead, 1 is perfectly elastic.
    pub restitution: f32,

    /// Representative friction coefficient. For an anisotropic material, the
    /// one a player meets most often — usually the floor-contact value.
    pub friction: f32,

    /// Density in kg/m³, as handed to the collider.
    pub density: f32,
}

impl PhysicalSurface {
    pub fn new(restitution: f32, friction: f32, density: f32) -> Self {
        Self {
            restitution,
            friction,
            density,
        }
    }

    /// The finish this material's physics implies.
    pub fn finish(&self) -> SurfaceFinish {
        let metallic = self.metallic();

        SurfaceFinish {
            roughness: self.roughness(metallic),
            metallic,
        }
    }

    /// Metal is a density readout, not a friction one: polished steel and
    /// polished plastic are equally slippery and only one of them is heavy.
    fn metallic(&self) -> f32 {
        let t = inverse_lerp(METALLIC_ONSET_DENSITY, METALLIC_FULL_DENSITY, self.density);
        smoothstep(t)
    }

    fn roughness(&self, metallic: f32) -> f32 {
        let grip = (self.friction / FULLY_DIFFUSE_FRICTION).clamp(0.0, 1.0);
        let from_friction = MIRROR_ROUGHNESS + (1.0 - MIRROR_ROUGHNESS) * grip;

        // Bounce overrides grip rather than averaging with it. Of the two, it
        // is the cue a player acts on — a grippy surface that also launches you
        // must not read as chalk.
        let bounce = self.restitution.clamp(0.0, 1.0);
        let with_bounce = lerp(from_friction, ELASTIC_GLOSS_ROUGHNESS, bounce);

        let density_tightening = lerp(1.0, METALLIC_ROUGHNESS_RETENTION, metallic);
        (with_bounce * density_tightening).clamp(MIRROR_ROUGHNESS, 1.0)
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn inverse_lerp(a: f32, b: f32, value: f32) -> f32 {
    ((value - a) / (b - a)).clamp(0.0, 1.0)
}

fn smoothstep(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Densities and coefficients for the archetypes the mapping is calibrated
    /// against. Kept here rather than in the module so nothing outside the
    /// tests can mistake them for authored defaults.
    const ICE: PhysicalSurface = PhysicalSurface {
        restitution: 0.05,
        friction: 0.05,
        density: 917.0,
    };
    const WOOD: PhysicalSurface = PhysicalSurface {
        restitution: 0.2,
        friction: 0.5,
        density: 500.0,
    };
    const RUBBER: PhysicalSurface = PhysicalSurface {
        restitution: 0.85,
        friction: 0.9,
        density: 1100.0,
    };
    const STONE: PhysicalSurface = PhysicalSurface {
        restitution: 0.1,
        friction: 0.9,
        density: 2600.0,
    };
    const STEEL: PhysicalSurface = PhysicalSurface {
        restitution: 0.4,
        friction: 0.35,
        density: 7800.0,
    };

    #[test]
    fn a_frictionless_surface_reads_as_a_mirror() {
        let finish = ICE.finish();
        assert!(
            finish.roughness < 0.15,
            "ice should be near mirror-sharp, got {}",
            finish.roughness
        );
        assert_eq!(finish.metallic, 0.0);
    }

    #[test]
    fn a_grippy_surface_reads_as_chalk() {
        let chalk = PhysicalSurface::new(0.0, FULLY_DIFFUSE_FRICTION, 2000.0);
        assert_eq!(chalk.finish().roughness, 1.0);
    }

    #[test]
    fn grip_beyond_the_diffuse_point_changes_nothing() {
        let at_limit = PhysicalSurface::new(0.0, FULLY_DIFFUSE_FRICTION, 2000.0);
        let far_beyond = PhysicalSurface::new(0.0, 4.0, 2000.0);
        assert_eq!(at_limit.finish().roughness, far_beyond.finish().roughness);
    }

    #[test]
    fn roughness_rises_with_friction() {
        let mut previous = 0.0;
        for step in 0..=12 {
            let friction = step as f32 * 0.1;
            let roughness = PhysicalSurface::new(0.0, friction, 1000.0)
                .finish()
                .roughness;
            assert!(
                roughness >= previous,
                "roughness fell from {} to {} at friction {}",
                previous,
                roughness,
                friction
            );
            previous = roughness;
        }
    }

    #[test]
    fn bounce_glosses_over_grip() {
        let dead = PhysicalSurface::new(0.0, 1.0, 1000.0).finish().roughness;
        let bouncy = PhysicalSurface::new(0.85, 1.0, 1000.0).finish().roughness;
        assert!(
            bouncy < dead - 0.3,
            "a grippy surface that bounces should still read glossy: {} vs {}",
            dead,
            bouncy
        );
    }

    #[test]
    fn bounce_pulls_towards_gloss_from_either_side() {
        let perfectly_elastic_and_grippy = PhysicalSurface::new(1.0, 1.2, 1000.0);
        let perfectly_elastic_and_slick = PhysicalSurface::new(1.0, 0.0, 1000.0);
        for surface in [perfectly_elastic_and_grippy, perfectly_elastic_and_slick] {
            assert!((surface.finish().roughness - ELASTIC_GLOSS_ROUGHNESS).abs() < 1e-5);
        }
    }

    #[test]
    fn stone_is_heavy_but_not_metal() {
        assert_eq!(STONE.finish().metallic, 0.0);
    }

    #[test]
    fn steel_is_metal_and_takes_a_tighter_highlight() {
        let steel = STEEL.finish();
        assert!(steel.metallic > 0.95, "got {}", steel.metallic);

        let same_but_light = PhysicalSurface {
            density: 500.0,
            ..STEEL
        };
        assert!(steel.roughness < same_but_light.finish().roughness);
    }

    #[test]
    fn metallic_rises_with_density() {
        let mut previous = 0.0;
        for step in 0..=20 {
            let density = step as f32 * 500.0;
            let metallic = PhysicalSurface::new(0.0, 0.5, density).finish().metallic;
            assert!(metallic >= previous, "metallic fell at density {}", density);
            previous = metallic;
        }
    }

    #[test]
    fn the_archetypes_are_visually_distinct() {
        // The whole point of the mapping: two materials that behave differently
        // must not land on the same finish. Roughness differences below this are
        // invisible on the bench sheet.
        let archetypes = [
            ("ice", ICE),
            ("wood", WOOD),
            ("rubber", RUBBER),
            ("stone", STONE),
            ("steel", STEEL),
        ];

        for (i, (name_a, a)) in archetypes.iter().enumerate() {
            for (name_b, b) in archetypes.iter().skip(i + 1) {
                let (fa, fb) = (a.finish(), b.finish());
                let separation =
                    (fa.roughness - fb.roughness).abs() + (fa.metallic - fb.metallic).abs();
                assert!(
                    separation > 0.1,
                    "{} and {} render alike: {:?} vs {:?}",
                    name_a,
                    name_b,
                    fa,
                    fb
                );
            }
        }
    }

    #[test]
    fn every_finish_is_in_range() {
        for restitution in [0.0, 0.5, 1.0] {
            for friction in [0.0, 0.5, 2.0] {
                for density in [0.0, 1000.0, 20000.0] {
                    let finish = PhysicalSurface::new(restitution, friction, density).finish();
                    assert!((MIRROR_ROUGHNESS..=1.0).contains(&finish.roughness));
                    assert!((0.0..=1.0).contains(&finish.metallic));
                }
            }
        }
    }
}
