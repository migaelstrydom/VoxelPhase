//! Configuration owned by the narrowphase.
//!
//! Built once per frame from `PhysicsConfig` so contact generation takes a
//! single config reference rather than a train of loose scalars.

/// Gating rule for speculative contacts.
///
/// Speculative contacts cover the band of speeds that are too fast for a purely
/// discrete narrowphase — the body would step past the contact margin within a
/// substep — yet too slow to justify a CCD sweep. Rather than generating a
/// contact where the body *is*, the narrowphase generates one where the body
/// *will be*, with zero depth, so the solver can arrest it on arrival.
///
/// The band has a floor and a ceiling, and both matter:
/// - below `min_speed` (or below the margin gate) there is nothing to predict,
///   because the discrete manifold already covers where the body will be;
/// - above `ccd_threshold` worth of travel, prediction is no longer enough and
///   the sweep must take over.
#[derive(Debug, Clone, Copy)]
pub struct SpeculativeConfig {
    /// Whether speculative contacts are generated at all.
    pub enabled: bool,
    /// Minimum linear speed for a body to be considered for prediction.
    pub min_speed: f32,
    /// Multiplier applied to `contact_margin` to form the lower travel gate.
    /// Below this, the discrete manifold already brackets the motion.
    pub margin_multiplier: f32,
    /// Upper edge of the band, as a multiple of the collider's bounding radius.
    ///
    /// Mirrors `PhysicsConfig::ccd_threshold` and must track it: the two
    /// mechanisms are defined against each other, speculative contacts covering
    /// exactly the band below where CCD engages. A gap between them is a
    /// tunnelling window.
    pub ccd_threshold: f32,
}

impl SpeculativeConfig {
    /// Whether a body of the given `radius` moving at `speed` falls inside the
    /// speculative band over a step of `dt`.
    ///
    /// `contact_margin` is passed rather than stored because the narrowphase
    /// uses it for far more than this gate; see [`NarrowphaseConfig`].
    pub fn admits(&self, speed: f32, dt: f32, radius: f32, contact_margin: f32) -> bool {
        if !self.enabled || dt <= 0.0 || speed < self.min_speed {
            return false;
        }
        let travel = speed * dt;
        travel > contact_margin * self.margin_multiplier && travel <= radius * self.ccd_threshold
    }
}

/// Everything contact generation needs to know beyond the world state itself.
#[derive(Debug, Clone, Copy)]
pub struct NarrowphaseConfig {
    /// Margin added to collision queries so contacts are detected slightly
    /// before geometric overlap. Contacts inside the margin skin receive
    /// velocity-only correction; real penetrations also get position correction.
    pub contact_margin: f32,
    /// Gating rule for speculative contacts.
    pub speculative: SpeculativeConfig,
}

impl NarrowphaseConfig {
    /// Whether either body of a pair falls inside the speculative band.
    ///
    /// A pair is worth predicting if *either* side is outrunning the discrete
    /// manifold, since it is their relative motion that closes the gap.
    pub fn admits_speculative_pair(
        &self,
        speed_a: f32,
        radius_a: f32,
        speed_b: f32,
        radius_b: f32,
        dt: f32,
    ) -> bool {
        self.admits_speculative(speed_a, dt, radius_a)
            || self.admits_speculative(speed_b, dt, radius_b)
    }

    /// Whether a single body falls inside the speculative band.
    pub fn admits_speculative(&self, speed: f32, dt: f32, radius: f32) -> bool {
        self.speculative
            .admits(speed, dt, radius, self.contact_margin)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> NarrowphaseConfig {
        NarrowphaseConfig {
            contact_margin: 0.02,
            speculative: SpeculativeConfig {
                enabled: true,
                min_speed: 1.0,
                margin_multiplier: 2.0,
                ccd_threshold: 0.5,
            },
        }
    }

    #[test]
    fn rejects_when_disabled() {
        let mut c = config();
        c.speculative.enabled = false;
        assert!(!c.admits_speculative(5.0, 1.0 / 60.0, 0.5));
    }

    #[test]
    fn rejects_below_min_speed() {
        let c = config();
        assert!(!c.admits_speculative(0.5, 1.0 / 60.0, 0.5));
    }

    #[test]
    fn rejects_non_positive_dt() {
        let c = config();
        assert!(!c.admits_speculative(5.0, 0.0, 0.5));
    }

    /// Travel below the margin gate is already covered by the discrete manifold.
    #[test]
    fn rejects_travel_inside_margin_gate() {
        let c = config();
        // margin gate = 0.02 * 2.0 = 0.04; travel = 1.5 * 0.02 = 0.03
        assert!(!c.admits_speculative(1.5, 0.02, 0.5));
    }

    /// Travel past the CCD threshold belongs to the sweep, not to prediction.
    #[test]
    fn rejects_travel_past_ccd_handoff() {
        let c = config();
        // ccd ceiling = 0.5 * 0.5 = 0.25; travel = 20.0 * 0.02 = 0.4
        assert!(!c.admits_speculative(20.0, 0.02, 0.5));
    }

    #[test]
    fn admits_inside_the_band() {
        let c = config();
        // gate = 0.04, ceiling = 0.25, travel = 5.0 * 0.02 = 0.1
        assert!(c.admits_speculative(5.0, 0.02, 0.5));
    }

    #[test]
    fn pair_admitted_when_either_side_is_fast() {
        let c = config();
        assert!(c.admits_speculative_pair(5.0, 0.5, 0.0, 0.5, 0.02));
        assert!(c.admits_speculative_pair(0.0, 0.5, 5.0, 0.5, 0.02));
        assert!(!c.admits_speculative_pair(0.0, 0.5, 0.0, 0.5, 0.02));
    }
}
