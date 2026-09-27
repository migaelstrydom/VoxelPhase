//! Configuration owned by the narrowphase.
//!
//! Built once per frame from `PhysicsConfig` so contact generation takes a
//! single config reference rather than a train of loose scalars.

/// The span of time one narrowphase pass must cover.
///
/// Contacts are generated once per frame and then solved for every substep of
/// it, so anything the narrowphase predicts has to look this far ahead — not
/// one substep.
#[derive(Debug, Clone, Copy)]
pub struct ContactHorizon {
    /// Length of one substep.
    pub substep_dt: f32,
    /// Substeps that will consume the contacts before the next pass.
    pub substeps: u32,
}

impl ContactHorizon {
    /// Total time until the narrowphase next runs.
    pub fn frame_dt(&self) -> f32 {
        self.substep_dt * self.substeps.max(1) as f32
    }
}

/// Gating rule for speculative contacts.
///
/// Speculative contacts cover the band of speeds that are too fast for a purely
/// discrete narrowphase — the body would travel past the contact margin before
/// contacts are next generated — yet too slow for CCD to engage. Rather than
/// generating a contact where the pair *is*, the narrowphase generates one
/// where it will meet, carrying the gap still to close, and the solver lets the
/// pair close exactly that gap before arresting it.
///
/// Every travel here is measured over the whole [`ContactHorizon`], the unit
/// the discrete narrowphase actually samples in:
/// - below `min_speed` (or below the margin gate) there is nothing to predict,
///   because the discrete manifold already covers where the body will be;
/// - above the travel at which either CCD gate fires, the sweep takes over.
///   The ceiling is computed from those gates rather than tuned beside them, so
///   the two mechanisms meet with no band between them.
#[derive(Debug, Clone, Copy)]
pub struct SpeculativeConfig {
    /// Whether speculative contacts are generated at all.
    pub enabled: bool,
    /// Minimum linear speed for a body to be considered for prediction.
    pub min_speed: f32,
    /// Multiplier applied to `contact_margin` to form the lower travel gate.
    /// Below this, the discrete manifold already brackets the motion.
    pub margin_multiplier: f32,
    /// Mirrors `PhysicsConfig::speculative_relative_speed_gate`: whether a pair
    /// is predicted only when its relative speed outruns the margin, rather
    /// than whenever either body is in the band. See that field for the
    /// tradeoff.
    pub relative_speed_gate: bool,
    /// Mirrors `PhysicsConfig::ccd_threshold`: CCD's per-substep gate, as a
    /// multiple of the collider's bounding radius.
    pub ccd_threshold: f32,
    /// Mirrors `PhysicsConfig::ccd_frame_coverage`: CCD's per-frame gate, as a
    /// multiple of the collider's bounding radius.
    pub ccd_frame_coverage: f32,
}

impl SpeculativeConfig {
    /// Whether a body of the given `radius` moving at `speed` falls inside the
    /// speculative band over `horizon`.
    ///
    /// `contact_margin` is passed rather than stored because the narrowphase
    /// uses it for far more than this gate; see [`NarrowphaseConfig`].
    pub fn admits(
        &self,
        speed: f32,
        horizon: ContactHorizon,
        radius: f32,
        contact_margin: f32,
    ) -> bool {
        self.outruns_margin(speed, horizon, contact_margin)
            && speed * horizon.frame_dt() <= self.ccd_floor(radius, horizon)
    }

    /// Whether `speed` over `horizon` carries further than the discrete
    /// manifold brackets: the band's floor, without its ceiling.
    pub fn outruns_margin(&self, speed: f32, horizon: ContactHorizon, contact_margin: f32) -> bool {
        self.enabled
            && horizon.substep_dt > 0.0
            && speed >= self.min_speed
            && speed * horizon.frame_dt() > contact_margin * self.margin_multiplier
    }

    /// Frame travel at which CCD engages for a collider of `radius`: the lower
    /// of its two gates, the per-substep one restated in frame travel.
    pub fn ccd_floor(&self, radius: f32, horizon: ContactHorizon) -> f32 {
        let per_substep = self.ccd_threshold * horizon.substeps.max(1) as f32;
        radius * per_substep.min(self.ccd_frame_coverage)
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
    /// Whether a pair closes fast enough to be worth predicting.
    ///
    /// With the relative-speed gate on, judged on the pair's relative speed,
    /// not either body's own: two bodies flying the same way close no faster
    /// than two at rest, and the discrete margin brackets them just as well.
    /// With it off, every pair is worth predicting. Whether either body is in
    /// the band at all — and so bounded over its travel, and paired — is
    /// [`admits_speculative`](Self::admits_speculative)'s question.
    pub fn admits_speculative_pair(&self, relative_speed: f32, horizon: ContactHorizon) -> bool {
        !self.speculative.relative_speed_gate
            || self
                .speculative
                .outruns_margin(relative_speed, horizon, self.contact_margin)
    }

    /// Whether a single body falls inside the speculative band.
    pub fn admits_speculative(&self, speed: f32, horizon: ContactHorizon, radius: f32) -> bool {
        self.speculative
            .admits(speed, horizon, radius, self.contact_margin)
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
                relative_speed_gate: true,
                ccd_threshold: 0.5,
                ccd_frame_coverage: 1.5,
            },
        }
    }

    /// A 60 Hz frame of four 1/240 s substeps.
    fn frame() -> ContactHorizon {
        ContactHorizon {
            substep_dt: 1.0 / 240.0,
            substeps: 4,
        }
    }

    #[test]
    fn rejects_when_disabled() {
        let mut c = config();
        c.speculative.enabled = false;
        assert!(!c.admits_speculative(5.0, frame(), 0.5));
    }

    #[test]
    fn rejects_below_min_speed() {
        let c = config();
        assert!(!c.admits_speculative(0.5, frame(), 0.5));
    }

    #[test]
    fn rejects_non_positive_dt() {
        let c = config();
        let horizon = ContactHorizon {
            substep_dt: 0.0,
            substeps: 4,
        };
        assert!(!c.admits_speculative(5.0, horizon, 0.5));
    }

    /// Travel below the margin gate is already covered by the discrete manifold.
    #[test]
    fn rejects_travel_inside_margin_gate() {
        let c = config();
        // margin gate = 0.02 * 2.0 = 0.04; frame travel = 2.0 / 60 = 0.033
        assert!(!c.admits_speculative(2.0, frame(), 0.5));
    }

    /// Travel past where CCD engages belongs to the sweep, not to prediction.
    #[test]
    fn rejects_travel_past_ccd_handoff() {
        let c = config();
        // CCD floor = 0.5 * min(0.5 * 4, 1.5) = 0.75; frame travel = 60 / 60 = 1.0
        assert!(!c.admits_speculative(60.0, frame(), 0.5));
    }

    #[test]
    fn admits_inside_the_band() {
        let c = config();
        // gate = 0.04, ceiling = 0.75, frame travel = 12 / 60 = 0.2
        assert!(c.admits_speculative(12.0, frame(), 0.5));
    }

    /// The band's ceiling is exactly where CCD's gates fire, whichever of the
    /// two is lower for the frame's substep count. Checked against the gate
    /// `ccd::candidate` applies, restated here in its own terms.
    #[test]
    fn ceiling_meets_ccd_with_no_gap() {
        let c = config();
        let radius = 0.2;
        for substeps in 1..=8 {
            let horizon = ContactHorizon {
                substep_dt: 1.0 / 240.0,
                substeps,
            };
            let ccd_engages = |speed: f32| {
                let dt = horizon.substep_dt;
                speed * dt > radius * c.speculative.ccd_threshold
                    || speed * horizon.frame_dt() > radius * c.speculative.ccd_frame_coverage
            };
            for step in 1..2000 {
                let speed = step as f32 * 0.05;
                if speed * horizon.frame_dt() <= c.contact_margin * 2.0 {
                    continue;
                }
                assert!(
                    c.admits_speculative(speed, horizon, radius) || ccd_engages(speed),
                    "{speed} m/s over {substeps} substeps is covered by neither mechanism"
                );
            }
        }
    }

    #[test]
    fn pair_admitted_on_its_relative_speed() {
        let c = config();
        // Closing at 12 m/s, however the speed is split between the two.
        assert!(c.admits_speculative_pair(12.0, frame()));
        // Two bodies flying together at 12 m/s, 1 m/s apart: 0.017 m a frame,
        // inside the 0.04 m the margin brackets.
        assert!(!c.admits_speculative_pair(1.0, frame()));
    }

    /// With the gate off, a pair barely closing is predicted all the same.
    #[test]
    fn pair_admitted_regardless_of_relative_speed_with_the_gate_off() {
        let mut c = config();
        c.speculative.relative_speed_gate = false;
        assert!(c.admits_speculative_pair(1.0, frame()));
        assert!(c.admits_speculative_pair(0.0, frame()));
    }
}
