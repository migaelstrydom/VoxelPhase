//! How willingly a platform's deck tips under a load, and how fast the tilt
//! that follows dies away.
//!
//! A rigid deck is a lie the player notices: they drop onto it from a height
//! and it does not acknowledge them. The suspension is what makes the platform
//! answer — it tips towards the load and rings back.
//!
//! ```text
//!   DeckSuspension ──tune(half_extents)──► DeckTuning ─┬─► KeepUpright.compliance
//!   (tilt, damping)                                    └─► RigidBody.angular_damping
//! ```
//!
//! The tilt is authored in degrees under a stated reference load, which is the
//! only form the number is meaningful in: a compliance is rad per N·m, and what
//! that buys depends on how wide the deck is. Everything else about the wobble
//! — how fast it rings, how far a jump throws it — follows from the deck's own
//! mass and size, and is not authored at all.

use nalgebra::Vector3;
use serde::Deserialize;

/// Standard gravity, for turning the reference load into a torque.
const GRAVITY: f32 = 9.81;

/// The load the authored tilt is quoted against: the player, whose capsule
/// masses about this much. A platform tuned to tip 2° under a player tips
/// twice as far under two of them — the spring is linear, and the reference is
/// a unit, not a limit.
pub const REFERENCE_LOAD_KG: f32 = 209.0;

/// The solver's position-correction factor for joint constraints, which scales
/// the spring the compliance buys: a row's steady deflection under torque `t`
/// is `compliance · t / beta`, not `compliance · t`.
///
/// Duplicated from `PgsNgsConfig::constraint_position_beta`, which is not
/// reachable through the `ConstraintSolver` trait. `beta_matches_the_solver`
/// fails loudly if the two ever drift apart, rather than letting every platform
/// in the game quietly detune.
const CONSTRAINT_POSITION_BETA: f32 = 0.2;

/// Angular damping for a deck with no suspension. Matches `RigidBodyDesc`'s
/// default: a rigid deck has no ring to damp, so the number only has to be
/// harmless.
const RIGID_ANGULAR_DAMPING: f32 = 0.05;

/// How a deck answers a load standing on it.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct DeckSuspension {
    /// How far the deck tips, in degrees, with [`REFERENCE_LOAD_KG`] standing
    /// on its edge. Zero is a rigid deck.
    ///
    /// This is a steady deflection, not the peak of a landing: a player who
    /// drops onto the edge rather than stepping onto it swings several times
    /// further before settling here.
    pub tilt_degrees: f32,
    /// How hard the ring after a knock is damped, from 0 to 1.
    ///
    /// Zero is not undamped — the attitude rows bleed a deck of this size down
    /// to a twentieth of its peak in about 1.7 s on their own. The dial buys
    /// the range below that: 0.5 gives about 1 s, 0.8 about 0.6 s. Those are
    /// measured on the default deck, and shift with its size and weight, so
    /// treat them as a feel dial rather than a clock.
    pub damping: f32,
}

/// The suspension in the terms the physics engine takes it: a compliance for
/// the attitude constraint and a damping rate for the body.
#[derive(Debug, Clone, Copy)]
pub struct DeckTuning {
    /// Hand to `ConstraintKind::KeepUpright`'s `compliance`, in rad per N·m.
    pub compliance: f32,
    /// Hand to `RigidBodyDesc::angular_damping`.
    pub angular_damping: f32,
}

impl DeckSuspension {
    /// A deck that does not acknowledge what stands on it.
    pub const RIGID: Self = Self {
        tilt_degrees: 0.0,
        damping: 0.0,
    };

    /// Whether this suspension does anything at all.
    pub fn is_rigid(&self) -> bool {
        self.tilt_degrees <= 0.0
    }

    /// Resolve against the deck's half-extents.
    ///
    /// Only the lever arm the reference load stands at is needed — the deck's
    /// longest horizontal half-extent, which is the worst case and the edge a
    /// player actually walks to. The deck's mass and inertia set how fast the
    /// wobble rings, but not how far it sags, so they do not appear here.
    pub fn tune(&self, half_extents: &Vector3<f32>) -> DeckTuning {
        if self.is_rigid() {
            return DeckTuning {
                compliance: 0.0,
                angular_damping: RIGID_ANGULAR_DAMPING,
            };
        }

        let lever_arm = half_extents.x.max(half_extents.z);
        let reference_torque = REFERENCE_LOAD_KG * GRAVITY * lever_arm;
        let tilt = self.tilt_degrees.to_radians();

        DeckTuning {
            compliance: CONSTRAINT_POSITION_BETA * tilt / reference_torque,
            angular_damping: self.damping.clamp(0.0, 1.0),
        }
    }
}

impl Default for DeckSuspension {
    fn default() -> Self {
        Self::RIGID
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::solver::PgsNgsConfig;

    /// The deflection formula is only right for the beta the solver actually
    /// runs. If that moves, every platform's authored tilt becomes a
    /// differently-sized lie, and nothing else would say so.
    #[test]
    fn beta_matches_the_solver() {
        assert_eq!(
            PgsNgsConfig::default().constraint_position_beta,
            CONSTRAINT_POSITION_BETA
        );
    }

    #[test]
    fn a_rigid_deck_asks_for_no_compliance() {
        let tuning = DeckSuspension::RIGID.tune(&Vector3::new(2.0, 0.3, 2.0));
        assert_eq!(tuning.compliance, 0.0);
    }

    /// A wider deck gives the same load a longer lever, so holding the same
    /// tilt takes a stiffer spring.
    #[test]
    fn a_wider_deck_needs_a_stiffer_spring() {
        let suspension = DeckSuspension {
            tilt_degrees: 2.0,
            damping: 0.5,
        };
        let narrow = suspension.tune(&Vector3::new(1.0, 0.3, 1.0));
        let wide = suspension.tune(&Vector3::new(4.0, 0.3, 4.0));
        assert!(
            wide.compliance < narrow.compliance,
            "wide {} should be stiffer than narrow {}",
            wide.compliance,
            narrow.compliance
        );
    }

    /// The lever is the longest horizontal half-extent: a long thin deck is
    /// walked to the far end of, not to the near side.
    #[test]
    fn the_lever_is_the_longest_horizontal_half_extent() {
        let suspension = DeckSuspension {
            tilt_degrees: 2.0,
            damping: 0.0,
        };
        let long_x = suspension.tune(&Vector3::new(4.0, 0.3, 1.0));
        let long_z = suspension.tune(&Vector3::new(1.0, 0.3, 4.0));
        assert_eq!(long_x.compliance, long_z.compliance);
        // Height is not a lever at all.
        let tall = suspension.tune(&Vector3::new(4.0, 9.0, 1.0));
        assert_eq!(tall.compliance, long_x.compliance);
    }

    /// Twice the tilt is twice the give. The spring is linear and the dial
    /// should read that way.
    #[test]
    fn tilt_scales_the_compliance_linearly() {
        let half_extents = Vector3::new(2.0, 0.3, 2.0);
        let soft = DeckSuspension {
            tilt_degrees: 4.0,
            damping: 0.0,
        }
        .tune(&half_extents);
        let stiff = DeckSuspension {
            tilt_degrees: 2.0,
            damping: 0.0,
        }
        .tune(&half_extents);
        assert!((soft.compliance / stiff.compliance - 2.0).abs() < 1e-5);
    }
}
