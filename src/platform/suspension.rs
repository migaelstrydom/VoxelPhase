//! How willingly a platform's deck tips under a load, and how fast the tilt
//! that follows dies away.
//!
//! A rigid deck is a lie the player notices: they drop onto it from a height
//! and it does not acknowledge them. The suspension is what makes the platform
//! answer — it tips towards the load and rings back.
//!
//! ```text
//!   DeckSuspension ──tune(half_extents)──► DeckTuning ─┬─► KeepUpright.compliance
//!   (tilt, damping, yaw)                               ├─► RigidBody.angular_damping
//!                                                      └─► RigidBody.scale_local_inertia
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

/// A deck that spins around its own yaw axis exactly as its mass and shape
/// say it should. The neutral value of [`DeckSuspension::yaw_resistance`], and
/// the multiplier a rigid deck asks for.
pub const NEUTRAL_YAW_RESISTANCE: f32 = 1.0;

/// The floor on [`DeckSuspension::yaw_resistance`]. Zero would divide by a zero
/// inertia; anything this small already spins on the lightest touch.
const MIN_YAW_RESISTANCE: f32 = 0.01;

/// Angular damping for a deck with no suspension. Matches `RigidBodyDesc`'s
/// default: a rigid deck has no ring to damp, so the number only has to be
/// harmless.
const RIGID_ANGULAR_DAMPING: f32 = 0.05;

/// How a deck answers a load standing on it.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
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
    /// How much harder than its own shape says the deck is to turn about its
    /// up axis. 1 is the honest value; 50 is about what the player capsule
    /// uses, and reads as a deck that simply does not yaw.
    ///
    /// Yaw is the one rotation `KeepUpright` leaves free, so nothing else
    /// opposes it. A light deck is easy to spin — resistance goes as the deck's
    /// mass — and a platform that has been slewed round by a passenger walking
    /// its edge, or by a crate clipping one corner, arrives at its next
    /// waypoint facing somewhere else. This is the dial that buys that back
    /// without welding the deck to the world: it multiplies the yaw term of the
    /// inertia tensor only, so the tilt ring above is untouched and the platform
    /// still gives a little when shoved.
    #[serde(default = "DeckSuspension::default_yaw_resistance")]
    pub yaw_resistance: f32,
}

/// The suspension in the terms the physics engine takes it: a compliance for
/// the attitude constraint and a damping rate for the body.
#[derive(Debug, Clone, Copy)]
pub struct DeckTuning {
    /// Hand to `ConstraintKind::KeepUpright`'s `compliance`, in rad per N·m.
    pub compliance: f32,
    /// Hand to `RigidBodyDesc::angular_damping`.
    pub angular_damping: f32,
    /// Hand to `RigidBody::scale_local_inertia` as the y component, with 1 on
    /// the other two, *after* the collider is attached — the tensor it scales
    /// does not exist until then.
    pub yaw_inertia_scale: f32,
}

impl DeckSuspension {
    /// A deck that does not acknowledge what stands on it.
    pub const RIGID: Self = Self {
        tilt_degrees: 0.0,
        damping: 0.0,
        yaw_resistance: NEUTRAL_YAW_RESISTANCE,
    };

    /// What a moving platform's deck is built with, and the one place the
    /// numbers live — `MovingPlatformDef`'s serde default delegates here rather
    /// than repeating them.
    ///
    /// Enough give that a player landing on the edge visibly throws the deck
    /// (about 5° at the peak of the swing, against the 2° they hold it at
    /// standing still) and enough damping that it has settled by the time they
    /// have crossed it.
    pub const PLATFORM_DECK: Self = Self {
        tilt_degrees: 2.0,
        damping: 0.5,
        yaw_resistance: NEUTRAL_YAW_RESISTANCE,
    };

    /// The neutral yaw resistance, for a level file that does not mention it.
    pub fn default_yaw_resistance() -> f32 {
        NEUTRAL_YAW_RESISTANCE
    }

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
                yaw_inertia_scale: self.yaw_inertia_scale(),
            };
        }

        let lever_arm = half_extents.x.max(half_extents.z);
        let reference_torque = REFERENCE_LOAD_KG * GRAVITY * lever_arm;
        let tilt = self.tilt_degrees.to_radians();

        DeckTuning {
            compliance: CONSTRAINT_POSITION_BETA * tilt / reference_torque,
            angular_damping: self.damping.clamp(0.0, 1.0),
            yaw_inertia_scale: self.yaw_inertia_scale(),
        }
    }

    fn yaw_inertia_scale(&self) -> f32 {
        self.yaw_resistance.max(MIN_YAW_RESISTANCE)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::spawnables::MovingPlatformDef;
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

    /// The platform's deck tuning must be the constant, not a copy of it.
    ///
    /// A second copy of a default is inert in the worst way: it looks
    /// authoritative, it is the one a reader edits, and the game never reads
    /// it. `MovingPlatformDef::default_spin_up` had exactly this bug.
    #[test]
    fn the_platform_default_is_the_named_constant() {
        assert_eq!(
            MovingPlatformDef::default_suspension(),
            DeckSuspension::PLATFORM_DECK
        );
    }

    /// The two named tunings must stay distinguishable: a platform deck that
    /// had quietly become rigid would pass every other test in this file.
    #[test]
    fn a_platform_deck_is_not_a_rigid_one() {
        assert!(DeckSuspension::RIGID.is_rigid());
        assert!(!DeckSuspension::PLATFORM_DECK.is_rigid());
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
            yaw_resistance: DeckSuspension::default_yaw_resistance(),
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
            yaw_resistance: DeckSuspension::default_yaw_resistance(),
        };
        let long_x = suspension.tune(&Vector3::new(4.0, 0.3, 1.0));
        let long_z = suspension.tune(&Vector3::new(1.0, 0.3, 4.0));
        assert_eq!(long_x.compliance, long_z.compliance);
        // Height is not a lever at all.
        let tall = suspension.tune(&Vector3::new(4.0, 9.0, 1.0));
        assert_eq!(tall.compliance, long_x.compliance);
    }

    /// The yaw dial multiplies the deck's own yaw inertia and leaves the tilt
    /// ring alone — that separation is the whole point of it.
    #[test]
    fn yaw_resistance_scales_only_the_yaw_inertia() {
        let half_extents = Vector3::new(2.0, 0.1, 2.0);
        let neutral = DeckSuspension::PLATFORM_DECK.tune(&half_extents);
        let stiff = DeckSuspension {
            yaw_resistance: 20.0,
            ..DeckSuspension::PLATFORM_DECK
        }
        .tune(&half_extents);

        assert_eq!(neutral.yaw_inertia_scale, 1.0);
        assert_eq!(stiff.yaw_inertia_scale, 20.0);
        assert_eq!(stiff.compliance, neutral.compliance);
        assert_eq!(stiff.angular_damping, neutral.angular_damping);
    }

    /// Yaw resistance is independent of whether the deck tips at all, so a
    /// rigid deck still gets to say how hard it is to slew.
    #[test]
    fn a_rigid_deck_still_carries_its_yaw_resistance() {
        let tuning = DeckSuspension {
            yaw_resistance: 6.0,
            ..DeckSuspension::RIGID
        }
        .tune(&Vector3::new(2.0, 0.1, 2.0));
        assert_eq!(tuning.compliance, 0.0);
        assert_eq!(tuning.yaw_inertia_scale, 6.0);
    }

    /// Zero would invert to an infinite inverse inertia and spin the deck out
    /// of the world on the first contact.
    #[test]
    fn yaw_resistance_never_reaches_zero() {
        let tuning = DeckSuspension {
            yaw_resistance: 0.0,
            ..DeckSuspension::PLATFORM_DECK
        }
        .tune(&Vector3::new(2.0, 0.1, 2.0));
        assert!(tuning.yaw_inertia_scale >= MIN_YAW_RESISTANCE);
    }

    /// The dial has to survive the level file, and a level written before it
    /// existed has to keep loading.
    #[test]
    fn the_level_file_can_author_the_yaw_resistance() {
        let authored: DeckSuspension =
            ron::from_str("(tilt_degrees: 2.0, damping: 0.5, yaw_resistance: 12.0)").unwrap();
        assert_eq!(authored.yaw_resistance, 12.0);

        let silent: DeckSuspension = ron::from_str("(tilt_degrees: 2.0, damping: 0.5)").unwrap();
        assert_eq!(silent.yaw_resistance, NEUTRAL_YAW_RESISTANCE);
    }

    /// Twice the tilt is twice the give. The spring is linear and the dial
    /// should read that way.
    #[test]
    fn tilt_scales_the_compliance_linearly() {
        let half_extents = Vector3::new(2.0, 0.3, 2.0);
        let soft = DeckSuspension {
            tilt_degrees: 4.0,
            damping: 0.0,
            yaw_resistance: DeckSuspension::default_yaw_resistance(),
        }
        .tune(&half_extents);
        let stiff = DeckSuspension {
            tilt_degrees: 2.0,
            damping: 0.0,
            yaw_resistance: DeckSuspension::default_yaw_resistance(),
        }
        .tune(&half_extents);
        assert!((soft.compliance / stiff.compliance - 2.0).abs() < 1e-5);
    }
}
