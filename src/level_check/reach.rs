//! What the player can actually reach, derived from tuning rather than quoted.
//!
//! Every figure here comes from [`LocomotionConfig`] and the physics gravity at
//! runtime. Retuning the player therefore retunes what levels are validated
//! against, which is the whole point — a hardcoded jump table goes stale the
//! first time anyone touches `jump_speed` and then quietly lies.

use nalgebra::Vector3;

use crate::character::LocomotionConfig;

/// One ballistic launch, as a point mass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JumpArc {
    /// Human name of the move, e.g. `"Sprint jump"`.
    pub name: &'static str,
    /// Horizontal launch speed (m/s).
    pub horizontal_speed: f32,
    /// Vertical launch speed (m/s).
    pub vertical_speed: f32,
    /// Peak height above the launch point (m).
    pub apex: f32,
    /// Time from launch back to launch height (s).
    pub airtime: f32,
    /// Horizontal distance covered over that airtime, flat ground to flat
    /// ground (m).
    pub flat_range: f32,
}

impl JumpArc {
    /// Solve a launch under constant downward acceleration `gravity` (positive
    /// magnitude, m/s²).
    fn ballistic(name: &'static str, horizontal: f32, vertical: f32, gravity: f32) -> Self {
        let apex = vertical * vertical / (2.0 * gravity);
        let airtime = 2.0 * vertical / gravity;
        Self {
            name,
            horizontal_speed: horizontal,
            vertical_speed: vertical,
            apex,
            airtime,
            flat_range: horizontal * airtime,
        }
    }
}

/// The three moves that define what a level may ask of a player.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JumpEnvelope {
    pub standing: JumpArc,
    pub sprint: JumpArc,
    pub long: JumpArc,
}

impl JumpEnvelope {
    /// Derive the envelope from player tuning and gravity.
    ///
    /// `gravity` is the physics world's acceleration vector; only its magnitude
    /// is used.
    pub fn derive(player: &LocomotionConfig, gravity: Vector3<f32>) -> Self {
        let g = gravity.magnitude();
        assert!(g > 0.0, "jump reach is undefined without gravity");

        Self {
            standing: JumpArc::ballistic("Standing jump", player.walk_speed, player.jump_speed, g),
            sprint: JumpArc::ballistic(
                "Sprint jump",
                player.walk_speed * player.sprint_speed_mul,
                player.jump_speed,
                g,
            ),
            long: JumpArc::ballistic(
                "Long jump",
                player.walk_speed * player.long_jump_speed_mul,
                player.jump_speed * player.long_jump_vertical_mul,
                g,
            ),
        }
    }

    pub fn arcs(&self) -> [JumpArc; 3] {
        [self.standing, self.sprint, self.long]
    }

    /// The largest flat gap any move can clear, ignoring safety margin.
    pub fn max_flat_range(&self) -> f32 {
        self.arcs()
            .iter()
            .map(|a| a.flat_range)
            .fold(f32::NEG_INFINITY, f32::max)
    }

    /// The highest ledge any move can reach.
    pub fn max_apex(&self) -> f32 {
        self.arcs()
            .iter()
            .map(|a| a.apex)
            .fold(f32::NEG_INFINITY, f32::max)
    }
}

/// How many collider diameters wide a route has to be to walk along.
///
/// One diameter is the width at which the player merely *fits*; a deck that
/// narrow is a tightrope, and there is no ledge detection to help. Two leaves
/// half a body either side of the centreline, which is the point at which a
/// catwalk stops being a stunt.
const WALKABLE_WIDTH_IN_DIAMETERS: f32 = 2.0;

/// What the player has to be able to stand on, derived from their collider.
///
/// Named for the player's side of the question deliberately. `Footprint` is
/// the ground a *level object* covers, in `level::footprint`; this is how much
/// ground the *player* needs under them, which is the same measurement asked
/// from the other end and would be a confusing thing to share a name with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stance {
    /// The player's collider radius.
    pub radius: f32,
    /// The narrowest deck the player physically fits on.
    pub fits: f32,
    /// The narrowest deck they can be asked to walk along.
    pub walkable: f32,
}

impl Stance {
    pub fn derive(player: &LocomotionConfig) -> Self {
        let diameter = player.collider_radius * 2.0;
        Self {
            radius: player.collider_radius,
            fits: diameter,
            walkable: diameter * WALKABLE_WIDTH_IN_DIAMETERS,
        }
    }
}

/// Caveat printed alongside the envelope, so nobody authors to the limit.
pub const OPTIMISM_CAVEAT: &str =
    "Point-mass figures: perfect input, flat launch and landing, no collider size, \
     ignoring jump_cutoff_factor and air steering. Author gaps well inside them.";

#[cfg(test)]
mod tests {
    use super::*;

    /// The plan quotes these numbers for the shipped defaults. They are a
    /// cross-check on the derivation, not a source: if the player is retuned
    /// this test should be updated from the new tuning, not the derivation bent
    /// back to the old numbers.
    #[test]
    fn derived_reach_matches_hand_computation() {
        let player = LocomotionConfig::default();
        let envelope = JumpEnvelope::derive(&player, Vector3::new(0.0, -9.81, 0.0));

        // Standing: v = 7.0 m/s, g = 9.81 => apex 7²/(2g), airtime 2*7/g,
        // range = walk_speed * airtime.
        let close = |a: f32, b: f32, what: &str| {
            assert!((a - b).abs() < 0.01, "{what}: got {a}, expected {b}");
        };
        close(envelope.standing.apex, 2.497, "standing apex");
        close(envelope.standing.airtime, 1.427, "standing airtime");
        close(envelope.standing.flat_range, 7.135, "standing range");

        // Sprint differs only in horizontal speed (5.0 * 1.6 = 8.0).
        close(envelope.sprint.apex, envelope.standing.apex, "sprint apex");
        close(envelope.sprint.flat_range, 11.417, "sprint range");

        // Long jump: 5.0 * 2.5 = 12.5 horizontal, 7.0 * 0.6 = 4.2 vertical.
        close(envelope.long.apex, 0.899, "long apex");
        close(envelope.long.airtime, 0.856, "long airtime");
        close(envelope.long.flat_range, 10.703, "long range");

        // The sprint jump out-ranges the long jump at these defaults; the long
        // jump's value is its flat arc.
        assert!(envelope.sprint.flat_range > envelope.long.flat_range);
    }

    /// Halving gravity must double apex and quadruple range — the derivation
    /// has to respond to tuning, which a table of constants would not.
    #[test]
    fn reach_scales_with_gravity() {
        let player = LocomotionConfig::default();
        let normal = JumpEnvelope::derive(&player, Vector3::new(0.0, -9.81, 0.0));
        let moon = JumpEnvelope::derive(&player, Vector3::new(0.0, -9.81 / 2.0, 0.0));

        assert!((moon.standing.apex / normal.standing.apex - 2.0).abs() < 1e-4);
        assert!((moon.standing.flat_range / normal.standing.flat_range - 2.0).abs() < 1e-4);
    }
}
