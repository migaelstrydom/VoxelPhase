use super::immersion::Immersion;

/// How a character takes to the water: when it swims, how fast, and how it
/// lies while it does.
///
/// ```text
///   depth (level to floor)
///     ▲
///     │  swimming          ── swim_depth: too deep to stand, start swimming
///     │  wading               (a swimmer stands once its feet are down here)
///     │
/// ```
///
/// The depth sits just under the depth the character's body floats off
/// its feet at, which its bulk decides: a character that is still on its feet
/// in chest-deep water starts swimming anyway, because a wader that waits for
/// buoyancy to lift it off the floor spends its last stride skating, and one
/// that swims only deeper than that is floated off its feet without being a
/// swimmer. The player's spawner holds its figure to these.
#[derive(Debug, Clone, Copy)]
pub struct SwimConfig {
    /// Still water deeper than this, level to floor, is too deep to stand in,
    /// in metres.
    pub swim_depth: f32,
    /// How far above the still level a swimmer's centre may rise before it has
    /// left the water, in metres. A jump off a wave, or the water draining out
    /// from under it.
    pub leave_height: f32,
    /// How far below the still level a swimmer's centre may be and still
    /// jump out, in metres: a swimmer at the surface can, a diver cannot.
    pub jump_depth: f32,
    /// Upward speed a jump out of the water leaves with, in m/s. The water
    /// takes about 1.5 m/s of it as the body leaves.
    pub jump_speed: f32,
    /// Swimming speed, in m/s, relative to the water.
    pub swim_speed: f32,
    /// Swimming speed with sprint held, in m/s.
    pub sprint_swim_speed: f32,
    /// Stroke acceleration, in m/s². Spent against the actuator's air
    /// allowance, which bounds it, and against the water's drag, which is what
    /// it is sized to beat at `swim_speed`: the drag on a swimmer at 2 m/s is
    /// about 11 m/s², so less than that never reaches the speed asked for.
    pub swim_accel: f32,
    /// Stroke acceleration with sprint held, sized the same way against
    /// `sprint_swim_speed`.
    pub sprint_swim_accel: f32,
    /// How far the body lies over while stroking, in radians from upright.
    pub stroke_pitch: f32,
    /// How far it lies over while treading water.
    pub tread_pitch: f32,
    /// How fast the body is laid over to stroke, in rad/s.
    pub pitch_rate: f32,
    /// How fast it is stood up again, in rad/s. Slower than laying over: the
    /// capsule turns about its centre, so its lower end swings down at this
    /// rate times its half height, and in the shallows that end meets the
    /// floor — at the lay-over rate hard enough to hop the body out of the
    /// water.
    pub stand_rate: f32,
}

impl Default for SwimConfig {
    fn default() -> Self {
        Self {
            swim_depth: 0.6,
            leave_height: 0.45,
            jump_depth: 0.5,
            jump_speed: 6.1,
            swim_speed: 2.0,
            sprint_swim_speed: 3.0,
            swim_accel: 14.0,
            sprint_swim_accel: 24.0,
            stroke_pitch: 80f32.to_radians(),
            tread_pitch: 0.0,
            pitch_rate: 3.0,
            stand_rate: 2.4,
        }
    }
}

impl SwimConfig {
    /// Whether a character not yet swimming should start: in water too deep
    /// to stand in, and in it — up to the chest of a body standing on the
    /// floor, or over the centre of one that is not.
    ///
    /// A body standing on a pier over deep water is in a column that is too
    /// deep, but the water is two metres below its feet. `clearance` is how
    /// far a standing body's centre rides above its feet.
    pub fn takes_to_water(
        &self,
        immersion: &Immersion,
        body_y: f32,
        clearance: f32,
        is_grounded: bool,
    ) -> bool {
        let Some(water) = immersion.water else {
            return false;
        };
        let in_it = if is_grounded {
            water.below_level(body_y - clearance) >= self.swim_depth
        } else {
            water.below_level(body_y) >= 0.0
        };
        water.depth() >= self.swim_depth && in_it
    }

    /// Whether the water at the body is shallow enough to wade: a swimmer
    /// whose feet are down in it stands up.
    pub fn wadeable(&self, immersion: &Immersion) -> bool {
        immersion
            .water
            .map_or(true, |water| water.depth() < self.swim_depth)
    }

    /// Whether a body centred at `body_y` is out of the water: none at the
    /// body, or risen more than `leave_height` above its level (a jump off a
    /// wave, or the water draining out from under it).
    pub fn out_of_water(&self, immersion: &Immersion, body_y: f32) -> bool {
        immersion
            .water
            .map_or(true, |water| -water.below_level(body_y) > self.leave_height)
    }

    /// Whether a swimmer centred at `body_y` is near enough the surface to
    /// jump out.
    pub fn can_jump(&self, immersion: &Immersion, body_y: f32) -> bool {
        immersion
            .water
            .is_some_and(|water| water.below_level(body_y) <= self.jump_depth)
    }

    /// Swimming speed, sprinting or not.
    pub fn speed(&self, sprint: bool) -> f32 {
        if sprint {
            self.sprint_swim_speed
        } else {
            self.swim_speed
        }
    }

    /// Stroke acceleration, sprinting or not.
    pub fn accel(&self, sprint: bool) -> f32 {
        if sprint {
            self.sprint_swim_accel
        } else {
            self.swim_accel
        }
    }

    /// Rate at which the body moves from `current` pitch toward `target`.
    pub fn pitch_rate_toward(&self, current: f32, target: f32) -> f32 {
        if target > current {
            self.pitch_rate
        } else {
            self.stand_rate
        }
    }

    /// Pitch a swimmer holds, stroking or treading.
    pub fn pitch(&self, stroking: bool) -> f32 {
        if stroking {
            self.stroke_pitch
        } else {
            self.tread_pitch
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::WaterAtBody;
    use nalgebra::Vector3;

    /// Sea level at 0, floor `depth` under it.
    fn sea(depth: f32) -> Immersion {
        Immersion::in_water(WaterAtBody {
            level: 0.0,
            surface: 0.0,
            floor: -depth,
            current: Vector3::zeros(),
        })
    }

    const CLEARANCE: f32 = 0.5;

    #[test]
    fn a_pier_over_deep_water_is_not_water() {
        let swim = SwimConfig::default();
        // Standing on a pier two metres over a three-metre sea.
        assert!(!swim.takes_to_water(&sea(3.0), 2.0 + CLEARANCE, CLEARANCE, true));
    }

    #[test]
    fn a_wader_chest_deep_starts_swimming() {
        let swim = SwimConfig::default();
        let depth = swim.swim_depth + 0.05;
        let standing = -depth + CLEARANCE;
        assert!(swim.takes_to_water(&sea(depth), standing, CLEARANCE, true));
        let shallower = swim.swim_depth - 0.1;
        assert!(!swim.takes_to_water(&sea(shallower), -shallower + CLEARANCE, CLEARANCE, true));
    }

    #[test]
    fn a_fall_swims_once_it_is_in_the_water() {
        let swim = SwimConfig::default();
        assert!(
            !swim.takes_to_water(&sea(3.0), 1.0, CLEARANCE, false),
            "still above it"
        );
        assert!(swim.takes_to_water(&sea(3.0), -0.1, CLEARANCE, false));
    }

    #[test]
    fn wading_depth_is_shallower_than_swimming_depth() {
        let swim = SwimConfig::default();
        assert!(swim.wadeable(&sea(swim.swim_depth - 0.02)));
        assert!(
            !swim.wadeable(&sea(3.0)),
            "touching a wall out of its depth"
        );
        assert!(swim.wadeable(&Immersion::dry()));
    }

    #[test]
    fn a_swimmer_out_of_the_water_has_left_it() {
        let swim = SwimConfig::default();
        assert!(swim.out_of_water(&Immersion::dry(), 0.0));
        assert!(swim.out_of_water(&sea(3.0), swim.leave_height + 0.1));
        assert!(!swim.out_of_water(&sea(3.0), -0.1));
    }
}
