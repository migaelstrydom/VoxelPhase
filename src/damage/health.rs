use specs::{Component, DenseVecStorage};

/// Hit points, and what happens to the body when they run out.
///
/// Anything can carry this — creatures, the player, and eventually breakable
/// props. Nothing reads it except `DamageApplySystem`, which is the only writer
/// too: damage sources push [`DamageEvent`](super::DamageEvent)s rather than
/// mutating health directly, so the order damage lands in is deterministic and
/// a single frame's worth of blast, burn and impact all resolve together.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Health {
    /// Starting and maximum hit points.
    pub max: f32,
    /// Remaining hit points. Reaches exactly 0.0 on death, never negative.
    pub current: f32,
    /// Seconds a corpse lingers before the entity is deleted. `None` leaves it
    /// in the world forever — the player uses this, so death is a state to
    /// recover from rather than a despawn.
    pub corpse_lifetime: Option<f32>,
    /// Multiplier applied to incoming fire damage. Something made of stone
    /// shrugs off a burn that would finish a straw effigy.
    pub burn_resistance: f32,
    /// Speed change (m/s) a body can absorb in one frame before it starts
    /// taking impact damage. Above this, damage scales with the excess.
    pub impact_tolerance: f32,
}

impl Health {
    /// Full health with the given maximum, despawning `corpse_lifetime`
    /// seconds after death.
    pub fn new(max: f32, corpse_lifetime: f32) -> Self {
        Self {
            max,
            current: max,
            corpse_lifetime: Some(corpse_lifetime),
            burn_resistance: 1.0,
            impact_tolerance: 12.0,
        }
    }

    /// Health for something that should never be removed from the world.
    pub fn persistent(max: f32) -> Self {
        Self {
            corpse_lifetime: None,
            ..Self::new(max, 0.0)
        }
    }

    pub fn with_burn_resistance(mut self, resistance: f32) -> Self {
        self.burn_resistance = resistance;
        self
    }

    pub fn with_impact_tolerance(mut self, tolerance: f32) -> Self {
        self.impact_tolerance = tolerance;
        self
    }

    pub fn is_dead(&self) -> bool {
        self.current <= 0.0
    }

    /// Remaining health as a 0..1 fraction. Drives death-throes behaviour
    /// (fleeing at low health) and any future health bar.
    pub fn fraction(&self) -> f32 {
        if self.max <= 0.0 {
            0.0
        } else {
            (self.current / self.max).clamp(0.0, 1.0)
        }
    }

    /// Subtract hit points, clamping at zero. Returns true if this call is
    /// what killed it, so the caller can fire death effects exactly once.
    pub fn apply(&mut self, amount: f32) -> bool {
        if self.is_dead() {
            return false;
        }
        self.current = (self.current - amount).max(0.0);
        self.is_dead()
    }
}

/// Marks an entity whose health has run out.
///
/// Added by `DamageApplySystem` on the frame of death. `DeathSystem` reads it
/// to run the one-time limp-body transition and, later, to despawn the corpse.
#[derive(Component, Debug, Default)]
#[storage(DenseVecStorage)]
pub struct Dead {
    /// Seconds since death.
    pub elapsed: f32,
    /// False until `DeathSystem` has released constraints and stopped the
    /// character driving itself. Keeps that transition to a single frame.
    pub limp: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_reports_the_killing_blow_exactly_once() {
        let mut health = Health::new(10.0, 5.0);
        assert!(!health.apply(4.0), "survivable damage is not a kill");
        assert!(
            health.apply(6.0),
            "the blow that empties health is the kill"
        );
        assert!(
            !health.apply(100.0),
            "further damage to a corpse must not re-trigger death effects"
        );
    }

    #[test]
    fn health_floors_at_zero() {
        let mut health = Health::new(10.0, 5.0);
        health.apply(1000.0);
        assert_eq!(health.current, 0.0);
        assert_eq!(health.fraction(), 0.0);
    }

    #[test]
    fn persistent_health_never_despawns() {
        assert_eq!(Health::persistent(100.0).corpse_lifetime, None);
    }
}
