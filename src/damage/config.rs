/// Switches for the damage pipeline.
///
/// Death is off by default. The mechanic exists end-to-end — damage sources,
/// health, ragdoll, corpse despawn — but nothing downstream of it does: there
/// is no respawn, no death feedback, and no way back. A dead player is a live
/// entity that has silently lost its `Actuator`, which drops it out of
/// `CharacterControlSystem` and reads, from behind the keyboard, as the
/// controls locking up.
///
/// Turning `deaths_enabled` on is therefore a design decision, not a
/// configuration one. Flip it when there is something for death to lead to.
#[derive(Debug, Clone)]
pub struct DamageConfig {
    /// Whether running out of health marks an entity `Dead`.
    ///
    /// When false, damage still accumulates and health still drops, but the
    /// killing blow is floored instead of fatal — so damage tuning and the
    /// health UI stay honest while the consequence is switched off.
    pub deaths_enabled: bool,
}

impl Default for DamageConfig {
    fn default() -> Self {
        Self {
            deaths_enabled: false,
        }
    }
}
