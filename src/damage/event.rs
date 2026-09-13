use specs::Entity;

/// What hurt something. Kept separate from the amount so damage sources stay
/// interchangeable and so future armour can resist one kind and not another.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DamageKind {
    /// Explosion overpressure, scaled by distance falloff.
    Blast,
    /// Being on fire, applied continuously while `OnFire` is present.
    Burn,
    /// A creature's close-range blow. Separate from `Impact` because
    /// nothing physical happened: no collision, no velocity change, and
    /// armour against being thrown into a wall should not help here.
    Melee,
    /// A sudden change in velocity: falling, being crushed, being thrown into
    /// a wall. One mechanism covers all three.
    Impact,
}

/// A single pending injury.
#[derive(Debug, Clone, Copy)]
pub struct DamageEvent {
    pub target: Entity,
    pub amount: f32,
    pub kind: DamageKind,
}

/// Frame-scoped queue of damage waiting to be applied.
///
/// Damage sources push here instead of writing `Health` directly. That keeps
/// every source a pure producer — they need no write access to health, cannot
/// disagree about who died first, and a new source (acid, drowning) is a system
/// that pushes events and nothing else. `DamageApplySystem` drains it.
#[derive(Debug, Default)]
pub struct DamageQueue {
    events: Vec<DamageEvent>,
}

impl DamageQueue {
    pub fn push(&mut self, target: Entity, amount: f32, kind: DamageKind) {
        if amount > 0.0 {
            self.events.push(DamageEvent {
                target,
                amount,
                kind,
            });
        }
    }

    /// Take every queued event, leaving the queue empty for the next frame.
    pub fn drain(&mut self) -> Vec<DamageEvent> {
        std::mem::take(&mut self.events)
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use specs::{Builder, World, WorldExt};

    use super::*;

    fn an_entity() -> Entity {
        let mut world = World::new();
        world.create_entity().build()
    }

    #[test]
    fn zero_and_negative_damage_is_dropped() {
        let mut queue = DamageQueue::default();
        let entity = an_entity();
        queue.push(entity, 0.0, DamageKind::Burn);
        queue.push(entity, -5.0, DamageKind::Blast);
        assert!(
            queue.is_empty(),
            "a resistance of zero must not queue a no-op event"
        );
    }

    #[test]
    fn drain_empties_the_queue() {
        let mut queue = DamageQueue::default();
        let entity = an_entity();
        queue.push(entity, 3.0, DamageKind::Impact);
        assert_eq!(queue.drain().len(), 1);
        assert!(
            queue.is_empty(),
            "a drained queue must not replay next frame"
        );
    }
}
