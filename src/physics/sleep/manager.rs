use std::collections::HashSet;

use generational_arena::{Arena, Index};
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::constraint::types::Constraint;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;
use crate::physics::sleep::energy::EnergyTracker;
use crate::physics::sleep::islands::IslandBuilder;
use crate::physics::sleep::wake::WakeEvents;

/// Configuration for the sleep system.
#[derive(Debug, Clone, Copy)]
pub struct SleepManagerConfig {
    /// Enable sleeping for dynamic bodies.
    pub enabled: bool,
    /// Kinetic energy threshold below which a body is a sleep candidate.
    pub threshold: f32,
    /// Frames a body must remain below the threshold before sleeping.
    pub delay_frames: u32,
}

impl Default for SleepManagerConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            threshold: 0.05,
            delay_frames: 30,
        }
    }
}

pub struct SleepManager {
    /// Master toggle for sleeping behavior.
    enabled: bool,
    /// Tracks per-body energy and sleep candidacy.
    energy: EnergyTracker,
    /// Builds contact islands for group sleep/wake decisions.
    island_builder: IslandBuilder,
    /// Pending wake events to apply this frame.
    wake_events: WakeEvents,
    /// Set of bodies currently sleeping.
    sleeping: HashSet<RigidBodyHandle>,
}

impl SleepManager {
    pub fn new(config: SleepManagerConfig) -> Self {
        Self {
            enabled: config.enabled,
            energy: EnergyTracker::new(config.threshold, config.delay_frames),
            island_builder: IslandBuilder,
            wake_events: WakeEvents::new(),
            sleeping: HashSet::new(),
        }
    }

    pub fn sync_bodies(&mut self, bodies: &Arena<RigidBody>) {
        let live: HashSet<Index> = bodies.iter().map(|(idx, _)| idx).collect();
        self.sleeping.retain(|handle| live.contains(&handle.0));
        self.energy.retain_indices(&live);
    }

    pub fn is_sleeping(&self, handle: RigidBodyHandle) -> bool {
        self.sleeping.contains(&handle)
    }

    pub fn sleeping_snapshot(&self) -> HashSet<RigidBodyHandle> {
        self.sleeping.clone()
    }

    pub fn wake_body(&mut self, handle: RigidBodyHandle) {
        self.sleeping.remove(&handle);
        self.energy.clear_body(handle);
    }

    pub fn note_kinematic_move(&mut self, handle: RigidBodyHandle) {
        self.wake_events.push(handle);
    }

    pub fn note_contact_wakes(&mut self, manifolds: &[SolverManifold], bodies: &Arena<RigidBody>) {
        if !self.enabled {
            return;
        }
        for manifold in manifolds {
            let handle_b = manifold.header.body_b;
            let Some(body_b) = bodies.get(handle_b.0) else {
                continue;
            };
            let body_a_handle = manifold.header.body_a;
            let body_a = body_a_handle.and_then(|h| bodies.get(h.0));

            let sleeping_a = body_a_handle.map(|h| self.is_sleeping(h)).unwrap_or(false);
            let sleeping_b = self.is_sleeping(handle_b);

            if sleeping_a == sleeping_b {
                continue;
            }

            let other_is_active = if let Some(body_a) = body_a {
                body_a.is_dynamic() || body_a.is_kinematic()
            } else {
                false
            };
            let b_is_active = body_b.is_dynamic() || body_b.is_kinematic();

            if sleeping_a && b_is_active {
                if let Some(handle_a) = body_a_handle {
                    self.wake_events.push(handle_a);
                }
            } else if sleeping_b && other_is_active {
                self.wake_events.push(handle_b);
            }
        }
    }

    pub fn apply_wake_events(&mut self, manifolds: &[SolverManifold], bodies: &Arena<RigidBody>) {
        if !self.enabled {
            self.wake_events.clear();
            return;
        }
        let wake_seeds: HashSet<RigidBodyHandle> =
            self.wake_events.drain().map(|event| event.body).collect();
        if wake_seeds.is_empty() {
            return;
        }
        for handle in &wake_seeds {
            self.sleeping.remove(handle);
            self.energy.clear_body(*handle);
        }

        let islands = self.island_builder.build(bodies, manifolds);
        for island in islands {
            if island
                .bodies
                .iter()
                .any(|handle| wake_seeds.contains(handle))
            {
                for handle in island.bodies {
                    self.sleeping.remove(&handle);
                    self.energy.clear_body(handle);
                }
            }
        }
    }

    pub fn filter_active_manifolds(&self, manifolds: &[SolverManifold]) -> Vec<SolverManifold> {
        if !self.enabled {
            return manifolds.to_vec();
        }

        manifolds
            .iter()
            .filter(|manifold| {
                let body_b_awake = !self.is_sleeping(manifold.header.body_b);
                let body_a_awake = manifold
                    .header
                    .body_a
                    .map(|h| !self.is_sleeping(h))
                    .unwrap_or(false);
                body_a_awake || body_b_awake
            })
            .cloned()
            .collect()
    }

    pub fn update_sleep_states(
        &mut self,
        bodies: &mut Arena<RigidBody>,
        manifolds: &[SolverManifold],
        constraints: &Arena<Constraint>,
    ) {
        if !self.enabled {
            return;
        }

        let mut candidates: HashSet<RigidBodyHandle> = HashSet::new();
        for (idx, body) in bodies.iter() {
            if !body.is_dynamic() {
                continue;
            }
            let handle = RigidBodyHandle(idx);
            if self.is_sleeping(handle) {
                continue;
            }
            // Bodies with active constraints stay awake unconditionally.
            if has_active_constraint(constraints, handle) {
                continue;
            }
            if self.energy.update_body(handle, body) {
                candidates.insert(handle);
            }
        }

        let islands = self.island_builder.build(bodies, manifolds);
        for island in islands {
            if island
                .bodies
                .iter()
                .all(|handle| candidates.contains(handle))
            {
                for handle in island.bodies {
                    if let Some(body) = bodies.get_mut(handle.0) {
                        if body.is_dynamic() {
                            body.set_linear_velocity(Vector3::zeros());
                            body.set_angular_velocity(Vector3::zeros());
                        }
                    }
                    self.sleeping.insert(handle);
                }
            }
        }
    }
}

/// Whether any active constraint references the given body.
///
/// This is a conservative policy: any active constraint prevents sleep. Correct
/// for grab (FollowPoint) and player upright (KeepUpright), but overly
/// conservative for future two-body constraints where both bodies are at rest
/// (e.g. a "glue" constraint). The proper fix is constraint-aware island
/// building — constraints become edges in the contact graph, and entire islands
/// sleep/wake as a unit. See `CONSTRAINT_SYSTEM_PLAN.md` ("Constraint islands
/// for sleeping").
fn has_active_constraint(constraints: &Arena<Constraint>, body: RigidBodyHandle) -> bool {
    constraints
        .iter()
        .any(|(_, c)| c.active && c.kind.references_body(body))
}
