use rustc_hash::FxHashSet;

use generational_arena::{Arena, Index};
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::constraint::types::Constraint;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;
use crate::physics::sleep::energy::SleepTracker;
use crate::physics::sleep::islands::IslandBuilder;
use crate::physics::sleep::wake::WakeEvents;

/// Configuration for the sleep system.
#[derive(Debug, Clone, Copy)]
pub struct SleepManagerConfig {
    /// Enable sleeping for dynamic bodies.
    pub enabled: bool,
    /// Linear velocity threshold (m/s) below which a body is a sleep candidate.
    pub linear_threshold: f32,
    /// Angular velocity threshold (rad/s) below which a body is a sleep candidate.
    pub angular_threshold: f32,
    /// Frames a body must remain below both thresholds before sleeping.
    pub delay_frames: u32,
}

impl Default for SleepManagerConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            linear_threshold: 0.1,
            angular_threshold: 0.15,
            delay_frames: 30,
        }
    }
}

pub struct SleepManager {
    /// Master toggle for sleeping behavior.
    enabled: bool,
    /// Tracks per-body velocity and sleep candidacy.
    sleep_tracker: SleepTracker,
    /// Builds contact islands for group sleep/wake decisions.
    island_builder: IslandBuilder,
    /// Pending wake events to apply this frame.
    wake_events: WakeEvents,
    /// Set of bodies currently sleeping.
    sleeping: FxHashSet<RigidBodyHandle>,
}

impl SleepManager {
    pub fn new(config: SleepManagerConfig) -> Self {
        Self {
            enabled: config.enabled,
            sleep_tracker: SleepTracker::new(
                config.linear_threshold,
                config.angular_threshold,
                config.delay_frames,
            ),
            island_builder: IslandBuilder,
            wake_events: WakeEvents::new(),
            sleeping: FxHashSet::default(),
        }
    }

    pub fn sync_bodies(&mut self, bodies: &Arena<RigidBody>) {
        let live: FxHashSet<Index> = bodies.iter().map(|(idx, _)| idx).collect();
        self.sleeping.retain(|handle| live.contains(&handle.0));
        self.sleep_tracker.retain_indices(&live);
    }

    pub fn is_sleeping(&self, handle: RigidBodyHandle) -> bool {
        self.sleeping.contains(&handle)
    }

    pub fn sleeping_snapshot(&self) -> FxHashSet<RigidBodyHandle> {
        self.sleeping.clone()
    }

    /// Immediately put a body to sleep.
    pub fn sleep_body(&mut self, handle: RigidBodyHandle) {
        self.sleeping.insert(handle);
        self.sleep_tracker.clear_body(handle);
    }

    pub fn wake_body(&mut self, handle: RigidBodyHandle) {
        self.sleeping.remove(&handle);
        self.sleep_tracker.clear_body(handle);
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
        let wake_seeds: FxHashSet<RigidBodyHandle> =
            self.wake_events.drain().map(|event| event.body).collect();
        if wake_seeds.is_empty() {
            return;
        }
        for handle in &wake_seeds {
            self.sleeping.remove(handle);
            self.sleep_tracker.clear_body(*handle);
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
                    self.sleep_tracker.clear_body(handle);
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
        dt: f32,
    ) {
        if !self.enabled {
            return;
        }

        let mut candidates: FxHashSet<RigidBodyHandle> = FxHashSet::default();
        for (idx, body) in bodies.iter() {
            if !body.is_dynamic() {
                continue;
            }
            let handle = RigidBodyHandle(idx);
            if self.is_sleeping(handle) {
                continue;
            }
            if is_kept_awake(constraints, handle) {
                continue;
            }
            if self.sleep_tracker.update_body(handle, body, dt) {
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

/// Whether an active constraint on the body forbids it to sleep; each kind
/// says for itself, through [`ConstraintKind::permits_sleep`].
///
/// Conservative for two-body constraints where both bodies are at rest (e.g.
/// a "glue" constraint), which keep both awake. The proper fix is
/// constraint-aware island building — constraints become edges in the contact
/// graph, and entire islands sleep/wake as a unit. See
/// `CONSTRAINT_SYSTEM_PLAN.md` ("Constraint islands for sleeping").
///
/// [`ConstraintKind::permits_sleep`]: crate::physics::constraint::ConstraintKind::permits_sleep
fn is_kept_awake(constraints: &Arena<Constraint>, body: RigidBodyHandle) -> bool {
    constraints
        .iter()
        .any(|(_, c)| c.active && c.kind.references_body(body) && !c.kind.permits_sleep())
}
