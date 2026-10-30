use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::constraint::types::Constraint;
use crate::physics::pipeline::pair::SolverManifold;

use super::super::conditioning::{ManifoldConditioner, ManifoldConditions};
use super::contact_graph::ContactGraph;
use super::statics_gate::StaticsGate;
use super::warm_start_remap::WarmStartRemap;

/// Configuration for shock propagation.
#[derive(Debug, Clone, Copy)]
pub struct ShockPropagationConfig {
    /// Decay factor controlling how much lower bodies are stabilised.
    /// 0.0 = full propagation (lower body immovable during upper solve).
    /// 1.0 = disabled (no mass scaling).
    pub shock_alpha: f32,
    /// Contacts with `|dot(normal, gravity_dir)| < horizontal_threshold` are
    /// considered horizontal: they do not order the graph, and their warm
    /// starts are not remapped. Whether a pair is scaled is the statics
    /// gate's to decide, whatever its normal.
    pub horizontal_threshold: f32,
}

impl Default for ShockPropagationConfig {
    fn default() -> Self {
        Self {
            shock_alpha: 0.3,
            horizontal_threshold: 0.7,
        }
    }
}

/// Shock propagation conditioner: orders manifolds top-down, scales lower
/// bodies heavier where the scaling stays a contact solution, and carries
/// warm starts over to the frame's scales.
///
/// Owns its components, whose internal allocations persist across frames.
pub struct ShockPropagationConditioner {
    config: ShockPropagationConfig,
    graph: ContactGraph,
    gate: StaticsGate,
    remap: WarmStartRemap,
}

impl ShockPropagationConditioner {
    pub fn new(config: ShockPropagationConfig) -> Self {
        Self {
            config,
            graph: ContactGraph::new(),
            gate: StaticsGate::default(),
            remap: WarmStartRemap::default(),
        }
    }
}

impl Default for ShockPropagationConditioner {
    fn default() -> Self {
        Self::new(ShockPropagationConfig::default())
    }
}

impl ManifoldConditioner for ShockPropagationConditioner {
    fn condition(
        &mut self,
        bodies: &Arena<RigidBody>,
        constraints: &Arena<Constraint>,
        manifolds: &mut [SolverManifold],
        gravity: Vector3<f32>,
        dt: f32,
        conditions: &mut ManifoldConditions,
    ) {
        let gravity_dir = gravity
            .try_normalize(1e-6)
            .unwrap_or_else(|| Vector3::new(0.0, -1.0, 0.0));
        self.graph.rebuild(
            bodies,
            constraints,
            manifolds,
            gravity_dir,
            self.config.horizontal_threshold,
        );
        self.graph
            .sort_manifolds(manifolds, self.config.shock_alpha, conditions);
        self.gate
            .apply(bodies, manifolds, &self.graph, gravity, dt, conditions);
        self.remap.apply(
            bodies,
            manifolds,
            &self.graph,
            conditions,
            -gravity_dir,
            self.config.horizontal_threshold,
        );
    }
}
