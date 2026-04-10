//! Manifold conditioning: reordering and per-manifold metadata computed
//! between contact generation and solving.

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::constraint::types::Constraint;
use crate::physics::pipeline::pair::SolverManifold;

/// Per-manifold data produced by a `ManifoldConditioner`.
///
/// Persisted on `PhysicsWorld` and reused across substeps within a frame.
/// Internal allocations grow to a high-water mark and are never freed.
pub struct ManifoldConditions {
    /// Shock propagation mass-scaling factors, indexed in parallel with the
    /// manifold slice. `(scale_a, scale_b)` where 1.0 means no scaling and
    /// values < 1.0 reduce the body's effective inverse mass.
    pub shock_scales: Vec<(f32, f32)>,
}

impl ManifoldConditions {
    pub fn new() -> Self {
        Self {
            shock_scales: Vec::new(),
        }
    }

    /// Fill with identity (no-op) values for `count` manifolds.
    pub fn fill_identity(&mut self, count: usize) {
        self.shock_scales.clear();
        self.shock_scales.resize(count, (1.0, 1.0));
    }

    /// Look up shock scales for a manifold by index, defaulting to (1.0, 1.0).
    #[inline]
    pub fn shock_scales_for(&self, index: usize) -> (f32, f32) {
        self.shock_scales.get(index).copied().unwrap_or((1.0, 1.0))
    }
}

/// Preprocessor that can reorder manifolds and compute per-manifold metadata
/// (e.g. shock propagation mass scaling) before the solver runs.
///
/// Plugged into `PhysicsWorld` as a `Box<dyn ManifoldConditioner>`. The
/// conditioner runs once per frame after contact generation; the resulting
/// `ManifoldConditions` are reused across all substeps.
pub trait ManifoldConditioner: Send + Sync {
    /// Reorder `manifolds` and populate `conditions` with per-manifold data.
    ///
    /// `gravity_dir` is the normalised gravity vector (typically `(0, -1, 0)`).
    fn condition(
        &mut self,
        bodies: &Arena<RigidBody>,
        constraints: &Arena<Constraint>,
        manifolds: &mut [SolverManifold],
        gravity_dir: Vector3<f32>,
        conditions: &mut ManifoldConditions,
    );
}

/// No-op conditioner: preserves manifold order and sets all shock scales to 1.0.
pub struct IdentityConditioner;

impl ManifoldConditioner for IdentityConditioner {
    fn condition(
        &mut self,
        _bodies: &Arena<RigidBody>,
        _constraints: &Arena<Constraint>,
        manifolds: &mut [SolverManifold],
        _gravity_dir: Vector3<f32>,
        conditions: &mut ManifoldConditions,
    ) {
        conditions.fill_identity(manifolds.len());
    }
}
