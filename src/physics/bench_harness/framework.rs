use crate::debug::DebugLines;
use crate::physics::world::PhysicsConfig;
use crate::physics::{PhysicsImpulse, PhysicsWorld, RigidBodyHandle, StaticGeometry};

// ═══════════════════════════════════════════════════════════════════════════
// Framework: run config, sample capture, result aggregation
// ═══════════════════════════════════════════════════════════════════════════

/// Fixed-step configuration for deterministic physics benchmark runs.
#[derive(Debug, Clone, Copy)]
pub(crate) struct BenchRunConfig {
    /// Fixed physics step size in seconds.
    pub fixed_dt: f32,
    /// Total simulated time in seconds.
    pub duration: f32,
    /// Upper bound on substeps consumed per render-style frame.
    pub max_substeps_per_frame: usize,
}

impl Default for BenchRunConfig {
    fn default() -> Self {
        Self {
            fixed_dt: 1.0 / 240.0,
            duration: 8.0,
            max_substeps_per_frame: 8,
        }
    }
}

/// Per-step sample captured by the benchmark runner.
#[derive(Debug, Clone, Copy)]
pub(crate) struct BenchSample {
    pub sim_time: f32,
    pub linear_speed: f32,
    pub angular_speed: f32,
    pub x: f32,
    pub y: f32,
    pub contact_count: usize,
    pub max_contact_depth: f32,
    pub manifold_points: usize,
    pub manifold_churn: usize,
}

/// Aggregated output from a benchmark run.
#[derive(Debug, Default)]
pub(crate) struct BenchRunResult {
    pub scenario_name: String,
    pub restitution: f32,
    pub fixed_dt: f32,
    pub samples: Vec<BenchSample>,
    pub physics_steps: u64,
    pub dropped_steps: u64,
}

impl BenchRunResult {
    pub fn new(scenario_name: &str, restitution: f32, fixed_dt: f32) -> Self {
        Self {
            scenario_name: scenario_name.to_string(),
            restitution,
            fixed_dt,
            samples: Vec::new(),
            physics_steps: 0,
            dropped_steps: 0,
        }
    }

    pub fn tail_max_speeds(&self, tail_seconds: f32) -> (f32, f32) {
        let Some(last) = self.samples.last() else {
            return (0.0, 0.0);
        };
        let start_t = (last.sim_time - tail_seconds).max(0.0);
        let mut max_linear = 0.0f32;
        let mut max_angular = 0.0f32;
        for sample in self.samples.iter().filter(|s| s.sim_time >= start_t) {
            max_linear = max_linear.max(sample.linear_speed);
            max_angular = max_angular.max(sample.angular_speed);
        }
        (max_linear, max_angular)
    }

    pub fn to_csv(&self) -> String {
        let mut out = String::from(
            "scenario,restitution,fixed_dt,sim_time,linear_speed,angular_speed,x,y,contact_count,max_contact_depth,manifold_points,manifold_churn\n",
        );
        for s in &self.samples {
            out.push_str(&format!(
                "{},{:.3},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{:.6},{},{}\n",
                self.scenario_name,
                self.restitution,
                self.fixed_dt,
                s.sim_time,
                s.linear_speed,
                s.angular_speed,
                s.x,
                s.y,
                s.contact_count,
                s.max_contact_depth,
                s.manifold_points,
                s.manifold_churn,
            ));
        }
        out
    }

    pub fn to_json(&self) -> String {
        let mut samples = String::new();
        for (i, s) in self.samples.iter().enumerate() {
            if i > 0 {
                samples.push(',');
            }
            samples.push_str(&format!(
                "{{\"sim_time\":{:.6},\"linear_speed\":{:.6},\"angular_speed\":{:.6},\"x\":{:.6},\"y\":{:.6},\"contact_count\":{},\"max_contact_depth\":{:.6},\"manifold_points\":{},\"manifold_churn\":{}}}",
                s.sim_time,
                s.linear_speed,
                s.angular_speed,
                s.x,
                s.y,
                s.contact_count,
                s.max_contact_depth,
                s.manifold_points,
                s.manifold_churn,
            ));
        }
        format!(
            "{{\"scenario\":\"{}\",\"restitution\":{:.3},\"fixed_dt\":{:.6},\"physics_steps\":{},\"dropped_steps\":{},\"sample_count\":{},\"samples\":[{}]}}",
            self.scenario_name,
            self.restitution,
            self.fixed_dt,
            self.physics_steps,
            self.dropped_steps,
            self.samples.len(),
            samples
        )
    }
}

/// Scenario contract for deterministic benchmark execution.
pub(crate) trait PhysicsBenchScenario {
    fn name(&self) -> &'static str;
    fn restitution(&self) -> f32;
    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        PhysicsWorld::new(config)
    }
    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle;
    fn geometry(&self) -> &dyn StaticGeometry;
    fn frame_dt(&self, _frame_idx: u64) -> f32 {
        1.0 / 60.0
    }
    fn external_impulses(&self, _sim_time: f32) -> Vec<PhysicsImpulse> {
        Vec::new()
    }
}

pub(crate) fn run_scenario<S: PhysicsBenchScenario>(
    scenario: &S,
    cfg: BenchRunConfig,
) -> BenchRunResult {
    let mut world = scenario.build_world();
    let tracked_body = scenario.setup(&mut world);
    let mut debug_lines = DebugLines::default();
    let mut accumulator = 0.0f32;
    let mut sim_time = 0.0f32;
    let mut frame_idx = 0u64;
    let mut out = BenchRunResult::new(scenario.name(), scenario.restitution(), cfg.fixed_dt);

    while sim_time + cfg.fixed_dt <= cfg.duration + 1e-6 {
        accumulator += scenario.frame_dt(frame_idx).max(0.0);
        frame_idx = frame_idx.saturating_add(1);

        if accumulator < cfg.fixed_dt {
            continue;
        }

        // Contact generation once per frame, then N substeps (matches game loop).
        let impulses = scenario.external_impulses(sim_time);
        world.update_contacts(cfg.fixed_dt, scenario.geometry(), &impulses, &mut debug_lines);
        debug_lines.clear();

        let mut consumed = 0usize;
        while accumulator >= cfg.fixed_dt
            && consumed < cfg.max_substeps_per_frame
            && sim_time + cfg.fixed_dt <= cfg.duration + 1e-6
        {
            world.substep(cfg.fixed_dt, scenario.geometry());
            sim_time += cfg.fixed_dt;
            accumulator -= cfg.fixed_dt;
            consumed += 1;
            out.physics_steps = out.physics_steps.saturating_add(1);
            out.samples
                .push(capture_sample(&world, tracked_body, sim_time));
        }

        if consumed == cfg.max_substeps_per_frame && accumulator >= cfg.fixed_dt {
            let dropped = (accumulator / cfg.fixed_dt).floor() as u64;
            if dropped > 0 {
                out.dropped_steps = out.dropped_steps.saturating_add(dropped);
                accumulator -= dropped as f32 * cfg.fixed_dt;
            }
        }
    }

    out
}

fn capture_sample(world: &PhysicsWorld, handle: RigidBodyHandle, sim_time: f32) -> BenchSample {
    let body = world
        .body(handle)
        .expect("benchmark tracked body should exist");
    let mut contact_count = 0usize;
    let mut max_contact_depth = 0.0f32;
    for c in world
        .contact_events()
        .iter()
        .filter(|c| c.body_b == handle || c.body_a == Some(handle))
    {
        contact_count += 1;
        max_contact_depth = max_contact_depth.max(c.depth);
    }
    let manifold = world.manifold_frame_stats();
    let manifold_churn = manifold.point_adds + manifold.point_replacements + manifold.point_pruned;
    BenchSample {
        sim_time,
        linear_speed: body.linear_velocity().magnitude(),
        angular_speed: body.angular_velocity().magnitude(),
        x: body.position().x,
        y: body.position().y,
        contact_count,
        max_contact_depth,
        manifold_points: manifold.points,
        manifold_churn,
    }
}
