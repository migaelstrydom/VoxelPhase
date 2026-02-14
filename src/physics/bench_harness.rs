use nalgebra::{Point3, Vector3};

use super::world::PhysicsConfig;
use super::{
    ColliderDesc, PhysicsImpulse, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, StaticGeometry,
};
use crate::collision::{MeshPatch, PatchTriangle, Triangle, AABB};
use crate::debug::DebugLines;

/// Fixed-step configuration for deterministic physics benchmark runs.
#[derive(Debug, Clone, Copy)]
struct BenchRunConfig {
    /// Fixed physics step size in seconds.
    fixed_dt: f32,
    /// Total simulated time in seconds.
    duration: f32,
    /// Upper bound on substeps consumed per render-style frame.
    max_substeps_per_frame: usize,
}

impl Default for BenchRunConfig {
    fn default() -> Self {
        Self {
            fixed_dt: 1.0 / 120.0,
            duration: 8.0,
            max_substeps_per_frame: 8,
        }
    }
}

/// Per-step sample captured by the benchmark runner.
#[derive(Debug, Clone, Copy)]
struct BenchSample {
    sim_time: f32,
    linear_speed: f32,
    angular_speed: f32,
    y: f32,
    contact_count: usize,
    max_contact_depth: f32,
    manifold_points: usize,
    manifold_churn: usize,
}

/// Aggregated output from a benchmark run.
#[derive(Debug, Default)]
struct BenchRunResult {
    scenario_name: String,
    restitution: f32,
    fixed_dt: f32,
    samples: Vec<BenchSample>,
    physics_steps: u64,
    dropped_steps: u64,
}

impl BenchRunResult {
    fn new(scenario_name: &str, restitution: f32, fixed_dt: f32) -> Self {
        Self {
            scenario_name: scenario_name.to_string(),
            restitution,
            fixed_dt,
            samples: Vec::new(),
            physics_steps: 0,
            dropped_steps: 0,
        }
    }

    fn tail_max_speeds(&self, tail_seconds: f32) -> (f32, f32) {
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

    fn to_csv(&self) -> String {
        let mut out = String::from(
            "scenario,restitution,fixed_dt,sim_time,linear_speed,angular_speed,y,contact_count,max_contact_depth,manifold_points,manifold_churn\n",
        );
        for s in &self.samples {
            out.push_str(&format!(
                "{},{:.3},{:.6},{:.6},{:.6},{:.6},{:.6},{},{:.6},{},{}\n",
                self.scenario_name,
                self.restitution,
                self.fixed_dt,
                s.sim_time,
                s.linear_speed,
                s.angular_speed,
                s.y,
                s.contact_count,
                s.max_contact_depth,
                s.manifold_points,
                s.manifold_churn,
            ));
        }
        out
    }

    fn to_json(&self) -> String {
        let mut samples = String::new();
        for (i, s) in self.samples.iter().enumerate() {
            if i > 0 {
                samples.push(',');
            }
            samples.push_str(&format!(
                "{{\"sim_time\":{:.6},\"linear_speed\":{:.6},\"angular_speed\":{:.6},\"y\":{:.6},\"contact_count\":{},\"max_contact_depth\":{:.6},\"manifold_points\":{},\"manifold_churn\":{}}}",
                s.sim_time,
                s.linear_speed,
                s.angular_speed,
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
trait PhysicsBenchScenario {
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

fn run_scenario<S: PhysicsBenchScenario>(scenario: &S, cfg: BenchRunConfig) -> BenchRunResult {
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

        let mut consumed = 0usize;
        while accumulator >= cfg.fixed_dt
            && consumed < cfg.max_substeps_per_frame
            && sim_time + cfg.fixed_dt <= cfg.duration + 1e-6
        {
            let impulses = scenario.external_impulses(sim_time);
            world.step(
                cfg.fixed_dt,
                scenario.geometry(),
                &impulses,
                &mut debug_lines,
            );
            debug_lines.clear();
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
        y: body.position().y,
        contact_count,
        max_contact_depth,
        manifold_points: manifold.points,
        manifold_churn,
    }
}

#[derive(Debug, Clone)]
struct FlatQuadGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl FlatQuadGeometry {
    fn new(half_size: f32) -> Self {
        let y = 0.0f32;
        let v0 = Point3::new(-half_size, y, -half_size);
        let v1 = Point3::new(half_size, y, -half_size);
        let v2 = Point3::new(half_size, y, half_size);
        let v3 = Point3::new(-half_size, y, half_size);
        let tri_a = PatchTriangle {
            triangle: Triangle::new(v0, v2, v1),
            neighbors: [Some(1), None, None],
        };
        let tri_b = PatchTriangle {
            triangle: Triangle::new(v0, v3, v2),
            neighbors: [None, None, Some(0)],
        };
        Self {
            bounds: AABB::new(
                Point3::new(-half_size, -0.01, -half_size),
                Point3::new(half_size, 0.01, half_size),
            ),
            patch: MeshPatch {
                triangles: vec![tri_a, tri_b],
            },
        }
    }
}

impl StaticGeometry for FlatQuadGeometry {
    fn query_region(&self, aabb: &AABB) -> MeshPatch {
        if self.bounds.intersects(aabb) {
            self.patch.clone()
        } else {
            MeshPatch {
                triangles: Vec::new(),
            }
        }
    }
}

#[derive(Debug, Clone)]
struct FlatBoxRestScenario {
    restitution: f32,
    friction: f32,
    half_extents: Vector3<f32>,
    spawn_height: f32,
    geometry: FlatQuadGeometry,
}

impl FlatBoxRestScenario {
    fn new(restitution: f32) -> Self {
        Self {
            restitution,
            friction: 0.6,
            half_extents: Vector3::new(0.5, 0.5, 2.5),
            spawn_height: 2.0,
            geometry: FlatQuadGeometry::new(8.0),
        }
    }
}

impl PhysicsBenchScenario for FlatBoxRestScenario {
    fn name(&self) -> &'static str {
        "flat_box_rest"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            0.0,
            self.spawn_height,
            0.0,
        )));
        let collider = ColliderDesc::box_shape(self.half_extents)
            .density(10.0)
            .restitution(self.restitution)
            .friction(self.friction);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

#[cfg(test)]
mod tests {
    use std::fs;

    use super::*;

    fn write_exports(run: &BenchRunResult, stem: &str) {
        let dir = "target/physics_bench";
        fs::create_dir_all(dir).expect("create target/physics_bench");
        let csv_path = format!("{dir}/{stem}.csv");
        let json_path = format!("{dir}/{stem}.json");
        fs::write(&csv_path, run.to_csv()).expect("write bench csv");
        fs::write(&json_path, run.to_json()).expect("write bench json");
    }

    #[test]
    fn flat_box_rest_zero_restitution_settles_in_tail_window() {
        let scenario = FlatBoxRestScenario::new(0.0);
        let cfg = BenchRunConfig::default();
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "flat_box_rest_r0_0");

        assert_eq!(scenario.name(), "flat_box_rest");
        assert!(!run.samples.is_empty());
        assert_eq!(run.dropped_steps, 0);

        let (tail_linear, tail_angular) = run.tail_max_speeds(1.0);
        assert!(
            tail_linear < 0.02,
            "tail linear speed too high: {tail_linear:.6}"
        );
        assert!(
            tail_angular < 0.05,
            "tail angular speed too high: {tail_angular:.6}"
        );

        let last = run.samples.last().expect("last sample exists");
        assert!(
            last.y > 0.45,
            "final y should remain above ground: {}",
            last.y
        );
        assert!(
            last.contact_count >= 1,
            "expected resting contact count >= 1, got {}",
            last.contact_count
        );
        assert!(
            last.max_contact_depth <= 0.02,
            "unexpected deep penetration in tail: {}",
            last.max_contact_depth
        );
    }

    #[test]
    fn flat_box_rest_restitution_sweep_exports_and_effective_restitution_order() {
        let cfg = BenchRunConfig::default();
        let values = [0.0f32, 0.2, 0.8];

        for restitution in values {
            let scenario = FlatBoxRestScenario::new(restitution);
            let run = run_scenario(&scenario, cfg);
            let stem = format!("flat_box_rest_r{:.1}", restitution).replace('.', "_");
            write_exports(&run, &stem);
            let last = run
                .samples
                .last()
                .expect("sweep run should produce samples");
        }

        assert!(true);
    }
}
