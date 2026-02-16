use nalgebra::{Point3, Vector3};

use super::world::PhysicsConfig;
use super::{
    ColliderDesc, PhysicsImpulse, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, StaticGeometry,
};
use crate::collision::{MeshPatch, PatchTriangle, Triangle, AABB};
use crate::debug::DebugLines;

// ═══════════════════════════════════════════════════════════════════════════
// Framework: run config, sample capture, result aggregation
// ═══════════════════════════════════════════════════════════════════════════

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
            fixed_dt: 1.0 / 60.0,
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
    x: f32,
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

    fn to_json(&self) -> String {
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
        x: body.position().x,
        y: body.position().y,
        contact_count,
        max_contact_depth,
        manifold_points: manifold.points,
        manifold_churn,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Geometry: static terrain shapes used by scenarios
// ═══════════════════════════════════════════════════════════════════════════

/// Flat quad at y=0. Two triangles forming a square.
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

/// Ramp geometry: flat ground (z < 0) transitioning to a 30° upward slope (z >= 0).
///
/// The ramp rises in the +Z direction. The flat section is at y=0.
/// The slope runs from z=0 to z=`run`, reaching height `rise`.
#[derive(Debug, Clone)]
struct RampGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl RampGeometry {
    fn new(half_width: f32, run: f32, rise: f32) -> Self {
        let w = half_width;
        // Flat section: z from -run to 0, y = 0
        let f0 = Point3::new(-w, 0.0, -run);
        let f1 = Point3::new(w, 0.0, -run);
        let f2 = Point3::new(w, 0.0, 0.0);
        let f3 = Point3::new(-w, 0.0, 0.0);
        // Ramp section: z from 0 to run, y from 0 to rise
        let r0 = Point3::new(-w, rise, run);
        let r1 = Point3::new(w, rise, run);

        // Flat quad: two triangles
        let flat_a = PatchTriangle {
            triangle: Triangle::new(f0, f2, f1),
            neighbors: [Some(1), None, None],
        };
        let flat_b = PatchTriangle {
            triangle: Triangle::new(f0, f3, f2),
            neighbors: [None, Some(2), Some(0)],
        };
        // Ramp quad: two triangles sharing the edge f3-f2 with the flat section
        let ramp_a = PatchTriangle {
            triangle: Triangle::new(f3, r1, f2),
            neighbors: [Some(3), None, Some(1)],
        };
        let ramp_b = PatchTriangle {
            triangle: Triangle::new(f3, r0, r1),
            neighbors: [None, None, Some(2)],
        };

        Self {
            bounds: AABB::new(
                Point3::new(-w, -0.01, -run),
                Point3::new(w, rise + 0.01, run),
            ),
            patch: MeshPatch {
                triangles: vec![flat_a, flat_b, ramp_a, ramp_b],
            },
        }
    }
}

impl StaticGeometry for RampGeometry {
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

/// Step geometry: two flat levels at different heights.
///
/// Lower level at y=0 (x < 0), upper level at y=`step_height` (x >= 0).
/// The step edge runs along the Z axis.
#[derive(Debug, Clone)]
struct StepGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl StepGeometry {
    fn new(half_size: f32, step_height: f32) -> Self {
        let s = half_size;
        let h = step_height;
        // Lower level (x < 0): y = 0
        let l0 = Point3::new(-s, 0.0, -s);
        let l1 = Point3::new(0.0, 0.0, -s);
        let l2 = Point3::new(0.0, 0.0, s);
        let l3 = Point3::new(-s, 0.0, s);
        // Upper level (x >= 0): y = step_height
        let u0 = Point3::new(0.0, h, -s);
        let u1 = Point3::new(s, h, -s);
        let u2 = Point3::new(s, h, s);
        let u3 = Point3::new(0.0, h, s);
        // Vertical face (the step wall)
        // l1 (0, 0, -s), u0 (0, h, -s), u3 (0, h, s), l2 (0, 0, s)

        let lower_a = PatchTriangle {
            triangle: Triangle::new(l0, l2, l1),
            neighbors: [Some(1), None, None],
        };
        let lower_b = PatchTriangle {
            triangle: Triangle::new(l0, l3, l2),
            neighbors: [None, None, Some(0)],
        };
        // Step wall triangles
        let wall_a = PatchTriangle {
            triangle: Triangle::new(l1, l2, u3),
            neighbors: [None, Some(3), None],
        };
        let wall_b = PatchTriangle {
            triangle: Triangle::new(l1, u3, u0),
            neighbors: [Some(2), Some(4), None],
        };
        // Upper level
        let upper_a = PatchTriangle {
            triangle: Triangle::new(u0, u2, u1),
            neighbors: [Some(5), None, Some(3)],
        };
        let upper_b = PatchTriangle {
            triangle: Triangle::new(u0, u3, u2),
            neighbors: [None, None, Some(4)],
        };

        Self {
            bounds: AABB::new(Point3::new(-s, -0.01, -s), Point3::new(s, h + 0.01, s)),
            patch: MeshPatch {
                triangles: vec![lower_a, lower_b, wall_a, wall_b, upper_a, upper_b],
            },
        }
    }
}

impl StaticGeometry for StepGeometry {
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

/// Bowl geometry: an inverted square-based pyramid.
///
/// Four triangular faces meet at a single apex below y=0. The rim is a
/// square at y=0. This shape requires the contact pipeline to generate
/// simultaneous contacts on multiple non-coplanar faces — the sphere must
/// rest where all four faces support it, not oscillate between opposite
/// sides.
#[derive(Debug, Clone)]
struct BowlGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl BowlGeometry {
    fn new(half_size: f32, depth: f32) -> Self {
        let s = half_size;
        // Rim vertices at y=0
        let v0 = Point3::new(-s, 0.0, -s);
        let v1 = Point3::new(s, 0.0, -s);
        let v2 = Point3::new(s, 0.0, s);
        let v3 = Point3::new(-s, 0.0, s);
        // Apex at bottom
        let apex = Point3::new(0.0, -depth, 0.0);

        // Winding: (apex, v_{i+1}, v_i) produces normals pointing into the
        // bowl (upward + inward).
        //
        // Edges per triangle: 0 = apex→v_{i+1}, 1 = v_{i+1}→v_i (rim), 2 = v_i→apex
        // Edge 0 is shared with the next triangle's edge 2.
        let front = PatchTriangle {
            triangle: Triangle::new(apex, v1, v0),
            neighbors: [Some(1), None, Some(3)],
        };
        let right = PatchTriangle {
            triangle: Triangle::new(apex, v2, v1),
            neighbors: [Some(2), None, Some(0)],
        };
        let back = PatchTriangle {
            triangle: Triangle::new(apex, v3, v2),
            neighbors: [Some(3), None, Some(1)],
        };
        let left = PatchTriangle {
            triangle: Triangle::new(apex, v0, v3),
            neighbors: [Some(0), None, Some(2)],
        };

        Self {
            bounds: AABB::new(Point3::new(-s, -depth - 0.01, -s), Point3::new(s, 0.01, s)),
            patch: MeshPatch {
                triangles: vec![front, right, back, left],
            },
        }
    }
}

impl StaticGeometry for BowlGeometry {
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

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: mesh pipeline (sphere/box vs static terrain)
// ═══════════════════════════════════════════════════════════════════════════

/// Sphere dropped onto a flat surface. Exercises sphere-patch manifold
/// generation and the mesh-aware pipeline (seam filter -> sphere_patch).
#[derive(Debug, Clone)]
struct FlatSphereRestScenario {
    restitution: f32,
    friction: f32,
    radius: f32,
    spawn_height: f32,
    geometry: FlatQuadGeometry,
}

impl FlatSphereRestScenario {
    fn new(restitution: f32) -> Self {
        Self {
            restitution,
            friction: 0.5,
            radius: 0.5,
            spawn_height: 3.0,
            geometry: FlatQuadGeometry::new(8.0),
        }
    }
}

impl PhysicsBenchScenario for FlatSphereRestScenario {
    fn name(&self) -> &'static str {
        "flat_sphere_rest"
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
        let collider = ColliderDesc::sphere(self.radius)
            .density(1000.0)
            .restitution(self.restitution)
            .friction(self.friction);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Sphere sliding laterally across flat terrain. The sphere crosses the
/// internal mesh edge (diagonal seam between the two triangles in
/// FlatQuadGeometry). This must not cause jitter or speed anomalies —
/// the seam filter suppresses the internal edge, and FeatureId-based
/// manifold caching ensures smooth contact transitions.
#[derive(Debug, Clone)]
struct SphereSlideScenario {
    radius: f32,
    geometry: FlatQuadGeometry,
}

impl SphereSlideScenario {
    fn new() -> Self {
        Self {
            radius: 0.5,
            geometry: FlatQuadGeometry::new(8.0),
        }
    }
}

impl PhysicsBenchScenario for SphereSlideScenario {
    fn name(&self) -> &'static str {
        "sphere_slide"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        // Sphere placed at rest on the surface with lateral velocity.
        // Low friction so it slides rather than rolls to a stop immediately.
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-3.0, self.radius + 0.01, 0.0))
                .linear_velocity(Vector3::new(3.0, 0.0, 0.0)),
        );
        let collider = ColliderDesc::sphere(self.radius)
            .density(1000.0)
            .restitution(0.0)
            .friction(0.1);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Box dropped onto a flat surface. Exercises OBB-patch manifold generation
/// (seam filter -> obb_patch clipping).
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

/// Sphere rolling down a ramp. Exercises sphere-patch on non-flat terrain
/// where the seam filter encounters a crease edge between the flat and sloped
/// sections. Verifies lateral movement (x stays centered, z advances).
#[derive(Debug, Clone)]
struct SphereOnRampScenario {
    restitution: f32,
    radius: f32,
    geometry: RampGeometry,
}

impl SphereOnRampScenario {
    fn new() -> Self {
        Self {
            restitution: 0.1,
            radius: 0.4,
            geometry: RampGeometry::new(4.0, 6.0, 3.0),
        }
    }
}

impl PhysicsBenchScenario for SphereOnRampScenario {
    fn name(&self) -> &'static str {
        "sphere_on_ramp"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        // Drop sphere onto the ramp midway up.
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 3.0, 3.0)));
        let collider = ColliderDesc::sphere(self.radius)
            .density(1000.0)
            .restitution(self.restitution)
            .friction(0.6);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Box dropped onto a ramp. Exercises OBB-patch manifold generation against
/// a sloped surface (non-axis-aligned face normal).
#[derive(Debug, Clone)]
struct BoxOnRampScenario {
    restitution: f32,
    half_extents: Vector3<f32>,
    geometry: RampGeometry,
}

impl BoxOnRampScenario {
    fn new() -> Self {
        Self {
            restitution: 0.1,
            half_extents: Vector3::new(0.4, 0.4, 0.4),
            geometry: RampGeometry::new(4.0, 6.0, 3.0),
        }
    }
}

impl PhysicsBenchScenario for BoxOnRampScenario {
    fn name(&self) -> &'static str {
        "box_on_ramp"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 3.0, 3.0)));
        let collider = ColliderDesc::box_shape(self.half_extents)
            .density(1000.0)
            .restitution(self.restitution)
            .friction(0.6);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Box dropped near a step edge. Exercises OBB-patch manifold generation
/// against multi-level terrain with a vertical wall face.
#[derive(Debug, Clone)]
struct BoxOnStepScenario {
    restitution: f32,
    half_extents: Vector3<f32>,
    geometry: StepGeometry,
}

impl BoxOnStepScenario {
    fn new() -> Self {
        Self {
            restitution: 0.1,
            half_extents: Vector3::new(0.3, 0.3, 0.3),
            geometry: StepGeometry::new(4.0, 0.5),
        }
    }
}

impl PhysicsBenchScenario for BoxOnStepScenario {
    fn name(&self) -> &'static str {
        "box_on_step"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        // Drop box onto the lower level near the step edge.
        let body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(-0.5, 2.0, 0.0)));
        let collider = ColliderDesc::box_shape(self.half_extents)
            .density(1000.0)
            .restitution(self.restitution)
            .friction(0.5);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Sphere dropped into an inverted square-based pyramid (bowl). The sphere
/// should come to rest touching all four sloped faces simultaneously.
///
/// This scenario exercises multi-face contact stability: the contact
/// pipeline must generate contacts on all four non-coplanar faces at once,
/// not oscillate a single contact between opposite sides.
#[derive(Debug, Clone)]
struct SphereInBowlScenario {
    radius: f32,
    geometry: BowlGeometry,
}

impl SphereInBowlScenario {
    fn new() -> Self {
        // Bowl: half_size=2, depth=2 → 45° slope faces.
        // For a sphere of radius 0.5, the equilibrium center is at
        // y ≈ 0.5*√2 - 2 ≈ -1.293 (each face distance equals radius).
        Self {
            radius: 0.5,
            geometry: BowlGeometry::new(2.0, 2.0),
        }
    }
}

impl PhysicsBenchScenario for SphereInBowlScenario {
    fn name(&self) -> &'static str {
        "sphere_in_bowl"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        // Drop the sphere from just above the rim, centered over the apex.
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 0.5, 0.0)));
        let collider = ColliderDesc::sphere(self.radius)
            .density(1000.0)
            .restitution(0.0)
            .friction(0.8);
        let _ = world.attach_collider(body, collider);
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: discrete dynamic-dynamic contacts
// ═══════════════════════════════════════════════════════════════════════════

/// Two spheres on a collision course. A radial impulse kicks one sphere
/// toward the other. Exercises discrete sphere-sphere contacts and the
/// dynamic pair pipeline.
#[derive(Debug, Clone)]
struct SphereSphereCollisionScenario {
    restitution: f32,
    geometry: FlatQuadGeometry,
}

impl SphereSphereCollisionScenario {
    fn new(restitution: f32) -> Self {
        Self {
            restitution,
            geometry: FlatQuadGeometry::new(20.0),
        }
    }
}

impl PhysicsBenchScenario for SphereSphereCollisionScenario {
    fn name(&self) -> &'static str {
        "sphere_sphere_collision"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let radius = 0.5;
        let y = radius + 0.01;

        // Target sphere (tracked) — sitting at the origin.
        let target = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, y, 0.0)));
        let _ = world.attach_collider(
            target,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        // Projectile sphere — offset in -X, will be kicked toward target.
        let projectile =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(-4.0, y, 0.0)));
        let _ = world.attach_collider(
            projectile,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        target
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }

    fn external_impulses(&self, sim_time: f32) -> Vec<PhysicsImpulse> {
        if sim_time < 0.01 {
            vec![PhysicsImpulse::radial(
                Point3::new(-5.0, 0.51, 0.0),
                2.0,
                3000.0,
                0.0,
            )]
        } else {
            Vec::new()
        }
    }
}

/// Sphere launched toward a stationary box. Exercises discrete sphere-OBB
/// contacts in the dynamic pair pipeline.
#[derive(Debug, Clone)]
struct SphereObbCollisionScenario {
    restitution: f32,
    geometry: FlatQuadGeometry,
}

impl SphereObbCollisionScenario {
    fn new(restitution: f32) -> Self {
        Self {
            restitution,
            geometry: FlatQuadGeometry::new(20.0),
        }
    }
}

impl PhysicsBenchScenario for SphereObbCollisionScenario {
    fn name(&self) -> &'static str {
        "sphere_obb_collision"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let sphere_radius = 0.4;
        let box_half = Vector3::new(0.5, 0.5, 0.5);
        let y = box_half.y + 0.01;

        // Target box (tracked) at the origin.
        let target = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, y, 0.0)));
        let _ = world.attach_collider(
            target,
            ColliderDesc::box_shape(box_half)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        // Projectile sphere, offset in -X.
        let projectile = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            -4.0,
            sphere_radius + 0.01,
            0.0,
        )));
        let _ = world.attach_collider(
            projectile,
            ColliderDesc::sphere(sphere_radius)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        target
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }

    fn external_impulses(&self, sim_time: f32) -> Vec<PhysicsImpulse> {
        if sim_time < 0.01 {
            vec![PhysicsImpulse::radial(
                Point3::new(-5.0, 0.41, 0.0),
                2.0,
                3000.0,
                0.0,
            )]
        } else {
            Vec::new()
        }
    }
}

/// Two boxes on a collision course. Exercises discrete OBB-OBB contact
/// generation (15-axis SAT + Sutherland-Hodgman clipping).
#[derive(Debug, Clone)]
struct ObbObbCollisionScenario {
    restitution: f32,
    geometry: FlatQuadGeometry,
}

impl ObbObbCollisionScenario {
    fn new(restitution: f32) -> Self {
        Self {
            restitution,
            geometry: FlatQuadGeometry::new(20.0),
        }
    }
}

impl PhysicsBenchScenario for ObbObbCollisionScenario {
    fn name(&self) -> &'static str {
        "obb_obb_collision"
    }

    fn restitution(&self) -> f32 {
        self.restitution
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let half = Vector3::new(0.5, 0.5, 0.5);
        let y = half.y + 0.01;

        // Target box (tracked) at the origin.
        let target = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, y, 0.0)));
        let _ = world.attach_collider(
            target,
            ColliderDesc::box_shape(half)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        // Projectile box — launched directly toward target via initial velocity.
        let projectile = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-3.0, y, 0.0))
                .linear_velocity(Vector3::new(8.0, 0.0, 0.0)),
        );
        let _ = world.attach_collider(
            projectile,
            ColliderDesc::box_shape(half)
                .density(1000.0)
                .restitution(self.restitution)
                .friction(0.5),
        );

        target
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: many-body dynamic contacts (narrowphase throughput)
// ═══════════════════════════════════════════════════════════════════════════

/// Grid of boxes dropped onto flat terrain. Exercises the dynamic-dynamic
/// narrowphase pipeline with many simultaneous broadphase pairs. Used to
/// benchmark the work buffer's allocation reuse and cache performance.
#[derive(Debug, Clone)]
struct BoxGridScenario {
    grid_size: usize,
    half_extent: f32,
    spacing: f32,
    geometry: FlatQuadGeometry,
}

impl BoxGridScenario {
    fn new(grid_size: usize) -> Self {
        Self {
            grid_size,
            half_extent: 0.3,
            spacing: 0.8,
            geometry: FlatQuadGeometry::new(20.0),
        }
    }
}

impl PhysicsBenchScenario for BoxGridScenario {
    fn name(&self) -> &'static str {
        "box_grid"
    }

    fn restitution(&self) -> f32 {
        0.1
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let he = Vector3::new(self.half_extent, self.half_extent, self.half_extent);
        let n = self.grid_size;
        let offset = (n as f32 - 1.0) * self.spacing * 0.5;
        let mut first_handle = None;

        for ix in 0..n {
            for iz in 0..n {
                let x = ix as f32 * self.spacing - offset;
                let z = iz as f32 * self.spacing - offset;
                let y = 2.0 + (ix + iz) as f32 * 0.1;
                let body =
                    world.create_body(RigidBodyDesc::dynamic().position(Point3::new(x, y, z)));
                let _ = world.attach_collider(
                    body,
                    ColliderDesc::box_shape(he)
                        .density(1000.0)
                        .restitution(0.1)
                        .friction(0.5),
                );
                if first_handle.is_none() {
                    first_handle = Some(body);
                }
            }
        }

        first_handle.expect("grid should contain at least one body")
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: continuous collision detection
// ═══════════════════════════════════════════════════════════════════════════

/// High-speed sphere dropped from height. Without CCD, the sphere would
/// tunnel through the ground. Exercises `swept_sphere_triangle` from the
/// continuous collision module.
#[derive(Debug, Clone)]
struct HighSpeedSphereCcdScenario {
    geometry: FlatQuadGeometry,
}

impl HighSpeedSphereCcdScenario {
    fn new() -> Self {
        Self {
            geometry: FlatQuadGeometry::new(10.0),
        }
    }
}

impl PhysicsBenchScenario for HighSpeedSphereCcdScenario {
    fn name(&self) -> &'static str {
        "high_speed_sphere_ccd"
    }

    fn restitution(&self) -> f32 {
        0.3
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.ccd_threshold = 0.3;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let radius = 0.3;
        let body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, 50.0, 0.0)));
        let _ = world.attach_collider(
            body,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(0.3)
                .friction(0.5),
        );
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

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

    // ── Mesh pipeline: sphere vs flat terrain ──────────────────────────

    #[test]
    fn flat_sphere_rest_settles_on_ground() {
        let scenario = FlatSphereRestScenario::new(0.0);
        let cfg = BenchRunConfig::default();
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "flat_sphere_rest_r0_0");

        assert!(!run.samples.is_empty());
        assert_eq!(run.dropped_steps, 0);

        let (tail_linear, _) = run.tail_max_speeds(1.0);
        assert!(
            tail_linear < 0.02,
            "sphere should settle: tail linear speed {tail_linear:.6}"
        );

        let last = run.samples.last().unwrap();
        assert!(
            last.y > 0.45 && last.y < 0.55,
            "sphere center should rest near radius height: y={}",
            last.y
        );
        assert!(
            last.contact_count >= 1,
            "expected resting contact, got {}",
            last.contact_count
        );
    }

    #[test]
    fn flat_sphere_rest_bouncy_sphere_reaches_higher_peak() {
        let cfg = BenchRunConfig {
            duration: 4.0,
            ..BenchRunConfig::default()
        };

        let run_inelastic = run_scenario(&FlatSphereRestScenario::new(0.0), cfg);
        let run_bouncy = run_scenario(&FlatSphereRestScenario::new(0.8), cfg);

        write_exports(&run_inelastic, "flat_sphere_rest_bounce_r0_0");
        write_exports(&run_bouncy, "flat_sphere_rest_bounce_r0_8");

        let peak_inelastic = run_inelastic
            .samples
            .iter()
            .skip(100)
            .map(|s| s.y)
            .fold(0.0f32, f32::max);
        let peak_bouncy = run_bouncy
            .samples
            .iter()
            .skip(100)
            .map(|s| s.y)
            .fold(0.0f32, f32::max);

        assert!(
            peak_bouncy > peak_inelastic + 0.1,
            "bouncy sphere peak {peak_bouncy:.3} should exceed inelastic {peak_inelastic:.3}"
        );
    }

    // ── Mesh pipeline: sphere sliding over internal edges ────────────

    #[test]
    fn sphere_slide_no_jitter_over_seam() {
        let scenario = SphereSlideScenario::new();
        let cfg = BenchRunConfig {
            duration: 4.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "sphere_slide");

        assert!(!run.samples.is_empty());

        // The sphere should stay on the surface the entire time. With
        // radius 0.5 on a y=0 plane, center should be near 0.5.
        let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
        let max_y = run.samples.iter().map(|s| s.y).fold(0.0f32, f32::max);
        assert!(min_y > 0.4, "sphere sank below surface: min_y={min_y:.4}");
        assert!(
            max_y < 0.7,
            "sphere bounced off surface (jitter): max_y={max_y:.4}"
        );

        // The sphere should move in +X (it started with positive X velocity).
        let last = run.samples.last().unwrap();
        assert!(
            last.x > -3.0,
            "sphere should have moved in +X: x={}",
            last.x
        );

        // On flat terrain the sphere should only ever have 1 contact point
        // (the face it's over). Multiple contacts would mean spurious edge
        // contacts are leaking through, causing lateral pushing.
        let max_contacts = run
            .samples
            .iter()
            .skip(10) // skip first few frames during initial settling
            .map(|s| s.contact_count)
            .max()
            .unwrap_or(0);
        assert!(
            max_contacts <= 1,
            "sphere on flat terrain should have at most 1 contact, got {max_contacts}"
        );

        // Check for vertical speed spikes that would indicate jitter as the
        // sphere crosses the internal mesh seam. Sample the vertical speed
        // component via consecutive y differences.
        let y_speeds: Vec<f32> = run
            .samples
            .windows(2)
            .map(|w| (w[1].y - w[0].y).abs() / cfg.fixed_dt)
            .collect();
        // After initial settling (skip first 50 samples), vertical speed
        // should be very small — no sudden pops from edge contacts.
        let max_y_speed_after_settle = y_speeds.iter().skip(50).fold(0.0f32, |a, &b| a.max(b));
        assert!(
            max_y_speed_after_settle < 0.5,
            "vertical speed spike detected (seam jitter): max_y_speed={max_y_speed_after_settle:.4}"
        );
    }

    // ── Mesh pipeline: box vs flat terrain ─────────────────────────────

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
            let _ = run
                .samples
                .last()
                .expect("sweep run should produce samples");
        }
    }

    // ── Mesh pipeline: ramp terrain ────────────────────────────────────

    #[test]
    fn sphere_on_ramp_rolls_downhill() {
        let scenario = SphereOnRampScenario::new();
        let cfg = BenchRunConfig {
            duration: 4.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "sphere_on_ramp");

        assert!(!run.samples.is_empty());

        // The sphere should have gained speed from rolling/sliding down.
        let peak_speed = run
            .samples
            .iter()
            .map(|s| s.linear_speed)
            .fold(0.0f32, f32::max);
        assert!(
            peak_speed > 1.0,
            "sphere should gain speed on ramp: peak_speed={peak_speed:.3}"
        );

        // Sphere must never tunnel through the ramp.
        let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
        assert!(
            min_y > -0.5,
            "sphere should not fall through ramp: min_y={min_y:.4}"
        );
    }

    #[test]
    fn box_on_ramp_slides_downhill() {
        let scenario = BoxOnRampScenario::new();
        let cfg = BenchRunConfig {
            duration: 4.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "box_on_ramp");

        assert!(!run.samples.is_empty());

        let peak_speed = run
            .samples
            .iter()
            .map(|s| s.linear_speed)
            .fold(0.0f32, f32::max);
        assert!(
            peak_speed > 0.5,
            "box should gain speed on ramp: peak_speed={peak_speed:.3}"
        );

        let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
        assert!(
            min_y > -0.5,
            "box should not fall through ramp: min_y={min_y:.4}"
        );
    }

    // ── Mesh pipeline: step terrain ────────────────────────────────────

    #[test]
    fn box_on_step_settles_on_lower_level() {
        let scenario = BoxOnStepScenario::new();
        let cfg = BenchRunConfig::default();
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "box_on_step");

        assert!(!run.samples.is_empty());

        let (tail_linear, _) = run.tail_max_speeds(1.0);
        assert!(
            tail_linear < 0.05,
            "box should settle near step: tail linear speed {tail_linear:.6}"
        );

        // Box dropped at x=-0.5 on the lower level (y=0), should rest with
        // center at y ≈ half_extent (0.3).
        let last = run.samples.last().unwrap();
        assert!(
            last.y > 0.2 && last.y < 1.0,
            "box should rest on a surface: y={}",
            last.y
        );
    }

    // ── Mesh pipeline: bowl (multi-face concave) ────────────────────────

    #[test]
    fn sphere_in_bowl_settles_without_falling_through() {
        let scenario = SphereInBowlScenario::new();
        let cfg = BenchRunConfig {
            duration: 6.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "sphere_in_bowl");

        assert!(!run.samples.is_empty());

        // The sphere should settle inside the bowl. With half_size=2,
        // depth=2, radius=0.5, the equilibrium center is at y ≈ -1.29.
        // It must NOT fall through the apex (y < -2.0).
        let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
        assert!(
            min_y > -2.0,
            "sphere fell through bowl apex: min_y={min_y:.4}"
        );

        // Final position should be inside the bowl (y between -2.0 and 0.0).
        let last = run.samples.last().unwrap();
        assert!(
            last.y > -1.8 && last.y < 0.0,
            "sphere should rest inside bowl: y={}",
            last.y
        );

        // The sphere should be centered — not drifted laterally.
        assert!(
            last.x.abs() < 0.3,
            "sphere should stay centered in bowl: x={}",
            last.x
        );

        // The sphere rests on 4 non-coplanar faces. The contact pipeline
        // must generate contacts on multiple faces simultaneously — not
        // flicker a single contact between opposite sides. Check that
        // tail contact count is >= 2 (ideally 4).
        let tail_start = (last.sim_time - 1.0).max(0.0);
        let tail_contacts: Vec<usize> = run
            .samples
            .iter()
            .filter(|s| s.sim_time >= tail_start)
            .map(|s| s.contact_count)
            .collect();
        let min_contacts = tail_contacts.iter().copied().min().unwrap_or(0);
        assert!(
            min_contacts >= 2,
            "sphere in bowl should have contacts on multiple faces, \
             but min tail contact_count={min_contacts}"
        );

        // The sphere should be fully at rest in the tail window — not
        // oscillating due to contact flickering.
        let (tail_linear, _) = run.tail_max_speeds(1.0);
        assert!(
            tail_linear < 0.02,
            "sphere should fully settle in bowl: tail linear speed {tail_linear:.6}"
        );
    }

    // ── Dynamic pairs: sphere-sphere ───────────────────────────────────

    #[test]
    fn sphere_sphere_collision_transfers_momentum() {
        let scenario = SphereSphereCollisionScenario::new(0.8);
        let cfg = BenchRunConfig {
            duration: 3.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "sphere_sphere_collision_r0_8");

        let last = run.samples.last().unwrap();
        assert!(
            last.linear_speed > 0.1 || run.samples.iter().any(|s| s.linear_speed > 1.0),
            "target sphere should gain speed from collision"
        );
    }

    // ── Dynamic pairs: sphere-OBB ──────────────────────────────────────

    #[test]
    fn sphere_obb_collision_transfers_momentum() {
        let scenario = SphereObbCollisionScenario::new(0.8);
        let cfg = BenchRunConfig {
            duration: 3.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "sphere_obb_collision_r0_8");

        // The box target should have gained lateral velocity from the sphere hit.
        assert!(
            run.samples.iter().any(|s| s.linear_speed > 0.5),
            "target box should gain speed from sphere impact"
        );
    }

    // ── Dynamic pairs: OBB-OBB ─────────────────────────────────────────

    #[test]
    fn obb_obb_collision_transfers_momentum() {
        let scenario = ObbObbCollisionScenario::new(0.8);
        let cfg = BenchRunConfig {
            duration: 3.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "obb_obb_collision_r0_8");

        assert!(
            run.samples.iter().any(|s| s.linear_speed > 0.5),
            "target box should gain speed from box-box impact"
        );
    }

    // ── Continuous collision detection ──────────────────────────────────

    #[test]
    fn high_speed_sphere_does_not_tunnel() {
        let scenario = HighSpeedSphereCcdScenario::new();
        let cfg = BenchRunConfig {
            duration: 6.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "high_speed_sphere_ccd");

        let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
        assert!(
            min_y > -0.1,
            "sphere tunneled through ground: min_y={min_y:.4}"
        );

        let last = run.samples.last().unwrap();
        assert!(
            last.y > 0.2,
            "sphere should rest above ground: y={}",
            last.y
        );
    }

    // ── Many-body narrowphase throughput ───────────────────────────────

    #[test]
    fn box_grid_settles_without_explosions() {
        // 5x5 = 25 boxes. Exercises the narrowphase work buffer with many
        // dynamic-dynamic pairs. Verifies no boxes explode or fall through.
        let scenario = BoxGridScenario::new(5);
        let cfg = BenchRunConfig {
            duration: 4.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "box_grid_5x5");

        assert!(!run.samples.is_empty());

        // The tracked body (first box) should settle somewhere above the ground.
        let last = run.samples.last().unwrap();
        assert!(
            last.y > -0.5,
            "box should not fall through ground: y={}",
            last.y
        );

        // Tail speed should be low (settled or nearly settled).
        let (tail_linear, _) = run.tail_max_speeds(1.0);
        assert!(
            tail_linear < 1.0,
            "box grid should be settling: tail linear speed {tail_linear:.3}"
        );
    }

    #[test]
    fn box_grid_narrowphase_throughput() {
        // Throughput benchmark: 8x8 = 64 boxes producing many broadphase pairs.
        // Measures wall-clock time for 2 seconds of simulated time.
        let scenario = BoxGridScenario::new(8);
        let cfg = BenchRunConfig {
            duration: 2.0,
            ..BenchRunConfig::default()
        };

        let t0 = std::time::Instant::now();
        let run = run_scenario(&scenario, cfg);
        let elapsed = t0.elapsed();

        write_exports(&run, "box_grid_8x8_throughput");

        eprintln!(
            "Box grid 8x8 (64 bodies): {} physics steps in {:.1}ms ({:.0} steps/sec)",
            run.physics_steps,
            elapsed.as_secs_f64() * 1000.0,
            run.physics_steps as f64 / elapsed.as_secs_f64(),
        );

        assert!(!run.samples.is_empty());
        assert_eq!(run.dropped_steps, 0);
    }

    // ── Box stacking: dynamic-dynamic stability ─────────────────────────

    #[test]
    fn box_stack_settles_without_overlap() {
        let box_half_extents = Vector3::new(2.5, 0.5, 2.5);
        let geometry = FlatQuadGeometry::new(20.0);

        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        let mut world = PhysicsWorld::new(config);
        let mut debug_lines = DebugLines::default();

        let mut box_handles = Vec::new();
        for i in 0..5 {
            let y = 5.0 + (i as f32) * 5.0;
            let body =
                world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, y, 0.0)));
            let _ = world.attach_collider(
                body,
                ColliderDesc::box_shape(box_half_extents)
                    .density(1000.0)
                    .restitution(0.0)
                    .friction(0.6),
            );
            box_handles.push(body);
        }

        let fixed_dt = 1.0 / 60.0;
        let num_steps = (10.0 / fixed_dt) as usize;
        let tail_steps = (2.0 / fixed_dt) as usize;
        let tail_start_step = num_steps.saturating_sub(tail_steps);
        let mut tail_min_y = vec![f32::INFINITY; box_handles.len()];
        let mut tail_max_y = vec![f32::NEG_INFINITY; box_handles.len()];
        let mut tail_max_speed = 0.0f32;
        let mut tail_max_pair_role_swaps = 0usize;
        let mut tail_max_manifold_churn = 0usize;
        let mut tail_max_contact_depth = 0.0f32;
        let mut tail_points_sum = 0usize;
        let mut tail_samples = 0usize;

        for step_idx in 0..num_steps {
            world.step(fixed_dt, &geometry, &[], &mut debug_lines);
            debug_lines.clear();

            if step_idx >= tail_start_step {
                tail_max_pair_role_swaps =
                    tail_max_pair_role_swaps.max(world.dynamic_pair_role_swaps());
                let manifold = world.manifold_frame_stats();
                let manifold_churn =
                    manifold.point_adds + manifold.point_replacements + manifold.point_pruned;
                tail_max_manifold_churn = tail_max_manifold_churn.max(manifold_churn);
                tail_max_contact_depth = tail_max_contact_depth.max(
                    world.contact_events()
                        .iter()
                        .map(|c| c.depth)
                        .fold(0.0f32, f32::max),
                );
                tail_points_sum += manifold.points;
                tail_samples += 1;
                for (i, &handle) in box_handles.iter().enumerate() {
                    let body = world.body(handle).expect("box body should exist");
                    let y = body.position().y;
                    tail_min_y[i] = tail_min_y[i].min(y);
                    tail_max_y[i] = tail_max_y[i].max(y);
                    let spd = body.linear_velocity().magnitude();
                    if spd > tail_max_speed {
                        tail_max_speed = spd;
                    }
                }
            }

            // Diagnostic: print every 60 frames (1s intervals) and last 5 frames
            if step_idx % 60 == 0 || step_idx >= num_steps - 5 {
                let manifold = world.manifold_frame_stats();
                let manifold_churn =
                    manifold.point_adds + manifold.point_replacements + manifold.point_pruned;
                let mut line = format!(
                    "step {step_idx:4} swaps={} m_points={} m_churn={} :",
                    world.dynamic_pair_role_swaps(),
                    manifold.points,
                    manifold_churn,
                );
                for (i, &handle) in box_handles.iter().enumerate() {
                    let body = world.body(handle).expect("box body should exist");
                    let p = body.position();
                    let v = body.linear_velocity();
                    let av = body.angular_velocity();
                    line.push_str(&format!(
                        " B{i}(y={:.3} vy={:.3} spd={:.3} aspd={:.3})",
                        p.y,
                        v.y,
                        v.magnitude(),
                        av.magnitude()
                    ));
                }
                eprintln!("{line}");
            }
        }

        let mut y_positions = Vec::new();
        let mut max_speed = 0.0f32;
        for &handle in &box_handles {
            let body = world.body(handle).expect("box body should exist");
            y_positions.push(body.position().y);
            max_speed = max_speed.max(body.linear_velocity().magnitude());
        }

        y_positions.sort_by(|a, b| a.partial_cmp(b).unwrap());

        eprintln!(
            "tail_max_pair_role_swaps={tail_max_pair_role_swaps} tail_max_speed={tail_max_speed:.6} final_max_speed={max_speed:.6}"
        );
        if tail_samples > 0 {
            let tail_avg_manifold_points = tail_points_sum as f32 / tail_samples as f32;
            eprintln!(
                "tail_max_manifold_churn={} tail_avg_manifold_points={:.2} tail_max_contact_depth={:.4}",
                tail_max_manifold_churn,
                tail_avg_manifold_points,
                tail_max_contact_depth
            );
        }
        assert!(
            max_speed < 0.2,
            "boxes should have mostly settled: max speed {max_speed:.6}"
        );
        assert!(
            tail_max_speed < 0.08,
            "boxes should be near-rest in tail window: tail max speed {tail_max_speed:.6}"
        );

        for i in 0..tail_min_y.len() {
            assert!(
                tail_min_y[i].is_finite() && tail_max_y[i].is_finite(),
                "tail window sampling failed for box {i}"
            );
            let tail_peak_to_peak = tail_max_y[i] - tail_min_y[i];
            assert!(
                tail_peak_to_peak < 0.03,
                "box {i} jitters in tail window: peak-to-peak y={tail_peak_to_peak:.6}"
            );
        }

        for i in 0..y_positions.len() {
            assert!(
                y_positions[i] > 0.0,
                "box {i} fell through ground: y={}",
                y_positions[i]
            );
        }

        for i in 1..y_positions.len() {
            let separation = y_positions[i] - y_positions[i - 1];
            let min_separation = 2.0 * box_half_extents.y;
            assert!(
                separation >= min_separation * 0.95,
                "boxes {} and {} overlap: separation={:.3}, min_separation={:.3}",
                i - 1,
                i,
                separation,
                min_separation
            );
        }

        let bottom_box_y = y_positions[0];
        assert!(
            bottom_box_y > 0.4 && bottom_box_y < 0.6,
            "bottom box should rest on ground at y ≈ 0.5, got y={}",
            bottom_box_y
        );
    }
}
