use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::world::PhysicsConfig;
use super::{
    ColliderDesc, ForceField, PhysicsImpulse, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
    StaticGeometry,
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
            fixed_dt: 1.0 / 240.0,
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

/// Combined floor (y=0, normal +Y) and wall (x=0, normal +X) geometry.
///
/// The floor covers the full XZ footprint. The wall is a vertical face at x=0
/// spanning y from 0 to `wall_height`. Together they form a right-angle corner.
/// Rigid bodies placed at x > 0 may collide with both faces.
#[derive(Debug, Clone)]
struct WallAndFloorGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl WallAndFloorGeometry {
    fn new(half_size: f32, wall_height: f32) -> Self {
        let s = half_size;
        let h = wall_height;

        // Floor at y=0, normal +Y.
        // Winding check: Triangle::new(a,b,c) → normal = (b-a)×(c-a).
        // f0=(-s,0,-s), f2=(s,0,s), f1=(s,0,-s):
        //   (f2-f0)×(f1-f0) = (2s,0,2s)×(2s,0,0) = (0,4s²,0) → +Y ✓
        let f0 = Point3::new(-s, 0.0, -s);
        let f1 = Point3::new(s, 0.0, -s);
        let f2 = Point3::new(s, 0.0, s);
        let f3 = Point3::new(-s, 0.0, s);
        let floor_a = PatchTriangle {
            triangle: Triangle::new(f0, f2, f1),
            neighbors: [Some(1), None, None],
        };
        let floor_b = PatchTriangle {
            triangle: Triangle::new(f0, f3, f2),
            neighbors: [None, None, Some(0)],
        };

        // Wall at x=0, normal +X (faces the positive-X side where bodies rest).
        // w0=(0,0,-s), w2=(0,h,-s), w1=(0,0,s):
        //   (w2-w0)×(w1-w0) = (0,h,0)×(0,0,2s) = (2hs,0,0) → +X ✓
        // w2=(0,h,-s), w3=(0,h,s), w1=(0,0,s):
        //   (w3-w2)×(w1-w2) = (0,0,2s)×(0,-h,2s) = (2hs,0,0) → +X ✓
        let w0 = Point3::new(0.0, 0.0, -s);
        let w1 = Point3::new(0.0, 0.0, s);
        let w2 = Point3::new(0.0, h, -s);
        let w3 = Point3::new(0.0, h, s);
        let wall_a = PatchTriangle {
            triangle: Triangle::new(w0, w2, w1),
            neighbors: [None, None, Some(3)],
        };
        let wall_b = PatchTriangle {
            triangle: Triangle::new(w2, w3, w1),
            neighbors: [None, None, Some(2)],
        };

        Self {
            bounds: AABB::new(Point3::new(-s, -0.01, -s), Point3::new(s, h + 0.01, s)),
            patch: MeshPatch {
                triangles: vec![floor_a, floor_b, wall_a, wall_b],
            },
        }
    }
}

impl StaticGeometry for WallAndFloorGeometry {
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

/// Heavy sphere resting near the edge of a thin platform box on flat terrain.
///
/// This reproduces a dynamic-on-dynamic resting-contact case where the lower
/// platform can develop rotational jitter under asymmetric load.
#[derive(Debug, Clone)]
struct HeavySphereOnPlatformScenario {
    platform_half_extents: Vector3<f32>,
    platform_density: f32,
    sphere_radius: f32,
    sphere_density: f32,
    geometry: FlatQuadGeometry,
}

impl HeavySphereOnPlatformScenario {
    fn new() -> Self {
        Self {
            platform_half_extents: Vector3::new(3.5, 0.08, 3.5),
            platform_density: 350.0,
            sphere_radius: 0.7,
            sphere_density: 30_000.0,
            geometry: FlatQuadGeometry::new(30.0),
        }
    }
}

impl PhysicsBenchScenario for HeavySphereOnPlatformScenario {
    fn name(&self) -> &'static str {
        "heavy_sphere_on_platform"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let he = self.platform_half_extents;
        let platform = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            0.0,
            he.y + 0.01,
            0.0,
        )));
        let _ = world.attach_collider(
            platform,
            ColliderDesc::box_shape(he)
                .density(self.platform_density)
                .restitution(0.0)
                .friction(0.8),
        );

        let sphere_x = he.x - self.sphere_radius * 0.35;
        let sphere_z = he.z - self.sphere_radius * 0.55;
        let sphere_y = he.y * 2.0 + self.sphere_radius + 0.2;
        let sphere = world.create_body(
            RigidBodyDesc::dynamic().position(Point3::new(sphere_x, sphere_y, sphere_z)),
        );
        let _ = world.attach_collider(
            sphere,
            ColliderDesc::sphere(self.sphere_radius)
                .density(self.sphere_density)
                .restitution(0.0)
                .friction(0.8),
        );

        platform
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

/// Box leaning against a vertical wall at x=0 with its bottom corner on the
/// floor at y=0 — the classic "ladder sliding down a wall" problem.
///
/// Both contact corners lie on the box's −X face:
///   - Wall contact at local (−a, +L) → world (0, 2L cos θ)
///   - Floor contact at local (−a, −L) → world (2L sin θ, 0)
///
/// The constrained center of mass follows:
///   x_c = L sin θ + a cos θ
///   y_c = L cos θ + a sin θ
///
/// where a = half_extents.x (thin), L = half_extents.y (long), and θ is the
/// CCW tilt angle from vertical. This is a 1-DOF system with an analytical ODE
/// solution (see `LadderOde`), so the simulation can be compared against the
/// reference trajectory.
///
/// The scenario uses zero friction and zero restitution for a clean comparison
/// against the frictionless analytical solution.
#[derive(Debug, Clone)]
struct BoxSlidesDownWallScenario {
    /// Half-extents: x = thin (perpendicular to wall), y = long (parallel).
    half_extents: Vector3<f32>,
    /// Initial tilt angle from vertical (radians).
    theta0: f32,
    /// Friction coefficient for both wall and floor contacts.
    friction: f32,
    geometry: WallAndFloorGeometry,
}

impl BoxSlidesDownWallScenario {
    fn new(friction: f32) -> Self {
        let theta0 = std::f32::consts::FRAC_PI_6; // 30°
        Self {
            half_extents: Vector3::new(0.15, 1.2, 0.4),
            theta0,
            friction,
            geometry: WallAndFloorGeometry::new(8.0, 8.0),
        }
    }
}

impl PhysicsBenchScenario for BoxSlidesDownWallScenario {
    fn name(&self) -> &'static str {
        "box_slides_down_wall"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let a = self.half_extents.x;
        let big_l = self.half_extents.y;
        let theta = self.theta0;

        // Place the box exactly in the constrained configuration: wall corner
        // at x=0, floor corner at y=0, both on the −X face.
        let x_c = big_l * theta.sin() + a * theta.cos();
        let y_c = big_l * theta.cos() + a * theta.sin();

        let mut desc = RigidBodyDesc::dynamic().position(Point3::new(x_c, y_c, 0.0));
        desc.rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), theta);
        let body = world.create_body(desc);
        let collider = ColliderDesc::box_shape(self.half_extents)
            .density(500.0)
            .restitution(0.0)
            .friction(self.friction);
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

        // Projectile sphere — offset in -X, launched directly toward target.
        let projectile = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-3.0, y, 0.0))
                .linear_velocity(Vector3::new(8.0, 0.0, 0.0)),
        );
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

        // Projectile sphere — launched directly toward target.
        let projectile = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-3.0, sphere_radius + 0.01, 0.0))
                .linear_velocity(Vector3::new(8.0, 0.0, 0.0)),
        );
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
// Solver stability validation scenarios
// ═══════════════════════════════════════════════════════════════════════════

/// Sphere launched horizontally across a flat surface with moderate friction.
///
/// Tests that friction warm-start does not inject angular torque spikes as
/// the sphere decelerates. The sphere should slow smoothly and come to rest.
#[derive(Debug, Clone)]
struct SlidingSphereScenario {
    geometry: FlatQuadGeometry,
}

impl SlidingSphereScenario {
    fn new() -> Self {
        Self {
            geometry: FlatQuadGeometry::new(50.0),
        }
    }
}

impl PhysicsBenchScenario for SlidingSphereScenario {
    fn name(&self) -> &'static str {
        "sliding_sphere"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let radius = 0.5;
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            0.0,
            radius + 0.01,
            0.0,
        )));
        let _ = world.attach_collider(
            body,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(0.0)
                .friction(0.5),
        );
        // Give it a horizontal push
        world.set_body_velocity(body, Vector3::new(5.0, 0.0, 0.0), Vector3::zeros());
        body
    }

    fn geometry(&self) -> &dyn StaticGeometry {
        &self.geometry
    }
}

/// Sphere on a low-friction 30° ramp. With friction = 0.1 (below tan(30°) ≈ 0.577),
/// the sphere should slide down and accelerate rather than sticking.
#[derive(Debug, Clone)]
struct LowFrictionRampScenario {
    geometry: RampGeometry,
}

impl LowFrictionRampScenario {
    fn new() -> Self {
        // 30° slope: rise/run = tan(30°) ≈ 0.577
        let run = 20.0;
        let rise = run * 0.577;
        Self {
            geometry: RampGeometry::new(5.0, run, rise),
        }
    }
}

impl PhysicsBenchScenario for LowFrictionRampScenario {
    fn name(&self) -> &'static str {
        "low_friction_ramp"
    }

    fn restitution(&self) -> f32 {
        0.0
    }

    fn build_world(&self) -> PhysicsWorld {
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        PhysicsWorld::new(config)
    }

    fn setup(&self, world: &mut PhysicsWorld) -> RigidBodyHandle {
        let radius = 0.5;
        // Start on the ramp (z > 0 is the ramp section, partway up)
        let z = 10.0;
        let y_on_ramp = 0.577 * z + radius + 0.01;
        let body =
            world.create_body(RigidBodyDesc::dynamic().position(Point3::new(0.0, y_on_ramp, z)));
        let _ = world.attach_collider(
            body,
            ColliderDesc::sphere(radius)
                .density(1000.0)
                .restitution(0.0)
                .friction(0.1),
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

    #[derive(Debug, Clone)]
    struct CubeShellGeometry {
        bounds: AABB,
        patch: MeshPatch,
    }

    impl CubeShellGeometry {
        fn new(half_extent: f32) -> Self {
            let h = half_extent;
            let v000 = Point3::new(-h, -h, -h);
            let v001 = Point3::new(-h, -h, h);
            let v010 = Point3::new(-h, h, -h);
            let v011 = Point3::new(-h, h, h);
            let v100 = Point3::new(h, -h, -h);
            let v101 = Point3::new(h, -h, h);
            let v110 = Point3::new(h, h, -h);
            let v111 = Point3::new(h, h, h);

            let triangles = vec![
                // -X face
                PatchTriangle {
                    triangle: Triangle::new(v000, v011, v010),
                    neighbors: [None; 3],
                },
                PatchTriangle {
                    triangle: Triangle::new(v000, v001, v011),
                    neighbors: [None; 3],
                },
                // +X face
                PatchTriangle {
                    triangle: Triangle::new(v100, v110, v111),
                    neighbors: [None; 3],
                },
                PatchTriangle {
                    triangle: Triangle::new(v100, v111, v101),
                    neighbors: [None; 3],
                },
                // -Y face
                PatchTriangle {
                    triangle: Triangle::new(v000, v100, v101),
                    neighbors: [None; 3],
                },
                PatchTriangle {
                    triangle: Triangle::new(v000, v101, v001),
                    neighbors: [None; 3],
                },
                // +Y face
                PatchTriangle {
                    triangle: Triangle::new(v010, v011, v111),
                    neighbors: [None; 3],
                },
                PatchTriangle {
                    triangle: Triangle::new(v010, v111, v110),
                    neighbors: [None; 3],
                },
                // -Z face
                PatchTriangle {
                    triangle: Triangle::new(v000, v010, v110),
                    neighbors: [None; 3],
                },
                PatchTriangle {
                    triangle: Triangle::new(v000, v110, v100),
                    neighbors: [None; 3],
                },
                // +Z face
                PatchTriangle {
                    triangle: Triangle::new(v001, v101, v111),
                    neighbors: [None; 3],
                },
                PatchTriangle {
                    triangle: Triangle::new(v001, v111, v011),
                    neighbors: [None; 3],
                },
            ];

            Self {
                bounds: AABB::new(Point3::new(-h, -h, -h), Point3::new(h, h, h)),
                patch: MeshPatch { triangles },
            }
        }
    }

    impl StaticGeometry for CubeShellGeometry {
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
            .filter(|s| s.sim_time >= 1.5)
            .map(|s| s.y)
            .fold(0.0f32, f32::max);
        let peak_bouncy = run_bouncy
            .samples
            .iter()
            .filter(|s| s.sim_time >= 1.5)
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

    #[test]
    fn heavy_sphere_on_platform_near_edge_settles_without_rotational_jitter() {
        let scenario = HeavySphereOnPlatformScenario::new();
        let cfg = BenchRunConfig {
            duration: 10.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "heavy_sphere_on_platform");

        assert!(!run.samples.is_empty());

        let tail_start = (cfg.duration - 3.0).max(0.0);
        let tail_max_angular_speed = run
            .samples
            .iter()
            .filter(|s| s.sim_time >= tail_start)
            .map(|s| s.angular_speed)
            .fold(0.0f32, f32::max);
        eprintln!("heavy_sphere_on_platform tail_max_angular_speed={tail_max_angular_speed:.6}");
        assert!(
            tail_max_angular_speed < 0.005,
            "platform rotational jitter detected: tail_max_angular_speed={tail_max_angular_speed:.6}"
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

    // ── Mesh pipeline: box sliding down a wall (analytical reference) ──

    /// Analytical ODE for the "ladder sliding down a wall" problem.
    ///
    /// State: `[θ, θ̇]` where θ is the CCW tilt angle from vertical.
    ///
    /// The constrained box has center of mass at:
    ///   x_c = L sin θ + a cos θ
    ///   y_c = L cos θ + a sin θ
    ///
    /// **Frictionless case** (Lagrangian):
    ///   J(θ) θ̈ = 2mLa cos(2θ) θ̇² + mg(L sin θ − a cos θ)
    ///   where J(θ) = m(L² + a² − 2La sin 2θ) + I_z.
    ///
    /// **With friction** (Newton + torque about CM): solve a 3×3 linear system
    /// for (θ̈, N_w, N_f) at each step:
    ///
    /// ```text
    ///   ┌ −m·u        1        −μ_f         ┐ ┌ θ̈ ┐   ┌ −m·w₁·θ̇²      ┐
    ///   │ −m·v        μ_w       1            │ │ Nw │ = │  mg − m·w₂·θ̇²  │
    ///   │  I      u+μ_w·w₁  −(p−μ_f·w₂)     │ └    ┘   └  0              ┘
    ///   └                                    ┘
    /// ```
    ///
    /// where u = L cosθ − a sinθ, v = −L sinθ + a cosθ, w₁ = L sinθ + a cosθ,
    /// w₂ = L cosθ + a sinθ, p = L sinθ − a cosθ, I = I_z/m = (a² + L²)/3.
    struct LadderOde {
        a: f32,
        big_l: f32,
        gravity: f32,
        mu_wall: f32,
        mu_floor: f32,
    }

    impl LadderOde {
        fn new(half_extents: Vector3<f32>, gravity: f32) -> Self {
            Self {
                a: half_extents.x,
                big_l: half_extents.y,
                gravity,
                mu_wall: 0.0,
                mu_floor: 0.0,
            }
        }

        fn with_friction(mut self, mu_wall: f32, mu_floor: f32) -> Self {
            self.mu_wall = mu_wall;
            self.mu_floor = mu_floor;
            self
        }

        /// Moment of inertia per unit mass about the Z axis (cuboid).
        fn i_per_mass(&self) -> f32 {
            (self.a * self.a + self.big_l * self.big_l) / 3.0
        }

        /// Helper quantities that appear repeatedly in the equations.
        fn coeffs(&self, theta: f32) -> LadderCoeffs {
            let (s, c) = (theta.sin(), theta.cos());
            let a = self.a;
            let l = self.big_l;
            LadderCoeffs {
                u: l * c - a * s,
                v: -l * s + a * c,
                w1: l * s + a * c,
                w2: l * c + a * s,
                p: l * s - a * c,
            }
        }

        /// Solve the 3×3 system for (θ̈, N_w, N_f) given (θ, θ̇).
        /// Returns (theta_ddot, n_wall_per_mass, n_floor_per_mass).
        fn solve(&self, theta: f32, theta_dot: f32) -> (f32, f32, f32) {
            let c = self.coeffs(theta);
            let g = self.gravity;
            let i = self.i_per_mass();
            let mw = self.mu_wall;
            let mf = self.mu_floor;
            let td2 = theta_dot * theta_dot;

            // Rows of [A | b] for the system A·x = b, where x = [θ̈, Nw, Nf].
            // Row 0: −u·θ̈ + Nw − μ_f·Nf = −w₁·θ̇²
            // Row 1: −v·θ̈ + μ_w·Nw + Nf = g − w₂·θ̇²
            // Row 2:  I·θ̈ + (u+μ_w·w₁)·Nw − (p−μ_f·w₂)·Nf = 0
            let a00 = -c.u;
            let a01 = 1.0;
            let a02 = -mf;
            let b0 = -c.w1 * td2;

            let a10 = -c.v;
            let a11 = mw;
            let a12 = 1.0;
            let b1 = g - c.w2 * td2;

            let a20 = i;
            let a21 = c.u + mw * c.w1;
            let a22 = -(c.p - mf * c.w2);
            let b2 = 0.0;

            // Cramer's rule.
            let det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20)
                + a02 * (a10 * a21 - a11 * a20);

            if det.abs() < 1e-12 {
                return (0.0, 0.0, 0.0);
            }

            let inv = 1.0 / det;

            let theta_ddot = inv
                * (b0 * (a11 * a22 - a12 * a21) - a01 * (b1 * a22 - a12 * b2)
                    + a02 * (b1 * a21 - a11 * b2));

            let nw = inv
                * (a00 * (b1 * a22 - a12 * b2) - b0 * (a10 * a22 - a12 * a20)
                    + a02 * (a10 * b2 - b1 * a20));

            let nf = inv
                * (a00 * (a11 * b2 - b1 * a21) - a01 * (a10 * b2 - b1 * a20)
                    + b0 * (a10 * a21 - a11 * a20));

            (theta_ddot, nw, nf)
        }

        /// Center-of-mass position from angle.
        fn cm_position(&self, theta: f32) -> (f32, f32) {
            let a = self.a;
            let l = self.big_l;
            (
                l * theta.sin() + a * theta.cos(),
                l * theta.cos() + a * theta.sin(),
            )
        }

        /// Solve with stiction: if friction would reverse the slide direction
        /// (θ̇ ≈ 0 and θ̈ < 0), the system is in static equilibrium and θ̈ = 0.
        fn solve_with_stiction(&self, theta: f32, theta_dot: f32) -> (f32, f32, f32) {
            let (theta_ddot, nw, nf) = self.solve(theta, theta_dot);

            // If the body is at rest (or nearly so) and friction would push it
            // backwards, the system is stuck. Clamp to zero acceleration.
            if theta_dot.abs() < 1e-6 && theta_ddot < 0.0 {
                return (0.0, nw, nf);
            }

            // If the body is sliding but friction brings it to rest, clamp.
            if theta_dot > 0.0 && theta_ddot < 0.0 {
                // Still decelerating — kinetic friction is correct. Only clamp
                // in the integrator when θ̇ actually reaches zero.
            }

            (theta_ddot, nw, nf)
        }

        /// Integrate the ODE from θ₀ with θ̇₀ = 0 using RK4.
        /// Returns samples at each dt step: (time, θ, θ̇, x_c, y_c).
        /// Stops when the box separates from the wall (N_w ≤ 0) or the box
        /// sticks (friction holds it in static equilibrium).
        fn integrate(&self, theta0: f32, dt: f64, duration: f64) -> Vec<LadderSample> {
            let mut samples = Vec::new();
            let mut t = 0.0f64;
            let mut state = [theta0 as f64, 0.0f64]; // [θ, θ̇]

            while t <= duration + 1e-9 {
                let theta = state[0] as f32;
                let theta_dot = state[1] as f32;
                let (x_c, y_c) = self.cm_position(theta);
                let (_, nw, _) = self.solve_with_stiction(theta, theta_dot);

                samples.push(LadderSample {
                    time: t as f32,
                    theta,
                    theta_dot,
                    x_c,
                    y_c,
                });

                // Wall separation: box lifts off the wall.
                if nw < -1e-6 {
                    break;
                }

                // RK4 step (in f64 for precision).
                let f = |s: [f64; 2]| -> [f64; 2] {
                    let th = s[0] as f32;
                    let th_dot = s[1] as f32;
                    let (th_ddot, _, _) = self.solve_with_stiction(th, th_dot);
                    [s[1], th_ddot as f64]
                };

                let k1 = f(state);
                let s2 = [state[0] + 0.5 * dt * k1[0], state[1] + 0.5 * dt * k1[1]];
                let k2 = f(s2);
                let s3 = [state[0] + 0.5 * dt * k2[0], state[1] + 0.5 * dt * k2[1]];
                let k3 = f(s3);
                let s4 = [state[0] + dt * k3[0], state[1] + dt * k3[1]];
                let k4 = f(s4);

                state[0] += dt / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
                state[1] += dt / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);

                // Clamp θ̇ to non-negative: the ladder can't slide backwards.
                if state[1] < 0.0 {
                    state[1] = 0.0;
                }

                t += dt;
            }

            samples
        }
    }

    struct LadderCoeffs {
        u: f32,
        v: f32,
        w1: f32,
        w2: f32,
        p: f32,
    }

    #[derive(Debug, Clone, Copy)]
    struct LadderSample {
        time: f32,
        theta: f32,
        theta_dot: f32,
        x_c: f32,
        y_c: f32,
    }

    /// Extract the tilt angle θ (CCW from vertical, around Z) from a quaternion.
    fn extract_theta_z(q: &UnitQuaternion<f32>) -> f32 {
        // For a pure Z rotation: q = [cos(θ/2), 0, 0, sin(θ/2)].
        2.0 * q.quaternion().k.atan2(q.quaternion().w)
    }

    /// Jitter analysis results for a wall-slide trajectory.
    #[derive(Debug)]
    struct JitterReport {
        /// Number of frames where x moved backwards (dx < 0) during the
        /// wall-contact phase. Should be 0 for a smooth slide.
        x_reversals: usize,
        /// Number of frames where y moved upward (dy > 0) during the
        /// wall-contact phase. Should be 0 for a smooth slide.
        y_reversals: usize,
        /// Maximum frame-to-frame jerk in x (|d²x/dt²| between consecutive
        /// frames). Large values indicate discontinuous contact forces.
        max_x_jerk: f32,
        /// Maximum frame-to-frame jerk in y.
        max_y_jerk: f32,
        /// Number of frames where contact count changed. Frequent changes
        /// indicate the manifold is flickering.
        contact_flips: usize,
        /// Total frames analysed in the wall-contact phase.
        wall_contact_frames: usize,
    }

    /// Analyse a wall-slide trajectory for jitter during the wall-contact phase.
    ///
    /// `wall_contact_end` is the sim_time at which wall separation occurs
    /// (from the analytical ODE). Only samples before this time are checked.
    fn analyse_jitter(
        samples: &[BenchSample],
        fixed_dt: f32,
        wall_contact_end: f32,
    ) -> JitterReport {
        let wall_samples: Vec<&BenchSample> = samples
            .iter()
            .filter(|s| s.sim_time <= wall_contact_end)
            .collect();

        let mut x_reversals = 0usize;
        let mut y_reversals = 0usize;
        let mut contact_flips = 0usize;

        // Compute velocities from consecutive position samples.
        let mut velocities: Vec<(f32, f32)> = Vec::new();
        for w in wall_samples.windows(2) {
            let dx = w[1].x - w[0].x;
            let dy = w[1].y - w[0].y;
            let vx = dx / fixed_dt;
            let vy = dy / fixed_dt;
            velocities.push((vx, vy));

            // During the slide, x should always increase (box moves away
            // from wall). A negative dx means the wall contact pushed the
            // box backwards.
            if dx < -1e-6 {
                x_reversals += 1;
            }

            // y should always decrease (box moves down). A positive dy
            // means the floor contact bounced the box upward.
            if dy > 1e-6 {
                y_reversals += 1;
            }

            // Contact count changes.
            if w[1].contact_count != w[0].contact_count {
                contact_flips += 1;
            }
        }

        // Compute jerk: change in velocity between consecutive frames.
        // This detects acceleration spikes from discontinuous contact forces.
        let mut max_x_jerk = 0.0f32;
        let mut max_y_jerk = 0.0f32;
        for w in velocities.windows(2) {
            let jerk_x = ((w[1].0 - w[0].0) / fixed_dt).abs();
            let jerk_y = ((w[1].1 - w[0].1) / fixed_dt).abs();
            max_x_jerk = max_x_jerk.max(jerk_x);
            max_y_jerk = max_y_jerk.max(jerk_y);
        }

        JitterReport {
            x_reversals,
            y_reversals,
            max_x_jerk,
            max_y_jerk,
            contact_flips,
            wall_contact_frames: wall_samples.len(),
        }
    }

    #[test]
    fn box_slides_down_wall_frictionless_matches_analytical_ode() {
        let scenario = BoxSlidesDownWallScenario::new(0.0);
        let fixed_dt = 1.0 / 60.0;
        let duration = 3.0;

        let cfg = BenchRunConfig {
            fixed_dt,
            duration,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "box_slides_down_wall");

        assert!(!run.samples.is_empty());
        assert_eq!(run.dropped_steps, 0);

        // Integrate the analytical reference with a fine time step.
        let ode = LadderOde::new(scenario.half_extents, 9.81);
        let ref_dt = (fixed_dt as f64) / 10.0; // 10× finer than physics step
        let ref_samples = ode.integrate(scenario.theta0, ref_dt, duration as f64);

        assert!(
            !ref_samples.is_empty(),
            "analytical ODE should produce samples"
        );

        // The ODE should show wall separation before θ reaches 90°.
        let last_ref = ref_samples.last().unwrap();
        let sep_theta_deg = last_ref.theta.to_degrees();
        eprintln!(
            "wall separation at θ={sep_theta_deg:.2}°, t={:.4}s",
            last_ref.time
        );

        // Compare simulation samples against the analytical reference.
        // For each physics step, find the closest reference sample and check
        // that x_c and y_c agree within tolerance.
        let mut max_x_err = 0.0f32;
        let mut max_y_err = 0.0f32;
        let mut comparisons = 0usize;

        // Only compare while the analytical solution is in the wall-contact
        // phase (before separation). The simulation continues after separation
        // but the simple ODE stops.
        let ref_end_time = last_ref.time;

        for sample in &run.samples {
            if sample.sim_time > ref_end_time - fixed_dt {
                break;
            }

            // Find the closest reference sample by time.
            let ref_sample = ref_samples
                .iter()
                .min_by(|a, b| {
                    (a.time - sample.sim_time)
                        .abs()
                        .partial_cmp(&(b.time - sample.sim_time).abs())
                        .unwrap()
                })
                .unwrap();

            let x_err = (sample.x - ref_sample.x_c).abs();
            let y_err = (sample.y - ref_sample.y_c).abs();
            max_x_err = max_x_err.max(x_err);
            max_y_err = max_y_err.max(y_err);
            comparisons += 1;
        }

        eprintln!("compared {comparisons} samples against analytical ODE");
        eprintln!("max position error: x={max_x_err:.6}, y={max_y_err:.6}");

        // Jitter detection: during the wall-contact phase, the trajectory
        // should be perfectly smooth. Any velocity reversals or acceleration
        // spikes indicate contact manifold flickering.
        let jitter = analyse_jitter(&run.samples, fixed_dt, ref_end_time);
        eprintln!(
            "jitter: x_rev={} y_rev={} max_jerk=({:.1},{:.1}) contact_flips={} frames={}",
            jitter.x_reversals,
            jitter.y_reversals,
            jitter.max_x_jerk,
            jitter.max_y_jerk,
            jitter.contact_flips,
            jitter.wall_contact_frames,
        );
        assert_eq!(
            jitter.x_reversals, 0,
            "x moved backwards during wall slide ({} reversals = contact jitter)",
            jitter.x_reversals
        );
        assert_eq!(
            jitter.y_reversals, 0,
            "y moved upward during wall slide ({} reversals = contact jitter)",
            jitter.y_reversals
        );
        // Jerk threshold: for gravity g=9.81 at 60Hz, the baseline
        // gravitational jerk is ~g/dt² ≈ 35000. Allow 2× that to catch
        // spikes from contact flickering without false-positiving on
        // normal acceleration changes.
        let jerk_limit = 80000.0;
        assert!(
            jitter.max_x_jerk < jerk_limit,
            "x acceleration spike during wall slide: jerk={:.1} (limit={jerk_limit})",
            jitter.max_x_jerk
        );
        assert!(
            jitter.max_y_jerk < jerk_limit,
            "y acceleration spike during wall slide: jerk={:.1} (limit={jerk_limit})",
            jitter.max_y_jerk
        );

        assert!(
            comparisons >= 10,
            "expected at least 10 comparison samples, got {comparisons}"
        );

        // Position tolerance: 0.05 allows for discrete-time integration error
        // and the contact margin. The physics runs at 60 Hz with iterative
        // constraint solving, so exact agreement isn't expected.
        let tol = 0.05;
        assert!(
            max_x_err < tol,
            "x_c diverges from analytical solution: max error {max_x_err:.6} (tol={tol})"
        );
        assert!(
            max_y_err < tol,
            "y_c diverges from analytical solution: max error {max_y_err:.6} (tol={tol})"
        );

        // Verify the simulation also extracts a reasonable tilt angle.
        // At the first sample, θ should match the initial value.
        let world_check = {
            let mut world = scenario.build_world();
            let handle = scenario.setup(&mut world);
            let body = world.body(handle).unwrap();
            let theta_initial = extract_theta_z(&body.rotation());
            (theta_initial - scenario.theta0).abs()
        };
        assert!(
            world_check < 1e-4,
            "initial rotation should match theta0: error={world_check:.6}"
        );

        // The box must not fall through the floor at any point.
        let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
        assert!(min_y > -0.1, "box fell through floor: min_y={min_y:.4}");
    }

    #[test]
    fn box_slides_down_wall_with_friction_matches_analytical_ode() {
        // Critical μ for static equilibrium at θ=30° is ~0.25 (from
        // tanθ = 2μ/(1−μ²)). Use 0.2 so the ladder slides but is
        // noticeably slowed compared to frictionless.
        let friction = 0.2;
        let scenario = BoxSlidesDownWallScenario::new(friction);
        let fixed_dt = 1.0 / 60.0;
        let duration = 3.0;

        let cfg = BenchRunConfig {
            fixed_dt,
            duration,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "box_slides_down_wall_friction");

        assert!(!run.samples.is_empty());
        assert_eq!(run.dropped_steps, 0);

        // Integrate the analytical reference with friction.
        let ode = LadderOde::new(scenario.half_extents, 9.81).with_friction(friction, friction);
        let ref_dt = (fixed_dt as f64) / 10.0;
        let ref_samples = ode.integrate(scenario.theta0, ref_dt, duration as f64);

        assert!(
            !ref_samples.is_empty(),
            "analytical ODE should produce samples"
        );

        let last_ref = ref_samples.last().unwrap();
        let sep_theta_deg = last_ref.theta.to_degrees();
        eprintln!(
            "with friction {friction}: wall separation at θ={sep_theta_deg:.2}°, t={:.4}s",
            last_ref.time
        );

        // Friction should delay separation: the box stays in contact longer
        // than the frictionless case (~0.48s).
        assert!(
            last_ref.time > 0.48,
            "friction should delay wall separation (t={:.4}s vs ~0.48s frictionless)",
            last_ref.time
        );

        let mut max_x_err = 0.0f32;
        let mut max_y_err = 0.0f32;
        let mut comparisons = 0usize;

        let ref_end_time = last_ref.time;

        for sample in &run.samples {
            if sample.sim_time > ref_end_time - fixed_dt {
                break;
            }

            let ref_sample = ref_samples
                .iter()
                .min_by(|a, b| {
                    (a.time - sample.sim_time)
                        .abs()
                        .partial_cmp(&(b.time - sample.sim_time).abs())
                        .unwrap()
                })
                .unwrap();

            let x_err = (sample.x - ref_sample.x_c).abs();
            let y_err = (sample.y - ref_sample.y_c).abs();
            max_x_err = max_x_err.max(x_err);
            max_y_err = max_y_err.max(y_err);
            comparisons += 1;
        }

        eprintln!("compared {comparisons} samples against analytical ODE (friction={friction})");
        eprintln!("max position error: x={max_x_err:.6}, y={max_y_err:.6}");

        // Jitter detection (same checks as frictionless).
        let jitter = analyse_jitter(&run.samples, fixed_dt, ref_end_time);
        eprintln!(
            "jitter: x_rev={} y_rev={} max_jerk=({:.1},{:.1}) contact_flips={} frames={}",
            jitter.x_reversals,
            jitter.y_reversals,
            jitter.max_x_jerk,
            jitter.max_y_jerk,
            jitter.contact_flips,
            jitter.wall_contact_frames,
        );
        assert_eq!(
            jitter.x_reversals, 0,
            "x moved backwards during wall slide ({} reversals = contact jitter)",
            jitter.x_reversals
        );
        assert_eq!(
            jitter.y_reversals, 0,
            "y moved upward during wall slide ({} reversals = contact jitter)",
            jitter.y_reversals
        );
        let jerk_limit = 80000.0;
        assert!(
            jitter.max_x_jerk < jerk_limit,
            "x acceleration spike during wall slide: jerk={:.1} (limit={jerk_limit})",
            jitter.max_x_jerk
        );
        assert!(
            jitter.max_y_jerk < jerk_limit,
            "y acceleration spike during wall slide: jerk={:.1} (limit={jerk_limit})",
            jitter.max_y_jerk
        );

        assert!(
            comparisons >= 10,
            "expected at least 10 comparison samples, got {comparisons}"
        );

        // Wider tolerance than frictionless: iterative friction solving
        // accumulates error over the much longer contact phase (~1.4s vs
        // ~0.5s frictionless). The primary value is the qualitative check
        // (separation angle and timing) below.
        let tol = 0.2;
        assert!(
            max_x_err < tol,
            "x_c diverges from analytical solution: max error {max_x_err:.6} (tol={tol})"
        );
        assert!(
            max_y_err < tol,
            "y_c diverges from analytical solution: max error {max_y_err:.6} (tol={tol})"
        );

        // Compare against the frictionless reference to verify friction has
        // the expected qualitative effect: later separation and larger angle.
        let ode_frictionless = LadderOde::new(scenario.half_extents, 9.81);
        let ref_frictionless = ode_frictionless.integrate(scenario.theta0, ref_dt, duration as f64);
        let sep_time_frictionless = ref_frictionless.last().unwrap().time;

        assert!(
            last_ref.time > sep_time_frictionless * 1.5,
            "friction should significantly delay separation: \
             t_friction={:.4}s vs t_frictionless={:.4}s",
            last_ref.time,
            sep_time_frictionless
        );
        assert!(
            last_ref.theta > ref_frictionless.last().unwrap().theta,
            "friction should increase separation angle"
        );

        // The box must not fall through the floor.
        let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
        assert!(min_y > -0.1, "box fell through floor: min_y={min_y:.4}");
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
    fn boundary_and_grenade_like_impulses_keep_states_finite() {
        let geometry = CubeShellGeometry::new(32.0);

        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        let mut world = PhysicsWorld::new(config);
        let mut debug_lines = DebugLines::default();

        // Match the app's boundary spring so this stress test exercises
        // the same energy-injection path as gameplay.
        world.add_force_field(ForceField::boundary(geometry.bounds, 500.0));

        // Dense jumble of boxes near the center.
        let box_half = Vector3::new(0.3, 0.3, 0.3);
        for x in 0..5 {
            for z in 0..5 {
                let px = -1.2 + x as f32 * 0.6;
                let pz = -1.2 + z as f32 * 0.6;
                let py = 1.0 + ((x + z) % 3) as f32 * 0.6;
                let body =
                    world.create_body(RigidBodyDesc::dynamic().position(Point3::new(px, py, pz)));
                let _ = world.attach_collider(
                    body,
                    ColliderDesc::box_shape(box_half)
                        .density(0.5)
                        .restitution(0.2)
                        .friction(0.6),
                );
            }
        }

        // Small spheres mixed into the pile (grenade-sized).
        let sphere_radius = 0.2622022;
        for i in 0..10 {
            let t = i as f32;
            let px = (t * 0.37).sin() * 1.0;
            let pz = (t * 0.61).cos() * 1.0;
            let py = 1.3 + (i % 4) as f32 * 0.45;
            let body =
                world.create_body(RigidBodyDesc::dynamic().position(Point3::new(px, py, pz)));
            let _ = world.attach_collider(
                body,
                ColliderDesc::sphere(sphere_radius)
                    .density(1000.0)
                    .restitution(0.2)
                    .friction(0.5),
            );
        }

        let fixed_dt = 1.0 / 60.0;
        let num_steps = (20.0 / fixed_dt) as usize;
        let mut failure: Option<String> = None;

        for step in 0..num_steps {
            let impulses = if step > 30 && step % 15 == 0 {
                vec![
                    PhysicsImpulse::radial(Point3::new(0.0, 0.8, 0.0), 10.0, 26000.0, 0.6),
                    PhysicsImpulse::radial(Point3::new(0.0, 0.2, 0.0), 8.0, 22000.0, 0.4),
                ]
            } else {
                Vec::new()
            };

            world.step(fixed_dt, &geometry, &impulses, &mut debug_lines);
            debug_lines.clear();

            for (idx, body) in world.bodies().iter() {
                let pos = body.position();
                let lin = body.linear_velocity();
                let ang = body.angular_velocity();
                let lin_speed = lin.magnitude();
                let ang_speed = ang.magnitude();

                let finite = pos.x.is_finite()
                    && pos.y.is_finite()
                    && pos.z.is_finite()
                    && lin.x.is_finite()
                    && lin.y.is_finite()
                    && lin.z.is_finite()
                    && ang.x.is_finite()
                    && ang.y.is_finite()
                    && ang.z.is_finite()
                    && lin_speed.is_finite()
                    && ang_speed.is_finite();

                if !finite || lin_speed > 1.0e8 || ang_speed > 1.0e8 {
                    failure = Some(format!(
                        "step={step} body={idx:?} pos=({:.3},{:.3},{:.3}) lin=({:.3},{:.3},{:.3}) ang=({:.3},{:.3},{:.3}) lin_speed={:.3} ang_speed={:.3}",
                        pos.x, pos.y, pos.z, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z, lin_speed, ang_speed
                    ));
                    break;
                }
            }

            if failure.is_some() {
                break;
            }
        }

        assert!(
            failure.is_none(),
            "physics state became unstable under grenade-like stress: {}",
            failure.unwrap_or_default()
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

    #[test]
    fn large_sphere_sliding_into_low_box_does_not_end_intersecting() {
        let geometry = FlatQuadGeometry::new(30.0);
        let box_half_extents = Vector3::new(3.0, 0.1, 3.0);
        let sphere_radius = 0.9;

        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        let mut world = PhysicsWorld::new(config);
        let mut debug_lines = DebugLines::default();

        let box_handle = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            0.0,
            box_half_extents.y + 0.01,
            0.0,
        )));
        let _ = world.attach_collider(
            box_handle,
            ColliderDesc::box_shape(box_half_extents)
                .density(10000.0)
                .restitution(0.0)
                .friction(0.7),
        );

        let sphere_handle = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-8.0, sphere_radius + 0.02, 0.0))
                .linear_velocity(Vector3::new(9.0, 0.0, 0.0)),
        );
        let _ = world.attach_collider(
            sphere_handle,
            ColliderDesc::sphere(sphere_radius)
                .density(1000.0)
                .restitution(0.0)
                .friction(0.2),
        );

        let fixed_dt = 1.0 / 60.0;
        let num_steps = (4.0 / fixed_dt) as usize;
        for _ in 0..num_steps {
            world.step(fixed_dt, &geometry, &[], &mut debug_lines);
            debug_lines.clear();
        }

        let sphere = world.body(sphere_handle).expect("sphere body should exist");
        let obstacle = world.body(box_handle).expect("box body should exist");

        let sphere_center_world = sphere.position();
        let box_center_world = obstacle.position();
        let sphere_center_in_box = obstacle
            .rotation()
            .inverse_transform_vector(&(sphere_center_world - box_center_world));
        let closest_on_box_local = Vector3::new(
            sphere_center_in_box
                .x
                .clamp(-box_half_extents.x, box_half_extents.x),
            sphere_center_in_box
                .y
                .clamp(-box_half_extents.y, box_half_extents.y),
            sphere_center_in_box
                .z
                .clamp(-box_half_extents.z, box_half_extents.z),
        );
        let separation_vec = sphere_center_in_box - closest_on_box_local;
        let separation_sq = separation_vec.magnitude_squared();
        let penetration = sphere_radius - separation_sq.sqrt();

        assert!(
            penetration <= 1.0e-3,
            "sphere should not end intersecting low box: penetration={penetration:.6}"
        );
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
        for i in 0..4 {
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
        let mut tail_max_manifold_churn = 0usize;
        let mut tail_max_contact_depth = 0.0f32;
        let mut tail_points_sum = 0usize;
        let mut tail_samples = 0usize;

        for step_idx in 0..num_steps {
            world.step(fixed_dt, &geometry, &[], &mut debug_lines);
            debug_lines.clear();

            if step_idx >= tail_start_step {
                let manifold = world.manifold_frame_stats();
                let manifold_churn =
                    manifold.point_adds + manifold.point_replacements + manifold.point_pruned;
                tail_max_manifold_churn = tail_max_manifold_churn.max(manifold_churn);
                tail_max_contact_depth = tail_max_contact_depth.max(
                    world
                        .contact_events()
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
                    "step {step_idx:4} m_points={} m_churn={} :",
                    manifold.points, manifold_churn,
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

        eprintln!("tail_max_speed={tail_max_speed:.6} final_max_speed={max_speed:.6}");
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

    // ═══════════════════════════════════════════════════════════════════════
    // Solver stability validation tests
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn sliding_sphere_decelerates_without_angular_spikes() {
        let scenario = SlidingSphereScenario::new();
        let cfg = BenchRunConfig {
            duration: 6.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "sliding_sphere");

        assert!(!run.samples.is_empty());

        // Sphere should decelerate and nearly stop
        let final_speed = run.samples.last().unwrap().linear_speed;
        assert!(
            final_speed < 0.5,
            "sliding sphere should have mostly stopped: final_speed={final_speed:.4}"
        );

        // Check for angular speed spikes. During smooth friction deceleration,
        // angular speed should remain modest (rolling friction, not torque spikes).
        let max_angular = run
            .samples
            .iter()
            .map(|s| s.angular_speed)
            .fold(0.0f32, f32::max);
        eprintln!("sliding_sphere max_angular_speed={max_angular:.6}");
        assert!(
            max_angular < 30.0,
            "sliding sphere angular speed spike: max_angular={max_angular:.4}"
        );

        // Angular speed should settle in the tail window
        let tail_start = (cfg.duration - 2.0).max(0.0);
        let tail_max_angular = run
            .samples
            .iter()
            .filter(|s| s.sim_time >= tail_start)
            .map(|s| s.angular_speed)
            .fold(0.0f32, f32::max);
        assert!(
            tail_max_angular < 1.0,
            "sliding sphere tail angular jitter: tail_max_angular={tail_max_angular:.4}"
        );
    }

    #[test]
    fn low_friction_ramp_sphere_slides_down() {
        let scenario = LowFrictionRampScenario::new();
        let cfg = BenchRunConfig {
            duration: 4.0,
            ..BenchRunConfig::default()
        };
        let run = run_scenario(&scenario, cfg);
        write_exports(&run, "low_friction_ramp");

        assert!(!run.samples.is_empty());

        // Sphere should be moving faster at the end than at the start.
        // It starts at rest and slides down the low-friction ramp.
        let early_window = 0.5;
        let early_max_speed = run
            .samples
            .iter()
            .filter(|s| s.sim_time <= early_window)
            .map(|s| s.linear_speed)
            .fold(0.0f32, f32::max);

        let tail_start = cfg.duration - 1.0;
        let tail_min_speed = run
            .samples
            .iter()
            .filter(|s| s.sim_time >= tail_start)
            .map(|s| s.linear_speed)
            .fold(f32::MAX, f32::min);

        eprintln!(
            "low_friction_ramp early_max_speed={early_max_speed:.4} tail_min_speed={tail_min_speed:.4}"
        );
        assert!(
            tail_min_speed > early_max_speed + 1.0,
            "sphere should accelerate on low-friction ramp: \
             early_max={early_max_speed:.4}, tail_min={tail_min_speed:.4}"
        );

        // Sphere should not have fallen through
        let min_y = run.samples.iter().map(|s| s.y).fold(f32::MAX, f32::min);
        assert!(min_y > -1.0, "sphere fell through ramp: min_y={min_y:.4}");
    }

    // ── Sphere pushing box across flat ground ────────────────────────────

    #[test]
    fn sphere_pushing_box_no_jitter() {
        let geometry = FlatQuadGeometry::new(50.0);
        let box_half_extents = Vector3::new(0.5, 0.5, 0.5);
        let sphere_radius = 0.5;

        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        config.deterministic_contact_ordering = true;
        let mut world = PhysicsWorld::new(config);
        let mut debug_lines = DebugLines::default();

        // Box: matching game properties
        let box_handle = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(2.0, box_half_extents.y + 0.01, 0.0))
                .gravity_scale(1.0)
                .linear_damping(0.01)
                .angular_damping(0.005),
        );
        let _ = world.attach_collider(
            box_handle,
            ColliderDesc::box_shape(box_half_extents)
                .density(50.5)
                .restitution(0.2)
                .friction(0.6),
        );

        // Player sphere: matching game properties
        let sphere_handle = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(-1.0, sphere_radius + 0.01, 0.0))
                .gravity_scale(1.0)
                .linear_damping(0.0)
                .angular_damping(1.0),
        );
        let _ = world.attach_collider(
            sphere_handle,
            ColliderDesc::sphere(sphere_radius)
                .density(30.0)
                .restitution(0.0)
                .friction(0.3),
        );

        // Match the game's timing: 240Hz physics, ~60fps render loop.
        // Velocity is set once per render frame, then update_contacts once,
        // then multiple substeps. This matches PhysicsSyncSystem::step_fixed.
        let fixed_dt = 1.0 / 240.0;
        let frame_dt = 1.0f32 / 60.0;
        let substeps_per_frame = (frame_dt / fixed_dt).round() as usize; // ~4
        let duration = 6.0;
        let num_frames = (duration / frame_dt) as usize;
        let push_speed = 4.0; // sustained forward speed simulating player input

        struct PushSample {
            sim_time: f32,
            sphere_x: f32,
            sphere_vx: f32,
            box_x: f32,
            box_vx: f32,
            contact_count: usize,
        }
        let mut samples: Vec<PushSample> = Vec::new();
        let mut sim_time = 0.0f32;

        for frame in 0..num_frames {
            // Once per render frame: set velocity from ECS (game does this
            // in sync_velocity_driven_from_ecs before step_fixed).
            let sphere_body = world.body(sphere_handle).expect("sphere exists");
            let current_vel = *sphere_body.linear_velocity();
            world.set_body_velocity(
                sphere_handle,
                Vector3::new(push_speed, current_vel.y, current_vel.z),
                Vector3::zeros(),
            );

            // Narrowphase once per frame (matches step_fixed).
            world.update_contacts(fixed_dt, &geometry, &[], &mut debug_lines);
            debug_lines.clear();

            // Count dynamic contacts between sphere and box this frame.
            let dyn_contacts = world
                .contact_events()
                .iter()
                .filter(|c| {
                    (c.body_b == sphere_handle && c.body_a == Some(box_handle))
                        || (c.body_b == box_handle && c.body_a == Some(sphere_handle))
                })
                .count();

            // Multiple substeps per frame.
            for _ in 0..substeps_per_frame {
                world.substep(fixed_dt, &geometry);
                sim_time += fixed_dt;

                let sb = world.body(sphere_handle).expect("sphere exists");
                let bb = world.body(box_handle).expect("box exists");
                samples.push(PushSample {
                    sim_time,
                    sphere_x: sb.position().x,
                    sphere_vx: sb.linear_velocity().x,
                    box_x: bb.position().x,
                    box_vx: bb.linear_velocity().x,
                    contact_count: dyn_contacts,
                });
            }

            // Print diagnostic at regular intervals.
            if frame % 30 == 0 {
                let sb = world.body(sphere_handle).expect("sphere exists");
                let bb = world.body(box_handle).expect("box exists");
                let gap = bb.position().x - sb.position().x;
                eprintln!(
                    "frame {:3} t={:.2} sphere(x={:.3} vx={:.3}) box(x={:.3} vx={:.3}) gap={:.3} contacts={}",
                    frame, sim_time, sb.position().x, sb.linear_velocity().x,
                    bb.position().x, bb.linear_velocity().x, gap, dyn_contacts
                );
            }
        }

        // ── Assertions ──

        // The box should have been pushed forward.
        let final_box = world.body(box_handle).expect("box exists");
        assert!(
            final_box.position().x > 3.0,
            "box should have been pushed forward: x={:.3}",
            final_box.position().x
        );

        // Identify the "push window": all samples where the sphere is close
        // enough to the box that contact SHOULD exist. Skip the initial
        // approach phase (first contact onset) and measure from 0.2s after
        // first contact to avoid the impact transient.
        let contact_distance = sphere_radius + box_half_extents.x;
        let proximity_threshold = contact_distance + 0.1;
        let first_contact_time = samples
            .iter()
            .find(|s| s.contact_count > 0)
            .map(|s| s.sim_time)
            .expect("sphere should contact box at least once");
        let push_window_start = first_contact_time + 0.3;
        let push_window: Vec<&PushSample> = samples
            .iter()
            .filter(|s| {
                s.sim_time >= push_window_start
                    && (s.box_x - s.sphere_x) < proximity_threshold
                    && (s.box_x - s.sphere_x) > 0.0
            })
            .collect();
        eprintln!(
            "push window: {} samples ({:.2}s - {:.2}s)",
            push_window.len(),
            push_window.first().map(|s| s.sim_time).unwrap_or(0.0),
            push_window.last().map(|s| s.sim_time).unwrap_or(0.0),
        );
        assert!(
            push_window.len() > 50,
            "push window too short — sphere may not be sustaining contact"
        );

        // Contact flickering: during the push window the sphere is right next
        // to the box. Every sample should have contact. Count dropouts.
        let contact_dropouts = push_window
            .iter()
            .filter(|s| s.contact_count == 0)
            .count();
        let contact_ratio =
            1.0 - (contact_dropouts as f32 / push_window.len() as f32);
        eprintln!(
            "contact dropouts in push window: {contact_dropouts}/{} ({:.1}% contact ratio)",
            push_window.len(),
            contact_ratio * 100.0
        );
        assert!(
            contact_dropouts == 0,
            "contact flickering: {contact_dropouts} frames lost contact while \
             sphere was within push distance (causes jitter in direction of motion)"
        );

        // Box x-velocity smoothness: compute per-substep acceleration and
        // check for spikes that would cause visible jitter.
        let mut max_box_ax = 0.0f32;
        let mut box_ax_spike_count = 0usize;
        let ax_spike_threshold = 30.0; // m/s² — smooth pushing should be gentle
        for w in push_window.windows(2) {
            let dt_between = w[1].sim_time - w[0].sim_time;
            if dt_between < 1e-6 {
                continue;
            }
            let ax = (w[1].box_vx - w[0].box_vx) / dt_between;
            let abs_ax = ax.abs();
            max_box_ax = max_box_ax.max(abs_ax);
            if abs_ax > ax_spike_threshold {
                box_ax_spike_count += 1;
            }
        }
        eprintln!(
            "box max x-accel in push window: {max_box_ax:.2} m/s², \
             spikes (>{ax_spike_threshold}): {box_ax_spike_count}"
        );
        assert!(
            box_ax_spike_count == 0,
            "box x-acceleration spikes during push: {box_ax_spike_count} frames \
             exceeded {ax_spike_threshold} m/s² (max={max_box_ax:.2})"
        );

        // Gap stability: the center-to-center gap should stay near the
        // contact distance during the push window — not oscillate.
        let push_gaps: Vec<f32> = push_window
            .iter()
            .map(|s| s.box_x - s.sphere_x)
            .collect();
        let gap_min = push_gaps.iter().fold(f32::MAX, |a, &b| a.min(b));
        let gap_max = push_gaps
            .iter()
            .fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        let gap_range = gap_max - gap_min;
        eprintln!(
            "gap in push window: min={gap_min:.4}, max={gap_max:.4}, \
             range={gap_range:.4}"
        );
        assert!(
            gap_range < 0.15,
            "sphere-box gap oscillation during push: range={gap_range:.4}"
        );
    }
}
