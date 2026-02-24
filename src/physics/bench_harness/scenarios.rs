use nalgebra::{Point3, UnitQuaternion, Vector3};

use super::framework::PhysicsBenchScenario;
use super::geometry::*;
use crate::physics::world::PhysicsConfig;
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, StaticGeometry};

// ═══════════════════════════════════════════════════════════════════════════
// Scenarios: mesh pipeline (sphere/box vs static terrain)
// ═══════════════════════════════════════════════════════════════════════════

/// Sphere dropped onto a flat surface. Exercises sphere-patch manifold
/// generation and the mesh-aware pipeline (seam filter -> sphere_patch).
#[derive(Debug, Clone)]
pub(crate) struct FlatSphereRestScenario {
    pub restitution: f32,
    pub friction: f32,
    pub radius: f32,
    pub spawn_height: f32,
    geometry: FlatQuadGeometry,
}

impl FlatSphereRestScenario {
    pub fn new(restitution: f32) -> Self {
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
pub(crate) struct SphereSlideScenario {
    pub radius: f32,
    geometry: FlatQuadGeometry,
}

impl SphereSlideScenario {
    pub fn new() -> Self {
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
pub(crate) struct FlatBoxRestScenario {
    pub restitution: f32,
    pub friction: f32,
    pub half_extents: Vector3<f32>,
    pub spawn_height: f32,
    geometry: FlatQuadGeometry,
}

impl FlatBoxRestScenario {
    pub fn new(restitution: f32) -> Self {
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
pub(crate) struct SphereOnRampScenario {
    pub restitution: f32,
    pub radius: f32,
    geometry: RampGeometry,
}

impl SphereOnRampScenario {
    pub fn new() -> Self {
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
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, 3.0, 3.0))
                .linear_damping(0.3)
                .angular_damping(0.3),
        );
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
pub(crate) struct BoxOnRampScenario {
    pub restitution: f32,
    pub half_extents: Vector3<f32>,
    geometry: RampGeometry,
}

impl BoxOnRampScenario {
    pub fn new() -> Self {
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
pub(crate) struct BoxOnStepScenario {
    pub restitution: f32,
    pub half_extents: Vector3<f32>,
    geometry: StepGeometry,
}

impl BoxOnStepScenario {
    pub fn new() -> Self {
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
pub(crate) struct HeavySphereOnPlatformScenario {
    pub platform_half_extents: Vector3<f32>,
    pub platform_density: f32,
    pub sphere_radius: f32,
    pub sphere_density: f32,
    geometry: FlatQuadGeometry,
}

impl HeavySphereOnPlatformScenario {
    pub fn new() -> Self {
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
pub(crate) struct SphereInBowlScenario {
    pub radius: f32,
    geometry: BowlGeometry,
}

impl SphereInBowlScenario {
    pub fn new() -> Self {
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
pub(crate) struct BoxSlidesDownWallScenario {
    /// Half-extents: x = thin (perpendicular to wall), y = long (parallel).
    pub half_extents: Vector3<f32>,
    /// Initial tilt angle from vertical (radians).
    pub theta0: f32,
    /// Friction coefficient for both wall and floor contacts.
    pub friction: f32,
    geometry: WallAndFloorGeometry,
}

impl BoxSlidesDownWallScenario {
    pub fn new(friction: f32) -> Self {
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
pub(crate) struct SphereSphereCollisionScenario {
    pub restitution: f32,
    geometry: FlatQuadGeometry,
}

impl SphereSphereCollisionScenario {
    pub fn new(restitution: f32) -> Self {
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
pub(crate) struct SphereObbCollisionScenario {
    pub restitution: f32,
    geometry: FlatQuadGeometry,
}

impl SphereObbCollisionScenario {
    pub fn new(restitution: f32) -> Self {
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
pub(crate) struct ObbObbCollisionScenario {
    pub restitution: f32,
    geometry: FlatQuadGeometry,
}

impl ObbObbCollisionScenario {
    pub fn new(restitution: f32) -> Self {
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
pub(crate) struct BoxGridScenario {
    pub grid_size: usize,
    pub half_extent: f32,
    pub spacing: f32,
    geometry: FlatQuadGeometry,
}

impl BoxGridScenario {
    pub fn new(grid_size: usize) -> Self {
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
pub(crate) struct HighSpeedSphereCcdScenario {
    geometry: FlatQuadGeometry,
}

impl HighSpeedSphereCcdScenario {
    pub fn new() -> Self {
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
pub(crate) struct SlidingSphereScenario {
    geometry: FlatQuadGeometry,
}

impl SlidingSphereScenario {
    pub fn new() -> Self {
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
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::new(0.0, radius + 0.01, 0.0))
                .linear_damping(0.4)
                .angular_damping(0.5),
        );
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
pub(crate) struct LowFrictionRampScenario {
    geometry: RampGeometry,
}

impl LowFrictionRampScenario {
    pub fn new() -> Self {
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
