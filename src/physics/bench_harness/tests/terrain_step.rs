//! Convex hull vs real marching-cubes terrain at a step edge.
//!
//! Unlike the synthetic geometries in `geometry.rs`, this builds an actual
//! voxel segment and meshes it, so the hull meets the same face layout the
//! game produces: many small triangles with mixed normals around the step.

use nalgebra::{Point3, UnitQuaternion, Vector3};
use std::sync::Arc;

use crate::collision::ray_triangle::ray_triangle;
use crate::collision::AABB;
use crate::debug::DebugLines;
use crate::level::{Extent, Terrain, TerrainFeature};
use crate::physics::{ColliderDesc, PhysicsWorld, RigidBodyDesc, RigidBodyHandle, StaticGeometry};
use crate::terrain::{
    generate_terrain, ChunkGrid, DurabilityConfig, Segment, SegmentFrame, TerrainWorld,
};

use super::super::scenarios::menhir_hull_for_tests;

/// Voxel edge length of the test terrain.
const VOXEL: f32 = 2.0;

/// Build terrain that is flat at y=0 and steps up to `step_height` for
/// x >= 0, generated and meshed exactly like level terrain.
fn stepped_terrain(step_height: f32) -> TerrainWorld {
    let terrain = Terrain {
        voxel_size: VOXEL,
        bounds: Extent {
            min: (-10.0, -6.0, -10.0),
            max: (10.0, 6.0, 10.0),
        },
        base_height: 0.0,
        material_layers: Vec::new(),
        features: vec![TerrainFeature::Plateau {
            min: (0.0, -10.0),
            max: (10.0, 10.0),
            height: step_height,
        }],
        volumes: Vec::new(),
    };

    let bounds = terrain.bounds.to_aabb();
    let mut grid = ChunkGrid::new(terrain.voxel_size);
    generate_terrain(&mut grid, &terrain, &DurabilityConfig::default(), &bounds);

    TerrainWorld::from_segments_headless(vec![Segment::new(
        "step",
        SegmentFrame::identity(),
        grid,
        Vec::new(),
    )])
}

/// Height of the meshed terrain surface directly below `(x, z)`, found by
/// casting a ray down through the collision triangles.
fn mesh_surface_y(terrain: &TerrainWorld, x: f32, z: f32) -> f32 {
    let probe = AABB::new(
        Point3::new(x - 0.01, -8.0, z - 0.01),
        Point3::new(x + 0.01, 8.0, z + 0.01),
    );
    let patch = terrain.query_region(&probe);
    let origin = Point3::new(x, 8.0, z);
    let mut best = f32::NEG_INFINITY;
    for t in &patch.triangles {
        if let Some(hit) = ray_triangle(origin, -Vector3::y(), &t.triangle, 32.0) {
            best = best.max(origin.y - hit.t);
        }
    }
    best
}

struct Sample {
    time: f32,
    pos: Point3<f32>,
    contacts: usize,
    max_depth: f32,
}

/// Drop a menhir-shaped hull next to the step and tip it toward the edge.
fn topple_menhir(terrain: &TerrainWorld, start_x: f32, tip_rate: f32) -> Vec<Sample> {
    let surface = mesh_surface_y(terrain, start_x, 0.0);
    let mut config = crate::physics::world::PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);

    let hull = Arc::new(menhir_hull_for_tests(1.0, 0.4, 0.25));
    let body: RigidBodyHandle = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(start_x, surface + 1.05, 0.0))
            .rotation(UnitQuaternion::identity())
            .angular_velocity(Vector3::new(0.0, 0.0, -tip_rate)),
    );
    let collider = ColliderDesc::convex_hull(hull)
        .density(2700.0)
        .restitution(0.0)
        .friction(0.6);
    let _ = world.attach_collider(body, collider);

    let dt = 1.0 / 60.0;
    let mut debug = DebugLines::default();
    let mut samples = Vec::new();
    for step in 0..600 {
        world.update_contacts(dt, terrain as &dyn StaticGeometry, &[], &mut debug);
        debug.clear();
        world.substep(dt, terrain as &dyn StaticGeometry, &[]);
        let b = world.body(body).unwrap();
        let max_depth = world
            .contact_events()
            .iter()
            .map(|c| c.depth)
            .fold(0.0f32, f32::max);
        samples.push(Sample {
            time: step as f32 * dt,
            pos: b.position(),
            contacts: world.contact_events().len(),
            max_depth,
        });
    }
    samples
}

#[test]
fn menhir_topples_onto_terrain_step_without_sinking() {
    let terrain = stepped_terrain(2.0);

    for &start_x in &[-4.0f32, -3.6, -3.2, -2.8, -2.4, -2.0] {
        for &tip in &[0.5f32, 1.0, 1.5, 2.0, 3.0] {
            let samples = topple_menhir(&terrain, start_x, tip);
            let min_y = samples
                .iter()
                .map(|s| s.pos.y)
                .fold(f32::INFINITY, f32::min);
            let last = samples.last().unwrap();
            let max_depth = samples.iter().map(|s| s.max_depth).fold(0.0f32, f32::max);
            let max_jump = samples
                .windows(2)
                .map(|w| (w[1].pos - w[0].pos).magnitude())
                .fold(0.0f32, f32::max);
            eprintln!(
                "start_x={start_x:+.1} tip={tip:.1} min_y={min_y:+.3} max_depth={max_depth:.3} \
                 max_jump={max_jump:.3} final=({:+.2},{:+.2},{:+.2})",
                last.pos.x, last.pos.y, last.pos.z
            );

            assert!(
                min_y > 0.0,
                "hull sank into the terrain step (start_x={start_x}, tip={tip}): min_y={min_y:.3}"
            );
            assert!(
                max_depth < 0.25,
                "phantom deep contact at the step edge (start_x={start_x}, tip={tip}): \
                 max_depth={max_depth:.3}"
            );
            assert!(
                max_jump < 0.2,
                "body teleported in a single step (start_x={start_x}, tip={tip}): \
                 max_jump={max_jump:.3}"
            );
        }
    }
}

/// Regression: replay the captured in-game "pop" pose against the captured
/// step patch. The stone starts deeply penetrating the slope face, which is a
/// state the solver has to resolve *gradually* — NGS must not translate the
/// body metres per frame to get rid of it.
#[test]
fn pop_replay_captured_pose_resolves_without_teleporting() {
    use crate::collision::{MeshPatch, PatchTriangle, Triangle};

    struct CapturedPatch(MeshPatch);
    impl StaticGeometry for CapturedPatch {
        fn query_region(&self, _aabb: &AABB) -> MeshPatch {
            self.0.clone()
        }
    }

    let quad = |a: Point3<f32>, b: Point3<f32>, c: Point3<f32>, d: Point3<f32>| {
        vec![
            PatchTriangle {
                triangle: Triangle::new(a, b, c),
                neighbors: [None; 3],
            },
            PatchTriangle {
                triangle: Triangle::new(a, c, d),
                neighbors: [None; 3],
            },
        ]
    };

    let mut triangles = Vec::new();
    triangles.extend(quad(
        Point3::new(12.0, -3.0, -4.0),
        Point3::new(10.0, -3.0, -4.0),
        Point3::new(10.0, -3.0, -2.0),
        Point3::new(12.0, -3.0, -2.0),
    ));
    triangles.extend(quad(
        Point3::new(12.0, -3.0, -2.0),
        Point3::new(10.0, -3.0, -2.0),
        Point3::new(10.0, -3.0, 0.0),
        Point3::new(12.0, -3.0, 0.0),
    ));
    triangles.extend(quad(
        Point3::new(9.0, -2.0, -4.0),
        Point3::new(10.0, -2.0, -5.0),
        Point3::new(10.0, -1.0, -6.0),
        Point3::new(8.0, -1.0, -4.0),
    ));
    triangles.push(PatchTriangle {
        triangle: Triangle::new(
            Point3::new(10.0, -3.0, -4.0),
            Point3::new(10.0, -2.0, -5.0),
            Point3::new(9.0, -2.0, -4.0),
        ),
        neighbors: [None; 3],
    });

    let geometry = CapturedPatch(MeshPatch { triangles });

    let mut config = crate::physics::world::PhysicsConfig::default();
    config.sleep.enabled = false;
    let mut world = PhysicsWorld::new(config);
    let hull = Arc::new(menhir_hull_for_tests(4.0, 1.8, 1.0));
    let body = world.create_body(
        RigidBodyDesc::dynamic()
            .position(Point3::new(10.586787, -1.690402, -4.410592))
            .rotation(UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                0.335174, 0.163065, -0.581374, 0.723237,
            ))),
    );
    let _ = world.attach_collider(body, ColliderDesc::convex_hull(hull).density(2700.0));

    let dt = 1.0 / 60.0;
    let mut debug = DebugLines::default();
    let mut previous = world.body(body).unwrap().position();
    let mut max_jump = 0.0f32;
    for step in 0..30 {
        world.update_contacts(dt, &geometry as &dyn StaticGeometry, &[], &mut debug);
        debug.clear();
        world.substep(dt, &geometry as &dyn StaticGeometry, &[]);
        let b = world.body(body).unwrap();
        let jump = (b.position() - previous).magnitude();
        max_jump = max_jump.max(jump);
        eprintln!(
            "step {step}: pos={:?} jump={jump:.3} v={:.3} contacts={}",
            b.position(),
            b.linear_velocity().magnitude(),
            world.contact_events().len()
        );
        previous = b.position();
    }

    // The body's speed stays around 1 m/s, so anything beyond ~0.02 m in a
    // 1/60 s step is positional correction teleporting it, not motion.
    assert!(
        max_jump < 0.05,
        "NGS teleported the body {max_jump:.3} m in one step while resolving \
         the captured deep contact"
    );
}
