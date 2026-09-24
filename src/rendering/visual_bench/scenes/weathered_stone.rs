//! The voussoir arch's stone, close enough to judge.
//!
//! A level shows the arch from across a courtyard, where the shape of the
//! blocks carries it; this sheet shows one block from arm's length, where the
//! finish does. Four questions, one shot each:
//!
//! - `voussoir` — does a single block read as old stone: rounded, chipped
//!   arrises, uneven faces, streaks and lichen, cracks that catch the light?
//! - `cracked` — does a break read as a break: raw, pale and sharp-edged
//!   against the weathered faces, the two halves still fitting each other?
//! - `split` — the same break opened up, so the fresh faces can be seen.
//! - `ring` — several blocks side by side as the arch lays them, for the
//!   joints between them and the variety from block to block.
//! - `collapse` — the whole arch after an abutment is pulled from under it,
//!   simulated by the game's own physics, cleave and fracture systems: which
//!   stones cracked, and how the pieces lie.
//!
//! Everything here is the game's own: the arch's own geometry, texture and
//! substances, the weathered mesh, and the stone's own cleave rule.

use nalgebra::{Matrix4, Point3, Vector2, Vector3};
use specs::{Join, RunNow, World, WorldExt};

use crate::app::spawnables::shared::models::PieceHull;
use crate::app::spawnables::shared::models::PiecePlacement;
use crate::app::spawnables::Spawnable;
use crate::app::spawnables::{
    stone_cleaving, weathered_box_mesh, weathered_hull_mesh, StoneTexture, VoussoirArchDef,
};
use crate::cleave::{BrittleSolid, CleavePiece, SolidCleaveSystem};
use crate::collision::convex_hull::ConvexHull;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::debug::{DebugLines, DebugLog};
use crate::fracture::{CompoundFracture, FractureSystem};
use crate::physics::bench_harness::geometry::FlatQuadGeometry;
use crate::physics::stepping::{SequentialStepper, Stepper};
use crate::physics::ColliderShape;
use crate::physics::{PhysicsConfig, PhysicsImpulseQueue, PhysicsWorld};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::rendering::material::SurfaceParams;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::resources::textures::TextureHandle;
use crate::systems::PhysicsResource;
use crate::time::Time;

/// Half-width of the ground plane.
const GROUND_HALF_EXTENT: f32 = 12.0;

/// Fixed, so the sheet shows the same break every time it is rendered.
const CLEAVE_SALT: u32 = 5;

/// How far the halves of the opened break are pulled apart, in metres.
const OPENED: f32 = 0.35;

/// Where the blocks stand: away from the origin, as a level's would.
const SITE: Vector3<f32> = Vector3::new(31.0, 0.0, 17.0);

pub struct WeatheredStone;

impl VisualScene for WeatheredStone {
    fn name(&self) -> &str {
        "weathered_stone"
    }

    fn description(&self) -> &str {
        "The voussoir arch's stone up close: a block, a crack, and a ring of blocks."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let arch = VoussoirArchDef::default();
        let recipe = StoneTexture::ARCH;
        let spread = recipe.spread();
        let size = spread.texture_size(recipe.tile_size);
        let substance = arch.voussoir_substance();
        let texture = ctx.textures.create_from_rgba(
            size,
            size,
            &recipe
                .pattern
                .bake_spread(recipe.tile_size, &substance.palette, recipe.seed, spread),
            true,
        )?;
        let relief = recipe.pattern.relief_depth() / spread.tiles() as f32;
        let surface = substance
            .material(texture.clone())
            .with_relief(relief)
            .surface_params();
        let stone = Stone {
            texture,
            surface,
            arch,
        };

        let environment = SceneEnvironment::default().with_sun(Vector3::new(-0.55, 0.62, 0.56));
        let close = |target: Vector3<f32>| {
            SceneCamera::looking_at(
                Point3::from(SITE + target + Vector3::new(0.75, 0.55, 1.05)),
                Point3::from(SITE + target),
            )
            .with_fov(42.0)
        };

        let crown = stone.arch.num_voussoirs / 2;
        let (hull, _) = stone.arch.voussoir(crown);
        let lift = Vector3::new(0.0, 0.3, 0.0);
        let voussoir = vec![ground(), stone.whole(&hull, lift)];
        let cracked = stone.cracked(&hull, lift, 0.0);
        let split = stone.cracked(&hull, lift, OPENED);

        let ring_camera = SceneCamera::looking_at(
            Point3::from(SITE + Vector3::new(3.6, 2.1, 4.4)),
            Point3::from(SITE + Vector3::new(1.4, 2.5, 0.0)),
        )
        .with_fov(55.0);
        let collapse_camera = SceneCamera::looking_at(
            Point3::from(SITE + Vector3::new(0.6, 2.4, 4.2)),
            Point3::from(SITE + Vector3::new(0.6, 0.4, 0.0)),
        )
        .with_fov(55.0);

        Ok(vec![
            SceneShot::new("voussoir", close(lift))
                .with_environment(environment.clone())
                .with_meshes(voussoir),
            SceneShot::new("cracked", close(lift))
                .with_environment(environment.clone())
                .with_meshes(cracked),
            SceneShot::new("split", close(lift))
                .with_environment(environment.clone())
                .with_meshes(split),
            SceneShot::new("ring", ring_camera)
                .with_environment(environment.clone())
                .with_meshes(stone.ring()),
            SceneShot::new("collapse", collapse_camera)
                .with_environment(environment)
                .with_meshes(stone.collapse()),
        ])
    }
}

/// The arch's stone, ready to draw blocks in.
struct Stone {
    texture: TextureHandle,
    surface: SurfaceParams,
    arch: VoussoirArchDef,
}

impl Stone {
    /// One whole block at `at` (relative to the site), drawn as the game
    /// draws it at spawn: textured from where it stands.
    fn whole(&self, hull: &ConvexHull, at: Vector3<f32>) -> SceneMesh {
        let anchor = SITE + at;
        let whole = hull.translated(anchor);
        let (vertices, indices) = weathered_hull_mesh(
            &PieceHull::new(hull, anchor).within(Some(&whole)),
            self.arch.surface_uvs(),
        );
        self.mesh(vertices, indices, anchor)
    }

    /// The block cracked by the stone's own rule, its halves drawn as the
    /// fracture system draws them and pulled `opened` metres apart.
    fn cracked(&self, hull: &ConvexHull, at: Vector3<f32>, opened: f32) -> Vec<SceneMesh> {
        let anchor = SITE + at;
        let whole = hull.translated(anchor);
        let shape = ColliderShape::ConvexHull {
            hull: std::sync::Arc::new(hull.clone()),
        };
        let pieces = stone_cleaving(1000.0)
            .cleave(&shape, Vector3::new(0.1, 0.2, 0.0), CLEAVE_SALT)
            .unwrap_or_default();

        let mut meshes = vec![ground()];
        for CleavePiece {
            hull: piece,
            centre,
        } in pieces
        {
            let (vertices, indices) = weathered_hull_mesh(
                &PieceHull::new(&piece, anchor + centre).within(Some(&whole)),
                self.arch.surface_uvs(),
            );
            let apart = centre.normalize() * opened * 0.5;
            meshes.push(self.mesh(vertices, indices, anchor + centre + apart));
        }
        meshes
    }

    /// The lower right-hand quarter of the arch: an abutment and the
    /// voussoirs it carries, as the arch lays them.
    fn ring(&self) -> Vec<SceneMesh> {
        let arch = &self.arch;
        let centre = Vector3::new(0.0, arch.abutment_height, 0.0);
        let mut meshes = vec![ground()];
        for index in 0..=arch.num_voussoirs / 2 {
            let (hull, offset) = arch.voussoir(index);
            meshes.push(self.whole(&hull, centre + offset));
        }
        let half = Vector3::new(
            arch.thickness / 2.0,
            arch.abutment_height / 2.0,
            arch.depth / 2.0,
        );
        let at = Vector3::new(
            arch.inner_radius + arch.thickness / 2.0,
            arch.abutment_height / 2.0,
            0.0,
        );
        let (vertices, indices) =
            weathered_box_mesh(&PiecePlacement::new(half, SITE + at), arch.surface_uvs());
        meshes.push(self.mesh(vertices, indices, SITE + at));
        meshes
    }

    /// The arch built at the site, an abutment pulled from under it, and the
    /// fall run to rest through the game's own systems. Each piece is drawn
    /// with the model the fracture system gave it, where physics left it.
    fn collapse(&self) -> Vec<SceneMesh> {
        let arch = VoussoirArchDef {
            base: (SITE.x, SITE.y, SITE.z),
            ..VoussoirArchDef::default()
        };
        let mut world = collapse_world();
        let stones = arch.spawn(&mut world, &[MaterialId(0), MaterialId(1)]);
        settle(&mut world, SETTLE_FRAMES);
        let abutment = *stones.last().expect("the arch has abutments");
        let body = world
            .read_storage::<RigidBodyComponent>()
            .get(abutment)
            .expect("an abutment has a body")
            .0;
        world
            .write_resource::<PhysicsResource>()
            .world
            .remove_body(body);
        let _ = world.delete_entity(abutment);
        settle(&mut world, FALL_FRAMES);

        let physics = world.read_resource::<PhysicsResource>();
        let bodies = world.read_storage::<RigidBodyComponent>();
        let models = world.read_storage::<ModelInstance>();
        let mut meshes = vec![ground()];
        for (body, model) in (&bodies, &models).join() {
            let Some(body) = physics.world.body(body.0) else {
                continue;
            };
            let placed = Matrix4::new_translation(&body.position().coords)
                * body.rotation().to_homogeneous();
            for part in &model.model.parts {
                let transform = placed * part.local_transform.to_matrix();
                for primitive in &part.primitives {
                    meshes.push(
                        SceneMesh::new(primitive.vertices.clone(), primitive.indices.clone())
                            .with_transform(transform)
                            .with_texture(self.texture.clone())
                            .with_surface(self.surface),
                    );
                }
            }
        }
        meshes
    }

    fn mesh(&self, vertices: Vec<Vertex>, indices: Vec<u32>, at: Vector3<f32>) -> SceneMesh {
        SceneMesh::new(vertices, indices)
            .at(at)
            .with_texture(self.texture.clone())
            .with_surface(self.surface)
    }
}

/// A matte quad at `y = 0` for the blocks to sit on and shadow.
fn ground() -> SceneMesh {
    let e = GROUND_HALF_EXTENT;
    let colour = Colour::new(0.34, 0.38, 0.30, 1.0).to_vec4();
    let normal = Vector3::y();

    let corners = [
        (Vector3::new(-e, 0.0, -e), [0.0, 0.0]),
        (Vector3::new(e, 0.0, -e), [1.0, 0.0]),
        (Vector3::new(e, 0.0, e), [1.0, 1.0]),
        (Vector3::new(-e, 0.0, e), [0.0, 1.0]),
    ];

    let vertices: Vec<Vertex> = corners
        .iter()
        .map(|(position, uv)| Vertex {
            pos: *position + SITE,
            color: colour,
            tex_coords: Vector2::new(uv[0], uv[1]),
            normal,
            ao: 1.0,
        })
        .collect();

    SceneMesh::new(vertices, vec![0, 1, 2, 0, 2, 3])
}

/// Frames the arch stands before its abutment goes, to bed in.
const SETTLE_FRAMES: usize = 30;

/// Frames the fall is given to come to rest.
const FALL_FRAMES: usize = 360;

const FRAME_DT: f32 = 1.0 / 60.0;

/// A world with what the arch and its fall need, and no more.
///
/// Sleep is off: a removed body wakes nothing that was resting on it, and
/// the ring would hang in the air.
fn collapse_world() -> World {
    let mut world = World::new();
    world.register::<Position>();
    world.register::<Velocity>();
    world.register::<Orientation>();
    world.register::<RigidBodyComponent>();
    world.register::<ModelInstance>();
    world.register::<Renderable>();
    world.register::<CompoundFracture>();
    world.register::<BrittleSolid>();
    world.insert(Time::fixed(FRAME_DT));
    world.insert(PhysicsImpulseQueue::default());
    world.insert(DebugLog::default());
    let mut config = PhysicsConfig::default();
    config.sleep.enabled = false;
    world.insert(PhysicsResource::new(
        PhysicsWorld::new(config),
        Box::new(SequentialStepper::new(FRAME_DT, 4)),
    ));
    world
}

/// Run physics and the break pipeline for `frames` frames on flat ground.
fn settle(world: &mut World, frames: usize) {
    // Centred on the world origin, so wide enough to reach the site.
    let geometry = FlatQuadGeometry::new(SITE.magnitude() + GROUND_HALF_EXTENT);
    let mut stepper = SequentialStepper::new(FRAME_DT, 4);
    for _ in 0..frames {
        {
            let mut physics = world.write_resource::<PhysicsResource>();
            let mut debug = DebugLines::default();
            stepper.step(
                &mut physics.world,
                FRAME_DT,
                &geometry,
                &[],
                &[],
                &mut debug,
            );
        }
        SolidCleaveSystem.run_now(world);
        FractureSystem.run_now(world);
        world.maintain();
    }
}
