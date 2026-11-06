//! A plant's wood up close: a forked tree as it stands, one lifted clear of
//! its ground to show its roots, a branch snapped off lying on the ground,
//! and the tree it left.
//!
//! The meshes are the game's own looks (`plants::PlantDrawing`), posed by hand
//! rather than broken by a blast, so the sheet is the same every run;
//! `character_viewer branch_blast` shows the break under real physics.

use nalgebra::{Isometry3, Matrix4, Point3, Translation3, UnitQuaternion, Vector3};

use crate::app::spawnables::{bark_material, leaf_material};
use crate::core::error::EngineResult;
use crate::plants::{BarkSpec, LimbBase, LollipopSpec, Plant, PlantDrawing, PlantLook};
use crate::rendering::material::{MaterialId, SurfaceParams};
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::flat_ground::ground;
use crate::resources::textures::TextureHandle;

/// The seed every shot's tree grows from.
const SEED: u32 = 4;
/// Material ids standing for the leaves and the bark in the drawing.
const LOOK: PlantLook = PlantLook {
    leaves: MaterialId(0),
    bark: MaterialId(1),
};

pub struct Plants;

impl VisualScene for Plants {
    fn name(&self) -> &str {
        "plants"
    }

    fn description(&self) -> &str {
        "A forked tree standing, lifted to show its roots, a branch snapped off it, and what it left."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let spec = BarkSpec::default();
        let size = spec.texture_size;
        let tile = ctx
            .textures
            .create_from_rgba(size, size, &spec.texture(SEED), true)?;
        let look = Look {
            bark: bark_material(tile.clone()).surface_params(),
            leaves: leaf_material().surface_params(),
            tile,
        };
        let plant = LollipopSpec {
            crowns: 3,
            ..LollipopSpec::default()
        }
        .grow(SEED);
        let drawing = PlantDrawing::grow(&plant, SEED, LOOK);
        let all: Vec<usize> = (0..plant.limbs.limbs.len()).collect();
        let branch = plant.masses[1].limb;
        let kept: Vec<usize> = all.iter().copied().filter(|&l| l != branch).collect();
        let fork = plant.limbs.limbs[branch].base;

        // The piece holding `limbs` with the plant's frame at `pose`, on
        // the ground.
        let draw = |limbs: &[usize], pose: Isometry3<f32>| {
            let mut meshes = look.meshes(&drawing, &plant, limbs, pose);
            meshes.push(ground());
            meshes
        };
        let standing = draw(&all, Isometry3::identity());
        let lifted = draw(&all, Isometry3::translation(0.0, 0.5, 0.0));
        // The branch lying on the ground, its broken base towards the camera.
        let limb = plant.limbs.limbs[branch];
        let lie = UnitQuaternion::rotation_between(
            &(limb.tip - limb.base),
            &Vector3::new(1.0, 0.05, 0.0),
        )
        .unwrap_or_else(UnitQuaternion::identity);
        let laid = Isometry3::from_parts(Translation3::new(0.0, 0.3, 0.0), lie)
            * Isometry3::translation(-limb.base.x, -limb.base.y, -limb.base.z);
        let snapped = draw(&[branch], laid);
        let left = draw(&kept, Isometry3::identity());

        let environment = SceneEnvironment::default().with_sun(Vector3::new(-0.5, 0.7, 0.5));
        let shot = |label: &str, eye: Point3<f32>, target: Point3<f32>, meshes: Vec<SceneMesh>| {
            SceneShot::new(label, SceneCamera::looking_at(eye, target).with_fov(45.0))
                .with_environment(environment.clone())
                .with_meshes(meshes)
        };
        Ok(vec![
            shot(
                "standing",
                Point3::new(4.5, 2.2, 4.5),
                Point3::new(0.0, 1.6, 0.0),
                standing,
            ),
            shot(
                "foot",
                Point3::new(0.9, 0.35, 0.9),
                Point3::new(0.0, 0.1, 0.0),
                draw(&all, Isometry3::identity()),
            ),
            shot(
                "roots",
                Point3::new(1.3, 0.6, 1.3),
                Point3::new(0.0, 0.35, 0.0),
                lifted,
            ),
            shot(
                "snapped",
                Point3::new(-0.35, 0.45, 0.35),
                Point3::new(0.0, 0.3, 0.0),
                snapped,
            ),
            shot(
                "left",
                Point3::from(fork.coords + Vector3::new(0.7, 0.3, 0.7)),
                fork,
                left,
            ),
        ])
    }
}

/// How a plant's materials are drawn on the bench.
struct Look {
    tile: TextureHandle,
    bark: SurfaceParams,
    leaves: SurfaceParams,
}

impl Look {
    /// The piece of `plant` holding `limbs`, its frame at `pose`: each
    /// limb's bark, snapped where it broke off a limb not in the piece, and
    /// each crown's leaves, all drawn in the plant's frame.
    fn meshes(
        &self,
        drawing: &PlantDrawing,
        plant: &Plant,
        limbs: &[usize],
        pose: Isometry3<f32>,
    ) -> Vec<SceneMesh> {
        let plant_frame = Isometry3::identity();
        let bark = limbs.iter().map(|&l| {
            let base = if plant.limbs.broken_off(l, limbs) {
                LimbBase::Snapped
            } else {
                LimbBase::Rounded
            };
            drawing.bark(l, base, &plant_frame)
        });
        let leaves = plant
            .masses_on(limbs)
            .into_iter()
            .map(|m| drawing.leaves(m, &plant_frame));
        let transform: Matrix4<f32> = pose.to_homogeneous();
        bark.chain(leaves)
            .flat_map(|look| look.primitives.clone())
            .map(|primitive| {
                let mesh =
                    SceneMesh::new(primitive.vertices, primitive.indices).with_transform(transform);
                if primitive.material == LOOK.bark {
                    mesh.with_surface(self.bark).with_texture(self.tile.clone())
                } else {
                    mesh.with_surface(self.leaves)
                }
            })
            .collect()
    }
}
