//! The temple's loose marble, whole and cracked.
//!
//! The meshes are the game's own (`TempleDef::marble_samples`): a column, a
//! beam and a gable, each beside itself cracked as a hard landing would
//! crack it, the halves drawn a little apart. That is how to judge what a
//! half keeps of its piece's dressed face and how its break reads, which a
//! level viewer cannot show: nothing there falls.

use nalgebra::{Point3, Vector3};

use crate::app::spawnables::{TempleDef, TEMPLE_MARBLE, TEMPLE_MARBLE_PATTERN};
use crate::core::error::EngineResult;
use crate::model::Look;
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::rendering::visual_bench::scenes::flat_ground::ground;
use crate::resources::textures::TextureHandle;

/// Side of the baked marble, texels, and its seed: the temple's own.
const TEXTURE_SIZE: u32 = 256;
const SEED: u32 = 202;
/// How far a half is drawn from its piece's middle, against where it lies.
const SPREAD: f32 = 1.5;
/// How far apart a piece and its cracked twin stand, and how far back one
/// kind of piece stands from the next, in the piece's own reach.
const TWIN: f32 = 2.6;
const ROW: f32 = 6.0;
/// How far from a pair the camera stands, in the piece's reach.
const VIEW: f32 = 4.2;
/// Where the blow that cracks a piece comes from, in its frame.
const BLOW: Vector3<f32> = Vector3::new(1.0, 0.4, 0.6);

pub struct Marble;

/// `look`'s meshes with its frame at `at`, in the temple's marble.
fn meshes(look: &Look, texture: &TextureHandle, at: Vector3<f32>) -> Vec<SceneMesh> {
    let surface = TEMPLE_MARBLE.material(texture.clone()).surface_params();
    look.primitives
        .iter()
        .map(|primitive| {
            SceneMesh::new(primitive.vertices.clone(), primitive.indices.clone())
                .at(at)
                .with_texture(texture.clone())
                .with_surface(surface)
        })
        .collect()
}

impl VisualScene for Marble {
    fn name(&self) -> &str {
        "marble"
    }

    fn description(&self) -> &str {
        "A temple's column, beam and gable, each whole and cracked in two."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let texture = ctx.textures.create_from_rgba(
            TEXTURE_SIZE,
            TEXTURE_SIZE,
            &TEMPLE_MARBLE_PATTERN.bake(TEXTURE_SIZE, &TEMPLE_MARBLE.palette, SEED),
            true,
        )?;
        let temple = TempleDef {
            pos: (0.0, 0.0, 0.0),
            column_height: TempleDef::default_column_height(),
            front_columns: 2,
            side_columns: 2,
        };
        let materials = [MaterialId(0), MaterialId(1), MaterialId(2)];

        let samples = temple.marble_samples(&materials, BLOW);
        // Each kind's reach, and where its pair's middle stands: every pair
        // clear of the ground and of the pair before it.
        let mut pairs = Vec::new();
        let mut back = 0.0;
        for sample in &samples {
            let corners = sample
                .whole
                .primitives
                .iter()
                .flat_map(|mesh| &mesh.vertices);
            let reach = corners.map(|vertex| vertex.pos.norm()).fold(0.0, f32::max);
            back += ROW * reach;
            pairs.push((reach, Vector3::new(0.0, 1.2 * reach, -back)));
        }
        let scene = || {
            let mut scene = vec![ground()];
            for (sample, &(reach, middle)) in samples.iter().zip(&pairs) {
                let aside = Vector3::x() * 0.5 * TWIN * reach;
                scene.extend(meshes(&sample.whole, &texture, middle - aside));
                for (half, at) in &sample.halves {
                    scene.extend(meshes(half, &texture, middle + aside + at * SPREAD));
                }
            }
            scene
        };

        let environment = SceneEnvironment::default()
            .with_sun(Vector3::new(0.5, 0.5, 0.7))
            .with_sun_colour(Colour::new(1.0, 0.9, 0.78, 1.0));
        let shots = ["column", "beam", "gable"].iter().zip(&pairs);
        Ok(shots
            .map(|(label, &(reach, middle))| {
                let eye = middle + Vector3::new(0.35, 0.3, 1.0).normalize() * VIEW * reach;
                let camera = SceneCamera::looking_at(Point3::from(eye), Point3::from(middle));
                SceneShot::new(*label, camera.with_fov(40.0))
                    .with_environment(environment.clone())
                    .with_meshes(scene())
            })
            .collect())
    }
}
