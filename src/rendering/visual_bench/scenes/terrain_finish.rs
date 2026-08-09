//! Can a player see what terrain is made of?
//!
//! Terrain shades its finish from [`VoxelMaterial::hardness`] — the normalised
//! form of the toughness a blast spends against — so that a surface predicts
//! what will happen to it. This sheet is the test of that claim, and it uses
//! the moment the claim exists for: a crater cut through layered ground, which
//! lays every material in the stack bare at once, softest at the rim and
//! hardest at the floor.
//!
//! ```text
//!        grass ─┐
//!         sand ─┤
//!         dirt ─┤   layered flat ground
//!          ite ─┤            │
//!    limestone ─┤            │  detonate(CRATER_RADIUS)
//!         rock ─┤            ▼
//!        slate ─┤     ╲___________╱   every layer exposed as a ring
//!      bedrock ─┘
//! ```
//!
//! # How to read it
//!
//! As a set, never tile by tile. One crater always looks plausible; the failure
//! mode is a bowl whose rings differ only in colour, which is exactly where
//! this started — grass and rock told apart by a flat colour somebody chose.
//! What should be visible walking down the rings is the **finish** changing:
//! chalky and lightless at the rim, tightening to a dense sheen on the floor.
//!
//! | Shot | The question it answers |
//! |---|---|
//! | `bowl` | do the rings read as different materials, not just different colours |
//! | `bowl — sun across` | is it the *finish* differing, or just how each ring happens to face the sun |
//! | `rim grazing` | the specular test: at a glancing angle a rough surface stays flat and a tight one lifts |
//!
//! The middle shot is the control. Roughness shows itself through a highlight,
//! so a single sun angle cannot distinguish a genuinely tighter lobe from a
//! ring that merely tilts more favourably. Two sun azimuths over identical
//! geometry can: what survives both is the material.
//!
//! ```bash
//! cargo run --bin visual_bench -- terrain_finish --out /tmp/terrain_finish.png --columns 3
//! ```

use nalgebra::{Point3, Vector3};

use crate::collision::AABB;
use crate::core::error::EngineResult;
use crate::level::{Extent, MaterialLayer, Terrain, VoxelMaterialId};
use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;
use crate::rendering::visual_bench::scene::{
    SceneCamera, SceneContext, SceneEnvironment, SceneMesh, SceneShot, VisualScene,
};
use crate::terrain::{generate_terrain, surface, BlastConfig, ChunkGrid, Segment, SegmentFrame};

/// Fine enough that a 0.7 m layer is nearly three voxels thick, so no ring is
/// thinner than the mesh can resolve.
const VOXEL_SIZE: f32 = 0.25;

/// Side length of the square plot, in metres.
const PLOT_EXTENT: f32 = 24.0;

/// Height of the flat ground. Deliberately off the voxel lattice: a surface
/// landing exactly on a sample plane is the degenerate case that cracks
/// marching-cubes meshes, and a bench plot should not be sitting on it.
const GROUND_HEIGHT: f32 = 9.1;

/// Bottom of the generated volume.
const FLOOR_HEIGHT: f32 = 0.0;

/// Thickness of each destructible layer.
///
/// Thin, so that a *shallow* cut still reaches the bottom of the stack, and
/// equal across materials so the rings are evenly spaced and nothing is
/// emphasised by being given more room than its neighbours.
const LAYER_DEPTH: f32 = 0.45;

/// Depth at which the destructible stack ends and bedrock begins.
const STACK_DEPTH: f32 = LAYERS.len() as f32 * LAYER_DEPTH;

/// The stack, softest first. Ordered by toughness rather than by taste, so the
/// sheet reads as a ladder: if the finish is a readout, it must change
/// monotonically from the rim inwards.
const LAYERS: [VoxelMaterialId; 7] = [
    VoxelMaterialId::Grass,
    VoxelMaterialId::Sand,
    VoxelMaterialId::Dirt,
    VoxelMaterialId::Ite,
    VoxelMaterialId::Limestone,
    VoxelMaterialId::Rock,
    VoxelMaterialId::Slate,
];

/// Where the charge goes: over the middle of the plot rather than in it.
///
/// A charge buried at the surface cuts a deep pit, and a deep pit shades its
/// own interior — which is fatal here, because a finish shows itself through a
/// highlight and there is no highlight in shadow. Detonating above the ground
/// takes a shallow dish out of it instead: every ring stays open to the sun and
/// to the sky, and the rings come out wider and easier to read.
const CRATER: Point3<f32> = Point3::new(
    PLOT_EXTENT * 0.5,
    GROUND_HEIGHT + CRATER_STANDOFF,
    PLOT_EXTENT * 0.5,
);

/// How far above the ground the charge sits.
const CRATER_STANDOFF: f32 = 1.2;

/// Wide enough that the dish still cuts past the bottom of the destructible
/// stack and onto the bedrock the blast cannot take, so the sheet ends on the
/// one material that never yields.
const CRATER_RADIUS: f32 = 5.0;

/// Elevation shared by every shot. Low enough for a legible highlight, high
/// enough that the dish is not simply in its own shadow — the deepest rings are
/// the hard ones, and they are the whole point.
const SUN_ELEVATION: f32 = 48.0;

pub struct TerrainFinish;

impl VisualScene for TerrainFinish {
    fn name(&self) -> &str {
        "terrain_finish"
    }

    fn description(&self) -> &str {
        "Terrain finish derived from material toughness, shown as a crater through a layered stack. Use --columns 3."
    }

    fn shots(&self, ctx: &SceneContext) -> EngineResult<Vec<SceneShot>> {
        let (vertices, indices) = build();
        let surface_texture = surface::create_surface_texture(ctx.textures)?;
        let mesh = || {
            SceneMesh::new(vertices.clone(), indices.clone())
                .with_surface(surface::surface_params())
                .with_texture(surface_texture.clone())
        };

        let centre = Point3::new(CRATER.x, GROUND_HEIGHT - 1.5, CRATER.z);
        let into_the_bowl = SceneCamera::looking_at(
            Point3::new(CRATER.x - 5.0, GROUND_HEIGHT + 5.5, CRATER.z - 9.5),
            centre,
        );

        Ok(vec![
            SceneShot::new("bowl", into_the_bowl)
                .with_environment(environment(-155.0))
                .with_mesh(mesh()),
            SceneShot::new("bowl — sun across", into_the_bowl)
                .with_environment(environment(-35.0))
                .with_mesh(mesh()),
            SceneShot::new(
                "rim grazing",
                SceneCamera::looking_at(
                    Point3::new(CRATER.x - 0.5, GROUND_HEIGHT + 0.9, CRATER.z - 7.5),
                    centre,
                ),
            )
            .with_environment(environment(-155.0))
            .with_mesh(mesh()),
        ])
    }
}

/// Generate the layered plot and blow the crater in it.
///
/// The crater goes in through `detonate` rather than through the generator,
/// because that is the path a grenade takes, and because a hand-authored bowl
/// would not prove that a *carved* surface carries the material it was cut out
/// of.
fn build() -> (Vec<Vertex>, Vec<u32>) {
    let terrain = description();
    let bounds = terrain.bounds.to_aabb();

    let mut grid = ChunkGrid::new(terrain.voxel_size);
    generate_terrain(&mut grid, &terrain, &bounds);

    let mut segment = Segment::new("visual_bench", SegmentFrame::identity(), grid, Vec::new());
    let mut rebuilt: Vec<AABB> = Vec::new();
    segment.update(&mut rebuilt);

    segment.detonate(CRATER, &BlastConfig::fixed_radius(CRATER_RADIUS));
    rebuilt.clear();
    segment.update(&mut rebuilt);

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    segment.append_render_data(&mut vertices, &mut indices);
    (vertices, indices)
}

/// Flat ground over the full material stack, with no features at all.
///
/// Featureless on purpose. Every other terrain sheet is about form; this one is
/// about substance, and a rolling surface would let a reader attribute a
/// difference in finish to a difference in slope.
fn description() -> Terrain {
    Terrain {
        voxel_size: VOXEL_SIZE,
        bounds: Extent {
            min: (0.0, FLOOR_HEIGHT, 0.0),
            max: (PLOT_EXTENT, GROUND_HEIGHT + 2.0, PLOT_EXTENT),
        },
        base_height: GROUND_HEIGHT,
        material_layers: LAYERS
            .iter()
            .enumerate()
            .map(|(index, material)| MaterialLayer {
                depth: (index + 1) as f32 * LAYER_DEPTH,
                material: *material,
            })
            .collect(),
        // Starts exactly where the named layers end, which is load-bearing
        // rather than tidy: `material_at_depth` falls through to a built-in
        // default ladder below the deepest layer given to it, so any gap here
        // shows up as a stray ring of dirt in the middle of the dish. Bedrock
        // covering the remainder means no sample can reach the fallback.
        bedrock_thickness: GROUND_HEIGHT - STACK_DEPTH,
        features: Vec::new(),
        volumes: Vec::new(),
    }
}

fn environment(azimuth_degrees: f32) -> SceneEnvironment {
    let elevation = SUN_ELEVATION.to_radians();
    let azimuth = azimuth_degrees.to_radians();
    SceneEnvironment::default()
        .with_sun(Vector3::new(
            elevation.cos() * azimuth.sin(),
            elevation.sin(),
            elevation.cos() * azimuth.cos(),
        ))
        .with_sun_colour(Colour::new(1.0, 0.96, 0.88, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::VoxelMaterial;

    /// The sheet is worthless if the crater does not actually reach the bottom
    /// of the stack, and a reader cannot tell a bowl that stopped in the dirt
    /// from one that went all the way. Vertex hardness is what the shader
    /// shades from, so assert on that directly: every material in the plot must
    /// be represented in the geometry, bedrock included.
    #[test]
    fn the_crater_exposes_the_whole_material_stack() {
        let (vertices, _) = build();
        assert!(!vertices.is_empty(), "the plot meshed to nothing");

        // By hardness rather than by material, because hardness is what the
        // shader sees. Grass and sand share a toughness and so share a finish;
        // this cannot tell them apart, and neither can the renderer.
        let expected = [
            VoxelMaterial::Grass,
            VoxelMaterial::Sand,
            VoxelMaterial::Dirt,
            VoxelMaterial::Ite,
            VoxelMaterial::Limestone,
            VoxelMaterial::Rock,
            VoxelMaterial::Slate,
            VoxelMaterial::Bedrock,
        ];

        for material in expected {
            let hardness = material.hardness();
            let exposed = vertices
                .iter()
                .filter(|v| (v.tex_coords.x - hardness).abs() < 1e-5)
                .count();
            assert!(
                exposed > 20,
                "{material:?} (hardness {hardness}) shows on only {exposed} vertices"
            );
        }
    }

    /// The bench compares a sheet against its own past, which is worthless if
    /// the subject wanders.
    #[test]
    fn the_plot_is_identical_across_builds() {
        let (first, first_indices) = build();
        let (second, second_indices) = build();

        assert_eq!(first_indices, second_indices);
        assert_eq!(first.len(), second.len());
        for (index, (a, b)) in first.iter().zip(&second).enumerate() {
            assert_eq!(a.pos, b.pos, "vertex {index} position");
            assert_eq!(a.tex_coords, b.tex_coords, "vertex {index} hardness");
        }
    }
}
