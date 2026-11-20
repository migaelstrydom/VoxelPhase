//! Burnished gold: the gilding on the things of reason (`docs/WORLD_AND_STORY.md`),
//! the same on every solid's frame and every scroll's knobs and figure, so
//! they read as one set.

use nalgebra::Vector3;

use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, SurfaceFinish};
use crate::rendering::reflection::Reflects;

/// Gold, as the colour of what it mirrors (linear), carried on the vertices
/// of a gilt mesh.
pub const GOLD: Vector3<f32> = Vector3::new(1.0, 0.62, 0.2);

/// Gold a little bright, where the light catches a bead.
pub const GOLD_BEAD: Vector3<f32> = Vector3::new(1.0, 0.68, 0.26);

/// A mirror, softened.
pub const GILT_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.2,
    metallic: 1.0,
};

/// The gilt material: white, so the gold rides on the vertices, mirroring
/// what is round it.
pub fn gilt() -> Material {
    Material::coloured(Colour::WHITE)
        .with_finish(GILT_FINISH)
        .with_reflections(Reflects::Surroundings)
}
