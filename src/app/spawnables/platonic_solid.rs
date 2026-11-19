//! Platonic solid spawnable: one of the five a level asks the player to
//! take.
//!
//! ```text
//!   PlatonicSolid ──polyhedron──▶ core: the solid small, a light that blooms
//!                             ├──▶ shell: the solid in honeyed crystal,
//!                             │    the core seen through it
//!                             └──▶ frame: a gilt rod down every edge and a
//!                                  bead at every corner
//!   PlatonicSolidDef ──spawn──▶ Hover (no rigid body), Collectable, PointLight
//! ```
//!
//! Drawn as Leonardo drew them for Pacioli's *De divina proportione*, the
//! edges a frame, here a frame of gold round crystal with a light in it. They belong
//! to reason, not to the garden (`docs/VISUAL_DIRECTION.md`): true
//! proportions and crisp edges. A solid has no rigid body. It hangs where
//! it was authored, hovers, turns and is taken by walking into it; nothing
//! can knock it down a slope or into a lake.
//!
//! `visual_bench objective` shows all five.

use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector2, Vector3, Vector4};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::{MaterialCtx, Spawnable};
use crate::app::sculpt::{Ellipsoid, Strand};
use crate::components::{MaterialModulation, ModelInstance, Orientation, Position, Renderable};
use crate::core::error::EngineResult;
use crate::creature::Collectable;
use crate::geometry::{PlatonicSolid, Polyhedron};
use crate::lighting::PointLight;
use crate::model::{MeshPrimitive, Model, ModelPart, Transform};
use crate::objective::{Hover, LevelProgress};
use crate::rendering::colour::Colour;
use crate::rendering::material::{
    Emission, Material, MaterialId, SurfaceFinish, SurfaceModulation,
};
use crate::rendering::reflection::Reflects;
use crate::rendering::transparency::Transparency;
use crate::rendering::vertex::Vertex;

/// The name the solids' materials are shared under in a level.
const PALETTE: &str = "platonic_solids";

/// Slots in the palette.
const CORE: usize = 0;
const SHELL: usize = 1;
const GILT: usize = 2;

/// How big every solid looks, m: the mean of the radius through its
/// corners and the radius to its faces. Equal circumradii would make the
/// tetrahedron look half the size of the dodecahedron.
const MEAN_RADIUS: f32 = 0.3;

/// How far the shell's faces sit inside the frame's edges, as a share of
/// the radius: enough that every rod stands proud of the faces it divides.
const SHELL_INSET: f32 = 0.965;

/// How big the light inside is, as a share of the radius.
const CORE_SCALE: f32 = 0.5;

/// How much of what is behind it the shell hides head on, and how it bends
/// light: a clouded crystal, which turns to a mirror along its faces.
const SHELL_OPTICS: Transparency = Transparency::new(0.38, 1.55);

/// Radii of the frame's rods and of the beads at its corners, m.
const ROD_RADIUS: f32 = 0.0125;
const BEAD_RADIUS: f32 = 0.024;
/// Sides round a rod: round to the eye at any distance a solid is seen.
const ROD_SIDES: u32 = 12;

/// How close the player has to get. Comfortably wider than the solid itself:
/// brushing past one and not taking it reads as a bug.
const REACH: f32 = 1.2;

/// Warm white light: reason's colour against the garden's.
const LIGHT: Colour = Colour {
    r: 1.0,
    g: 0.78,
    b: 0.45,
    a: 1.0,
};

/// The shell's colour where the sun falls on it: honeyed crystal, deeper
/// than its light, so each face takes the sun differently.
const ALABASTER: Colour = Colour {
    r: 0.6,
    g: 0.42,
    b: 0.2,
    a: 1.0,
};

/// The light inside, as luminance: well over the bloom threshold, so a
/// solid carries a halo and is found at a distance.
const CORE_GLOW: f32 = 3.2;

/// The shell's own faint glow, the core's light caught in the crystal, so
/// it reads as lit from within even in shadow.
const SHELL_GLOW: f32 = 0.35;
const SHELL_RIM_STRENGTH: f32 = 1.5;
const SHELL_RIM_POWER: f32 = 3.0;

/// Gold, as the colour of what it mirrors (linear): a little bright where
/// the light catches a bead.
const GOLD: Vector3<f32> = Vector3::new(1.0, 0.62, 0.2);
const GOLD_BEAD: Vector3<f32> = Vector3::new(1.0, 0.68, 0.26);

/// Burnished gold: a mirror, softened.
const GILT_FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.2,
    metallic: 1.0,
};

/// The solid lights its own surroundings, which is what makes one tucked
/// into a corner findable.
const LIGHT_INTENSITY: f32 = 0.7;
const LIGHT_RANGE: f32 = 5.5;

#[derive(Deserialize)]
pub struct PlatonicSolidDef {
    pub pos: (f32, f32, f32),

    /// Which of the five.
    pub kind: PlatonicSolid,
}

impl PlatonicSolidDef {
    /// The materials of every solid, one to a slot: the light inside, the
    /// crystal round it, and the frame's gilt.
    pub fn materials() -> Vec<Material> {
        vec![
            Material::coloured(Colour::WHITE).with_emission(Emission {
                colour: LIGHT,
                strength: CORE_GLOW,
                ..Emission::default()
            }),
            Material::coloured(Colour::WHITE)
                .with_finish(SurfaceFinish::POLISHED)
                .with_transparency(SHELL_OPTICS)
                .with_emission(Emission {
                    colour: LIGHT,
                    strength: SHELL_GLOW,
                    rim_strength: SHELL_RIM_STRENGTH,
                    rim_power: SHELL_RIM_POWER,
                }),
            Material::coloured(Colour::WHITE)
                .with_finish(GILT_FINISH)
                .with_reflections(Reflects::Surroundings),
        ]
    }

    /// The radius through `kind`'s corners, m.
    pub fn circumradius(kind: PlatonicSolid) -> f32 {
        2.0 * MEAN_RADIUS / (1.0 + kind.inradius())
    }

    /// How `kind` is held as it turns: standing on a corner, with a corner
    /// up, which is how each reads as a solid and not as a box or a ball.
    pub fn rest(kind: PlatonicSolid) -> UnitQuaternion<f32> {
        let corner = kind.vertices()[0];
        UnitQuaternion::rotation_between(&corner, &Vector3::y())
            .unwrap_or_else(UnitQuaternion::identity)
    }

    /// `kind`'s model about its centre, in the palette whose ids are `ids`.
    pub fn model(kind: PlatonicSolid, ids: &[MaterialId]) -> Model {
        let radius = Self::circumradius(kind);
        let shape = kind.polyhedron();
        Model::flat(vec![ModelPart {
            local_transform: Transform::default(),
            primitives: vec![
                faces_mesh(&shape, radius * CORE_SCALE, Colour::WHITE, ids[CORE]),
                faces_mesh(&shape, radius * SHELL_INSET, ALABASTER, ids[SHELL]),
                frame_mesh(&shape, radius, ids[GILT]),
            ],
        }])
    }

    /// Phase offset, so that two solids authored at different places do not
    /// bob in lockstep. Derived from the position rather than a counter: the
    /// same level always looks the same.
    fn phase(&self) -> f32 {
        let (x, y, z) = self.pos;
        (x * 0.37 + y * 0.61 + z * 0.23).rem_euclid(10.0)
    }
}

impl Spawnable for PlatonicSolidDef {
    fn material_count(&self) -> usize {
        3
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(ctx.palette(PALETTE, Self::materials))
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let anchor = Vector3::new(self.pos.0, self.pos.1, self.pos.2);
        let model = Arc::new(Self::model(self.kind, materials));

        world
            .entry::<LevelProgress>()
            .or_insert_with(LevelProgress::default)
            .add_solid();

        let hover = Hover::new(anchor, Self::rest(self.kind), self.phase());
        let (position, orientation) = hover.pose();

        vec![world
            .create_entity()
            .with(Position(position))
            .with(Orientation(orientation))
            .with(hover)
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(MaterialModulation(SurfaceModulation::emissive(
                hover.glow(),
            )))
            .with(Collectable::solid(REACH))
            .with(PointLight::new(LIGHT, LIGHT_INTENSITY, LIGHT_RANGE))
            .build()]
    }
}

/// The solid's faces at `radius`, each flat-shaded, in `colour`.
fn faces_mesh(
    shape: &Polyhedron,
    radius: f32,
    colour: Colour,
    material: MaterialId,
) -> MeshPrimitive {
    let mut mesh = MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material,
    };
    for (index, face) in shape.faces.iter().enumerate() {
        let normal = shape.face_normal(index);
        let first = mesh.vertices.len() as u32;
        for &corner in face {
            mesh.vertices.push(Vertex {
                pos: shape.vertices[corner] * radius,
                color: Vector4::new(colour.r, colour.g, colour.b, 1.0),
                tex_coords: Vector2::zeros(),
                normal,
                ao: 1.0,
            });
        }
        for i in 1..face.len() as u32 - 1 {
            mesh.indices.extend([first, first + i, first + i + 1]);
        }
    }
    mesh
}

/// A gilt rod down every edge of the solid at `radius`, and a bead at every
/// corner.
fn frame_mesh(shape: &Polyhedron, radius: f32, material: MaterialId) -> MeshPrimitive {
    let mut mesh = MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material,
    };
    for (a, b) in shape.edges() {
        let (from, to) = (shape.vertices[a] * radius, shape.vertices[b] * radius);
        Strand::through(vec![(from, ROD_RADIUS), (to, ROD_RADIUS)])
            .sides(ROD_SIDES)
            .rings(1)
            .lay(&mut mesh, |_| (GOLD, 1.0));
    }
    let bead = Colour::new(GOLD_BEAD.x, GOLD_BEAD.y, GOLD_BEAD.z, 1.0);
    for corner in &shape.vertices {
        let ball = Ellipsoid::ball(BEAD_RADIUS, corner * radius).mesh(bead, material);
        let first = mesh.vertices.len() as u32;
        mesh.vertices.extend(ball.vertices);
        mesh.indices
            .extend(ball.indices.into_iter().map(|index| index + first));
    }
    mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every solid looks about as big as the others: none reaches out much
    /// further than the rest or sits much further in.
    #[test]
    fn the_five_look_the_same_size() {
        for kind in PlatonicSolid::ALL {
            let outer = PlatonicSolidDef::circumradius(kind);
            let inner = outer * kind.inradius();
            assert!(
                (0.5 * (outer + inner) - MEAN_RADIUS).abs() < 1e-5,
                "{kind:?}"
            );
            assert!(outer < REACH * 0.5, "{kind:?} reaches past its own reach");
        }
    }

    #[test]
    fn each_is_held_with_a_corner_up() {
        for kind in PlatonicSolid::ALL {
            let rest = PlatonicSolidDef::rest(kind);
            let top = kind
                .vertices()
                .iter()
                .map(|corner| (rest * corner).y)
                .fold(f32::MIN, f32::max);
            assert!((top - 1.0).abs() < 1e-5, "{kind:?} has no corner up");
        }
    }

    /// The frame is whole: every index names a vertex, and no rod or bead
    /// strays from the solid it outlines.
    #[test]
    fn the_model_is_whole_and_holds_to_its_solid() {
        let ids = [MaterialId(0), MaterialId(1), MaterialId(2)];
        for kind in PlatonicSolid::ALL {
            let model = PlatonicSolidDef::model(kind, &ids);
            let reach = PlatonicSolidDef::circumradius(kind) + BEAD_RADIUS + 1e-4;
            for mesh in model.parts.iter().flat_map(|part| &part.primitives) {
                assert!(!mesh.indices.is_empty(), "{kind:?} has an empty mesh");
                let count = mesh.vertices.len() as u32;
                assert!(mesh.indices.iter().all(|&i| i < count), "{kind:?}");
                for vertex in &mesh.vertices {
                    assert!(vertex.pos.norm() <= reach, "{kind:?}");
                }
            }
        }
    }
}
