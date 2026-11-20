//! Platonic solid spawnable: one of the five a level asks the player to
//! take.
//!
//! ```text
//!   PlatonicSolid ──polyhedron──▶ star: the solid small and stellated, as
//!                             │    Kepler stellated it, burning gold
//!                             ├──▶ rays: a shaft of white light out
//!                             │    through the middle of every face
//!                             ├──▶ shell: the solid in clear crystal that
//!                             │    flashes along its faces
//!                             ├──▶ frame: a gilt rod down every edge,
//!                             │    gold wire twisted round it, and a pearl
//!                             │    in a gold cup at every corner
//!                             └──▶ ring: a beaded gilt ring about it, level
//!                                  as it is held, as an armillary's
//!   motes (particles): points of light winking in and out about it
//!   PlatonicSolidDef ──spawn──▶ Hover (no rigid body), Collectable, PointLight
//! ```
//!
//! Drawn as Leonardo drew them for Pacioli's *De divina proportione*, the
//! edges a frame, here a frame of gold round crystal with a star burning in
//! it, its rays out through every face like a monstrance's. They belong
//! to reason, not to the garden (`docs/VISUAL_DIRECTION.md`): true
//! proportions and crisp edges. A solid has no rigid body. It hangs where
//! it was authored, hovers, turns and is taken by walking into it; nothing
//! can knock it down a slope or into a lake.
//!
//! `visual_bench objective` shows all five.

use std::f32::consts::{PI, TAU};
use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector2, Vector3, Vector4};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::gilt::{gilt, GOLD, GOLD_BEAD};
use super::{MaterialCtx, Spawnable};
use crate::app::sculpt::{Ellipsoid, Strand};
use crate::components::{MaterialModulation, ModelInstance, Orientation, Position, Renderable};
use crate::core::error::EngineResult;
use crate::creature::Collectable;
use crate::geometry::{PlatonicSolid, Polyhedron};
use crate::lighting::PointLight;
use crate::model::{MeshPrimitive, Model, ModelPart, Transform};
use crate::objective::{Hover, LevelProgress};
use crate::particles::{Palette, ParticleEffectType, ParticleEmitter};
use crate::rendering::colour::Colour;
use crate::rendering::material::{
    Emission, Material, MaterialId, SurfaceFinish, SurfaceModulation,
};
use crate::rendering::reflection::Reflects;
use crate::rendering::transparency::Transparency;
use crate::rendering::vertex::Vertex;

/// The colours of a solid's element, after Leonardo, whose treatise on
/// painting gives red to fire, yellow to earth, green to water, blue to air
/// and white to light; the dodecahedron, Plato's cosmos, takes the light of
/// the heavens, white with a breath of violet. The star carries the deep
/// colour, the crystal only a tint of it, and the rays and motes are its
/// light: the colour lifted toward white. All linear.
#[derive(Clone, Copy, Debug)]
pub struct Element {
    /// The star's fire, saturated, so its bloom is the element's colour.
    pub star: Colour,
    /// The rays and motes: the element's light.
    pub rays: Colour,
    /// The crystal's tint where the sun falls on it.
    pub crystal: Colour,
    /// What it casts on its surroundings.
    pub light: Colour,
}

impl Element {
    /// The element `kind` stands for, in the *Timaeus*.
    pub fn of(kind: PlatonicSolid) -> Self {
        let c = |r, g, b| Colour::new(r, g, b, 1.0);
        match kind {
            // Fire: vermilion in carnelian.
            PlatonicSolid::Tetrahedron => Self {
                star: c(1.0, 0.24, 0.02),
                rays: c(1.0, 0.7, 0.42),
                crystal: c(0.98, 0.66, 0.4),
                light: c(1.0, 0.52, 0.24),
            },
            // Earth: ochre in topaz.
            PlatonicSolid::Cube => Self {
                star: c(1.0, 0.56, 0.05),
                rays: c(1.0, 0.86, 0.55),
                crystal: c(0.94, 0.8, 0.5),
                light: c(1.0, 0.74, 0.36),
            },
            // Air: cerulean in pale sapphire.
            PlatonicSolid::Octahedron => Self {
                star: c(0.28, 0.55, 1.0),
                rays: c(0.78, 0.89, 1.0),
                crystal: c(0.7, 0.82, 0.96),
                light: c(0.62, 0.78, 1.0),
            },
            // Water: sea green in aquamarine.
            PlatonicSolid::Icosahedron => Self {
                star: c(0.04, 0.66, 0.52),
                rays: c(0.66, 0.98, 0.9),
                crystal: c(0.6, 0.86, 0.84),
                light: c(0.48, 0.95, 0.8),
            },
            // The cosmos: white light, violet at its heart.
            PlatonicSolid::Dodecahedron => Self {
                star: c(0.72, 0.52, 1.0),
                rays: c(1.0, 0.97, 1.0),
                crystal: c(0.86, 0.82, 0.95),
                light: c(0.88, 0.84, 1.0),
            },
        }
    }

    /// The name its materials are shared under in a level.
    fn palette(kind: PlatonicSolid) -> &'static str {
        match kind {
            PlatonicSolid::Tetrahedron => "platonic_fire",
            PlatonicSolid::Cube => "platonic_earth",
            PlatonicSolid::Octahedron => "platonic_air",
            PlatonicSolid::Icosahedron => "platonic_water",
            PlatonicSolid::Dodecahedron => "platonic_cosmos",
        }
    }
}

/// Slots in the palette.
const STAR: usize = 0;
const SHELL: usize = 1;
const GILT: usize = 2;
const RAYS: usize = 3;
const PEARL: usize = 4;

/// How big every solid looks, m: the mean of the radius through its
/// corners and the radius to its faces. Equal circumradii would make the
/// tetrahedron look half the size of the dodecahedron.
const MEAN_RADIUS: f32 = 0.3;

/// How far the shell's faces sit inside the frame's edges, as a share of
/// the radius: enough that every rod stands proud of the faces it divides.
const SHELL_INSET: f32 = 0.965;

/// The star inside: the radius of the solid it is raised on, and how far
/// out its points reach, as shares of the radius.
const STAR_BODY: f32 = 0.34;
const STAR_POINTS: f32 = 0.66;

/// The rays: where each starts and ends, as shares of the radius, and how
/// thick it is at its root, m. Out past the frame, so they are seen to
/// come out of the solid.
const RAY_FROM: f32 = 0.3;
const RAY_TO: f32 = 1.75;
const RAY_ROOT: f32 = 0.011;

/// How much of what is behind it the shell hides head on, and how it bends
/// light: clear as rock crystal, and as quick as a diamond to turn into a
/// mirror away from head on, so its faces flash as it turns.
const SHELL_OPTICS: Transparency = Transparency::new(0.16, 2.2);

/// Radius of the frame's rods, m.
const ROD_RADIUS: f32 = 0.0105;
/// Sides round a rod: round to the eye at any distance a solid is seen.
const ROD_SIDES: u32 = 12;

/// The wire twisted round each rod: two strands, how thick, how far along
/// the rod a strand takes to go once round it, m, and stations to a turn.
const WIRE_RADIUS: f32 = 0.0036;
const WIRE_PITCH: f32 = 0.06;
const WIRE_STATIONS: usize = 8;

/// The pearl at each corner, and the gold cup it sits in: its radius, how
/// far out from the corner its middle sits, m, and the cup's rim.
const PEARL_RADIUS: f32 = 0.021;
const PEARL_RISE: f32 = 0.012;
const CUP_RADIUS: f32 = 0.019;
const CUP_RIM: f32 = 0.0045;

/// The ring about it: its radius as a share of the radius, how thick it is,
/// m, and the beads set round it: how far apart, and how big.
const RING_RADIUS: f32 = 1.22;
const RING_THICKNESS: f32 = 0.0065;
const RING_BEAD_SPACING: f32 = 0.045;
const RING_BEAD_RADIUS: f32 = 0.0095;
const RING_STATIONS: usize = 72;

/// Motes of light shed about it each second.
const MOTES_PER_SECOND: f32 = 20.0;

/// How close the player has to get. Comfortably wider than the solid itself:
/// brushing past one and not taking it reads as a bug.
const REACH: f32 = 1.2;

/// The star, as luminance, its points brighter where they turn from the
/// eye; the rays, brightest of all and thin, so they bloom into shafts.
/// Both well over the bloom threshold.
const STAR_GLOW: f32 = 1.45;
const STAR_RIM_STRENGTH: f32 = 1.8;
const STAR_RIM_POWER: f32 = 2.0;
const RAY_GLOW: f32 = 7.0;

/// The crystal's own faint glow, the star's light caught in it, brightest
/// along its silhouette.
const SHELL_GLOW: f32 = 0.2;
const SHELL_RIM_STRENGTH: f32 = 3.0;
const SHELL_RIM_POWER: f32 = 3.0;

/// A pearl's lustre (linear): white with a blush of rose and cream.
const PEARL_COLOUR: Colour = Colour {
    r: 0.95,
    g: 0.9,
    b: 0.86,
    a: 1.0,
};

/// The solid lights its own surroundings, which is what makes one tucked
/// into a corner findable.
const LIGHT_INTENSITY: f32 = 1.8;
const LIGHT_RANGE: f32 = 7.0;

#[derive(Deserialize)]
pub struct PlatonicSolidDef {
    pub pos: (f32, f32, f32),

    /// Which of the five.
    pub kind: PlatonicSolid,
}

impl PlatonicSolidDef {
    /// The materials of a solid of `kind`, one to a slot: the star inside,
    /// the crystal round it, the frame's gilt, the rays and the pearls.
    pub fn materials(kind: PlatonicSolid) -> Vec<Material> {
        let element = Element::of(kind);
        vec![
            Material::coloured(Colour::WHITE).with_emission(Emission {
                colour: element.star,
                strength: STAR_GLOW,
                rim_strength: STAR_RIM_STRENGTH,
                rim_power: STAR_RIM_POWER,
            }),
            Material::coloured(Colour::WHITE)
                .with_finish(SurfaceFinish::POLISHED)
                .with_transparency(SHELL_OPTICS)
                .with_emission(Emission {
                    colour: element.star,
                    strength: SHELL_GLOW,
                    rim_strength: SHELL_RIM_STRENGTH,
                    rim_power: SHELL_RIM_POWER,
                }),
            gilt(),
            Material::coloured(Colour::WHITE).with_emission(Emission {
                colour: element.rays,
                strength: RAY_GLOW,
                ..Emission::default()
            }),
            Material::coloured(Colour::WHITE)
                .with_finish(SurfaceFinish {
                    roughness: 0.14,
                    metallic: 0.25,
                })
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
                star_mesh(&shape, radius, ids[STAR]),
                rays_mesh(&shape, radius, ids[RAYS]),
                faces_mesh(
                    &shape,
                    radius * SHELL_INSET,
                    Element::of(kind).crystal,
                    ids[SHELL],
                ),
                gilt_mesh(&shape, radius, Self::rest(kind), ids[GILT]),
                pearls_mesh(&shape, radius, ids[PEARL]),
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
        5
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let kind = self.kind;
        Ok(ctx.palette(Element::palette(kind), || Self::materials(kind)))
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let anchor = Vector3::new(self.pos.0, self.pos.1, self.pos.2);
        let element = Element::of(self.kind);
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
            .with(PointLight::new(element.light, LIGHT_INTENSITY, LIGHT_RANGE))
            .with(
                ParticleEmitter::new(ParticleEffectType::Motes)
                    .with_spawn_rate(MOTES_PER_SECOND)
                    .with_palette(Palette::single(Vector3::new(
                        element.rays.r,
                        element.rays.g,
                        element.rays.b,
                    ))),
            )
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

/// The star: the solid at `radius` × [`STAR_BODY`], a point raised on
/// every face to `radius` × [`STAR_POINTS`], each side flat.
fn star_mesh(shape: &Polyhedron, radius: f32, material: MaterialId) -> MeshPrimitive {
    let mut mesh = MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material,
    };
    for (index, face) in shape.faces.iter().enumerate() {
        let apex = shape.face_normal(index) * radius * STAR_POINTS;
        for i in 0..face.len() {
            let a = shape.vertices[face[i]] * radius * STAR_BODY;
            let b = shape.vertices[face[(i + 1) % face.len()]] * radius * STAR_BODY;
            let normal = (b - a).cross(&(apex - a)).normalize();
            let first = mesh.vertices.len() as u32;
            for pos in [a, b, apex] {
                mesh.vertices.push(Vertex {
                    pos,
                    color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                    tex_coords: Vector2::zeros(),
                    normal,
                    ao: 1.0,
                });
            }
            mesh.indices.extend([first, first + 1, first + 2]);
        }
    }
    mesh
}

/// A ray out through the middle of every face, tapering to nothing.
fn rays_mesh(shape: &Polyhedron, radius: f32, material: MaterialId) -> MeshPrimitive {
    let mut mesh = MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material,
    };
    for index in 0..shape.faces.len() {
        let out = shape.face_normal(index) * radius;
        Strand::through(vec![(out * RAY_FROM, RAY_ROOT), (out * RAY_TO, 0.0)])
            .sides(6)
            .rings(3)
            .lay(&mut mesh, |_| (Vector3::new(1.0, 1.0, 1.0), 1.0));
    }
    mesh
}

/// An empty mesh in `material`.
fn mesh_in(material: MaterialId) -> MeshPrimitive {
    MeshPrimitive {
        vertices: Vec::new(),
        indices: Vec::new(),
        material,
    }
}

/// Add `more` to `mesh`.
fn append(mesh: &mut MeshPrimitive, more: MeshPrimitive) {
    let first = mesh.vertices.len() as u32;
    mesh.vertices.extend(more.vertices);
    mesh.indices
        .extend(more.indices.into_iter().map(|index| index + first));
}

/// Two directions square to `axis` and to each other.
fn square_to(axis: Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    let spare = if axis.y.abs() < 0.9 {
        Vector3::y()
    } else {
        Vector3::x()
    };
    let u = axis.cross(&spare).normalize();
    (u, axis.cross(&u))
}

/// Everything gilt: a rod down every edge of the solid at `radius` with
/// two strands of wire twisted round it, a cup at every corner, and the
/// beaded ring about it, level when the solid is held at `rest`.
fn gilt_mesh(
    shape: &Polyhedron,
    radius: f32,
    rest: UnitQuaternion<f32>,
    material: MaterialId,
) -> MeshPrimitive {
    let mut mesh = mesh_in(material);
    let gold = |_| (GOLD, 1.0);
    for (a, b) in shape.edges() {
        let (from, to) = (shape.vertices[a] * radius, shape.vertices[b] * radius);
        Strand::through(vec![(from, ROD_RADIUS), (to, ROD_RADIUS)])
            .sides(ROD_SIDES)
            .rings(1)
            .lay(&mut mesh, gold);

        let along = to - from;
        let (u, v) = square_to(along.normalize());
        let stations = ((along.norm() / WIRE_PITCH) * WIRE_STATIONS as f32).ceil() as usize;
        for strand in 0..2 {
            let start = strand as f32 * PI;
            let path = (0..=stations)
                .map(|i| {
                    let t = i as f32 / stations as f32;
                    let turn = start + t * along.norm() / WIRE_PITCH * TAU;
                    let round = (u * turn.cos() + v * turn.sin()) * ROD_RADIUS;
                    (from + along * t + round, WIRE_RADIUS)
                })
                .collect();
            Strand::through(path).sides(5).rings(1).lay(&mut mesh, gold);
        }
    }

    // The cups: a rim of wire round each pearl's waist, on a gold boss.
    let cup = Colour::new(GOLD_BEAD.x, GOLD_BEAD.y, GOLD_BEAD.z, 1.0);
    for corner in &shape.vertices {
        let out = corner.normalize();
        let at = out * radius;
        append(
            &mut mesh,
            Ellipsoid::ball(CUP_RADIUS, at).mesh(cup, material),
        );
        let (u, v) = square_to(out);
        let rim = at + out * PEARL_RISE * 0.6;
        let path = (0..=24)
            .map(|i| {
                let a = TAU * i as f32 / 24.0;
                (
                    rim + (u * a.cos() + v * a.sin()) * PEARL_RADIUS * 0.95,
                    CUP_RIM,
                )
            })
            .collect();
        Strand::through(path).sides(6).rings(1).lay(&mut mesh, gold);
    }

    // The ring, level as the solid is held, with its beads.
    let level = rest.inverse() * Vector3::y();
    let (u, v) = square_to(level);
    let ring = radius * RING_RADIUS;
    let path = (0..=RING_STATIONS)
        .map(|i| {
            let a = TAU * i as f32 / RING_STATIONS as f32;
            ((u * a.cos() + v * a.sin()) * ring, RING_THICKNESS)
        })
        .collect();
    Strand::through(path).sides(8).rings(1).lay(&mut mesh, gold);
    let beads = (TAU * ring / RING_BEAD_SPACING).round() as usize;
    for i in 0..beads {
        let a = TAU * i as f32 / beads as f32;
        let at = (u * a.cos() + v * a.sin()) * ring;
        append(
            &mut mesh,
            Ellipsoid::ball(RING_BEAD_RADIUS, at).mesh(cup, material),
        );
    }
    mesh
}

/// A pearl at every corner, set out from it in its cup.
fn pearls_mesh(shape: &Polyhedron, radius: f32, material: MaterialId) -> MeshPrimitive {
    let mut mesh = mesh_in(material);
    for corner in &shape.vertices {
        let out = corner.normalize();
        let at = out * (radius + PEARL_RISE);
        append(
            &mut mesh,
            Ellipsoid::ball(PEARL_RADIUS, at).mesh(PEARL_COLOUR, material),
        );
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
        let ids: Vec<MaterialId> = (0..5).map(MaterialId).collect();
        for kind in PlatonicSolid::ALL {
            let model = PlatonicSolidDef::model(kind, &ids);
            let reach = PlatonicSolidDef::circumradius(kind) * RAY_TO + 1e-4;
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
