//! Greek temple spawnable — classical Doric peristyle temple with columns,
//! entablature, pediments, and pitched roof. Golden-ratio proportions
//! throughout, except the pediment, which is Doric-shallow instead — see
//! [`DORIC_PEDIMENT_RATIO`]. Every piece above the stylobate is an independent
//! rigid body; the stylobate is static, and stands on a `Foundation` the level
//! authors under it. A column, beam or gable that lands hard cracks in two
//! (`cleave::BrittleSolid`), each half keeping its dressed surface; the roof
//! panels never crack.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector2, Vector3, Vector4};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{
    build_convex_hull, compound_cuboid_model, convex_solid_model, cuboid_model, SolidFace,
};
use super::stone::stone_cleaving;
use super::{MaterialCtx, Spawnable};
use crate::cleave::{BlowMeasure, BrittleSolid, CleaveRule};
use crate::collision::convex_hull::{cube_hull, ConvexHull};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::{
    BreakFace, CarvedBlock, CarvedPieces, CompoundFracture, Cuttable, FaceMesher, PartGraph,
};
use crate::model::{Look, MeshPrimitive, Model, ModelPart, PartLooks};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{MaterialId, SurfaceFinish};
use crate::rendering::pattern;
use crate::rendering::substance::palette::Palette;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;

const TEXTURE_SIZE: u32 = 256;
const PHI: f32 = 1.618034;

/// Number of sides on the Doric column polygon — matches the 20 canonical
/// flutes so each polygon face aligns with exactly one flute channel.
const COLUMN_SIDES: u32 = 20;

/// Pediment height as a fraction of the temple's half-width.
///
/// The one proportion here that is *not* golden-ratio. A Doric gable is shallow
/// — the Parthenon's is about 0.22 of its half-width, roughly a 13° roof — and
/// `half_w / PHI` gave a triangle nearly three times too tall, which read as
/// gothic rather than Greek and put the roof panels above the friction angle of
/// stone. Raising it steepens the roof and raises the friction every marble
/// joint needs; see [`MARBLE`].
const DORIC_PEDIMENT_RATIO: f32 = 0.22;

/// Marble where it has just broken, as a tint on its texture: whiter and
/// colder than its weathered face.
const FRESH_MARBLE: Colour = Colour::new(1.04, 1.1, 1.18, 1.0);
/// How far across a break one repeat of the marble's texture reaches, m.
const BREAK_TEXTURE_SPAN: f32 = 2.0;

const MAT_STONE: usize = 0;
const MAT_MARBLE: usize = 1;
const MAT_FLUTED: usize = 2;

/// Seeds for the three baked surfaces. Fixed rather than random so that a
/// temple looks the same every run, and so that the many pieces cut from one
/// material share one texture rather than baking a near-identical tile each.
const STONE_SEED: u32 = 101;
const MARBLE_SEED: u32 = 202;
/// The flutes are the same marble, differently cut — a separate seed is what
/// keeps a column's shaft from repeating its own capital's markings.
const FLUTED_SEED: u32 = 303;
const MATERIAL_COUNT: usize = 3;

/// Rough quarried stone, used for the stylobate the temple stands on.
///
/// Sandstone's appearance over authored physics. The friction is well above
/// anything real masonry offers — it is the value that keeps a stack of loose
/// blocks standing, and it stays exactly where it was because a stack of steps
/// depends on it.
///
/// It used to also be why the steps read as chalk, the finish being derived
/// from it. That link is gone: a surface may now be grippy without being
/// chalky, so the steps read as the rough stone they are rather than as the
/// roughest thing the renderer can draw.
const RUBBLE: Substance = substance::SANDSTONE.with_coefficients(0.05, 1.5, 2400.0);

/// Honed marble left out of doors: the columns, the entablature, the
/// pediments and the roof. Warm ivory with a rose cast, veined in grey-rose,
/// and matte: stone that has stood in the weather scatters the sky instead
/// of mirroring it, so its shaded faces stay stone-coloured.
///
/// Denser and slicker than the rubble it rests on. How it grips and how it
/// shines are authored apart: the friction is the solver's, the finish the
/// eye's.
///
/// **The roof pitch is the floor under this friction.** The panels are sloped
/// slabs held by friction alone, so one needs `mu >= pediment_h / eave_dist` to
/// stay on: 0.19 for a six-column temple, tending to [`DORIC_PEDIMENT_RATIO`]
/// itself, 0.22, as it widens. 0.4 clears the widest case by more than 80%, and
/// is a real coefficient for dressed stone on dressed stone.
pub const MARBLE: Substance = Substance {
    finish: SurfaceFinish {
        roughness: 0.68,
        metallic: 0.0,
    },
    palette: Palette::from_base_const(Colour::new(0.9, 0.83, 0.74, 1.0), 0.08)
        .with_dark(Colour::new(0.8, 0.7, 0.63, 1.0))
        .with_accent(Colour::new(0.62, 0.52, 0.53, 1.0)),
    ..substance::MARBLE.with_coefficients(0.05, 0.4, 2700.0)
};

/// The marble's markings.
pub const MARBLE_PATTERN: &pattern::Pattern = &pattern::HONED_MARBLE;

#[derive(Deserialize)]
pub struct TempleDef {
    /// World position of the ground-level center of the temple.
    pub pos: (f32, f32, f32),
    /// Height of the columns (everything else scales from this).
    #[serde(default = "TempleDef::default_column_height")]
    pub column_height: f32,
    /// Number of columns along the front (short) side.
    #[serde(default = "TempleDef::default_front_columns")]
    pub front_columns: u32,
    /// Number of columns along the side (long) side, including corners.
    #[serde(default = "TempleDef::default_side_columns")]
    pub side_columns: u32,
}

impl TempleDef {
    pub fn default_column_height() -> f32 {
        8.0
    }
    pub fn default_front_columns() -> u32 {
        6
    }
    pub fn default_side_columns() -> u32 {
        9
    }

    /// Half-extents of the ground the temple stands on, in `(x, z)`.
    ///
    /// The bottom step, which is the widest thing it puts on the terrain.
    /// Exposed because everything here is derived from `column_height` through
    /// ratios that belong in one place: a validator that wants to know how much
    /// ground the temple covers should ask rather than re-derive.
    pub fn ground_half_extents(&self) -> (f32, f32) {
        let lay = TempleLayout::from_def(self);
        (
            lay.half_w + lay.col_base_r + lay.step_margin,
            lay.half_l + lay.col_base_r + lay.step_margin,
        )
    }
}

/// All measurements derived from column_height using classical Doric ratios
/// and the golden ratio.
struct TempleLayout {
    col_base_r: f32,
    col_top_r: f32,
    col_height: f32,
    spacing: f32,

    half_w: f32,
    half_l: f32,

    num_steps: u32,
    step_h: f32,
    step_margin: f32,
    stylobate_top: f32,

    entab_h: f32,
    entab_overhang: f32,
    entab_base_y: f32,

    pediment_h: f32,
    pediment_base_y: f32,
    roof_t: f32,
}

impl TempleLayout {
    fn from_def(def: &TempleDef) -> Self {
        let ch = def.column_height;

        let col_base_r = ch / 12.0;
        let col_top_r = col_base_r * 0.82;
        let spacing = ch / PHI.powi(2);

        let half_w = (def.front_columns - 1) as f32 * spacing / 2.0;
        let half_l = (def.side_columns - 1) as f32 * spacing / 2.0;

        let num_steps = 3_u32;
        let step_h = ch / 24.0;
        let step_margin = spacing * 0.12;
        let stylobate_top = def.pos.1 + num_steps as f32 * step_h;

        let entab_h = ch / PHI.powi(2);
        let entab_overhang = col_base_r * 0.6;
        let entab_base_y = stylobate_top + ch;

        let pediment_h = half_w * DORIC_PEDIMENT_RATIO;
        let pediment_base_y = entab_base_y + entab_h;
        let roof_t = ch / 40.0;

        Self {
            col_base_r,
            col_top_r,
            col_height: ch,
            spacing,
            half_w,
            half_l,
            num_steps,
            step_h,
            step_margin,
            stylobate_top,
            entab_h,
            entab_overhang,
            entab_base_y,
            pediment_h,
            pediment_base_y,
            roof_t,
        }
    }
}

/// One kind of a temple's loose marble, whole and cracked: for tools that
/// show its broken stone.
pub struct MarbleSample {
    /// How the piece looks whole, about its centre of mass.
    pub whole: Arc<Look>,
    /// Each half it cracks into, and where the half's frame sits in the
    /// whole's.
    pub halves: Vec<(Look, Vector3<f32>)>,
}

/// How the marble shows where it breaks: the same stone, laid flat on the
/// break.
fn marble_breaks(marble: MaterialId) -> BreakFace {
    BreakFace {
        material: marble,
        colour: FRESH_MARBLE.to_vec4(),
        texture_span: BREAK_TEXTURE_SPAN,
    }
}

impl TempleDef {
    /// A column, a beam of the entablature and a gable of this temple, each
    /// whole and as a blow from `from` (in the piece's frame) cracks it,
    /// drawn in `materials` as [`Spawnable::create_materials`] orders them.
    pub fn marble_samples(
        &self,
        materials: &[MaterialId],
        from: Vector3<f32>,
    ) -> Vec<MarbleSample> {
        let lay = TempleLayout::from_def(self);
        let (marble, fluted) = (materials[MAT_MARBLE], materials[MAT_FLUTED]);
        let depth = lay.col_base_r + lay.entab_overhang;
        let beam = Vector3::new(lay.half_w + depth, lay.entab_h / 2.0, depth);
        let pieces = [
            MarblePiece::column(&lay, fluted, Vector2::zeros()).0,
            MarblePiece::block(beam, marble),
            MarblePiece::gable(&lay, marble),
        ];
        let samples = pieces.iter().enumerate().map(|(salt, piece)| {
            let (rule, mesher) = piece
                .cracking(marble_breaks(marble))
                .expect("each sample is a piece that cracks");
            let halves = rule
                .cleave(&piece.collider.shape, from, salt as u32)
                .unwrap_or_default();
            MarbleSample {
                whole: piece.look.clone(),
                halves: halves
                    .iter()
                    .map(|half| (mesher.look(&half.hull, half.centre), half.centre))
                    .collect(),
            }
        });
        samples.collect()
    }
}

impl Spawnable for TempleDef {
    fn validate(&self) -> Result<(), String> {
        if self.front_columns < 2 || self.side_columns < 2 {
            return Err(format!(
                "a temple needs at least 2 columns on each side, got front_columns {} and \
                 side_columns {}",
                self.front_columns, self.side_columns
            ));
        }
        Ok(())
    }

    fn material_count(&self) -> usize {
        MATERIAL_COUNT
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![
            // Fixed seeds: every temple in a level should be built of the
            // same quarry's stone, and a shared seed is also what lets the
            // seven columns share one baked texture instead of seven.
            ctx.patterned(&RUBBLE, &pattern::STONE, STONE_SEED, TEXTURE_SIZE)?,
            ctx.patterned(&MARBLE, MARBLE_PATTERN, MARBLE_SEED, TEXTURE_SIZE)?,
            ctx.patterned(&MARBLE, MARBLE_PATTERN, FLUTED_SEED, TEXTURE_SIZE)?,
        ])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let px = self.pos.0;
        let py = self.pos.1;
        let pz = self.pos.2;
        let stone = materials[MAT_STONE];
        let marble = materials[MAT_MARBLE];
        let fluted = materials[MAT_FLUTED];

        let lay = TempleLayout::from_def(self);

        let mut entities = Vec::new();

        let breaks = marble_breaks(marble);

        // =================================================================
        // Stylobate — three stepped platforms as one static compound body.
        // It doesn't move, so nothing that rests on it or leans on it is held
        // awake by it. A level puts a `Foundation` of bedrock under it, so no
        // blast can leave it hanging.
        // =================================================================

        let top_step_hw = lay.half_w + lay.col_base_r + lay.step_margin;
        let top_step_hl = lay.half_l + lay.col_base_r + lay.step_margin;

        let stylobate_center_y = py + lay.num_steps as f32 * lay.step_h / 2.0;
        let stylobate_pos = Point3::new(px, stylobate_center_y, pz);

        let mut step_boxes = Vec::new();
        for i in 0..lay.num_steps {
            let grow = (lay.num_steps - 1 - i) as f32 * lay.step_margin;
            let hw = top_step_hw + grow;
            let hl = top_step_hl + grow;
            let hh = lay.step_h / 2.0;
            let offset_y = i as f32 * lay.step_h + hh - lay.num_steps as f32 * lay.step_h / 2.0;

            step_boxes.push((Vector3::new(hw, hh, hl), Vector3::new(0.0, offset_y, 0.0)));
        }

        let stylobate_model = compound_cuboid_model(&step_boxes, stone);

        let stylobate_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let bh = physics
                .world
                .create_body(RigidBodyDesc::static_body().position(stylobate_pos));
            for &(ref he, ref offset) in &step_boxes {
                physics.world.attach_collider(
                    bh,
                    ColliderDesc::box_shape(*he)
                        .offset_translation(*offset)
                        .of(&RUBBLE),
                );
            }
            bh
        };

        entities.push(
            world
                .create_entity()
                .with(Position(Vector3::new(
                    stylobate_pos.x,
                    stylobate_pos.y,
                    stylobate_pos.z,
                )))
                .with(Velocity(Vector3::zeros()))
                .with(Orientation::default())
                .with(RigidBodyComponent(stylobate_handle))
                .with(ModelInstance::new(stylobate_model))
                .with(Renderable)
                .build(),
        );

        // =================================================================
        // Columns — fluted tapered prisms around the perimeter
        // =================================================================

        let col_y_base = lay.stylobate_top;

        let mut col_positions = Vec::new();

        // Front and back rows.
        for i in 0..self.front_columns {
            let x = px - lay.half_w + i as f32 * lay.spacing;
            col_positions.push((x, pz + lay.half_l));
            col_positions.push((x, pz - lay.half_l));
        }
        // Side rows (excluding corners already placed).
        for j in 1..(self.side_columns - 1) {
            let z = pz - lay.half_l + j as f32 * lay.spacing;
            col_positions.push((px - lay.half_w, z));
            col_positions.push((px + lay.half_w, z));
        }

        for (index, &(cx, cz)) in col_positions.iter().enumerate() {
            // Each column a different part of the one texture: steps of the
            // golden ratio never land two columns on the same turn.
            let turn = Vector2::new(index as f32 * PHI, index as f32 * PHI * PHI).map(f32::fract);
            let (column, centre) = MarblePiece::column(&lay, fluted, turn);
            let pos = Point3::new(cx, col_y_base, cz) + centre;
            entities.push(column.spawn(world, pos, UnitQuaternion::identity(), breaks));
        }

        // =================================================================
        // Entablature — four beams (architrave + frieze + cornice)
        // =================================================================

        let entab_hh = lay.entab_h / 2.0;
        let entab_cy = lay.entab_base_y + entab_hh;
        let beam_depth = lay.col_base_r + lay.entab_overhang;

        let fb_hw = lay.half_w + beam_depth;
        for &z_sign in &[1.0_f32, -1.0] {
            entities.push(
                MarblePiece::block(Vector3::new(fb_hw, entab_hh, beam_depth), marble).spawn(
                    world,
                    Point3::new(px, entab_cy, pz + z_sign * lay.half_l),
                    UnitQuaternion::identity(),
                    breaks,
                ),
            );
        }

        let side_hl = lay.half_l - beam_depth;
        for &x_sign in &[1.0_f32, -1.0] {
            entities.push(
                MarblePiece::block(Vector3::new(beam_depth, entab_hh, side_hl), marble).spawn(
                    world,
                    Point3::new(px + x_sign * lay.half_w, entab_cy, pz),
                    UnitQuaternion::identity(),
                    breaks,
                ),
            );
        }

        // =================================================================
        // Pediments — triangular gable ends (front and back)
        // =================================================================

        let ped_base = lay.pediment_base_y;
        let ped_hw = lay.half_w + beam_depth;

        for &z_sign in &[1.0_f32, -1.0] {
            let z_outer = pz + z_sign * (lay.half_l + beam_depth);
            let z_inner = pz + z_sign * (lay.half_l - beam_depth);
            let centre = Point3::new(
                px,
                ped_base + lay.pediment_h / 3.0,
                0.5 * (z_outer + z_inner),
            );
            entities.push(MarblePiece::gable(&lay, marble).spawn(
                world,
                centre,
                UnitQuaternion::identity(),
                breaks,
            ));
        }

        // =================================================================
        // Roof — two sloped panels meeting at the ridge
        // =================================================================

        let roof_overhang = lay.step_margin * 2.0;
        let eave_dist = ped_hw;
        let peak_h = lay.pediment_h;
        let slope_len = (eave_dist * eave_dist + peak_h * peak_h).sqrt();
        let roof_angle = peak_h.atan2(eave_dist);
        let roof_half_depth = lay.half_l + beam_depth + roof_overhang;
        let roof_he = Vector3::new(slope_len / 2.0, lay.roof_t / 2.0, roof_half_depth);

        for &side in &[-1.0_f32, 1.0] {
            let mid_x = px + side * eave_dist / 2.0;
            let mid_y = ped_base + peak_h / 2.0;
            let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -side * roof_angle);

            entities.push(MarblePiece::slab(roof_he, marble).spawn(
                world,
                Point3::new(mid_x, mid_y, pz),
                rot,
                breaks,
            ));
        }

        entities
    }
}

/// A loose piece of the temple's marble, about its own centre of mass: what
/// it collides and cracks as, and how it looks whole.
struct MarblePiece {
    /// Its shape, which it is cut by when it cracks. `None` for a piece
    /// that never cracks.
    hull: Option<Arc<ConvexHull>>,
    collider: ColliderDesc,
    look: Arc<Look>,
}

impl MarblePiece {
    /// A squared block of `half_extents`.
    fn block(half_extents: Vector3<f32>, material: MaterialId) -> Self {
        Self {
            hull: Some(Arc::new(cube_hull(half_extents))),
            ..Self::slab(half_extents, material)
        }
    }

    /// A slab of `half_extents` that never cracks: a roof panel, far too
    /// thin for its length to be cut as one hull.
    fn slab(half_extents: Vector3<f32>, material: MaterialId) -> Self {
        Self {
            hull: None,
            collider: ColliderDesc::box_shape(half_extents).of(&MARBLE),
            look: Arc::new(look_of(&cuboid_model(half_extents, material))),
        }
    }

    /// A column of the temple laid out as `lay`, and how far above the
    /// middle of its foot its centre of mass is: the centroid of its
    /// volume, which a column narrowing upward holds below half its height.
    ///
    /// `turn` slides the marble's texture round the shaft and up it, in
    /// repeats, so no two columns wear the same veins at the same height.
    fn column(lay: &TempleLayout, fluted: MaterialId, turn: Vector2<f32>) -> (Self, Vector3<f32>) {
        let (verts, faces) =
            column_geometry(lay.col_base_r, lay.col_top_r, lay.col_height, COLUMN_SIDES);
        let centre = build_convex_hull(&verts, &faces).centroid();
        let local: Vec<_> = verts.iter().map(|v| v - centre).collect();
        let hull = Arc::new(build_convex_hull(&local, &faces));
        // Drawn with cylindrical UVs, about the same centre.
        let model = fluted_column_model(
            lay.col_base_r,
            lay.col_top_r,
            lay.col_height,
            centre.y,
            COLUMN_SIDES,
            fluted,
        );
        let mut look = look_of(&model);
        for vertex in look
            .primitives
            .iter_mut()
            .flat_map(|mesh| &mut mesh.vertices)
        {
            vertex.tex_coords += turn;
        }
        let column = Self {
            collider: ColliderDesc::convex_hull(hull.clone()).of(&MARBLE),
            hull: Some(hull),
            look: Arc::new(look),
        };
        (column, centre)
    }

    /// A gable: the triangular block over an end of the temple laid out as
    /// `lay`, about the mean of its corners.
    fn gable(lay: &TempleLayout, material: MaterialId) -> Self {
        let depth = lay.col_base_r + lay.entab_overhang;
        let (verts, faces) =
            gable_geometry(0.0, 0.0, lay.pediment_h, lay.half_w + depth, depth, -depth);
        let centre = verts.iter().copied().sum::<Vector3<f32>>() / verts.len() as f32;
        let local: Vec<_> = verts.iter().map(|v| v - centre).collect();
        Self::solid(&local, &faces, material)
    }

    /// How it cracks, and what makes and draws its halves when it does,
    /// their `breaks` drawn as given. `None` for a piece that never cracks.
    fn cracking(&self, breaks: BreakFace) -> Option<(CleaveRule, CarvedPieces)> {
        let hull = self.hull.clone()?;
        let rule = stone_cleaving(MARBLE.physics.density * hull.compute_volume());
        let halves = CarvedPieces::new(
            vec![CarvedBlock {
                hull,
                frame: Isometry3::identity(),
                look: self.look.clone(),
            }],
            breaks,
            |hull| ColliderDesc::convex_hull(hull).of(&MARBLE),
        );
        Some((rule, halves))
    }

    /// A solid with these corners and faces.
    fn solid(vertices: &[Vector3<f32>], faces: &[SolidFace], material: MaterialId) -> Self {
        let hull = Arc::new(build_convex_hull(vertices, faces));
        Self {
            collider: ColliderDesc::convex_hull(hull.clone()).of(&MARBLE),
            hull: Some(hull),
            look: Arc::new(look_of(&convex_solid_model(vertices, faces, material))),
        }
    }

    /// Stand it at `pos`, turned by `rotation`: a body of its own. One that
    /// can crack does when it lands hard, each half keeping its share of
    /// the piece's dressed surface and showing `breaks` where it parted.
    fn spawn(
        self,
        world: &mut World,
        pos: Point3<f32>,
        rotation: UnitQuaternion<f32>,
        breaks: BreakFace,
    ) -> Entity {
        let cracking = self.cracking(breaks);

        let mut physics = world.write_resource::<PhysicsResource>();
        let body = physics.world.create_body(
            RigidBodyDesc::dynamic()
                .position(pos)
                .rotation(rotation)
                .linear_damping(0.01)
                .angular_damping(0.005),
        );
        let collider = physics
            .world
            .attach_collider(body, self.collider)
            .expect("a piece of marble attaches to its own new body");
        let looks = PartLooks::new([(collider, self.look)]);
        let model = looks
            .drawn(&physics.world, body)
            .expect("a piece of marble has something to draw");
        drop(physics);

        let entity = world
            .create_entity()
            .with(Position(pos.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(rotation))
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(looks);
        let Some((rule, halves)) = cracking else {
            return entity.build();
        };
        // A compound of one part, which is all a block needs to crack.
        let mut graph = PartGraph::default();
        graph.add_part([collider]);
        entity
            .with(CompoundFracture::new(graph))
            .with(
                BrittleSolid::new(
                    rule,
                    Cuttable::new(Arc::new(halves)).with(collider, Vector3::zeros()),
                )
                .measured_by(BlowMeasure::Arrest),
            )
            .build()
    }
}

/// A model's meshes as one look. The model's parts must sit at its origin.
fn look_of(model: &Model) -> Look {
    Look {
        primitives: model
            .parts
            .iter()
            .flat_map(|part| part.primitives.iter().cloned())
            .collect(),
    }
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Tapered polygonal prism for a Doric column (used for physics hull).
///
/// Base sits at y=0, top at y=`height`. Returns vertices + face definitions.
fn column_geometry(
    base_radius: f32,
    top_radius: f32,
    height: f32,
    num_sides: u32,
) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let n = num_sides as usize;
    let angle_step = std::f32::consts::TAU / num_sides as f32;

    let mut vertices = Vec::with_capacity(2 * n);

    for i in 0..n {
        let angle = i as f32 * angle_step;
        vertices.push(Vector3::new(
            base_radius * angle.cos(),
            0.0,
            base_radius * angle.sin(),
        ));
    }
    for i in 0..n {
        let angle = i as f32 * angle_step;
        vertices.push(Vector3::new(
            top_radius * angle.cos(),
            height,
            top_radius * angle.sin(),
        ));
    }

    let mut faces = Vec::with_capacity(n + 2);

    faces.push(SolidFace {
        vertex_indices: (0..n).rev().collect(),
        opposite_vertex: n,
    });
    faces.push(SolidFace {
        vertex_indices: (n..2 * n).collect(),
        opposite_vertex: 0,
    });
    for i in 0..n {
        let i_next = (i + 1) % n;
        faces.push(SolidFace {
            vertex_indices: vec![i, i_next, n + i_next, n + i],
            opposite_vertex: (i + n / 2) % n,
        });
    }

    (vertices, faces)
}

/// Build a column rendering model with cylindrical UV mapping.
///
/// U wraps around the circumference (0..1 maps to one full revolution) so the
/// fluted texture tiles seamlessly. V runs from 0 (base) to 1 (capital).
/// Side vertices get smooth radial normals; caps get flat up/down normals.
/// Geometry is centred on `centroid_y`, the height of the physics hull's
/// centroid above the column's base, to match the hull's local space.
fn fluted_column_model(
    base_radius: f32,
    top_radius: f32,
    height: f32,
    centroid_y: f32,
    num_sides: u32,
    material: MaterialId,
) -> Arc<Model> {
    let n = num_sides as usize;
    let angle_step = std::f32::consts::TAU / num_sides as f32;
    let white = Vector4::new(1.0, 1.0, 1.0, 1.0);

    // The hull's centroid lies on the axis (a symmetric polygon), at
    // `centroid_y`. Shift the model down by the same amount so the rendering
    // mesh and physics hull share the same origin.
    let oy = -centroid_y;

    let mut verts: Vec<Vertex> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    // --- Side faces (cylindrical UVs, smooth radial normals) ---
    for i in 0..n {
        let base_idx = verts.len() as u32;

        let angle_a = i as f32 * angle_step;
        let angle_b = (i + 1) as f32 * angle_step;

        // UV: each face spans one flute. U goes from i/n to (i+1)/n.
        let u_a = i as f32 / n as f32;
        let u_b = (i + 1) as f32 / n as f32;

        let (cos_a, sin_a) = (angle_a.cos(), angle_a.sin());
        let (cos_b, sin_b) = (angle_b.cos(), angle_b.sin());

        // Smooth outward-pointing normals.
        let na = Vector3::new(cos_a, 0.0, sin_a);
        let nb = Vector3::new(cos_b, 0.0, sin_b);

        // Bottom-left (base, angle_a).
        verts.push(Vertex {
            pos: Vector3::new(base_radius * cos_a, oy, base_radius * sin_a),
            color: white,
            tex_coords: Vector2::new(u_a, 0.0),
            normal: na,
            ao: 1.0,
        });
        // Bottom-right (base, angle_b).
        verts.push(Vertex {
            pos: Vector3::new(base_radius * cos_b, oy, base_radius * sin_b),
            color: white,
            tex_coords: Vector2::new(u_b, 0.0),
            normal: nb,
            ao: 1.0,
        });
        // Top-right (capital, angle_b).
        verts.push(Vertex {
            pos: Vector3::new(top_radius * cos_b, height + oy, top_radius * sin_b),
            color: white,
            tex_coords: Vector2::new(u_b, 1.0),
            normal: nb,
            ao: 1.0,
        });
        // Top-left (capital, angle_a).
        verts.push(Vertex {
            pos: Vector3::new(top_radius * cos_a, height + oy, top_radius * sin_a),
            color: white,
            tex_coords: Vector2::new(u_a, 1.0),
            normal: na,
            ao: 1.0,
        });

        indices.extend_from_slice(&[
            base_idx,
            base_idx + 2,
            base_idx + 1,
            base_idx,
            base_idx + 3,
            base_idx + 2,
        ]);
    }

    // --- Bottom cap (flat downward normal, radial UVs) ---
    let cap_base = verts.len() as u32;
    let down = Vector3::new(0.0, -1.0, 0.0);
    for i in 0..n {
        let angle = i as f32 * angle_step;
        let (c, s) = (angle.cos(), angle.sin());
        verts.push(Vertex {
            pos: Vector3::new(base_radius * c, oy, base_radius * s),
            color: white,
            tex_coords: Vector2::new(0.5 + 0.5 * c, 0.5 + 0.5 * s),
            normal: down,
            ao: 1.0,
        });
    }
    for i in 1..(n as u32 - 1) {
        indices.extend_from_slice(&[cap_base, cap_base + i, cap_base + i + 1]);
    }

    // --- Top cap (flat upward normal, radial UVs) ---
    let top_base = verts.len() as u32;
    let up = Vector3::new(0.0, 1.0, 0.0);
    for i in 0..n {
        let angle = i as f32 * angle_step;
        let (c, s) = (angle.cos(), angle.sin());
        verts.push(Vertex {
            pos: Vector3::new(top_radius * c, height + oy, top_radius * s),
            color: white,
            tex_coords: Vector2::new(0.5 + 0.5 * c, 0.5 + 0.5 * s),
            normal: up,
            ao: 1.0,
        });
    }
    for i in 1..(n as u32 - 1) {
        indices.extend_from_slice(&[top_base, top_base + i + 1, top_base + i]);
    }

    let parts = vec![ModelPart::new(vec![MeshPrimitive {
        vertices: verts,
        indices,
        material,
    }])];
    Arc::new(Model::flat(parts))
}

/// Triangular gable prism.
fn gable_geometry(
    cx: f32,
    base_y: f32,
    peak_y: f32,
    half_w: f32,
    z_a: f32,
    z_b: f32,
) -> (Vec<Vector3<f32>>, Vec<SolidFace>) {
    let vertices = vec![
        Vector3::new(cx - half_w, base_y, z_a), // 0
        Vector3::new(cx + half_w, base_y, z_a), // 1
        Vector3::new(cx, peak_y, z_a),          // 2
        Vector3::new(cx - half_w, base_y, z_b), // 3
        Vector3::new(cx + half_w, base_y, z_b), // 4
        Vector3::new(cx, peak_y, z_b),          // 5
    ];

    let faces = vec![
        SolidFace {
            vertex_indices: vec![0, 1, 2],
            opposite_vertex: 3,
        },
        SolidFace {
            vertex_indices: vec![5, 4, 3],
            opposite_vertex: 0,
        },
        SolidFace {
            vertex_indices: vec![0, 3, 4, 1],
            opposite_vertex: 2,
        },
        SolidFace {
            vertex_indices: vec![0, 2, 5, 3],
            opposite_vertex: 1,
        },
        SolidFace {
            vertex_indices: vec![1, 4, 5, 2],
            opposite_vertex: 0,
        },
    ];

    (vertices, faces)
}

// ---------------------------------------------------------------------------
// Procedural textures
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use specs::Join;

    use super::*;
    use crate::app::breakage;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{PhysicsConfig, PhysicsWorld};
    use crate::time::Time;

    fn layout_for(front_columns: u32) -> TempleLayout {
        TempleLayout::from_def(&TempleDef {
            pos: (0.0, 0.0, 0.0),
            column_height: TempleDef::default_column_height(),
            front_columns,
            side_columns: TempleDef::default_side_columns(),
        })
    }

    /// The tangent of the roof slope, which is exactly the friction coefficient
    /// a panel needs to stay where it was placed. Mirrors how `spawn` builds the
    /// roof: the panels run from the ridge out to the eaves, and the eaves sit a
    /// beam's depth beyond the columns.
    fn roof_slope(lay: &TempleLayout) -> f32 {
        let beam_depth = lay.col_base_r + lay.entab_overhang;
        lay.pediment_h / (lay.half_w + beam_depth)
    }

    #[test]
    fn the_roof_cannot_slide_off_at_any_temple_width() {
        for front_columns in 2..=24 {
            let needed = roof_slope(&layout_for(front_columns));
            assert!(
                MARBLE.physics.friction > needed * 1.2,
                "a {}-column temple's roof needs mu >= {:.3} and marble offers {:.3}",
                front_columns,
                needed,
                MARBLE.physics.friction
            );
        }
    }

    /// The slope rises with width and converges on the pediment ratio, so the
    /// widest temple is always the worst case and the test above never has to
    /// guess where that is.
    #[test]
    fn a_wider_temple_has_the_steeper_roof() {
        let mut previous = 0.0;
        for front_columns in 2..=24 {
            let slope = roof_slope(&layout_for(front_columns));
            assert!(slope > previous, "slope fell at {} columns", front_columns);
            assert!(slope < DORIC_PEDIMENT_RATIO);
            previous = slope;
        }
    }

    /// A Doric gable is shallow. Guards the proportion itself, so replacing it
    /// with something golden-ratio shaped fails here rather than in the game.
    #[test]
    fn the_gable_stays_shallow_enough_to_read_as_greek() {
        let slope = roof_slope(&layout_for(6));
        assert!(
            slope.atan().to_degrees() < 15.0,
            "roof pitch is {:.1} deg",
            slope.atan().to_degrees()
        );
    }

    const FRAME_DT: f32 = 1.0 / 60.0;
    const FLUTED: MaterialId = MaterialId(2);
    const MARBLE_MATERIAL: MaterialId = MaterialId(1);

    /// A fluted column of the default temple lying on its side, its lowest
    /// point `drop` metres above a floor.
    fn dropped_column(drop: f32) -> World {
        let mut world = World::new();
        breakage::register(&mut world);
        world.insert(Time::fixed(FRAME_DT));
        let mut config = PhysicsConfig::default();
        config.sleep.enabled = false;
        world.insert(PhysicsResource::new(
            PhysicsWorld::new(config),
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));

        let lay = layout_for(TempleDef::default_front_columns());
        MarblePiece::column(&lay, FLUTED, Vector2::zeros()).0.spawn(
            &mut world,
            Point3::new(0.0, lay.col_base_r + drop, 0.0),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2),
            marble_breaks(MARBLE_MATERIAL),
        );
        world
    }

    /// Run the game's break pipeline for `frames` frames.
    fn settle(world: &mut World, frames: usize) {
        let geometry = FlatQuadGeometry::new(50.0);
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
            breakage::run(world);
        }
    }

    /// The area drawn in `material`, over every piece in the world.
    fn drawn(world: &World, material: MaterialId) -> f32 {
        let looks = world.read_storage::<PartLooks>();
        let physics = world.read_resource::<PhysicsResource>();
        let bodies = world.read_storage::<RigidBodyComponent>();
        let mut area = 0.0;
        for (looks, body) in (&looks, &bodies).join() {
            let colliders = physics.world.body(body.0).unwrap().colliders();
            for mesh in colliders
                .iter()
                .filter_map(|collider| looks.get(*collider))
                .flat_map(|look| &look.primitives)
                .filter(|mesh| mesh.material == material)
            {
                for t in mesh.indices.chunks_exact(3) {
                    let [a, b, c] = [t[0], t[1], t[2]].map(|i| mesh.vertices[i as usize].pos);
                    area += 0.5 * (b - a).cross(&(c - a)).norm();
                }
            }
        }
        area
    }

    fn colliders(world: &World) -> usize {
        let physics = world.read_resource::<PhysicsResource>();
        let bodies = world.read_storage::<RigidBodyComponent>();
        (&bodies)
            .join()
            .map(|body| {
                physics
                    .world
                    .body(body.0)
                    .map_or(0, |b| b.colliders().len())
            })
            .sum()
    }

    #[test]
    fn a_column_laid_down_gently_stays_whole() {
        let mut world = dropped_column(0.1);
        settle(&mut world, 120);
        assert_eq!(colliders(&world), 1);
        assert_eq!(drawn(&world, MARBLE_MATERIAL), 0.0, "a break is showing");
    }

    /// The halves share out the column's fluted face between them and show
    /// its section, twice, where they parted.
    #[test]
    fn a_column_that_falls_far_cracks_and_its_halves_keep_their_flutes() {
        let mut world = dropped_column(6.0);
        let fluted = drawn(&world, FLUTED);
        settle(&mut world, 150);
        assert_eq!(colliders(&world), 2, "the column did not crack in two");

        let lay = layout_for(TempleDef::default_front_columns());
        let section = std::f32::consts::PI * lay.col_top_r * lay.col_top_r;
        let broken = drawn(&world, MARBLE_MATERIAL);
        assert!(broken > 2.0 * section, "{broken} m² of break showing");
        let kept = drawn(&world, FLUTED);
        assert!(
            (kept - fluted).abs() < 0.01 * fluted,
            "{kept} m² of {fluted}"
        );
    }
}

/// A whole temple brought down, which takes seconds to simulate.
#[cfg(all(test, feature = "physics_scenarios"))]
mod collapse_tests {
    use specs::Join;

    use super::*;
    use crate::app::breakage;
    use crate::debug::DebugLines;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::stepping::{SequentialStepper, Stepper};
    use crate::physics::{PhysicsConfig, PhysicsWorld};
    use crate::time::Time;

    const FRAME_DT: f32 = 1.0 / 60.0;

    /// Every collider in the world, and whether each one on a piece that
    /// keeps looks has something drawn for it.
    fn colliders(world: &World) -> (usize, bool) {
        let physics = world.read_resource::<PhysicsResource>();
        let bodies = world.read_storage::<RigidBodyComponent>();
        let looks = world.read_storage::<PartLooks>();
        let (mut count, mut drawn) = (0, true);
        for (entity, body) in (&world.entities(), &bodies).join() {
            let colliders = physics.world.body(body.0).unwrap().colliders();
            count += colliders.len();
            if let Some(looks) = looks.get(entity) {
                drawn &= colliders.iter().all(|collider| {
                    looks
                        .get(*collider)
                        .is_some_and(|look| !look.primitives.is_empty())
                });
            }
        }
        (count, drawn)
    }

    #[test]
    fn a_temple_shoved_over_cracks_its_marble_and_every_piece_is_drawn() {
        let mut world = World::new();
        breakage::register(&mut world);
        world.insert(Time::fixed(FRAME_DT));
        world.insert(PhysicsResource::new(
            PhysicsWorld::new(PhysicsConfig::default()),
            Box::new(SequentialStepper::new(FRAME_DT, 4)),
        ));
        let temple = TempleDef {
            pos: (0.0, 0.0, 0.0),
            column_height: TempleDef::default_column_height(),
            front_columns: TempleDef::default_front_columns(),
            side_columns: TempleDef::default_side_columns(),
        };
        let pieces = temple.spawn(&mut world, &[MaterialId(0), MaterialId(1), MaterialId(2)]);
        let (standing, _) = colliders(&world);

        // Every loose piece thrown sideways: columns, beams, gables and roof.
        {
            let bodies = world.read_storage::<RigidBodyComponent>();
            let mut physics = world.write_resource::<PhysicsResource>();
            for body in pieces.iter().skip(1).filter_map(|piece| bodies.get(*piece)) {
                let mass = physics.world.body(body.0).unwrap().mass();
                physics
                    .world
                    .apply_impulse(body.0, Vector3::new(6.0, 0.0, 2.0) * mass);
            }
        }
        let geometry = FlatQuadGeometry::new(200.0);
        let mut stepper = SequentialStepper::new(FRAME_DT, 4);
        for _ in 0..360 {
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
            breakage::run(&mut world);
        }

        let (fallen, drawn) = colliders(&world);
        assert!(fallen > standing + 4, "{standing} pieces became {fallen}");
        assert!(drawn, "a piece of marble is not drawn");
    }
}
