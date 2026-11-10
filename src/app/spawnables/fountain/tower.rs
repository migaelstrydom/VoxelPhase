//! The tower: a carved rose-stone spire standing in a lake, which lifts
//! the lake's water up through itself and throws it from spouts round its
//! upper bulb.
//!
//! ```text
//!         ╿        spire        ┐
//!       ╭─┴─╮                   │ parts of one compound, bonded at the
//!     ↖ │   │ ↗    bulb, spouts │ necks between them: a blast at a neck
//!       ╰─┬─╯                   │ breaks off what stands above it
//!     ════╪════    basin        │
//!    o    │    o   column       │ the arms, each with its orb, are bonded
//!     ╲ ╭─┴─╮ ╱                 │ to the base
//!   ~~~~│   │~~~~  base, intake ┘
//!   ▒▒▒▒╰───╯▒▒▒▒  bed: the base's weld, let go when the bed is blasted
//! ```
//!
//! The pipe runs base, column, bulb (`pump::Plumbing`): with the bulb gone
//! the water wells out of the column's top, with the column gone out of the
//! base's, and a base lying loose pumps while its intake is under water.
//!
//! Everything but the base and the arms is brittle stone
//! (`cleave::BrittleSolid`): a blast right beside a block, or a hard
//! landing, cleaves it into wedges. A cleaved length of pipe is a broken
//! one, so the water wells out below it.

use std::f32::consts::{FRAC_PI_4, TAU};
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::blocks::Block;
use super::broken::BrokenStone;
use crate::app::sculpt::{painted_meshes, Ellipsoid, SculptedPart, Warp};
use crate::app::spawnables::shared::finish::ColliderSurface;
use crate::app::spawnables::shared::rose_stone::{self, MATERIAL_COUNT, STONE};
use crate::app::spawnables::{MaterialCtx, Spawnable};
use crate::cleave::{BrittleSolid, CleaveRule};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, TerrainAnchored, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::{Bond, CompoundFracture, Cuttable, FaceMesher, PartGraph};
use crate::model::{assemble_by_material, Look, Model, PartLooks, PlacedMesh};
use crate::physics::{ColliderHandle, ConstraintKind, RigidBodyDesc};
use crate::pump::{PipeSection, Plumbing, Spout};
use crate::rendering::material::MaterialId;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;

// Every length below is for a tower of scale 1, measured up from the bed.

/// The base: a bulb half bedded.
const BASE_RADIUS: f32 = 1.2;
const BASE_HEIGHT: f32 = 0.5;
/// How far below the base's centre the intake is: just over the bed.
const INTAKE_DROP: f32 = 0.25;

/// The column: its radius, and the heights of its foot and top.
const COLUMN_RADIUS: f32 = 0.3;
const COLUMN: (f32, f32) = (1.5, 3.3);
/// The basin round the column: its height, and its radius and half its
/// thickness.
const BASIN_HEIGHT: f32 = 2.75;
const BASIN_HALF: (f32, f32) = (1.25, 0.09);

/// The upper bulb, which the spouts open from.
const BULB_RADIUS: f32 = 0.72;
const BULB_HEIGHT: f32 = 3.95;

/// The spire: its radius, and the heights of its foot and tip.
const SPIRE_RADIUS: f32 = 0.12;
const SPIRE: (f32, f32) = (4.55, 6.4);

/// Arms round the base, each holding up an orb. Four, set between the
/// spouts, so a quarter turn changes nothing.
const ARMS: usize = 4;
/// Where on the base an arm is rooted, above level, radians; how steeply
/// it climbs, as (out, up); and its length and radius.
const ARM_ROOT: f32 = 0.6;
const ARM_CLIMB: (f32, f32) = (0.75, 1.0);
const ARM_LENGTH: f32 = 1.5;
const ARM_RADIUS: f32 = 0.09;
const ORB_RADIUS: f32 = 0.3;

/// Spouts round the bulb, and how steeply each throws, above level,
/// radians.
const SPOUTS: usize = 4;
const SPOUT_PITCH: f32 = 0.9;
/// How fast water wells out of a broken pipe, as a share of the spouts'
/// speed.
const WELL_SPEED: f32 = 0.6;

/// The blow that cleaves a block: a blast loading, N·s/m², or a knock,
/// N·s. A grenade within a metre or so of a block's centre cleaves it,
/// where one that only breaks a neck can be nearly twice as far; and a block
/// the weight of the bulb cleaves when it lands from any height, a spire
/// only from a long fall.
const CLEAVE_BLOW: f32 = 1550.0;
/// Most wedges one blow makes of a block, how far its cuts lean off the
/// block's axes, degrees, and the least a wedge may hold, m³ at scale 1.
const CLEAVE_PIECES: u32 = 3;
const CLEAVE_TILT: f32 = 18.0;
const MIN_WEDGE_VOLUME: f32 = 0.01;

/// How far under the bed the ground must still be there for the weld to
/// hold, m.
const ANCHOR_PROBE: f32 = 0.1;

/// Blast loading that breaks a neck, N·s/m², and the knock that does,
/// N·s: a grenade beside a neck breaks it, one across the lake does not,
/// and nothing a body does by walking into it comes near.
const NECK_STRENGTH: f32 = 1500.0;
const NECK_TOUGHNESS: f32 = 6000.0;
/// The same for an arm's root, which is a tenth as thick.
const ARM_TOUGHNESS: f32 = 1500.0;

/// A fountain tower standing in a lake.
#[derive(Deserialize)]
pub struct FountainDef {
    /// Position (x, z). It stands on the bed there.
    pub pos: (f32, f32),
    /// Its size against the 6.4 m tower of scale 1.
    #[serde(default = "FountainDef::default_scale")]
    pub scale: f32,
    /// The most water it moves, m³/s.
    #[serde(default = "FountainDef::default_rate")]
    pub rate: f32,
    /// How fast its water leaves a spout, m/s.
    #[serde(default = "FountainDef::default_jet_speed")]
    pub jet_speed: f32,
}

impl FountainDef {
    pub fn default_scale() -> f32 {
        1.0
    }
    pub fn default_rate() -> f32 {
        0.6
    }
    pub fn default_jet_speed() -> f32 {
        4.5
    }

    /// A fountain at `pos`, everything else its default.
    pub fn at(pos: (f32, f32)) -> Self {
        Self {
            pos,
            scale: Self::default_scale(),
            rate: Self::default_rate(),
            jet_speed: Self::default_jet_speed(),
        }
    }
}

/// The tower as designed: its colliders, which of them make each part,
/// how each is carved, and its pipe. Indices are into `blocks`.
struct Design {
    blocks: Vec<Block>,
    /// The parts, each a list of blocks: base, column, bulb, spire, then
    /// the arms.
    parts: Vec<Vec<usize>>,
    /// Bonds between parts, as (part, part, where in the tower's frame,
    /// reach, toughness).
    bonds: Vec<(usize, usize, Vector3<f32>, f32, f32)>,
    /// The carving, each piece with the block it is drawn on. A block
    /// that shatters loses its carving for its wedges' broken stone.
    carving: Vec<(usize, SculptedPart)>,
    /// The pipe's sections from the base up, each a block and the spouts
    /// its water leaves by, in the tower's frame.
    pipe: Vec<(usize, Vec<Spout>)>,
    /// Where the intake is, in the tower's frame, on block 0.
    intake: Vector3<f32>,
}

/// Blocks of the design, by name.
const BASE: usize = 0;
const COLUMN_ROD: usize = 1;
const BASIN: usize = 2;
const BULB: usize = 3;
const SPIRE_ROD: usize = 4;
const FIRST_ARM: usize = 5;

/// The way spout or arm `index` of `count` faces, level, turned
/// `offset` radians round.
fn facing(index: usize, count: usize, offset: f32) -> Vector3<f32> {
    let azimuth = offset + TAU * index as f32 / count as f32;
    Vector3::new(azimuth.cos(), 0.0, azimuth.sin())
}

impl FountainDef {
    fn design(&self, looks: rose_stone::Looks) -> Design {
        let s = self.scale;
        let up = Vector3::y();
        let at = |height: f32| up * height * s;
        let base_centre = at(BASE_HEIGHT);
        let bulb_centre = at(BULB_HEIGHT);

        let (rose, blush, lapis, gold) = (looks.rose(), looks.blush(), looks.lapis(), looks.gold());
        let mut blocks = vec![
            Block::ball(BASE_RADIUS * s, base_centre),
            Block::stone_rod(COLUMN_RADIUS * s, at(COLUMN.0), at(COLUMN.1), blush.0),
            Block::stone_disc(BASIN_HALF.0 * s, BASIN_HALF.1 * s, at(BASIN_HEIGHT), rose.0),
            Block::stone_ball(BULB_RADIUS * s, bulb_centre, rose.0),
            Block::stone_rod(SPIRE_RADIUS * s, at(SPIRE.0), at(SPIRE.1), blush.0),
        ];
        let mut parts = vec![
            vec![BASE],
            vec![COLUMN_ROD, BASIN],
            vec![BULB],
            vec![SPIRE_ROD],
        ];
        let neck = BULB_RADIUS * s;
        let mut bonds = vec![
            (0, 1, at(COLUMN.0), neck, NECK_TOUGHNESS),
            (1, 2, at(COLUMN.1), neck, NECK_TOUGHNESS),
            (2, 3, at(SPIRE.0), neck, NECK_TOUGHNESS),
        ];

        let ellipsoid = |radii: (f32, f32, f32), centre: Vector3<f32>| {
            Ellipsoid::new(
                (radii.0 * s, radii.1 * s, radii.2 * s),
                (centre.x, centre.y, centre.z),
            )
        };
        let column_middle = 0.5 * (COLUMN.0 + COLUMN.1);
        let column_half = 0.5 * (COLUMN.1 - COLUMN.0);
        let spire_middle = 0.5 * (SPIRE.0 + SPIRE.1);
        let spire_half = 0.5 * (SPIRE.1 - SPIRE.0);
        let mut carving = vec![
            // The base, lobed like a gourd, with a gilt collar at its neck.
            (
                BASE,
                SculptedPart::new(
                    ellipsoid((BASE_RADIUS, 0.95 * BASE_RADIUS, BASE_RADIUS), base_centre),
                    rose,
                )
                .warped(Warp::Scallop {
                    lobes: 8,
                    bulge: 0.07,
                    lift: 0.0,
                }),
            ),
            (
                BASE,
                SculptedPart::new(ellipsoid((0.6, 0.12, 0.6), at(COLUMN.0 + 0.1)), gold),
            ),
            // The column, narrowing as it rises, with a knop under the basin.
            (
                COLUMN_ROD,
                SculptedPart::new(
                    ellipsoid(
                        (
                            1.25 * COLUMN_RADIUS,
                            column_half + 0.2,
                            1.25 * COLUMN_RADIUS,
                        ),
                        at(column_middle),
                    ),
                    blush,
                )
                .warped(Warp::Taper {
                    along: up,
                    tip: 0.7,
                }),
            ),
            (
                COLUMN_ROD,
                SculptedPart::new(ellipsoid((0.5, 0.25, 0.5), at(BASIN_HEIGHT - 0.4)), rose),
            ),
            // The basin, its rim scalloped and gilded.
            (
                BASIN,
                SculptedPart::new(ellipsoid((1.3, 0.2, 1.3), at(BASIN_HEIGHT)), rose).warped(
                    Warp::Scallop {
                        lobes: 12,
                        bulge: 0.06,
                        lift: 0.25,
                    },
                ),
            ),
            (
                BASIN,
                SculptedPart::new(ellipsoid((0.75, 0.12, 0.75), at(BASIN_HEIGHT + 0.12)), gold),
            ),
            // The bulb, banded in lapis, collared in gilt above and below.
            (
                BULB,
                SculptedPart::new(
                    ellipsoid((BULB_RADIUS, 0.95 * BULB_RADIUS, BULB_RADIUS), bulb_centre),
                    rose,
                )
                .warped(Warp::Scallop {
                    lobes: 6,
                    bulge: 0.05,
                    lift: 0.0,
                }),
            ),
            (
                BULB,
                SculptedPart::new(
                    ellipsoid(
                        (1.03 * BULB_RADIUS, 0.16, 1.03 * BULB_RADIUS),
                        at(BULB_HEIGHT - 0.2),
                    ),
                    lapis,
                ),
            ),
            (
                BULB,
                SculptedPart::new(ellipsoid((0.36, 0.1, 0.36), at(SPIRE.0 + 0.08)), gold),
            ),
            // The spire, a bead part-way up it and a gilt finial.
            (
                SPIRE_ROD,
                SculptedPart::new(
                    ellipsoid(
                        (1.5 * SPIRE_RADIUS, spire_half + 0.15, 1.5 * SPIRE_RADIUS),
                        at(spire_middle),
                    ),
                    blush,
                )
                .warped(Warp::Taper {
                    along: up,
                    tip: 0.25,
                }),
            ),
            (
                SPIRE_ROD,
                SculptedPart::new(ellipsoid((0.24, 0.2, 0.24), at(SPIRE.0 + 0.75)), lapis),
            ),
            (
                SPIRE_ROD,
                SculptedPart::new(ellipsoid((0.11, 0.16, 0.11), at(SPIRE.1 + 0.05)), gold),
            ),
        ];

        // The spouts: a gilt mouth each, on the bulb's widest round.
        let (sin, cos) = SPOUT_PITCH.sin_cos();
        let mut spouts = Vec::new();
        for index in 0..SPOUTS {
            let out = facing(index, SPOUTS, 0.0);
            let mouth = bulb_centre + out * BULB_RADIUS * s;
            carving.push((
                BULB,
                SculptedPart::new(ellipsoid((0.13, 0.13, 0.13), mouth), gold),
            ));
            spouts.push(Spout {
                outlet: Point3::from(mouth + out * 0.14 * s),
                velocity: (out * cos + up * sin) * self.jet_speed,
            });
        }

        // The arms, each a thin stem climbing out of the base to an orb.
        let climb = Vector3::new(ARM_CLIMB.0, ARM_CLIMB.1, 0.0).normalize();
        for index in 0..ARMS {
            let out = facing(index, ARMS, FRAC_PI_4);
            let root =
                base_centre + (out * ARM_ROOT.cos() + up * ARM_ROOT.sin()) * 0.95 * BASE_RADIUS * s;
            let along = out * climb.x + up * climb.y;
            let tip = root + along * ARM_LENGTH * s;
            let orb = tip + along * 0.6 * ORB_RADIUS * s;
            let (stem_block, orb_block) = (blocks.len(), blocks.len() + 1);
            blocks.push(Block::rod(ARM_RADIUS * s, root, tip));
            blocks.push(Block::ball(ORB_RADIUS * s, orb));
            bonds.push((0, parts.len(), root, ARM_LENGTH * s, ARM_TOUGHNESS));
            parts.push(vec![stem_block, orb_block]);

            let mut stem = ellipsoid(
                (ARM_RADIUS, 0.5 * ARM_LENGTH + 0.1, ARM_RADIUS),
                0.5 * (root + tip),
            );
            stem.turn = blocks[stem_block].frame.rotation;
            carving.push((stem_block, SculptedPart::new(stem, blush)));
            carving.push((
                orb_block,
                SculptedPart::new(ellipsoid((ORB_RADIUS, ORB_RADIUS, ORB_RADIUS), orb), lapis),
            ));
            carving.push((
                orb_block,
                SculptedPart::new(
                    ellipsoid((0.1, 0.1, 0.1), orb + up * 0.95 * ORB_RADIUS * s),
                    gold,
                ),
            ));
        }
        debug_assert_eq!(blocks.len(), FIRST_ARM + 2 * ARMS);

        // A broken pipe wells straight up out of its top.
        let well = |top: Vector3<f32>| {
            vec![Spout {
                outlet: Point3::from(top),
                velocity: up * WELL_SPEED * self.jet_speed,
            }]
        };
        let pipe = vec![
            (BASE, well(base_centre + up * BASE_RADIUS * s)),
            (COLUMN_ROD, well(at(COLUMN.1))),
            (BULB, spouts),
        ];

        Design {
            blocks,
            parts,
            bonds,
            carving,
            pipe,
            intake: base_centre - up * INTAKE_DROP * s,
        }
    }
}

impl FountainDef {
    /// How one of its blocks cleaves.
    fn cleaving(&self) -> CleaveRule {
        CleaveRule {
            threshold: CLEAVE_BLOW,
            pieces: CLEAVE_PIECES,
            tilt: CLEAVE_TILT,
            min_volume: MIN_WEDGE_VOLUME * self.scale.powi(3),
        }
    }

    /// What makes and draws the wedges of `design`'s blocks, each block
    /// read where it stands in the tower's frame.
    fn broken_stone(&self, design: &Design, materials: &[MaterialId]) -> BrokenStone {
        BrokenStone::new(
            design
                .blocks
                .iter()
                .filter_map(|block| Some((block.stone.clone()?, block.frame.translation.vector))),
            materials[0],
            self.scale,
        )
    }

    /// How it looks standing whole on its foot, drawn in `materials`
    /// ([`rose_stone::materials`]): for tools that show it.
    pub fn model(&self, materials: &[MaterialId]) -> Arc<Model> {
        self.drawn(materials, None)
    }

    /// How it looks with every block that can shatter cleaved by a blow
    /// from `from` (in its own frame) and its wedges drawn `spread` times
    /// as far from their block's middle as they lie: for tools that show
    /// its broken stone.
    pub fn shattered_model(
        &self,
        materials: &[MaterialId],
        from: Point3<f32>,
        spread: f32,
    ) -> Arc<Model> {
        self.drawn(materials, Some((from, spread)))
    }

    fn drawn(&self, materials: &[MaterialId], shattered: Option<(Point3<f32>, f32)>) -> Arc<Model> {
        let design = self.design(rose_stone::Looks::of(materials));
        let broken = self.broken_stone(&design, materials);
        let mut placed = Vec::new();
        for (index, (block, look)) in design
            .blocks
            .iter()
            .zip(design.looks(self.scale))
            .enumerate()
        {
            let at = block.frame.translation.vector;
            let wedges = shattered.and_then(|(from, _)| {
                block.stone.as_ref()?;
                let hit = block.frame.inverse() * from;
                self.cleaving()
                    .cleave(&block.collider.shape, hit.coords, index as u32)
            });
            let looks: Vec<(Look, Vector3<f32>)> = match (wedges, shattered) {
                (Some(wedges), Some((_, spread))) => wedges
                    .iter()
                    .map(|wedge| {
                        (
                            broken.look(&wedge.hull, at + wedge.centre),
                            at + wedge.centre * spread,
                        )
                    })
                    .collect(),
                _ => vec![(look, at)],
            };
            for (look, offset) in looks {
                placed.extend(look.primitives.into_iter().map(|mesh| PlacedMesh {
                    vertices: mesh.vertices,
                    indices: mesh.indices,
                    offset,
                    rotation: block.frame.rotation,
                    material: mesh.material,
                }));
            }
        }
        assemble_by_material(placed)
    }
}

impl Design {
    /// How each block looks: its share of the carving, painted and shaded
    /// as one tower, in the block's own frame.
    fn looks(&self, scale: f32) -> Vec<Look> {
        let carved: Vec<SculptedPart> = self.carving.iter().map(|(_, part)| part.clone()).collect();
        let meshes = painted_meshes(
            &carved,
            &rose_stone::mottle(scale),
            &rose_stone::contact_shade(scale),
        );
        let mut looks = vec![Look::default(); self.blocks.len()];
        for (&(block, _), mut mesh) in self.carving.iter().zip(meshes) {
            let into = self.blocks[block].frame.inverse();
            for vertex in &mut mesh.vertices {
                vertex.pos = (into * Point3::from(vertex.pos)).coords;
                vertex.normal = into.rotation * vertex.normal;
            }
            looks[block].primitives.push(mesh);
        }
        looks
    }
}

impl Spawnable for FountainDef {
    fn validate(&self) -> Result<(), String> {
        if self.scale <= 0.0 {
            return Err(format!(
                "a fountain's scale must be positive, not {}",
                self.scale
            ));
        }
        if self.rate < 0.0 || self.jet_speed <= 0.0 {
            return Err("a fountain's rate cannot be negative and its jets must move".into());
        }
        Ok(())
    }

    fn material_count(&self) -> usize {
        MATERIAL_COUNT
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(rose_stone::materials()
            .into_iter()
            .map(|material| ctx.materials.register(material))
            .collect())
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let bed = {
            let terrain = world.read_resource::<TerrainWorld>();
            terrain.mesh_surface_height_at(self.pos.0, self.pos.1)
        };
        let Some(bed) = bed else {
            return Vec::new();
        };
        let foot = Point3::new(self.pos.0, bed, self.pos.1);
        vec![self.spawn_on(world, foot, materials)]
    }
}

impl FountainDef {
    /// Spawn it standing on the bed at `foot`, welded there, drawn in
    /// `materials` ([`rose_stone::materials`]).
    fn spawn_on(&self, world: &mut World, foot: Point3<f32>, materials: &[MaterialId]) -> Entity {
        let design = self.design(rose_stone::Looks::of(materials));
        let block_looks = design.looks(self.scale);

        let mut physics = world.write_resource::<PhysicsResource>();
        // Welded with its base in the bed, which it passes through until
        // released: the weld holds it where the bed would.
        let body = physics.world.create_body(
            RigidBodyDesc::dynamic()
                .position(foot)
                .linear_damping(0.02)
                .angular_damping(0.1)
                .ignores_static(true),
        );
        let handles: Vec<ColliderHandle> = design
            .blocks
            .iter()
            .map(|block| {
                physics
                    .world
                    .attach_collider(
                        body,
                        block
                            .collider
                            .clone()
                            .offset_translation(block.frame.translation.vector)
                            .offset_rotation(block.frame.rotation)
                            .with_physical_surface(STONE),
                    )
                    .expect("a body just made takes colliders")
            })
            .collect();
        // The body's origin goes to its centre of mass; what was measured
        // from the foot follows it.
        let moved = physics.world.recenter_on_colliders(body);
        let weld = physics.world.create_constraint(ConstraintKind::world_fixed(
            body,
            foot,
            -moved,
            &UnitQuaternion::identity(),
            0.0,
            f32::MAX,
        ));

        let mut graph = PartGraph::default();
        let parts: Vec<_> = design
            .parts
            .iter()
            .map(|blocks| graph.add_part(blocks.iter().map(|&block| handles[block])))
            .collect();
        for &(a, b, at, reach, toughness) in &design.bonds {
            graph.bond(Bond {
                a: parts[a],
                b: parts[b],
                anchor: Point3::from(at - moved),
                reach,
                threshold: NECK_STRENGTH,
                toughness: Some(toughness),
            });
        }
        graph.tie(weld, parts[0]);

        let cuttable = design
            .blocks
            .iter()
            .zip(&handles)
            .filter(|(block, _)| block.stone.is_some())
            .fold(
                Cuttable::new(Arc::new(self.broken_stone(&design, materials))),
                |cuttable, (block, &handle)| cuttable.with(handle, block.frame.translation.vector),
            );
        let brittle = BrittleSolid::new(self.cleaving(), cuttable);

        let looks = PartLooks::new(
            handles
                .iter()
                .zip(block_looks)
                .filter(|(_, look)| !look.primitives.is_empty())
                .map(|(&handle, look)| (handle, Arc::new(look))),
        );
        let model = looks
            .drawn(&physics.world, body)
            .expect("a fountain has stone to draw");

        // The pipe, each point taken into its own block's frame.
        let within =
            |block: usize, point: Point3<f32>| design.blocks[block].frame.inverse() * point;
        let plumbing = Plumbing {
            intake: (handles[BASE], within(BASE, Point3::from(design.intake))),
            sections: design
                .pipe
                .iter()
                .map(|(block, spouts)| PipeSection {
                    collider: handles[*block],
                    spouts: spouts
                        .iter()
                        .map(|spout| Spout {
                            outlet: within(*block, spout.outlet),
                            velocity: design.blocks[*block]
                                .frame
                                .rotation
                                .inverse_transform_vector(&spout.velocity),
                        })
                        .collect(),
                })
                .collect(),
            rate: self.rate,
        };
        let pump = plumbing
            .pump_on(&physics.world, body)
            .expect("a whole fountain holds its intake");
        drop(physics);

        world
            .create_entity()
            .with(Position(foot.coords + moved))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(looks)
            .with(CompoundFracture::new(graph))
            .with(brittle)
            .with(plumbing)
            .with(pump)
            .with(TerrainAnchored {
                constraints: vec![weld],
                anchor_points: vec![foot - Vector3::y() * ANCHOR_PROBE],
                released_model: None,
            })
            .build()
    }
}

#[cfg(test)]
mod tests {
    use specs::Join;

    use super::*;
    use crate::app::breakage;
    use crate::debug::DebugLines;
    use crate::explosion::Explosion;
    use crate::physics::bench_harness::geometry::FlatQuadGeometry;
    use crate::physics::{PhysicsImpulseQueue, SequentialStepper, Stepper};
    use crate::pump::WaterPump;
    use crate::time::Time;

    fn world() -> World {
        let mut world = World::new();
        breakage::register(&mut world);
        world.insert(Time::fixed(1.0 / 60.0));
        world.insert(PhysicsResource::default());
        world
    }

    /// `frames` frames of physics, each followed by the break pipeline.
    fn step(world: &mut World, frames: usize) {
        let mut stepper = SequentialStepper::new(1.0 / 240.0, 12);
        let ground = FlatQuadGeometry::new(50.0);
        let mut lines = DebugLines::default();
        for _ in 0..frames {
            {
                let mut queue = world.write_resource::<PhysicsImpulseQueue>();
                let impulses: Vec<_> = queue.drain().collect();
                let mut physics = world.write_resource::<PhysicsResource>();
                stepper.step(
                    &mut physics.world,
                    1.0 / 60.0,
                    &ground,
                    &impulses,
                    &[],
                    &mut lines,
                );
            }
            lines.clear();
            breakage::run(world);
        }
    }

    /// A tower of scale 1 standing at the origin.
    fn tower(world: &mut World) -> Entity {
        FountainDef::at((0.0, 0.0)).spawn_on(
            world,
            Point3::origin(),
            &[MaterialId(0); MATERIAL_COUNT],
        )
    }

    fn blast(world: &mut World, at: Point3<f32>) {
        world
            .write_resource::<PhysicsImpulseQueue>()
            .push(Explosion::new(at).physics_impulse());
        step(world, 1);
    }

    /// How many entities the tower is now.
    fn pieces(world: &World) -> usize {
        (&world.entities(), &world.read_storage::<Plumbing>())
            .join()
            .count()
    }

    /// The world heights of the outlets `entity` throws from.
    fn outlets(world: &World, entity: Entity) -> Vec<f32> {
        let pumps = world.read_storage::<WaterPump>();
        let positions = world.read_storage::<Position>();
        let origin = positions.get(entity).unwrap().0;
        pumps.get(entity).map_or_else(Vec::new, |pump| {
            pump.spouts
                .iter()
                .map(|spout| origin.y + spout.outlet.y)
                .collect()
        })
    }

    #[test]
    fn whole_it_throws_from_the_spouts_round_its_bulb() {
        let mut world = world();
        let tower = tower(&mut world);
        let outlets = outlets(&world, tower);
        assert_eq!(outlets.len(), SPOUTS);
        for height in outlets {
            assert!((height - BULB_HEIGHT).abs() < 1.0e-3, "a spout at {height}");
        }
    }

    #[test]
    fn a_blast_across_the_lake_breaks_nothing() {
        let mut world = world();
        tower(&mut world);
        blast(&mut world, Point3::new(9.0, 2.0, 0.0));
        assert_eq!(pieces(&world), 1);
    }

    #[test]
    fn a_blast_beside_the_spire_breaks_it_off_and_the_spouts_run_on() {
        let mut world = world();
        let tower = tower(&mut world);
        blast(&mut world, Point3::new(1.0, 0.5 * (SPIRE.0 + SPIRE.1), 0.0));
        assert!(pieces(&world) > 1, "the spire held");
        assert_eq!(outlets(&world, tower).len(), SPOUTS, "the bulb went too");
        // What broke off carries no pump.
        let plumbed = world.read_storage::<Plumbing>();
        let pumps = world.read_storage::<WaterPump>();
        for (entity, _) in (&world.entities(), &plumbed).join() {
            assert_eq!(pumps.contains(entity), entity == tower);
        }
    }

    #[test]
    fn a_blast_beside_the_bulb_breaks_it_off_and_the_column_wells() {
        let mut world = world();
        let tower = tower(&mut world);
        blast(&mut world, Point3::new(1.6, BULB_HEIGHT, 0.0));
        assert!(pieces(&world) > 1, "the bulb held");
        let outlets = outlets(&world, tower);
        assert_eq!(outlets.len(), 1, "the tower still throws from its bulb");
        assert!((outlets[0] - COLUMN.1).abs() < 0.02);
    }

    /// Whether pipe section `section` of the tower still exists whole.
    fn section_stands(world: &World, tower: Entity, section: usize) -> bool {
        let plumbed = world.read_storage::<Plumbing>();
        let collider = plumbed.get(tower).unwrap().sections[section].collider;
        world
            .read_resource::<PhysicsResource>()
            .world
            .collider(collider)
            .is_some()
    }

    #[test]
    fn a_blast_against_the_bulb_shatters_it_and_the_column_wells() {
        let mut world = world();
        let tower = tower(&mut world);
        blast(&mut world, Point3::new(BULB_RADIUS + 0.2, BULB_HEIGHT, 0.0));
        assert!(!section_stands(&world, tower, 2), "the bulb is whole");
        assert!(section_stands(&world, tower, 1), "the column went too");
        let outlets = outlets(&world, tower);
        assert_eq!(outlets.len(), 1);
        assert!((outlets[0] - COLUMN.1).abs() < 0.02);
    }

    #[test]
    fn a_blast_that_breaks_a_neck_from_further_off_shatters_nothing() {
        let mut world = world();
        let tower = tower(&mut world);
        blast(&mut world, Point3::new(1.6, BULB_HEIGHT, 0.0));
        assert!(pieces(&world) > 1, "the bulb held");
        let physics = world.read_resource::<PhysicsResource>();
        let plumbed = world.read_storage::<Plumbing>();
        for section in &plumbed.get(tower).unwrap().sections {
            assert!(physics.world.collider(section.collider).is_some());
        }
    }

    #[test]
    fn with_the_bulb_off_the_water_wells_out_of_the_column() {
        let mut world = world();
        let tower = tower(&mut world);
        {
            // Break the neck under the bulb.
            let plumbed = world.read_storage::<Plumbing>();
            let mut compounds = world.write_storage::<CompoundFracture>();
            let bulb = plumbed.get(tower).unwrap().sections[2].collider;
            let compound = compounds.get_mut(tower).unwrap();
            let bulb = compound.graph.part_of(bulb).unwrap();
            let neck = compound
                .graph
                .bonds()
                .iter()
                .position(|bond| bond.holds(bulb))
                .unwrap();
            let mut index = 0;
            compound.graph.break_bonds(|_| {
                index += 1;
                index - 1 == neck
            });
            compound.split_pending = true;
        }
        step(&mut world, 1);
        assert_eq!(pieces(&world), 2);
        let outlets = outlets(&world, tower);
        assert_eq!(outlets.len(), 1);
        assert!(
            (outlets[0] - COLUMN.1).abs() < 0.02,
            "it wells out at {}, not the column's top",
            outlets[0]
        );
    }
}
