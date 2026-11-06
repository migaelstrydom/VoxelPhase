//! Glass sheet — a pane that cracks where it is hit and drops the shards.
//!
//! ```text
//!   GlassSheetDef ──▶ substance::GLASS ──┬──▶ one box collider   (the pane)
//!                                        ├──▶ PartLooks         (a cuboid)
//!                                        ├──▶ CompoundFracture  (one part, no bonds;
//!                                        │                       held: a weld per long edge)
//!                                        └──▶ BrittleSheet      (how it cracks, GlassFaces)
//! ```
//!
//! It spawns as one dynamic body with one collider. That is all it is until
//! something hits it: then the glass system replaces the collider under the
//! hit with a web of convex shards, the fracture system knocks out the ones
//! directly under the blow, and the rest stay in place, cracked, holding
//! each other up — until the next hit, or, for a pane that is stood on,
//! until it has borne the weight for longer than it can.
//!
//! A **held** pane (`fixed`) is welded to the world along its two long
//! edges, a weld per edge, and once it crazes a weld per shard along them:
//! a glass floor over a drop, cracked right across, stays up as two pieces.
//! A **loose** one falls over and shatters on whatever it lands on.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Vector2, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::{cuboid_look, hull_mesh, PieceHull, SurfaceUvs};
use super::shared::orientation::Yaw;
use super::shared::textures::seed_from_position;
use super::{MaterialCtx, Spawnable};
use crate::collision::convex_hull::ConvexHull;
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::{CompoundFracture, Cuttable, FaceMesher, PartGraph};
use crate::glass::{
    weld_along, BrittleSheet, ConvexPolygon, CrackWeb, CrazeRule, FatigueRule, HeldEdge, SheetFrame,
};
use crate::model::{Look, PartLooks};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::material::MaterialId;
use crate::rendering::pattern;
use crate::rendering::substance::{self, ColliderSubstance, Substance};
use crate::systems::PhysicsResource;

const TEXTURE_SIZE: u32 = 128;

#[derive(Deserialize)]
pub struct GlassSheetDef {
    /// Centre of the pane.
    pub pos: (f32, f32, f32),

    /// Width and height of the pane, in metres. Width runs along the pane's
    /// own `+X`; height runs up an upright pane and along `+Z` for one that
    /// is lying flat.
    #[serde(default = "GlassSheetDef::default_size")]
    pub size: (f32, f32),

    /// Thickness, in metres. Thin glass makes thin shards, and a shard
    /// cannot be much wider than sixty times its thickness, so a thinner
    /// pane crazes into more, smaller pieces.
    #[serde(default = "GlassSheetDef::default_thickness")]
    pub thickness: f32,

    /// Rotation about `+Y`, in degrees.
    #[serde(default)]
    pub yaw: f32,

    /// Flat like a floor rather than upright like a window.
    #[serde(default)]
    pub lying: bool,

    /// Held along its two long edges, welded to the world where it stands,
    /// as a pane spanning a gap is. Off gives a loose sheet that falls over
    /// and shatters on whatever it lands on.
    #[serde(default = "GlassSheetDef::default_fixed")]
    pub fixed: bool,

    /// Contact spike, in N·s, that cracks the glass. A footstep is a few
    /// N·s; a thrown crate is tens.
    #[serde(default = "GlassSheetDef::default_impact_threshold")]
    pub impact_threshold: f32,

    /// Blast loading, in N·s/m², that breaks a shard free of its neighbours.
    #[serde(default = "GlassSheetDef::default_blast_threshold")]
    pub blast_threshold: f32,

    /// Force, in newtons, the pane bears indefinitely. Zero means it never
    /// tires; a value below a person's weight makes a floor that has to be
    /// crossed without stopping.
    #[serde(default)]
    pub bearing: f32,

    /// Seconds a cell lasts at twice its bearing.
    #[serde(default = "GlassSheetDef::default_endurance")]
    pub endurance: f32,
}

impl GlassSheetDef {
    pub fn default_size() -> (f32, f32) {
        (2.0, 1.5)
    }

    pub fn default_thickness() -> f32 {
        0.015
    }

    pub fn default_fixed() -> bool {
        true
    }

    /// Measured: a 20 kg crate dropped from a metre spikes at about 60 N·s
    /// on a held pane, a person walking across one at 5–15 N·s. A window
    /// takes the walker and not the crate.
    pub fn default_impact_threshold() -> f32 {
        50.0
    }

    /// The plank bridge's number: a grenade next to it takes shards out, a
    /// grenade across the room does not.
    pub fn default_blast_threshold() -> f32 {
        8.0
    }

    pub fn default_endurance() -> f32 {
        0.6
    }

    fn substance() -> Substance {
        substance::GLASS
    }

    /// Every shard the whole pane crazes into around `hit`, a point in its
    /// plane from its centre, cut as the crack system cuts them: each hull
    /// with its offset from the pane's centre, in the pane's own axes. Empty
    /// if no layout for `salt` produced shards the engine can take.
    pub fn shards(&self, hit: Vector2<f32>, salt: u32) -> Vec<(ConvexHull, Vector3<f32>)> {
        let frame = self.frame();
        let (half_u, half_v) = self.half_size();
        let pane = ConvexPolygon::rectangle(Vector2::zeros(), half_u, half_v);
        self.crazing()
            .craze(&pane, hit, frame.thickness, salt)
            .unwrap_or_default()
            .iter()
            .map(|cell| frame.shard(cell, 0.0))
            .collect()
    }

    fn frame(&self) -> SheetFrame {
        if self.lying {
            SheetFrame::lying(self.thickness)
        } else {
            SheetFrame::upright(self.thickness)
        }
    }

    /// Half-extents along the frame's `u` and `v`.
    fn half_size(&self) -> (f32, f32) {
        let (width, height) = (self.size.0 * 0.5, self.size.1 * 0.5);
        // The lying frame runs `u` along `+Z`, so `height` is the `u` extent.
        if self.lying {
            (height, width)
        } else {
            (width, height)
        }
    }

    fn crazing(&self) -> CrazeRule {
        CrazeRule {
            threshold: self.impact_threshold,
            web: CrackWeb {
                core_radius: 0.08,
                ring_growth: 1.7,
                rings: 5,
                background_spacing: 0.35,
            },
            min_area: 0.002,
        }
    }

    fn fatigue(&self) -> Option<FatigueRule> {
        (self.bearing > 0.0).then_some(FatigueRule {
            bearing: self.bearing,
            endurance: self.endurance,
        })
    }
}

impl Spawnable for GlassSheetDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        Ok(vec![ctx.patterned(
            &Self::substance(),
            &pattern::GLASS,
            seed_from_position(self.pos, 0),
            TEXTURE_SIZE,
        )?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let material = materials[0];
        let centre = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let rotation = Yaw::degrees(self.yaw).rotation();
        let frame = self.frame();
        let (half_u, half_v) = self.half_size();
        let half_extents = frame.box_half_extents(half_u, half_v);

        let mut physics = world.write_resource::<PhysicsResource>();
        let body = physics.world.create_body(
            RigidBodyDesc::dynamic()
                .linear_damping(0.01)
                .angular_damping(0.005)
                .position(centre)
                .rotation(rotation),
        );
        let Some(pane) = physics.world.attach_collider(
            body,
            ColliderDesc::box_shape(half_extents).of(&Self::substance()),
        ) else {
            return Vec::new();
        };

        let mut graph = PartGraph::default();
        let pane_part = graph.add_part([pane]);
        let glass = Cuttable::new(Arc::new(GlassFaces { material })).with(pane, Vector3::zeros());
        let mut sheet = BrittleSheet::whole(frame, self.crazing(), self.blast_threshold, glass);
        if let Some(fatigue) = self.fatigue() {
            sheet = sheet.wearing_out_under(fatigue, 0.35);
        }
        if self.fixed {
            let whole = ConvexPolygon::rectangle(Vector2::zeros(), half_u, half_v);
            let pose = Isometry3::from_parts(centre.coords.into(), rotation);
            for edge in self.long_edges(&frame, &pose) {
                if let Some((weld, held)) =
                    weld_along(&mut physics.world, body, &frame, &whole, &edge)
                {
                    graph.tie(weld, pane_part);
                    sheet.held.insert(weld, held);
                }
            }
        }

        // Contact between shards is judged on the same spike that cracks the
        // pane: a shard hit hard enough to craze is hit hard enough to fall.
        let fracture = CompoundFracture::new(graph)
            .breaking_on_impact_at(self.impact_threshold)
            .shedding_debris();
        let looks = PartLooks::new([(pane, cuboid_look(half_extents, material))]);
        let model = looks
            .drawn(&physics.world, body)
            .expect("a pane has something to draw");
        drop(physics);

        vec![world
            .create_entity()
            .with(Position(centre.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(rotation))
            .with(RigidBodyComponent(body))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .with(looks)
            .with(fracture)
            .with(sheet)
            .build()]
    }
}

impl GlassSheetDef {
    /// The pane's two long edges, in the world, at `pose`.
    fn long_edges(&self, frame: &SheetFrame, pose: &Isometry3<f32>) -> [HeldEdge; 2] {
        let (half_u, half_v) = self.half_size();
        let corner = |u: f32, v: f32| pose * Point3::from(frame.lift(Vector2::new(u, v), 0.0));
        if half_u >= half_v {
            [-half_v, half_v].map(|v| HeldEdge {
                from: corner(-half_u, v),
                to: corner(half_u, v),
            })
        } else {
            [-half_u, half_u].map(|u| HeldEdge {
                from: corner(u, -half_v),
                to: corner(u, half_v),
            })
        }
    }
}

/// How a shard of glass is made: a hull of glass, drawn as it collides with
/// the pattern fitted to each face, as the pane is.
pub(crate) struct GlassFaces {
    /// What the glass is drawn with.
    pub material: MaterialId,
}

impl FaceMesher for GlassFaces {
    fn collider(&self, hull: Arc<ConvexHull>) -> ColliderDesc {
        ColliderDesc::convex_hull(hull).of(&GlassSheetDef::substance())
    }

    fn look(&self, hull: &ConvexHull, at: Vector3<f32>) -> Look {
        let (vertices, indices) = hull_mesh(&PieceHull::new(hull, at), SurfaceUvs::Fitted);
        Look::single(vertices, indices, self.material)
    }
}
