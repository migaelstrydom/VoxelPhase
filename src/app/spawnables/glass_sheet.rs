//! Glass sheet — a pane that cracks where it is hit and drops the shards.
//!
//! ```text
//!   GlassSheetDef ──▶ substance::GLASS ──┬──▶ one box collider   (the pane)
//!                                        ├──▶ CompoundFracture  (no joints yet)
//!                                        └──▶ BrittleSheet      (how it cracks)
//! ```
//!
//! It spawns as one rigid body with one collider and one cuboid model. That
//! is all it is until something hits it: then the glass system replaces the
//! collider under the hit with a web of convex shards, the fracture system
//! knocks out the ones directly under the blow, and the rest stay in place,
//! cracked, holding each other up — until the next hit, or, for a pane that
//! is stood on, until it has borne the weight for longer than it can.
//!
//! Two kinds of pane come out of the same definition:
//!
//! - a **window**: upright, fixed in place, with an impact threshold a
//!   shoulder does not reach but a thrown crate does;
//! - a **floor**: lying flat, fixed, with a bearing capacity below a
//!   person's weight, so it crazes under a footstep and gives way under
//!   someone who stops walking.

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::models::cuboid_model;
use super::shared::orientation::Yaw;
use super::shared::textures::seed_from_position;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::fracture::CompoundFracture;
use crate::glass::{BrittleSheet, CrackWeb, CrazeRule, FatigueRule, SheetFrame};
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

    /// Held in place, as a pane in a frame is. Off gives a loose sheet that
    /// falls over and shatters on whatever it lands on.
    #[serde(default = "GlassSheetDef::default_fixed")]
    pub fixed: bool,

    /// Contact spike, in N·s, that cracks the glass. A footstep is a few
    /// N·s; a thrown crate is tens.
    #[serde(default = "GlassSheetDef::default_impact_threshold")]
    pub impact_threshold: f32,

    /// Blast impulse, in N·s, that breaks a shard free of its neighbours.
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
    /// on a fixed pane, a person walking across one at 5–15 N·s. A window
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

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();
            let desc = if self.fixed {
                RigidBodyDesc::static_body()
            } else {
                RigidBodyDesc::dynamic()
                    .linear_damping(0.01)
                    .angular_damping(0.005)
            };
            let body = physics
                .world
                .create_body(desc.position(centre).rotation(rotation));
            physics.world.attach_collider(
                body,
                ColliderDesc::box_shape(half_extents).of(&Self::substance()),
            );
            body
        };

        // Contact between shards is judged on the same spike that cracks the
        // pane: a shard hit hard enough to craze is hit hard enough to fall.
        let fracture = CompoundFracture::boxes(Vec::new(), 1, material)
            .breaking_on_impact_at(self.impact_threshold)
            .shedding_debris();
        let mut sheet = BrittleSheet::whole(frame, self.crazing(), self.blast_threshold, material);
        if let Some(fatigue) = self.fatigue() {
            sheet = sheet.wearing_out_under(fatigue, 0.35);
        }

        vec![world
            .create_entity()
            .with(Position(centre.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation(rotation))
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(cuboid_model(half_extents, material)))
            .with(Renderable)
            .with(fracture)
            .with(sheet)
            .build()]
    }
}
