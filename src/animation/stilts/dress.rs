//! What a stilt walker is seen as: a model whose parts each ride a joint.
//!
//! ```text
//!   joint    a part modelled for it has its origin at…          and +Y…
//!   ───────  ─────────────────────────────────────────────────  ─────────────
//!   Hips     the pelvis, level, +Z the way the walker faces     up
//!   Neck     the shoulder                                       to the head
//!   Head     the head's middle, +Z the way it looks             its crown
//!   Eye      its own middle, +Z out through its pupil           its top
//!   Hinge    its pivot, on the head                             the head's
//!   Thigh    the hip, +Z the way the walker faces               to the hip,
//!   Shin     the knee                                           from below
//!   Foot     the middle of the sole's print, +Z to the toes     the instep
//! ```
//!
//! The parts are rigid and only their poses change, so each may be as finely
//! modelled as its maker likes. Where two meet (a shin on a thigh) each is
//! modelled a little past the joint, and the overlap hides the turn.

use std::sync::Arc;

use nalgebra::Vector3;

use crate::animation::FootSide;
use crate::model::{MeshPrimitive, Model, ModelPart};

/// What turns a part hinged on the head.
#[derive(Clone, Copy, Debug)]
pub enum Drive {
    /// A lid: turns `shut` radians as the eye closes, blinks included.
    Lid { shut: f32 },
    /// A mouth gasping: opens `open` radians and shuts again, `rate` times
    /// a second.
    Gasp { open: f32, rate: f32 },
    /// A jaw that strikes: opens `open` radians as the strike winds up, and
    /// snaps shut as it lands.
    Strike { open: f32 },
}

/// Where on the rig a part rides.
#[derive(Clone, Copy, Debug)]
pub enum Joint {
    Hips,
    Neck,
    Head,
    /// An eye in the head, its middle at `socket` in the head's frame. It
    /// turns in its socket to look at what the walker watches, as far as
    /// `swivel` radians off the way the head faces.
    Eye {
        socket: Vector3<f32>,
        swivel: f32,
    },
    /// A part hinged on the head at `pivot`, turning about `axis` (both in
    /// the head's frame) as `drive` says.
    Hinge {
        pivot: Vector3<f32>,
        axis: Vector3<f32>,
        drive: Drive,
    },
    Thigh(FootSide),
    Shin(FootSide),
    Foot(FootSide),
}

/// How one part of the model is worn.
#[derive(Clone, Copy, Debug)]
pub struct Worn {
    pub joint: Joint,
    /// How far it swells and sinks as the walker breathes, as a share of
    /// its width. Zero for anything that is not flesh.
    pub breath: f32,
}

impl Worn {
    /// Worn on `joint`, still.
    pub fn on(joint: Joint) -> Self {
        Self { joint, breath: 0.0 }
    }

    /// Swelling by `breath` of its width with each breath.
    pub fn breathing(mut self, breath: f32) -> Self {
        self.breath = breath;
        self
    }
}

/// A walker's whole look.
pub struct Dress {
    /// One part to a joint ridden, each modelled about its joint.
    pub model: Arc<Model>,
    /// How each part of the model is worn, in part order.
    pub worn: Vec<Worn>,
}

impl Dress {
    /// A dress of these pieces: each a way of being worn, and the meshes
    /// worn that way.
    pub fn of(pieces: Vec<(Worn, Vec<MeshPrimitive>)>) -> Self {
        let (worn, parts): (Vec<Worn>, Vec<ModelPart>) = pieces
            .into_iter()
            .map(|(worn, meshes)| (worn, ModelPart::new(meshes)))
            .unzip();
        Self {
            model: Arc::new(Model::flat(parts)),
            worn,
        }
    }

    /// The same dress drawn in other materials: `material` gives each of
    /// its primitives' materials its replacement.
    pub fn recoloured(
        &self,
        material: impl Fn(
            crate::rendering::material::MaterialId,
        ) -> crate::rendering::material::MaterialId,
    ) -> Self {
        let mut model = (*self.model).clone();
        for primitive in model.parts.iter_mut().flat_map(|part| &mut part.primitives) {
            primitive.material = material(primitive.material);
        }
        Self {
            model: Arc::new(model),
            worn: self.worn.clone(),
        }
    }

    /// Triangles in the whole dress.
    pub fn triangles(&self) -> usize {
        let meshes = self.model.parts.iter().flat_map(|part| &part.primitives);
        meshes.map(|mesh| mesh.indices.len() / 3).sum()
    }
}
