//! Cutting makes parts: one part of a compound replaced by the pieces it
//! broke into, each with its collider, bulk and look made as it is cut.
//!
//! ```text
//!   cutter (glass, cleave) ── hulls ──▶ Cuttable ── FaceMesher ──▶ CutPiece
//!                                        (where each collider       collider
//!                                         sits in its texture)      + look
//!                                                                      │
//!   body: [ a ][ b ][ c ]                replace_part(b, pieces) ◀─────┘
//!                 │
//!                 ▼
//!   body: [ a ][ c ][ b1 ][ b2 ]   b's bonds and ties go to the piece
//!                                  nearest each anchor
//! ```
//!
//! What a piece *is* — how it is meshed, in which texture frame, knowing the
//! whole it was cut from — is the spawnable's: it implements [`FaceMesher`]
//! and hands it to the cutter's own component. Where a piece's shape comes
//! from is the cutter's: a sheet of glass cuts a polygon into shards and a
//! block of ice cleaves along a plane.

use std::sync::Arc;

use nalgebra::{Isometry3, Point3, Vector3};
use rustc_hash::FxHashMap;

use super::components::CompoundFracture;
use super::parts::{NewPart, PartId};
use crate::collision::convex_hull::ConvexHull;
use crate::model::{Look, PartLooks};
use crate::physics::{ColliderDesc, ColliderHandle, RigidBodyHandle};
use crate::systems::PhysicsResource;

/// How a spawnable's brittle pieces are made when they are cut: what each is
/// made of and how it looks.
pub trait FaceMesher: Send + Sync {
    /// The collider of a piece shaped `hull`, unplaced: its substance and any
    /// bulk it declares.
    fn collider(&self, hull: Arc<ConvexHull>) -> ColliderDesc;

    /// How a piece shaped `hull` looks, in its own frame. `at` is where that
    /// frame's origin sits in the frame the object's texture was laid in, so
    /// a piece reads the pattern its object wore there.
    fn look(&self, hull: &ConvexHull, at: Vector3<f32>) -> Look;
}

/// One piece a cut made, ready to replace the part it came from.
pub struct CutPiece {
    /// Its collider, placed in the body's frame.
    pub collider: ColliderDesc,
    /// How it looks, in its collider's frame.
    pub look: Arc<Look>,
    /// Where its collider's origin sits in the object's texture frame.
    pub texture_at: Vector3<f32>,
    /// Its centre in the body's frame, which decides the bonds and ties it
    /// inherits.
    pub centre: Point3<f32>,
    /// Whether it may inherit the old part's bonds and ties: false for a
    /// piece the cutter knocks out at once.
    pub inherits: bool,
}

/// A piece [`replace_part`] put on the body.
#[derive(Debug, Clone, Copy)]
pub struct MadePart {
    /// Its place in the pieces given.
    pub index: usize,
    pub part: PartId,
    pub collider: ColliderHandle,
    /// Where its collider's origin sits in the object's texture frame.
    pub texture_at: Vector3<f32>,
}

/// The colliders of a compound a cutter may cut, each with where it sits in
/// the frame the object's texture was laid in, and the spawnable's
/// [`FaceMesher`] that makes the pieces.
///
/// Keyed by collider handle, so a piece's share is a filter.
#[derive(Clone)]
pub struct Cuttable {
    mesher: Arc<dyn FaceMesher>,
    /// Where each cuttable collider's origin sits in the texture frame.
    frames: FxHashMap<ColliderHandle, Vector3<f32>>,
}

impl Cuttable {
    /// Nothing cuttable yet, its pieces made by `mesher`.
    pub fn new(mesher: Arc<dyn FaceMesher>) -> Self {
        Self {
            mesher,
            frames: FxHashMap::default(),
        }
    }

    /// `collider` may be cut, and sits at `texture_at` in the texture frame.
    pub fn with(mut self, collider: ColliderHandle, texture_at: Vector3<f32>) -> Self {
        self.frames.insert(collider, texture_at);
        self
    }

    /// Whether `collider` is one the cutter may cut.
    pub fn covers(&self, collider: ColliderHandle) -> bool {
        self.frames.contains_key(&collider)
    }

    /// Where `collider` sits in the texture frame.
    pub fn texture_at(&self, collider: ColliderHandle) -> Option<Vector3<f32>> {
        self.frames.get(&collider).copied()
    }

    /// The pieces `old`, at `old_offset` in the body, is cut into: each a
    /// hull and where its frame sits in the body.
    pub fn pieces(
        &self,
        old: ColliderHandle,
        old_offset: &Isometry3<f32>,
        cuts: impl IntoIterator<Item = (Arc<ConvexHull>, Isometry3<f32>)>,
    ) -> Vec<CutPiece> {
        let origin = self.texture_at(old).unwrap_or_else(Vector3::zeros);
        cuts.into_iter()
            .map(|(hull, offset)| {
                let texture_at =
                    origin + (offset.translation.vector - old_offset.translation.vector);
                CutPiece {
                    look: Arc::new(self.mesher.look(&hull, texture_at)),
                    collider: self
                        .mesher
                        .collider(hull)
                        .offset_translation(offset.translation.vector)
                        .offset_rotation(offset.rotation),
                    texture_at,
                    centre: Point3::from(offset.translation.vector),
                    inherits: true,
                }
            })
            .collect()
    }

    /// `old` is gone and `made` replaced it.
    pub fn record(&mut self, old: ColliderHandle, made: &[MadePart]) {
        self.frames.remove(&old);
        for piece in made {
            self.frames.insert(piece.collider, piece.texture_at);
        }
    }

    /// What a piece split off with `colliders` may cut.
    pub fn share(&self, colliders: &[ColliderHandle]) -> Self {
        Self {
            mesher: self.mesher.clone(),
            frames: colliders
                .iter()
                .filter_map(|c| Some((*c, *self.frames.get(c)?)))
                .collect(),
        }
    }

    /// Forget every collider but `colliders`.
    pub fn retain(&mut self, colliders: &[ColliderHandle]) {
        self.frames.retain(|c, _| colliders.contains(c));
    }
}

/// Replace `part` of the compound on `body` with `pieces`, a part each: its
/// colliders and looks swapped for theirs, its bonds and ties passed to the
/// inheriting piece nearest each anchor.
///
/// The body is not re-centred: pieces that fill the volume the part did
/// leave its mass where it was. A new part arrives bonded only to what it
/// inherited, with no load history and no kick; the cutter says what its
/// pieces are bonded to among themselves.
///
/// `None` if the part is not on this body. A piece the engine refuses is
/// left out of what is returned.
pub fn replace_part(
    physics: &mut PhysicsResource,
    fracture: &mut CompoundFracture,
    looks: &mut PartLooks,
    body: RigidBodyHandle,
    part: PartId,
    pieces: Vec<CutPiece>,
) -> Option<Vec<MadePart>> {
    let old = fracture.graph.colliders_of(&[part]);
    let on_body = physics.world.body(body)?.colliders();
    if old.is_empty() || !old.iter().all(|c| on_body.contains(c)) {
        return None;
    }

    let world = &mut physics.world;
    for collider in &old {
        world.detach_collider(body, *collider);
        looks.remove(*collider);
    }
    let mut made = Vec::with_capacity(pieces.len());
    let mut new_parts = Vec::with_capacity(pieces.len());
    for (index, piece) in pieces.into_iter().enumerate() {
        let Some(collider) = world.attach_collider(body, piece.collider) else {
            continue;
        };
        looks.insert(collider, piece.look);
        made.push((index, collider, piece.texture_at));
        new_parts.push(NewPart {
            colliders: [collider].into_iter().collect(),
            centre: piece.centre,
            inherits: piece.inherits,
        });
    }
    world.wake_body(body);

    let tie_anchor = |constraint| {
        world
            .constraint(constraint)?
            .kind
            .anchor_on(body)
            .map(Point3::from)
    };
    let ids = fracture.graph.replace(part, new_parts, tie_anchor)?;
    fracture.kicks.remove(&part);
    Some(
        made.into_iter()
            .zip(ids)
            .map(|((index, collider, texture_at), part)| MadePart {
                index,
                part,
                collider,
                texture_at,
            })
            .collect(),
    )
}
