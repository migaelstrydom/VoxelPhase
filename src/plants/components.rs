//! A plant's ECS components.

use std::sync::Arc;

use nalgebra::Isometry3;
use specs::{Component, VecStorage};

use super::bark::LimbBase;
use super::bodies::{Joinery, PlantColliders};
use super::drawing::PlantDrawing;
use super::plant::Plant;
use crate::foliage::{Foliage, Rooting};
use crate::model::PartLooks;
use crate::physics::{PhysicsWorld, RigidBodyHandle};

/// A plant as grown and built: how it is built, joined and drawn, and what
/// each limb and crown became. Shared by every piece of it.
#[derive(Debug)]
pub struct GrownPlant {
    /// Its limbs and leaves.
    pub plant: Plant,
    /// How its body is made and broken.
    pub joinery: Joinery,
    /// Its meshes.
    pub drawing: PlantDrawing,
    /// The colliders and parts its limbs and crowns became.
    pub colliders: PlantColliders,
}

impl GrownPlant {
    /// The limbs on `body`, by index.
    pub fn limbs_on(&self, world: &PhysicsWorld, body: RigidBodyHandle) -> Vec<usize> {
        world.body(body).map_or_else(Vec::new, |body| {
            self.colliders.limbs_among(body.colliders())
        })
    }

    /// Where the plant's frame sits in `body`'s, read off limb `l`'s
    /// collider on it. `None` if the collider is gone.
    pub fn frame_on(&self, world: &PhysicsWorld, l: usize) -> Option<Isometry3<f32>> {
        let collider = world.collider(self.colliders.wood[l])?;
        Some(collider.offset() * self.plant.limbs.limbs[l].pose().inverse())
    }

    /// How the piece of the plant holding `limbs` looks, the plant's frame
    /// at `frame` in its body's: each limb's bark on its capsule, snapped
    /// for a branch off the limb it grew from, and each crown's leaves on
    /// one of its core pieces, the rest undrawn.
    pub fn looks(
        &self,
        world: &PhysicsWorld,
        limbs: &[usize],
        frame: &Isometry3<f32>,
    ) -> PartLooks {
        let bark = limbs.iter().map(|&l| {
            let limb = &self.plant.limbs.limbs[l];
            (
                self.colliders.wood[l],
                self.drawing.bark(l, self.base_of(limbs, l), &limb.pose()),
            )
        });
        let leaves = self.plant.masses_on(limbs).into_iter().filter_map(|m| {
            let piece = *self.colliders.cores[m].first()?;
            let at = frame.inverse() * world.collider(piece)?.offset();
            Some((piece, self.drawing.leaves(m, &at)))
        });
        PartLooks::new(bark.chain(leaves))
    }

    /// Gives every branch in `limbs` whose parent is not among them its
    /// snapped bark in `looks`.
    pub fn snap_broken(&self, limbs: &[usize], looks: &mut PartLooks) {
        for &l in limbs {
            if self.base_of(limbs, l) == LimbBase::Snapped {
                let limb = &self.plant.limbs.limbs[l];
                looks.insert(
                    self.colliders.wood[l],
                    self.drawing.bark(l, LimbBase::Snapped, &limb.pose()),
                );
            }
        }
    }

    /// The leaves of every crown `limbs` hold, as one clump riding on
    /// `body` with the plant's frame at `frame` in it. `None` if they hold
    /// none.
    pub fn foliage(
        &self,
        limbs: &[usize],
        body: RigidBodyHandle,
        frame: Isometry3<f32>,
    ) -> Option<Foliage> {
        let masses = self.plant.masses_on(limbs);
        if masses.is_empty() {
            return None;
        }
        let rooting = Rooting::Loose {
            body,
            offset: frame,
        };
        let mut clump = Foliage::new(Arc::new(self.plant.leaves_of(&masses)), rooting);
        clump.core = self.joinery.core;
        Some(clump)
    }

    /// How limb `l`'s base is drawn in a piece holding `limbs`: snapped
    /// for a branch whose parent is not in it, rounded otherwise.
    fn base_of(&self, limbs: &[usize], l: usize) -> LimbBase {
        if self.plant.limbs.broken_off(l, limbs) {
            LimbBase::Snapped
        } else {
            LimbBase::Rounded
        }
    }
}

/// A piece of a plant: the whole of it until a branch breaks off, which is
/// then a piece of its own.
///
/// On the piece's entity beside its `CompoundFracture` and `PartLooks`, and
/// the `Foliage` of the crowns it holds, which `PlantShareSystem` rebuilds
/// whenever a piece splits off.
#[derive(Component)]
#[storage(VecStorage)]
pub struct PlantPiece {
    /// The plant it is a piece of.
    pub plant: Arc<GrownPlant>,
}
