//! A plant built in the physics world: one compound body holding every
//! limb and the core of every crown, its parts bonded where branches fork
//! and the trunk welded into the ground.
//!
//! ```text
//!   ground ══weld══ [ trunk · roots · main crown │ branch + its crown │ … ]
//!            (a tie)  one body; one part each      bonded at the fork
//!
//!   fracture: a bond's load at its fork past what it holds ──▶ the branch
//!   comes off as a piece of its own (PieceSplit)
//! ```
//!
//! A body's colliders never meet one another, so a branch overlapping the
//! trunk at its fork, or roots crossing under the foot, cost nothing while
//! the tree is whole; once a branch breaks off, it and what it left meet
//! like any two bodies.
//!
//! What fluids and blasts see of a plant is its crowns' leaves: each crown's
//! core carries them as a loose clump's does ([`LeafCore::colliders`]), and
//! the wood, which adds little to them, declares an empty envelope. Each
//! collider's bulk goes with it when a branch breaks off.

use nalgebra::{Isometry3, Point3};

use super::limbs::LimbKind;
use super::plant::Plant;
use crate::foliage::LeafCore;
use crate::fracture::{Bond, CompoundFracture, PartGraph, PartId};
use crate::physics::{
    BulkShape, ColliderHandle, ConstraintHandle, ConstraintKind, PhysicsWorld, RigidBodyDesc,
    RigidBodyHandle,
};
use crate::rendering::substance::{self, ColliderSubstance, Substance};

/// How a plant's body is made, held together and broken.
#[derive(Debug, Clone, Copy)]
pub struct Joinery {
    /// What the limbs are made of.
    pub wood: Substance,
    /// Blast loading at a branch's fork, N·s/m², past which it breaks off:
    /// a grenade 2 m beyond a side crown loads its fork with about 680, one
    /// 3 m beyond with about 290, so the first takes the branch off and the
    /// second does not. Every branch off one fork breaks together.
    pub branch_strength: f32,
    /// Contact spike felt at a branch's fork, N·s, past which it breaks
    /// off: a tree felled onto its crowns puts about 130 through a fork,
    /// so it keeps its branches; one dropped lying on its side from 6 m
    /// about 310, from 15 m about 580 and from 20 m about 660, so a hard
    /// landing takes a branch off.
    pub branch_toughness: f32,
    /// How each crown's core is built.
    pub core: LeafCore,
}

impl Default for Joinery {
    fn default() -> Self {
        Self {
            wood: substance::OAK,
            branch_strength: 500.0,
            branch_toughness: 220.0,
            core: LeafCore::default(),
        }
    }
}

/// Which colliders a plant's limbs and crowns became, and the part each
/// limb is in. Handles and part ids last whichever body a piece of the
/// plant ends up on, so one map serves every piece.
#[derive(Debug, Clone)]
pub struct PlantColliders {
    /// Each limb's capsule, by limb index.
    pub wood: Vec<ColliderHandle>,
    /// Each crown's core pieces, by leaf mass index.
    pub cores: Vec<Vec<ColliderHandle>>,
    /// The part each limb is in, by limb index: a branch's own, the trunk's
    /// for the trunk and its roots.
    pub parts: Vec<PartId>,
}

impl PlantColliders {
    /// The limbs whose wood is among `colliders`, by index.
    pub fn limbs_among(&self, colliders: &[ColliderHandle]) -> Vec<usize> {
        (0..self.wood.len())
            .filter(|&l| colliders.contains(&self.wood[l]))
            .collect()
    }
}

/// A plant just built: its body, where the plant's frame sits in it, and
/// its parts and bonds.
pub struct BuiltPlant {
    /// The body.
    pub body: RigidBodyHandle,
    /// Where the plant's frame sits in the body's: the body's origin is its
    /// centre of mass.
    pub frame: Isometry3<f32>,
    /// What each limb and crown became.
    pub colliders: PlantColliders,
    /// A part for the trunk with its roots and its crown, and one for each
    /// branch with its crown, bonded to the limb it grows from at the fork.
    pub graph: PartGraph,
}

impl Joinery {
    /// `plant` built in `world` as one body with the plant's frame at `at`.
    pub fn build(&self, world: &mut PhysicsWorld, plant: &Plant, at: Isometry3<f32>) -> BuiltPlant {
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::from(at.translation.vector))
                .rotation(at.rotation),
        );
        let limbs = &plant.limbs.limbs;
        let wood: Vec<ColliderHandle> = limbs
            .iter()
            .map(|limb| {
                let collider = limb
                    .collider()
                    .of(&self.wood)
                    .bulk(BulkShape::displacing(Vec::new()));
                world
                    .attach_collider(body, collider)
                    .expect("a body just made takes colliders")
            })
            .collect();
        let cores: Vec<Vec<ColliderHandle>> = plant
            .masses
            .iter()
            .map(|mass| {
                self.core
                    .colliders(&mass.shape, mass.pose)
                    .into_iter()
                    .filter_map(|collider| world.attach_collider(body, collider))
                    .collect()
            })
            .collect();
        let moved = world.recenter_on_colliders(body);
        let frame = Isometry3::translation(-moved.x, -moved.y, -moved.z);

        // Each limb is in its own part if it can break off, else in the one
        // it grows from; limbs come after their parents.
        let mut graph = PartGraph::default();
        let mut owner: Vec<usize> = Vec::with_capacity(limbs.len());
        for (l, limb) in limbs.iter().enumerate() {
            owner.push(match (limb.kind, limb.parent) {
                (LimbKind::Root, Some(parent)) => owner[parent],
                _ => l,
            });
        }
        let part_of_owner: Vec<Option<PartId>> = (0..limbs.len())
            .map(|o| {
                let held: Vec<usize> = (0..limbs.len()).filter(|&l| owner[l] == o).collect();
                if held.is_empty() {
                    return None;
                }
                let colliders = held.iter().map(|&l| wood[l]).chain(
                    plant
                        .masses_on(&held)
                        .into_iter()
                        .flat_map(|m| cores[m].iter().copied()),
                );
                Some(graph.add_part(colliders))
            })
            .collect();
        let parts: Vec<PartId> = owner
            .iter()
            .map(|&o| part_of_owner[o].expect("every owner holds itself"))
            .collect();
        for (l, limb) in limbs.iter().enumerate() {
            let Some(parent) = limb.parent.filter(|_| limb.kind == LimbKind::Branch) else {
                continue;
            };
            graph.bond(Bond {
                a: parts[parent],
                b: parts[l],
                anchor: frame * limb.base,
                // A blow on the crown out at the branch's tip is felt at
                // half strength at the fork.
                reach: (limb.tip - limb.base).magnitude(),
                threshold: self.branch_strength,
                toughness: None,
            });
        }
        BuiltPlant {
            body,
            frame,
            colliders: PlantColliders { wood, cores, parts },
            graph,
        }
    }

    /// Welds `built`'s trunk into the ground where it stands, ties it to
    /// the trunk's part, and lets the body pass through the ground its foot
    /// and roots are buried in until the weld is removed.
    pub fn root(
        &self,
        world: &mut PhysicsWorld,
        plant: &Plant,
        built: &mut BuiltPlant,
    ) -> ConstraintHandle {
        world.set_ignores_static(built.body, true);
        let body = world
            .body(built.body)
            .expect("a plant just built has a body");
        let body_pose = Isometry3::from_parts(body.position().coords.into(), body.rotation());
        let foot = body_pose * built.frame * plant.limbs.limbs[0].base;
        let weld = world.create_constraint(ConstraintKind::Fixed {
            body_a: None,
            body_b: built.body,
            local_anchor_a: foot.coords,
            local_anchor_b: (body_pose.inverse() * foot).coords,
            reference: body_pose.rotation,
            compliance: 0.0,
            max_impulse: f32::MAX,
        });
        built.graph.tie(weld, built.colliders.parts[0]);
        weld
    }

    /// The compound `graph` makes, breaking by this joinery's rules.
    pub fn fracture(&self, graph: PartGraph) -> CompoundFracture {
        CompoundFracture::new(graph).breaking_on_impact_at(self.branch_toughness)
    }
}
