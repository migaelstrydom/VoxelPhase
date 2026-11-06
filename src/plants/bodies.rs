//! A plant built in the physics world: one compound body holding every
//! limb and the core of every crown, the trunk welded into the ground.
//!
//! ```text
//!   ground ══weld══ [ trunk · roots · main crown · branch + its crown · … ]
//!                     one body: its own parts never meet
//!
//!   a blast on a branch's crown past its strength ──break_off──▶
//!   [ trunk · roots · main crown · … ]      [ branch + its crown ]
//!                     two bodies, which meet like any others
//! ```
//!
//! A body's colliders never meet one another, so a branch overlapping the
//! trunk at its fork, or roots crossing under the foot, cost nothing while
//! the tree is whole; once a branch breaks off, it and what it left meet
//! like any two bodies.

use nalgebra::{Isometry3, Point3, Translation3, Vector3};

use super::limbs::LimbKind;
use super::plant::Plant;
use crate::foliage::LeafCore;
use crate::physics::{
    ColliderHandle, ConstraintHandle, ConstraintKind, PhysicsImpulse, PhysicsWorld, RigidBodyDesc,
    RigidBodyHandle,
};
use crate::rendering::substance::{self, ColliderSubstance, Substance};

/// How a plant's bodies are made, held together and broken.
#[derive(Debug, Clone, Copy)]
pub struct Joinery {
    /// What the limbs are made of.
    pub wood: Substance,
    /// The blast impulse on a branch's crowns, N·s, past which the branch
    /// breaks off: a grenade within about 2 m of a branch's crown takes it
    /// off, one 3 m away does not.
    pub branch_strength: f32,
    /// How each crown's core is built.
    pub core: LeafCore,
}

impl Default for Joinery {
    fn default() -> Self {
        Self {
            wood: substance::OAK,
            branch_strength: 250.0,
            core: LeafCore::default(),
        }
    }
}

/// A piece of a plant built in the world: some of its limbs and the crowns
/// they hold, as one body. A plant starts as one piece and loses branches
/// as pieces of their own.
#[derive(Debug, Clone)]
pub struct PlantPieceBody {
    /// The body.
    pub body: RigidBodyHandle,
    /// Where the plant's frame sits in the body's: the body's origin is its
    /// centre of mass.
    pub frame: Isometry3<f32>,
    /// Each limb's colliders, by limb index: its wood and the cores of the
    /// crowns it holds. Empty for a limb in another piece.
    pub colliders: Vec<Vec<ColliderHandle>>,
}

impl PlantPieceBody {
    /// The limbs in this piece, by index.
    pub fn limbs(&self) -> Vec<usize> {
        (0..self.colliders.len())
            .filter(|&l| !self.colliders[l].is_empty())
            .collect()
    }

    /// Whether limb `l` is in this piece.
    pub fn holds(&self, l: usize) -> bool {
        self.colliders.get(l).is_some_and(|c| !c.is_empty())
    }

    /// Where the plant's frame is in the world now. `None` if the body has
    /// gone.
    pub fn plant_pose(&self, world: &PhysicsWorld) -> Option<Isometry3<f32>> {
        let body = world.body(self.body)?;
        Some(Isometry3::from_parts(body.position().coords.into(), body.rotation()) * self.frame)
    }

    /// The branches that could break off this piece: those whose parent is
    /// in it too.
    pub fn branches(&self, plant: &Plant) -> Vec<usize> {
        self.limbs()
            .into_iter()
            .filter(|&l| {
                let limb = &plant.limbs.limbs[l];
                limb.kind == LimbKind::Branch && limb.parent.is_some_and(|p| self.holds(p))
            })
            .collect()
    }
}

impl Joinery {
    /// `limbs` of `plant`, and the crowns they hold, built in `world` as one
    /// body with the plant's frame at `at`.
    pub fn build(
        &self,
        world: &mut PhysicsWorld,
        plant: &Plant,
        limbs: &[usize],
        at: Isometry3<f32>,
    ) -> PlantPieceBody {
        let body = world.create_body(
            RigidBodyDesc::dynamic()
                .position(Point3::from(at.translation.vector))
                .rotation(at.rotation),
        );
        let mut colliders = vec![Vec::new(); plant.limbs.limbs.len()];
        for &l in limbs {
            let limb = &plant.limbs.limbs[l];
            let mut wood = limb.collider().of(&self.wood);
            wood.offset = Translation3::from(limb.midpoint().coords) * wood.offset;
            colliders[l].extend(world.attach_collider(body, wood));
        }
        for m in plant.masses_on(limbs) {
            let mass = &plant.masses[m];
            for collider in self.core.colliders(&mass.shape, mass.pose) {
                colliders[mass.limb].extend(world.attach_collider(body, collider));
            }
        }
        let moved = world.recenter_on_colliders(body);
        let piece = PlantPieceBody {
            body,
            frame: Isometry3::translation(-moved.x, -moved.y, -moved.z),
            colliders,
        };
        self.declare_bulk(world, plant, &piece);
        piece
    }

    /// Welds `piece`'s trunk into the ground where it stands, holding
    /// whatever it is given, and lets it pass through the ground its foot
    /// and roots are buried in until the weld is removed.
    pub fn root(
        &self,
        world: &mut PhysicsWorld,
        plant: &Plant,
        piece: &PlantPieceBody,
    ) -> ConstraintHandle {
        world.set_ignores_static(piece.body, true);
        let body = world
            .body(piece.body)
            .expect("a piece just built has a body");
        let body_pose = Isometry3::from_parts(body.position().coords.into(), body.rotation());
        let foot = body_pose * piece.frame * plant.limbs.limbs[0].base;
        world.create_constraint(ConstraintKind::Fixed {
            body_a: None,
            body_b: piece.body,
            local_anchor_a: foot.coords,
            local_anchor_b: (body_pose.inverse() * foot).coords,
            reference: body_pose.rotation,
            compliance: 0.0,
            max_impulse: f32::MAX,
        })
    }

    /// The impulse a blast gives the crowns `limb` and the limbs growing
    /// from it hold, in `piece`, as if they stood alone. `None` if it holds
    /// no crowns or the blast does not reach them.
    pub fn blast_on_branch(
        &self,
        world: &PhysicsWorld,
        plant: &Plant,
        piece: &PlantPieceBody,
        limb: usize,
        blast: &PhysicsImpulse,
    ) -> Option<Vector3<f32>> {
        let held: Vec<usize> = plant
            .limbs
            .subtree(limb)
            .into_iter()
            .filter(|&l| piece.holds(l))
            .collect();
        let masses = plant.masses_on(&held);
        if masses.is_empty() {
            return None;
        }
        // Struck at the crowns' middle, not at the plant's foot.
        let middle = masses
            .iter()
            .map(|&m| plant.masses[m].pose.translation.vector)
            .sum::<Vector3<f32>>()
            / masses.len() as f32;
        let to_middle = Isometry3::translation(middle.x, middle.y, middle.z);
        let bulk = self
            .core
            .bulk(&plant.leaves_of(&masses), to_middle.inverse());
        bulk.blast(&(piece.plant_pose(world)? * to_middle), blast)
    }

    /// Breaks `limb` and the limbs growing from it off `piece`, crowns and
    /// all, as a piece of their own moving as they were: what was left
    /// re-centres on what it still holds.
    pub fn break_off(
        &self,
        world: &mut PhysicsWorld,
        plant: &Plant,
        piece: &mut PlantPieceBody,
        limb: usize,
    ) -> Option<PlantPieceBody> {
        let at = piece.plant_pose(world)?;
        let (linear, angular, centre) = {
            let body = world.body(piece.body)?;
            (
                body.linear_velocity(),
                body.angular_velocity(),
                body.position(),
            )
        };
        let going: Vec<usize> = plant
            .limbs
            .subtree(limb)
            .into_iter()
            .filter(|&l| piece.holds(l))
            .collect();
        for &l in &going {
            for collider in std::mem::take(&mut piece.colliders[l]) {
                world.detach_collider(piece.body, collider);
            }
        }
        let moved = world.recenter_on_colliders(piece.body);
        piece.frame = Translation3::from(-moved) * piece.frame;
        self.declare_bulk(world, plant, piece);

        let broken = self.build(world, plant, &going, at);
        let position = world.body(broken.body)?.position();
        let velocity = linear + angular.cross(&(position - centre));
        world.set_body_velocity(broken.body, velocity, angular);
        world.wake_body(broken.body);
        world.wake_body(piece.body);
        Some(broken)
    }

    /// Declares what fluids and blasts see of `piece`: its crowns' leaves,
    /// porous, which a plant's thin wood adds little to. A piece with no
    /// crowns is left to its colliders.
    fn declare_bulk(&self, world: &mut PhysicsWorld, plant: &Plant, piece: &PlantPieceBody) {
        let masses = plant.masses_on(&piece.limbs());
        if !masses.is_empty() {
            let bulk = self.core.bulk(&plant.leaves_of(&masses), piece.frame);
            world.set_bulk(piece.body, bulk);
        }
    }
}
