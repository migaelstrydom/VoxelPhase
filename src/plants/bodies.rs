//! A plant built in the physics world: a body per limb, carrying the crown
//! it holds up, the limbs welded together and the trunk welded into the
//! ground.
//!
//! ```text
//!   ground ══weld══ trunk + main crown ──weld (breakable)── branch + its crown
//! ```
//!
//! A crown's core is part of its limb's body rather than a body welded to
//! it: a thin limb has almost no inertia about its own length, and a crown
//! many times heavier welded to its tip set the pair ringing until the welds
//! broke. Between a branch and what it grows from, both carrying crowns, a
//! weld holds steady. It holds a fixed impulse per substep: its crown's
//! weight is a small share of that and a blast beside it many times more,
//! so the branch comes off. The trunk's weld into the ground holds whatever
//! it is given, until the ground under it is blown away (`TerrainAnchored`,
//! which removes it).

use nalgebra::{Isometry3, Point3, Translation3};

use super::limbs::Limb;
use super::plant::Plant;
use crate::collision_layers;
use crate::foliage::LeafCore;
use crate::physics::{
    ColliderHandle, ConstraintHandle, ConstraintKind, PhysicsWorld, RigidBodyDesc, RigidBodyHandle,
};
use crate::rendering::substance::{self, ColliderSubstance, Substance};

/// How a plant's bodies are made and held together.
#[derive(Debug, Clone, Copy)]
pub struct Joinery {
    /// What the limbs are made of.
    pub wood: Substance,
    /// The impulse a branch's weld holds per substep, per axis, N·s or
    /// N·m·s: past it the branch breaks off. A grenade within about 2 m of
    /// a branch's crown takes it off, one 3 m away does not; the crown's own
    /// weight is a few per cent of it.
    pub branch_strength: f32,
    /// How each crown's core is built.
    pub core: LeafCore,
}

impl Default for Joinery {
    fn default() -> Self {
        Self {
            wood: substance::OAK,
            branch_strength: 80.0,
            core: LeafCore::default(),
        }
    }
}

/// The bodies and welds of a plant built in the world.
#[derive(Debug, Clone)]
pub struct PlantBodies {
    /// A body per limb, in the plant's limb order: the trunk first.
    pub limbs: Vec<RigidBodyHandle>,
    /// Where each limb's frame (the plant's, moved to the limb) sits in its
    /// body's: the body's origin is its centre of mass, limb and crown
    /// together.
    pub limb_frames: Vec<Isometry3<f32>>,
    /// Each limb's weld to the limb it grows from, `None` for the trunk.
    pub branch_welds: Vec<Option<ConstraintHandle>>,
    /// Where each leaf mass's frame sits in its limb's body's.
    pub mass_offsets: Vec<Isometry3<f32>>,
    /// Each leaf mass's core colliders on its limb's body: what comes away
    /// when the crown breaks off.
    pub mass_colliders: Vec<Vec<ColliderHandle>>,
    /// The trunk's weld into the ground.
    pub rooting: ConstraintHandle,
}

impl Joinery {
    /// `plant` built in `world` with its frame at `root`.
    pub fn build(
        &self,
        world: &mut PhysicsWorld,
        plant: &Plant,
        root: Isometry3<f32>,
    ) -> PlantBodies {
        let mut limbs = Vec::new();
        let mut limb_frames = Vec::new();
        let mut mass_offsets = vec![Isometry3::identity(); plant.masses.len()];
        let mut mass_colliders = vec![Vec::new(); plant.masses.len()];
        for (index, limb) in plant.limbs.limbs.iter().enumerate() {
            let crowns: Vec<usize> = (0..plant.masses.len())
                .filter(|&m| plant.masses[m].limb == index)
                .collect();
            // The body's origin, in the plant's frame: limb and crowns' centre
            // of mass. A crown's frame is at its own centre of mass.
            let mut weighted = limb.midpoint().coords * self.limb_mass(limb);
            let mut total = self.limb_mass(limb);
            for &m in &crowns {
                let mass = self.core.mass(&plant.masses[m].shape);
                weighted += plant.masses[m].pose.translation.vector * mass;
                total += mass;
            }
            let origin =
                Isometry3::from_parts(Translation3::from(weighted / total), Default::default());
            let frame = origin.inverse();

            let mut desc = RigidBodyDesc::dynamic()
                .position(Point3::from((root * origin).translation.vector))
                .rotation(root.rotation)
                .ignores_static(limb.parent.is_none())
                .filter(collision_layers::PLANT);
            if let Some(&first) = crowns.first() {
                let at = frame * plant.masses[first].pose;
                desc = desc.bulk(self.core.bulk(&plant.masses[first].shape, at));
            }
            let body = world.create_body(desc);
            let mut wood = limb.collider().of(&self.wood);
            wood.offset = frame * Translation3::from(limb.midpoint().coords) * wood.offset;
            world.attach_collider(body, wood);
            for &m in &crowns {
                let at = frame * plant.masses[m].pose;
                for collider in self.core.colliders(&plant.masses[m].shape, at) {
                    mass_colliders[m].extend(world.attach_collider(body, collider));
                }
                mass_offsets[m] = at;
            }
            limbs.push(body);
            limb_frames.push(frame);
        }

        let branch_welds = plant
            .limbs
            .limbs
            .iter()
            .zip(&limbs)
            .map(|(limb, &body)| {
                let parent = limbs[limb.parent?];
                Some(weld(
                    world,
                    Some(parent),
                    body,
                    root * limb.base,
                    self.branch_strength,
                ))
            })
            .collect();
        let rooting = weld(
            world,
            None,
            limbs[0],
            root * plant.limbs.limbs[0].base,
            f32::MAX,
        );
        PlantBodies {
            limbs,
            limb_frames,
            branch_welds,
            mass_offsets,
            mass_colliders,
            rooting,
        }
    }

    /// A limb's mass: its capsule of wood.
    fn limb_mass(&self, limb: &Limb) -> f32 {
        let r = limb.radius;
        let length = (limb.tip - limb.base).magnitude();
        let volume = std::f32::consts::PI * r * r * (length + 4.0 / 3.0 * r);
        volume * self.wood.physics.density
    }
}

/// Welds `b` to `a` (or to the world) where they are now, at the world point
/// `at`, holding `max_impulse` per substep per axis before it breaks.
fn weld(
    world: &mut PhysicsWorld,
    a: Option<RigidBodyHandle>,
    b: RigidBodyHandle,
    at: Point3<f32>,
    max_impulse: f32,
) -> ConstraintHandle {
    let pose = |h: RigidBodyHandle| {
        let body = world.body(h).expect("just made");
        Isometry3::from_parts(body.position().coords.into(), body.rotation())
    };
    let pose_b = pose(b);
    let pose_a = a.map(pose).unwrap_or_else(Isometry3::identity);
    world.create_constraint(ConstraintKind::Fixed {
        body_a: a,
        body_b: b,
        local_anchor_a: (pose_a.inverse() * at).coords,
        local_anchor_b: (pose_b.inverse() * at).coords,
        reference: pose_a.rotation.inverse() * pose_b.rotation,
        compliance: 0.0,
        max_impulse,
    })
}
