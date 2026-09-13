//! Appendages that flop.
//!
//! A chain of verlet particles per appendage, pinned at the body and
//! pulled toward a rest shape that is carried along by it. Gravity and the
//! pull toward rest decide the droop; the lag between the pinned root
//! moving and the rest of the chain catching up is what does the flopping,
//! and it comes out of the integration for free.
//!
//! Left to gravity alone a chain hangs straight down and lies flat against
//! the body — the rest-shape spring is what keeps an ear an ear and a
//! feeler a feeler.
//!
//! A pair, not a single chain, because every use of it so far is
//! symmetric: a rig hangs two of these off a head and wants them splayed
//! to opposite sides from one set of numbers.

use nalgebra::{Point3, Vector3};

use crate::rendering::colour::Colour;
use crate::skeleton::verlet::{DistanceConstraint, Particle, VerletSystem};

use super::mesh::Frame;

/// The shape and feel of one floppy appendage.
///
/// The defaults describe a small animal ear; a longer, whippier feeler is
/// the same numbers stretched.
#[derive(Clone, Copy, Debug)]
pub struct DroopConfig {
    /// Bones in the chain. Two is a hinge, four is a noodle.
    pub segments: usize,
    /// Root-to-tip length when the ear is held straight.
    pub length: f32,
    /// Radius where the chain meets the body.
    pub root_radius: f32,
    /// Radius at the tip.
    pub tip_radius: f32,
    /// How far it leans out to the side at rest, as a fraction of its
    /// length. Zero stands it straight up.
    pub splay: f32,
    /// Total bend from root to tip at rest, in radians, away from the
    /// root direction and downward.
    ///
    /// This, not gravity, is what makes an ear floppy-looking while it is
    /// standing still. A straight rest shape reads as a horn however
    /// loosely it is simulated.
    pub curl: f32,
    /// How hard gravity pulls, relative to the world's. Below 1 makes a
    /// lighter, springier chain that lags the body more than it droops.
    pub weight: f32,
    /// Fraction of its velocity the chain sheds each step. Higher is more
    /// sluggish; at zero it swings forever.
    pub damping: f32,
    /// How hard the chain is pulled back toward its rest shape, in 1/s².
    ///
    /// This, not the bones, is what makes an ear an ear: a chain with only
    /// gravity on it hangs straight down and lies flat against the head.
    /// Raise it for a stiff ear that barely moves, lower it for one that
    /// swings about and takes its time coming back.
    pub shape_stiffness: f32,
    /// Backward lean of the rest pose, as a fraction of ear length.
    /// Positive sweeps the ears back off the face.
    pub sweep_back: f32,
    pub colour: Colour,
}

impl Default for DroopConfig {
    fn default() -> Self {
        Self {
            segments: 5,
            length: 0.23,
            root_radius: 0.032,
            tip_radius: 0.01,
            splay: 0.14,
            curl: 1.15,
            weight: 0.55,
            damping: 0.08,
            shape_stiffness: 200.0,
            sweep_back: 0.2,
            colour: Colour::new(0.98, 0.85, 0.30, 1.0),
        }
    }
}

/// Which of the pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DroopSide {
    Left,
    Right,
}

impl DroopSide {
    pub const BOTH: [DroopSide; 2] = [DroopSide::Left, DroopSide::Right];

    /// +1 for right, -1 for left.
    #[inline]
    fn sign(self) -> f32 {
        match self {
            DroopSide::Right => 1.0,
            DroopSide::Left => -1.0,
        }
    }

    #[inline]
    fn index(self) -> usize {
        match self {
            DroopSide::Left => 0,
            DroopSide::Right => 1,
        }
    }
}

/// A pair of floppy appendages.
pub struct DroopPair {
    config: DroopConfig,
    system: VerletSystem,
    /// Particle indices per ear, root first.
    chains: [Vec<usize>; 2],
}

impl DroopPair {
    /// Hang a pair off `anchors`, given in the body's local frame, with
    /// the body currently at `frame`.
    pub fn new(config: DroopConfig, anchors: [Vector3<f32>; 2], frame: &Frame) -> Self {
        let mut system = VerletSystem::new();
        system.gravity = Vector3::new(0.0, -9.81 * config.weight, 0.0);

        let segment = config.length / config.segments.max(1) as f32;
        let chains = DroopSide::BOTH.map(|side| {
            let root = frame.point(anchors[side.index()]);
            let offsets = rest_offsets(&config, side, frame);

            let mut chain = Vec::with_capacity(config.segments + 1);
            for (node, offset) in offsets.iter().enumerate() {
                let position = root + offset;
                let particle = if node == 0 {
                    Particle::pinned(position)
                } else {
                    let mut particle = Particle::new(position);
                    particle.damping = config.damping;
                    particle
                };
                chain.push(system.add_particle(particle));
            }
            for pair in chain.windows(2) {
                system.add_distance_constraint(DistanceConstraint::new(pair[0], pair[1], segment));
            }
            chain
        });

        Self {
            config,
            system,
            chains,
        }
    }

    /// Carry the pair along with the body and let them catch up.
    ///
    /// `anchors` are in the body's local frame; `body` is a sphere the
    /// chains are kept out of so they drape over it rather than through
    /// it.
    pub fn update(
        &mut self,
        dt: f32,
        anchors: [Vector3<f32>; 2],
        frame: &Frame,
        body: (Point3<f32>, f32),
    ) {
        if dt <= 0.0 {
            return;
        }
        for side in DroopSide::BOTH {
            let root = frame.point(anchors[side.index()]);
            let offsets = rest_offsets(&self.config, side, frame);
            let chain = &self.chains[side.index()];

            self.system.set_particle_position(chain[0], root);
            for (node, index) in chain.iter().enumerate().skip(1) {
                let rest = root + offsets[node];
                let Some(position) = self.system.get_particle_position(*index) else {
                    continue;
                };
                self.system
                    .apply_force(*index, (rest - position) * self.config.shape_stiffness);
            }
        }

        self.system.update(dt);
        self.push_out_of(body);
    }

    /// Every joint of one chain, root first.
    pub fn joints(&self, side: DroopSide) -> Vec<Point3<f32>> {
        self.chains[side.index()]
            .iter()
            .filter_map(|index| self.system.get_particle_position(*index))
            .collect()
    }

    /// Radius at a given joint, tapering root to tip.
    pub fn radius_at(&self, node: usize) -> f32 {
        let t = node as f32 / self.config.segments.max(1) as f32;
        self.config.root_radius + (self.config.tip_radius - self.config.root_radius) * t
    }

    /// Keep the chains outside the body they are attached to.
    fn push_out_of(&mut self, (centre, radius): (Point3<f32>, f32)) {
        for chain in &self.chains {
            for index in chain.iter().skip(1) {
                let Some(position) = self.system.get_particle_position(*index) else {
                    continue;
                };
                let offset: Vector3<f32> = position - centre;
                let distance = offset.magnitude();
                if distance >= radius || distance < 1e-5 {
                    continue;
                }
                self.system
                    .set_particle_position(*index, centre + offset * (radius / distance));
            }
        }
    }
}

/// The shape a chain holds when nothing has disturbed it: up, splayed out
/// to its own side, swept a little back, and curling over toward the tip.
///
/// Returned as the position of every joint relative to the root, so the
/// curl accumulates along the chain the way a real ear's does.
///
/// Built in the body's local frame and mapped out at the end. Bending is
/// a blend between the root direction and the outward one rather than a
/// rotation about an axis: the frame's basis is left-handed, and a
/// rotation authored in it comes out mirrored on one side.
fn rest_offsets(config: &DroopConfig, side: DroopSide, frame: &Frame) -> Vec<Vector3<f32>> {
    let segments = config.segments.max(1);
    let segment = config.length / segments as f32;

    let root = Vector3::new(side.sign() * config.splay, 1.0, -config.sweep_back).normalize();
    // The direction the ear curls toward: straight out to its own side,
    // squared up against the root direction so the blend between them
    // sweeps a quarter turn and no more.
    let outward = Vector3::new(side.sign(), 0.0, 0.0);
    let outward = (outward - root * outward.dot(&root))
        .try_normalize(1e-4)
        .unwrap_or(outward);

    let mut offsets = Vec::with_capacity(segments + 1);
    let mut local = Vector3::zeros();
    offsets.push(Vector3::zeros());
    for node in 0..segments {
        // Bend a little further with every bone, so the chain is straight
        // where it leaves the body and floppiest at the tip.
        let angle = config.curl * (node as f32 + 0.5) / segments as f32;
        local += (root * angle.cos() + outward * angle.sin()) * segment;
        offsets.push(frame.direction(local));
    }
    offsets
}
