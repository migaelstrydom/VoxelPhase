//! Ears that flop.
//!
//! A chain of verlet particles per ear, pinned at the head and pulled
//! toward a rest shape that is carried along by the body. Gravity and the
//! pull toward rest decide the droop; the lag between the pinned root
//! moving and the rest of the chain catching up is what does the flopping,
//! and it comes out of the integration for free.
//!
//! Left to gravity alone a chain hangs straight down and the ears lie flat
//! against the head — the rest-shape spring is what keeps them ears.

use nalgebra::{Point3, Vector3};

use crate::animation::rig::Frame;
use crate::skeleton::verlet::{DistanceConstraint, Particle, VerletSystem};

use super::config::EarConfig;

/// Which ear.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EarSide {
    Left,
    Right,
}

impl EarSide {
    pub const BOTH: [EarSide; 2] = [EarSide::Left, EarSide::Right];

    /// +1 for right, -1 for left.
    #[inline]
    fn sign(self) -> f32 {
        match self {
            EarSide::Right => 1.0,
            EarSide::Left => -1.0,
        }
    }

    #[inline]
    fn index(self) -> usize {
        match self {
            EarSide::Left => 0,
            EarSide::Right => 1,
        }
    }
}

/// A pair of floppy ears.
pub struct Ears {
    config: EarConfig,
    system: VerletSystem,
    /// Particle indices per ear, root first.
    chains: [Vec<usize>; 2],
}

impl Ears {
    /// Hang a pair of ears off `anchors`, given in the head's local frame,
    /// with the body currently at `frame`.
    pub fn new(config: EarConfig, anchors: [Vector3<f32>; 2], frame: &Frame) -> Self {
        let mut system = VerletSystem::new();
        system.gravity = Vector3::new(0.0, -9.81 * config.weight, 0.0);

        let segment = config.length / config.segments.max(1) as f32;
        let chains = EarSide::BOTH.map(|side| {
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

    /// Carry the ears along with the head and let them catch up.
    ///
    /// `anchors` are in the head's local frame; `body` is a sphere the
    /// ears are kept out of so they drape over the head rather than
    /// through it.
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
        for side in EarSide::BOTH {
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

    /// Every joint of one ear, root first.
    pub fn joints(&self, side: EarSide) -> Vec<Point3<f32>> {
        self.chains[side.index()]
            .iter()
            .filter_map(|index| self.system.get_particle_position(*index))
            .collect()
    }

    /// Radius of the ear at a given joint, tapering root to tip.
    pub fn radius_at(&self, node: usize) -> f32 {
        let t = node as f32 / self.config.segments.max(1) as f32;
        self.config.root_radius + (self.config.tip_radius - self.config.root_radius) * t
    }

    /// Keep the ears outside the body they are attached to.
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

/// The shape an ear holds when nothing has disturbed it: up, splayed out
/// to its own side, swept a little back, and curling over toward the tip.
///
/// Returned as the position of every joint relative to the root, so the
/// curl accumulates along the chain the way a real ear's does.
///
/// Built in the head's local frame and mapped out at the end. Bending is
/// a blend between the root direction and the outward one rather than a
/// rotation about an axis: the frame's basis is left-handed, and a
/// rotation authored in it comes out mirrored on one side.
fn rest_offsets(config: &EarConfig, side: EarSide, frame: &Frame) -> Vec<Vector3<f32>> {
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
        // Bend a little further with every bone, so the ear is straight
        // where it leaves the head and floppiest at the tip.
        let angle = config.curl * (node as f32 + 0.5) / segments as f32;
        local += (root * angle.cos() + outward * angle.sin()) * segment;
        offsets.push(frame.direction(local));
    }
    offsets
}
