//! The rig itself: what is posed and shaped, and the clock it is posed by.

use std::f32::consts::TAU;
use std::sync::Arc;

use nalgebra::{UnitQuaternion, Vector3};
use specs::{Component, DenseVecStorage};

use super::jaw::{GulpingJaw, Mouth};
use crate::creature::WingBeat;
use crate::model::{Hinge, MeshPrimitive, Ripple, Transform};

/// The quickest an eye's turn to a new aim is over, s: a fish's eye does not
/// drift, it jumps.
const SACCADE: f32 = 0.06;
/// Shortest and longest an eye holds a glance, s.
const GLANCE: (f32, f32) = (0.5, 2.6);

/// How the wave runs down a fish's fins, and how its pace follows its speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FinWave {
    /// From one crest to the next along a fin, in its root's measure.
    pub wavelength: f32,
    /// Waves a second hanging still, and at cruising speed.
    pub idle_rate: f32,
    pub cruise_rate: f32,
    /// Share of each fin's swing it keeps hanging still.
    pub idle_swing: f32,
    /// How much further than at its root a fin turns at its edge, as a
    /// share: a fin that bends as it swings rather than swinging stiff.
    pub curl: f32,
}

/// A fin the wave runs down.
#[derive(Clone)]
pub struct RippledFin {
    /// The fin, hinged along its root.
    pub ripple: Arc<Ripple>,
    /// How far it swings either way at its root at cruising speed, radians.
    pub swing: f32,
}

/// An eye that turns in its socket on its own, as a flatfish's do: it
/// glances about, and now and then at whatever it is watching.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RovingEye {
    /// The model's part it is.
    pub part: usize,
    /// Where it turns, in the entity's frame.
    pub pivot: Vector3<f32>,
    /// The way it looks at rest, in the entity's frame.
    pub rest: Vector3<f32>,
    /// The farthest it turns from rest, radians.
    pub reach: f32,
    /// The share of its glances it gives what it is watching, 0 to 1.
    pub interest: f32,
    /// Seed of its glances.
    pub seed: u32,
}

/// Where one eye is looking.
#[derive(Debug, Clone, Copy)]
struct Gaze {
    /// Which way it looks now, in the entity's frame.
    now: Vector3<f32>,
    /// Which way it is turning to.
    aim: Vector3<f32>,
    /// Whether this glance is at what it watches, followed as that moves.
    watching: bool,
    /// When it next glances, on the rig's clock.
    until: f32,
    /// How many glances it has made.
    glances: u32,
}

/// A fish's rig: fins down which a wave runs, wings that beat, eyes that
/// rove, each quicker the faster it goes; and a jaw that gulps.
#[derive(Component, Clone)]
#[storage(DenseVecStorage)]
pub struct FishRig {
    /// The fins, each drawn into one of the entity's procedural meshes, in
    /// order.
    pub fins: Vec<RippledFin>,
    /// How the wave runs down them.
    pub wave: FinWave,
    /// Its wings, if it has any.
    pub wings: Option<WingBeat>,
    /// Its eyes.
    pub eyes: Vec<RovingEye>,
    /// The speed at which it works its hardest, m/s.
    pub cruise_speed: f32,
    /// How far off something is still worth an eye's glance, m.
    pub sight: f32,
    /// Where through a wave its fins are, 0 to 1.
    fin_phase: f32,
    /// Where through a beat its wings are, 0 to 1.
    wing_phase: f32,
    /// How hard it is working, 0 to 1, as last carried on.
    effort: f32,
    /// Seconds it has been running.
    clock: f32,
    /// Where each eye is looking.
    gazes: Vec<Gaze>,
    /// Its mouth, if it has a jaw that moves.
    mouth: Option<Mouth>,
}

impl FishRig {
    /// A rig at rest, every eye looking where it does at rest.
    pub fn new(
        fins: Vec<RippledFin>,
        wave: FinWave,
        wings: Option<WingBeat>,
        eyes: Vec<RovingEye>,
        cruise_speed: f32,
        sight: f32,
    ) -> Self {
        let gazes = eyes
            .iter()
            .map(|eye| Gaze {
                now: eye.rest.normalize(),
                aim: eye.rest.normalize(),
                watching: false,
                until: 0.0,
                glances: 0,
            })
            .collect();
        Self {
            fins,
            wave,
            wings,
            eyes,
            cruise_speed,
            sight,
            fin_phase: 0.0,
            wing_phase: 0.0,
            effort: 0.0,
            clock: 0.0,
            gazes,
            mouth: None,
        }
    }

    /// The rig with a jaw that breathes and gulps.
    pub fn with_jaw(mut self, jaw: GulpingJaw) -> Self {
        self.mouth = Some(Mouth::new(jaw));
        self
    }

    /// Gulp now, if it has a jaw and is not gulping already.
    pub fn gulp(&mut self) {
        if let Some(mouth) = &mut self.mouth {
            mouth.gulp(self.clock);
        }
    }

    /// Carry the rig `dt` seconds on, moving at `speed` m/s and with
    /// `watched` (a point in the entity's frame) to look at, if anything.
    pub fn advance(&mut self, dt: f32, speed: f32, watched: Option<Vector3<f32>>) {
        self.effort = (speed / self.cruise_speed.max(1e-3)).clamp(0.0, 1.0);
        let rate =
            self.wave.idle_rate + (self.wave.cruise_rate - self.wave.idle_rate) * self.effort;
        self.fin_phase = (self.fin_phase + rate * dt).rem_euclid(1.0);
        if let Some(wings) = &self.wings {
            self.wing_phase = (self.wing_phase + wings.rate(self.effort) * dt).rem_euclid(1.0);
        }
        self.clock += dt;
        if let Some(mouth) = &mut self.mouth {
            mouth.advance(self.clock);
        }

        let sight = self.sight;
        let watched = watched.filter(|at| at.norm() < sight);
        let settle = 1.0 - (-dt / SACCADE).exp();
        for (eye, gaze) in self.eyes.iter().zip(&mut self.gazes) {
            if self.clock >= gaze.until {
                gaze.glances += 1;
                let roll = |salt: u32| hashed(eye.seed.wrapping_add(gaze.glances * 7919), salt);
                gaze.watching = watched.is_some() && roll(1) < eye.interest;
                if !gaze.watching {
                    // Anywhere within its reach, the farther out the rarer.
                    let rest = eye.rest.normalize();
                    let side = rest
                        .cross(&Vector3::y())
                        .try_normalize(1e-6)
                        .unwrap_or_else(Vector3::x);
                    let up = side.cross(&rest);
                    let (turn, off) = (roll(2) * TAU, eye.reach * roll(3).sqrt());
                    let way = side * turn.cos() + up * turn.sin();
                    gaze.aim = UnitQuaternion::from_scaled_axis(rest.cross(&way) * off) * rest;
                }
                gaze.until = self.clock + GLANCE.0 + (GLANCE.1 - GLANCE.0) * roll(4);
            }
            if let (true, Some(at)) = (gaze.watching, watched) {
                gaze.aim = (at - eye.pivot).try_normalize(1e-6).unwrap_or(gaze.aim);
            }
            let aim = within_reach(eye, gaze.aim);
            gaze.now = gaze
                .now
                .lerp(&aim, settle)
                .try_normalize(1e-6)
                .unwrap_or(aim);
        }
    }

    /// Write the pose of every part the rig moves (the wings, the eyes, the
    /// jaw) into `poses`, a model's poses in part order. The rest it leaves.
    pub fn pose(&self, poses: &mut [Transform]) {
        if let Some(wings) = &self.wings {
            for wing in &wings.wings {
                if let Some(posed) = poses.get_mut(wing.part) {
                    *posed = wings.pose(wing, self.wing_phase, 0.0);
                }
            }
        }
        for (eye, gaze) in self.eyes.iter().zip(&self.gazes) {
            let Some(posed) = poses.get_mut(eye.part) else {
                continue;
            };
            let turn = UnitQuaternion::rotation_between(&eye.rest.normalize(), &gaze.now)
                .unwrap_or_else(UnitQuaternion::identity);
            *posed = Transform {
                translation: eye.pivot - turn * eye.pivot,
                rotation: turn,
                scale: Vector3::repeat(1.0),
            };
        }
        if let Some(mouth) = &self.mouth {
            if let Some(posed) = poses.get_mut(mouth.jaw.part) {
                *posed = mouth.pose(self.clock);
            }
        }
    }

    /// Shape the fins into `meshes`, one to a fin, in order.
    pub fn shape(&self, meshes: &mut Vec<MeshPrimitive>) {
        if meshes.len() != self.fins.len() {
            *meshes = self
                .fins
                .iter()
                .map(|fin| fin.ripple.rest().clone())
                .collect();
        }
        let share = self.wave.idle_swing + (1.0 - self.wave.idle_swing) * self.effort;
        for (fin, mesh) in self.fins.iter().zip(meshes.iter_mut()) {
            let swing = fin.swing * share;
            fin.ripple.shape(|hinge| self.swing_at(hinge, swing), mesh);
        }
    }

    /// How far a vertex on `hinge` is turned, for a fin swinging `swing`
    /// radians at its root.
    fn swing_at(&self, hinge: &Hinge, swing: f32) -> f32 {
        let crest = (self.fin_phase - hinge.along / self.wave.wavelength) * TAU;
        swing * crest.sin() * (1.0 + self.wave.curl * hinge.out)
    }

    /// Which way each eye is looking now, in the entity's frame, in order.
    pub fn gazes(&self) -> impl Iterator<Item = Vector3<f32>> + '_ {
        self.gazes.iter().map(|gaze| gaze.now)
    }
}

/// `aim`, turned back toward `eye`'s rest as far as it must be to lie
/// within its reach.
fn within_reach(eye: &RovingEye, aim: Vector3<f32>) -> Vector3<f32> {
    let rest = eye.rest.normalize();
    let aim = aim.try_normalize(1e-6).unwrap_or(rest);
    let off = rest.dot(&aim).clamp(-1.0, 1.0).acos();
    if off <= eye.reach {
        return aim;
    }
    let axis = rest
        .cross(&aim)
        .try_normalize(1e-6)
        .unwrap_or_else(Vector3::y);
    UnitQuaternion::from_scaled_axis(axis * eye.reach) * rest
}

/// A number in `0..1` that is always the same for `key` and `salt`, and
/// looks unrelated to its neighbours'.
pub(super) fn hashed(key: u32, salt: u32) -> f32 {
    let mut n = key.wrapping_mul(0x27d4_eb2d) ^ salt.wrapping_mul(0x1656_67b1);
    n ^= n >> 15;
    n = n.wrapping_mul(0x85eb_ca6b);
    n ^= n >> 13;
    n = n.wrapping_mul(0xc2b2_ae35);
    n ^= n >> 16;
    n as f32 / u32::MAX as f32
}

#[cfg(test)]
mod tests {
    use nalgebra::{Vector2, Vector4};

    use super::*;
    use crate::rendering::material::MaterialId;
    use crate::rendering::vertex::Vertex;

    /// One strip of fin, rooted along Z from 0 to 4 m and reaching 0.5 m
    /// out along X, a vertex at each metre along its edge.
    fn fin() -> RippledFin {
        let vertex = |x: f32, z: f32| Vertex {
            pos: Vector3::new(x, 0.0, z),
            color: Vector4::repeat(1.0),
            tex_coords: Vector2::zeros(),
            normal: Vector3::y(),
            ao: 1.0,
        };
        let mut vertices = Vec::new();
        for along in 0..=4 {
            vertices.push(vertex(0.0, along as f32));
            vertices.push(vertex(0.5, along as f32));
        }
        let rest = MeshPrimitive {
            vertices,
            indices: Vec::new(),
            material: MaterialId(0),
        };
        let root = [(Vector3::zeros(), 0.0), (Vector3::new(0.0, 0.0, 4.0), 4.0)];
        RippledFin {
            ripple: Arc::new(Ripple::rooted(rest, &root)),
            swing: 0.4,
        }
    }

    const WAVE: FinWave = FinWave {
        wavelength: 2.0,
        idle_rate: 0.5,
        cruise_rate: 1.5,
        idle_swing: 0.3,
        curl: 0.0,
    };

    fn eye() -> RovingEye {
        RovingEye {
            part: 1,
            pivot: Vector3::new(0.0, 0.2, 1.0),
            rest: Vector3::y(),
            reach: 0.6,
            interest: 1.0,
            seed: 5,
        }
    }

    fn edge_heights(rig: &FishRig) -> Vec<f32> {
        let mut meshes = Vec::new();
        rig.shape(&mut meshes);
        meshes[0]
            .vertices
            .iter()
            .skip(1)
            .step_by(2)
            .map(|v| v.pos.y)
            .collect()
    }

    #[test]
    fn a_wave_runs_down_the_fin_from_its_head_and_harder_at_speed() {
        let mut rig = FishRig::new(vec![fin()], WAVE, None, Vec::new(), 3.0, 20.0);
        rig.advance(1.0 / 6.0, 3.0, None);
        // A quarter of the way through a wave the crest is at the head, a
        // wavelength on is another, and half a wavelength on a trough.
        let heights = edge_heights(&rig);
        let crest = 0.5 * 0.4f32.sin();
        assert!((heights[0] - crest).abs() < 1e-3, "{heights:?}");
        assert!((heights[2] - crest).abs() < 1e-3, "{heights:?}");
        assert!((heights[1] + crest).abs() < 1e-3, "{heights:?}");
        // A moment later the crest has moved down the fin, toward the tail.
        rig.advance(0.05, 3.0, None);
        let later = edge_heights(&rig);
        assert!(later[0] < heights[0] && later[1] > heights[1], "{later:?}");

        // Hanging still it swings less, and the wave runs slower.
        let mut idle = FishRig::new(vec![fin()], WAVE, None, Vec::new(), 3.0, 20.0);
        idle.advance(0.5, 0.0, None);
        let heights = edge_heights(&idle);
        let crest = 0.5 * (0.4f32 * 0.3).sin();
        assert!((heights[0] - crest).abs() < 1e-3, "{heights:?}");
    }

    #[test]
    fn an_eye_turns_to_what_it_watches_but_no_further_than_it_reaches() {
        let mut rig = FishRig::new(Vec::new(), WAVE, None, vec![eye()], 3.0, 20.0);
        let mut poses = vec![Transform::default(); 2];
        // Watched from up and ahead, within its reach: it looks there.
        let watched = Vector3::new(0.0, 3.2, 2.0);
        for _ in 0..60 {
            rig.advance(1.0 / 60.0, 0.0, Some(watched));
        }
        let wanted = (watched - eye().pivot).normalize();
        let looking = rig.gazes().next().unwrap();
        assert!((looking - wanted).norm() < 1e-2, "{looking:?}");
        rig.pose(&mut poses);
        assert!((poses[1].rotation * Vector3::y() - looking).norm() < 1e-4);
        // Its pivot stays where it is.
        let pivot = poses[1].rotation * eye().pivot + poses[1].translation;
        assert!((pivot - eye().pivot).norm() < 1e-5);

        // Watched from straight behind and below, it turns as far as it can.
        let behind = Vector3::new(0.0, -5.0, -6.0);
        for _ in 0..60 {
            rig.advance(1.0 / 60.0, 0.0, Some(behind));
        }
        let looking = rig.gazes().next().unwrap();
        let off = looking.dot(&Vector3::y()).acos();
        assert!((off - 0.6).abs() < 1e-2, "{off}");
    }

    #[test]
    fn an_eye_with_nothing_to_watch_roves_within_its_reach() {
        let mut rig = FishRig::new(Vec::new(), WAVE, None, vec![eye()], 3.0, 20.0);
        let mut seen = Vec::new();
        for _ in 0..600 {
            rig.advance(1.0 / 60.0, 0.0, Some(Vector3::new(0.0, 0.0, 50.0)));
            seen.push(rig.gazes().next().unwrap());
        }
        let off = |gaze: &Vector3<f32>| gaze.dot(&Vector3::y()).clamp(-1.0, 1.0).acos();
        assert!(seen.iter().all(|gaze| off(gaze) <= 0.6 + 1e-3));
        let farthest = seen.iter().map(off).fold(0.0, f32::max);
        assert!(farthest > 0.2, "it never looked about: {farthest}");
    }
}
