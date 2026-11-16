//! A lobber's body: the parts of its model that show a throw coming, and
//! how each is posed.
//!
//! ```text
//!   bag ── Swell: scaled about its seat ─┬─▶ head  ── Rider ──▶ cheeks
//!                                        │                     (a Bulge),
//!                                        │                     eyes (Gaze)
//!                                        └─▶ riders ── Rider (pipes, ...)
//!   frame: what stands under the bag (legs), moved by nothing but the heave
//!   struts: held on the frame, turned to reach the bag or a rider (an arm)
//!   then the whole: squatting, trembling, heaving (Throw::heave)
//! ```
//!
//! What is fastened into the bag rides it: it is carried with the point it
//! is fastened at as the bag swells, and turned as the skin there turns, so
//! a pipe stays in its stock and the head in its collar. What rests on the
//! frame at one end and is held by the bag at the other (an arm, its elbow
//! on a thigh and its hand on a pipe) turns about where it rests.

use std::f32::consts::TAU;
use std::fmt;
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector3};

use crate::model::{Bulge, MeshPrimitive, Transform};

/// Breaths a second, and how much of the way from slack to tight a breath
/// moves the bag.
const BREATH_RATE: f32 = 0.3;
const BREATH_DEPTH: f32 = 0.07;

/// How far past slack a dead lobber's bag sinks, as a share of the way from
/// slack to tight.
const DEAD_SAG: f32 = 0.6;

/// Seconds before a throw it squats into it, how far it sinks and spreads
/// at the bottom of the squat (shares of its height and width), and how
/// fast and how hard it trembles there.
const WINDUP_SECONDS: f32 = 0.55;
pub(super) const WINDUP_SQUAT: f32 = 0.09;
const WINDUP_SPREAD: f32 = 0.05;
const TREMBLE_RATE: f32 = 11.0;
const TREMBLE: f32 = 0.015;

/// How far it stretches up as a bomb leaves (a share of its height), how
/// fast that dies away (seconds to fall to a third), and how many times a
/// second it bounces as it settles.
pub(super) const HEAVE: f32 = 0.16;
const HEAVE_DECAY: f32 = 0.2;
const HEAVE_RATE: f32 = 2.4;

/// A part of a lobber's model that swells and sags.
#[derive(Debug, Clone, Copy)]
pub struct Swell {
    /// Which part of the model it is.
    pub part: usize,
    /// The point of it that stays where it is as it swells, in the
    /// entity's frame: where a bag sits on the ground.
    pub seat: Vector3<f32>,
    /// Its size when it is empty and when it is full, as scales of the
    /// size it was sculpted.
    pub slack: Vector3<f32>,
    pub tight: Vector3<f32>,
}

impl Swell {
    /// Its scale `full` of the way from slack to tight. Below zero is
    /// emptier than slack.
    pub fn scale(&self, full: f32) -> Vector3<f32> {
        self.slack + (self.tight - self.slack) * full
    }

    /// Its pose `full` of the way from slack to tight.
    pub fn pose(&self, full: f32) -> Transform {
        let scale = self.scale(full);
        Transform {
            translation: self.seat - self.seat.component_mul(&scale),
            rotation: UnitQuaternion::identity(),
            scale,
        }
    }
}

/// A rigid part of a lobber's model fastened into its bag: a pipe in its
/// stock, a head sewn into the bag's mouth.
#[derive(Debug, Clone, Copy)]
pub struct Rider {
    /// Which part of the model it is.
    pub part: usize,
    /// Where it is fastened into the bag, in the entity's frame.
    pub anchor: Vector3<f32>,
    /// The way out of the bag there.
    pub out: Vector3<f32>,
}

impl Rider {
    /// Where it is carried when `bag` is scaled by `scale` about its seat:
    /// with the point it is fastened at, and turned as the skin there
    /// turns.
    pub fn carried(&self, bag: &Swell, scale: Vector3<f32>) -> Transform {
        let anchor = bag.seat + (self.anchor - bag.seat).component_mul(&scale);
        // A surface's normal goes by the inverse of a scale.
        let out = self.out.component_div(&scale);
        let turn = UnitQuaternion::rotation_between(&self.out, &out)
            .unwrap_or_else(UnitQuaternion::identity);
        Transform {
            translation: anchor - turn * self.anchor,
            rotation: turn,
            scale: Vector3::repeat(1.0),
        }
    }
}

/// A rigid part of a lobber's model that rests on its frame at one end and
/// is held at the other by the bag, or by something riding it: a forearm,
/// its elbow on a thigh, its wrist in a hand on a pipe.
#[derive(Debug, Clone, Copy)]
pub struct Strut {
    /// Which part of the model it is.
    pub part: usize,
    /// Where it rests on the frame, which it turns about, in the entity's
    /// frame.
    pub pivot: Vector3<f32>,
    /// The point of it that is held, in the entity's frame as sculpted.
    pub reach: Vector3<f32>,
    /// The part of the rider that holds it, or nothing if the bag itself
    /// does.
    pub held_by: Option<usize>,
}

impl Strut {
    /// Its pose turned about its pivot toward where what holds it has
    /// carried its reach to: `held`, which carries a point of the entity's
    /// frame as sculpted to where it is now.
    pub fn turned(&self, held: impl Fn(Vector3<f32>) -> Vector3<f32>) -> Transform {
        let turn = UnitQuaternion::rotation_between(
            &(self.reach - self.pivot),
            &(held(self.reach) - self.pivot),
        )
        .unwrap_or_else(UnitQuaternion::identity);
        Transform {
            translation: self.pivot - turn * self.pivot,
            rotation: turn,
            scale: Vector3::repeat(1.0),
        }
    }
}

/// An eye in a lobber's model that turns to what it watches.
#[derive(Debug, Clone, Copy)]
pub struct Gaze {
    /// Which part of the model it is: an eye about its own middle, looking
    /// along +Z.
    pub part: usize,
    /// Its middle, and the way it looks at rest, in the entity's frame.
    pub socket: Vector3<f32>,
    pub ahead: Vector3<f32>,
    /// How far off that it can turn, radians.
    pub swivel: f32,
}

impl Gaze {
    /// Its pose looking toward `target` (in the entity's frame), as far as
    /// its swivel lets it; at rest with nothing to look at.
    pub fn pose(&self, target: Option<Vector3<f32>>) -> Transform {
        let ahead = self.ahead.normalize();
        let wanted = target
            .and_then(|target| (target - self.socket).try_normalize(1e-4))
            .unwrap_or(ahead);
        let off = ahead.dot(&wanted).clamp(-1.0, 1.0).acos();
        let look = if off <= self.swivel {
            wanted
        } else {
            // Round toward it as far as the socket allows.
            let round = (wanted - ahead * ahead.dot(&wanted))
                .try_normalize(1e-5)
                .unwrap_or_else(Vector3::y);
            ahead * self.swivel.cos() + round * self.swivel.sin()
        };
        Transform {
            translation: self.socket,
            rotation: UnitQuaternion::face_towards(&look, &Vector3::y()),
            scale: Vector3::repeat(1.0),
        }
    }
}

/// Cheeks that blow the bag full: flesh blown out of the face, not a part
/// scaled, so that they stay part of the face at every size.
#[derive(Clone)]
pub struct Cheeks {
    /// The cheeks as sculpted, in the entity's frame, and how they lie
    /// empty.
    pub bulge: Arc<Bulge>,
    /// How far they are blown between puffs and at the top of one, as
    /// shares of the way from empty to as sculpted.
    pub slack: f32,
    pub tight: f32,
}

impl fmt::Debug for Cheeks {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Cheeks")
            .field("vertices", &self.bulge.blown().vertices.len())
            .field("slack", &self.slack)
            .field("tight", &self.tight)
            .finish()
    }
}

/// The parts of a lobber's model that show what it is about to do.
#[derive(Debug, Clone)]
pub struct LobberBody {
    /// The bag that empties with a throw and fills for the next.
    pub bag: Swell,
    /// The head, if it has one fastened into the bag: it rides the bag and
    /// carries the cheeks and eyes with it.
    pub head: Option<Rider>,
    /// Cheeks that blow the bag full, if it has them.
    pub cheeks: Option<Cheeks>,
    /// Eyes that follow what it throws at.
    pub eyes: [Option<Gaze>; 2],
    /// Everything else fastened into the bag (pipes, a hand on one).
    pub riders: Vec<Rider>,
    /// What rests on the frame and is held by the bag or a rider (an arm).
    pub struts: Vec<Strut>,
    /// The part that stands under the bag (legs), if it has one: it moves
    /// only with the heave of the whole.
    pub frame: Option<usize>,
}

/// Where a lobber is in a throw, for its body to show.
#[derive(Debug, Clone, Copy)]
pub struct Throw {
    /// How far it is from one throw to the next, 0 to 1.
    pub fullness: f32,
    /// Seconds since it last threw, if it has.
    pub since: Option<f32>,
    /// Seconds until it throws, if it has something to throw at.
    pub until: Option<f32>,
}

impl Throw {
    /// How the whole body is scaled about its seat this instant: squatting
    /// and trembling into a throw, stretched up and bouncing after one.
    pub fn heave(&self, clock: f32) -> Vector3<f32> {
        let windup = self.until.map_or(0.0, |until| {
            let into = (1.0 - until / WINDUP_SECONDS).clamp(0.0, 1.0);
            into * into * (3.0 - 2.0 * into)
        });
        let tremble = TREMBLE * windup * (clock * TREMBLE_RATE * TAU).sin();
        let release = self.since.map_or(0.0, |since| {
            HEAVE * (-since / HEAVE_DECAY).exp() * (since * HEAVE_RATE * TAU).cos()
        });
        let up = 1.0 - WINDUP_SQUAT * windup + tremble + release;
        let across = 1.0 + WINDUP_SPREAD * windup - 0.5 * release;
        Vector3::new(across, up, across)
    }
}

/// What a lobber's body shows at an instant.
#[derive(Debug, Clone, Copy)]
pub struct Showing {
    /// Where it is in its throw.
    pub throw: Throw,
    /// Seconds into its breathing.
    pub clock: f32,
    /// How far its cheeks are blown, 0 between puffs to 1 at the top of
    /// the hardest.
    pub blown: f32,
    /// What it watches, in the entity's frame.
    pub target: Option<Vector3<f32>>,
}

impl LobberBody {
    /// Pose the parts it moves in `poses` (one to a part of the model),
    /// and shape its cheeks into `shaped`, as `now` shows.
    pub fn pose(&self, poses: &mut [Transform], shaped: &mut Vec<MeshPrimitive>, now: &Showing) {
        let fullness = now.throw.fullness.clamp(0.0, 1.0);
        // It fills fast at first and strains at the end.
        let filled = 1.0 - (1.0 - fullness) * (1.0 - fullness);
        let breath = BREATH_DEPTH * (now.clock * BREATH_RATE * TAU).sin();
        let cheeks = self.cheeks.as_ref().map_or(0.0, |cheeks| {
            cheeks.slack + (cheeks.tight - cheeks.slack) * now.blown
        });
        self.place(
            poses,
            shaped,
            filled + breath,
            cheeks,
            now.target,
            now.throw.heave(now.clock),
        );
    }

    /// Pose the parts it moves as a dead lobber's lie: the bag sunk, the
    /// cheeks empty, the eyes on nothing.
    pub fn slump(&self, poses: &mut [Transform], shaped: &mut Vec<MeshPrimitive>) {
        self.place(
            poses,
            shaped,
            -DEAD_SAG,
            -DEAD_SAG,
            None,
            Vector3::repeat(1.0),
        );
    }

    /// Pose every part it moves: the bag `full`, what rides it carried
    /// with it, the cheeks `blown`, the eyes on `target`, and the whole
    /// scaled by `heave` about the bag's seat.
    fn place(
        &self,
        poses: &mut [Transform],
        shaped: &mut Vec<MeshPrimitive>,
        full: f32,
        blown: f32,
        target: Option<Vector3<f32>>,
        heave: Vector3<f32>,
    ) {
        let bag = &self.bag;
        let scale = bag.scale(full);
        set(poses, bag.part, bag.pose(full));
        for rider in &self.riders {
            set(poses, rider.part, rider.carried(bag, scale));
        }
        for strut in &self.struts {
            let held = strut.held_by.map_or_else(
                || bag.pose(full),
                |part| poses.get(part).cloned().unwrap_or_default(),
            );
            let pose = strut.turned(|point| {
                held.to_matrix()
                    .transform_point(&Point3::from(point))
                    .coords
            });
            set(poses, strut.part, pose);
        }
        if let Some(frame) = self.frame {
            set(poses, frame, Transform::default());
        }
        let head = self
            .head
            .map_or_else(Transform::default, |head| head.carried(bag, scale));
        if let Some(rider) = self.head {
            set(poses, rider.part, head.clone());
        }
        for eye in self.eyes.iter().flatten() {
            set(poses, eye.part, head.compose(&eye.pose(target)));
        }

        let parts = [Some(bag.part), self.head.map(|head| head.part), self.frame]
            .into_iter()
            .chain(self.riders.iter().map(|rider| Some(rider.part)))
            .chain(self.struts.iter().map(|strut| Some(strut.part)))
            .chain(self.eyes.iter().map(|eye| eye.map(|eye| eye.part)))
            .flatten();
        for part in parts {
            if let Some(pose) = poses.get_mut(part) {
                *pose = heaved(bag.seat, heave).compose(pose);
            }
        }

        if let Some(cheeks) = &self.cheeks {
            if shaped.is_empty() {
                shaped.push(cheeks.bulge.blown().clone());
            }
            let place = heaved(bag.seat, heave).to_matrix() * head.to_matrix();
            cheeks.bulge.shape(blown, &place, &mut shaped[0]);
        }
    }
}

/// The scale of the whole by `heave` about the bag's `seat`.
fn heaved(seat: Vector3<f32>, heave: Vector3<f32>) -> Transform {
    Transform {
        translation: seat - seat.component_mul(&heave),
        rotation: UnitQuaternion::identity(),
        scale: heave,
    }
}

/// Put `pose` in `poses` for `part`, if the model has such a part.
fn set(poses: &mut [Transform], part: usize, pose: Transform) {
    if let Some(slot) = poses.get_mut(part) {
        *slot = pose;
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Vector2, Vector4};

    use super::*;
    use crate::rendering::material::MaterialId;
    use crate::rendering::vertex::Vertex;

    /// A lobber whose model is a bag (part 0) about a ball 0.4 m across,
    /// a head on top of it (part 1), an eye in that (part 2), legs (part
    /// 3) and a pipe out of the bag's side (part 4).
    fn body() -> LobberBody {
        let up = Vector3::y();
        LobberBody {
            bag: Swell {
                part: 0,
                seat: Vector3::new(0.0, -0.4, 0.0),
                slack: Vector3::new(1.0, 0.85, 1.0),
                tight: Vector3::new(1.05, 1.1, 1.05),
            },
            head: Some(Rider {
                part: 1,
                anchor: up * 0.4,
                out: up,
            }),
            cheeks: None,
            eyes: [
                Some(Gaze {
                    part: 2,
                    socket: Vector3::new(0.0, 0.5, 0.1),
                    ahead: Vector3::z(),
                    swivel: 0.6,
                }),
                None,
            ],
            riders: vec![Rider {
                part: 4,
                anchor: Vector3::new(1.0, 1.0, 0.0).normalize() * 0.4,
                out: Vector3::new(1.0, 1.0, 0.0).normalize(),
            }],
            struts: Vec::new(),
            frame: Some(3),
        }
    }

    /// At `fullness`, with no throw lately and none coming, at the top of
    /// its breath's cycle.
    fn resting(fullness: f32) -> Showing {
        Showing {
            throw: Throw {
                fullness,
                since: None,
                until: None,
            },
            clock: 0.0,
            blown: 0.0,
            target: None,
        }
    }

    fn posed(body: &LobberBody, now: &Showing) -> Vec<Transform> {
        let mut poses = vec![Transform::default(); 5];
        body.pose(&mut poses, &mut Vec::new(), now);
        poses
    }

    fn placed(pose: &Transform, point: Vector3<f32>) -> Vector3<f32> {
        pose.to_matrix()
            .transform_point(&Point3::from(point))
            .coords
    }

    #[test]
    fn its_bag_sags_after_a_throw_and_swells_tight_as_the_next_comes() {
        let body = body();
        // At the same instant of its breath, from the throw to the next.
        let mut last = 0.0;
        for step in 0..=8 {
            let poses = posed(&body, &resting(step as f32 / 8.0));
            let height = poses[0].scale.y;
            assert!(height > last, "it sank at step {step}: {height}");
            last = height;
            // Wherever it has got to, it still sits where it sat.
            assert!((placed(&poses[0], body.bag.seat) - body.bag.seat).norm() < 1e-5);
        }
        assert!((posed(&body, &resting(0.0))[0].scale - body.bag.slack).norm() < 1e-5);
        assert!((posed(&body, &resting(1.0))[0].scale - body.bag.tight).norm() < 1e-5);

        // A breath moves it a little, and never as far as a throw does.
        let breathing: Vec<f32> = (0..40)
            .map(|tick| {
                let mut now = resting(0.5);
                now.clock = tick as f32 * 0.1;
                posed(&body, &now)[0].scale.y
            })
            .collect();
        let (low, high) = breathing
            .iter()
            .fold((f32::MAX, f32::MIN), |(lo, hi), h| (lo.min(*h), hi.max(*h)));
        assert!(high > low, "it does not breathe");
        assert!(high - low < 0.25 * (body.bag.tight.y - body.bag.slack.y));

        // Dead, it lies lower than it ever did alive.
        let mut poses = vec![Transform::default(); 5];
        body.slump(&mut poses, &mut Vec::new());
        assert!(poses[0].scale.y < body.bag.slack.y);
    }

    #[test]
    fn what_is_fastened_into_the_bag_rides_it_and_the_legs_do_not() {
        let body = body();
        for now in [resting(0.0), resting(1.0), {
            let mut breathing = resting(0.5);
            breathing.clock = 0.8;
            breathing
        }] {
            let poses = posed(&body, &now);
            let bag = &poses[0];
            for rider in body.riders.iter().chain(&body.head) {
                let pose = &poses[rider.part];
                // Its anchor is where the bag's skin it is fastened to is.
                let skin = placed(bag, rider.anchor);
                assert!((placed(pose, rider.anchor) - skin).norm() < 1e-5);
                // It stands out of the skin square, as it was sculpted: the
                // bag is a ball scaled, so its way out is the inverse
                // scale's.
                let out = pose.rotation * rider.out;
                let square = rider.out.component_div(&bag.scale).normalize();
                assert!(out.dot(&square) > 1.0 - 1e-5, "{out:?} not {square:?}");
                assert!((pose.scale - Vector3::repeat(1.0)).norm() < 1e-6);
            }
            assert!((poses[3].translation).norm() < 1e-6, "the legs moved");
        }
        // The pipe out of its shoulder leans over as the bag swells taller
        // than it swells wide, and stands up as it sags.
        let rider = body.riders[0];
        let rise = |fullness| (posed(&body, &resting(fullness))[4].rotation * rider.out).y;
        assert!(rise(0.0) > rider.out.y + 0.01 && rise(1.0) < rider.out.y - 0.01);
    }

    #[test]
    fn an_arm_resting_on_its_legs_turns_to_keep_hold_of_what_rides_the_bag() {
        let mut body = body();
        let pipe = body.riders[0];
        let (pivot, reach) = (Vector3::new(0.5, -0.3, 0.0), pipe.anchor);
        body.struts = vec![
            Strut {
                part: 5,
                pivot,
                reach,
                held_by: Some(pipe.part),
            },
            Strut {
                part: 6,
                pivot,
                reach,
                held_by: None,
            },
        ];
        for fullness in [0.0, 1.0] {
            let mut poses = vec![Transform::default(); 7];
            body.pose(&mut poses, &mut Vec::new(), &resting(fullness));
            let held = placed(&poses[pipe.part], reach);
            assert!((placed(&poses[0], reach) - held).norm() < 1e-5);
            for strut in [5, 6] {
                let arm = &poses[strut];
                assert!(
                    (placed(arm, pivot) - pivot).norm() < 1e-5,
                    "it left its rest"
                );
                let toward = (placed(arm, reach) - pivot).normalize();
                assert!(toward.dot(&(held - pivot).normalize()) > 1.0 - 1e-5);
            }
        }
    }

    #[test]
    fn its_eyes_ride_in_its_head() {
        let body = body();
        for fullness in [0.0, 1.0] {
            let poses = posed(&body, &resting(fullness));
            let socket = body.eyes[0].unwrap().socket;
            assert!((poses[2].translation - placed(&poses[1], socket)).norm() < 1e-5);
        }
    }

    #[test]
    fn it_squats_into_a_throw_and_heaves_up_as_the_bomb_leaves() {
        let body = body();
        let height = |throw: Throw| {
            // At a crossing of its tremble, so only the squat shows.
            let mut now = resting(0.5);
            now.throw = throw;
            posed(&body, &now)[3].scale.y
        };
        let still = height(resting(0.5).throw);
        assert!((still - 1.0).abs() < 1e-5, "at rest it is as sculpted");
        let due = |until| Throw {
            fullness: 1.0,
            since: None,
            until: Some(until),
        };
        assert!((height(due(2.0)) - 1.0).abs() < 1e-5, "too early to squat");
        let squat = height(due(0.0));
        assert!(
            squat < 1.0 - 0.8 * WINDUP_SQUAT,
            "it did not squat: {squat}"
        );
        let thrown = |since| Throw {
            fullness: 0.0,
            since: Some(since),
            until: None,
        };
        let heave = height(thrown(0.0));
        assert!(heave > 1.0 + 0.9 * HEAVE, "it did not heave: {heave}");
        assert!((height(thrown(2.0)) - 1.0).abs() < 0.01, "it never settled");

        // Squatting or heaving, it stays on its seat.
        let mut now = resting(1.0);
        now.throw = due(0.0);
        let poses = posed(&body, &now);
        assert!((placed(&poses[0], body.bag.seat) - body.bag.seat).norm() < 1e-5);
    }

    #[test]
    fn its_cheeks_blow_out_of_its_head_and_go_where_it_goes() {
        // One triangle: its first corner on the face at every size, its
        // others 2 cm proud of it when blown.
        let vertex = |x: f32, z: f32, proud: f32| Vertex {
            pos: Vector3::new(x, 0.5 + proud, z),
            color: Vector4::repeat(1.0),
            tex_coords: Vector2::zeros(),
            normal: Vector3::y(),
            ao: 1.0,
        };
        let blown = MeshPrimitive {
            vertices: vec![
                vertex(0.0, 0.0, 0.0),
                vertex(0.05, 0.0, 0.02),
                vertex(0.0, 0.05, 0.02),
            ],
            indices: vec![0, 1, 2],
            material: MaterialId(0),
        };
        let empty = blown
            .vertices
            .iter()
            .map(|vertex| (Vector3::new(vertex.pos.x, 0.5, vertex.pos.z), Vector3::y()))
            .collect();
        let bulge = Bulge::new(blown, empty);
        let mut body = body();
        body.cheeks = Some(Cheeks {
            bulge: Arc::new(bulge),
            slack: 0.0,
            tight: 1.0,
        });
        let mut poses = vec![Transform::default(); 5];
        let mut shaped = Vec::new();
        let mut proud = |blown: f32, fullness: f32| {
            let mut now = resting(fullness);
            now.blown = blown;
            body.pose(&mut poses, &mut shaped, &now);
            let head = &poses[1];
            let face = |at: Vector3<f32>| placed(head, at);
            let edge = shaped[0].vertices[0].pos;
            assert!((edge - face(Vector3::new(0.0, 0.5, 0.0))).norm() < 1e-5);
            shaped[0].vertices[1].pos.y - face(Vector3::new(0.05, 0.5, 0.0)).y
        };
        for fullness in [0.0, 1.0] {
            assert!(proud(0.0, fullness).abs() < 1e-5);
            assert!((proud(1.0, fullness) - 0.02).abs() < 1e-5);
            assert!((proud(0.5, fullness) - 0.01).abs() < 1e-5);
        }
    }
}
