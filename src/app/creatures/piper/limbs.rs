//! The piper's limbs: two small legs folded under the bag, and the hand
//! that fingers the chanter on an arm out of the bag's side.

use nalgebra::{UnitQuaternion, Vector3};

use super::frame::RADIUS;
use super::pipes::Pipe;
use super::shaping::{placed, triple, turned_to, wash};
use crate::animation::FootSide;
use crate::app::creatures::anatomy::{self, DIGIT_GRAIN, FLESH, FLUSH};
use crate::app::sculpt::{Clay, Grime, Lump, Mottle, VertexPaint};
use crate::model::MeshPrimitive;

/// Side of the grid the legs and the arm are skinned from, m.
const LIMB_GRAIN: f32 = 0.011;
/// What the limbs' skin is washed with: darker and redder than a walker's
/// legs, as limbs are that are sat on and never stretched.
const LIVID: Vector3<f32> = Vector3::new(1.0, 0.76, 0.66);

/// A leg folded under the bag: the thigh out of the bag's side, the
/// shin doubled back under it, the foot flat and turned out.
pub(super) fn leg(side: FootSide) -> Vec<MeshPrimitive> {
    let out = match side {
        FootSide::Left => 1.0,
        FootSide::Right => -1.0,
    };
    let girth = 0.054;
    let splay: f32 = 0.55 * out;
    let toes = Vector3::new(splay.sin(), 0.0, splay.cos());
    let sole = Vector3::new(0.3 * out, -RADIUS, 0.36);
    let knee = sole + Vector3::new(0.05 * out, 0.17, 0.11);
    let hip = Vector3::new(0.23 * out, -0.28, 0.25);

    let mut thigh = anatomy::flesh(&anatomy::thigh((knee - hip).norm(), girth), LIMB_GRAIN);
    let mut shin = anatomy::flesh(&anatomy::shin((knee - sole).norm(), girth), LIMB_GRAIN);
    // Legs that have borne the bag a long time without moving.
    for mesh in [&mut thigh, &mut shin] {
        Mottle {
            frequency: 9.0,
            value: 0.2,
            warmth: -0.16,
            seed: 93,
        }
        .paint(mesh);
        wash(mesh, |_| LIVID);
    }
    Grime {
        below: -(knee - sole).norm() + 0.05,
        fade: 0.12,
        tint: Vector3::new(0.5, 0.42, 0.34),
    }
    .paint(&mut shin);
    vec![
        placed(thigh, turned_to(hip - knee, Vector3::y() + toes * 0.4), hip),
        placed(shin, turned_to(knee - sole, toes), knee),
        placed(
            {
                let mut foot = anatomy::foot(0.165, side);
                wash(&mut foot, |_| LIVID);
                foot
            },
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), splay),
            sole,
        ),
    ]
}

/// Where the arm that fingers the chanter bends, in the piper's frame.
pub(super) struct ArmJoints {
    /// The wrist, where the hand meets the forearm.
    pub(super) wrist: Vector3<f32>,
    /// The elbow, resting on the thigh.
    pub(super) elbow: Vector3<f32>,
    /// The shoulder, in the bag.
    pub(super) shoulder: Vector3<f32>,
}

/// The arm's hand, forearm and upper arm, in the piper's frame.
pub(super) struct Arm {
    pub(super) hand: MeshPrimitive,
    pub(super) forearm: MeshPrimitive,
    pub(super) upper_arm: MeshPrimitive,
}

/// Size of the hand, m.
const HAND_SIZE: f32 = 0.16;

/// How the hand holds `chanter`: the turn that sets it on the pipe, and
/// where the arm's joints are.
fn hold(chanter: &Pipe) -> (UnitQuaternion<f32>, ArmJoints) {
    let (top, end) = (chanter.foot, chanter.tip);
    let down = (end - top).normalize();
    let held = top + (end - top) * 0.47;
    // The fingers lie across the pipe from outside it, the back of the
    // hand to whoever looks.
    let inward = -Vector3::x();
    let fingers = (inward - down * inward.dot(&down)).normalize();
    let turn = UnitQuaternion::from_basis_unchecked(&[down, fingers.cross(&down), fingers]);
    // The pipe lies in the crook of the fingers.
    let crook = Vector3::new(0.0, -0.19 * HAND_SIZE, 0.56 * HAND_SIZE);
    let joints = ArmJoints {
        wrist: held - turn * crook,
        // The elbow rests on the thigh; the rest of the arm is in the bag.
        elbow: Vector3::new(0.36, -0.2, 0.38),
        shoulder: Vector3::new(0.27, -0.02, 0.2),
    };
    (turn, joints)
}

/// Where the arm that fingers `chanter` bends.
pub(super) fn arm_joints(chanter: &Pipe) -> ArmJoints {
    hold(chanter).1
}

/// The hand that fingers the chanter, and the arm it is on, out of the
/// bag's side.
pub(super) fn arm(chanter: &Pipe) -> Arm {
    let (
        turn,
        ArmJoints {
            wrist,
            elbow,
            shoulder,
        },
    ) = hold(chanter);
    let fingers = turn * Vector3::z();
    // The forearm is thin at the wrist and thick with muscle below the
    // elbow.
    let cuff = wrist - fingers * 0.03;
    let belly = elbow + (cuff - elbow) * 0.38;
    let forearm = Clay::new(0.025)
        .add(
            Lump::limb(triple(cuff), triple(belly), (0.017, 0.029)),
            FLESH,
        )
        .add(
            Lump::limb(triple(belly), triple(elbow), (0.029, 0.025)),
            FLESH,
        )
        .add(Lump::ball(0.026, triple(elbow)), FLUSH);
    let upper_arm = Clay::new(0.025).add(
        Lump::limb(triple(elbow), triple(shoulder), (0.027, 0.036)),
        FLESH,
    );
    let mut hand = anatomy::flesh(
        &anatomy::hand(HAND_SIZE, 1.12, FootSide::Left),
        1.2 * DIGIT_GRAIN,
    );
    wash(&mut hand, |_| LIVID);
    let skin = |clay: &Clay| {
        let mut mesh = anatomy::flesh(clay, LIMB_GRAIN);
        wash(&mut mesh, |_| LIVID);
        mesh
    };
    Arm {
        hand: placed(hand, turn, wrist),
        forearm: skin(&forearm),
        upper_arm: skin(&upper_arm),
    }
}
