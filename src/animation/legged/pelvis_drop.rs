//! Lowering the pelvis so the lower foot can reach its ground.
//!
//! The pelvis hangs a fixed distance under the physics body, and a body on a
//! slope rides as high above the ground under its centre as it does on the
//! flat. Walking along a hillside, the downhill hip is then further from its
//! ground than the leg is long: the foot cannot plant, and the placer holds it
//! in a swing that drags along beside the body. A person bends the uphill knee
//! instead, and this does the same: it lowers the pelvis until the lower hip
//! stands no higher over its ground than it would on the flat.
//!
//! The ground is the plane of the surface holding the body up, through a
//! planted foot. Neither alone will do: a swinging foot's probe reads its
//! landing, and a landing over a ledge is a drop the body has not taken yet;
//! the support normal alone says how the ground tilts but not where it is.
//!
//! ```text
//!   support normal + planted foothold ──► ground plane ──► height under each hip
//!                                                                 │
//!               target = deepest hip gap − standing_height ◄──────┘
//!                       │  (clamped to [0, max]; held with no foot down,
//!                       ▼   zero unsupported)
//!                 eased drop ──► pelvis_for
//! ```

use nalgebra::{Point3, Vector3};

use super::super::foot_placer::{floor_height_at, is_floor, FootSide};
use super::dims::LegRigDims;

/// Deepest the pelvis may drop, as a fraction of leg length. Beyond this the
/// uphill knee folds past anything that reads as walking.
const MAX_DROP_RATIO: f32 = 0.4;

/// Time constant of the ease toward the target drop, in seconds: quick enough
/// to follow a hillside's turns, slow enough that a foot probing a lower
/// landing for one swing does not bob the body.
const EASE_SECONDS: f32 = 0.12;

/// How far the pelvis sits below where the body alone would hang it.
#[derive(Clone, Copy, Debug, Default)]
pub struct PelvisDrop {
    /// Current drop, in metres, never negative.
    drop: f32,
}

impl PelvisDrop {
    /// The current drop, in metres.
    pub fn drop(&self) -> f32 {
        self.drop
    }

    /// Ease toward the drop the ground under the hips asks for.
    ///
    /// `pelvis` is the pelvis before any drop. `support` is the normal of the
    /// surface holding the body up, `None` when nothing is. `footholds` are
    /// where the planted feet stand, `[left, right]`, `None` for a foot in
    /// the air. With no foot down there is nothing to measure from and the
    /// drop holds; an unsupported body eases back to none, since nothing is
    /// under its feet to reach for.
    pub fn update(
        &mut self,
        pelvis: Point3<f32>,
        yaw: f32,
        support: Option<Vector3<f32>>,
        footholds: [Option<Point3<f32>>; 2],
        dims: &LegRigDims,
        dt: f32,
    ) {
        let target = match support {
            Some(normal) => Self::target(pelvis, yaw, normal, footholds, dims),
            None => Some(0.0),
        };
        if let Some(target) = target {
            let blend = 1.0 - (-dt / EASE_SECONDS).exp();
            self.drop += (target - self.drop) * blend;
        }
    }

    /// The drop that puts the lower hip `standing_height` over the ground
    /// beneath it. Each hip reads the plane through its own foot when that
    /// is planted, the other foot's when not. `None` with no foot planted.
    fn target(
        pelvis: Point3<f32>,
        yaw: f32,
        normal: Vector3<f32>,
        footholds: [Option<Point3<f32>>; 2],
        dims: &LegRigDims,
    ) -> Option<f32> {
        if !is_floor(normal) {
            return Some(0.0);
        }
        let [left, right] = footholds;
        let across = Vector3::new(yaw.sin(), 0.0, yaw.cos()).cross(&Vector3::y());
        let deepest = [
            (FootSide::Left, left.or(right)),
            (FootSide::Right, right.or(left)),
        ]
        .into_iter()
        .filter_map(|(side, foothold)| {
            let foothold = foothold?;
            let hip = pelvis + across * (side.side_sign() * dims.hip_width);
            Some(hip.y - floor_height_at(foothold, normal, hip.x, hip.z))
        })
        .reduce(f32::max)?;
        Some((deepest - dims.standing_height).clamp(0.0, MAX_DROP_RATIO * dims.leg_length))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dims() -> LegRigDims {
        LegRigDims {
            hip_width: 0.12,
            leg_length: 0.5,
            standing_height: 0.425,
            probe_length: 1.0,
        }
    }

    /// A foothold on the plane through the origin whose normal is `normal`.
    fn on_plane(at: Point3<f32>, normal: Vector3<f32>) -> Option<Point3<f32>> {
        let y = -(normal.x * at.x + normal.z * at.z) / normal.y;
        Some(Point3::new(at.x, y, at.z))
    }

    fn settle(
        drop: &mut PelvisDrop,
        pelvis: Point3<f32>,
        support: Option<Vector3<f32>>,
        footholds: [Option<Point3<f32>>; 2],
    ) -> f32 {
        for _ in 0..120 {
            drop.update(pelvis, 0.0, support, footholds, &dims(), 1.0 / 60.0);
        }
        drop.drop()
    }

    fn settled(
        pelvis: Point3<f32>,
        support: Option<Vector3<f32>>,
        footholds: [Option<Point3<f32>>; 2],
    ) -> f32 {
        settle(&mut PelvisDrop::default(), pelvis, support, footholds)
    }

    #[test]
    fn level_ground_asks_for_nothing() {
        let dims = dims();
        let pelvis = Point3::new(0.0, dims.standing_height, 0.0);
        let flat = on_plane(Point3::new(0.1, 0.0, 0.1), Vector3::y());
        assert!(settled(pelvis, Some(Vector3::y()), [flat, flat]) < 1e-4);
    }

    /// Facing +z the right hip is at −x. A hillside falling toward −x puts
    /// that hip's ground `hip_width · grade` lower than the ground under the
    /// body, and that is what the pelvis comes down by.
    #[test]
    fn a_hillside_lowers_the_pelvis_to_the_downhill_hip() {
        let dims = dims();
        let grade = 35f32.to_radians().tan();
        let normal = Vector3::new(-grade, 1.0, 0.0).normalize();
        let pelvis = Point3::new(0.0, dims.standing_height, 0.0);
        // Only the uphill foot is down: the downhill one is the foot that
        // cannot reach, and it is the plane through the other that finds
        // its ground.
        let footholds = [on_plane(Point3::new(0.12, 0.0, 0.1), normal), None];
        let drop = settled(pelvis, Some(normal), footholds);
        let expected = dims.hip_width * grade;
        assert!(
            (drop - expected).abs() < 2e-3,
            "dropped {drop:.3} m, the downhill hip is {expected:.3} m low"
        );
    }

    #[test]
    fn an_unsupported_body_does_not_reach_for_the_ground() {
        let dims = dims();
        let pelvis = Point3::new(0.0, dims.standing_height + 0.2, 0.0);
        let flat = on_plane(Point3::new(0.1, 0.0, 0.1), Vector3::y());
        let mut drop = PelvisDrop::default();
        assert!(settle(&mut drop, pelvis, Some(Vector3::y()), [flat, None]) > 0.1);
        assert!(settle(&mut drop, pelvis, None, [flat, None]) < 1e-4);
    }

    /// Walking to the rim of a raised floor, the swinging foot's probe finds
    /// the ground below the edge. The body has not stepped down, and the
    /// planted foot on the rim says so.
    #[test]
    fn ground_past_a_ledge_is_not_reached_for() {
        let dims = dims();
        let pelvis = Point3::new(0.0, dims.standing_height, 0.0);
        let rim = Some(Point3::new(-0.12, 0.0, 0.0));
        assert!(settled(pelvis, Some(Vector3::y()), [rim, None]) < 1e-4);
    }

    /// With both feet in the air there is no foothold to measure from, and a
    /// running body's flight phase is no reason to stand back up.
    #[test]
    fn the_drop_holds_through_a_flight_phase() {
        let dims = dims();
        let pelvis = Point3::new(0.0, dims.standing_height + 0.1, 0.0);
        let foothold = on_plane(Point3::new(0.12, 0.0, 0.0), Vector3::y());
        let mut drop = PelvisDrop::default();
        let held = settle(&mut drop, pelvis, Some(Vector3::y()), [foothold, None]);
        assert!(held > 0.05);
        assert_eq!(
            settle(&mut drop, pelvis, Some(Vector3::y()), [None, None]),
            held
        );
    }
}
