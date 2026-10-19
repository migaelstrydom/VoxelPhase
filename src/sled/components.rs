//! The hover sled component: the deck a rider stands on and how it glides.

use nalgebra::{Point3, Vector2, Vector3};
use specs::{Component, DenseVecStorage};

use super::glide::Glide;

/// How high above the deck a body may stand and still be riding it, m.
/// A capsule's centre stands half its height up, under a metre.
const RIDE_HEIGHT: f32 = 1.4;
/// How far past the deck's edge a rider's centre may be, m: a capsule
/// standing on the very edge has its centre a radius out.
const EDGE_MARGIN: f32 = 0.3;

/// A body held up by hover pods, which carries and is steered by whoever
/// stands on its deck.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct HoverSled {
    /// Half the deck's length (`x`, forward) and width (`z`), in the body's
    /// frame.
    pub deck: Vector2<f32>,
    /// Height of the deck above the body's origin, in its frame.
    pub deck_height: f32,
    /// How it answers its rider's lean.
    pub glide: Glide,
    /// The heading it holds while ridden, in radians about `+y` from `+x`:
    /// the one it had when its rider boarded. `None` while nobody rides.
    pub held_heading: Option<f32>,
}

impl HoverSled {
    /// A sled with this deck and glide, ridden by nobody yet.
    pub fn new(deck: Vector2<f32>, deck_height: f32, glide: Glide) -> Self {
        Self {
            deck,
            deck_height,
            glide,
            held_heading: None,
        }
    }

    /// The heading of a sled whose `x` axis points along `forward`, in
    /// radians about `+y` from `+x`.
    pub fn heading(forward: &Vector3<f32>) -> f32 {
        (-forward.z).atan2(forward.x)
    }

    /// Where a body at `local`, in the sled's frame, stands on the deck: its
    /// offset from the deck's middle as a share of the deck's half extents.
    /// `None` if it is not over the deck.
    pub fn stance(&self, local: &Point3<f32>) -> Option<Vector2<f32>> {
        let up = local.y - self.deck_height;
        let over = local.x.abs() <= self.deck.x + EDGE_MARGIN
            && local.z.abs() <= self.deck.y + EDGE_MARGIN;
        (over && (0.0..=RIDE_HEIGHT).contains(&up))
            .then(|| Vector2::new(local.x / self.deck.x, local.z / self.deck.y))
    }

    /// The lean of a rider standing at `stance`, as a level world direction
    /// of length 0..1, for the sled turned by `forward` and `across` (its
    /// `x` and `z` axes in the world).
    pub fn lean(
        &self,
        stance: Vector2<f32>,
        forward: &Vector3<f32>,
        across: &Vector3<f32>,
    ) -> Vector3<f32> {
        let lean = self.glide.lean(stance);
        let level = |v: &Vector3<f32>| Vector3::new(v.x, 0.0, v.z).try_normalize(1e-6);
        let (Some(forward), Some(across)) = (level(forward), level(across)) else {
            return Vector3::zeros();
        };
        forward * lean.x + across * lean.y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sled() -> HoverSled {
        HoverSled::new(Vector2::new(0.6, 0.4), 0.1, Glide::HOVER)
    }

    #[test]
    fn a_body_standing_on_the_deck_rides_it() {
        let stance = sled().stance(&Point3::new(0.3, 0.6, -0.2));
        assert_eq!(stance, Some(Vector2::new(0.5, -0.5)));
    }

    #[test]
    fn a_body_beside_or_under_it_does_not() {
        assert_eq!(sled().stance(&Point3::new(1.2, 0.6, 0.0)), None);
        assert_eq!(sled().stance(&Point3::new(0.0, -0.5, 0.0)), None);
        assert_eq!(sled().stance(&Point3::new(0.0, 2.0, 0.0)), None);
    }

    #[test]
    fn a_rider_at_the_nose_leans_the_way_the_sled_points() {
        let forward = Vector3::new(0.0, 0.0, -1.0);
        let across = Vector3::new(1.0, 0.0, 0.0);
        let lean = sled().lean(Vector2::new(1.0, 0.0), &forward, &across);
        assert!((lean - forward).norm() < 1e-6);
    }
}
