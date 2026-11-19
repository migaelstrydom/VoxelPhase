//! A shaft of light on an entity, and how it changes.

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, Join, Read, System, WriteStorage};

use crate::time::Time;

/// How a shaft looks at a moment: what an owner aims it at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShaftLook {
    /// Its colour (linear).
    pub colour: Vector3<f32>,
    /// How much light it gives, as luminance through its brightest part.
    pub radiance: f32,
    /// How much of it there is: 0 is none, 1 all of it.
    pub presence: f32,
}

impl ShaftLook {
    /// No shaft at all.
    pub const NONE: Self = Self {
        colour: Vector3::new(0.0, 0.0, 0.0),
        radiance: 0.0,
        presence: 0.0,
    };

    /// Part of the way from `self` to `to`.
    fn toward(&self, to: &Self, share: f32) -> Self {
        Self {
            colour: self.colour.lerp(&to.colour, share),
            radiance: self.radiance + (to.radiance - self.radiance) * share,
            presence: self.presence + (to.presence - self.presence) * share,
        }
    }
}

/// A column of light standing straight up from a point on its entity.
#[derive(Component, Clone, Copy, Debug)]
#[storage(DenseVecStorage)]
pub struct LightShaft {
    /// The middle of its foot, in the entity's frame, m. The shaft itself
    /// always stands plumb.
    pub base: Vector3<f32>,
    /// How tall it stands, m.
    pub height: f32,
    /// Its radius at its foot and at its top, m.
    pub foot_radius: f32,
    pub top_radius: f32,
    /// How it looks now.
    pub look: ShaftLook,
    /// How its owner wants it to look; `look` follows over `ease` seconds.
    pub aim: ShaftLook,
    pub ease: f32,
    /// How fast the streaks in it rise, m/s, and how far they have risen.
    pub rise: f32,
    pub risen: f32,
}

impl LightShaft {
    /// A shaft standing from `base` to `height`, looking as `look` already.
    pub fn new(base: Vector3<f32>, height: f32, radii: (f32, f32), look: ShaftLook) -> Self {
        Self {
            base,
            height,
            foot_radius: radii.0,
            top_radius: radii.1,
            look,
            aim: look,
            ease: DEFAULT_EASE,
            rise: DEFAULT_RISE,
            risen: 0.0,
        }
    }

    /// The same shaft with its foot moved by `by`, in its entity's frame.
    pub fn moved_by(mut self, by: Vector3<f32>) -> Self {
        self.base += by;
        self
    }

    /// Move on by `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        let share = if self.ease > 0.0 {
            (dt / self.ease).min(1.0)
        } else {
            1.0
        };
        self.look = self.look.toward(&self.aim, share);
        self.risen = (self.risen + self.rise * dt) % RISE_WRAP;
    }
}

/// Seconds for a shaft to settle on a new look: a slow kindling.
const DEFAULT_EASE: f32 = 0.6;
/// Metres a second its streaks rise.
const DEFAULT_RISE: f32 = 1.6;
/// Where the distance risen wraps round, m: far longer than any streak, so
/// the wrap is never seen, and short enough to keep its precision.
const RISE_WRAP: f32 = 1000.0;

/// Eases every shaft toward its aim and lifts its streaks.
pub struct LightShaftSystem;

impl<'a> System<'a> for LightShaftSystem {
    type SystemData = (Read<'a, Time>, WriteStorage<'a, LightShaft>);

    fn run(&mut self, (time, mut shafts): Self::SystemData) {
        let dt = time.delta_seconds();
        for shaft in (&mut shafts).join() {
            shaft.advance(dt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_shaft_eases_to_its_aim_and_stays_there() {
        let lit = ShaftLook {
            colour: Vector3::new(1.0, 0.7, 0.3),
            radiance: 1.0,
            presence: 1.0,
        };
        let mut shaft = LightShaft::new(Vector3::zeros(), 30.0, (0.3, 1.0), ShaftLook::NONE);
        shaft.aim = lit;
        shaft.advance(0.1);
        assert!(shaft.look.presence > 0.0 && shaft.look.presence < 1.0);
        for _ in 0..600 {
            shaft.advance(1.0 / 60.0);
        }
        assert!((shaft.look.presence - 1.0).abs() < 1e-3);
        assert!((shaft.look.colour - lit.colour).norm() < 1e-3);
    }
}
