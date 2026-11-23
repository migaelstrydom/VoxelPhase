use nalgebra::Vector3;

use super::config::HarmConfig;
use crate::physics::{Hit, RigidBodyHandle, Shove};

/// The least give a blow is judged over, m: one claiming none would be
/// infinitely sudden.
const MIN_GIVE: f32 = 1.0e-4;

/// How bad a blow was, mildest first.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Band {
    /// Shoved; the body keeps its footing and control.
    Light,
    /// Knocked down: limp for a while, then getting up.
    Heavy,
    /// Knocked down, and the room restarts.
    Deadly,
}

/// One blow on a body, as harm reads it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Blow {
    /// The change of speed it gave the body, m/s.
    pub delta_v: f32,
    /// What it was spread over, m: the pair's give, and the limbs' if the
    /// body was braced for it.
    pub give: f32,
    /// Whether the body met it braced.
    pub braced: bool,
    /// The worst band it can fall in, however sudden: deadly for a hit, and
    /// for a blast's shove what the body's harm allows
    /// (`HarmConfig::blast_at_worst`).
    pub at_worst: Band,
}

impl Blow {
    /// The blow `hit` dealt `body`, which weighs `mass` and was moving at
    /// `velocity` before it. A body in control braces for a blow it moved
    /// into at least as fast as the other side came: the legs take a
    /// landing, the arms a wall. Anything that came at it faster (a falling
    /// stone, a swinging log) is sudden. `None` if the hit was not on `body`.
    pub fn of_hit(
        hit: &Hit,
        body: RigidBodyHandle,
        mass: f32,
        velocity: Vector3<f32>,
        in_control: bool,
        config: &HarmConfig,
    ) -> Option<Self> {
        let push = hit.side(body)?.push();
        let moved_into = (-velocity.dot(&push)).max(0.0);
        let braced = in_control && moved_into >= 0.5 * hit.approach;
        Some(Self {
            delta_v: hit.delta_v(mass),
            give: hit.give + if braced { config.brace_give } else { 0.0 },
            braced,
            at_worst: Band::Deadly,
        })
    }

    /// The blow a blast's `shove` dealt a body weighing `mass`: all at once,
    /// as sudden as the bands are stated for, and no worse than
    /// `blast_at_worst`.
    pub fn of_shove(shove: &Shove, mass: f32, config: &HarmConfig) -> Self {
        Self {
            delta_v: shove.impulse.norm() / mass,
            give: config.sudden_give,
            braced: false,
            at_worst: config.blast_at_worst,
        }
    }

    /// The change of speed, m/s, that stops as hard over the give the bands
    /// are stated for (`HarmConfig::sudden_give`) as this one did over its
    /// own: `delta_v · √(sudden_give / give)`. The same stop spread over
    /// more give is gentler.
    pub fn sudden(&self, config: &HarmConfig) -> f32 {
        self.delta_v * (config.sudden_give / self.give.max(MIN_GIVE)).sqrt()
    }

    /// The band this blow falls in.
    pub fn band(&self, config: &HarmConfig) -> Band {
        config.band(self.sudden(config)).min(self.at_worst)
    }
}

/// The worst of `blows`: the one in the worst band, and the most sudden of
/// those. A blast's shove capped at heavy is not worse than a deadly hit
/// for being more sudden.
pub fn worst(blows: impl IntoIterator<Item = Blow>, config: &HarmConfig) -> Option<Blow> {
    blows.into_iter().max_by(|a, b| {
        a.band(config)
            .cmp(&b.band(config))
            .then(a.sudden(config).total_cmp(&b.sudden(config)))
    })
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::harm::FLESH_GIVE;
    use crate::physics::{Patch, HARD_GIVE, YIELDING_GIVE};

    const MASS: f32 = 56.0;

    fn body() -> RigidBodyHandle {
        RigidBodyHandle(generational_arena::Index::from_raw_parts(0, 0))
    }

    /// `body` struck from below at `approach` by something static, over
    /// the body's flesh and a surface of `surface_give`.
    fn from_below(approach: f32, surface_give: f32) -> Hit {
        Hit {
            body_a: None,
            collider_a: None,
            body_b: body(),
            collider_b: None,
            point: Point3::origin(),
            normal: Vector3::y(),
            approach,
            restitution: 0.0,
            give: FLESH_GIVE + surface_give,
            bluntness: 0.25,
            effective_mass: MASS,
            side_masses: [f32::INFINITY, MASS],
            backed: [false; 2],
            patch: Patch::Point,
            continues: false,
        }
    }

    fn band_landing(speed: f32, in_control: bool) -> Band {
        let config = HarmConfig::default();
        let hit = from_below(speed, HARD_GIVE);
        let falling = Vector3::new(0.0, -speed, 0.0);
        Blow::of_hit(&hit, body(), MASS, falling, in_control, &config)
            .unwrap()
            .band(&config)
    }

    /// The legs take a landing: a full jump's (7 m/s, at which a blow to
    /// the body knocks it down) is light; a fall of about 6 m knocks the
    /// body down and one of about 15 m kills.
    #[test]
    fn a_landing_on_the_feet_is_judged_through_the_legs() {
        assert_eq!(band_landing(7.0, true), Band::Light);
        assert_eq!(band_landing(11.0, true), Band::Heavy);
        assert_eq!(band_landing(17.0, true), Band::Deadly);
    }

    /// Thrown limp, the same landings land on the body.
    #[test]
    fn a_limp_body_lands_unbraced() {
        assert_eq!(band_landing(4.0, false), Band::Light);
        assert_eq!(band_landing(7.0, false), Band::Heavy);
        assert_eq!(band_landing(9.0, false), Band::Deadly);
    }

    /// A blow the body did not move into is sudden however it is met: a
    /// stone dropped on a standing body at 6 m/s knocks it down.
    #[test]
    fn a_blow_that_comes_at_the_body_is_sudden() {
        let config = HarmConfig::default();
        let hit = Hit {
            normal: -Vector3::y(),
            ..from_below(6.0, HARD_GIVE)
        };
        let blow = Blow::of_hit(&hit, body(), MASS, Vector3::zeros(), true, &config).unwrap();
        assert!(!blow.braced);
        assert_eq!(blow.band(&config), Band::Heavy);
    }

    /// Soft ground spreads a stop: a limp landing that knocks the body
    /// down on stone is light on mud.
    #[test]
    fn soft_ground_breaks_a_fall() {
        let config = HarmConfig::default();
        let falling = Vector3::new(0.0, -5.5, 0.0);
        let on = |surface_give| {
            Blow::of_hit(
                &from_below(5.5, surface_give),
                body(),
                MASS,
                falling,
                false,
                &config,
            )
            .unwrap()
            .band(&config)
        };
        assert_eq!(on(HARD_GIVE), Band::Heavy);
        assert_eq!(on(YIELDING_GIVE), Band::Light);
    }

    fn shove(speed: f32) -> Shove {
        Shove {
            body: body(),
            impulse: Vector3::new(speed * MASS, 0.0, 0.0),
            point: Point3::origin(),
        }
    }

    /// A blast's shove is judged at its full change of speed, and knocks the
    /// body down at worst: one as sudden as a deadly hit does not kill.
    #[test]
    fn a_shove_is_as_sudden_as_its_change_of_speed_and_never_kills() {
        let config = HarmConfig::default();
        let band = |speed| Blow::of_shove(&shove(speed), MASS, &config).band(&config);
        assert_eq!(band(4.0), Band::Light);
        assert_eq!(band(6.0), Band::Heavy);
        let close = Blow::of_shove(&shove(12.0), MASS, &config);
        assert_eq!(close.sudden(&config), 12.0);
        assert!(close.sudden(&config) >= config.deadly);
        assert_eq!(close.band(&config), Band::Heavy);
    }

    /// The worst blow of several is the one in the worst band: a deadly
    /// landing in the step a blast shoved the body harder still.
    #[test]
    fn the_worst_blow_is_in_the_worst_band() {
        let config = HarmConfig::default();
        let falling = Vector3::new(0.0, -9.0, 0.0);
        let landing = Blow::of_hit(
            &from_below(9.0, HARD_GIVE),
            body(),
            MASS,
            falling,
            false,
            &config,
        )
        .unwrap();
        let blast = Blow::of_shove(&shove(12.0), MASS, &config);
        let worst = worst([blast, landing], &config).unwrap();
        assert_eq!(worst, landing);
        assert_eq!(worst.band(&config), Band::Deadly);
    }

    #[test]
    fn a_hit_on_another_body_is_no_blow_on_this_one() {
        let other = RigidBodyHandle(generational_arena::Index::from_raw_parts(1, 0));
        let config = HarmConfig::default();
        let hit = from_below(6.0, HARD_GIVE);
        assert!(Blow::of_hit(&hit, other, MASS, Vector3::zeros(), true, &config).is_none());
    }
}
