//! One pod's field: what a suspension's readout looks like, frame by frame.
//!
//! ```text
//!   PodSample ──update──▶ PodField ──look──▶ PodLook
//!   (readout, pose)        (heat, strain,      (column, rings, glow,
//!                           flash, bands,       all in world space)
//!                           ground mark)
//! ```
//!
//! Everything shown is read from the suspension. The smoothing here is only
//! what the eye needs to follow it: heat eases over a few frames, a lost
//! ground fades in a few more, and a load spike leaves a strain that decays.

use nalgebra::{Point3, Vector3};

use crate::physics::SuspensionReadout;

use super::style::FieldStyle;

/// Time constant of the heat following the load, s.
const HEAT_TIME: f32 = 0.06;
/// Time constant of the settled heat a spike is measured against, s.
const SETTLE_TIME: f32 = 0.6;
/// Heat above the settled heat that counts as no strain at all.
const STRAIN_FLOOR: f32 = 0.04;
/// Strain per unit of heat above the floor.
const STRAIN_GAIN: f32 = 5.0;
/// How quickly strain dies away, s.
const STRAIN_TIME: f32 = 0.3;
/// Strain at which a pulse ring is thrown out.
const PULSE_FROM: f32 = 0.35;
/// How quickly a bottoming flash fades, s.
const FLASH_TIME: f32 = 0.15;
/// A pod has bottomed out at this fraction of its force cap...
const BOTTOM_LOAD: f32 = 0.97;
/// ...or pressed in this fraction of its rest length.
const BOTTOM_COMPRESSION: f32 = 0.9;
/// Compression, as a fraction of rest length, that reads as pressed right in.
/// A pod sags a few percent at rest and a landing presses it a quarter of
/// the way, so the look is driven by a multiple of the fraction.
const PRESS_GAIN: f32 = 3.0;
/// Presence of the field with no ground within reach: a dim stub, reaching.
const REACHING_PRESENCE: f32 = 0.22;
/// Length of that stub, as a fraction of the rest length.
const REACHING_LENGTH: f32 = 0.35;
/// How quickly the field and its ground ring come back, and go, s.
const APPEAR_TIME: f32 = 0.03;
const VANISH_TIME: f32 = 0.07;
/// A ring is opaque, so one that has faded to a dim glow is a dark hoop:
/// below this visibility a vanishing ground ring is gone.
const LEAST_VISIBILITY: f32 = 0.35;
/// The least glow a pulse ring is drawn at, for the same reason: just above
/// what blooms.
const LEAST_PULSE_GLOW: f32 = 1.4;
/// Least cosine between the spring's axis and the ground's normal at which
/// the foot is found along the axis; past it, at the hit itself.
const LEAST_FOOT_COSINE: f32 = 0.25;

/// What the visual reads of one suspension, this frame.
#[derive(Debug, Clone, Copy)]
pub struct PodSample {
    /// The suspension's anchor, in world space.
    pub anchor: Point3<f32>,
    /// The way its spring pushes, unit, in world space.
    pub axis: Vector3<f32>,
    /// Its spring's rest length, m.
    pub rest_length: f32,
    /// Its state as of the last step.
    pub readout: SuspensionReadout,
    /// How fast the pod moves across what it stands on, m/s.
    pub ground_speed: f32,
}

/// One frame's look of a pod, in world space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PodLook {
    /// Where the field leaves the pod.
    pub anchor: Point3<f32>,
    /// The way the spring pushes: the column runs against it.
    pub axis: Vector3<f32>,
    /// The field's colour, flash included.
    pub colour: Vector3<f32>,
    /// 0..1, the load as the eye reads it.
    pub heat: f32,
    /// Glow of the ring on the pod's face.
    pub pod_glow: f32,
    /// The column, while there is any field to see.
    pub column: Option<ColumnLook>,
    /// The ring on the ground, while there is ground to put it on.
    pub ground_ring: Option<RingLook>,
    /// A ring thrown outward by a load spike.
    pub pulse_ring: Option<RingLook>,
    /// How fast the pod moves across the ground, m/s.
    pub ground_speed: f32,
}

/// The column from a pod to its ground.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ColumnLook {
    /// From the pod to the ground, along the axis, m.
    pub length: f32,
    /// Radius where it leaves the pod, m.
    pub top_radius: f32,
    /// Radius where it meets the ground, m.
    pub foot_radius: f32,
    /// Multiplier on the colour: the glow's radiance.
    pub radiance: f32,
    /// 0..1: how much field there is to see.
    pub presence: f32,
    /// Distance between bands, m.
    pub band_spacing: f32,
    /// How far the bands have scrolled, in bands, 0..1.
    pub phase: f32,
    /// 0..1: how hard a load spike strains it.
    pub strain: f32,
}

/// A glowing ring lying in a plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RingLook {
    pub centre: Point3<f32>,
    /// The plane's normal.
    pub normal: Vector3<f32>,
    pub radius: f32,
    /// Its glow, as luminance.
    pub glow: f32,
}

/// Where the field last met the ground.
#[derive(Debug, Clone, Copy)]
struct GroundMark {
    /// On the ground, under the pod along its axis.
    foot: Point3<f32>,
    normal: Vector3<f32>,
    /// 0..1, fading when the ground is lost.
    visibility: f32,
}

/// A pod's field, carried from frame to frame.
#[derive(Debug, Clone)]
pub struct PodField {
    /// The load as the eye reads it, eased.
    heat: f32,
    /// The heat the pod has settled at, which a spike stands above.
    settled_heat: f32,
    /// 0..1, left by a load spike.
    strain: f32,
    /// 0..1, left by bottoming out.
    flash: f32,
    /// How far the bands have scrolled, in bands, 0..1.
    phase: f32,
    /// 0..1: how much field there is to see.
    presence: f32,
    /// Spring compression as a fraction of rest length, this frame.
    compression: f32,
    /// The column's length, m.
    length: f32,
    /// Age of the pulse ring, s.
    pulse: Option<f32>,
    /// This frame's brightness under strain, 0..1.
    flicker: f32,
    /// The flicker's random state.
    noise: u32,
    ground: Option<GroundMark>,
    /// The pod's place this frame.
    anchor: Point3<f32>,
    axis: Vector3<f32>,
    ground_speed: f32,
}

impl PodField {
    /// A pod with no field yet. `seed` decorrelates its flicker from its
    /// neighbours'.
    pub fn new(seed: u32) -> Self {
        Self {
            heat: 0.0,
            settled_heat: 0.0,
            strain: 0.0,
            flash: 0.0,
            phase: 0.0,
            presence: 0.0,
            compression: 0.0,
            length: 0.0,
            pulse: None,
            flicker: 1.0,
            noise: seed.wrapping_mul(2_654_435_761).max(1),
            ground: None,
            anchor: Point3::origin(),
            axis: Vector3::y(),
            ground_speed: 0.0,
        }
    }

    /// The heat a pod at `load_fraction` of its cap reads as. The square root
    /// spreads the low end, where a pod spends its life: at rest a pod
    /// carries a few percent of its cap, a landing a third of it.
    pub fn heat_of(load_fraction: f32) -> f32 {
        load_fraction.clamp(0.0, 1.0).sqrt()
    }

    /// Follow this frame's sample, `dt` seconds after the last.
    pub fn update(&mut self, sample: &PodSample, style: &FieldStyle, dt: f32) {
        let readout = &sample.readout;
        self.anchor = sample.anchor;
        self.axis = sample.axis;
        self.ground_speed = sample.ground_speed;

        let grounded = readout.grounded && readout.hit.is_some();
        let target = if grounded {
            Self::heat_of(readout.load_fraction)
        } else {
            0.0
        };
        self.heat = ease(self.heat, target, dt, HEAT_TIME);
        self.settled_heat = ease(self.settled_heat, self.heat, dt, SETTLE_TIME);

        let spike = ((self.heat - self.settled_heat - STRAIN_FLOOR) * STRAIN_GAIN).clamp(0.0, 1.0);
        self.strain = (self.strain * (-dt / STRAIN_TIME).exp()).max(spike);
        self.pulse = self
            .pulse
            .map(|age| age + dt)
            .filter(|&age| age < style.pulse_time);
        let pulse_free = self.pulse.is_none_or(|age| age > 0.6 * style.pulse_time);
        if spike > PULSE_FROM && pulse_free {
            self.pulse = Some(0.0);
        }

        let rest = sample.rest_length.max(1e-3);
        self.compression = if grounded {
            readout.compression / rest
        } else {
            0.0
        };
        let bottomed = grounded
            && (readout.load_fraction >= BOTTOM_LOAD || self.compression >= BOTTOM_COMPRESSION);
        self.flash = if bottomed {
            1.0
        } else {
            self.flash * (-dt / FLASH_TIME).exp()
        };

        self.phase = (self.phase + style.band_rate.at(self.heat) * dt).fract();
        self.noise ^= self.noise << 13;
        self.noise ^= self.noise >> 17;
        self.noise ^= self.noise << 5;
        let random = (self.noise >> 8) as f32 / (1u32 << 24) as f32;
        self.flicker = 1.0 - 0.5 * self.strain * random;

        let foot = readout
            .hit
            .filter(|_| grounded)
            .map(|hit| foot_on(&sample.anchor, &sample.axis, &hit.point, &hit.normal));
        match foot {
            Some((point, depth)) => {
                self.presence = ease(self.presence, 1.0, dt, APPEAR_TIME);
                self.length = depth;
                let visibility = self.ground.map_or(0.0, |mark| mark.visibility);
                self.ground = Some(GroundMark {
                    foot: point,
                    normal: readout.hit.map_or(Vector3::y(), |hit| hit.normal),
                    visibility: ease(visibility, 1.0, dt, APPEAR_TIME),
                });
            }
            None => {
                self.presence = ease(self.presence, REACHING_PRESENCE, dt, VANISH_TIME);
                self.length = ease(self.length, REACHING_LENGTH * rest, dt, VANISH_TIME);
                self.ground = self
                    .ground
                    .map(|mark| GroundMark {
                        visibility: ease(mark.visibility, 0.0, dt, VANISH_TIME),
                        ..mark
                    })
                    .filter(|mark| mark.visibility > LEAST_VISIBILITY);
            }
        }
    }

    /// How the pod looks this frame.
    pub fn look(&self, style: &FieldStyle) -> PodLook {
        let pressed = (self.compression * PRESS_GAIN).clamp(0.0, 1.0);
        let colour = style.colour(self.heat).lerp(&style.hot, self.flash);
        let flash_glow = style.flash_glow * self.flash;

        let column = (self.presence > 0.01 && self.length > 1e-3).then(|| ColumnLook {
            length: self.length,
            top_radius: style.column_radius,
            foot_radius: style.column_radius * style.column_flare.at(pressed),
            radiance: (style.column_radiance.at(self.heat) + flash_glow * 0.3) * self.flicker,
            presence: self.presence,
            band_spacing: style.band_spacing.at(self.heat),
            phase: self.phase,
            strain: self.strain,
        });

        let ground_radius = style.ground_ring_radius.at(pressed);
        let ground_ring = self.ground.map(|mark| RingLook {
            centre: mark.foot,
            normal: mark.normal,
            radius: ground_radius * (0.6 + 0.4 * mark.visibility),
            glow: (style.ground_glow.at(self.heat) + flash_glow) * mark.visibility * self.flicker,
        });

        let pulse_ring = self
            .pulse
            .zip(self.ground)
            .map(|(age, mark)| {
                let t = (age / style.pulse_time).clamp(0.0, 1.0);
                RingLook {
                    centre: mark.foot,
                    normal: mark.normal,
                    radius: ground_radius * (1.0 + style.pulse_reach * t.sqrt()),
                    glow: style.ground_glow.full * (1.0 - t) * mark.visibility,
                }
            })
            .filter(|ring| ring.glow >= LEAST_PULSE_GLOW);

        PodLook {
            anchor: self.anchor,
            axis: self.axis,
            colour,
            heat: self.heat,
            pod_glow: (style.pod_glow.at(self.heat) + flash_glow) * self.flicker,
            column,
            ground_ring,
            pulse_ring,
            ground_speed: self.ground_speed,
        }
    }
}

/// Move `value` toward `target` over `dt`, with time constant `time`.
fn ease(value: f32, target: f32, dt: f32, time: f32) -> f32 {
    value + (target - value) * (1.0 - (-dt / time).exp())
}

/// Where the line down from `anchor` against `axis` meets the ground plane
/// through `point` with `normal`, and how far down it is. At the hit itself
/// where the ground is too steep to the axis for that to be near.
fn foot_on(
    anchor: &Point3<f32>,
    axis: &Vector3<f32>,
    point: &Point3<f32>,
    normal: &Vector3<f32>,
) -> (Point3<f32>, f32) {
    let cosine = axis.dot(normal);
    if cosine < LEAST_FOOT_COSINE {
        let depth = (anchor - point).dot(axis).max(0.0);
        return (*point, depth);
    }
    let depth = ((anchor - point).dot(normal) / cosine).max(0.0);
    (anchor - axis * depth, depth)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::SuspensionHit;

    const DT: f32 = 1.0 / 60.0;
    const REST: f32 = 0.45;

    /// A pod 0.5 m above flat ground carrying `load_fraction` of its cap,
    /// pressed in by `compression`.
    fn on_ground(load_fraction: f32, compression: f32) -> PodSample {
        let anchor = Point3::new(0.0, 0.5, 0.0);
        PodSample {
            anchor,
            axis: Vector3::y(),
            rest_length: REST,
            readout: SuspensionReadout {
                hit: Some(SuspensionHit {
                    point: Point3::new(0.03, 0.0, 0.0),
                    normal: Vector3::y(),
                    partner: None,
                    friction: 0.0,
                    axis: Vector3::y(),
                    anchor,
                    length: REST - compression,
                }),
                grounded: true,
                length: REST - compression,
                compression,
                load: 0.0,
                load_fraction,
                stored_energy: 0.0,
            },
            ground_speed: 0.0,
        }
    }

    fn over_a_gap() -> PodSample {
        PodSample {
            readout: SuspensionReadout {
                hit: None,
                grounded: false,
                length: REST,
                compression: 0.0,
                load: 0.0,
                load_fraction: 0.0,
                stored_energy: 0.0,
            },
            ..on_ground(0.0, 0.0)
        }
    }

    fn run(field: &mut PodField, sample: &PodSample, seconds: f32) {
        for _ in 0..(seconds / DT) as usize {
            field.update(sample, &FieldStyle::HOVER, DT);
        }
    }

    #[test]
    fn the_column_runs_to_the_ground_under_the_pod() {
        let mut field = PodField::new(1);
        run(&mut field, &on_ground(0.04, 0.02), 0.5);
        let look = field.look(&FieldStyle::HOVER);
        let column = look.column.expect("a grounded pod shows its column");
        assert!((column.length - 0.5).abs() < 1e-5, "{}", column.length);
        let ring = look.ground_ring.expect("and its ring on the ground");
        assert!(
            (ring.centre - Point3::origin()).norm() < 1e-5,
            "{:?}",
            ring.centre
        );
    }

    #[test]
    fn a_heavier_load_is_hotter_brighter_and_quicker() {
        let style = FieldStyle::HOVER;
        let mut resting = PodField::new(1);
        let mut loaded = PodField::new(1);
        run(&mut resting, &on_ground(0.04, 0.02), 1.0);
        run(&mut loaded, &on_ground(0.5, 0.12), 1.0);
        let (rest, load) = (resting.look(&style), loaded.look(&style));
        assert!(load.heat > rest.heat + 0.4);
        assert!(load.pod_glow > rest.pod_glow);
        let (rest_column, load_column) = (rest.column.unwrap(), load.column.unwrap());
        assert!(load_column.band_spacing < rest_column.band_spacing);
        assert!(load_column.foot_radius > rest_column.foot_radius);
        assert!(load.ground_ring.unwrap().radius > rest.ground_ring.unwrap().radius);
    }

    #[test]
    fn a_load_spike_strains_the_field_and_throws_a_pulse_that_dies_away() {
        let style = FieldStyle::HOVER;
        let mut field = PodField::new(1);
        run(&mut field, &on_ground(0.04, 0.02), 2.0);
        assert!(field.look(&style).pulse_ring.is_none());
        assert!(field.strain < 0.01, "settled: {}", field.strain);

        run(&mut field, &on_ground(0.4, 0.15), 0.1);
        assert!(field.strain > 0.5, "spiked: {}", field.strain);
        assert!(field.look(&style).pulse_ring.is_some());

        run(&mut field, &on_ground(0.4, 0.15), 2.0);
        assert!(field.strain < 0.05, "settled again: {}", field.strain);
        assert!(field.look(&style).pulse_ring.is_none());
    }

    #[test]
    fn bottoming_out_flashes_white() {
        let style = FieldStyle::HOVER;
        let mut field = PodField::new(1);
        run(&mut field, &on_ground(1.0, 0.3), 0.05);
        let look = field.look(&style);
        assert!((look.colour - style.hot).norm() < 1e-6, "{:?}", look.colour);
        assert_eq!(field.flash, 1.0);
        assert!(look.pod_glow > style.pod_glow.rest + 0.5 * style.flash_glow);

        run(&mut field, &on_ground(0.04, 0.02), 1.0);
        assert!(field.flash < 0.01);
    }

    #[test]
    fn over_a_gap_the_ring_vanishes_and_the_column_shrinks_to_a_stub() {
        let style = FieldStyle::HOVER;
        let mut field = PodField::new(1);
        run(&mut field, &on_ground(0.04, 0.02), 0.5);
        run(&mut field, &over_a_gap(), 0.5);
        let look = field.look(&style);
        assert!(look.ground_ring.is_none());
        let column = look.column.expect("a reaching stub");
        assert!(column.length < 0.5 * REST);
        assert!(column.presence < 0.3);

        run(&mut field, &on_ground(0.04, 0.02), 0.2);
        assert!(
            field.look(&style).ground_ring.is_some(),
            "back on the far plank"
        );
    }

    #[test]
    fn on_a_slope_the_foot_is_on_the_axis_not_at_the_hit() {
        let normal = Vector3::new(0.3, 1.0, 0.0).normalize();
        let (foot, depth) = foot_on(
            &Point3::new(0.0, 1.0, 0.0),
            &Vector3::y(),
            &Point3::new(0.1, 0.0, 0.0),
            &normal,
        );
        assert!((foot.x).abs() < 1e-6 && (foot.z).abs() < 1e-6);
        assert!(((foot - Point3::new(0.1, 0.0, 0.0)).dot(&normal)).abs() < 1e-5);
        assert!((depth - (1.0 - foot.y)).abs() < 1e-5);
    }
}
