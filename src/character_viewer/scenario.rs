//! What a character_viewer run is: a level, a place to start, a script, and
//! somewhere to watch from.

use nalgebra::{Point3, Vector3};

use crate::anim_viewer::Script;

use super::traversal::Station;

/// Where the camera sits for a scenario's filmstrip.
///
/// Always relative to the character, so a swim across the bay stays in frame,
/// and always in world terms rather than the character's own, so a turn reads
/// as a turn rather than as the scenery swinging round.
#[derive(Clone, Copy, Debug)]
pub struct CameraRig {
    /// Compass bearing from the character to the camera, in degrees: 0 looks
    /// from +z, 90 from +x.
    pub azimuth: f32,
    /// Height of the camera's line of sight above the horizontal, in degrees.
    pub elevation: f32,
    /// Distance from the point watched, in metres.
    pub distance: f32,
    /// The point watched, in the world, if not the character: for a
    /// scenario about what the character does to something else.
    pub target: Option<Point3<f32>>,
}

impl CameraRig {
    /// Side on to a character travelling along +x, a little above the water.
    pub fn side() -> Self {
        Self {
            azimuth: 0.0,
            elevation: 12.0,
            distance: 4.0,
            target: None,
        }
    }

    /// Three-quarter view from ahead and to the side of travel along +x.
    pub fn three_quarter() -> Self {
        Self {
            azimuth: 50.0,
            elevation: 20.0,
            distance: 4.0,
            target: None,
        }
    }

    pub fn with_azimuth(mut self, degrees: f32) -> Self {
        self.azimuth = degrees;
        self
    }

    pub fn with_elevation(mut self, degrees: f32) -> Self {
        self.elevation = degrees;
        self
    }

    pub fn with_distance(mut self, metres: f32) -> Self {
        self.distance = metres;
        self
    }

    /// Watching `point` in the world rather than the character.
    pub fn watching(mut self, point: Point3<f32>) -> Self {
        self.target = Some(point);
        self
    }

    /// Camera eye and look-at point for a character whose body is at `body`,
    /// in water whose still level is `surface` if it is in any: round the
    /// character, or round the point it is set to watch.
    ///
    /// The eye is kept clear of the water: a camera that follows a plunging
    /// body under the surface films the underside of the water, not the
    /// character.
    pub fn frame(&self, body: Point3<f32>, surface: Option<f32>) -> (Point3<f32>, Point3<f32>) {
        let (azimuth, elevation) = (self.azimuth.to_radians(), self.elevation.to_radians());
        let offset = Vector3::new(
            azimuth.sin() * elevation.cos(),
            elevation.sin(),
            azimuth.cos() * elevation.cos(),
        ) * self.distance;
        let watched = self.target.unwrap_or(body);
        let mut eye = watched + offset;
        if let Some(surface) = surface {
            eye.y = eye.y.max(surface + EYE_CLEARANCE);
        }
        (eye, watched)
    }
}

/// Least height the camera's eye keeps over the water, in metres.
const EYE_CLEARANCE: f32 = 0.6;

/// One scripted run of the character in a real level.
#[derive(Clone, Debug)]
pub struct Scenario {
    pub name: &'static str,
    /// One line saying what the scenario is for, shown by `--list`.
    pub summary: &'static str,
    /// Level file, relative to the repository root.
    pub level: &'static str,
    /// Where the character's feet start, in x and z. The body is placed
    /// standing on whatever is there, or floating at the surface if that is
    /// water.
    pub start: (f32, f32),
    /// Facing at the start, in radians about +y; 0 faces +z, π/2 faces +x.
    pub yaw: f32,
    pub script: Script,
    pub camera: CameraRig,
    /// What the run is judged on.
    pub judged: Judged,
}

/// What a scenario's run is judged on, beyond its frame-by-frame report.
#[derive(Clone, Copy, Debug)]
pub enum Judged {
    /// Nothing more: how the body moved.
    Motion,
    /// How it got across a traversal station.
    Crossing(Station),
    /// How the plan kept up with a steady turn ([`TurnSummary`]), the run-up
    /// to it along `axis`, a horizontal unit direction.
    ///
    /// [`TurnSummary`]: super::turning::TurnSummary
    Turn { axis: Vector3<f32> },
}

impl Judged {
    /// The station crossed, if the run crosses one.
    pub fn station(&self) -> Option<Station> {
        match self {
            Judged::Crossing(station) => Some(*station),
            _ => None,
        }
    }

    /// The way the run sets off, if it is judged on more than its motion.
    fn axis(&self) -> Option<Vector3<f32>> {
        match self {
            Judged::Motion => None,
            Judged::Crossing(station) => Some(station.axis),
            Judged::Turn { axis } => Some(*axis),
        }
    }
}

impl Scenario {
    /// The scenario started `back` metres further back along the way it sets
    /// off, if it is judged on more than its motion: the same crossing or
    /// turn met at another phase of the frames and the stride.
    pub fn started_back(&self, back: f32) -> Scenario {
        let Some(axis) = self.judged.axis() else {
            return self.clone();
        };
        let (x, z) = self.start;
        Scenario {
            start: (x - back * axis.x, z - back * axis.z),
            ..self.clone()
        }
    }
}
