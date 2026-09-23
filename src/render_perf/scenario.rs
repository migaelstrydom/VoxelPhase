use nalgebra::{Point3, Vector3};

/// A grenade left to go off at a spot on the level, watched from a fixed
/// camera.
///
/// ```text
///        eye ●
///             ╲
///              ╲  look
///           ╭───▼───╮
///           │   ●   │  grenade, dropped at the site after `drop_at`
///      ─────┴───────┴─────
/// ```
///
/// The quiet stretch before the drop is the baseline the blast is read
/// against, so it should be long enough to settle whatever the level spawns.
#[derive(Debug, Clone)]
pub struct GrenadeBlast {
    /// World column (x, z) the grenade is dropped in.
    pub site: (f32, f32),
    /// Simulated seconds before the grenade appears.
    pub drop_at: f32,
    /// Height above the terrain surface the grenade starts at.
    pub lift: f32,
    /// Where the camera stands. `None` puts it at `default_eye_offset` from
    /// the site.
    pub eye: Option<Point3<f32>>,
    /// What the camera looks at. `None` looks at the site, a little above
    /// the ground.
    pub look: Option<Point3<f32>>,
}

impl GrenadeBlast {
    /// A player-height view from a follow-camera distance away.
    pub fn default_eye_offset() -> Vector3<f32> {
        Vector3::new(0.0, 4.0, -10.0)
    }

    /// Height above the site's surface the default camera aims at.
    const LOOK_HEIGHT: f32 = 1.0;

    /// The camera's eye and target, given where the site's surface is.
    pub fn camera(&self, surface: Point3<f32>) -> (Point3<f32>, Point3<f32>) {
        let look = self
            .look
            .unwrap_or(surface + Vector3::y() * Self::LOOK_HEIGHT);
        let eye = self.eye.unwrap_or(surface + Self::default_eye_offset());
        (eye, look)
    }
}

impl Default for GrenadeBlast {
    /// The test arena's igloo.
    fn default() -> Self {
        Self {
            site: (12.0, 29.0),
            drop_at: 1.0,
            lift: 0.3,
            eye: None,
            look: None,
        }
    }
}
