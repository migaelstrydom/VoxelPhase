//! Choosing where to stand to look at a level.
//!
//! A viewer is only as useful as its default views. Left to aim the camera by
//! hand every time, the tool costs more attention than reading the numbers did,
//! and the whole reason for having it is that looking should be cheap.
//!
//! So the standard set is derived from the level rather than authored: an
//! overview from two opposite corners so nothing can hide behind anything, one
//! three-quarter view per segment, and the player's own view from the spawn.
//! Between them they answer "does this look the way I meant it to" without a
//! single coordinate being typed.

use nalgebra::{Point3, Vector3};

use crate::collision::AABB;
use crate::level::Level;
use crate::lighting::{ActiveLight, LightId};
use crate::rendering::colour::Colour;
use crate::rendering::visual_bench::{SceneCamera, SceneEnvironment};
use crate::terrain::{Segment, TerrainWorld};

/// Where the sun sits for every standard shot, as a direction towards it.
///
/// Mid-morning and off-axis. High enough that a pit floor is not in permanent
/// shadow, low enough that a two-metre bench casts a shadow long enough to read
/// as a step — which is exactly the feature a level is most likely to be wrong
/// about.
const SUN: Vector3<f32> = Vector3::new(-0.45, 0.78, -0.44);

/// How much room to leave around the subject, as a multiple of the distance
/// that would frame it exactly.
///
/// Terrain is generated with noise, so a segment's meshed extent overshoots its
/// authored bounds slightly. Framing tight enough to clip that is worse than
/// framing loose.
const FRAMING_MARGIN: f32 = 1.08;

/// Eye height above the spawn point for the player's view.
const EYE_HEIGHT: f32 = 1.7;

/// One rendered view of a level.
///
/// Carries no geometry, unlike a `SceneShot`: the content is the level, and
/// what varies between shots is only where the camera is and what light is on
/// it.
pub struct ViewerShot {
    /// Caption for the contact sheet, and the filename stem when shots are
    /// written individually.
    pub label: String,
    pub camera: SceneCamera,
    pub environment: SceneEnvironment,
}

impl ViewerShot {
    pub fn new(label: impl Into<String>, camera: SceneCamera) -> Self {
        Self {
            label: label.into(),
            camera,
            environment: daylight(),
        }
    }
}

/// The lighting every standard shot uses.
///
/// Deliberately one environment for all of them. A viewer is for judging the
/// *level*, and a sheet whose tiles are lit differently invites the difference
/// between two tiles to be read as a difference in the level.
fn daylight() -> SceneEnvironment {
    SceneEnvironment::default().with_sun(SUN)
}

/// The default set of views for a level.
///
/// Ordered so the sheet reads outside-in: the whole level first, then each
/// segment, then the view the player actually starts with.
pub fn standard_shots(level: &Level, terrain: &TerrainWorld, aspect: f32) -> Vec<ViewerShot> {
    let mut shots = Vec::new();

    let whole = content_bounds(terrain).unwrap_or(*terrain.bounds());
    shots.push(ViewerShot::new(
        "overview ne",
        frame(&whole, Vector3::new(1.0, 0.85, 1.0), aspect),
    ));
    shots.push(ViewerShot::new(
        "overview sw",
        frame(&whole, Vector3::new(-1.0, 0.85, -1.0), aspect),
    ));

    for (index, segment) in terrain.segments().iter().enumerate() {
        let name = level
            .segments
            .get(index)
            .map(|def| def.name.clone())
            .unwrap_or_else(|| format!("segment {index}"));
        let Some(bounds) = segment_content_bounds(segment) else {
            continue;
        };
        shots.push(ViewerShot::new(
            name,
            frame(&bounds, Vector3::new(1.0, 0.7, 1.0), aspect),
        ));
    }

    shots.push(spawn_view(level, &whole));
    shots
}

/// The extent of the terrain that actually got meshed.
///
/// Framing against `TerrainWorld::bounds` looks right and is not: that is the
/// union of *allocated chunks*, which reaches a segment's authored floor and
/// ceiling whether or not anything was generated there. A level authored 48 m
/// tall whose surface occupies 14 of them frames as a small thing in a large
/// empty box, and the empty part is sky.
///
/// Returns `None` when nothing was meshed, which is a real answer for a segment
/// that is all air.
fn content_bounds(terrain: &TerrainWorld) -> Option<AABB> {
    extent(
        terrain
            .render_vertices()
            .iter()
            .map(|v| Point3::from(v.pos)),
    )
}

/// The same, for one segment alone.
///
/// Asking the segment for its own geometry rather than filtering the world's by
/// the segment's box, because the boxes overlap: a segment joined above another
/// one contains its neighbour's ground, and framing on that aims the camera at
/// the wrong area entirely.
fn segment_content_bounds(segment: &Segment) -> Option<AABB> {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    segment.append_render_data(&mut vertices, &mut indices);
    extent(vertices.iter().map(|v| Point3::from(v.pos)))
}

fn extent(points: impl Iterator<Item = Point3<f32>>) -> Option<AABB> {
    let mut min = Point3::new(f32::MAX, f32::MAX, f32::MAX);
    let mut max = Point3::new(f32::MIN, f32::MIN, f32::MIN);
    let mut found = false;

    for p in points {
        min = Point3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
        max = Point3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
        found = true;
    }

    found.then(|| AABB::new(min, max))
}

/// A single view from an explicit eye and target, for when a standard shot has
/// shown something worth going in for a closer look at.
pub fn custom_shot(eye: Point3<f32>, target: Point3<f32>) -> ViewerShot {
    ViewerShot::new("custom", SceneCamera::looking_at(eye, target))
}

/// What the player sees on the first frame.
///
/// The one view in the set that is not about inspecting the level from outside.
/// It carries a lamp at the eye, because a spawn inside a cave or an adit is
/// otherwise a black tile, and "did I spawn somewhere pitch dark" is a real
/// question about a level rather than a defect in the shot.
fn spawn_view(level: &Level, whole: &AABB) -> ViewerShot {
    let (x, y, z) = level.player_spawn;
    let eye = Point3::new(x, y + EYE_HEIGHT, z);

    let centre = whole.center();
    let target = Point3::new(centre.x, eye.y, centre.z);

    let mut shot = ViewerShot::new(
        "spawn eye",
        SceneCamera::looking_at(eye, target).with_fov(70.0),
    );
    shot.environment = daylight().with_point_light(ActiveLight {
        id: LightId(0),
        position: eye.coords,
        range: 25.0,
        colour: Colour::new(1.0, 0.95, 0.85, 1.0),
        intensity: 12.0,
    });
    shot
}

/// A three-quarter view that fits `bounds` in frame from direction `from`.
///
/// Framing against the bounding *sphere* is the obvious thing and it leaves
/// pixels on the table, in two ways that compound. A sphere ignores the aspect
/// ratio, so a wide subject is fitted to the *narrow* field of view and stands
/// half again too far back. And it is conservative by construction: no camera
/// angle ever sees the whole diagonal, so the distance it asks for is one no
/// shape actually needs.
///
/// The eight corners are fitted directly instead, which for a box is both exact
/// and closed-form.
///
/// For a corner at `r` from the centre, with the camera at distance `d` along
/// `from`, the depth in front of the camera is `d + r·(-from)` and the lateral
/// offsets are `r·right` and `r·up`. Requiring each to fall inside the frustum
/// gives `d ≥ |lateral| / tan(half fov) + r·from` per corner and per axis, and
/// the largest of those is the answer.
fn frame(bounds: &AABB, from: Vector3<f32>, aspect: f32) -> SceneCamera {
    let centre = bounds.center();
    let direction = from.normalize();
    let forward = -direction;

    // Any camera looking along `forward` shares these axes; `Matrix4::look_at`
    // builds them the same way from world up.
    let right = forward.cross(&Vector3::y()).normalize();
    let up = right.cross(&forward);

    let template = SceneCamera::looking_at(centre, centre);
    let tan_y = (template.fov_y_degrees.to_radians() * 0.5).tan();
    let tan_x = tan_y * aspect;

    let mut distance: f32 = 0.0;
    let mut radius: f32 = 0.0;
    for corner in corners(bounds) {
        let r = corner - centre;
        let behind = r.dot(&direction);
        distance = distance
            .max(r.dot(&right).abs() / tan_x + behind)
            .max(r.dot(&up).abs() / tan_y + behind);
        radius = radius.max(r.norm());
    }
    let distance = distance * FRAMING_MARGIN;

    let eye = centre + direction * distance;
    let mut camera = SceneCamera::looking_at(eye, centre);
    // The far plane defaults to 500 m, which a level of any size sits behind.
    camera.far = (distance + radius * 2.0).max(camera.far);
    camera
}

/// The eight corners of a box.
fn corners(bounds: &AABB) -> [Point3<f32>; 8] {
    let mut out = [Point3::origin(); 8];
    for (i, corner) in out.iter_mut().enumerate() {
        *corner = Point3::new(
            if i & 1 == 0 {
                bounds.min.x
            } else {
                bounds.max.x
            },
            if i & 2 == 0 {
                bounds.min.y
            } else {
                bounds.max.y
            },
            if i & 4 == 0 {
                bounds.min.z
            } else {
                bounds.max.z
            },
        );
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const ASPECT: f32 = 640.0 / 420.0;

    /// Every corner, projected the way the shader will project it, inside the
    /// frustum. This is the contract the whole module exists to keep, and it is
    /// checkable exactly because the subject is a box.
    fn corners_within_frame(bounds: &AABB, camera: &SceneCamera) -> bool {
        let view = camera.view();
        let projection = camera.projection(ASPECT);
        corners(bounds).iter().all(|corner| {
            let clip = projection * view * corner.to_homogeneous();
            clip.w > 0.0
                && clip.x.abs() <= clip.w
                && clip.y.abs() <= clip.w
                && (0.0..=clip.w).contains(&clip.z)
        })
    }

    /// The subject has to be in front of the camera, not behind it. Easy to get
    /// backwards, and a sheet of sky is a slow way to find out.
    #[test]
    fn framing_puts_the_camera_on_the_side_it_was_asked_for() {
        let bounds = AABB::new(Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 4.0, 10.0));
        let camera = frame(&bounds, Vector3::new(1.0, 1.0, 1.0), ASPECT);

        assert!(camera.eye.x > bounds.max.x, "eye at {:?}", camera.eye);
        assert!(camera.eye.y > bounds.max.y, "eye at {:?}", camera.eye);
        assert!(camera.eye.z > bounds.max.z, "eye at {:?}", camera.eye);
    }

    /// A bigger subject has to be further away, or a large level frames as a
    /// close-up of its middle.
    #[test]
    fn framing_distance_grows_with_the_subject() {
        let small = AABB::new(Point3::origin(), Point3::new(10.0, 10.0, 10.0));
        let large = AABB::new(Point3::origin(), Point3::new(100.0, 100.0, 100.0));
        let direction = Vector3::new(1.0, 1.0, 1.0);

        let near = (frame(&small, direction, ASPECT).eye - small.center()).norm();
        let far = (frame(&large, direction, ASPECT).eye - large.center()).norm();
        assert!(far > near * 9.0, "{near} then {far}");
    }

    /// A camera that clips the level it is framing is worse than no camera.
    /// Checked over shapes and angles that break different terms: a flat wide
    /// plain, a deep narrow shaft, a cube, and a level offset from the origin.
    #[test]
    fn the_whole_subject_lands_inside_the_frustum() {
        let cases = [
            AABB::new(Point3::origin(), Point3::new(120.0, 16.0, 120.0)),
            AABB::new(Point3::origin(), Point3::new(8.0, 60.0, 8.0)),
            AABB::new(Point3::origin(), Point3::new(30.0, 30.0, 30.0)),
            AABB::new(
                Point3::new(-200.0, 40.0, 90.0),
                Point3::new(-120.0, 58.0, 190.0),
            ),
        ];
        let directions = [
            Vector3::new(1.0, 0.85, 1.0),
            Vector3::new(-1.0, 0.85, -1.0),
            Vector3::new(1.0, 0.7, 1.0),
            Vector3::new(0.0, 1.0, 0.001),
        ];

        for bounds in &cases {
            for direction in &directions {
                let camera = frame(bounds, *direction, ASPECT);
                assert!(
                    corners_within_frame(bounds, &camera),
                    "{bounds:?} from {direction:?} does not fit"
                );
            }
        }
    }

    /// Fitting must be *tight*, not merely sufficient.
    ///
    /// Standing far enough back always fits, and produces a tile that is mostly
    /// sky — which is the failure that matters, because the pixels are the whole
    /// product. So the subject has to reach the edge of the frame: some corner
    /// must land within the framing margin of it, whatever the shape.
    #[test]
    fn framing_is_tight_rather_than_merely_safe() {
        let shapes = [
            AABB::new(Point3::origin(), Point3::new(160.0, 12.0, 160.0)),
            AABB::new(Point3::origin(), Point3::new(200.0, 50.0, 90.0)),
            AABB::new(Point3::origin(), Point3::new(8.0, 60.0, 8.0)),
        ];

        for bounds in &shapes {
            let camera = frame(bounds, Vector3::new(1.0, 0.85, 1.0), ASPECT);
            let view = camera.view();
            let projection = camera.projection(ASPECT);

            let filled = corners(bounds)
                .iter()
                .map(|corner| {
                    let clip = projection * view * corner.to_homogeneous();
                    (clip.x.abs() / clip.w).max(clip.y.abs() / clip.w)
                })
                .fold(0.0_f32, f32::max);

            assert!(
                filled > 1.0 / FRAMING_MARGIN - 0.05,
                "{bounds:?} fills only {filled} of the frame"
            );
            assert!(filled <= 1.0, "{bounds:?} clips at {filled}");
        }
    }
}
