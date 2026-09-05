//! Turning a recorded take into frames you can look at.
//!
//! A filmstrip, not a video. The numbers say a foot stayed down 1.6× too long;
//! the strip says what that looks like — and the two are cut from the same
//! take, so a frame and a number can never be describing different runs.
//!
//! Frames are picked evenly across the take rather than consecutively. A gait
//! cycle at this rig's cadence is a handful of frames long, so consecutive
//! frames would all show the same pose; spreading them over the take shows the
//! gait progressing.
//!
//! ```text
//!   Take ─┬─ frame  ─▶ character mesh ─┐
//!         ├─ ground ─▶ checkered patch ─┼─▶ SceneShot ─▶ VisualBench ─▶ tile
//!         └─ placer ─▶ marker spheres  ─┘
//! ```

use nalgebra::{Matrix4, Point3, Vector3};

use crate::geometry::{generate_sphere_indices, generate_sphere_vertices};
use crate::rendering::colour::Colour;
use crate::rendering::material::SurfaceParams;
use crate::rendering::visual_bench::{SceneCamera, SceneEnvironment, SceneMesh, SceneShot};

use super::ground::{Ground, GroundArea};
use super::support::Carried;
use super::take::{FrameSample, Side, Take};

/// Where the sun sits. Off-axis and fairly low, so a foot that is not quite on
/// the ground casts a shadow with a visible gap under it — which is the whole
/// reason to render a gait rather than plot it.
const SUN: Vector3<f32> = Vector3::new(-0.35, 0.62, -0.70);

/// Half-width of the ground patch drawn around the character, in metres.
const PATCH_AHEAD: f32 = 3.0;

/// Radius of a debug marker sphere.
const MARKER_RADIUS: f32 = 0.028;

/// Where the camera stands relative to the character.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Angle {
    /// Square to the direction of travel. The default, and the only one that
    /// shows stride length honestly.
    Side,
    /// Behind and above, for judging stance width and lateral wobble.
    Behind,
    /// Off one shoulder, for reading the pose as a whole.
    ThreeQuarter,
}

impl Angle {
    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "side" => Some(Angle::Side),
            "behind" => Some(Angle::Behind),
            "three-quarter" | "threequarter" | "tq" => Some(Angle::ThreeQuarter),
            _ => None,
        }
    }

    /// Eye offset from the character, in world axes. Scenarios all travel along
    /// x, so these are fixed rather than derived from facing — a camera that
    /// swings with a turning character makes two tiles incomparable.
    fn offset(self) -> Vector3<f32> {
        match self {
            Angle::Side => Vector3::new(0.0, 0.35, -1.9),
            Angle::Behind => Vector3::new(-2.0, 0.9, 0.0),
            Angle::ThreeQuarter => Vector3::new(-1.3, 0.7, -1.6),
        }
    }
}

/// How to cut a take into a strip.
#[derive(Clone, Copy, Debug)]
pub struct FilmConfig {
    /// How many tiles to render.
    pub tiles: usize,
    /// Draw the placer's anchors, ideal targets and probe hits.
    pub markers: bool,
    pub angle: Angle,
    /// Restrict to a window of the take, as fractions of its duration. Lets a
    /// long scenario be cut down to the transition that matters.
    pub from: f32,
    pub to: f32,
}

impl Default for FilmConfig {
    fn default() -> Self {
        Self {
            tiles: 8,
            markers: true,
            angle: Angle::Side,
            from: 0.0,
            to: 1.0,
        }
    }
}

/// Build one shot per tile.
///
/// Frames without a captured mesh are skipped, so the driver's capture stride
/// bounds how fine a strip can be — ask for more tiles than were captured and
/// you get the ones that exist.
pub fn strip(take: &Take, ground: &dyn Ground, config: &FilmConfig) -> Vec<SceneShot> {
    let candidates: Vec<&FrameSample> = take
        .frames
        .iter()
        .filter(|f| f.mesh.is_some())
        .filter(|f| {
            let t = f.time / take.duration().max(1e-3);
            t >= config.from && t <= config.to
        })
        .collect();

    if candidates.is_empty() {
        return Vec::new();
    }

    pick_evenly(&candidates, config.tiles)
        .into_iter()
        .map(|frame| shot(take, frame, ground, config))
        .collect()
}

fn shot(take: &Take, frame: &FrameSample, ground: &dyn Ground, config: &FilmConfig) -> SceneShot {
    let camera = SceneCamera::looking_at(
        frame.pelvis + config.angle.offset(),
        frame.pelvis + Vector3::new(0.0, -0.18, 0.0),
    )
    .with_fov(42.0);

    let mut shot = SceneShot::new(caption(take, frame), camera)
        .with_environment(SceneEnvironment::default().with_sun(SUN));

    let area = GroundArea::along_x(frame.pelvis.x - PATCH_AHEAD, frame.pelvis.x + PATCH_AHEAD);
    let area = GroundArea {
        z_min: frame.pelvis.z - 2.0,
        z_max: frame.pelvis.z + 2.0,
        ..area
    };
    // Drawn where the floor had got to on this frame, checker and all. On a
    // moving platform that checker is the reference a sliding foot reads
    // against — a foot holding a world position visibly travels across it.
    let ground = Carried::new(ground, frame.support_offset);
    let (vertices, indices) = ground.mesh(area);
    shot = shot.with_mesh(SceneMesh::new(vertices, indices));

    if let Some(mesh) = &frame.mesh {
        shot = shot.with_mesh(SceneMesh::new(mesh.vertices.clone(), mesh.indices.clone()));
    }

    if config.markers {
        shot = shot.with_meshes(markers(frame));
    }

    shot
}

/// Caption carrying everything needed to line a tile up against the report: the
/// time, the beat, and what each foot was doing.
fn caption(take: &Take, frame: &FrameSample) -> String {
    let phase = |planted: bool| if planted { "plant" } else { "swing" };
    format!(
        "{}  t={:.2}s  {}/{}  L:{} R:{}  {:.1} m/s",
        take.scenario,
        frame.time,
        frame.beat,
        frame.pose,
        phase(frame.left.planted),
        phase(frame.right.planted),
        frame.horizontal_speed()
    )
}

/// The placer's intent, drawn.
///
/// Same colour vocabulary as the in-game overlay in `animation::systems`, so
/// what is learned from one reads in the other: yellow anchor, green ideal.
/// Cyan is added for the probe hit, which the in-game overlay does not draw.
fn markers(frame: &FrameSample) -> Vec<SceneMesh> {
    let mut out = Vec::new();
    for side in Side::BOTH {
        let foot = frame.foot(side);
        out.push(marker(foot.anchor, Colour::YELLOW));
        out.push(marker(foot.ideal, Colour::GREEN));
        if let Some(probe) = foot.probe {
            out.push(marker(probe, Colour::new(0.2, 0.9, 1.0, 1.0)));
        }
    }
    out
}

fn marker(at: Point3<f32>, colour: Colour) -> SceneMesh {
    let vertices = generate_sphere_vertices(MARKER_RADIUS, 8, 6, colour);
    let indices = generate_sphere_indices(8, 6);
    SceneMesh::new(vertices, indices)
        .with_transform(Matrix4::new_translation(&at.coords))
        // Emissive so a marker reads as instrumentation rather than as a small
        // ball someone left on the floor.
        .with_surface(SurfaceParams {
            emissive: [colour.r, colour.g, colour.b, 0.8],
            ..SurfaceParams::MATTE
        })
}

/// Pick `count` items spread evenly across `items`, including both ends.
fn pick_evenly<'a, T>(items: &[&'a T], count: usize) -> Vec<&'a T> {
    if count == 0 || items.is_empty() {
        return Vec::new();
    }
    if count >= items.len() {
        return items.to_vec();
    }
    (0..count)
        .map(|i| {
            let position = i as f32 / (count - 1).max(1) as f32;
            items[((items.len() - 1) as f32 * position).round() as usize]
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn even_picks_include_both_ends() {
        let values: Vec<usize> = (0..10).collect();
        let refs: Vec<&usize> = values.iter().collect();
        let picked = pick_evenly(&refs, 4);
        assert_eq!(picked.len(), 4);
        assert_eq!(*picked[0], 0);
        assert_eq!(*picked[3], 9);
    }

    #[test]
    fn asking_for_more_tiles_than_frames_returns_what_exists() {
        let values = vec![1usize, 2, 3];
        let refs: Vec<&usize> = values.iter().collect();
        assert_eq!(pick_evenly(&refs, 10).len(), 3);
    }
}
