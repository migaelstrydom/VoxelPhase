//! The ground an animation scenario walks over.
//!
//! Deliberately analytic rather than voxel terrain. A gait is judged against
//! the shape of the surface — this slope, that stair rise, the exact x of a
//! ledge lip — and a marching-cubes field cannot be authored to a shape that
//! precisely. It also generates in milliseconds, which matters when the point
//! of the tool is to re-run a scenario after every tuning change.
//!
//! ```text
//!   Ground ──height(x,z)──▶ body sim (where the pelvis rides)
//!         ├─raycast()─────▶ foot probes (what the animator senses)
//!         └─mesh()────────▶ the rendered floor
//! ```
//!
//! All three come from the same function, so what the character senses, what
//! holds it up, and what is drawn cannot disagree — which is exactly the class
//! of bug that makes a foot look like it is floating.

use nalgebra::{Point3, Vector3};

use crate::physics::{ColliderDesc, PhysicsConfig, PhysicsWorld, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;
use crate::sensing::{ProbeHit, ProbeTarget};

/// Spacing of the rendered ground grid, in metres. Also the checker cell size,
/// so the floor carries a visible reference against which a sliding foot reads
/// immediately.
const MESH_STEP: f32 = 0.25;

/// Ray-march step for the default `raycast`. Fine enough that a probe cannot
/// step over a stair nosing, coarse enough to stay cheap.
const MARCH_STEP: f32 = 0.02;

/// Bisection iterations after the march brackets a crossing. 24 halvings of a
/// 2 cm bracket resolve to well under a micron.
const BISECT_ITERATIONS: u32 = 24;

/// A surface a character can walk on.
///
/// The only required method is `height`; everything else has a default derived
/// from it. An implementation is therefore a single closed-form expression,
/// which is what keeps the scenario catalogue readable.
pub trait Ground: Send + Sync {
    /// Short stable name, used in scenario labels and filenames.
    fn name(&self) -> &str;

    /// Surface height at `(x, z)`, or `None` where there is no ground at all —
    /// past a ledge lip, over a gap. `None` is not "height zero": a probe finds
    /// nothing there and the body falls.
    fn height(&self, x: f32, z: f32) -> Option<f32>;

    /// Surface normal from central differences.
    ///
    /// Samples that fall in a void hold the centre height, so the normal at a
    /// lip is the lip's own rather than a cliff face's.
    fn normal(&self, x: f32, z: f32) -> Vector3<f32> {
        let e = 0.05;
        let Some(centre) = self.height(x, z) else {
            return Vector3::y();
        };
        let at = |x: f32, z: f32| self.height(x, z).unwrap_or(centre);
        let dx = (at(x + e, z) - at(x - e, z)) / (2.0 * e);
        let dz = (at(x, z + e) - at(x, z - e)) / (2.0 * e);
        Vector3::new(-dx, 1.0, -dz).normalize()
    }

    /// Cast a ray at the surface, as the sensing system would.
    ///
    /// March until the ray crosses from above the surface to below it, then
    /// bisect. A ray that starts already below the surface reports nothing:
    /// probes are rangefinders, and one that begins inside the floor has no
    /// meaningful answer to give.
    fn raycast(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Option<ProbeHit> {
        let direction = direction.try_normalize(1e-6)?;
        let above = |t: f32| {
            let p = origin + direction * t;
            self.height(p.x, p.z).map(|h| p.y - h)
        };

        let mut previous_t = 0.0;
        let mut previous = above(0.0)?;
        if previous < 0.0 {
            return None;
        }

        let mut t = MARCH_STEP;
        while t <= length {
            if let Some(current) = above(t) {
                if current <= 0.0 {
                    let (mut lo, mut hi) = (previous_t, t);
                    for _ in 0..BISECT_ITERATIONS {
                        let mid = 0.5 * (lo + hi);
                        match above(mid) {
                            Some(v) if v > 0.0 => lo = mid,
                            _ => hi = mid,
                        }
                    }
                    let point = origin + direction * hi;
                    let surface = self.height(point.x, point.z).unwrap_or(point.y);
                    return Some(ProbeHit {
                        t: hi / length.max(1e-6),
                        point: Point3::new(point.x, surface, point.z),
                        normal: self.normal(point.x, point.z),
                    });
                }
                previous = current;
            }
            // A void along the way is skipped rather than treated as a miss:
            // a probe aimed across a gap at ground on the far side should
            // still find it.
            previous_t = t;
            t += MARCH_STEP;
        }
        let _ = previous;
        None
    }

    /// Tessellate the surface over `area` for rendering.
    ///
    /// Quads with any corner in a void are dropped, so a ledge renders with an
    /// actual edge rather than a skirt sloping into nothing.
    fn mesh(&self, area: GroundArea) -> (Vec<Vertex>, Vec<u32>) {
        let columns = ((area.x_max - area.x_min) / MESH_STEP).ceil() as usize;
        let rows = ((area.z_max - area.z_min) / MESH_STEP).ceil() as usize;

        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        for row in 0..rows {
            for column in 0..columns {
                let x0 = area.x_min + column as f32 * MESH_STEP;
                let z0 = area.z_min + row as f32 * MESH_STEP;
                let corners = [
                    (x0, z0),
                    (x0 + MESH_STEP, z0),
                    (x0 + MESH_STEP, z0 + MESH_STEP),
                    (x0, z0 + MESH_STEP),
                ];

                let mut heights = [0.0f32; 4];
                let mut complete = true;
                for (i, (x, z)) in corners.iter().enumerate() {
                    match self.height(*x, *z) {
                        Some(y) => heights[i] = y,
                        None => complete = false,
                    }
                }
                if !complete {
                    continue;
                }

                let colour = if (row + column) % 2 == 0 {
                    area.light
                } else {
                    area.dark
                };

                let base = vertices.len() as u32;
                for (i, (x, z)) in corners.iter().enumerate() {
                    vertices.push(Vertex {
                        pos: Vector3::new(*x, heights[i], *z),
                        color: colour.to_vec4(),
                        tex_coords: nalgebra::Vector2::new(0.0, 0.0),
                        normal: self.normal(*x, *z),
                        ao: 1.0,
                    });
                }
                indices.extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
            }
        }

        (vertices, indices)
    }
}

/// The patch of ground to tessellate for a shot.
#[derive(Clone, Copy, Debug)]
pub struct GroundArea {
    pub x_min: f32,
    pub x_max: f32,
    pub z_min: f32,
    pub z_max: f32,
    /// The two checker colours. A plain floor hides a sliding foot; a checker
    /// makes the slide obvious in a still frame.
    pub light: Colour,
    pub dark: Colour,
}

impl GroundArea {
    /// A patch centred on a walk along x, wide enough to fill a side view.
    pub fn along_x(x_min: f32, x_max: f32) -> Self {
        Self {
            x_min,
            x_max,
            z_min: -2.0,
            z_max: 2.0,
            light: Colour::new(0.62, 0.60, 0.56, 1.0),
            dark: Colour::new(0.46, 0.45, 0.43, 1.0),
        }
    }
}

/// Level ground at a fixed height. The control case: anything wrong here is
/// wrong everywhere.
pub struct Flat {
    pub y: f32,
}

impl Flat {
    pub fn at(y: f32) -> Self {
        Self { y }
    }
}

impl Ground for Flat {
    fn name(&self) -> &str {
        "flat"
    }

    fn height(&self, _x: f32, _z: f32) -> Option<f32> {
        Some(self.y)
    }
}

/// A constant incline along x. Positive `grade` rises in +x, so a character
/// walking +x is going uphill and one walking -x is going downhill.
///
/// `grade` is a tangent, not an angle: 0.18 is about 10°, 0.58 about 30°.
pub struct Slope {
    name: String,
    pub grade: f32,
    pub y0: f32,
}

impl Slope {
    /// A slope of the given angle in degrees, rising in +x when positive.
    pub fn degrees(name: impl Into<String>, degrees: f32) -> Self {
        Self {
            name: name.into(),
            grade: degrees.to_radians().tan(),
            y0: 0.0,
        }
    }
}

impl Ground for Slope {
    fn name(&self) -> &str {
        &self.name
    }

    fn height(&self, x: f32, _z: f32) -> Option<f32> {
        Some(self.y0 + self.grade * x)
    }
}

/// A flat approach that turns into a slope at `start_x`.
///
/// The transition is what a constant slope cannot test: the frame where the
/// body's support plane changes under a planted foot.
pub struct SlopeOnset {
    name: String,
    pub start_x: f32,
    pub grade: f32,
}

impl SlopeOnset {
    pub fn degrees(name: impl Into<String>, start_x: f32, degrees: f32) -> Self {
        Self {
            name: name.into(),
            start_x,
            grade: degrees.to_radians().tan(),
        }
    }
}

impl Ground for SlopeOnset {
    fn name(&self) -> &str {
        &self.name
    }

    fn height(&self, x: f32, _z: f32) -> Option<f32> {
        Some((x - self.start_x).max(0.0) * self.grade)
    }
}

/// A flight of stairs starting at `start_x`, climbing in +x.
///
/// Stairs are the hard case for foot placement: the surface is discontinuous,
/// so a landing target that lands a centimetre short lands a whole rise low.
pub struct Stairs {
    name: String,
    pub start_x: f32,
    pub run: f32,
    pub rise: f32,
}

impl Stairs {
    pub fn new(name: impl Into<String>, start_x: f32, run: f32, rise: f32) -> Self {
        Self {
            name: name.into(),
            start_x,
            run,
            rise,
        }
    }
}

impl Ground for Stairs {
    fn name(&self) -> &str {
        &self.name
    }

    fn height(&self, x: f32, _z: f32) -> Option<f32> {
        let steps = ((x - self.start_x) / self.run).floor().max(0.0);
        Some(steps * self.rise)
    }
}

/// Flat ground that simply stops at `edge_x`. Past the lip there is nothing:
/// probes find no surface and the body falls.
pub struct Ledge {
    name: String,
    pub edge_x: f32,
    pub y: f32,
}

impl Ledge {
    pub fn new(name: impl Into<String>, edge_x: f32) -> Self {
        Self {
            name: name.into(),
            edge_x,
            y: 0.0,
        }
    }
}

impl Ground for Ledge {
    fn name(&self) -> &str {
        &self.name
    }

    fn height(&self, x: f32, _z: f32) -> Option<f32> {
        (x <= self.edge_x).then_some(self.y)
    }
}

/// A single vertical drop of `drop` metres at `edge_x`, with ground on both
/// sides. Unlike `Ledge` the character lands rather than falling out of the
/// scenario, which is what makes the landing and the gait's recovery visible.
pub struct Drop {
    name: String,
    pub edge_x: f32,
    pub drop: f32,
}

impl Drop {
    pub fn new(name: impl Into<String>, edge_x: f32, drop: f32) -> Self {
        Self {
            name: name.into(),
            edge_x,
            drop,
        }
    }
}

impl Ground for Drop {
    fn name(&self) -> &str {
        &self.name
    }

    fn height(&self, x: f32, _z: f32) -> Option<f32> {
        Some(if x <= self.edge_x { 0.0 } else { -self.drop })
    }
}

/// Gently rolling ground. Nothing here is steep enough to challenge the gait's
/// reach; the point is that the surface height under a planted foot is never
/// constant, which is where a foot that tracks the terrain badly shows up.
pub struct Undulating {
    name: String,
    pub amplitude: f32,
    pub wavelength: f32,
}

impl Undulating {
    pub fn new(name: impl Into<String>, amplitude: f32, wavelength: f32) -> Self {
        Self {
            name: name.into(),
            amplitude,
            wavelength,
        }
    }
}

impl Ground for Undulating {
    fn name(&self) -> &str {
        &self.name
    }

    fn height(&self, x: f32, z: f32) -> Option<f32> {
        let k = std::f32::consts::TAU / self.wavelength;
        Some(self.amplitude * ((k * x).sin() + 0.35 * (k * z * 0.7).sin()))
    }
}

/// A raised floor that is a *spawned rigid body* rather than terrain — the
/// temple stylobate, the deck of a platform, a crate laid down as a step.
///
/// Every other ground here is an analytic height field sensed by marching that
/// same field, and that is a fair model of terrain. It is not a fair model of a
/// spawnable, and the two differ exactly where it matters, at the rim:
///
/// ```text
///   height field                    box collider
///   ────────────                    ────────────
///        ___________                     ___________
///   ____/                          _____|
///     ^ central differences          ^ the face is where the box says
///       smear the lip into a         it is; the probe reads the ground
///       near-vertical face, and      a stride below as flat, upward and
///       the placer discards it       perfectly plantable — and out of
///       as "not a floor"             reach of any leg
/// ```
///
/// Those are different paths through the placer, so reproducing what walking
/// about a temple looks like means sensing the floor the way the game does:
/// through `PhysicsWorld`'s own `ProbeTarget`. The apron around the slab stays
/// analytic — it stands in for the terrain the temple was built on.
pub struct Slab {
    name: String,
    /// The physics world the probes are actually cast against, holding one box
    /// body. Built once; nothing in it ever moves.
    world: PhysicsWorld,
    centre_x: f32,
    centre_z: f32,
    half_x: f32,
    half_z: f32,
    /// Height of the slab's top face.
    pub top: f32,
    /// Height of the ground the slab was set down on.
    pub apron: f32,
}

impl Slab {
    /// A slab `top` metres proud of an apron at zero, centred on `(x, z)`.
    pub fn new(
        name: impl Into<String>,
        centre: (f32, f32),
        half_extent: (f32, f32),
        top: f32,
    ) -> Self {
        let apron = 0.0;
        let (centre_x, centre_z) = centre;
        let (half_x, half_z) = half_extent;
        // Thick enough that no probe reaches its underside, and positioned so
        // the top face lands exactly at `top`.
        let half_y = 0.5 * (top - apron) + 0.5;
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::new(
            centre_x,
            top - half_y,
            centre_z,
        )));
        world.attach_collider(
            body,
            ColliderDesc::box_shape(Vector3::new(half_x, half_y, half_z)),
        );

        Self {
            name: name.into(),
            world,
            centre_x,
            centre_z,
            half_x,
            half_z,
            top,
            apron,
        }
    }

    /// Whether `(x, z)` is over the slab's footprint.
    fn over_slab(&self, x: f32, z: f32) -> bool {
        (x - self.centre_x).abs() <= self.half_x && (z - self.centre_z).abs() <= self.half_z
    }
}

impl Ground for Slab {
    fn name(&self) -> &str {
        &self.name
    }

    fn height(&self, x: f32, z: f32) -> Option<f32> {
        Some(if self.over_slab(x, z) {
            self.top
        } else {
            self.apron
        })
    }

    /// The slab answers through the physics engine; the apron is a plane.
    /// Earliest hit wins, which is the same rule `SensorProbeSystem` applies
    /// across its targets.
    fn raycast(
        &self,
        origin: Point3<f32>,
        direction: Vector3<f32>,
        length: f32,
    ) -> Option<ProbeHit> {
        let direction = direction.try_normalize(1e-6)?;
        let on_slab = ProbeTarget::raycast(&self.world, origin, direction, length);
        let on_apron = (direction.y < -1e-6 && origin.y > self.apron)
            .then(|| (self.apron - origin.y) / direction.y)
            .filter(|t| *t <= length)
            .map(|t| {
                let point = origin + direction * t;
                ProbeHit {
                    t: t / length.max(1e-6),
                    point: Point3::new(point.x, self.apron, point.z),
                    normal: Vector3::y(),
                }
            });

        match (on_slab, on_apron) {
            (Some(a), Some(b)) => Some(if a.t <= b.t { a } else { b }),
            (hit, None) | (None, hit) => hit,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raycast_finds_flat_ground_below() {
        let ground = Flat::at(1.0);
        let hit = ground
            .raycast(Point3::new(0.0, 2.0, 0.0), -Vector3::y(), 3.0)
            .expect("a downward ray must find ground below it");
        assert!((hit.point.y - 1.0).abs() < 1e-3);
        assert!((hit.normal - Vector3::y()).magnitude() < 1e-3);
    }

    #[test]
    fn raycast_misses_past_a_ledge() {
        let ground = Ledge::new("ledge", 0.0);
        assert!(ground
            .raycast(Point3::new(1.0, 2.0, 0.0), -Vector3::y(), 3.0)
            .is_none());
    }

    #[test]
    fn raycast_resolves_a_stair_nosing() {
        let ground = Stairs::new("stairs", 0.0, 0.3, 0.18);
        let hit = ground
            .raycast(Point3::new(0.35, 2.0, 0.0), -Vector3::y(), 3.0)
            .expect("ground exists above the first tread");
        assert!((hit.point.y - 0.18).abs() < 1e-3, "got {}", hit.point.y);
    }

    /// The point of the whole type: a probe aimed past the rim reports the
    /// apron as flat, upward and perfectly plantable — which is what a box
    /// collider does and a height field's central differences do not.
    #[test]
    fn a_slab_reports_the_ground_past_its_rim_as_flat_floor() {
        let slab = Slab::new("slab", (0.0, 0.0), (2.0, 2.0), 0.25);

        let on_top = slab
            .raycast(Point3::new(0.0, 1.0, 0.0), -Vector3::y(), 2.0)
            .expect("a probe over the slab must find its top");
        assert!((on_top.point.y - 0.25).abs() < 1e-3, "got {on_top:?}");
        assert!(on_top.normal.y > 0.99);

        // A hip standing on the slab, aiming a stride past the rim.
        let hip = Point3::new(1.9, 0.675, 0.0);
        let aim = Point3::new(2.4, 0.25, 0.0);
        let to_aim = aim - hip;
        let past_rim = slab
            .raycast(hip, to_aim.normalize(), to_aim.magnitude() + 0.425)
            .expect("there is ground past the rim, a whole slab lower");
        assert!(
            (past_rim.point.y - slab.apron).abs() < 1e-3,
            "the probe should find the apron, not the rim: {past_rim:?}"
        );
        assert!(
            past_rim.normal.y > 0.99,
            "and report it as floor, which is what makes it plantable: {:?}",
            past_rim.normal
        );
    }

    #[test]
    fn slope_normal_leans_downhill() {
        let ground = Slope::degrees("up10", 10.0);
        let normal = ground.normal(3.0, 0.0);
        assert!(normal.x < 0.0, "a +x rise tilts its normal towards -x");
        assert!((normal.magnitude() - 1.0).abs() < 1e-4);
    }
}
