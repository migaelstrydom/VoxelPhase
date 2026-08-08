//! Banana spawnable — swept curved mesh with a three-capsule compound collider.

use std::f32::consts::TAU;
use std::sync::Arc;

use nalgebra::{Point3, UnitQuaternion, Vector2, Vector3};
use serde::Deserialize;
use specs::{Builder, Entity, World, WorldExt};

use super::shared::textures::Rgb;
use super::{MaterialCtx, Spawnable};
use crate::components::{
    ModelInstance, Orientation, Position, Renderable, RigidBodyComponent, Velocity,
};
use crate::core::error::EngineResult;
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::physics::{ColliderDesc, RigidBodyDesc};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId};
use crate::rendering::vertex::Vertex;
use crate::systems::PhysicsResource;
use crate::utils::noise::fbm_2d_periodic;

const TEXTURE_SIZE: u32 = 256;

/// Cross-sections along the centreline.
const LENGTH_RINGS: u32 = 32;
/// Vertices around each cross-section.
const RADIAL_SEGMENTS: u32 = 20;

/// Boundaries of the collider capsules in normalised length. The first capsule
/// reaches back over the stem, so it collides without paying for a fourth shape
/// in the narrowphase. The last stops short of the blossom tip: a capsule made
/// to reach the point would have to shrink to nothing, costing far more contact
/// volume along the body than the uncovered sliver is worth.
const COLLIDER_SPLITS: [f32; 4] = [-STEM_SPAN, 0.26, 0.62, 0.9];
/// Safety margin on the fitted capsule radii, covering the error in sampling
/// the tube at a finite number of points.
const COLLIDER_INSET: f32 = 0.97;
/// Samples per capsule used to fit its radius to the tapering, curving tube.
const COLLIDER_FIT_SAMPLES: u32 = 24;
/// How far a capsule's axis is shifted out towards the arc, as a fraction of
/// the arc's bow away from the chord. A half shift splits the worst-case
/// deviation evenly between the middle of the capsule and its ends.
const AXIS_SHIFT: f32 = 0.5;

/// Radius of the blunt blossom tip, as a fraction of `thickness`.
const TIP_RADIUS_FRACTION: f32 = 0.09;
/// Radius of the shoulder the stem grows out of, as a fraction of `thickness`.
/// Much fatter than the blossom end — the body barely narrows before the stem.
const NECK_RADIUS_FRACTION: f32 = 0.34;
/// Exponent of the blossom-end taper. Lower values keep the body fat for longer.
const TAPER_POWER: f32 = 0.32;
/// Fraction of the body over which it swells from the neck to full thickness.
const NECK_SWELL_LENGTH: f32 = 0.4;
/// Depth of the three longitudinal ridges, as a fraction of the local radius.
const RIDGE_AMPLITUDE: f32 = 0.09;
/// Fraction of the body over which the ridges fade in from the neck.
const RIDGE_FADE: f32 = 0.25;

/// Length of the stem, in units of the body's normalised length. The stem
/// occupies `t` in `-STEM_SPAN..0`, continuing the same arc.
const STEM_SPAN: f32 = 0.2;
/// Cross-sections along the stem.
const STEM_RINGS: u32 = 6;
/// Waist depth of the stem, as a fraction of its base radius.
const STEM_WAIST: f32 = 0.24;
/// Flare of the woody cut end, as a fraction of the stem's base radius.
const STEM_FLARE: f32 = 0.28;

#[derive(Deserialize)]
pub struct BananaDef {
    pub pos: (f32, f32, f32),
    /// Arc length of the centreline, in metres.
    #[serde(default = "BananaDef::default_length")]
    pub length: f32,
    /// Cross-section radius at the belly, in metres.
    #[serde(default = "BananaDef::default_thickness")]
    pub thickness: f32,
    /// Total sweep angle of the centreline arc, in radians. Larger is bendier.
    #[serde(default = "BananaDef::default_curvature")]
    pub curvature: f32,
    /// 0 = green and unripe, 1 = deeply spotted and brown.
    #[serde(default = "BananaDef::default_ripeness")]
    pub ripeness: f32,
    #[serde(default = "BananaDef::default_density")]
    pub density: f32,
    #[serde(default = "BananaDef::default_restitution")]
    pub restitution: f32,
    #[serde(default = "BananaDef::default_friction")]
    pub friction: f32,
}

impl BananaDef {
    pub fn default_length() -> f32 {
        0.95
    }
    pub fn default_thickness() -> f32 {
        0.11
    }
    pub fn default_curvature() -> f32 {
        1.5
    }
    pub fn default_ripeness() -> f32 {
        0.4
    }
    /// Fruit is a little denser than water.
    pub fn default_density() -> f32 {
        1050.0
    }
    pub fn default_restitution() -> f32 {
        0.1
    }
    /// Bananas are famously bad at holding onto the floor.
    pub fn default_friction() -> f32 {
        0.15
    }

    fn shape(&self) -> BananaShape {
        BananaShape {
            length: self.length.max(0.05),
            thickness: self.thickness.max(0.005),
            curvature: self.curvature.clamp(0.05, 3.0),
        }
    }
}

impl Spawnable for BananaDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        let pixels = generate_peel_texture(self.ripeness.clamp(0.0, 1.0), rand::random::<u32>());
        let texture = ctx
            .textures
            .create_from_rgba(TEXTURE_SIZE, TEXTURE_SIZE, &pixels, true)?;
        Ok(vec![ctx.materials.register(Material::textured(texture))])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let initial_pos = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let shape = self.shape();

        let (vertices, indices) = shape.build_mesh();
        let parts = vec![ModelPart::new(vec![MeshPrimitive {
            vertices,
            indices,
            material: materials[0],
        }])];
        let model = Arc::new(Model::flat(parts));

        let body_handle = {
            let mut physics = world.write_resource::<PhysicsResource>();

            let body_handle = physics.world.create_body(
                RigidBodyDesc::dynamic()
                    .position(initial_pos)
                    .gravity_scale(1.0)
                    .linear_damping(0.02)
                    .angular_damping(0.01),
            );

            for segment in shape.collider_segments() {
                physics.world.attach_collider(
                    body_handle,
                    ColliderDesc::capsule(segment.half_height, segment.radius)
                        .offset_translation(segment.centre)
                        .offset_rotation(segment.rotation)
                        .density(self.density)
                        .restitution(self.restitution)
                        .friction(self.friction),
                );
            }

            body_handle
        };

        vec![world
            .create_entity()
            .with(Position(initial_pos.coords))
            .with(Velocity(Vector3::zeros()))
            .with(Orientation::default())
            .with(RigidBodyComponent(body_handle))
            .with(ModelInstance::new(model))
            .with(Renderable)
            .build()]
    }
}

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// One capsule of the compound collider, in body-local space.
struct ColliderSegment {
    /// Midpoint of the capsule axis.
    centre: Vector3<f32>,
    /// Rotation taking the capsule's local +Y axis onto the arc tangent.
    rotation: UnitQuaternion<f32>,
    /// Half the capsule length, caps included.
    half_height: f32,
    radius: f32,
}

/// The banana as a circular arc swept by a tapering, three-lobed cross-section.
///
/// The arc lies in the XY plane with its centre of curvature above, so the
/// fruit rests on its belly with both tips pointing up. Body-local origin sits
/// at the midpoint of the arc's vertical extent.
struct BananaShape {
    length: f32,
    thickness: f32,
    curvature: f32,
}

impl BananaShape {
    fn arc_radius(&self) -> f32 {
        self.length / self.curvature
    }

    /// Vertical distance from the belly to the tips.
    fn rise(&self) -> f32 {
        self.arc_radius() * (1.0 - (self.curvature * 0.5).cos())
    }

    /// Angle along the arc at normalised length `t` in 0..1.
    fn arc_angle(&self, t: f32) -> f32 {
        (t - 0.5) * self.curvature
    }

    /// Centreline point at normalised length `t`.
    fn centreline(&self, t: f32) -> Vector3<f32> {
        let a = self.arc_angle(t);
        let r = self.arc_radius();
        Vector3::new(r * a.sin(), r * (1.0 - a.cos()) - self.rise() * 0.5, 0.0)
    }

    /// Unit tangent of the centreline at normalised length `t`.
    fn tangent(&self, t: f32) -> Vector3<f32> {
        let a = self.arc_angle(t);
        Vector3::new(a.cos(), a.sin(), 0.0)
    }

    /// Cross-section radius at normalised length `t`, before ridge modulation.
    ///
    /// Negative `t` is the stem. Both the stem's waist and the body's swell use
    /// smoothstep, which flattens to zero slope at `t = 0`, so the two meet
    /// without a shoulder rather than merely without a step.
    fn radius(&self, t: f32) -> f32 {
        let neck = self.thickness * NECK_RADIUS_FRACTION;

        if t < 0.0 {
            let u = (-t / STEM_SPAN).clamp(0.0, 1.0);
            return neck * (1.0 - STEM_WAIST * smoothstep(u) + STEM_FLARE * u.powi(4));
        }

        // The body swells out of the neck over the first stretch, then holds
        // full thickness until the blossom end tapers to a point.
        let swell =
            NECK_RADIUS_FRACTION + (1.0 - NECK_RADIUS_FRACTION) * smoothstep(t / NECK_SWELL_LENGTH);
        let tip_taper = TIP_RADIUS_FRACTION
            + (1.0 - TIP_RADIUS_FRACTION)
                * (1.0 - (2.0 * t - 1.0).max(0.0).powi(2)).powf(TAPER_POWER);

        self.thickness * swell * tip_taper
    }

    /// Cross-section radius in the ridge valleys — the largest radius that is
    /// inside the surface all the way around.
    fn tube_radius(&self, t: f32) -> f32 {
        self.radius(t) * (1.0 - self.ridge_amplitude(t))
    }

    /// Ridge depth at normalised length `t`. The ridges fade out towards the
    /// neck so the stem is a plain round shaft.
    fn ridge_amplitude(&self, t: f32) -> f32 {
        RIDGE_AMPLITUDE * (t / RIDGE_FADE).clamp(0.0, 1.0)
    }

    /// Texture `v` coordinate for a point at normalised length `t`, mapping the
    /// stem and body together onto 0..1.
    fn texture_v(t: f32) -> f32 {
        (t + STEM_SPAN) / (1.0 + STEM_SPAN)
    }

    /// Surface point at normalised length `t` and cross-section angle `theta`.
    ///
    /// `theta = 0` faces the inside of the curve (the top of a resting banana).
    fn surface(&self, t: f32, theta: f32) -> Vector3<f32> {
        self.centreline(t) + self.rim_offset(t, theta)
    }

    /// Offset from the centreline out to the surface.
    fn rim_offset(&self, t: f32, theta: f32) -> Vector3<f32> {
        let a = self.arc_angle(t);
        let inward = Vector3::new(-a.sin(), a.cos(), 0.0);
        let side = Vector3::z();
        let radius = self.radius(t) * (1.0 + self.ridge_amplitude(t) * (3.0 * theta).cos());
        radius * (theta.cos() * inward + theta.sin() * side)
    }

    /// Outward surface normal, from finite differences of [`Self::surface`].
    fn normal(&self, t: f32, theta: f32) -> Vector3<f32> {
        const DELTA: f32 = 1e-3;
        let t_lo = (t - DELTA).max(-STEM_SPAN);
        let t_hi = (t + DELTA).min(1.0);

        let d_theta = self.surface(t, theta + DELTA) - self.surface(t, theta - DELTA);
        let d_t = self.surface(t_hi, theta) - self.surface(t_lo, theta);

        let normal = d_theta.cross(&d_t);
        if normal.magnitude_squared() < 1e-12 {
            return self.rim_offset(t, theta).normalize();
        }

        let normal = normal.normalize();
        if normal.dot(&self.rim_offset(t, theta)) < 0.0 {
            -normal
        } else {
            normal
        }
    }

    /// Normalised lengths of every cross-section, stem first.
    fn ring_positions() -> Vec<f32> {
        let stem = (0..STEM_RINGS).map(|ring| {
            let u = ring as f32 / STEM_RINGS as f32;
            -STEM_SPAN * (1.0 - u)
        });
        let body = (0..=LENGTH_RINGS).map(|ring| ring as f32 / LENGTH_RINGS as f32);
        stem.chain(body).collect()
    }

    /// Build the swept surface — stem and body — plus flat caps over the cut
    /// end of the stem and the blossom tip.
    ///
    /// Triangles are wound so that `(b - a) x (c - a)` points outward, matching
    /// the engine's clip-space front-face convention.
    fn build_mesh(&self) -> (Vec<Vertex>, Vec<u32>) {
        let rings = Self::ring_positions();
        let ring_verts = RADIAL_SEGMENTS + 1;
        let mut vertices = Vec::with_capacity(rings.len() * ring_verts as usize + 2);
        let mut indices = Vec::new();

        for &t in &rings {
            for seg in 0..=RADIAL_SEGMENTS {
                let u = seg as f32 / RADIAL_SEGMENTS as f32;
                let theta = u * TAU;
                let pos = self.surface(t, theta);
                vertices.push(Vertex {
                    pos: Vector3::new(pos.x, pos.y, pos.z),
                    color: Colour::WHITE.to_vec4(),
                    tex_coords: Vector2::new(u, Self::texture_v(t)),
                    normal: self.normal(t, theta),
                    ao: 1.0,
                });
            }
        }

        for ring in 0..rings.len() as u32 - 1 {
            let row = ring * ring_verts;
            let next_row = (ring + 1) * ring_verts;
            for seg in 0..RADIAL_SEGMENTS {
                let a = row + seg;
                let b = row + seg + 1;
                let c = next_row + seg;
                let d = next_row + seg + 1;
                indices.extend_from_slice(&[a, b, c, b, d, c]);
            }
        }

        self.push_cap(&mut vertices, &mut indices, -STEM_SPAN);
        self.push_cap(&mut vertices, &mut indices, 1.0);

        (vertices, indices)
    }

    /// Add a triangle fan closing the end at normalised length `t`.
    fn push_cap(&self, vertices: &mut Vec<Vertex>, indices: &mut Vec<u32>, t: f32) {
        let outward = if t > 0.5 {
            self.tangent(t)
        } else {
            -self.tangent(t)
        };

        let centre = self.centreline(t);
        let centre_index = vertices.len() as u32;
        vertices.push(Vertex {
            pos: Vector3::new(centre.x, centre.y, centre.z),
            color: Colour::WHITE.to_vec4(),
            tex_coords: Vector2::new(0.5, Self::texture_v(t)),
            normal: outward,
            ao: 1.0,
        });

        let rim_start = vertices.len() as u32;
        for seg in 0..RADIAL_SEGMENTS {
            let u = seg as f32 / RADIAL_SEGMENTS as f32;
            let pos = self.surface(t, u * TAU);
            vertices.push(Vertex {
                pos: Vector3::new(pos.x, pos.y, pos.z),
                color: Colour::WHITE.to_vec4(),
                tex_coords: Vector2::new(u, Self::texture_v(t)),
                normal: outward,
                ao: 1.0,
            });
        }

        // A fan wound around increasing theta faces along +tangent; the stem
        // cap needs the opposite order.
        for seg in 0..RADIAL_SEGMENTS {
            let rim = rim_start + seg;
            let next_rim = rim_start + (seg + 1) % RADIAL_SEGMENTS;
            if t > 0.5 {
                indices.extend_from_slice(&[centre_index, rim, next_rim]);
            } else {
                indices.extend_from_slice(&[centre_index, next_rim, rim]);
            }
        }
    }

    /// Approximate the stem and body with a chain of capsules along the arc.
    fn collider_segments(&self) -> Vec<ColliderSegment> {
        COLLIDER_SPLITS
            .windows(2)
            .map(|split| self.fit_segment(split[0], split[1]))
            .collect()
    }

    /// Fit one capsule to the tube over `t0..t1`.
    ///
    /// A plain chord sits entirely inside the arc, so the curve bows away from
    /// it by up to the full sagitta, and that deviation comes straight out of
    /// the radius budget. Shifting the axis halfway out towards the arc halves
    /// the worst case, at the cost of moving the end caps off the centreline.
    fn fit_segment(&self, t0: f32, t1: f32) -> ColliderSegment {
        let start = self.centreline(t0);
        let end = self.centreline(t1);
        let shift = (self.centreline((t0 + t1) * 0.5) - (start + end) * 0.5) * AXIS_SHIFT;
        let (start, end) = (start + shift, end + shift);

        let radius = self.inscribed_radius(t0, t1, &start, &end);

        // Push the caps out to make up for the shifted axis, so the capsule
        // still reaches the segment's endpoints — the stem's cut end included.
        let chord = (end - start).magnitude();
        let cap_reach = (radius * radius - shift.magnitude_squared())
            .max(0.0)
            .sqrt();

        ColliderSegment {
            centre: (start + end) * 0.5,
            rotation: UnitQuaternion::rotation_between(&Vector3::y(), &(end - start))
                .unwrap_or_else(UnitQuaternion::identity),
            half_height: (chord * 0.5 + radius - cap_reach).max(radius * 1.01),
            radius,
        }
    }

    /// Largest radius that keeps a straight capsule on `start..end` inside the
    /// tube. Two effects squeeze it: the tube narrows towards the tips, and the
    /// chord cuts inside the arc, so both are sampled along the segment.
    fn inscribed_radius(&self, t0: f32, t1: f32, start: &Vector3<f32>, end: &Vector3<f32>) -> f32 {
        (0..=COLLIDER_FIT_SAMPLES)
            .map(|i| {
                let t = t0 + (t1 - t0) * i as f32 / COLLIDER_FIT_SAMPLES as f32;
                let deviation = distance_to_segment(&self.centreline(t), start, end);
                self.tube_radius(t) * COLLIDER_INSET - deviation
            })
            .fold(f32::MAX, f32::min)
            .max(1e-3)
    }
}

/// Hermite ease with zero slope at both ends, clamped to 0..1.
fn smoothstep(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// Shortest distance from `point` to the line segment `start..end`.
fn distance_to_segment(point: &Vector3<f32>, start: &Vector3<f32>, end: &Vector3<f32>) -> f32 {
    let axis = end - start;
    let length_squared = axis.magnitude_squared();
    if length_squared < 1e-12 {
        return (point - start).magnitude();
    }
    let t = ((point - start).dot(&axis) / length_squared).clamp(0.0, 1.0);
    (point - (start + axis * t)).magnitude()
}

// ---------------------------------------------------------------------------
// Procedural peel texture
// ---------------------------------------------------------------------------

/// Peel colouring: yellow body, green when unripe, brown speckles and dark
/// tips as it ripens. `u` wraps around the circumference, `v` runs stem to tip.
fn generate_peel_texture(ripeness: f32, seed: u32) -> Vec<u8> {
    let size = TEXTURE_SIZE;
    let mut pixels = Vec::with_capacity((size * size * 4) as usize);

    let green = Rgb::new(0.55, 0.71, 0.20);
    let yellow = Rgb::new(0.97, 0.82, 0.18);
    let brown = Rgb::new(0.36, 0.21, 0.08);
    let stem = Rgb::new(0.34, 0.29, 0.12);
    let tip = Rgb::new(0.24, 0.15, 0.07);

    // Where the stem band ends, and how far the woody colour bleeds into the
    // peel across the neck.
    let stem_v = BananaShape::texture_v(0.0);
    const NECK_FADE: f32 = 0.12;

    // Spots start appearing part-way through ripening, then spread quickly.
    let spot_threshold = 0.72 - 0.45 * (ripeness - 0.35).max(0.0) / 0.65;

    for y in 0..size {
        for x in 0..size {
            let u = x as f32 / size as f32;
            let v = y as f32 / size as f32;

            let mut colour = green.lerp(yellow, ripeness.clamp(0.0, 1.0).powf(0.6));

            // Longitudinal fibre streaks running stem to tip.
            let streak = fbm_2d_periodic(u * 8.0, v * 2.0, 3, 0.5, 2.0, seed, Some(8));
            colour = colour.scale(0.92 + 0.14 * streak);

            // Blotchy ripening spots.
            let spots = fbm_2d_periodic(
                u * 10.0,
                v * 10.0,
                4,
                0.55,
                2.0,
                seed.wrapping_add(17),
                Some(10),
            );
            if spots > spot_threshold {
                let strength = ((spots - spot_threshold) / (1.0 - spot_threshold)).clamp(0.0, 1.0);
                colour = colour.lerp(brown, strength.powf(0.7) * 0.9);
            }

            // The stem band is woody, fading into the peel across the neck.
            let stem_blend = ((stem_v + NECK_FADE - v) / NECK_FADE).clamp(0.0, 1.0);
            colour = colour.lerp(stem, stem_blend.powf(0.7));

            // The blossom tip and the cut end of the stem both go near-black.
            let end_blend = if v > 0.95 {
                (v - 0.95) / 0.05
            } else {
                ((0.02 - v) / 0.02).max(0.0)
            };
            colour = colour.lerp(tip, end_blend.clamp(0.0, 1.0).powf(0.6));

            colour.write_rgba(&mut pixels);
        }
    }

    pixels
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_shape() -> BananaShape {
        BananaDef {
            pos: (0.0, 0.0, 0.0),
            length: BananaDef::default_length(),
            thickness: BananaDef::default_thickness(),
            curvature: BananaDef::default_curvature(),
            ripeness: BananaDef::default_ripeness(),
            density: BananaDef::default_density(),
            restitution: BananaDef::default_restitution(),
            friction: BananaDef::default_friction(),
        }
        .shape()
    }

    /// Every triangle must wind so its geometric normal agrees with the
    /// shaded vertex normals, or it gets back-face culled.
    #[test]
    fn triangles_face_outward() {
        let shape = test_shape();
        let (vertices, indices) = shape.build_mesh();

        for tri in indices.chunks(3) {
            let [a, b, c] = [tri[0], tri[1], tri[2]].map(|i| vertices[i as usize].pos.xyz());
            let geometric = (b - a).cross(&(c - a));
            assert!(geometric.magnitude() > 1e-9, "degenerate triangle");

            let shaded: Vector3<f32> = tri
                .iter()
                .map(|&i| vertices[i as usize].normal)
                .sum::<Vector3<f32>>();
            assert!(
                geometric.normalize().dot(&shaded.normalize()) > 0.0,
                "triangle wound inward"
            );
        }
    }

    /// The stem must join the neck without a step in radius, and must stay a
    /// slim shaft over its whole length.
    #[test]
    fn stem_profile_is_continuous_and_slim() {
        let shape = test_shape();
        let neck = shape.radius(0.0);

        assert!(
            (shape.radius(-1e-4) - neck).abs() < 1e-4,
            "step at the neck"
        );

        for i in 0..=20 {
            let t = -STEM_SPAN * i as f32 / 20.0;
            let radius = shape.radius(t);
            assert!(
                radius > 0.0 && radius < neck * (1.0 + STEM_FLARE) + 1e-5,
                "stem radius {radius} out of range at t={t}"
            );
        }

        // No shoulder: the profile flattens out on both sides of the junction,
        // so the stem grows out of the body rather than being stuck onto it.
        let slope = |a: f32, b: f32| (shape.radius(b) - shape.radius(a)).abs() / (b - a);
        let gentle = 0.05 * shape.thickness;
        assert!(slope(-2e-3, -1e-3) < gentle, "stem side kinks at the neck");
        assert!(slope(1e-3, 2e-3) < gentle, "body side kinks at the neck");

        // The waist is thinner than both the neck and the flared cut end.
        assert!(shape.radius(-STEM_SPAN * 0.5) < neck);
        assert!(shape.radius(-STEM_SPAN * 0.5) < shape.radius(-STEM_SPAN));
    }

    /// The whole capsule surface — not just its ends — must stay inside the
    /// visual surface. Sampled along the capsule axis, which is a different
    /// discretisation from the one `inscribed_radius` fits against.
    #[test]
    fn colliders_fit_inside_mesh() {
        let shape = test_shape();

        for segment in shape.collider_segments() {
            let axis = segment.rotation * Vector3::y();
            let reach = segment.half_height - segment.radius;

            for i in 0..=40 {
                let along = -reach + 2.0 * reach * i as f32 / 40.0;
                let point = segment.centre + axis * along;

                // Nearest point on the centreline, sampled densely over stem
                // and body alike.
                let (t, distance) = (0..=400)
                    .map(|j| {
                        let t = -STEM_SPAN + (1.0 + STEM_SPAN) * j as f32 / 400.0;
                        (t, (shape.centreline(t) - point).magnitude())
                    })
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .unwrap();

                assert!(
                    distance + segment.radius <= shape.tube_radius(t) + 1e-4,
                    "capsule pokes out near t={t}: {distance} + {} > {}",
                    segment.radius,
                    shape.tube_radius(t)
                );
            }
        }
    }

    /// Every capsule must carry real collision volume — a degenerate one would
    /// let that stretch of the fruit pass through the floor.
    #[test]
    fn colliders_cover_stem_and_body() {
        let shape = test_shape();
        let segments = shape.collider_segments();
        assert_eq!(segments.len(), COLLIDER_SPLITS.len() - 1);

        for segment in &segments {
            assert!(
                segment.radius > 0.05 * shape.thickness,
                "degenerate capsule of radius {}",
                segment.radius
            );
        }

        // The first capsule reaches back along the stem, not just up to it.
        let first = &segments[0];
        let axis = first.rotation * Vector3::y();
        let cap_centre = first.centre - axis * (first.half_height - first.radius);

        let reach = (0..=400)
            .map(|i| {
                let t = -STEM_SPAN + (1.0 + STEM_SPAN) * i as f32 / 400.0;
                (t, (shape.centreline(t) - cap_centre).magnitude())
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap()
            .0;

        assert!(
            reach <= -0.9 * STEM_SPAN,
            "first capsule only reaches t={reach}, leaving the stem bare"
        );
    }
}
