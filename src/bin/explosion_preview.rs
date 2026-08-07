//! Offline preview of the explosion effect.
//!
//! Runs the real [`ExplosionVisuals`] choreography and the real particle specs
//! headlessly, then draws the result to a PNG contact sheet — one tile per
//! sampled moment — so the look can be judged and tuned without launching the
//! game.
//!
//! ```bash
//! cargo run --bin explosion_preview
//! cargo run --bin explosion_preview -- --radius 8 --out /tmp/big_blast.png
//! ```
//!
//! The simulation is the shipping code: the same emitters, the same
//! `step_emitter`, the same `ParticlePool`. Only the *rasteriser* is a stand-in
//! — a CPU transcription of `shader/particle.vert` and `shader/particle.frag`,
//! kept deliberately literal so that when it disagrees with the game the answer
//! is to fix the transcription. It has no bloom and no scene behind the blast,
//! so it tells the truth about shape, timing and colour, and only approximates
//! brightness.

use std::path::PathBuf;

use image::{Rgb, RgbImage};
use nalgebra::{Vector2, Vector3, Vector4};
use rand::rngs::StdRng;
use rand::SeedableRng;
use voxel_phase::explosion::ExplosionVisuals;
use voxel_phase::lighting::PointLight;
use voxel_phase::particles::{
    step_emitter, EmitterMotion, EmitterState, Particle, ParticleConfig, ParticleEmitter,
    ParticlePool,
};

/// Simulation step. Finer than a game frame so that sub-frame staging (a 0.05s
/// burst-only emitter) is exercised rather than skipped over.
const DT: f32 = 1.0 / 120.0;

/// Moments after detonation to draw, in seconds. Weighted towards the start,
/// where everything that matters happens.
const SAMPLE_TIMES: [f32; 12] = [
    0.0, 0.04, 0.08, 0.14, 0.22, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0, 3.0,
];

/// Tile size in pixels, and the grid they are laid out in.
const TILE: (u32, u32) = (400, 260);
const GRID: (u32, u32) = (4, 3);

/// Camera placement. Far enough back and low enough that the ground reads.
const EYE: Vector3<f32> = Vector3::new(0.0, 2.8, -13.0);
const TARGET: Vector3<f32> = Vector3::new(0.0, 2.4, 0.0);
const FOV_Y_DEGREES: f32 = 45.0;

/// Ambient fill and ground albedo for the stand-in scene.
const AMBIENT: Vector3<f32> = Vector3::new(0.055, 0.06, 0.075);
const GROUND_ALBEDO: Vector3<f32> = Vector3::new(0.26, 0.23, 0.19);

fn main() {
    let options = Options::parse();

    let mut sheet = RgbImage::new(TILE.0 * GRID.0, TILE.1 * GRID.1);
    let frames = simulate(&options);

    for (index, frame) in frames.iter().enumerate() {
        let tile = render(frame);
        let origin_x = (index as u32 % GRID.0) * TILE.0;
        let origin_y = (index as u32 / GRID.0) * TILE.1;

        for y in 0..TILE.1 {
            for x in 0..TILE.0 {
                sheet.put_pixel(origin_x + x, origin_y + y, *tile.get_pixel(x, y));
            }
        }
    }

    sheet.save(&options.out).expect("failed to write preview");
    println!(
        "{} explosion frames ({:?}) -> {}",
        frames.len(),
        SAMPLE_TIMES,
        options.out.display()
    );
    for frame in &frames {
        println!(
            "  t={:>4.2}s  {:>4} particles  light {:>6.2} @ range {:>5.1}",
            frame.time,
            frame.particles.len(),
            frame.light.map(|l| l.intensity).unwrap_or(0.0),
            frame.light.map(|l| l.range).unwrap_or(0.0),
        );
    }
}

struct Options {
    radius: f32,
    out: PathBuf,
    seed: u64,
}

impl Options {
    fn parse() -> Self {
        let mut options = Self {
            radius: 5.0,
            out: PathBuf::from("scratch/explosion_preview.png"),
            seed: 20260807,
        };

        let args: Vec<String> = std::env::args().skip(1).collect();
        let mut index = 0;
        while index < args.len() {
            match args[index].as_str() {
                "--radius" => {
                    options.radius = args[index + 1].parse().expect("bad --radius");
                    index += 2;
                }
                "--out" => {
                    options.out = PathBuf::from(&args[index + 1]);
                    index += 2;
                }
                "--seed" => {
                    options.seed = args[index + 1].parse().expect("bad --seed");
                    index += 2;
                }
                other => panic!("unknown argument {other}"),
            }
        }
        options
    }
}

/// One sampled moment of the explosion.
struct Frame {
    time: f32,
    particles: Vec<Particle>,
    light: Option<PointLight>,
}

/// Run the explosion headlessly, capturing the pool at each sample time.
fn simulate(options: &Options) -> Vec<Frame> {
    let visuals = ExplosionVisuals::default();
    let config = ParticleConfig::default();
    let mut rng = StdRng::seed_from_u64(options.seed);

    // Mirrors what `ExplosionVisuals::spawn` puts in the world, minus the ECS.
    let scale = options.radius / visuals.reference_radius;
    let mut emitters: Vec<ParticleEmitter> = visuals
        .stages
        .iter()
        .map(|stage| stage_emitter(stage, scale))
        .collect();
    let mut light = visuals.light.at_scale(scale);

    let mut pool = ParticlePool::default();
    let origin = Vector3::new(0.0, 0.35, 0.0);
    let motion = EmitterMotion {
        previous: origin,
        current: origin,
        inherited_velocity: Vector3::zeros(),
    };

    let mut frames = Vec::new();
    let mut time = 0.0;
    let mut next_sample = 0;

    while next_sample < SAMPLE_TIMES.len() {
        if time >= SAMPLE_TIMES[next_sample] {
            frames.push(Frame {
                time: SAMPLE_TIMES[next_sample],
                particles: pool.particles().to_vec(),
                light: (!light.is_spent()).then(|| light.current()),
            });
            next_sample += 1;
            continue;
        }

        emitters.retain_mut(|emitter| {
            step_emitter(
                emitter,
                config.spec(emitter.effect_type),
                DT,
                motion,
                &mut pool,
                &mut rng,
            ) == EmitterState::Running
        });
        pool.update(DT, config.gravity);
        light.elapsed += DT;
        time += DT;
    }

    frames
}

/// Rebuild a stage's emitter. `Stage::emitter` is private to the explosion
/// module, so the preview restates it — deliberately, since a tool reaching
/// into private construction would make that construction hard to change.
fn stage_emitter(stage: &voxel_phase::explosion::Stage, scale: f32) -> ParticleEmitter {
    ParticleEmitter::new(stage.effect)
        .with_burst((stage.burst as f32 * scale).round().max(1.0) as u32)
        .with_spawn_rate(stage.rate * scale)
        .with_lifetime(stage.duration)
        .with_delay(stage.delay)
        .with_scale(scale)
}

// ---------------------------------------------------------------------------
// Rasteriser: a CPU transcription of the particle shaders.
// ---------------------------------------------------------------------------

/// Camera basis: right, up, forward.
fn camera_basis() -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    let forward = (TARGET - EYE).normalize();
    let right = forward.cross(&Vector3::y()).normalize();
    let up = right.cross(&forward);
    (right, up, forward)
}

fn render(frame: &Frame) -> RgbImage {
    let (width, height) = TILE;
    let (right, up, forward) = camera_basis();
    let tan_half = (FOV_Y_DEGREES.to_radians() * 0.5).tan();
    let aspect = width as f32 / height as f32;

    // Linear HDR accumulation buffer, seeded with the backdrop.
    let mut buffer = vec![Vector3::zeros(); (width * height) as usize];
    for y in 0..height {
        for x in 0..width {
            let ray = pixel_ray(x, y, width, height, aspect, tan_half, right, up, forward);
            buffer[(y * width + x) as usize] = backdrop(ray, frame.light.as_ref());
        }
    }

    // Particles, furthest first — the same ordering the renderer imposes.
    let mut order: Vec<(f32, usize)> = frame
        .particles
        .iter()
        .enumerate()
        .map(|(index, particle)| (view_depth(particle.position, forward), index))
        .collect();
    order.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());

    for &(depth, index) in &order {
        if depth <= 0.1 {
            continue;
        }
        draw_particle(
            &frame.particles[index],
            depth,
            &mut buffer,
            width,
            height,
            tan_half,
            right,
            up,
        );
    }

    let mut image = RgbImage::new(width, height);
    for y in 0..height {
        for x in 0..width {
            let colour = tonemap_aces(buffer[(y * width + x) as usize]);
            image.put_pixel(
                x,
                y,
                Rgb([
                    encode_srgb(colour.x),
                    encode_srgb(colour.y),
                    encode_srgb(colour.z),
                ]),
            );
        }
    }
    image
}

/// Distance along the view axis, positive in front of the camera.
fn view_depth(position: Vector3<f32>, forward: Vector3<f32>) -> f32 {
    (position - EYE).dot(&forward)
}

#[allow(clippy::too_many_arguments)]
fn pixel_ray(
    x: u32,
    y: u32,
    width: u32,
    height: u32,
    aspect: f32,
    tan_half: f32,
    right: Vector3<f32>,
    up: Vector3<f32>,
    forward: Vector3<f32>,
) -> Vector3<f32> {
    let sx = (2.0 * (x as f32 + 0.5) / width as f32 - 1.0) * aspect * tan_half;
    let sy = (1.0 - 2.0 * (y as f32 + 0.5) / height as f32) * tan_half;
    (right * sx + up * sy + forward).normalize()
}

/// Ground plane and sky, lit only by ambient and the blast.
fn backdrop(ray: Vector3<f32>, light: Option<&PointLight>) -> Vector3<f32> {
    if ray.y >= -1e-4 {
        // A dim night sky, so the flash has something to be brighter than.
        let horizon = (1.0 - ray.y.max(0.0)).powi(3);
        return Vector3::new(0.02, 0.025, 0.04) + Vector3::new(0.03, 0.02, 0.015) * horizon;
    }

    let distance = -EYE.y / ray.y;
    let point = EYE + ray * distance;
    if distance > 90.0 {
        return Vector3::new(0.02, 0.02, 0.03);
    }

    let mut radiance = AMBIENT;
    if let Some(light) = light {
        let light_position = light.world_position(Vector3::zeros());
        let to_light = light_position - point;
        let dist = to_light.magnitude();
        let attenuation = point_attenuation(dist, light.range);
        if attenuation > 0.0 {
            let facing = (to_light / dist).y.max(0.0);
            let colour = Vector3::new(light.colour.r, light.colour.g, light.colour.b);
            radiance += colour * light.radiance_scale() * attenuation * facing;
        }
    }

    // A grid, so the dust skirt's reach is legible against something.
    let grid = if (point.x.rem_euclid(4.0) < 0.06) || (point.z.rem_euclid(4.0) < 0.06) {
        1.6
    } else {
        1.0
    };

    GROUND_ALBEDO.component_mul(&radiance) * grid
}

/// Transcribed from `pointAttenuation` in shader/lighting.glsl.
fn point_attenuation(dist: f32, range: f32) -> f32 {
    if dist >= range || range <= 0.0 {
        return 0.0;
    }
    let ratio = dist / range;
    let window = (1.0 - ratio.powi(4)).max(0.0);
    (window * window) / (1.0 + dist * dist)
}

/// Linear to sRGB, as the sRGB swapchain format does in hardware.
///
/// Easy to forget and impossible to ignore: writing tonemapped linear values
/// straight into an 8-bit PNG darkens every midtone, which would have this tool
/// reporting smoke as pleasantly dark that the game shows as pale grey.
fn encode_srgb(value: f32) -> u8 {
    let encoded = if value <= 0.0031308 {
        value * 12.92
    } else {
        1.055 * value.powf(1.0 / 2.4) - 0.055
    };
    (encoded.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// Transcribed from `tonemapACES` in shader/tonemap.glsl.
fn tonemap_aces(colour: Vector3<f32>) -> Vector3<f32> {
    let curve = |x: f32| ((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14)).clamp(0.0, 1.0);
    Vector3::new(curve(colour.x), curve(colour.y), curve(colour.z))
}

#[allow(clippy::too_many_arguments)]
fn draw_particle(
    particle: &Particle,
    depth: f32,
    buffer: &mut [Vector3<f32>],
    width: u32,
    height: u32,
    tan_half: f32,
    right: Vector3<f32>,
    up: Vector3<f32>,
) {
    // World units to pixels at this depth.
    let pixels_per_unit = (height as f32 * 0.5) / (tan_half * depth);

    let offset = particle.position - EYE;
    let centre = Vector2::new(
        (offset.dot(&right) * pixels_per_unit) + width as f32 * 0.5,
        (-offset.dot(&up) * pixels_per_unit) + height as f32 * 0.5,
    );

    // Billboard axes, matching shader/particle.vert: a smeared particle is
    // drawn along its screen-space motion.
    let motion = particle.velocity * particle.stretch;
    let smear = Vector2::new(motion.dot(&right), -motion.dot(&up)) * pixels_per_unit;
    let smear_length = smear.magnitude();

    let (along, across) = if smear_length > 1e-4 {
        let along = smear / smear_length;
        (along, Vector2::new(along.y, -along.x))
    } else {
        (Vector2::new(0.0, 1.0), Vector2::new(1.0, 0.0))
    };

    let radius = particle.drawn_size() * pixels_per_unit;
    let half_along = radius + smear_length;
    if radius < 0.05 {
        return;
    }

    let extent = half_along.max(radius) + 1.0;
    let min_x = ((centre.x - extent).floor() as i32).max(0);
    let max_x = ((centre.x + extent).ceil() as i32).min(width as i32 - 1);
    let min_y = ((centre.y - extent).floor() as i32).max(0);
    let max_y = ((centre.y + extent).ceil() as i32).min(height as i32 - 1);

    let life = particle.normalized_age();
    let colour = particle.colour;

    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let to_pixel = Vector2::new(x as f32 + 0.5 - centre.x, y as f32 + 0.5 - centre.y);
            // Undo the billboard expansion to recover the quad's local -1..1.
            let local = Vector2::new(
                to_pixel.dot(&across) / radius,
                to_pixel.dot(&along) / half_along,
            );
            if local.x.abs() > 1.0 || local.y.abs() > 1.0 {
                continue;
            }

            let Some(fragment) = shade_fragment(local, colour, life, particle) else {
                continue;
            };

            let index = (y as u32 * width + x as u32) as usize;
            buffer[index] = fragment.premultiplied + buffer[index] * (1.0 - fragment.alpha);
        }
    }
}

struct Fragment {
    /// Colour already multiplied by coverage, as the shader emits it.
    premultiplied: Vector3<f32>,
    /// What the destination is attenuated by — zero for a purely additive
    /// particle.
    alpha: f32,
}

/// Transcribed from shader/particle.frag.
fn shade_fragment(
    local: Vector2<f32>,
    colour: Vector4<f32>,
    life: f32,
    particle: &Particle,
) -> Option<Fragment> {
    let (sin, cos) = particle.angle.sin_cos();
    let p = Vector2::new(cos * local.x - sin * local.y, sin * local.x + cos * local.y);

    let disc = 1.0 - smoothstep(0.25, 1.0, p.magnitude());
    if disc <= 0.0 {
        return None;
    }

    let mask = if particle.billow > 0.0 {
        let noise = fbm(p * 1.9 + Vector2::new(particle.seed * 31.7, particle.seed * 31.7));
        let density = lerp(disc, disc * (0.35 + 1.3 * noise), particle.billow);
        let erosion = particle.billow * life * 0.55;
        ((density - erosion) / (1.0 - erosion).max(0.22)).clamp(0.0, 1.0)
    } else {
        disc
    };

    let alpha = colour.w * mask;
    if alpha < 0.004 {
        return None;
    }

    let glow = colour.x.max(colour.y).max(colour.z).clamp(0.0, 1.0);
    let blend = particle.additive * glow;

    Some(Fragment {
        premultiplied: Vector3::new(colour.x, colour.y, colour.z) * alpha,
        alpha: alpha * (1.0 - blend),
    })
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn lerp(from: f32, to: f32, t: f32) -> f32 {
    from + (to - from) * t
}

fn fract(x: f32) -> f32 {
    x - x.floor()
}

fn hash21(p: Vector2<f32>) -> f32 {
    let mut p = Vector2::new(fract(p.x * 123.34), fract(p.y * 456.21));
    let d = p.dot(&Vector2::new(p.x + 45.32, p.y + 45.32));
    p += Vector2::new(d, d);
    fract(p.x * p.y)
}

fn value_noise(p: Vector2<f32>) -> f32 {
    let cell = Vector2::new(p.x.floor(), p.y.floor());
    let f = Vector2::new(fract(p.x), fract(p.y));
    let weight = Vector2::new(f.x * f.x * (3.0 - 2.0 * f.x), f.y * f.y * (3.0 - 2.0 * f.y));

    let a = hash21(cell);
    let b = hash21(cell + Vector2::new(1.0, 0.0));
    let c = hash21(cell + Vector2::new(0.0, 1.0));
    let d = hash21(cell + Vector2::new(1.0, 1.0));

    lerp(lerp(a, b, weight.x), lerp(c, d, weight.x), weight.y)
}

fn fbm(p: Vector2<f32>) -> f32 {
    0.70 * value_noise(p) + 0.30 * value_noise(p * 2.4 + Vector2::new(17.0, 17.0))
}
