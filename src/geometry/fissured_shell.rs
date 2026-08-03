//! A spherical shell broken open by a network of fissures.
//!
//! The shell is a UV sphere whose quads are sorted into three groups by a
//! procedural crack field:
//!
//! ```text
//!            field value ──▶  0                 gap_width      ember_width
//!                             │                     │               │
//!   quad becomes:             │◀──── missing ──────▶│◀── ember ────▶│◀── crust
//! ```
//!
//! Missing quads are simply not indexed, leaving holes. Whatever is drawn
//! inside the shell — a glowing core — shows through them, which is what makes
//! the object read as a crust over something molten rather than as a painted
//! sphere. Ember quads are the rock immediately around each crack, still solid
//! but hot; crust quads are the cold shell.
//!
//! The two index lists share one vertex array, so the caller decides what
//! material each group is drawn with without the mesh having to know.

use std::f32::consts::{PI, TAU};

use nalgebra::{Vector2, Vector3, Vector4};

use crate::rendering::colour::Colour;
use crate::rendering::vertex::Vertex;

/// One crack band: the set of directions where `sin(phi_freq * phi +
/// theta_freq * theta + phase)` crosses zero.
///
/// A single band draws a family of parallel great-circle-ish cracks. Several
/// bands at different orientations intersect to give a network rather than
/// stripes.
#[derive(Clone, Copy, Debug)]
pub struct FissureBand {
    /// Cracks per turn of longitude. Must be a whole number, or the pattern
    /// will not meet itself where the sphere's seam closes.
    pub phi_freq: f32,

    /// How fast the band drifts with latitude — the shear that stops the
    /// cracks from being plain meridians.
    pub theta_freq: f32,

    /// Offset along the band, in radians. Decorrelates bands that would
    /// otherwise all cross at the poles.
    pub phase: f32,
}

/// Shape of a fissured shell.
#[derive(Clone, Debug)]
pub struct FissuredShellConfig {
    /// Outer radius of the shell.
    pub radius: f32,

    /// Longitude divisions. Also the resolution of the cracks: a crack can be
    /// no narrower than one quad, so a finer shell gives finer cracks.
    pub segments: u32,

    /// Latitude divisions.
    pub rings: u32,

    /// Crack bands whose union forms the fissure network.
    pub bands: Vec<FissureBand>,

    /// Field value below which the shell is missing entirely.
    pub gap_width: f32,

    /// Field value below which the shell is hot rock. Must exceed `gap_width`
    /// for there to be any ember band at all.
    pub ember_width: f32,

    /// Colour of the cold shell, away from every crack.
    pub crust_colour: Colour,

    /// Colour at the lip of a crack, blended towards across the ember band.
    pub ember_colour: Colour,

    /// How far the shell sinks at a crack, as a fraction of `radius`.
    ///
    /// Cold plates stay at full radius and the rock between them dips, so the
    /// silhouette breaks up and the crust reads as plates over something rather
    /// than as a pattern painted on a ball. Only ever inward, so the mesh stays
    /// within the collider it was built for.
    pub plate_relief: f32,
}

impl FissuredShellConfig {
    /// A cooling volcanic crust: dark rock split by a few deep cracks.
    pub fn volcanic(radius: f32) -> Self {
        Self {
            radius,
            segments: 56,
            rings: 36,
            bands: vec![
                FissureBand {
                    phi_freq: 3.0,
                    theta_freq: 2.6,
                    phase: 0.0,
                },
                FissureBand {
                    phi_freq: 2.0,
                    theta_freq: -3.4,
                    phase: 1.7,
                },
                FissureBand {
                    phi_freq: 5.0,
                    theta_freq: 1.1,
                    phase: 3.9,
                },
            ],
            gap_width: 0.04,
            ember_width: 0.14,
            crust_colour: Colour::new(0.10, 0.09, 0.10, 1.0),
            ember_colour: Colour::new(0.85, 0.28, 0.05, 1.0),
            plate_relief: 0.05,
        }
    }
}

/// A generated fissured shell.
///
/// Both index lists address the same `vertices` array; a caller wanting two
/// materials clones the vertices into two primitives.
pub struct FissuredShell {
    /// Every vertex of the underlying UV sphere, coloured by its own heat, so
    /// the crust darkens away from the cracks whichever group it lands in.
    pub vertices: Vec<Vertex>,

    /// Triangles of the cold shell.
    pub crust_indices: Vec<u32>,

    /// Triangles of the hot rock bordering each crack.
    pub ember_indices: Vec<u32>,
}

/// Build a fissured shell.
pub fn generate_fissured_shell(config: &FissuredShellConfig) -> FissuredShell {
    let vertices = build_vertices(config);
    let (crust_indices, ember_indices) = build_indices(config);

    FissuredShell {
        vertices,
        crust_indices,
        ember_indices,
    }
}

/// Vertices of the underlying UV sphere, tinted from `crust_colour` towards
/// `ember_colour`, and sunk by `plate_relief`, according to how close each one
/// sits to a crack.
///
/// Both are sampled per vertex rather than per quad so the heat gradient and
/// the relief are smooth across the surface; the quad grouping only decides
/// which triangles go in which list, not how they are shaded or placed.
///
/// Normals stay radial rather than following the relief. The dip is a few
/// percent of the radius over a wide band, so the error is small, and radial
/// normals keep the shading of a plate continuous with the ember lip beside it
/// instead of creasing along a boundary the cracks already draw.
fn build_vertices(config: &FissuredShellConfig) -> Vec<Vertex> {
    let mut vertices = Vec::with_capacity(((config.rings + 1) * (config.segments + 1)) as usize);

    for ring in 0..=config.rings {
        let theta = ring as f32 * PI / config.rings as f32;
        let (sin_theta, cos_theta) = theta.sin_cos();

        for segment in 0..=config.segments {
            let phi = segment as f32 * TAU / config.segments as f32;
            let (sin_phi, cos_phi) = phi.sin_cos();

            let direction = Vector3::new(sin_theta * cos_phi, cos_theta, sin_theta * sin_phi);

            let heat = heat_at(config, phi, theta);
            let position = direction * config.radius * (1.0 - config.plate_relief * heat);
            let colour = config.crust_colour.lerp(config.ember_colour, heat);

            vertices.push(Vertex {
                pos: Vector4::new(position.x, position.y, position.z, 1.0),
                color: colour.to_vec4(),
                tex_coords: Vector2::new(
                    segment as f32 / config.segments as f32,
                    ring as f32 / config.rings as f32,
                ),
                normal: direction,
            });
        }
    }

    vertices
}

/// Sort every quad of the sphere into the crust list, the ember list, or
/// neither (a hole), by the crack field at the quad's centre.
///
/// Winding matches `generate_sphere_indices`, and the degenerate pole quads are
/// dropped the same way, so a shell can be swapped for a plain sphere without
/// the faces flipping.
fn build_indices(config: &FissuredShellConfig) -> (Vec<u32>, Vec<u32>) {
    let mut crust = Vec::new();
    let mut ember = Vec::new();

    for ring in 0..config.rings {
        // Sample the field at the quad centre: one decision per quad, so a
        // crack edge always falls on a quad boundary and never splits a face.
        let theta = (ring as f32 + 0.5) * PI / config.rings as f32;

        for segment in 0..config.segments {
            let phi = (segment as f32 + 0.5) * TAU / config.segments as f32;
            let field = fissure_field(config, phi, theta);

            if field < config.gap_width {
                continue;
            }

            let target = if field < config.ember_width {
                &mut ember
            } else {
                &mut crust
            };

            let current_ring_start = ring * (config.segments + 1);
            let next_ring_start = (ring + 1) * (config.segments + 1);

            let i0 = current_ring_start + segment;
            let i1 = next_ring_start + segment;
            let i2 = current_ring_start + segment + 1;
            let i3 = next_ring_start + segment + 1;

            // The pole rings collapse to a point, so one triangle of each quad
            // there is degenerate.
            if ring != 0 {
                target.extend_from_slice(&[i0, i2, i1]);
            }
            if ring != config.rings - 1 {
                target.extend_from_slice(&[i2, i3, i1]);
            }
        }
    }

    (crust, ember)
}

/// Distance-like measure to the nearest crack: 0 on a crack centre line, up to
/// 1 at the point furthest from every band.
///
/// Taking the minimum across bands means a point is as hot as its *nearest*
/// crack, which is what makes intersections read as one connected network
/// rather than as bands that happen to overlap.
fn fissure_field(config: &FissuredShellConfig, phi: f32, theta: f32) -> f32 {
    config
        .bands
        .iter()
        .map(|band| {
            (band.phi_freq * phi + band.theta_freq * theta + band.phase)
                .sin()
                .abs()
        })
        .fold(1.0, f32::min)
}

/// Heat at a point: 1 at a crack, falling to 0 by the outer edge of the ember
/// band. Squared so the glow hugs the crack instead of washing over the shell.
fn heat_at(config: &FissuredShellConfig, phi: f32, theta: f32) -> f32 {
    let field = fissure_field(config, phi, theta);
    let falloff = (1.0 - field / config.ember_width.max(1e-6)).clamp(0.0, 1.0);
    falloff * falloff
}

#[cfg(test)]
mod tests {
    use super::*;

    fn shell() -> FissuredShell {
        generate_fissured_shell(&FissuredShellConfig::volcanic(1.0))
    }

    #[test]
    fn the_shell_is_actually_broken_open() {
        let config = FissuredShellConfig::volcanic(1.0);
        let shell = generate_fissured_shell(&config);

        let full_sphere_triangles =
            (config.rings * config.segments * 2 - config.segments * 2) as usize;
        let kept = (shell.crust_indices.len() + shell.ember_indices.len()) / 3;

        assert!(
            kept < full_sphere_triangles,
            "no quads were removed: the shell has no fissures"
        );
        // A shell that has lost most of itself is a cage, not a crust.
        assert!(
            kept > full_sphere_triangles / 2,
            "only {kept} of {full_sphere_triangles} triangles survived"
        );
    }

    #[test]
    fn both_material_groups_are_populated() {
        let shell = shell();
        assert!(!shell.crust_indices.is_empty(), "no cold crust");
        assert!(!shell.ember_indices.is_empty(), "no ember band");
    }

    #[test]
    fn every_index_addresses_the_shared_vertex_array() {
        let shell = shell();
        let count = shell.vertices.len() as u32;
        for index in shell.crust_indices.iter().chain(shell.ember_indices.iter()) {
            assert!(*index < count, "index {index} outside {count} vertices");
        }
    }

    #[test]
    fn relief_only_ever_sinks_the_shell_inwards() {
        // The shell is built to match a collider of the same radius, so a
        // vertex outside it would poke through what the physics thinks is the
        // surface. Relief must dip, never bulge.
        let config = FissuredShellConfig::volcanic(1.0);
        let shell = generate_fissured_shell(&config);

        let mut deepest: f32 = 1.0;
        for vertex in &shell.vertices {
            let radius = vertex.pos.xyz().magnitude();
            assert!(radius <= config.radius + 1e-5, "vertex at radius {radius}");
            deepest = deepest.min(radius);
        }

        // And it must actually dip somewhere, or the plates have no relief.
        assert!(deepest < config.radius * (1.0 - config.plate_relief * 0.5));
    }

    #[test]
    fn the_crack_pattern_meets_itself_around_the_sphere() {
        // Whole-number longitude frequencies are what make the field at phi and
        // at phi + 2pi agree. Without that the seam shows as a visible scar.
        let config = FissuredShellConfig::volcanic(1.0);
        for band in &config.bands {
            assert_eq!(
                band.phi_freq,
                band.phi_freq.round(),
                "band {band:?} does not wrap"
            );
        }

        for step in 0..64 {
            let theta = step as f32 * PI / 64.0;
            let phi = step as f32 * TAU / 64.0;
            let here = fissure_field(&config, phi, theta);
            let round_the_back = fissure_field(&config, phi + TAU, theta);
            assert!((here - round_the_back).abs() < 1e-4);
        }
    }

    #[test]
    fn heat_peaks_on_a_crack_and_dies_outside_the_ember_band() {
        let config = FissuredShellConfig::volcanic(1.0);

        // Walk a line of longitude and check the extremes exist.
        let mut hottest: f32 = 0.0;
        let mut coldest: f32 = 1.0;
        for step in 0..2000 {
            let phi = step as f32 * TAU / 2000.0;
            let heat = heat_at(&config, phi, PI * 0.5);
            hottest = hottest.max(heat);
            coldest = coldest.min(heat);
        }
        assert!(hottest > 0.9, "cracks never get hot (peak {hottest})");
        assert_eq!(coldest, 0.0, "the whole shell glows");
    }
}
