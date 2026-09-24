//! Trilithon spawnable — neolithic stone gateway.
//!
//! Two upright megaliths with a horizontal lintel resting across the top,
//! in the style of Stonehenge. Each stone is an irregular convex polyhedron
//! (distorted cuboid), drawn as old weathered stone. All three pieces are
//! independent rigid bodies so the structure can be toppled, and each cracks
//! once if it lands hard enough. A single stone material is shared across all
//! pieces.

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use serde::Deserialize;
use specs::{Entity, World};

use super::shared::models::{build_convex_hull, SolidFace};
use super::shared::orientation::Yaw;
use super::shared::textures::seed_from_position;
use super::stone::{StoneBlock, StoneShape, StoneTexture};
use super::{MaterialCtx, Spawnable};
use crate::core::error::EngineResult;
use crate::rendering::material::MaterialId;
use crate::rendering::substance::{self, Substance};

#[derive(Deserialize)]
pub struct TrilithonDef {
    pub pos: (f32, f32, f32),
    /// Rotation about `+Y`, in degrees — which way the doorway faces.
    #[serde(default)]
    pub yaw: f32,
    /// Half-height of each upright stone.
    #[serde(default = "TrilithonDef::default_upright_half_height")]
    pub upright_half_height: f32,
    /// Half-width (X) of each upright stone.
    #[serde(default = "TrilithonDef::default_upright_half_width")]
    pub upright_half_width: f32,
    /// Half-depth (Z) of each upright stone.
    #[serde(default = "TrilithonDef::default_upright_half_depth")]
    pub upright_half_depth: f32,
    /// Distance between the inner faces of the two uprights.
    #[serde(default = "TrilithonDef::default_gap")]
    pub gap: f32,
    /// Half-thickness (Y) of the lintel.
    #[serde(default = "TrilithonDef::default_lintel_half_thickness")]
    pub lintel_half_thickness: f32,
    /// How far the lintel overhangs past each upright (X direction).
    #[serde(default = "TrilithonDef::default_lintel_overhang")]
    pub lintel_overhang: f32,
    /// Stone density (kg/m^3). Default is granite's.
    #[serde(default = "TrilithonDef::default_density")]
    pub density: f32,
}

impl TrilithonDef {
    pub fn default_upright_half_height() -> f32 {
        1.2
    }
    pub fn default_upright_half_width() -> f32 {
        0.3
    }
    pub fn default_upright_half_depth() -> f32 {
        0.4
    }
    pub fn default_gap() -> f32 {
        1.0
    }
    pub fn default_lintel_half_thickness() -> f32 {
        0.25
    }
    pub fn default_lintel_overhang() -> f32 {
        1.0
    }
    pub fn default_density() -> f32 {
        2700.0
    }

    /// The one declaration of what this object is made of. The collider takes
    /// the coefficients and the material takes the finish and grain, so the two
    /// cannot drift apart.
    ///
    /// Sarsen, with the density left authored per instance.
    fn substance(&self) -> Substance {
        substance::SARSEN.with_density(self.density)
    }
}

impl Spawnable for TrilithonDef {
    fn material_count(&self) -> usize {
        1
    }

    fn create_materials(&self, ctx: &mut MaterialCtx) -> EngineResult<Vec<MaterialId>> {
        // A seed per trilithon, so two in a level are not the same rock twice.
        let texture = StoneTexture::WEATHERED.with_seed(seed_from_position(self.pos, 0));
        Ok(vec![texture.material(ctx, &self.substance())?])
    }

    fn spawn(&self, world: &mut World, materials: &[MaterialId]) -> Vec<Entity> {
        let material = materials[0];
        let base = Point3::new(self.pos.0, self.pos.1, self.pos.2);
        let seed = seed_from_position(self.pos, 1);

        let upright_he = Vector3::new(
            self.upright_half_width,
            self.upright_half_height,
            self.upright_half_depth,
        );

        let x_offset = self.gap / 2.0 + self.upright_half_width;
        let upright_y = base.y + self.upright_half_height;

        // The trilithon's opening faces along its own ±Z, so yaw is what puts
        // the doorway where the author meant it.
        let yaw = Yaw::degrees(self.yaw);
        let origin = (base.x, base.y, base.z);
        let left_pos = yaw.place(origin, Vector3::new(-x_offset, upright_y - base.y, 0.0));
        let right_pos = yaw.place(origin, Vector3::new(x_offset, upright_y - base.y, 0.0));

        let lintel_half_x = x_offset + self.upright_half_width + self.lintel_overhang;
        let lintel_he = Vector3::new(
            lintel_half_x,
            self.lintel_half_thickness,
            self.upright_half_depth,
        );
        let lintel_y = base.y + self.upright_half_height * 2.0 + self.lintel_half_thickness;
        let lintel_pos = yaw.place(origin, Vector3::new(0.0, lintel_y - base.y, 0.0));

        // Each stone gets a different seed for unique distortion.
        let stones = [
            (left_pos, distorted_cuboid(upright_he, seed), upright_he),
            (
                right_pos,
                distorted_cuboid(upright_he, seed.wrapping_add(1)),
                upright_he,
            ),
            (
                lintel_pos,
                distorted_cuboid(lintel_he, seed.wrapping_add(2)),
                lintel_he,
            ),
        ];
        let faces = cuboid_faces();

        stones
            .into_iter()
            .map(|(centre, vertices, half_extents)| {
                let thickness = 2.0 * half_extents.min();
                StoneBlock {
                    centre,
                    rotation: yaw.rotation(),
                    shape: StoneShape::Hull(Arc::new(build_convex_hull(&vertices, &faces))),
                    substance: self.substance(),
                    material,
                    uvs: StoneTexture::WEATHERED.uvs(thickness),
                }
                .spawn(world)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Distorted cuboid geometry
// ---------------------------------------------------------------------------

/// Face definitions for a distorted cuboid's 8 vertices. Vertex layout:
///   0: (-x, -y, -z)  1: (+x, -y, -z)  2: (+x, +y, -z)  3: (-x, +y, -z)
///   4: (-x, -y, +z)  5: (+x, -y, +z)  6: (+x, +y, +z)  7: (-x, +y, +z)
fn cuboid_faces() -> Vec<SolidFace> {
    vec![
        // -Z face
        SolidFace {
            vertex_indices: vec![0, 3, 2, 1],
            opposite_vertex: 5,
        },
        // +Z face
        SolidFace {
            vertex_indices: vec![4, 5, 6, 7],
            opposite_vertex: 0,
        },
        // -X face
        SolidFace {
            vertex_indices: vec![0, 4, 7, 3],
            opposite_vertex: 1,
        },
        // +X face
        SolidFace {
            vertex_indices: vec![1, 2, 6, 5],
            opposite_vertex: 0,
        },
        // -Y face
        SolidFace {
            vertex_indices: vec![0, 1, 5, 4],
            opposite_vertex: 2,
        },
        // +Y face
        SolidFace {
            vertex_indices: vec![3, 7, 6, 2],
            opposite_vertex: 0,
        },
    ]
}

/// Generate 8 cuboid vertices with per-vertex random displacement for a
/// rough-hewn stone look. The `distort` fraction is relative to the smallest
/// half-extent so the shape stays convincingly solid.
///
/// Corners move only across the stone, never along Y, and the top and bottom
/// corner of each vertical edge move together: every face then stays a true
/// plane, which a hull built from its face list must have, and the top and
/// bottom stay level for stacking. A face bent out of plane has a normal that
/// its own corners stand proud of — a collider that disagrees with itself,
/// and a drawing that folds where the weathering follows the wrong plane.
fn distorted_cuboid(half_extents: Vector3<f32>, seed: u32) -> Vec<Vector3<f32>> {
    let he = half_extents;
    let distort = he.x.min(he.y).min(he.z) * 0.5;

    let base_verts = [
        Vector3::new(-he.x, -he.y, -he.z), // 0: -Y
        Vector3::new(he.x, -he.y, -he.z),  // 1: -Y
        Vector3::new(he.x, he.y, -he.z),   // 2: +Y
        Vector3::new(-he.x, he.y, -he.z),  // 3: +Y
        Vector3::new(-he.x, -he.y, he.z),  // 4: -Y
        Vector3::new(he.x, -he.y, he.z),   // 5: -Y
        Vector3::new(he.x, he.y, he.z),    // 6: +Y
        Vector3::new(-he.x, he.y, he.z),   // 7: +Y
    ];

    // Vertex pairs sharing the same column (bottom↔top at each corner).
    let column_peer: [usize; 8] = [3, 2, 1, 0, 7, 6, 5, 4];

    base_verts
        .iter()
        .enumerate()
        .map(|(i, v)| {
            let canonical = i.min(column_peer[i]);
            let s = seed.wrapping_add(canonical as u32);
            let dx = hash_float(s, 0) * distort;
            let dz = hash_float(s, 2) * distort;
            Vector3::new(v.x + dx, v.y, v.z + dz)
        })
        .collect()
}

/// Deterministic float in -1..1 from a seed and channel.
fn hash_float(seed: u32, channel: u32) -> f32 {
    let mut h = seed
        .wrapping_mul(2654435761)
        .wrapping_add(channel.wrapping_mul(2246822519));
    h ^= h >> 13;
    h = h.wrapping_mul(1597334677);
    h ^= h >> 16;
    (h as f32 / u32::MAX as f32) * 2.0 - 1.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::app::spawnables::shared::models::PieceHull;
    use crate::app::spawnables::stone::inside_out;
    use crate::app::spawnables::weathered_hull_mesh;

    /// Every stone of a trilithon, however it was distorted, is drawn with no
    /// fold in it.
    #[test]
    fn every_stone_of_a_trilithon_is_drawn_right_side_out() {
        let def = TrilithonDef {
            pos: (15.0, 0.0, -5.0),
            yaw: 0.0,
            upright_half_height: TrilithonDef::default_upright_half_height(),
            upright_half_width: TrilithonDef::default_upright_half_width(),
            upright_half_depth: TrilithonDef::default_upright_half_depth(),
            gap: TrilithonDef::default_gap(),
            lintel_half_thickness: TrilithonDef::default_lintel_half_thickness(),
            lintel_overhang: TrilithonDef::default_lintel_overhang(),
            density: TrilithonDef::default_density(),
        };
        let upright = Vector3::new(
            def.upright_half_width,
            def.upright_half_height,
            def.upright_half_depth,
        );
        let lintel = Vector3::new(2.1, def.lintel_half_thickness, def.upright_half_depth);
        for seed in 0..8u32 {
            for half_extents in [upright, lintel] {
                let vertices = distorted_cuboid(half_extents, seed * 97);
                let hull = build_convex_hull(&vertices, &cuboid_faces());
                let anchor = Vector3::new(15.0, 1.2, -5.0);
                let whole = hull.translated(anchor);
                let (v, idx) = weathered_hull_mesh(
                    &PieceHull::new(&hull, anchor).within(Some(&whole)),
                    StoneTexture::WEATHERED.uvs(2.0 * half_extents.min()),
                );
                assert_eq!(inside_out(&v, &idx), 0, "seed {seed}");
            }
        }
    }
}
