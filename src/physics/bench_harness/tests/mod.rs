#![cfg(all(test, feature = "bench_harness"))]

mod assertions;
mod ccd;
mod constraint;
mod dynamic_pairs;
mod many_body;
mod mesh_pipeline;
mod solver;
mod stability;
mod terrain_step;
mod wall_slide;
mod wobble;

use std::fs;

use nalgebra::Point3;

use crate::collision::{MeshPatch, PatchTriangle, Triangle, AABB};
use crate::physics::StaticGeometry;

use super::framework::BenchRunResult;

#[derive(Debug, Clone)]
pub(super) struct CubeShellGeometry {
    pub bounds: AABB,
    patch: MeshPatch,
}

impl CubeShellGeometry {
    pub fn new(half_extent: f32) -> Self {
        let h = half_extent;
        let v000 = Point3::new(-h, -h, -h);
        let v001 = Point3::new(-h, -h, h);
        let v010 = Point3::new(-h, h, -h);
        let v011 = Point3::new(-h, h, h);
        let v100 = Point3::new(h, -h, -h);
        let v101 = Point3::new(h, -h, h);
        let v110 = Point3::new(h, h, -h);
        let v111 = Point3::new(h, h, h);

        let triangles = vec![
            // -X face
            PatchTriangle {
                triangle: Triangle::new(v000, v011, v010),
                neighbors: [None; 3],
            },
            PatchTriangle {
                triangle: Triangle::new(v000, v001, v011),
                neighbors: [None; 3],
            },
            // +X face
            PatchTriangle {
                triangle: Triangle::new(v100, v110, v111),
                neighbors: [None; 3],
            },
            PatchTriangle {
                triangle: Triangle::new(v100, v111, v101),
                neighbors: [None; 3],
            },
            // -Y face
            PatchTriangle {
                triangle: Triangle::new(v000, v100, v101),
                neighbors: [None; 3],
            },
            PatchTriangle {
                triangle: Triangle::new(v000, v101, v001),
                neighbors: [None; 3],
            },
            // +Y face
            PatchTriangle {
                triangle: Triangle::new(v010, v011, v111),
                neighbors: [None; 3],
            },
            PatchTriangle {
                triangle: Triangle::new(v010, v111, v110),
                neighbors: [None; 3],
            },
            // -Z face
            PatchTriangle {
                triangle: Triangle::new(v000, v010, v110),
                neighbors: [None; 3],
            },
            PatchTriangle {
                triangle: Triangle::new(v000, v110, v100),
                neighbors: [None; 3],
            },
            // +Z face
            PatchTriangle {
                triangle: Triangle::new(v001, v101, v111),
                neighbors: [None; 3],
            },
            PatchTriangle {
                triangle: Triangle::new(v001, v111, v011),
                neighbors: [None; 3],
            },
        ];

        Self {
            bounds: AABB::new(Point3::new(-h, -h, -h), Point3::new(h, h, h)),
            patch: MeshPatch { triangles },
        }
    }
}

impl StaticGeometry for CubeShellGeometry {
    fn query_region(&self, aabb: &AABB) -> MeshPatch {
        if self.bounds.intersects(aabb) {
            self.patch.clone()
        } else {
            MeshPatch {
                triangles: Vec::new(),
            }
        }
    }
}

fn write_exports(run: &BenchRunResult, stem: &str) {
    let dir = "target/physics_bench";
    fs::create_dir_all(dir).expect("create target/physics_bench");
    let csv_path = format!("{dir}/{stem}.csv");
    let json_path = format!("{dir}/{stem}.json");
    fs::write(&csv_path, run.to_csv()).expect("write bench csv");
    fs::write(&json_path, run.to_json()).expect("write bench json");
}
