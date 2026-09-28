use nalgebra::{Point3, Vector3};

use crate::collision::{MeshPatch, PatchTriangle, SurfaceId, Triangle, AABB};
use crate::physics::StaticGeometry;

// ═══════════════════════════════════════════════════════════════════════════
// Geometry: static terrain shapes used by scenarios
// ═══════════════════════════════════════════════════════════════════════════

/// Empty geometry with no triangles. Used for scenarios that don't need
/// terrain (e.g. zero-gravity constraint tests).
#[derive(Debug, Clone)]
pub struct EmptyGeometry;

impl StaticGeometry for EmptyGeometry {
    fn query_region(&self, _aabb: &AABB) -> MeshPatch {
        MeshPatch {
            triangles: Vec::new(),
        }
    }
}

/// Flat quad at y=0. Two triangles forming a square.
#[derive(Debug, Clone)]
pub struct FlatQuadGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl FlatQuadGeometry {
    pub fn new(half_size: f32) -> Self {
        let y = 0.0f32;
        let v0 = Point3::new(-half_size, y, -half_size);
        let v1 = Point3::new(half_size, y, -half_size);
        let v2 = Point3::new(half_size, y, half_size);
        let v3 = Point3::new(-half_size, y, half_size);
        let tri_a = PatchTriangle {
            triangle: Triangle::new(v0, v2, v1),
            neighbors: [Some(1), None, None],
            surface: SurfaceId::UNSPECIFIED,
        };
        let tri_b = PatchTriangle {
            triangle: Triangle::new(v0, v3, v2),
            neighbors: [None, None, Some(0)],
            surface: SurfaceId::UNSPECIFIED,
        };
        Self {
            bounds: AABB::new(
                Point3::new(-half_size, -0.01, -half_size),
                Point3::new(half_size, 0.01, half_size),
            ),
            patch: MeshPatch {
                triangles: vec![tri_a, tri_b],
            },
        }
    }
}

impl StaticGeometry for FlatQuadGeometry {
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

/// Flat grid at y=0 made of unit-sized squares, each split into two triangles.
///
/// Mimics in-game terrain where a collider straddles many triangle edges and
/// diagonal seams. Each cell is `cell_size × cell_size` with a diagonal from
/// bottom-left to top-right, matching typical MC-style terrain tessellation.
#[derive(Debug, Clone)]
pub struct FlatGridGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl FlatGridGeometry {
    pub fn new(half_size: f32, cell_size: f32) -> Self {
        let y = 0.0f32;
        let cells = ((half_size * 2.0) / cell_size).ceil() as i32;
        let origin = -(cells as f32 * cell_size) / 2.0;
        let mut triangles = Vec::new();

        for iz in 0..cells {
            for ix in 0..cells {
                let x0 = origin + ix as f32 * cell_size;
                let z0 = origin + iz as f32 * cell_size;
                let x1 = x0 + cell_size;
                let z1 = z0 + cell_size;

                let bl = Point3::new(x0, y, z0);
                let br = Point3::new(x1, y, z0);
                let tr = Point3::new(x1, y, z1);
                let tl = Point3::new(x0, y, z1);

                let idx = triangles.len() as u32;
                let tri_a = PatchTriangle {
                    triangle: Triangle::new(bl, tr, br),
                    neighbors: [Some(idx + 1), None, None],
                    surface: SurfaceId::UNSPECIFIED,
                };
                let tri_b = PatchTriangle {
                    triangle: Triangle::new(bl, tl, tr),
                    neighbors: [None, None, Some(idx)],
                    surface: SurfaceId::UNSPECIFIED,
                };
                triangles.push(tri_a);
                triangles.push(tri_b);
            }
        }

        let extent = (cells as f32 * cell_size) / 2.0;
        Self {
            bounds: AABB::new(
                Point3::new(-extent, -0.01, -extent),
                Point3::new(extent, 0.01, extent),
            ),
            patch: MeshPatch { triangles },
        }
    }
}

impl StaticGeometry for FlatGridGeometry {
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

/// Ramp geometry: flat ground (z < 0) transitioning to a 30° upward slope (z >= 0).
///
/// The ramp rises in the +Z direction. The flat section is at y=0.
/// The slope runs from z=0 to z=`run`, reaching height `rise`.
#[derive(Debug, Clone)]
pub struct RampGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl RampGeometry {
    pub fn new(half_width: f32, run: f32, rise: f32) -> Self {
        let w = half_width;
        // Flat section: z from -run to 0, y = 0
        let f0 = Point3::new(-w, 0.0, -run);
        let f1 = Point3::new(w, 0.0, -run);
        let f2 = Point3::new(w, 0.0, 0.0);
        let f3 = Point3::new(-w, 0.0, 0.0);
        // Ramp section: z from 0 to run, y from 0 to rise
        let r0 = Point3::new(-w, rise, run);
        let r1 = Point3::new(w, rise, run);

        // Flat quad: two triangles
        let flat_a = PatchTriangle {
            triangle: Triangle::new(f0, f2, f1),
            neighbors: [Some(1), None, None],
            surface: SurfaceId::UNSPECIFIED,
        };
        let flat_b = PatchTriangle {
            triangle: Triangle::new(f0, f3, f2),
            neighbors: [None, Some(2), Some(0)],
            surface: SurfaceId::UNSPECIFIED,
        };
        // Ramp quad: two triangles sharing the edge f3-f2 with the flat section
        let ramp_a = PatchTriangle {
            triangle: Triangle::new(f3, r1, f2),
            neighbors: [Some(3), None, Some(1)],
            surface: SurfaceId::UNSPECIFIED,
        };
        let ramp_b = PatchTriangle {
            triangle: Triangle::new(f3, r0, r1),
            neighbors: [None, None, Some(2)],
            surface: SurfaceId::UNSPECIFIED,
        };

        Self {
            bounds: AABB::new(
                Point3::new(-w, -0.01, -run),
                Point3::new(w, rise + 0.01, run),
            ),
            patch: MeshPatch {
                triangles: vec![flat_a, flat_b, ramp_a, ramp_b],
            },
        }
    }
}

impl StaticGeometry for RampGeometry {
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

/// Step geometry: two flat levels at different heights.
///
/// Lower level at y=0 (x < 0), upper level at y=`step_height` (x >= 0).
/// The step edge runs along the Z axis.
#[derive(Debug, Clone)]
pub struct StepGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl StepGeometry {
    pub fn new(half_size: f32, step_height: f32) -> Self {
        let s = half_size;
        let h = step_height;
        // Lower level (x < 0): y = 0
        let l0 = Point3::new(-s, 0.0, -s);
        let l1 = Point3::new(0.0, 0.0, -s);
        let l2 = Point3::new(0.0, 0.0, s);
        let l3 = Point3::new(-s, 0.0, s);
        // Upper level (x >= 0): y = step_height
        let u0 = Point3::new(0.0, h, -s);
        let u1 = Point3::new(s, h, -s);
        let u2 = Point3::new(s, h, s);
        let u3 = Point3::new(0.0, h, s);
        // Vertical face (the step wall)
        // l1 (0, 0, -s), u0 (0, h, -s), u3 (0, h, s), l2 (0, 0, s)

        let lower_a = PatchTriangle {
            triangle: Triangle::new(l0, l2, l1),
            neighbors: [Some(1), None, None],
            surface: SurfaceId::UNSPECIFIED,
        };
        let lower_b = PatchTriangle {
            triangle: Triangle::new(l0, l3, l2),
            neighbors: [None, None, Some(0)],
            surface: SurfaceId::UNSPECIFIED,
        };
        // Step wall triangles
        let wall_a = PatchTriangle {
            triangle: Triangle::new(l1, l2, u3),
            neighbors: [None, Some(3), None],
            surface: SurfaceId::UNSPECIFIED,
        };
        let wall_b = PatchTriangle {
            triangle: Triangle::new(l1, u3, u0),
            neighbors: [Some(2), Some(4), None],
            surface: SurfaceId::UNSPECIFIED,
        };
        // Upper level
        let upper_a = PatchTriangle {
            triangle: Triangle::new(u0, u2, u1),
            neighbors: [Some(5), None, Some(3)],
            surface: SurfaceId::UNSPECIFIED,
        };
        let upper_b = PatchTriangle {
            triangle: Triangle::new(u0, u3, u2),
            neighbors: [None, None, Some(4)],
            surface: SurfaceId::UNSPECIFIED,
        };

        Self {
            bounds: AABB::new(Point3::new(-s, -0.01, -s), Point3::new(s, h + 0.01, s)),
            patch: MeshPatch {
                triangles: vec![lower_a, lower_b, wall_a, wall_b, upper_a, upper_b],
            },
        }
    }
}

impl StaticGeometry for StepGeometry {
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

/// Bowl geometry: an inverted square-based pyramid.
///
/// Four triangular faces meet at a single apex below y=0. The rim is a
/// square at y=0. This shape requires the contact pipeline to generate
/// simultaneous contacts on multiple non-coplanar faces — the sphere must
/// rest where all four faces support it, not oscillate between opposite
/// sides.
#[derive(Debug, Clone)]
pub struct BowlGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl BowlGeometry {
    pub fn new(half_size: f32, depth: f32) -> Self {
        let s = half_size;
        // Rim vertices at y=0
        let v0 = Point3::new(-s, 0.0, -s);
        let v1 = Point3::new(s, 0.0, -s);
        let v2 = Point3::new(s, 0.0, s);
        let v3 = Point3::new(-s, 0.0, s);
        // Apex at bottom
        let apex = Point3::new(0.0, -depth, 0.0);

        // Winding: (apex, v_{i+1}, v_i) produces normals pointing into the
        // bowl (upward + inward).
        //
        // Edges per triangle: 0 = apex→v_{i+1}, 1 = v_{i+1}→v_i (rim), 2 = v_i→apex
        // Edge 0 is shared with the next triangle's edge 2.
        let front = PatchTriangle {
            triangle: Triangle::new(apex, v1, v0),
            neighbors: [Some(1), None, Some(3)],
            surface: SurfaceId::UNSPECIFIED,
        };
        let right = PatchTriangle {
            triangle: Triangle::new(apex, v2, v1),
            neighbors: [Some(2), None, Some(0)],
            surface: SurfaceId::UNSPECIFIED,
        };
        let back = PatchTriangle {
            triangle: Triangle::new(apex, v3, v2),
            neighbors: [Some(3), None, Some(1)],
            surface: SurfaceId::UNSPECIFIED,
        };
        let left = PatchTriangle {
            triangle: Triangle::new(apex, v0, v3),
            neighbors: [Some(0), None, Some(2)],
            surface: SurfaceId::UNSPECIFIED,
        };

        Self {
            bounds: AABB::new(Point3::new(-s, -depth - 0.01, -s), Point3::new(s, 0.01, s)),
            patch: MeshPatch {
                triangles: vec![front, right, back, left],
            },
        }
    }
}

impl StaticGeometry for BowlGeometry {
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

/// Combined floor (y=0, normal +Y) and wall (x=0, normal +X) geometry.
///
/// The floor covers the full XZ footprint. The wall is a vertical face at x=0
/// spanning y from 0 to `wall_height`. Together they form a right-angle corner.
/// Rigid bodies placed at x > 0 may collide with both faces.
#[derive(Debug, Clone)]
pub struct WallAndFloorGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl WallAndFloorGeometry {
    pub fn new(half_size: f32, wall_height: f32) -> Self {
        let s = half_size;
        let h = wall_height;

        // Floor at y=0, normal +Y.
        // Winding check: Triangle::new(a,b,c) → normal = (b-a)×(c-a).
        // f0=(-s,0,-s), f2=(s,0,s), f1=(s,0,-s):
        //   (f2-f0)×(f1-f0) = (2s,0,2s)×(2s,0,0) = (0,4s²,0) → +Y ✓
        let f0 = Point3::new(-s, 0.0, -s);
        let f1 = Point3::new(s, 0.0, -s);
        let f2 = Point3::new(s, 0.0, s);
        let f3 = Point3::new(-s, 0.0, s);
        let floor_a = PatchTriangle {
            triangle: Triangle::new(f0, f2, f1),
            neighbors: [Some(1), None, None],
            surface: SurfaceId::UNSPECIFIED,
        };
        let floor_b = PatchTriangle {
            triangle: Triangle::new(f0, f3, f2),
            neighbors: [None, None, Some(0)],
            surface: SurfaceId::UNSPECIFIED,
        };

        // Wall at x=0, normal +X (faces the positive-X side where bodies rest).
        // w0=(0,0,-s), w2=(0,h,-s), w1=(0,0,s):
        //   (w2-w0)×(w1-w0) = (0,h,0)×(0,0,2s) = (2hs,0,0) → +X ✓
        // w2=(0,h,-s), w3=(0,h,s), w1=(0,0,s):
        //   (w3-w2)×(w1-w2) = (0,0,2s)×(0,-h,2s) = (2hs,0,0) → +X ✓
        let w0 = Point3::new(0.0, 0.0, -s);
        let w1 = Point3::new(0.0, 0.0, s);
        let w2 = Point3::new(0.0, h, -s);
        let w3 = Point3::new(0.0, h, s);
        let wall_a = PatchTriangle {
            triangle: Triangle::new(w0, w2, w1),
            neighbors: [None, None, Some(3)],
            surface: SurfaceId::UNSPECIFIED,
        };
        let wall_b = PatchTriangle {
            triangle: Triangle::new(w2, w3, w1),
            neighbors: [None, None, Some(2)],
            surface: SurfaceId::UNSPECIFIED,
        };

        Self {
            bounds: AABB::new(Point3::new(-s, -0.01, -s), Point3::new(s, h + 0.01, s)),
            patch: MeshPatch {
                triangles: vec![floor_a, floor_b, wall_a, wall_b],
            },
        }
    }
}

impl StaticGeometry for WallAndFloorGeometry {
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

/// A floor at y=0 with convex solids standing on it, each a closed surface
/// but for its base, which terrain never has either.
///
/// What a one-sided contact test has to read the inside of solid geometry
/// from: a thin wall's two faces point away from each other, a slot's two
/// walls face each other, and only their arrangement says which is which.
#[derive(Debug, Clone)]
pub struct SolidsGeometry {
    bounds: AABB,
    patch: MeshPatch,
}

impl SolidsGeometry {
    pub fn new(floor_half_size: f32, solids: &[ConvexSolid]) -> Self {
        let s = floor_half_size;
        let floor = ConvexSolid::orient(
            vec![vec![
                Point3::new(-s, 0.0, -s),
                Point3::new(s, 0.0, -s),
                Point3::new(s, 0.0, s),
                Point3::new(-s, 0.0, s),
            ]],
            Point3::new(0.0, -1.0, 0.0),
        );

        let mut triangles = Vec::new();
        let mut top = 0.0f32;
        for face in floor
            .faces
            .iter()
            .chain(solids.iter().flat_map(|solid| &solid.faces))
        {
            for i in 1..face.len() - 1 {
                top = top.max(face[i].y).max(face[i + 1].y);
                triangles.push(PatchTriangle {
                    triangle: Triangle::new(face[0], face[i], face[i + 1]),
                    neighbors: [None, None, None],
                    surface: SurfaceId::UNSPECIFIED,
                });
            }
        }

        Self {
            bounds: AABB::new(Point3::new(-s, -0.01, -s), Point3::new(s, top + 0.01, s)),
            patch: MeshPatch { triangles },
        }
    }
}

impl StaticGeometry for SolidsGeometry {
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

/// A convex solid as its faces, each wound so its normal points out.
#[derive(Debug, Clone)]
pub struct ConvexSolid {
    /// Convex polygons, wound so `Triangle::new(v0, v1, v2)` faces outward.
    faces: Vec<Vec<Point3<f32>>>,
}

impl ConvexSolid {
    /// An axis-aligned block from `min` to `max`, its base on the floor.
    pub fn block(min: Point3<f32>, max: Point3<f32>) -> Self {
        Self::upright_prism(
            &[
                (min.x, min.z),
                (max.x, min.z),
                (max.x, max.z),
                (min.x, max.z),
            ],
            max.y,
        )
    }

    /// A convex polygon in the floor's `(x, z)`, raised to `height`.
    pub fn upright_prism(outline: &[(f32, f32)], height: f32) -> Self {
        let at = |(x, z): (f32, f32), y: f32| Point3::new(x, y, z);
        let mut faces = vec![outline.iter().map(|&p| at(p, height)).collect()];
        for i in 0..outline.len() {
            let (a, b) = (outline[i], outline[(i + 1) % outline.len()]);
            faces.push(vec![at(a, 0.0), at(b, 0.0), at(b, height), at(a, height)]);
        }
        let inside = Self::mean(&faces);
        Self::orient(faces, inside)
    }

    /// A ridge along z: a triangle in `(x, y)` with its base on the floor from
    /// `-half_base` to `half_base` and its apex at `height` over `x = 0`,
    /// running from `-half_length` to `half_length`.
    pub fn ridge(half_base: f32, height: f32, half_length: f32) -> Self {
        let section = [(-half_base, 0.0), (0.0, height), (half_base, 0.0)];
        let at = |(x, y): (f32, f32), z: f32| Point3::new(x, y, z);
        let mut faces = vec![
            section.iter().map(|&p| at(p, -half_length)).collect(),
            section.iter().map(|&p| at(p, half_length)).collect(),
        ];
        for (a, b) in [(section[0], section[1]), (section[1], section[2])] {
            faces.push(vec![
                at(a, -half_length),
                at(b, -half_length),
                at(b, half_length),
                at(a, half_length),
            ]);
        }
        let inside = Self::mean(&faces);
        Self::orient(faces, inside)
    }

    /// Wind every face so its normal points away from `inside`.
    fn orient(mut faces: Vec<Vec<Point3<f32>>>, inside: Point3<f32>) -> Self {
        for face in &mut faces {
            let normal = (face[1] - face[0]).cross(&(face[2] - face[0]));
            let outward = Self::mean(std::slice::from_ref(face)) - inside;
            if normal.dot(&outward) < 0.0 {
                face.reverse();
            }
        }
        Self { faces }
    }

    fn mean(faces: &[Vec<Point3<f32>>]) -> Point3<f32> {
        let points: Vec<_> = faces.iter().flatten().collect();
        let sum = points
            .iter()
            .fold(Vector3::zeros(), |acc, p| acc + p.coords);
        Point3::from(sum / points.len() as f32)
    }
}
