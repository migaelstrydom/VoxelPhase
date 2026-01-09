use nalgebra::{Vector2, Vector3, Vector4};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

use crate::core::error::{EngineError, EngineResult};
use crate::model::{MeshPrimitive, Model, ModelPart};
use crate::rendering::colour::Colour;
use crate::rendering::material::MaterialId;
use crate::rendering::vertex::Vertex;

/// Helper function to compute smooth vertex normals from an indexed mesh
fn compute_smooth_normals(positions: &[Vector3<f32>], indices: &[u32]) -> Vec<Vector3<f32>> {
    let mut normals = vec![Vector3::zeros(); positions.len()];

    // Iterate over each triangle
    for triangle in indices.chunks(3) {
        if triangle.len() != 3 {
            continue;
        }

        let i0 = triangle[0] as usize;
        let i1 = triangle[1] as usize;
        let i2 = triangle[2] as usize;

        let p0 = positions[i0];
        let p1 = positions[i1];
        let p2 = positions[i2];

        // Compute face normal using cross product
        let edge1 = p1 - p0;
        let edge2 = p2 - p0;
        let face_normal = edge1.cross(&edge2);

        // Accumulate the face normal to each vertex of the triangle
        normals[i0] += face_normal;
        normals[i1] += face_normal;
        normals[i2] += face_normal;
    }

    // Normalize all accumulated normals
    for normal in normals.iter_mut() {
        if normal.magnitude_squared() > 0.0 {
            *normal = normal.normalize();
        } else {
            // Default to upward if no normal could be computed
            *normal = Vector3::new(0.0, 1.0, 0.0);
        }
    }

    normals
}

/// Loads landscape/terrain geometry and returns Models.
pub struct LandscapeLoader {
    material: MaterialId,
    colour: Colour,
}

impl LandscapeLoader {
    /// Create a new loader with the given material and vertex colour.
    pub fn new(material: MaterialId, colour: Colour) -> Self {
        Self { material, colour }
    }

    /// Create a loader with white vertex colour (texture shows through).
    pub fn textured(material: MaterialId) -> Self {
        Self::new(material, Colour::WHITE)
    }

    #[allow(dead_code)]
    pub fn gaia(&self) -> EngineResult<Model> {
        // Define positions
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 10.0, 0.0),
            Vector3::new(100.0, 0.0, 0.0),
            Vector3::new(100.0, 10.0, 0.0),
            Vector3::new(100.0, 0.0, 64.0),
            Vector3::new(100.0, 10.0, 64.0),
            Vector3::new(0.0, 0.0, 64.0),
            Vector3::new(0.0, 10.0, 64.0),
            Vector3::new(67.0, 0.0, 64.0),
            Vector3::new(72.0, 0.0, 46.0),
            Vector3::new(83.0, 0.0, 36.0),
            Vector3::new(93.0, 0.0, 34.0),
            Vector3::new(100.0, 0.0, 35.0),
            Vector3::new(82.0, 0.0, 64.0),
            Vector3::new(82.0, -10.0, 49.0),
            Vector3::new(100.0, 0.0, 49.0),
            Vector3::new(30.0, 10.0, 16.0),
            Vector3::new(40.5, 10.0, 20.5),
            Vector3::new(45.0, 10.0, 31.0),
            Vector3::new(40.5, 10.0, 41.5),
            Vector3::new(30.0, 10.0, 46.0),
            Vector3::new(19.5, 10.0, 41.5),
            Vector3::new(15.0, 10.0, 31.0),
            Vector3::new(19.5, 10.0, 20.5),
            Vector3::new(30.0, 7.5, 20.0),
            Vector3::new(37.0, 7.5, 23.0),
            Vector3::new(40.0, 7.5, 31.0),
            Vector3::new(37.0, 7.5, 39.0),
            Vector3::new(30.0, 7.5, 42.0),
            Vector3::new(23.0, 7.5, 39.0),
            Vector3::new(20.0, 7.5, 31.0),
            Vector3::new(23.0, 7.5, 23.0),
            Vector3::new(30.0, 5.0, 31.0),
            Vector3::new(0.0, 0.0, 20.0),
            Vector3::new(0.0, 0.0, 26.0),
            Vector3::new(-7.0, 0.0, 20.0),
            Vector3::new(-7.0, 0.0, 26.0),
            Vector3::new(0.0, 5.0, 20.0),
            Vector3::new(0.0, 5.0, 26.0),
            Vector3::new(-7.0, 5.0, 20.0),
            Vector3::new(-7.0, 5.0, 26.0),
        ];

        // Define indices
        let indices = vec![
            0, 2, 1, 2, 3, 1, 2, 4, 3, 4, 5, 3, 4, 6, 5, 6, 7, 5, 0, 1, 37, 0, 37, 33, 0, 6, 2, 6,
            8, 9, 6, 9, 10, 6, 10, 2, 2, 10, 11, 2, 11, 12, 8, 13, 14, 13, 4, 14, 14, 4, 15, 14,
            15, 12, 14, 12, 11, 14, 11, 10, 14, 10, 9, 8, 14, 9, 21, 23, 22, 21, 16, 23, 21, 17,
            16, 21, 18, 17, 21, 19, 18, 21, 20, 19, 16, 17, 24, 24, 17, 25, 25, 17, 18, 25, 18, 26,
            26, 18, 19, 26, 19, 27, 27, 19, 20, 27, 20, 28, 28, 20, 21, 28, 21, 29, 29, 21, 22, 29,
            22, 30, 30, 22, 23, 30, 23, 31, 31, 23, 16, 31, 16, 24, 32, 24, 25, 32, 25, 26, 32, 26,
            27, 32, 27, 28, 32, 28, 29, 32, 29, 30, 32, 30, 31, 32, 31, 24, 33, 35, 34, 35, 36, 34,
            33, 37, 39, 33, 39, 35, 34, 36, 40, 34, 40, 38, 39, 37, 38, 39, 38, 40, 1, 38, 37, 1,
            7, 38, 7, 6, 38, 6, 34, 38,
        ];

        // Compute smooth normals
        let normals = compute_smooth_normals(&positions, &indices);
        let colour_vec = self.colour.to_vec4();

        // Build vertices with computed normals
        let vertices: Vec<Vertex> = positions
            .iter()
            .zip(normals.iter())
            .map(|(pos, normal)| Vertex {
                pos: Vector4::new(pos.x, pos.y, pos.z, 1.0),
                color: colour_vec,
                tex_coords: Vector2::new(0.0, 0.0),
                normal: *normal,
            })
            .collect();

        Ok(Model::flat(vec![ModelPart::new(vec![MeshPrimitive {
            vertices,
            indices,
            material: self.material,
        }])]))
    }

    #[allow(dead_code)]
    pub fn flat_plane(&self) -> Model {
        let size = 50.0; // Half-size of the plane
        let y_level = 0.0;
        let colour_vec = self.colour.to_vec4();
        let up_normal = Vector3::new(0.0, 1.0, 0.0);

        Model::flat(vec![ModelPart::new(vec![MeshPrimitive {
            vertices: vec![
                Vertex {
                    pos: Vector4::new(-size, y_level, -size, 1.0),
                    color: colour_vec,
                    tex_coords: Vector2::new(0.0, 0.0),
                    normal: up_normal,
                },
                Vertex {
                    pos: Vector4::new(-size, y_level, size, 1.0),
                    color: colour_vec,
                    tex_coords: Vector2::new(0.0, 1.0),
                    normal: up_normal,
                },
                Vertex {
                    pos: Vector4::new(size, y_level, -size, 1.0),
                    color: colour_vec,
                    tex_coords: Vector2::new(1.0, 0.0),
                    normal: up_normal,
                },
                Vertex {
                    pos: Vector4::new(size, y_level, size, 1.0),
                    color: colour_vec,
                    tex_coords: Vector2::new(1.0, 1.0),
                    normal: up_normal,
                },
            ],
            indices: vec![0, 1, 2, 2, 1, 3],
            material: self.material,
        }])])
    }

    pub fn from_obj_string(&self, obj_data: &str, obj_name: &str) -> EngineResult<Model> {
        let mut obj_positions: Vec<Vector3<f32>> = Vec::new();
        let mut obj_tex_coords: Vec<Vector2<f32>> = Vec::new();
        let mut obj_normals: Vec<Vector3<f32>> = Vec::new();

        let mut final_vertices: Vec<Vertex> = Vec::new();
        let mut final_indices: Vec<u32> = Vec::new();
        let colour_vec = self.colour.to_vec4();

        // Key: (vertex_idx, tex_coord_idx_option, normal_idx_option)
        let mut vertex_map: HashMap<(usize, Option<usize>, Option<usize>), u32> = HashMap::new();

        for line in obj_data.lines() {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.is_empty() {
                continue;
            }

            match parts[0] {
                "#" => { /* Comment, ignore */ }
                "v" => {
                    // Vertex position
                    if parts.len() < 4 {
                        return Err(EngineError::Mesh {
                            path: Some(obj_name.to_string()),
                            reason: format!("Invalid vertex line: '{}'. Expected 'v x y z'", line),
                        });
                    }
                    let x = parts[1].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(obj_name.to_string()),
                        reason: format!("Failed to parse vertex x from '{}': {}", parts[1], e),
                    })?;
                    let y = parts[2].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(obj_name.to_string()),
                        reason: format!("Failed to parse vertex y from '{}': {}", parts[2], e),
                    })?;
                    let z = parts[3].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(obj_name.to_string()),
                        reason: format!("Failed to parse vertex z from '{}': {}", parts[3], e),
                    })?;
                    obj_positions.push(Vector3::new(x, y, z));
                }
                "vt" => {
                    // Texture coordinate
                    if parts.len() < 3 {
                        return Err(EngineError::Mesh {
                            path: Some(obj_name.to_string()),
                            reason: format!("Invalid texcoord line: '{}'", line),
                        });
                    }
                    let u = parts[1].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(obj_name.to_string()),
                        reason: format!("Failed to parse texcoord u from '{}': {}", parts[1], e),
                    })?;
                    // OBJ V coordinate can be inverted; often 1.0 - v is needed. Assuming direct use for now.
                    let v = parts[2].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(obj_name.to_string()),
                        reason: format!("Failed to parse texcoord v from '{}': {}", parts[2], e),
                    })?;
                    obj_tex_coords.push(Vector2::new(u, v));
                }
                "vn" => {
                    // Vertex normal
                    if parts.len() < 4 {
                        return Err(EngineError::Mesh {
                            path: Some(obj_name.to_string()),
                            reason: format!("Invalid normal line: '{}'. Expected 'vn x y z'", line),
                        });
                    }
                    let nx = parts[1].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(obj_name.to_string()),
                        reason: format!("Failed to parse normal x from '{}': {}", parts[1], e),
                    })?;
                    let ny = parts[2].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(obj_name.to_string()),
                        reason: format!("Failed to parse normal y from '{}': {}", parts[2], e),
                    })?;
                    let nz = parts[3].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(obj_name.to_string()),
                        reason: format!("Failed to parse normal z from '{}': {}", parts[3], e),
                    })?;
                    obj_normals.push(Vector3::new(nx, ny, nz));
                }
                "f" => {
                    // Face
                    if parts.len() < 4 {
                        return Err(EngineError::Mesh {
                            path: Some(obj_name.to_string()),
                            reason: format!("Face must have at least 3 vertices: '{}'", line),
                        });
                    }

                    let mut face_vertex_indices_in_final_list: Vec<u32> = Vec::new();

                    for i in 1..parts.len() {
                        // Iterate over face components like "v/vt/vn"
                        let face_part_str = parts[i];
                        let mut component_indices = face_part_str.split('/');

                        let v_idx_str =
                            component_indices.next().ok_or_else(|| EngineError::Mesh {
                                path: Some(obj_name.to_string()),
                                reason: format!(
                                    "Missing vertex index in face part: {}",
                                    face_part_str
                                ),
                            })?;
                        if v_idx_str.is_empty() {
                            return Err(EngineError::Mesh {
                                path: Some(obj_name.to_string()),
                                reason: format!(
                                    "Empty vertex index in face part: {}",
                                    face_part_str
                                ),
                            });
                        }
                        let v_idx = v_idx_str.parse::<usize>().map_err(|e| EngineError::Mesh {
                            path: Some(obj_name.to_string()),
                            reason: format!(
                                "Failed to parse vertex index '{}' from '{}': {}",
                                v_idx_str, face_part_str, e
                            ),
                        })?;

                        if v_idx == 0 || v_idx > obj_positions.len() {
                            return Err(EngineError::Mesh {
                                path: Some(obj_name.to_string()),
                                reason: format!(
                                    "Vertex position index {} out of bounds (1 to {}). Line: '{}'",
                                    v_idx,
                                    obj_positions.len(),
                                    line
                                ),
                            });
                        }

                        let vt_idx_option_str = component_indices.next();
                        let vt_idx_option = match vt_idx_option_str {
                            Some(s) if !s.is_empty() => {
                                let vt_idx = s.parse::<usize>().map_err(|e| EngineError::Mesh {
                                    path: Some(obj_name.to_string()),
                                    reason: format!(
                                        "Failed to parse texture coord index '{}' from '{}': {}",
                                        s, face_part_str, e
                                    ),
                                })?;
                                if vt_idx == 0 || vt_idx > obj_tex_coords.len() {
                                    return Err(EngineError::Mesh {
                                        path: Some(obj_name.to_string()),
                                        reason: format!(
                                            "Texture coord index {} out of bounds (1 to {}). Line: '{}'",
                                            vt_idx,
                                            obj_tex_coords.len(),
                                            line
                                        ),
                                    });
                                }
                                Some(vt_idx)
                            }
                            _ => None, // No texture coordinate index provided for this vertex component
                        };

                        // Parse normal index
                        let vn_idx_option_str = component_indices.next();
                        let vn_idx_option = match vn_idx_option_str {
                            Some(s) if !s.is_empty() => {
                                let vn_idx = s.parse::<usize>().map_err(|e| EngineError::Mesh {
                                    path: Some(obj_name.to_string()),
                                    reason: format!(
                                        "Failed to parse normal index '{}' from '{}': {}",
                                        s, face_part_str, e
                                    ),
                                })?;
                                if vn_idx == 0 || vn_idx > obj_normals.len() {
                                    return Err(EngineError::Mesh {
                                        path: Some(obj_name.to_string()),
                                        reason: format!(
                                            "Normal index {} out of bounds (1 to {}). Line: '{}'",
                                            vn_idx,
                                            obj_normals.len(),
                                            line
                                        ),
                                    });
                                }
                                Some(vn_idx)
                            }
                            _ => None, // No normal index provided for this vertex component
                        };

                        let vertex_key = (v_idx, vt_idx_option, vn_idx_option);

                        let final_vertex_idx = match vertex_map.get(&vertex_key) {
                            Some(&idx) => idx,
                            None => {
                                let pos3d = obj_positions[v_idx - 1]; // OBJ is 1-based
                                let tex_coords_2d = vt_idx_option
                                    .map(|vt_idx| obj_tex_coords[vt_idx - 1]) // OBJ is 1-based
                                    .unwrap_or_else(|| Vector2::new(0.0, 0.0)); // Default if not specified
                                let normal3d = vn_idx_option
                                    .map(|vn_idx| obj_normals[vn_idx - 1]) // OBJ is 1-based
                                    .unwrap_or_else(|| Vector3::new(0.0, 1.0, 0.0)); // Default upward normal if not specified

                                let new_vertex = Vertex {
                                    pos: Vector4::new(pos3d.x, pos3d.y, pos3d.z, 1.0),
                                    color: colour_vec,
                                    tex_coords: tex_coords_2d,
                                    normal: normal3d,
                                };
                                final_vertices.push(new_vertex);
                                let new_idx = (final_vertices.len() - 1) as u32;
                                vertex_map.insert(vertex_key, new_idx);
                                new_idx
                            }
                        };
                        face_vertex_indices_in_final_list.push(final_vertex_idx);
                    }

                    // Triangulate the face (simple fan triangulation from the first vertex of the face)
                    if face_vertex_indices_in_final_list.len() >= 3 {
                        let first_final_idx = face_vertex_indices_in_final_list[0];
                        for i in 2..face_vertex_indices_in_final_list.len() {
                            final_indices.push(first_final_idx);
                            final_indices.push(face_vertex_indices_in_final_list[i - 1]);
                            final_indices.push(face_vertex_indices_in_final_list[i]);
                        }
                    }
                }
                "g" => { /* Group name, ignored for this simple parser */ }
                "s" => { /* Smoothing group, ignored */ }
                "usemtl" => { /* Material name, ignored */ }
                "mtllib" => { /* Material library, ignored */ }
                _ => { /* Unknown or unhandled line type, log or ignore */ }
            }
        }

        // If the OBJ file didn't contain normals, compute them from the geometry
        if obj_normals.is_empty() && !final_vertices.is_empty() {
            println!(
                "OBJ file has no normals, computing {} normals from geometry",
                final_vertices.len()
            );

            // Extract positions from vertices
            let positions: Vec<Vector3<f32>> = final_vertices
                .iter()
                .map(|v| Vector3::new(v.pos.x, v.pos.y, v.pos.z))
                .collect();

            // Compute smooth normals
            let computed_normals = compute_smooth_normals(&positions, &final_indices);

            // Update vertices with computed normals
            for (vertex, normal) in final_vertices.iter_mut().zip(computed_normals.iter()) {
                vertex.normal = *normal;
            }
        }

        Ok(Model::flat(vec![ModelPart::new(vec![MeshPrimitive {
            vertices: final_vertices,
            indices: final_indices,
            material: self.material,
        }])]))
    }

    pub fn load_ripple_obj(&self) -> EngineResult<Model> {
        let obj_path_str = "data/ripple.obj";
        let obj_path = Path::new(obj_path_str);

        let obj_data = fs::read_to_string(obj_path).map_err(|e| EngineError::Io {
            path: obj_path_str.to_string(),
            reason: format!("Failed to read OBJ file: {}", e),
        })?;

        self.from_obj_string(&obj_data, obj_path_str)
    }
}
