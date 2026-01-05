use nalgebra::{Vector2, Vector3, Vector4};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

use crate::core::error::{EngineError, EngineResult};
use crate::{components::Mesh, rendering::vertex::Vertex, resources::textures::TextureManager};

pub struct Landscape {
    pub mesh: Mesh,
}

pub struct LandscapeLoader<'a> {
    texture_manager: &'a TextureManager,
}

impl<'a> LandscapeLoader<'a> {
    pub fn new(texture_manager: &'a TextureManager) -> Self {
        Self { texture_manager }
    }

    pub fn gaia(self) -> EngineResult<Landscape> {
        Ok(Landscape {
            mesh: Mesh {
                vertices: vec![
                    Vertex {
                        pos: Vector4::new(0.0, 0.0, 0.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(0.0, 10.0, 0.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(100.0, 0.0, 0.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(100.0, 10.0, 0.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(100.0, 0.0, 64.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(100.0, 10.0, 64.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(0.0, 0.0, 64.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(0.0, 10.0, 64.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(67.0, 0.0, 64.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(72.0, 0.0, 46.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(83.0, 0.0, 36.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(93.0, 0.0, 34.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(100.0, 0.0, 35.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(82.0, 0.0, 64.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(82.0, -10.0, 49.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(100.0, 0.0, 49.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(30.0, 10.0, 16.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(40.5, 10.0, 20.5, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(45.0, 10.0, 31.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(40.5, 10.0, 41.5, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(30.0, 10.0, 46.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(19.5, 10.0, 41.5, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(15.0, 10.0, 31.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(19.5, 10.0, 20.5, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(30.0, 7.5, 20.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(37.0, 7.5, 23.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(40.0, 7.5, 31.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(37.0, 7.5, 39.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(30.0, 7.5, 42.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(23.0, 7.5, 39.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(20.0, 7.5, 31.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(23.0, 7.5, 23.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(30.0, 5.0, 31.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(0.0, 0.0, 20.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(0.0, 0.0, 26.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(-7.0, 0.0, 20.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(-7.0, 0.0, 26.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(0.0, 5.0, 20.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(0.0, 5.0, 26.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(-7.0, 5.0, 20.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(-7.0, 5.0, 26.0, 1.0),
                        color: Vector4::new(1.0, 1.0, 1.0, 1.0),
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                ],
                indices: vec![
                    0, 2, 1, 2, 3, 1, 2, 4, 3, 4, 5, 3, 4, 6, 5, 6, 7, 5, 0, 1, 37, 0, 37, 33, 0,
                    6, 2, 6, 8, 9, 6, 9, 10, 6, 10, 2, 2, 10, 11, 2, 11, 12, 8, 13, 14, 13, 4, 14,
                    14, 4, 15, 14, 15, 12, 14, 12, 11, 14, 11, 10, 14, 10, 9, 8, 14, 9, 21, 23, 22,
                    21, 16, 23, 21, 17, 16, 21, 18, 17, 21, 19, 18, 21, 20, 19, 16, 17, 24, 24, 17,
                    25, 25, 17, 18, 25, 18, 26, 26, 18, 19, 26, 19, 27, 27, 19, 20, 27, 20, 28, 28,
                    20, 21, 28, 21, 29, 29, 21, 22, 29, 22, 30, 30, 22, 23, 30, 23, 31, 31, 23, 16,
                    31, 16, 24, 32, 24, 25, 32, 25, 26, 32, 26, 27, 32, 27, 28, 32, 28, 29, 32, 29,
                    30, 32, 30, 31, 32, 31, 24, 33, 35, 34, 35, 36, 34, 33, 37, 39, 33, 39, 35, 34,
                    36, 40, 34, 40, 38, 39, 37, 38, 39, 38, 40, 1, 38, 37, 1, 7, 38, 7, 6, 38, 6,
                    34, 38,
                ],
                texture_handles: vec![self.texture_manager.load_texture("data/grass.bmp")?],
            },
        })
    }

    pub fn flat_plane(self) -> EngineResult<Landscape> {
        let size = 50.0; // Half-size of the plane
        let y_level = 0.0;
        let grass_color = Vector4::new(0.2, 0.8, 0.2, 1.0); // Green

        Ok(Landscape {
            mesh: Mesh {
                vertices: vec![
                    Vertex {
                        pos: Vector4::new(-size, y_level, -size, 1.0),
                        color: grass_color,
                        tex_coords: Vector2::new(0.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(-size, y_level, size, 1.0),
                        color: grass_color,
                        tex_coords: Vector2::new(0.0, 1.0),
                    },
                    Vertex {
                        pos: Vector4::new(size, y_level, -size, 1.0),
                        color: grass_color,
                        tex_coords: Vector2::new(1.0, 0.0),
                    },
                    Vertex {
                        pos: Vector4::new(size, y_level, size, 1.0),
                        color: grass_color,
                        tex_coords: Vector2::new(1.0, 1.0),
                    },
                ],
                indices: vec![0, 1, 2, 2, 1, 3],
                texture_handles: vec![self.texture_manager.load_texture("data/grass.bmp")?],
            },
        })
    }

    pub fn from_obj_string(
        self,
        obj_data: &str,
        default_color: Vector4<f32>,
        texture_path: &str,
    ) -> EngineResult<Landscape> {
        let mut obj_positions: Vec<Vector3<f32>> = Vec::new();
        let mut obj_tex_coords: Vec<Vector2<f32>> = Vec::new();
        // let mut obj_normals: Vec<Vector3<f32>> = Vec::new(); // Normals not handled yet

        let mut final_vertices: Vec<Vertex> = Vec::new();
        let mut final_indices: Vec<u32> = Vec::new();

        // Key: (vertex_idx, tex_coord_idx_option)
        let mut vertex_map: HashMap<(usize, Option<usize>), u32> = HashMap::new();

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
                            path: Some(texture_path.to_string()),
                            reason: format!("Invalid vertex line: '{}'. Expected 'v x y z'", line),
                        });
                    }
                    let x = parts[1].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(texture_path.to_string()),
                        reason: format!("Failed to parse vertex x from '{}': {}", parts[1], e),
                    })?;
                    let y = parts[2].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(texture_path.to_string()),
                        reason: format!("Failed to parse vertex y from '{}': {}", parts[2], e),
                    })?;
                    let z = parts[3].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(texture_path.to_string()),
                        reason: format!("Failed to parse vertex z from '{}': {}", parts[3], e),
                    })?;
                    obj_positions.push(Vector3::new(x, y, z));
                }
                "vt" => {
                    // Texture coordinate
                    if parts.len() < 3 {
                        return Err(EngineError::Mesh {
                            path: Some(texture_path.to_string()),
                            reason: format!("Invalid texcoord line: '{}'", line),
                        });
                    }
                    let u = parts[1].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(texture_path.to_string()),
                        reason: format!("Failed to parse texcoord u from '{}': {}", parts[1], e),
                    })?;
                    // OBJ V coordinate can be inverted; often 1.0 - v is needed. Assuming direct use for now.
                    let v = parts[2].parse::<f32>().map_err(|e| EngineError::Mesh {
                        path: Some(texture_path.to_string()),
                        reason: format!("Failed to parse texcoord v from '{}': {}", parts[2], e),
                    })?;
                    obj_tex_coords.push(Vector2::new(u, v));
                }
                "vn" => { // Vertex normal - parsed but not directly used yet
                     // if parts.len() < 4 { return Err(format!("Invalid normal line: '{}'", line)); }
                     // let nx = parts[1].parse::<f32>().map_err(|e| format!("Failed to parse normal x: {}", e))?;
                     // let ny = parts[2].parse::<f32>().map_err(|e| format!("Failed to parse normal y: {}", e))?;
                     // let nz = parts[3].parse::<f32>().map_err(|e| format!("Failed to parse normal z: {}", e))?;
                     // obj_normals.push(Vector3::new(nx, ny, nz)); // Store if Vertex struct is extended for normals
                }
                "f" => {
                    // Face
                    if parts.len() < 4 {
                        return Err(EngineError::Mesh {
                            path: Some(texture_path.to_string()),
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
                                path: Some(texture_path.to_string()),
                                reason: format!(
                                    "Missing vertex index in face part: {}",
                                    face_part_str
                                ),
                            })?;
                        if v_idx_str.is_empty() {
                            return Err(EngineError::Mesh {
                                path: Some(texture_path.to_string()),
                                reason: format!(
                                    "Empty vertex index in face part: {}",
                                    face_part_str
                                ),
                            });
                        }
                        let v_idx = v_idx_str.parse::<usize>().map_err(|e| EngineError::Mesh {
                            path: Some(texture_path.to_string()),
                            reason: format!(
                                "Failed to parse vertex index '{}' from '{}': {}",
                                v_idx_str, face_part_str, e
                            ),
                        })?;

                        if v_idx == 0 || v_idx > obj_positions.len() {
                            return Err(EngineError::Mesh {
                                path: Some(texture_path.to_string()),
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
                                    path: Some(texture_path.to_string()),
                                    reason: format!(
                                        "Failed to parse texture coord index '{}' from '{}': {}",
                                        s, face_part_str, e
                                    ),
                                })?;
                                if vt_idx == 0 || vt_idx > obj_tex_coords.len() {
                                    return Err(EngineError::Mesh {
                                        path: Some(texture_path.to_string()),
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

                        // vn_idx_option would be parsed similarly if handling normals:
                        // let vn_idx_option_str = component_indices.next(); ...

                        let vertex_key = (v_idx, vt_idx_option);

                        let final_vertex_idx = match vertex_map.get(&vertex_key) {
                            Some(&idx) => idx,
                            None => {
                                let pos3d = obj_positions[v_idx - 1]; // OBJ is 1-based
                                let tex_coords_2d = vt_idx_option
                                    .map(|vt_idx| obj_tex_coords[vt_idx - 1]) // OBJ is 1-based
                                    .unwrap_or_else(|| Vector2::new(0.0, 0.0)); // Default if not specified

                                let new_vertex = Vertex {
                                    pos: Vector4::new(pos3d.x, pos3d.y, pos3d.z, 1.0),
                                    color: default_color,
                                    tex_coords: tex_coords_2d,
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

        let texture_handle = self.texture_manager.load_texture(texture_path)?;

        Ok(Landscape {
            mesh: Mesh {
                vertices: final_vertices,
                indices: final_indices,
                texture_handles: vec![texture_handle],
            },
        })
    }

    pub fn load_ripple_obj(self) -> EngineResult<Landscape> {
        let obj_path_str = "data/ripple.obj";
        let obj_path = Path::new(obj_path_str);

        let obj_data = fs::read_to_string(obj_path).map_err(|e| EngineError::Io {
            path: obj_path_str.to_string(),
            reason: format!("Failed to read OBJ file: {}", e),
        })?;

        // Let's use a light gray as the default color for the ripple object
        let default_color = Vector4::new(0.7, 0.7, 0.7, 1.0);

        self.from_obj_string(&obj_data, default_color, "data/grass.bmp")
    }
}
