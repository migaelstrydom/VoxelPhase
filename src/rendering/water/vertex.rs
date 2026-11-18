use ash::vk;
use nalgebra::{Vector2, Vector3, Vector4};

/// A water surface vertex. Its height is not stored: every draw pushes its
/// body's level, so the mesh outlives a change of level.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BasinVertex {
    /// World (x, z).
    pub xz: Vector2<f32>,
    /// The floor under this vertex, for the swell's shore fade.
    pub floor: f32,
    /// The share of the swell's height the geometry carries: 1, except in
    /// the ring past the map's edge, whose quads are too coarse to follow
    /// it. Its normal is the fragment shader's, at full swell.
    pub swell_share: f32,
    /// How far below the surface the vertex lies, m: only where the ring
    /// tucks under the map's own water.
    pub sink: f32,
}

impl BasinVertex {
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 4] {
        let float = std::mem::size_of::<f32>() as u32;
        [
            vk::VertexInputAttributeDescription {
                location: 0,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 0,
            },
            vk::VertexInputAttributeDescription {
                location: 1,
                binding: 0,
                format: vk::Format::R32_SFLOAT,
                offset: 2 * float,
            },
            vk::VertexInputAttributeDescription {
                location: 2,
                binding: 0,
                format: vk::Format::R32_SFLOAT,
                offset: 3 * float,
            },
            vk::VertexInputAttributeDescription {
                location: 3,
                binding: 0,
                format: vk::Format::R32_SFLOAT,
                offset: 4 * float,
            },
        ]
    }
}

/// A vertex of the fine ripple-tile grid: its position within the 8 m tile.
/// The tile's origin, its ripples and its body's level come from the draw.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FineVertex {
    pub local: Vector2<f32>,
}

impl FineVertex {
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 1] {
        [vk::VertexInputAttributeDescription {
            location: 0,
            binding: 0,
            format: vk::Format::R32G32_SFLOAT,
            offset: 0,
        }]
    }
}

/// A vertex of a river surface: the water's own surface there, its depth,
/// its flow, and on falling water what its curtain is drawn by.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RiverVertex {
    pub xz: Vector2<f32>,
    /// Water surface height.
    pub surface: f32,
    /// Depth of water: positive under water, negative over a bank standing
    /// above it. The draw cuts the surface below its shore depth, so a shore
    /// is a contour, not the edge of a column.
    pub depth: f32,
    /// Depth-averaged velocity, m/s, x and z. On falling water, where the
    /// water at this vertex left its lip, x and z: its streaks hang from
    /// there, whichever way it was thrown.
    pub flow: Vector2<f32>,
    /// On falling water, the way it flies, level and unit. Zero on the
    /// surface, which faces up.
    pub face: Vector2<f32>,
    /// On falling water, (how far this vertex has fallen below its lip, the
    /// whole fall of its strand, m; which side of its curtain's middle it is
    /// on, -1 or 1 at the ends of a fall's lip, along the level direction
    /// square to its flight, and 0 between; how thick the water is here,
    /// m): (0, fall, side, thickness) at the lip and (fall, fall, side,
    /// thickness) at the foot. On the foam where it lands, the first is its
    /// fall and 1 more and where across the foam the vertex lies, from -1
    /// behind the foot to 1 at its outer edge. Zero on the surface.
    pub sheet: Vector4<f32>,
    /// On falling water, the way its face faces: it is drawn as a body with
    /// a top face and an under face (`curtain.rs`). Up on the surface.
    pub normal: Vector3<f32>,
    /// On falling water, how far an end of a fall frays out into strands:
    /// 1 at the free end of a thin fall, 0 at the closed end of a thick one
    /// and wherever its curtain runs on into the next. Zero on the surface.
    pub fray: f32,
    /// On falling water, how much air it has taken in as it leaves its lip:
    /// 0 for water sliding glassy over a brink, 1 for water bursting white
    /// out of a hole. Zero on the surface.
    pub air: f32,
}

impl RiverVertex {
    pub fn binding_description() -> vk::VertexInputBindingDescription {
        vk::VertexInputBindingDescription {
            binding: 0,
            stride: std::mem::size_of::<Self>() as u32,
            input_rate: vk::VertexInputRate::VERTEX,
        }
    }

    pub fn attribute_descriptions() -> [vk::VertexInputAttributeDescription; 8] {
        let f = std::mem::size_of::<f32>() as u32;
        [
            vk::VertexInputAttributeDescription {
                location: 0,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 0,
            },
            // (surface, depth)
            vk::VertexInputAttributeDescription {
                location: 1,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 2 * f,
            },
            vk::VertexInputAttributeDescription {
                location: 2,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 4 * f,
            },
            vk::VertexInputAttributeDescription {
                location: 3,
                binding: 0,
                format: vk::Format::R32G32_SFLOAT,
                offset: 6 * f,
            },
            // (drop below the lip, whole fall, side, thickness)
            vk::VertexInputAttributeDescription {
                location: 4,
                binding: 0,
                format: vk::Format::R32G32B32A32_SFLOAT,
                offset: 8 * f,
            },
            vk::VertexInputAttributeDescription {
                location: 5,
                binding: 0,
                format: vk::Format::R32G32B32_SFLOAT,
                offset: 12 * f,
            },
            vk::VertexInputAttributeDescription {
                location: 6,
                binding: 0,
                format: vk::Format::R32_SFLOAT,
                offset: 15 * f,
            },
            vk::VertexInputAttributeDescription {
                location: 7,
                binding: 0,
                format: vk::Format::R32_SFLOAT,
                offset: 16 * f,
            },
        ]
    }
}
