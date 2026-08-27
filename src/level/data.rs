//! Pure data types for level file deserialization.
//!
//! These structs map directly to the RON level file format.
//!
//! # Coordinate frames
//!
//! A level is a set of **segments**, each authored entirely in its own local
//! frame. `Terrain` extents, anchors and objects are all segment-local as
//! written, and `player_spawn` is local to the root segment.
//!
//! [`load_level`] resolves the placement tree and then **rewrites the
//! world-space quantities in place**: `Level::frames` is filled in, and every
//! object position and the player spawn are lifted into world coordinates.
//! Terrain extents are deliberately *not* rewritten — generation is local, which
//! is what makes a segment relocatable.
//!
//! [`load_level`]: super::loader::load_level

use nalgebra::Point3;
use serde::Deserialize;

use crate::collision::AABB;
use crate::terrain::SegmentFrame;

use super::footprint::{Footprint, Support};

use crate::app::creatures::RollerDef;
use crate::app::spawnables::{
    BananaDef, BeachBallDef, BoxDef, BoxWallDef, CapsuleDef, CrateDef, DodecahedronDef, DolosDef,
    DominoDef, FencePostDef, GlowingOrbDef, HeavyCrateDef, HexPrismDef, HoneycombWallDef, HouseDef,
    IcosahedronDef, JackDef, JengaDef, MenhirDef, MovingPlatformDef, OctahedronDef, PendulumDef,
    PlankBridgeDef, PlankDef, PlayWheelDef, PyramidDef, SeesawDef, Spawnable, StackDef,
    StackItemDef, TableDef, TempleDef, TetrahedronDef, TowerDef, TrampolineDef, TrilithonDef,
    VoussoirArchDef, BEACH_BALL_RADIUS,
};

/// Top-level level description.
#[derive(Deserialize)]
pub struct Level {
    pub name: String,

    /// The areas the level is built from, in declaration order.
    pub segments: Vec<SegmentDef>,

    /// How segments are positioned. Exactly one entry must be a `Root`; every
    /// other segment needs exactly one `Join`.
    pub placements: Vec<Placement>,

    /// Anchor pairs that meet but derive nothing — the shortcut back to an
    /// earlier area, the second bridge across a chasm. Assertions, checked by
    /// `level_check`, never used to place anything.
    #[serde(default)]
    pub connections: Vec<Connection>,

    /// Authored in the root segment's local frame; rewritten to world
    /// coordinates by `load_level`.
    pub player_spawn: (f32, f32, f32),

    /// Water configuration. If omitted, no water system is created.
    #[serde(default)]
    pub water: Option<WaterConfig>,

    /// World frame of each segment, parallel to `segments`.
    ///
    /// Derived from `placements` by `load_level`; empty on a `Level` that was
    /// deserialized directly without going through the loader.
    #[serde(skip)]
    pub frames: Vec<SegmentFrame>,
}

impl Level {
    /// Every object in the level, paired with the index of its owning segment.
    pub fn objects(&self) -> impl Iterator<Item = (usize, &LevelObject)> {
        self.segments
            .iter()
            .enumerate()
            .flat_map(|(i, s)| s.objects.iter().map(move |o| (i, o)))
    }

    /// Total object count across every segment.
    pub fn object_count(&self) -> usize {
        self.segments.iter().map(|s| s.objects.len()).sum()
    }

    /// Index of a segment by name.
    pub fn segment_index(&self, name: &str) -> Option<usize> {
        self.segments.iter().position(|s| s.name == name)
    }

    /// World frame of a segment, once placement has been resolved.
    pub fn frame(&self, index: usize) -> SegmentFrame {
        self.frames
            .get(index)
            .copied()
            .unwrap_or_else(SegmentFrame::identity)
    }
}

/// One authored area: a coordinate frame's worth of terrain, anchors and
/// objects, all in that frame's local coordinates.
#[derive(Deserialize)]
pub struct SegmentDef {
    /// Unique within the level. Anchors are referred to as `segment.anchor`.
    pub name: String,
    /// Terrain generated in this segment's local frame.
    pub terrain: Terrain,
    /// Named local frames, for joining segments to each other.
    #[serde(default)]
    pub anchors: Vec<AnchorDef>,
    /// Objects authored in this segment's local frame.
    #[serde(default)]
    pub objects: Vec<LevelObject>,
}

/// A named local frame within a segment.
///
/// The anchor's local `+X` points **outward**, out of the segment — see
/// `terrain::anchor` for the diagram. `yaw` must be a multiple of 90°.
#[derive(Deserialize)]
pub struct AnchorDef {
    pub name: String,
    /// Segment-local position.
    pub pos: (f32, f32, f32),
    /// Segment-local yaw in degrees, a multiple of 90°. Defaults to 0, which
    /// faces outward along local `+X`.
    #[serde(default)]
    pub yaw: f32,
}

/// How one segment gets its world frame.
///
/// Placement is a spanning tree: exactly one `Root`, and one `Join` per
/// remaining segment. Two placements for the same segment is over-determined
/// and rejected; none is an orphan and rejected; a loop is a cycle and rejected.
/// Anything else that connects two anchors is a [`Connection`], not a placement.
#[derive(Deserialize)]
pub enum Placement {
    /// The one segment placed at an explicit world transform.
    Root {
        segment: String,
        #[serde(default)]
        origin: (f32, f32, f32),
        /// World yaw in degrees, a multiple of 90°.
        #[serde(default)]
        yaw: f32,
    },
    /// Snap this segment's `anchor` onto an already-placed `to` anchor,
    /// separated by `gap` metres.
    Join {
        segment: String,
        /// Anchor on `segment`.
        anchor: String,
        /// The anchor to mate with, as `segment.anchor`.
        to: String,
        /// Separation along the parent anchor's outward direction, in metres.
        gap: f32,
        /// Request continuous terrain across the join rather than a gap.
        ///
        /// **Not implemented** — rejected at load. Marching cubes reads an
        /// unallocated neighbour as air, so a welded boundary chunk emits a cap
        /// surface sealing the join. Cross-segment halo sampling is separate
        /// work; until it exists, express continuous structures as spawnables.
        #[serde(default)]
        weld: bool,
    },
}

/// An assertion that two anchors meet, deriving nothing.
///
/// `level_check` verifies the two anchors really do end up `gap` apart and
/// facing each other, and measures `gap` against the player's jump reach.
#[derive(Deserialize)]
pub struct Connection {
    /// `segment.anchor`.
    pub from: String,
    /// `segment.anchor`.
    pub to: String,
    /// Expected separation in metres.
    pub gap: f32,
}

/// An axis-aligned box, authored as two corners.
#[derive(Deserialize, Clone, Copy)]
pub struct Extent {
    pub min: (f32, f32, f32),
    pub max: (f32, f32, f32),
}

impl Extent {
    pub fn to_aabb(self) -> AABB {
        AABB::new(
            Point3::new(self.min.0, self.min.1, self.min.2),
            Point3::new(self.max.0, self.max.1, self.max.2),
        )
    }
}

/// Terrain description: a heightfield plus optional volumetric features.
#[derive(Deserialize)]
pub struct Terrain {
    /// Edge length of one voxel, in world units. Used directly — chunk depth is
    /// fixed by the chunk definition, so this is not scaled by any extent.
    pub voxel_size: f32,
    /// The region terrain is generated within. Chunks are allocated on demand,
    /// so an extent larger than the content costs storage only where filled.
    pub bounds: Extent,
    /// Base surface height before any features are applied.
    pub base_height: f32,
    /// Material layering by depth below the surface.
    #[serde(default)]
    pub material_layers: Vec<MaterialLayer>,
    /// Thickness of the indestructible bedrock band at the bottom of `bounds`.
    ///
    /// The floor of the world rather than a durability tier: it exists so the
    /// player cannot dig out of the level. Everything above it yields to a big
    /// enough charge. Zero leaves the level open at the bottom.
    #[serde(default = "default_bedrock_thickness")]
    pub bedrock_thickness: f32,
    /// Heightfield features — modify surface height at each (x, z) column.
    pub features: Vec<TerrainFeature>,
    /// Volumetric features — place or carve voxels in 3D (evaluated after heightfield).
    #[serde(default)]
    pub volumes: Vec<VolumeFeature>,
}

/// Two voxels of bedrock at a 1.0 voxel size — enough that a charge cannot
/// reach through it from above.
fn default_bedrock_thickness() -> f32 {
    2.0
}

/// Maps a depth range below the terrain surface to a voxel material.
#[derive(Deserialize)]
pub struct MaterialLayer {
    /// Maximum depth (from surface) at which this material appears — the
    /// layer's lower boundary, not its thickness. The first layer deeper than a
    /// sample wins, so a list whose depths do not increase leaves every layer
    /// after the first unreachable, silently.
    pub depth: f32,
    pub material: VoxelMaterialId,
}

/// Voxel material identifiers matching the engine's `VoxelMaterial` enum.
#[derive(Deserialize, Clone, Copy)]
pub enum VoxelMaterialId {
    Grass,
    Dirt,
    Rock,
    Ite,
    Limestone,
    Slate,
    Sand,
}

/// Heightfield features operate on 2D (xz) coordinates, modifying the terrain
/// surface height additively.
#[derive(Deserialize)]
pub enum TerrainFeature {
    /// Smooth dome with cosine falloff.
    Hill {
        center: (f32, f32),
        radius: f32,
        height: f32,
    },
    /// Inverted dome — subtracts height.
    Crater {
        center: (f32, f32),
        radius: f32,
        depth: f32,
    },
    /// Flat rectangular region set to a fixed height.
    Plateau {
        min: (f32, f32),
        max: (f32, f32),
        height: f32,
    },
    /// Thin raised strip along a line segment.
    Wall {
        from: (f32, f32),
        to: (f32, f32),
        height: f32,
        thickness: f32,
    },
    /// Linear height gradient between two points.
    Ramp {
        from: (f32, f32),
        to: (f32, f32),
        start_height: f32,
        end_height: f32,
        width: f32,
    },
    /// FBM noise layer for natural surface variation.
    TerrainRoughness {
        frequency: f32,
        amplitude: f32,
        octaves: u32,
        seed: u32,
    },
    /// Sharp height transition along a line segment, creating a cliff face.
    ///
    /// Terrain on the `high_side` of the edge is at `high_height`; terrain on
    /// the opposite side is at `low_height`. The transition between them is
    /// controlled by `steepness` — higher values produce a narrower, more
    /// vertical face.
    Cliff {
        /// Start of the cliff edge line.
        from: (f32, f32),
        /// End of the cliff edge line.
        to: (f32, f32),
        /// Terrain height on the low side.
        low_height: f32,
        /// Terrain height on the high side.
        high_height: f32,
        /// Direction (xz) pointing toward the high side of the cliff.
        /// Does not need to be normalised.
        high_side: (f32, f32),
        /// Controls how narrow the height transition is. Higher values
        /// produce a steeper, more vertical cliff face.
        steepness: f32,
        /// Distance over which the cliff effect fades to nothing beyond each
        /// endpoint of the edge line. 0.0 = cliff extends infinitely.
        #[serde(default)]
        end_falloff: f32,
        /// Optional noise roughness applied to the cliff face. 0.0 = smooth.
        #[serde(default)]
        roughness: f32,
        /// Noise seed for face roughness.
        #[serde(default = "default_cliff_seed")]
        roughness_seed: u32,
    },
}

fn default_cliff_seed() -> u32 {
    42
}

/// A point on a depth-vs-threshold curve for cave generation.
///
/// The carve threshold is linearly interpolated between consecutive points.
/// At a given depth below the surface, if the 3D noise value exceeds the
/// interpolated threshold, the voxel is carved to air.
#[derive(Deserialize, Clone)]
pub struct CaveDepthPoint {
    /// Depth below the terrain surface.
    pub depth: f32,
    /// Noise threshold at this depth (0.0 = carve everything, 1.0 = carve nothing).
    pub threshold: f32,
}

/// Spatial region that controls where caves are generated.
///
/// Caves are strongest within `radius` of the center point, then fade out
/// over the `falloff` distance by blending the carve threshold toward 1.0
/// (no carving). Outside `radius + falloff`, no caves are generated.
#[derive(Deserialize, Clone)]
pub struct CaveRegion {
    /// Center of the cave region in world coordinates (x, y, z).
    pub center: (f32, f32, f32),
    /// Radius within which caves are at full strength.
    pub radius: f32,
    /// Distance beyond `radius` over which caves fade to nothing.
    pub falloff: f32,
}

/// Volumetric features operate in 3D, directly placing or carving voxels.
/// Applied as a second pass after the heightfield.
#[derive(Deserialize)]
pub enum VolumeFeature {
    /// Floating solid mass with optional noisy edges for an organic look.
    Island {
        center: (f32, f32, f32),
        half_extents: (f32, f32, f32),
        edge_noise: f32,
    },
    /// Curved bridge between two 3D points.
    Arch {
        from: (f32, f32, f32),
        to: (f32, f32, f32),
        radius: f32,
        thickness: f32,
    },
    /// Vertical column rising from the heightfield surface.
    Pillar {
        center: (f32, f32),
        /// Absolute Y of the column's top, **not** its length. A pillar on a
        /// bench at y = 6 given `height: 12` stands 6 m tall, not 12.
        height: f32,
        radius: f32,
    },
    /// Horizontal bore that carves through existing terrain.
    Tunnel {
        center: (f32, f32),
        direction: (f32, f32),
        length: f32,
        radius: f32,
        depth: f32,
    },
    /// 3D noise-driven cave carving with depth-dependent threshold.
    ///
    /// Carves overhangs near the surface, cave networks at medium depth,
    /// and leaves deep rock solid. The `depth_curve` controls the carve
    /// threshold at each depth below the heightfield surface.
    Caves {
        /// Noise sample frequency — lower values produce larger caverns.
        frequency: f32,
        /// Number of FBM octaves — more octaves add finer wall detail.
        octaves: u32,
        /// Noise seed for reproducibility.
        seed: u32,
        /// Piecewise-linear curve mapping depth below surface to carve threshold.
        /// Must be sorted by depth ascending. Noise values above the threshold
        /// at a given depth will be carved.
        depth_curve: Vec<CaveDepthPoint>,
        /// Optional spatial region with soft falloff. If omitted, caves span
        /// the entire terrain.
        #[serde(default)]
        region: Option<CaveRegion>,
        /// Material layering for cave surfaces, by depth from the cave wall.
        /// If empty, falls back to the terrain's material layers.
        #[serde(default)]
        material_layers: Vec<MaterialLayer>,
        /// Downward bias applied below the local cave midpoint to flatten
        /// cave floors, making them more walkable. 0.0 = no flattening.
        #[serde(default)]
        floor_bias: f32,
    },
    /// Solid rock lip extending horizontally from a cliff edge, creating
    /// a dramatic overhang. Pairs with `TerrainFeature::Cliff`.
    Overhang {
        /// Start of the cliff edge line (xz).
        from: (f32, f32),
        /// End of the cliff edge line (xz).
        to: (f32, f32),
        /// Y level of the overhang (typically the cliff's `high_height`).
        height: f32,
        /// How far the lip extends outward from the cliff edge.
        depth: f32,
        /// Vertical thickness of the lip at the cliff edge.
        thickness: f32,
        /// Direction (xz) the lip extends toward (away from the cliff top).
        /// Does not need to be normalised.
        direction: (f32, f32),
        /// Noise amplitude for organic underside roughness. 0.0 = smooth.
        #[serde(default)]
        noise: f32,
        /// Noise seed for underside roughness.
        #[serde(default = "default_overhang_seed")]
        noise_seed: u32,
    },

    // --- Traversal primitives ---------------------------------------------
    //
    // Everything above shapes landscape. Everything below shapes a *route*:
    // deliberate, walkable geometry authored against the player's reach. They
    // are volume features rather than a category of their own because that
    // gets them segment-local generation, destructibility, physics through
    // `StaticGeometry` and rendering for free — see
    // `terrain::traversal` for the fields themselves.
    /// A walkable deck swept along a polyline: catwalk, bridge, cliff ledge,
    /// spiral ramp. Waypoints carry their own heights, so a route that climbs
    /// while it turns is one `Path` rather than a construction of many.
    Path {
        /// Waypoints in segment-local coordinates. At least two. Each `y` is
        /// the **walking surface**, not the middle of the deck.
        points: Vec<(f32, f32, f32)>,
        /// Full width of the deck.
        width: f32,
        /// How far the deck extends below its walking surface.
        #[serde(default = "default_deck_thickness")]
        thickness: f32,
        /// Cross-section shape.
        #[serde(default)]
        profile: PathProfile,
        #[serde(default = "default_route_material")]
        material: VoxelMaterialId,
    },

    /// A free-standing slab at a height — the atom of a jump sequence.
    Platform {
        /// Centre of the walking surface; its `y` is the surface height.
        center: (f32, f32, f32),
        /// Half-extents of the deck along local x and z.
        half_extents: (f32, f32),
        /// How far the slab extends below its walking surface.
        #[serde(default = "default_deck_thickness")]
        thickness: f32,
        #[serde(default = "default_route_material")]
        material: VoxelMaterialId,
    },

    /// Discrete steps between two heights.
    ///
    /// Not a ramp: the rise per step is what the player has to clear, and that
    /// is a number `level_check` can hold against the jump apex.
    Staircase {
        /// Foot of the flight: the level the first riser rises *from*.
        from: (f32, f32, f32),
        /// Head of the flight: the surface level of the last tread. Only its
        /// height and horizontal position are used; the run is the horizontal
        /// distance between the two.
        to: (f32, f32, f32),
        /// Full width of the treads.
        width: f32,
        /// Number of treads. The rise per step is `(to.y - from.y) / steps`.
        steps: u32,
        /// How far the flight extends below the lower of the two ends.
        #[serde(default = "default_deck_thickness")]
        thickness: f32,
        #[serde(default = "default_route_material")]
        material: VoxelMaterialId,
    },

    /// A vertical bore, for a tower or a descent, with an optional helical
    /// ledge spiralling down its wall.
    Shaft {
        /// Bore centre in local (x, z).
        center: (f32, f32),
        /// Bottom of the bore.
        from_y: f32,
        /// Top of the bore.
        to_y: f32,
        /// Bore radius.
        radius: f32,
        /// The route down the inside. Without one the shaft is a hole.
        #[serde(default)]
        ledge: Option<ShaftLedge>,
        #[serde(default = "default_route_material")]
        material: VoxelMaterialId,
    },
}

fn default_overhang_seed() -> u32 {
    99
}

/// Deck thickness used when a traversal primitive does not state one.
///
/// Thick enough to read as a structure from below at 0.5 m voxels rather than
/// as a floating sheet.
fn default_deck_thickness() -> f32 {
    1.0
}

/// Material used when a traversal primitive does not state one.
fn default_route_material() -> VoxelMaterialId {
    VoxelMaterialId::Rock
}

/// What a traversal primitive claims, in the terms an offline check can act on.
///
/// The mirror of [`LevelObject::describe`]: the primitives vary in shape, but
/// every check `level_check` runs over them reduces to one of these four
/// numbers, and keeping the reduction next to the format stops the check and
/// the generator drifting apart about what a "width" is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TraversalInfo {
    /// The RON variant name, e.g. `"Path"`.
    pub kind: &'static str,
    /// The narrowest authored dimension, in metres. Decides whether the
    /// primitive can read cleanly at its segment's voxel resolution.
    pub finest_detail: f32,
    /// Width of the surface the player walks on, where the primitive has one.
    pub walkable_width: Option<f32>,
    /// Height gained in a single move, where the primitive asks the player to
    /// climb — a staircase's rise per step.
    pub step_rise: Option<f32>,
}

impl VolumeFeature {
    /// Describe this feature if it is a traversal primitive.
    ///
    /// `None` for the landscape features: they shape ground, and nothing here
    /// is meaningful for them.
    pub fn traversal_info(&self) -> Option<TraversalInfo> {
        let info = match self {
            VolumeFeature::Path {
                width, thickness, ..
            } => TraversalInfo {
                kind: "Path",
                finest_detail: width.min(*thickness),
                walkable_width: Some(*width),
                step_rise: None,
            },

            VolumeFeature::Platform {
                half_extents,
                thickness,
                ..
            } => {
                let (x, z) = (half_extents.0 * 2.0, half_extents.1 * 2.0);
                TraversalInfo {
                    kind: "Platform",
                    finest_detail: x.min(z).min(*thickness),
                    walkable_width: Some(x.min(z)),
                    step_rise: None,
                }
            }

            VolumeFeature::Staircase {
                from,
                to,
                width,
                steps,
                thickness,
                ..
            } => {
                let steps = (*steps).max(1) as f32;
                TraversalInfo {
                    kind: "Staircase",
                    // Width and thickness only. The rise and the tread run are
                    // the *pattern*, not the resolution: a one-metre riser at
                    // one-metre voxels is a perfectly good single-voxel step,
                    // and folding it in here would demand a rise no player
                    // could climb.
                    finest_detail: width.min(*thickness),
                    walkable_width: Some(*width),
                    step_rise: Some((to.1 - from.1).abs() / steps),
                }
            }

            VolumeFeature::Shaft { radius, ledge, .. } => {
                let bore = radius * 2.0;
                match ledge {
                    Some(l) => TraversalInfo {
                        kind: "Shaft",
                        finest_detail: bore.min(l.width).min(l.thickness),
                        walkable_width: Some(l.width),
                        step_rise: None,
                    },
                    None => TraversalInfo {
                        kind: "Shaft",
                        finest_detail: bore,
                        walkable_width: None,
                        step_rise: None,
                    },
                }
            }

            _ => return None,
        };
        Some(info)
    }
}

/// Cross-section of a swept [`VolumeFeature::Path`].
#[derive(Deserialize, Clone, Copy, Default, Debug, PartialEq, Eq)]
pub enum PathProfile {
    /// Rectangular: square edges, the read of a built catwalk.
    #[default]
    Flat,
    /// Flat walking surface over a semi-elliptical underside: the read of a
    /// stone bridge, or of a ledge weathered out of a cliff.
    Rounded,
}

/// The helical ledge running down the inside of a [`VolumeFeature::Shaft`].
#[derive(Deserialize, Clone, Copy)]
pub struct ShaftLedge {
    /// Walkable width, measured inward from the bore wall. Must not exceed the
    /// bore radius.
    pub width: f32,
    /// Vertical thickness of the ledge slab.
    pub thickness: f32,
    /// Height gained per full turn. Sets how steep the descent is: at a 6 m
    /// bore radius, a 4 m pitch is a gentle spiral and a 12 m pitch is a
    /// scramble.
    pub pitch: f32,
    /// Local yaw in degrees at which the helix passes through `from_y`.
    #[serde(default)]
    pub start_angle: f32,
}

/// Visual style for box textures.
#[derive(Deserialize, Clone, Copy, Default)]
pub enum BoxStyle {
    WoodenCrate,
    Cardboard,
    Metal,
    Gift,
    Stone,
    Brick,
    Warning,
    #[default]
    Random,
}

/// Objects that can be placed in a level.
///
/// Each variant maps to a RON-compatible format. The [`LevelObject::to_spawnable`]
/// method converts any variant into a boxed [`Spawnable`] implementation.
/// To add a new object type:
/// 1. Create its `*Def` struct with a `Spawnable` impl in `src/app/spawnables/`.
/// 2. Add a variant here with the same fields.
/// 3. Add a conversion arm in `to_spawnable()`.
#[derive(Deserialize)]
pub enum LevelObject {
    /// Curved fruit: swept mesh, capsule-chain collider, very low friction.
    Banana {
        pos: (f32, f32, f32),
        #[serde(default = "BananaDef::default_length")]
        length: f32,
        #[serde(default = "BananaDef::default_thickness")]
        thickness: f32,
        #[serde(default = "BananaDef::default_curvature")]
        curvature: f32,
        #[serde(default = "BananaDef::default_ripeness")]
        ripeness: f32,
        #[serde(default = "BananaDef::default_density")]
        density: f32,
        #[serde(default = "BananaDef::default_restitution")]
        restitution: f32,
        #[serde(default = "BananaDef::default_friction")]
        friction: f32,
    },
    BeachBall {
        pos: (f32, f32, f32),
    },
    /// Self-illuminated sphere with beach-ball physics. Subject for lighting work.
    GlowingOrb {
        pos: (f32, f32, f32),
        /// Surface and glow tint. Defaults to cyan.
        #[serde(default)]
        colour: Option<(f32, f32, f32)>,
        /// Emitted luminance. Hue-independent: the same value is the same
        /// brightness for any `colour`, and blooms above the bloom threshold.
        #[serde(default)]
        glow: Option<f32>,
    },
    /// Raw box with full control over dimensions, appearance, and physics.
    Box {
        pos: (f32, f32, f32),
        half_extents: (f32, f32, f32),
        /// Rotation about `+Y` in degrees, within the segment's frame.
        #[serde(default)]
        yaw: f32,
        #[serde(default)]
        style: BoxStyle,
        #[serde(default = "BoxDef::default_density")]
        density: f32,
        #[serde(default = "BoxDef::default_restitution")]
        restitution: f32,
        #[serde(default = "BoxDef::default_friction")]
        friction: f32,
    },
    /// Thin wooden board. Defaults: style=WoodenCrate, density=20, thickness=0.1.
    Plank {
        pos: (f32, f32, f32),
        length: f32,
        width: f32,
        /// Rotation about `+Y` in degrees, within the segment's frame.
        #[serde(default)]
        yaw: f32,
    },
    /// Standard crate. Defaults: style=WoodenCrate, density=50.
    Crate {
        pos: (f32, f32, f32),
        /// Cube half-extent.
        size: f32,
    },
    /// Dense crate. Defaults: style=Metal, density=150.
    HeavyCrate {
        pos: (f32, f32, f32),
        size: f32,
    },
    /// Vertical stack — auto-computes Y positions bottom-up from base.
    Stack {
        base: (f32, f32, f32),
        items: Vec<StackItem>,
        /// Rotation about `+Y` in degrees, applied to every item that has an
        /// axis (the planks).
        #[serde(default)]
        yaw: f32,
    },
    /// Shorthand for N identical boxes stacked vertically.
    Tower {
        base: (f32, f32, f32),
        box_half_extents: (f32, f32, f32),
        count: u32,
        /// Rotation about `+Y` in degrees. Meaningful whenever the boxes are
        /// not cubes.
        #[serde(default)]
        yaw: f32,
        #[serde(default = "TowerDef::default_density")]
        density: f32,
    },
    /// Grid of boxes, optionally staggered for a brick-like pattern.
    BoxWall {
        base: (f32, f32, f32),
        box_half_extents: (f32, f32, f32),
        columns: u32,
        rows: u32,
        /// Rotation about `+Y` in degrees — which way the wall faces.
        #[serde(default)]
        yaw: f32,
        #[serde(default = "BoxWallDef::default_density")]
        density: f32,
        #[serde(default)]
        stagger: bool,
    },
    /// Prefab house structure built from boxes.
    House {
        pos: (f32, f32, f32),
        half_extents: (f32, f32, f32),
    },
    /// Capsule (cylinder + hemisphere caps).
    Capsule {
        pos: (f32, f32, f32),
        half_height: f32,
        radius: f32,
        #[serde(default = "CapsuleDef::default_density")]
        density: f32,
        #[serde(default = "CapsuleDef::default_restitution")]
        restitution: f32,
        #[serde(default = "CapsuleDef::default_friction")]
        friction: f32,
    },
    /// Standing stone anchored to terrain. Released when terrain is destroyed.
    Menhir {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "MenhirDef::default_half_height")]
        half_height: f32,
        #[serde(default = "MenhirDef::default_bottom_radius")]
        bottom_radius: f32,
        #[serde(default = "MenhirDef::default_top_radius")]
        top_radius: f32,
        #[serde(default = "MenhirDef::default_density")]
        density: f32,
    },
    /// Vertical post anchored to terrain. Released when terrain is destroyed.
    FencePost {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "FencePostDef::default_half_height")]
        half_height: f32,
        #[serde(default = "FencePostDef::default_radius")]
        radius: f32,
        #[serde(default = "FencePostDef::default_density")]
        density: f32,
    },
    /// Roller — a rolling boulder creature that hunts the player.
    ///
    /// Not a prop: it carries a brain and hit points, and it will come after
    /// whoever placed it. See `src/app/creatures/`.
    Roller {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "RollerDef::default_radius")]
        radius: f32,
        #[serde(default = "RollerDef::default_speed")]
        speed: f32,
        #[serde(default = "RollerDef::default_spin_up_time")]
        spin_up_time: f32,
        #[serde(default = "RollerDef::default_sight_range")]
        sight_range: f32,
    },
    /// Pendulum — terrain-anchored frame with a swinging ball.
    Pendulum {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "PendulumDef::default_frame_height")]
        frame_height: f32,
        #[serde(default = "PendulumDef::default_arm_length")]
        arm_length: f32,
        #[serde(default = "PendulumDef::default_rope_length")]
        rope_length: f32,
        #[serde(default = "PendulumDef::default_ball_radius")]
        ball_radius: f32,
        #[serde(default = "PendulumDef::default_ball_density")]
        ball_density: f32,
    },
    /// Powered platform shuttling between two authored points under its own
    /// motor. Unanchored: a blast can shove it off its route and it flies back,
    /// but stripping its drive drops it out of the sky for good.
    MovingPlatform {
        /// Where the platform spawns and the end it returns to. Explicit Y —
        /// the travel is authored, not derived from the ground under it.
        from: (f32, f32, f32),
        /// The far end of the run.
        to: (f32, f32, f32),
        #[serde(default = "MovingPlatformDef::default_half_extents")]
        half_extents: (f32, f32, f32),
        #[serde(default = "MovingPlatformDef::default_speed")]
        speed: f32,
    },
    /// Spinning playground wheel anchored to terrain. Spins freely around Y.
    PlayWheel {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "PlayWheelDef::default_radius")]
        radius: f32,
        #[serde(default = "PlayWheelDef::default_thickness")]
        thickness: f32,
        #[serde(default = "PlayWheelDef::default_density")]
        density: f32,
        #[serde(default = "PlayWheelDef::default_hub_height")]
        hub_height: f32,
    },
    /// Seesaw — tilting beam with seats, anchored at fulcrum.
    Seesaw {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "SeesawDef::default_beam_half_length")]
        beam_half_length: f32,
        #[serde(default = "SeesawDef::default_beam_half_width")]
        beam_half_width: f32,
        #[serde(default = "SeesawDef::default_beam_half_thickness")]
        beam_half_thickness: f32,
        #[serde(default = "SeesawDef::default_fulcrum_height")]
        fulcrum_height: f32,
        #[serde(default = "SeesawDef::default_density")]
        density: f32,
    },
    /// Regular tetrahedron with ConvexHull collider.
    Tetrahedron {
        pos: (f32, f32, f32),
        #[serde(default = "TetrahedronDef::default_size")]
        size: f32,
        #[serde(default = "TetrahedronDef::default_density")]
        density: f32,
        #[serde(default = "TetrahedronDef::default_restitution")]
        restitution: f32,
        #[serde(default = "TetrahedronDef::default_friction")]
        friction: f32,
    },
    /// Regular octahedron with ConvexHull collider.
    Octahedron {
        pos: (f32, f32, f32),
        #[serde(default = "OctahedronDef::default_size")]
        size: f32,
        #[serde(default = "OctahedronDef::default_density")]
        density: f32,
        #[serde(default = "OctahedronDef::default_restitution")]
        restitution: f32,
        #[serde(default = "OctahedronDef::default_friction")]
        friction: f32,
    },
    /// Regular dodecahedron with ConvexHull collider.
    Dodecahedron {
        pos: (f32, f32, f32),
        #[serde(default = "DodecahedronDef::default_size")]
        size: f32,
        #[serde(default = "DodecahedronDef::default_density")]
        density: f32,
        #[serde(default = "DodecahedronDef::default_restitution")]
        restitution: f32,
        #[serde(default = "DodecahedronDef::default_friction")]
        friction: f32,
    },
    /// Hexagonal prism with ConvexHull collider.
    HexPrism {
        pos: (f32, f32, f32),
        #[serde(default = "HexPrismDef::default_radius")]
        radius: f32,
        #[serde(default = "HexPrismDef::default_half_height")]
        half_height: f32,
        #[serde(default = "HexPrismDef::default_density")]
        density: f32,
        #[serde(default = "HexPrismDef::default_restitution")]
        restitution: f32,
        #[serde(default = "HexPrismDef::default_friction")]
        friction: f32,
    },
    /// Honeycomb wall — tiled hexagonal prisms in a honeycomb grid.
    HoneycombWall {
        base: (f32, f32, f32),
        #[serde(default = "HoneycombWallDef::default_columns")]
        columns: u32,
        /// Rotation about `+Y` in degrees — which way the wall faces.
        #[serde(default)]
        yaw: f32,
        #[serde(default = "HoneycombWallDef::default_rows")]
        rows: u32,
        #[serde(default = "HexPrismDef::default_radius")]
        radius: f32,
        #[serde(default = "HexPrismDef::default_half_height")]
        half_height: f32,
        #[serde(default = "HexPrismDef::default_density")]
        density: f32,
    },
    /// Regular icosahedron with ConvexHull collider.
    Icosahedron {
        pos: (f32, f32, f32),
        #[serde(default = "IcosahedronDef::default_size")]
        size: f32,
        #[serde(default = "IcosahedronDef::default_density")]
        density: f32,
        #[serde(default = "IcosahedronDef::default_restitution")]
        restitution: f32,
        #[serde(default = "IcosahedronDef::default_friction")]
        friction: f32,
    },
    /// Trampoline — bouncy pad on four short legs.
    Trampoline {
        pos: (f32, f32, f32),
        /// Rotation about `+Y` in degrees, within the segment's frame.
        #[serde(default)]
        yaw: f32,
        #[serde(default = "TrampolineDef::default_pad_half_extents")]
        pad_half_extents: (f32, f32, f32),
        #[serde(default = "TrampolineDef::default_leg_half_extents")]
        leg_half_extents: (f32, f32, f32),
        #[serde(default = "TrampolineDef::default_density")]
        density: f32,
        #[serde(default = "TrampolineDef::default_restitution")]
        restitution: f32,
    },
    /// Table — compound body (top slab + 4 legs).
    Table {
        pos: (f32, f32, f32),
        /// Rotation about `+Y` in degrees, within the segment's frame.
        #[serde(default)]
        yaw: f32,
        /// Half-extents of the table top (x, y_thickness, z).
        #[serde(default = "TableDef::default_top_half_extents")]
        top_half_extents: (f32, f32, f32),
        /// Half-extents of each leg.
        #[serde(default = "TableDef::default_leg_half_extents")]
        leg_half_extents: (f32, f32, f32),
        #[serde(default = "TableDef::default_density")]
        density: f32,
        #[serde(default = "TableDef::default_restitution")]
        restitution: f32,
        #[serde(default = "TableDef::default_friction")]
        friction: f32,
    },
    /// Pyramid — square-based pyramid of sandstone blocks.
    Pyramid {
        base: (f32, f32, f32),
        block_half_extents: (f32, f32, f32),
        /// Number of blocks along each side of the bottom layer.
        base_width: u32,
        #[serde(default = "PyramidDef::default_density")]
        density: f32,
    },
    /// Dolos — concrete breakwater armour unit. Central shank with two
    /// perpendicular flukes, one at each end.
    Dolos {
        pos: (f32, f32, f32),
        /// Rotation about `+Y` in degrees, within the segment's frame.
        #[serde(default)]
        yaw: f32,
        #[serde(default = "DolosDef::default_shank_length")]
        shank_length: f32,
        #[serde(default = "DolosDef::default_fluke_length")]
        fluke_length: f32,
        #[serde(default = "DolosDef::default_thickness")]
        thickness: f32,
        #[serde(default = "DolosDef::default_density")]
        density: f32,
        #[serde(default = "DolosDef::default_restitution")]
        restitution: f32,
        #[serde(default = "DolosDef::default_friction")]
        friction: f32,
    },
    /// Domino row — tall thin blocks spaced for chain toppling.
    Domino {
        base: (f32, f32, f32),
        /// Direction the row extends in (x, z).
        direction: (f32, f32),
        #[serde(default = "DominoDef::default_count")]
        count: u32,
        #[serde(default = "DominoDef::default_spacing")]
        spacing: f32,
        #[serde(default = "DominoDef::default_half_extents")]
        half_extents: (f32, f32, f32),
        #[serde(default = "DominoDef::default_density")]
        density: f32,
    },
    /// Voussoir arch — semicircular masonry arch with abutment pillars.
    VoussoirArch {
        base: (f32, f32, f32),
        #[serde(default = "VoussoirArchDef::default_inner_radius")]
        inner_radius: f32,
        #[serde(default = "VoussoirArchDef::default_thickness")]
        thickness: f32,
        #[serde(default = "VoussoirArchDef::default_depth")]
        depth: f32,
        #[serde(default = "VoussoirArchDef::default_num_voussoirs")]
        num_voussoirs: u32,
        #[serde(default = "VoussoirArchDef::default_abutment_height")]
        abutment_height: f32,
        #[serde(default = "VoussoirArchDef::default_density")]
        density: f32,
        #[serde(default = "VoussoirArchDef::default_friction")]
        friction: f32,
    },
    /// Jack — classic six-pointed metal toy. Three perpendicular bars.
    Jack {
        pos: (f32, f32, f32),
        #[serde(default = "JackDef::default_length")]
        length: f32,
        #[serde(default = "JackDef::default_thickness")]
        thickness: f32,
        #[serde(default = "JackDef::default_density")]
        density: f32,
        #[serde(default = "JackDef::default_restitution")]
        restitution: f32,
        #[serde(default = "JackDef::default_friction")]
        friction: f32,
    },
    /// Jenga tower — alternating layers of three planks rotated 90°.
    Jenga {
        base: (f32, f32, f32),
        #[serde(default = "JengaDef::default_layers")]
        layers: u32,
        #[serde(default = "JengaDef::default_block_half_length")]
        block_half_length: f32,
        #[serde(default = "JengaDef::default_density")]
        density: f32,
        #[serde(default = "JengaDef::default_friction")]
        friction: f32,
    },
    /// Plank bridge — rustic destructible bridge with two beams and cross-planks.
    PlankBridge {
        pos: (f32, f32, f32),
        #[serde(default = "PlankBridgeDef::default_length")]
        length: f32,
        #[serde(default = "PlankBridgeDef::default_beam_spacing")]
        beam_spacing: f32,
        #[serde(default = "PlankBridgeDef::default_plank_count")]
        plank_count: u32,
        #[serde(default = "PlankBridgeDef::default_beam_half_extents")]
        beam_half_extents: (f32, f32, f32),
        #[serde(default = "PlankBridgeDef::default_plank_half_extents")]
        plank_half_extents: (f32, f32, f32),
        #[serde(default = "PlankBridgeDef::default_density")]
        density: f32,
        #[serde(default)]
        yaw: f32,
        #[serde(default = "PlankBridgeDef::default_fracture_threshold")]
        fracture_threshold: f32,
    },
    /// Neolithic trilithon — two uprights with a lintel.
    Trilithon {
        pos: (f32, f32, f32),
        /// Rotation about `+Y` in degrees — which way the doorway faces.
        #[serde(default)]
        yaw: f32,
        #[serde(default = "TrilithonDef::default_upright_half_height")]
        upright_half_height: f32,
        #[serde(default = "TrilithonDef::default_upright_half_width")]
        upright_half_width: f32,
        #[serde(default = "TrilithonDef::default_upright_half_depth")]
        upright_half_depth: f32,
        #[serde(default = "TrilithonDef::default_gap")]
        gap: f32,
        #[serde(default = "TrilithonDef::default_lintel_half_thickness")]
        lintel_half_thickness: f32,
        #[serde(default = "TrilithonDef::default_lintel_overhang")]
        lintel_overhang: f32,
        #[serde(default = "TrilithonDef::default_density")]
        density: f32,
    },
    /// Classical Greek Doric temple with peristyle colonnade.
    Temple {
        pos: (f32, f32, f32),
        #[serde(default = "TempleDef::default_column_height")]
        column_height: f32,
        #[serde(default = "TempleDef::default_front_columns")]
        front_columns: u32,
        #[serde(default = "TempleDef::default_side_columns")]
        side_columns: u32,
    },
}

/// How much ground a stack covers, taken from its widest item.
///
/// The bottom item is what actually touches the terrain, but the ones above it
/// are what fall off if the ground steps away — and an author who stacks a long
/// plank on a small crate means the plank to be over solid ground. The widest is
/// therefore the honest answer, and it is the conservative one.
fn stack_ground_half_extents(items: &[StackItem]) -> (f32, f32) {
    items
        .iter()
        .map(|item| match item {
            StackItem::Crate { size } | StackItem::HeavyCrate { size } => (*size, *size),
            StackItem::Plank { length, width } => (length * 0.5, width * 0.5),
            StackItem::BeachBall => (BEACH_BALL_RADIUS, BEACH_BALL_RADIUS),
            StackItem::Capsule { radius, .. } => (*radius, *radius),
        })
        .fold((0.0_f32, 0.0_f32), |acc, (x, z)| {
            (acc.0.max(x), acc.1.max(z))
        })
}

/// Where a level object is authored, as far as validation and schematics are
/// concerned.
///
/// Spawnables vary enormously in what they build, but every one of them is
/// placed either at an explicit point or by being dropped onto the terrain.
/// That distinction is the only thing an offline check can act on: a free
/// object's height is authored and can therefore be wrong, whereas an anchored
/// one's is derived and cannot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObjectPlacement {
    /// Authored at an explicit position.
    Free(Point3<f32>),
    /// Authored in (x, z) only; the spawner resolves the height from the
    /// terrain surface.
    TerrainAnchored { x: f32, z: f32 },
}

impl ObjectPlacement {
    /// Horizontal position, which both forms have.
    pub fn xz(&self) -> (f32, f32) {
        match self {
            ObjectPlacement::Free(p) => (p.x, p.z),
            ObjectPlacement::TerrainAnchored { x, z } => (*x, *z),
        }
    }
}

/// How an object responds to the yaw of the segment it is authored in.
///
/// See [`LevelObject::orientability`] for the audit this encodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientable {
    /// A quarter turn changes nothing observable.
    Symmetric,
    /// Carries a yaw, which the segment's turn is added to.
    Turns,
    /// Has a meaningful axis but no yaw yet: it keeps its world orientation
    /// while its segment turns around it.
    Fixed,
}

/// An object's variant name and authored placement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObjectInfo {
    /// The RON variant name, e.g. `"BeachBall"`.
    pub kind: &'static str,
    pub placement: ObjectPlacement,
    /// The ground it covers, not just the point it was authored at.
    pub footprint: Footprint,
    /// Whether that ground is expected to be continuous under it.
    pub support: Support,
}

impl LevelObject {
    /// Variant name and authored placement, for validation and schematics.
    ///
    /// Deliberately exhaustive rather than derived: adding a spawnable should
    /// force a decision about whether it is free-standing or terrain-anchored,
    /// because getting that wrong silently mis-validates every level using it.
    pub fn describe(&self) -> ObjectInfo {
        use ObjectPlacement::{Free, TerrainAnchored};

        let point = |p: &(f32, f32, f32)| Free(Point3::new(p.0, p.1, p.2));
        let anchored = |p: &(f32, f32)| TerrainAnchored { x: p.0, z: p.1 };

        let (kind, placement) = match self {
            LevelObject::Banana { pos, .. } => ("Banana", point(pos)),
            LevelObject::BeachBall { pos } => ("BeachBall", point(pos)),
            LevelObject::GlowingOrb { pos, .. } => ("GlowingOrb", point(pos)),
            LevelObject::Box { pos, .. } => ("Box", point(pos)),
            LevelObject::Plank { pos, .. } => ("Plank", point(pos)),
            LevelObject::Crate { pos, .. } => ("Crate", point(pos)),
            LevelObject::HeavyCrate { pos, .. } => ("HeavyCrate", point(pos)),
            LevelObject::Stack { base, .. } => ("Stack", point(base)),
            LevelObject::Tower { base, .. } => ("Tower", point(base)),
            LevelObject::BoxWall { base, .. } => ("BoxWall", point(base)),
            LevelObject::House { pos, .. } => ("House", point(pos)),
            LevelObject::Capsule { pos, .. } => ("Capsule", point(pos)),
            LevelObject::Menhir { pos, .. } => ("Menhir", anchored(pos)),
            LevelObject::FencePost { pos, .. } => ("FencePost", anchored(pos)),
            LevelObject::Roller { pos, .. } => ("Roller", anchored(pos)),
            LevelObject::Pendulum { pos, .. } => ("Pendulum", anchored(pos)),
            LevelObject::MovingPlatform { from, .. } => ("MovingPlatform", point(from)),
            LevelObject::PlayWheel { pos, .. } => ("PlayWheel", anchored(pos)),
            LevelObject::Seesaw { pos, .. } => ("Seesaw", anchored(pos)),
            LevelObject::Tetrahedron { pos, .. } => ("Tetrahedron", point(pos)),
            LevelObject::Octahedron { pos, .. } => ("Octahedron", point(pos)),
            LevelObject::Dodecahedron { pos, .. } => ("Dodecahedron", point(pos)),
            LevelObject::HexPrism { pos, .. } => ("HexPrism", point(pos)),
            LevelObject::HoneycombWall { base, .. } => ("HoneycombWall", point(base)),
            LevelObject::Icosahedron { pos, .. } => ("Icosahedron", point(pos)),
            LevelObject::Trampoline { pos, .. } => ("Trampoline", point(pos)),
            LevelObject::Table { pos, .. } => ("Table", point(pos)),
            LevelObject::Pyramid { base, .. } => ("Pyramid", point(base)),
            LevelObject::Dolos { pos, .. } => ("Dolos", point(pos)),
            LevelObject::Domino { base, .. } => ("Domino", point(base)),
            LevelObject::VoussoirArch { base, .. } => ("VoussoirArch", point(base)),
            LevelObject::Jack { pos, .. } => ("Jack", point(pos)),
            LevelObject::Jenga { base, .. } => ("Jenga", point(base)),
            LevelObject::PlankBridge { pos, .. } => ("PlankBridge", point(pos)),
            LevelObject::Trilithon { pos, .. } => ("Trilithon", point(pos)),
            LevelObject::Temple { pos, .. } => ("Temple", point(pos)),
        };

        ObjectInfo {
            kind,
            placement,
            footprint: self.footprint(),
            support: self.support(),
        }
    }

    /// How much ground this object covers, in its own axes.
    ///
    /// Exhaustive for the same reason [`Self::describe`] is, and the stakes are
    /// the same: an object given a point footprint it does not deserve is
    /// validated at one corner, which is the failure this exists to end.
    ///
    /// Two rules keep the answers honest. It describes what touches the
    /// **ground**, so a pendulum's overhead arm and a temple's roof overhang do
    /// not count. And it is measured from the authored point, which is not
    /// always the middle — [`Footprint::Rect::offset`] carries the difference.
    ///
    /// Approximation is fine and intended. Anything within a voxel of a point
    /// is [`Footprint::Point`], and a shape whose corners do not matter gets a
    /// disc. The check downstream reports drops of a metre and more.
    pub fn footprint(&self) -> Footprint {
        use Footprint::{Disc, Point, Rect};

        // Objects built as a grid of parts run `n` cells either side of the
        // authored point, so the half-extent is the cell half-extent times the
        // count, not times the count less one.
        let grid_half = |count: u32, cell_half: f32| count as f32 * cell_half;

        let rect = |half_x: f32, half_z: f32, yaw: f32| Rect {
            offset: (0.0, 0.0),
            half: (half_x, half_z),
            yaw,
        };

        match self {
            // Loose props: a fruit, a ball, a die. Every one of them is within
            // a voxel of the point it was authored at.
            LevelObject::Banana { length, .. } => Disc {
                radius: length * 0.5,
            },
            LevelObject::BeachBall { .. } => Point,
            LevelObject::GlowingOrb { .. } => Point,
            LevelObject::Capsule { radius, .. } => Disc { radius: *radius },
            LevelObject::Tetrahedron { size, .. } => Disc { radius: *size },
            LevelObject::Octahedron { size, .. } => Disc { radius: *size },
            LevelObject::Dodecahedron { size, .. } => Disc { radius: *size },
            LevelObject::Icosahedron { size, .. } => Disc { radius: *size },
            LevelObject::HexPrism { radius, .. } => Disc { radius: *radius },
            LevelObject::Jack { length, .. } => Disc {
                radius: length * 0.5,
            },
            // Three perpendicular bars; the shank is the longest of them.
            LevelObject::Dolos { shank_length, .. } => Disc {
                radius: shank_length * 0.5,
            },

            // Single boxes. `size` on the crates is a half-extent.
            LevelObject::Box {
                half_extents, yaw, ..
            } => rect(half_extents.0, half_extents.2, *yaw),
            LevelObject::Plank {
                length, width, yaw, ..
            } => rect(length * 0.5, width * 0.5, *yaw),
            LevelObject::Crate { size, .. } => rect(*size, *size, 0.0),
            LevelObject::HeavyCrate { size, .. } => rect(*size, *size, 0.0),
            LevelObject::House { half_extents, .. } => rect(half_extents.0, half_extents.2, 0.0),

            // Assemblies that stand on one patch of ground.
            LevelObject::Stack { items, yaw, .. } => {
                let (half_x, half_z) = stack_ground_half_extents(items);
                rect(half_x, half_z, *yaw)
            }
            LevelObject::Tower {
                box_half_extents,
                yaw,
                ..
            } => rect(box_half_extents.0, box_half_extents.2, *yaw),
            LevelObject::BoxWall {
                box_half_extents,
                columns,
                yaw,
                ..
            } => rect(
                grid_half(*columns, box_half_extents.0),
                box_half_extents.2,
                *yaw,
            ),
            LevelObject::HoneycombWall {
                columns,
                radius,
                half_height,
                yaw,
                ..
            } => {
                // Pointy-topped hex tiling: centres are sqrt(3) * radius apart
                // along the wall's own +X, and the prism axis is Z.
                let cell_half = radius * 3.0_f32.sqrt() * 0.5;
                rect(grid_half(*columns, cell_half), *half_height, *yaw)
            }
            LevelObject::Pyramid {
                block_half_extents,
                base_width,
                ..
            } => rect(
                grid_half(*base_width, block_half_extents.0),
                grid_half(*base_width, block_half_extents.2),
                0.0,
            ),
            LevelObject::Jenga {
                block_half_length, ..
            } => rect(*block_half_length, *block_half_length, 0.0),
            LevelObject::Trampoline {
                pad_half_extents,
                yaw,
                ..
            } => rect(pad_half_extents.0, pad_half_extents.2, *yaw),
            LevelObject::Table {
                top_half_extents,
                yaw,
                ..
            } => rect(top_half_extents.0, top_half_extents.2, *yaw),
            LevelObject::VoussoirArch {
                inner_radius,
                thickness,
                depth,
                ..
            } => rect(inner_radius + thickness, depth * 0.5, 0.0),
            LevelObject::Trilithon {
                gap,
                upright_half_width,
                upright_half_depth,
                lintel_overhang,
                yaw,
                ..
            } => rect(
                gap * 0.5 + upright_half_width * 2.0 + lintel_overhang,
                *upright_half_depth,
                *yaw,
            ),
            LevelObject::Temple {
                pos,
                column_height,
                front_columns,
                side_columns,
            } => {
                let def = TempleDef {
                    pos: *pos,
                    column_height: *column_height,
                    front_columns: *front_columns,
                    side_columns: *side_columns,
                };
                let (half_x, half_z) = def.ground_half_extents();
                rect(half_x, half_z, 0.0)
            }

            // A domino row is authored at its *first* block and extends one way
            // from there. Blocks are turned so their thin axis — local +Z —
            // points along the row, which is the axis they topple along.
            LevelObject::Domino {
                direction,
                count,
                spacing,
                half_extents,
                ..
            } => {
                let span = count.saturating_sub(1) as f32 * spacing;
                Rect {
                    offset: (0.0, span * 0.5),
                    half: (half_extents.0, span * 0.5 + half_extents.2),
                    yaw: direction.0.atan2(direction.1).to_degrees(),
                }
            }

            // The bridge runs along its own +Z; the beams straddle +X.
            LevelObject::PlankBridge {
                length,
                beam_spacing,
                beam_half_extents,
                yaw,
                ..
            } => rect(beam_spacing * 0.5 + beam_half_extents.0, length * 0.5, *yaw),

            // Terrain-anchored: the height is derived, so placement cannot be
            // authored wrongly and the footprint is never validated. Recorded
            // truthfully anyway, for anything that draws the level rather than
            // checks it.
            LevelObject::Menhir { bottom_radius, .. } => Disc {
                radius: *bottom_radius,
            },
            LevelObject::FencePost { radius, .. } => Disc { radius: *radius },
            LevelObject::Roller { radius, .. } => Disc { radius: *radius },
            LevelObject::PlayWheel { radius, .. } => Disc { radius: *radius },
            // The deck at its spawn point. A platform spends most of its life
            // away from there, but the footprint is about what it is placed
            // over.
            LevelObject::MovingPlatform { half_extents, .. } => {
                rect(half_extents.0, half_extents.2, 0.0)
            }
            // Both stand on a single post or fulcrum; what reaches out sideways
            // is overhead and touches nothing.
            LevelObject::Pendulum { .. } => Point,
            LevelObject::Seesaw { .. } => Point,
        }
    }

    /// Whether the ground under this object is expected to be continuous.
    ///
    /// A short list on purpose. Marking something [`Support::Spanning`] silences
    /// the footprint check for it, so the bar is that having nothing underneath
    /// is the object's *purpose* rather than a shape it happens to tolerate.
    pub fn support(&self) -> Support {
        match self {
            // The one thing in the library authored to cross a gap. Reporting
            // the void under its middle would be reporting that it works.
            LevelObject::PlankBridge { .. } => Support::Spanning,
            _ => Support::Bedded,
        }
    }

    /// Whether this object's *shape* follows its segment's yaw, not just its
    /// position.
    ///
    /// The audit behind [`Self::place_in`], written out rather than inferred.
    /// Three answers, and the enum makes the difference explicit because it is
    /// the difference between "nothing to do" and "not done yet":
    ///
    /// - [`Orientable::Symmetric`] — a sphere, a cube, a vertical post, a
    ///   regular solid. A quarter turn changes nothing observable, so there is
    ///   no work and never will be.
    /// - [`Orientable::Turns`] — carries a `yaw` that `place_in` adds the
    ///   segment's turn to.
    /// - [`Orientable::Fixed`] — has a meaningful axis and does **not** yet
    ///   carry a yaw. `level_check` warns when one is placed in a rotated
    ///   segment; it will keep its world orientation while the segment turns
    ///   around it.
    pub fn orientability(&self) -> Orientable {
        use Orientable::{Fixed, Symmetric, Turns};
        match self {
            // Turned by their own yaw.
            LevelObject::Box { .. }
            | LevelObject::Plank { .. }
            | LevelObject::Stack { .. }
            | LevelObject::Tower { .. }
            | LevelObject::BoxWall { .. }
            | LevelObject::HoneycombWall { .. }
            | LevelObject::Trampoline { .. }
            | LevelObject::Table { .. }
            | LevelObject::Dolos { .. }
            | LevelObject::Trilithon { .. }
            | LevelObject::PlankBridge { .. }
            // Domino carries its axis as a vector, which place_in rotates.
            | LevelObject::Domino { .. } => Turns,

            // No observable orientation: spheres, cubes, bodies of revolution
            // about the vertical, and regular solids whose resting pose is
            // arbitrary anyway.
            LevelObject::BeachBall { .. }
            | LevelObject::GlowingOrb { .. }
            | LevelObject::Crate { .. }
            | LevelObject::HeavyCrate { .. }
            | LevelObject::Capsule { .. }
            | LevelObject::Menhir { .. }
            | LevelObject::FencePost { .. }
            | LevelObject::Roller { .. }
            | LevelObject::PlayWheel { .. }
            | LevelObject::Tetrahedron { .. }
            | LevelObject::Octahedron { .. }
            | LevelObject::Dodecahedron { .. }
            | LevelObject::Icosahedron { .. }
            | LevelObject::HexPrism { .. }
            | LevelObject::Jack { .. }
            // Square-based and layer-alternating: a quarter turn maps each of
            // these onto an equally valid instance of itself.
            | LevelObject::Pyramid { .. }
            | LevelObject::Jenga { .. } => Symmetric,

            // Genuinely directional, not yet turnable. Each builds internal
            // structure from more than a body rotation — a swing plane, a
            // colonnade, an arch — so each is its own piece of work rather
            // than one more `yaw` field.
            LevelObject::Banana { .. }
            | LevelObject::House { .. }
            | LevelObject::Pendulum { .. }
            | LevelObject::Seesaw { .. }
            // Its patrol axis is authored in world space, so a turned segment
            // would rotate the deck's footprint but not the direction it
            // travels. Turning it means turning `axis` too.
            | LevelObject::MovingPlatform { .. }
            | LevelObject::VoussoirArch { .. }
            | LevelObject::Temple { .. } => Fixed,
        }
    }

    /// Rewrite this object's authored placement from segment-local coordinates
    /// into world coordinates: position, and orientation where the spawnable
    /// has one.
    ///
    /// An object with a meaningful horizontal axis carries a `yaw` in degrees,
    /// authored relative to its segment; placing the segment adds the segment's
    /// own yaw to it. An object without one — a sphere, a cube, a vertical post
    /// — is unaffected, correctly.
    ///
    /// **Not every direction-sensitive spawnable has been given a `yaw` yet.**
    /// [`Self::turns_with_its_segment`] is the authoritative list, and
    /// `level_check` warns when one that has not is placed in a rotated
    /// segment, so what is left reads as a known gap rather than as silence.
    pub fn place_in(&mut self, frame: &SegmentFrame) {
        let p3 = |t: &mut (f32, f32, f32)| {
            let w = frame.to_world(Point3::new(t.0, t.1, t.2));
            *t = (w.x, w.y, w.z);
        };
        // Terrain-anchored objects carry (x, z) only; the height is resolved
        // from the surface. A yaw about +Y never mixes y into x or z, so
        // sending 0 through and keeping the horizontal result is exact.
        let p2 = |t: &mut (f32, f32)| {
            let w = frame.to_world(Point3::new(t.0, 0.0, t.1));
            *t = (w.x, w.z);
        };
        // The segment's own quarter turn, in the degrees the object fields use.
        let turn = frame.yaw_degrees();
        let dir = |t: &mut (f32, f32)| {
            // A quarter turn about +Y maps (x, z) to (z, -x).
            let v = frame.to_world(Point3::new(t.0, 0.0, t.1)) - frame.to_world(Point3::origin());
            *t = (v.x, v.z);
        };

        match self {
            LevelObject::Banana { pos, .. } => p3(pos),
            LevelObject::BeachBall { pos } => p3(pos),
            LevelObject::GlowingOrb { pos, .. } => p3(pos),
            LevelObject::Box { pos, yaw, .. } => {
                p3(pos);
                *yaw += turn;
            }
            LevelObject::Plank { pos, yaw, .. } => {
                p3(pos);
                *yaw += turn;
            }
            LevelObject::Crate { pos, .. } => p3(pos),
            LevelObject::HeavyCrate { pos, .. } => p3(pos),
            LevelObject::Stack { base, yaw, .. } => {
                p3(base);
                *yaw += turn;
            }
            LevelObject::Tower { base, yaw, .. } => {
                p3(base);
                *yaw += turn;
            }
            LevelObject::BoxWall { base, yaw, .. } => {
                p3(base);
                *yaw += turn;
            }
            LevelObject::House { pos, .. } => p3(pos),
            LevelObject::Capsule { pos, .. } => p3(pos),
            LevelObject::Menhir { pos, .. } => p2(pos),
            LevelObject::FencePost { pos, .. } => p2(pos),
            LevelObject::Roller { pos, .. } => p2(pos),
            LevelObject::Pendulum { pos, .. } => p2(pos),
            LevelObject::MovingPlatform { from, to, .. } => {
                p3(from);
                p3(to);
            }
            LevelObject::PlayWheel { pos, .. } => p2(pos),
            LevelObject::Seesaw { pos, .. } => p2(pos),
            LevelObject::Tetrahedron { pos, .. } => p3(pos),
            LevelObject::Octahedron { pos, .. } => p3(pos),
            LevelObject::Dodecahedron { pos, .. } => p3(pos),
            LevelObject::HexPrism { pos, .. } => p3(pos),
            LevelObject::HoneycombWall { base, yaw, .. } => {
                p3(base);
                *yaw += turn;
            }
            LevelObject::Icosahedron { pos, .. } => p3(pos),
            LevelObject::Trampoline { pos, yaw, .. } => {
                p3(pos);
                *yaw += turn;
            }
            LevelObject::Table { pos, yaw, .. } => {
                p3(pos);
                *yaw += turn;
            }
            LevelObject::Pyramid { base, .. } => p3(base),
            LevelObject::Dolos { pos, yaw, .. } => {
                p3(pos);
                *yaw += turn;
            }
            LevelObject::Domino {
                base, direction, ..
            } => {
                p3(base);
                // The row already carries its axis as a vector, so it needs
                // rotating rather than a yaw of its own.
                dir(direction);
            }
            LevelObject::VoussoirArch { base, .. } => p3(base),
            LevelObject::Jack { pos, .. } => p3(pos),
            LevelObject::Jenga { base, .. } => p3(base),
            LevelObject::PlankBridge { pos, yaw, .. } => {
                p3(pos);
                *yaw += turn;
            }
            LevelObject::Trilithon { pos, yaw, .. } => {
                p3(pos);
                *yaw += turn;
            }
            LevelObject::Temple { pos, .. } => p3(pos),
        }
    }

    /// Convert this level object into a boxed [`Spawnable`].
    ///
    /// This bridges the RON deserialization format (named-field enum variants)
    /// with the spawnable trait system. The allocation only happens at level
    /// load time, not per frame.
    pub fn to_spawnable(&self) -> Box<dyn Spawnable> {
        match self {
            LevelObject::Banana {
                pos,
                length,
                thickness,
                curvature,
                ripeness,
                density,
                restitution,
                friction,
            } => Box::new(BananaDef {
                pos: *pos,
                length: *length,
                thickness: *thickness,
                curvature: *curvature,
                ripeness: *ripeness,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::BeachBall { pos } => Box::new(BeachBallDef { pos: *pos }),

            LevelObject::GlowingOrb { pos, colour, glow } => Box::new(GlowingOrbDef {
                pos: *pos,
                colour: *colour,
                glow: *glow,
            }),

            LevelObject::Box {
                pos,
                half_extents,
                yaw,
                style,
                density,
                restitution,
                friction,
            } => Box::new(BoxDef {
                pos: *pos,
                half_extents: *half_extents,
                yaw: *yaw,
                style: *style,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Plank {
                pos,
                length,
                width,
                yaw,
            } => Box::new(PlankDef {
                pos: *pos,
                length: *length,
                width: *width,
                yaw: *yaw,
            }),

            LevelObject::Crate { pos, size } => Box::new(CrateDef {
                pos: *pos,
                size: *size,
            }),

            LevelObject::HeavyCrate { pos, size } => Box::new(HeavyCrateDef {
                pos: *pos,
                size: *size,
            }),

            LevelObject::Stack { base, items, yaw } => Box::new(StackDef {
                base: *base,
                yaw: *yaw,
                items: items
                    .iter()
                    .map(|item| match item {
                        StackItem::Crate { size } => StackItemDef::Crate { size: *size },
                        StackItem::HeavyCrate { size } => StackItemDef::HeavyCrate { size: *size },
                        StackItem::Plank { length, width } => StackItemDef::Plank {
                            length: *length,
                            width: *width,
                        },
                        StackItem::BeachBall => StackItemDef::BeachBall,
                        StackItem::Capsule {
                            half_height,
                            radius,
                        } => StackItemDef::Capsule {
                            half_height: *half_height,
                            radius: *radius,
                        },
                    })
                    .collect(),
            }),

            LevelObject::Tower {
                base,
                box_half_extents,
                count,
                yaw,
                density,
            } => Box::new(TowerDef {
                base: *base,
                box_half_extents: *box_half_extents,
                count: *count,
                yaw: *yaw,
                density: *density,
            }),

            LevelObject::BoxWall {
                base,
                box_half_extents,
                columns,
                rows,
                yaw,
                density,
                stagger,
            } => Box::new(BoxWallDef {
                base: *base,
                box_half_extents: *box_half_extents,
                columns: *columns,
                rows: *rows,
                yaw: *yaw,
                density: *density,
                stagger: *stagger,
            }),

            LevelObject::House { pos, half_extents } => Box::new(HouseDef {
                pos: *pos,
                half_extents: *half_extents,
            }),

            LevelObject::Capsule {
                pos,
                half_height,
                radius,
                density,
                restitution,
                friction,
            } => Box::new(CapsuleDef {
                pos: *pos,
                half_height: *half_height,
                radius: *radius,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Menhir {
                pos,
                half_height,
                bottom_radius,
                top_radius,
                density,
            } => Box::new(MenhirDef {
                pos: *pos,
                half_height: *half_height,
                bottom_radius: *bottom_radius,
                top_radius: *top_radius,
                density: *density,
            }),

            LevelObject::FencePost {
                pos,
                half_height,
                radius,
                density,
            } => Box::new(FencePostDef {
                pos: *pos,
                half_height: *half_height,
                radius: *radius,
                density: *density,
            }),

            LevelObject::Roller {
                pos,
                radius,
                speed,
                spin_up_time,
                sight_range,
            } => Box::new(RollerDef {
                pos: *pos,
                radius: *radius,
                speed: *speed,
                spin_up_time: *spin_up_time,
                sight_range: *sight_range,
            }),

            LevelObject::Pendulum {
                pos,
                frame_height,
                arm_length,
                rope_length,
                ball_radius,
                ball_density,
            } => Box::new(PendulumDef {
                pos: *pos,
                frame_height: *frame_height,
                arm_length: *arm_length,
                rope_length: *rope_length,
                ball_radius: *ball_radius,
                ball_density: *ball_density,
            }),

            LevelObject::MovingPlatform {
                from,
                to,
                half_extents,
                speed,
            } => Box::new(MovingPlatformDef {
                from: *from,
                to: *to,
                half_extents: *half_extents,
                speed: *speed,
            }),

            LevelObject::PlayWheel {
                pos,
                radius,
                thickness,
                density,
                hub_height,
            } => Box::new(PlayWheelDef {
                pos: *pos,
                radius: *radius,
                thickness: *thickness,
                density: *density,
                hub_height: *hub_height,
            }),

            LevelObject::Seesaw {
                pos,
                beam_half_length,
                beam_half_width,
                beam_half_thickness,
                fulcrum_height,
                density,
            } => Box::new(SeesawDef {
                pos: *pos,
                beam_half_length: *beam_half_length,
                beam_half_width: *beam_half_width,
                beam_half_thickness: *beam_half_thickness,
                fulcrum_height: *fulcrum_height,
                density: *density,
            }),

            LevelObject::Tetrahedron {
                pos,
                size,
                density,
                restitution,
                friction,
            } => Box::new(TetrahedronDef {
                pos: *pos,
                size: *size,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Octahedron {
                pos,
                size,
                density,
                restitution,
                friction,
            } => Box::new(OctahedronDef {
                pos: *pos,
                size: *size,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Dodecahedron {
                pos,
                size,
                density,
                restitution,
                friction,
            } => Box::new(DodecahedronDef {
                pos: *pos,
                size: *size,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::HexPrism {
                pos,
                radius,
                half_height,
                density,
                restitution,
                friction,
            } => Box::new(HexPrismDef {
                pos: *pos,
                radius: *radius,
                half_height: *half_height,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::HoneycombWall {
                base,
                yaw,
                columns,
                rows,
                radius,
                half_height,
                density,
            } => Box::new(HoneycombWallDef {
                base: *base,
                yaw: *yaw,
                columns: *columns,
                rows: *rows,
                radius: *radius,
                half_height: *half_height,
                density: *density,
            }),

            LevelObject::Icosahedron {
                pos,
                size,
                density,
                restitution,
                friction,
            } => Box::new(IcosahedronDef {
                pos: *pos,
                size: *size,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Trampoline {
                pos,
                yaw,
                pad_half_extents,
                leg_half_extents,
                density,
                restitution,
            } => Box::new(TrampolineDef {
                pos: *pos,
                yaw: *yaw,
                pad_half_extents: *pad_half_extents,
                leg_half_extents: *leg_half_extents,
                density: *density,
                restitution: *restitution,
            }),

            LevelObject::Table {
                pos,
                yaw,
                top_half_extents,
                leg_half_extents,
                density,
                restitution,
                friction,
            } => Box::new(TableDef {
                pos: *pos,
                yaw: *yaw,
                top_half_extents: *top_half_extents,
                leg_half_extents: *leg_half_extents,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Pyramid {
                base,
                block_half_extents,
                base_width,
                density,
            } => Box::new(PyramidDef {
                base: *base,
                block_half_extents: *block_half_extents,
                base_width: *base_width,
                density: *density,
            }),

            LevelObject::Dolos {
                pos,
                yaw,
                shank_length,
                fluke_length,
                thickness,
                density,
                restitution,
                friction,
            } => Box::new(DolosDef {
                pos: *pos,
                yaw: *yaw,
                shank_length: *shank_length,
                fluke_length: *fluke_length,
                thickness: *thickness,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Domino {
                base,
                direction,
                count,
                spacing,
                half_extents,
                density,
            } => Box::new(DominoDef {
                base: *base,
                direction: *direction,
                count: *count,
                spacing: *spacing,
                half_extents: *half_extents,
                density: *density,
            }),

            LevelObject::VoussoirArch {
                base,
                inner_radius,
                thickness,
                depth,
                num_voussoirs,
                abutment_height,
                density,
                friction,
            } => Box::new(VoussoirArchDef {
                base: *base,
                inner_radius: *inner_radius,
                thickness: *thickness,
                depth: *depth,
                num_voussoirs: *num_voussoirs,
                abutment_height: *abutment_height,
                density: *density,
                friction: *friction,
            }),

            LevelObject::Jack {
                pos,
                length,
                thickness,
                density,
                restitution,
                friction,
            } => Box::new(JackDef {
                pos: *pos,
                length: *length,
                thickness: *thickness,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Jenga {
                base,
                layers,
                block_half_length,
                density,
                friction,
            } => Box::new(JengaDef {
                base: *base,
                layers: *layers,
                block_half_length: *block_half_length,
                density: *density,
                friction: *friction,
            }),

            LevelObject::PlankBridge {
                pos,
                length,
                beam_spacing,
                plank_count,
                beam_half_extents,
                plank_half_extents,
                density,
                yaw,
                fracture_threshold,
            } => Box::new(PlankBridgeDef {
                pos: *pos,
                length: *length,
                beam_spacing: *beam_spacing,
                plank_count: *plank_count,
                beam_half_extents: *beam_half_extents,
                plank_half_extents: *plank_half_extents,
                density: *density,
                yaw: *yaw,
                fracture_threshold: *fracture_threshold,
            }),

            LevelObject::Trilithon {
                pos,
                yaw,
                upright_half_height,
                upright_half_width,
                upright_half_depth,
                gap,
                lintel_half_thickness,
                lintel_overhang,
                density,
            } => Box::new(TrilithonDef {
                pos: *pos,
                yaw: *yaw,
                upright_half_height: *upright_half_height,
                upright_half_width: *upright_half_width,
                upright_half_depth: *upright_half_depth,
                gap: *gap,
                lintel_half_thickness: *lintel_half_thickness,
                lintel_overhang: *lintel_overhang,
                density: *density,
            }),

            LevelObject::Temple {
                pos,
                column_height,
                front_columns,
                side_columns,
            } => Box::new(TempleDef {
                pos: *pos,
                column_height: *column_height,
                front_columns: *front_columns,
                side_columns: *side_columns,
            }),
        }
    }
}

/// Items that can appear inside a `Stack`.
#[derive(Deserialize)]
pub enum StackItem {
    Crate { size: f32 },
    HeavyCrate { size: f32 },
    Plank { length: f32, width: f32 },
    BeachBall,
    Capsule { half_height: f32, radius: f32 },
}

/// Level-authored water placement data.
#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct WaterConfig {
    /// Sea level for the infinite ocean plane. If None, no ocean is rendered
    /// and boundary cells do not act as sources/sinks.
    pub ocean_level: Option<f32>,

    /// Individual water bodies placed in the level.
    #[serde(default)]
    pub bodies: Vec<WaterBody>,
}

/// A discrete body of water placed at level load time.
///
/// Flood-fills from a seed point outward through all connected terrain cells
/// whose floor is below `surface_level`. The extent is dilated by one cell
/// to cover marching-cubes shore smoothing.
#[derive(Deserialize)]
pub enum WaterBody {
    Pool {
        /// Seed point (x, z) — must be inside a terrain depression.
        seed: (f32, f32),
        /// Target water surface height (world Y).
        surface_level: f32,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn water_config_serde_defaults_apply_when_fields_are_omitted() {
        let config: WaterConfig = ron::from_str("(bodies: [])").expect("WaterConfig should parse");

        assert_eq!(config.ocean_level, None);
        assert!(config.bodies.is_empty());
    }

    #[test]
    fn water_config_default_matches_serde_defaults() {
        let config = WaterConfig::default();

        assert_eq!(config.ocean_level, None);
        assert!(config.bodies.is_empty());
    }

    #[test]
    fn water_config_rejects_runtime_tuning_fields() {
        let parse_result: Result<WaterConfig, _> = ron::from_str("(wave_damping: 2.0, bodies: [])");

        assert!(
            parse_result.is_err(),
            "WaterConfig should reject runtime tuning fields from level data"
        );
    }
}

#[cfg(test)]
mod orientation_tests {
    use super::*;
    use crate::terrain::SegmentFrame;

    /// An object with a meaningful axis must turn with its segment, not merely
    /// move with it. A wall drawn along a segment's `+X` that keeps facing
    /// world `+X` after a quarter turn is the exact trap this closes.
    #[test]
    fn an_oriented_object_turns_with_a_yaw_90_segment() {
        let frame = SegmentFrame::new(Point3::new(100.0, 0.0, 0.0), 1);

        let mut wall = LevelObject::BoxWall {
            base: (10.0, 0.0, 4.0),
            box_half_extents: (0.5, 0.5, 0.25),
            columns: 6,
            rows: 3,
            yaw: 0.0,
            density: 50.0,
            stagger: false,
        };
        wall.place_in(&frame);

        let LevelObject::BoxWall { base, yaw, .. } = wall else {
            unreachable!()
        };
        // A quarter turn maps local +X onto world −Z.
        let expected = frame.to_world(Point3::new(10.0, 0.0, 4.0));
        assert!(
            (Point3::new(base.0, base.1, base.2) - expected).norm() < 1e-4,
            "base moved to {base:?}, expected {expected:?}"
        );
        assert!((yaw - 90.0).abs() < 1e-4, "yaw is {yaw}, expected 90");
    }

    /// A yaw-invariant object gets no phantom rotation and no warning.
    #[test]
    fn a_symmetric_object_needs_no_orientation() {
        let ball = LevelObject::BeachBall {
            pos: (1.0, 2.0, 3.0),
        };
        assert_eq!(ball.orientability(), Orientable::Symmetric);
    }

    /// A domino row carries its axis as a vector, so the vector is what has to
    /// turn — there is no yaw field to add to.
    #[test]
    fn a_domino_rows_direction_is_rotated() {
        let mut row = LevelObject::Domino {
            base: (0.0, 0.0, 0.0),
            direction: (1.0, 0.0),
            count: 5,
            spacing: 0.4,
            half_extents: (0.05, 0.3, 0.15),
            density: 50.0,
        };
        row.place_in(&SegmentFrame::new(Point3::origin(), 1));
        let LevelObject::Domino { direction, .. } = row else {
            unreachable!()
        };
        assert!(
            (direction.0).abs() < 1e-5 && (direction.1 + 1.0).abs() < 1e-5,
            "direction turned to {direction:?}, expected (0, -1)"
        );
    }

    /// The audit has to cover every variant: a new spawnable must be classified
    /// rather than silently defaulting to "fine".
    #[test]
    fn every_object_kind_is_classified() {
        // `orientability` is an exhaustive match, so this is really a check
        // that the three classes are all populated and none is empty by
        // accident.
        let kinds = [
            LevelObject::BeachBall {
                pos: (0.0, 0.0, 0.0),
            }
            .orientability(),
            LevelObject::Plank {
                pos: (0.0, 0.0, 0.0),
                length: 1.0,
                width: 1.0,
                yaw: 0.0,
            }
            .orientability(),
            LevelObject::Temple {
                pos: (0.0, 0.0, 0.0),
                column_height: 1.0,
                front_columns: 4,
                side_columns: 6,
            }
            .orientability(),
        ];
        assert_eq!(
            kinds,
            [Orientable::Symmetric, Orientable::Turns, Orientable::Fixed]
        );
    }
}
