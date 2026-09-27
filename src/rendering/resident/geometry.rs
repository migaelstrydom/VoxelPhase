use std::sync::{Arc, Weak};

use rustc_hash::FxHashMap;

use crate::core::device::ManagedDevice;
use crate::core::error::EngineResult;
use crate::model::Model;
use crate::rendering::mesh_source::MeshBuffers;
use crate::rendering::resident::arena::{MeshArena, ResidentMesh, UploadTally};
use crate::rendering::transparency::MeshBounds;
use crate::rendering::vertex::Vertex;

/// Frames a versioned mesh may go undrawn before it is let go.
const VERSIONED_IDLE_FRAMES: u64 = 2;

/// Names a mesh whose owner says when it changes, by bumping a version,
/// rather than by handing over a different object: the terrain.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VersionedMeshId(pub u32);

impl VersionedMeshId {
    /// The level's terrain.
    pub const TERRAIN: Self = Self(0);
}

/// One primitive of a resident model.
#[derive(Clone, Copy, Debug)]
pub struct ResidentPrimitive {
    /// Where its mesh sits in the arena.
    pub mesh: ResidentMesh,
    /// Its extent in model space, measured once at upload: a model's
    /// vertices never change, so there is no reason to walk them again on
    /// the frames that sort or cull by it.
    pub bounds: MeshBounds,
}

/// A model's meshes, as long as the model lives.
struct CachedModel {
    /// Tells the cache when the model is gone. Holding it also keeps the
    /// model's allocation from being reused, so the address the entry is keyed
    /// by cannot come to name a different model while the entry exists.
    model: Weak<Model>,
    /// One per primitive, parts in order and each part's primitives in order.
    primitives: Vec<ResidentPrimitive>,
}

/// A versioned mesh, as of the version last drawn.
struct VersionedMesh {
    version: u64,
    mesh: ResidentMesh,
    /// The frame it was last drawn in.
    last_drawn: u64,
}

/// Geometry that stays on the GPU between frames, uploaded once and drawn
/// from the arena for as long as its owner keeps it.
///
/// ```text
///   Arc<Model> ──first draw──▶ upload each primitive ──▶ CachedModel
///       │                                                   │ every later draw
///       │                                                   ▼
///       └──last Arc dropped──▶ swept at begin_frame ──▶ arena.release
///
///   (id, version) ──version changed──▶ release old, upload new
///                 ──undrawn for a while──▶ release
/// ```
///
/// Models never change in place — a fracture swaps a new `Arc<Model>` onto
/// the entity — so a model's identity is the whole of its cache key. Terrain
/// is one buffer rebuilt in place on every edit, so it is keyed by the
/// version its owner publishes instead.
pub struct ResidentGeometry {
    arena: MeshArena,
    /// Keyed by the model's address.
    models: FxHashMap<usize, CachedModel>,
    versioned: FxHashMap<VersionedMeshId, VersionedMesh>,
    /// The frame being recorded.
    frame: u64,
}

impl ResidentGeometry {
    pub fn new(device: Arc<ManagedDevice>) -> Self {
        Self {
            arena: MeshArena::new(device),
            models: FxHashMap::default(),
            versioned: FxHashMap::default(),
            frame: 0,
        }
    }

    /// Start frame `frame`: let go of what nothing draws any more, and reuse
    /// what the finished frames have stopped reading. Only after the fence of
    /// the slot being reused is waited on.
    pub fn begin_frame(&mut self, frame: u64) {
        self.frame = frame;
        self.arena.begin_frame(frame);

        let arena = &mut self.arena;
        self.models.retain(|_, cached| {
            let alive = cached.model.strong_count() > 0;
            if !alive {
                cached
                    .primitives
                    .iter()
                    .for_each(|primitive| arena.release(primitive.mesh));
            }
            alive
        });
        self.versioned.retain(|_, versioned| {
            let drawn = frame.saturating_sub(versioned.last_drawn) <= VERSIONED_IDLE_FRAMES;
            if !drawn {
                arena.release(versioned.mesh);
            }
            drawn
        });
    }

    /// The buffers of every block of the arena.
    pub fn buffers(&self) -> Vec<MeshBuffers> {
        self.arena.buffers()
    }

    /// A model's primitives, uploading them the first time it is seen.
    pub fn model(&mut self, model: &Arc<Model>) -> EngineResult<&[ResidentPrimitive]> {
        let key = Arc::as_ptr(model) as usize;
        if !self.models.contains_key(&key) {
            let mut primitives = Vec::new();
            for primitive in model.parts.iter().flat_map(|part| &part.primitives) {
                match self.arena.upload(&primitive.vertices, &primitive.indices) {
                    Ok(mesh) => primitives.push(ResidentPrimitive {
                        mesh,
                        bounds: MeshBounds::of(&primitive.vertices),
                    }),
                    Err(e) => {
                        primitives
                            .iter()
                            .for_each(|uploaded| self.arena.release(uploaded.mesh));
                        return Err(e);
                    }
                }
            }
            self.models.insert(
                key,
                CachedModel {
                    model: Arc::downgrade(model),
                    primitives,
                },
            );
        }
        Ok(&self.models[&key].primitives)
    }

    /// A versioned mesh as of `version`, uploading it when the version is
    /// not the one already resident.
    pub fn versioned(
        &mut self,
        id: VersionedMeshId,
        version: u64,
        vertices: &[Vertex],
        indices: &[u32],
    ) -> EngineResult<ResidentMesh> {
        let frame = self.frame;
        if let Some(resident) = self.versioned.get_mut(&id) {
            if resident.version == version {
                resident.last_drawn = frame;
                return Ok(resident.mesh);
            }
        }

        let mesh = self.arena.upload(vertices, indices)?;
        let replaced = self.versioned.insert(
            id,
            VersionedMesh {
                version,
                mesh,
                last_drawn: frame,
            },
        );
        if let Some(replaced) = replaced {
            self.arena.release(replaced.mesh);
        }
        Ok(mesh)
    }

    /// What was uploaded since the last call.
    pub fn take_tally(&mut self) -> UploadTally {
        self.arena.take_tally()
    }
}
