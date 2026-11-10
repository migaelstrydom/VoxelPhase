//! A level's meadow: what it was asked for, and the tiles it has grown.

use std::hash::{Hash, Hasher};
use std::sync::Arc;

use nalgebra::Vector2;
use rustc_hash::{FxHashMap, FxHasher};
use specs::Entity;

use super::config::MeadowConfig;
use super::sow::{resow, sow, MeshPiece, Plot, Sprig, TileCoord, Tiles};
use super::sprig;
use crate::collision::AABB;
use crate::model::{Model, ModelPart};
use crate::rendering::colour::Colour;
use crate::rendering::material::{Material, MaterialId, SurfaceFinish};
use crate::terrain::TerrainWorld;

/// A meadow's finish: matte, as leaves are.
const FINISH: SurfaceFinish = SurfaceFinish {
    roughness: 0.8,
    metallic: 0.0,
};

/// What a meadow is drawn in: white, lit as leaves, its colours riding on
/// its vertices.
pub fn meadow_material() -> Material {
    Material::coloured(Colour::WHITE)
        .with_finish(FINISH)
        .as_foliage()
}

/// A tile as it was last grown.
struct Tile {
    /// The entity that draws it.
    entity: Entity,
    /// A mark of everything growing in it: the same again means nothing in
    /// it changed.
    mark: u64,
}

/// What became of a tile when the meadow regrew.
pub enum TileChange {
    /// It has sprigs where it had none: draw `model` on a new entity, and
    /// tell the meadow which with [`Meadow::drawn_by`].
    Grown { tile: TileCoord, model: Arc<Model> },
    /// Its sprigs changed: `entity` draws `model` now.
    Regrown { entity: Entity, model: Arc<Model> },
    /// Nothing grows in it any more: `entity` goes.
    Bare { entity: Entity },
}

/// A level's meadow.
pub struct Meadow {
    config: MeadowConfig,
    material: MaterialId,
    /// Everything growing, by tile.
    sown: Tiles,
    /// The tiles being drawn.
    tiles: FxHashMap<TileCoord, Tile>,
    /// A tile grown and not yet given its entity, with its mark.
    pending: FxHashMap<TileCoord, u64>,
    /// The version of the terrain's drawn mesh the tiles were grown from.
    grown_from: Option<u64>,
}

impl Meadow {
    /// A meadow sown as `config` asks, drawn in `material`
    /// ([`meadow_material`]), with nothing grown yet.
    pub fn new(config: MeadowConfig, material: MaterialId) -> Self {
        Self {
            config,
            material,
            sown: Tiles::default(),
            tiles: FxHashMap::default(),
            pending: FxHashMap::default(),
            grown_from: None,
        }
    }

    /// Grow the meadow over `terrain` as it is drawn now, and say which
    /// tiles changed. Nothing, if the terrain has not changed since the
    /// meadow last grew.
    ///
    /// The first time, the whole terrain is sown. After that only the
    /// ground the terrain's last update changed is, so this must be called
    /// after every update: one missed leaves that update's ground as it was.
    pub fn regrow(&mut self, terrain: &TerrainWorld) -> Vec<TileChange> {
        let version = terrain.render_version();
        if self.grown_from == Some(version) {
            return Vec::new();
        }
        let (vertices, indices) = (terrain.render_vertices(), terrain.render_indices());
        let flat = |region: &AABB| Plot {
            min: Vector2::new(region.min.x, region.min.z),
            max: Vector2::new(region.max.x, region.max.z),
        };
        let plots: Vec<Plot> = terrain.changed_regions().iter().map(flat).collect();
        let touched: Vec<TileCoord> = if self.grown_from.is_none() || plots.is_empty() {
            let before: Vec<TileCoord> = self.sown.keys().copied().collect();
            self.sown = sow(vertices, indices, &self.config);
            before
                .into_iter()
                .chain(self.sown.keys().copied())
                .collect()
        } else {
            // Every chunk over the changed ground, whichever of them the
            // update remeshed: ground at another height over the same plot
            // is sown again with it.
            let chunks: Vec<_> = terrain
                .chunk_ids()
                .into_iter()
                .filter(|id| {
                    terrain
                        .chunk_world_bounds(*id)
                        .is_some_and(|bounds| plots.iter().any(|plot| plot.meets(&flat(&bounds))))
                })
                .filter_map(|id| terrain.chunk_geometry(id))
                .collect();
            let meshes: Vec<MeshPiece> = chunks
                .iter()
                .map(|chunk| (chunk.vertices.as_slice(), chunk.indices.as_slice()))
                .collect();
            resow(&mut self.sown, &meshes, &self.config, &plots)
        };
        self.grown_from = Some(version);

        let mut changes = Vec::new();
        for tile in touched {
            let sprigs = self.sown.get(&tile);
            let mark = sprigs.map(|sprigs| mark_of(sprigs));
            let drawn = self
                .tiles
                .get(&tile)
                .map(|drawn| (drawn.entity, drawn.mark));
            match (drawn, sprigs.zip(mark)) {
                (Some((entity, _)), None) => {
                    self.tiles.remove(&tile);
                    changes.push(TileChange::Bare { entity });
                }
                (Some((entity, old)), Some((sprigs, mark))) if old != mark => {
                    self.tiles.insert(tile, Tile { entity, mark });
                    changes.push(TileChange::Regrown {
                        entity,
                        model: model_of(sprigs, self.material),
                    });
                }
                (None, Some((sprigs, mark))) if !self.pending.contains_key(&tile) => {
                    self.pending.insert(tile, mark);
                    changes.push(TileChange::Grown {
                        tile,
                        model: model_of(sprigs, self.material),
                    });
                }
                _ => {}
            }
        }
        changes
    }

    /// `entity` draws the newly grown `tile`.
    pub fn drawn_by(&mut self, tile: TileCoord, entity: Entity) {
        if let Some(mark) = self.pending.remove(&tile) {
            self.tiles.insert(tile, Tile { entity, mark });
        }
    }

    /// How many tiles have something growing in them.
    pub fn tiles(&self) -> usize {
        self.tiles.len()
    }
}

/// The model of a tile's `sprigs`, in the world's frame.
fn model_of(sprigs: &[Sprig], material: MaterialId) -> Arc<Model> {
    Arc::new(Model::flat(vec![ModelPart::new(vec![sprig::mesh(
        sprigs, material,
    )])]))
}

/// A mark of `sprigs`: what grows in which cell, and at what height.
fn mark_of(sprigs: &[Sprig]) -> u64 {
    let mut hasher = FxHasher::default();
    for sprig in sprigs {
        (sprig.cell, sprig.kind, sprig.at.y.to_bits()).hash(&mut hasher);
    }
    hasher.finish()
}
