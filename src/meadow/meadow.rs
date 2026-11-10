//! A level's meadow: what it was asked for, and the tiles it has grown.

use std::hash::{Hash, Hasher};
use std::sync::Arc;

use rustc_hash::{FxHashMap, FxHasher};
use specs::Entity;

use super::config::MeadowConfig;
use super::sow::{sow, Sprig, TileCoord};
use super::sprig;
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
            tiles: FxHashMap::default(),
            pending: FxHashMap::default(),
            grown_from: None,
        }
    }

    /// Grow the meadow over `terrain` as it is drawn now, and say which
    /// tiles changed. Nothing, if the terrain has not changed since the
    /// meadow last grew.
    pub fn regrow(&mut self, terrain: &TerrainWorld) -> Vec<TileChange> {
        let version = terrain.render_version();
        if self.grown_from == Some(version) {
            return Vec::new();
        }
        self.grown_from = Some(version);
        let mut sown = sow(
            terrain.render_vertices(),
            terrain.render_indices(),
            &self.config,
        );

        let material = self.material;
        let mut changes = Vec::new();
        self.tiles.retain(|coord, tile| {
            let Some(sprigs) = sown.remove(coord) else {
                changes.push(TileChange::Bare {
                    entity: tile.entity,
                });
                return false;
            };
            let mark = mark_of(&sprigs);
            if mark != tile.mark {
                tile.mark = mark;
                changes.push(TileChange::Regrown {
                    entity: tile.entity,
                    model: model_of(&sprigs, material),
                });
            }
            true
        });
        for (tile, sprigs) in sown {
            self.pending.insert(tile, mark_of(&sprigs));
            changes.push(TileChange::Grown {
                tile,
                model: model_of(&sprigs, material),
            });
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
