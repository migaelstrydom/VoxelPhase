//! What the fracture system tells the owners of a compound when part of it
//! comes away as a piece of its own.

use specs::Entity;

/// A group of a compound's parts split off onto a body and entity of their
/// own. The piece already has its body, its share of the compound and of the
/// looks; each owner adds what is its own (a sheet's share of its cracking,
/// a share of the terrain anchoring).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PieceSplit {
    /// The compound's entity, which kept the rest.
    pub from: Entity,
    /// The piece's entity.
    pub piece: Entity,
}

/// This frame's splits, written by the fracture system and read by the
/// owners' systems after it. Cleared when the fracture system next runs.
#[derive(Debug, Default)]
pub struct PieceSplits {
    splits: Vec<PieceSplit>,
}

impl PieceSplits {
    /// Start a frame with no splits.
    pub fn clear(&mut self) {
        self.splits.clear();
    }

    pub fn push(&mut self, split: PieceSplit) {
        self.splits.push(split);
    }

    /// This frame's splits, in the order they were made.
    pub fn iter(&self) -> impl Iterator<Item = &PieceSplit> {
        self.splits.iter()
    }

    pub fn is_empty(&self) -> bool {
        self.splits.is_empty()
    }
}
