//! Handing fragments from the blast that cut them to what they become.
//!
//! ```text
//!   ExplosionSystem ──TerrainWorld::detonate──▶ fragments ──▶ RubbleQueue
//!                                                                 │
//!   RubbleSpawnSystem ◀───────────────────────────────────────────┘
//!         └─▶ a crumble emitter at each fragment's centroid
//! ```

use specs::{Builder, Entities, LazyUpdate, Read, System, Write};

use super::dust::Crumble;
use crate::components::Position;
use crate::terrain::Fragment;

/// Fragments cut loose this frame, waiting to be turned into rubble.
#[derive(Default)]
pub struct RubbleQueue {
    pending: Vec<Fragment>,
}

impl RubbleQueue {
    pub fn extend(&mut self, fragments: impl IntoIterator<Item = Fragment>) {
        self.pending.extend(fragments);
    }

    pub fn drain(&mut self) -> std::vec::Drain<'_, Fragment> {
        self.pending.drain(..)
    }
}

/// Turns each queued fragment into rubble.
#[derive(Default)]
pub struct RubbleSpawnSystem {
    crumble: Crumble,
}

impl<'a> System<'a> for RubbleSpawnSystem {
    type SystemData = (Entities<'a>, Read<'a, LazyUpdate>, Write<'a, RubbleQueue>);

    fn run(&mut self, (entities, lazy, mut queue): Self::SystemData) {
        for fragment in queue.drain() {
            lazy.create_entity(&entities)
                .with(Position(fragment.world_centroid().coords))
                .with(self.crumble.emitter(&fragment))
                .build();
        }
    }
}
