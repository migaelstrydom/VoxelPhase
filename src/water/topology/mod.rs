mod builder;
mod edit;
mod router;

pub use builder::{
    stands_above_outlet, PoolError, Topology, TopologyBuilder, DRIED_VOLUME, MERGE_LEVELS,
    SPLIT_BELOW,
};
pub use edit::TopologyEdit;
pub use router::{
    build_reaches, downstream_of, reach_footprint, rescan, walk, WalkEnd, MIN_CHANNEL_LENGTH,
    MIN_REACH_LENGTH, REACH_LENGTH,
};
