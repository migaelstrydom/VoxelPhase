mod builder;
mod edit;

pub use builder::{
    stands_above_outlet, PoolError, Topology, TopologyBuilder, DRIED_VOLUME, MERGE_LEVELS,
    SPLIT_BELOW,
};
pub use edit::TopologyEdit;
