//! The edits that change the network's topology.
//!
//! Every change to stores and links is one of these, applied between solver
//! ticks by the [`TopologyBuilder`](super::TopologyBuilder), and every volume
//! it moves is a `Transfer` through the ledger: the one place a topology
//! change can break conservation.

use crate::water::geometry::SpanRef;
use crate::water::ids::{LinkId, StoreId};
use crate::water::solver::Account;

/// One change to the network, as applied and logged.
#[derive(Debug, Clone, PartialEq)]
pub enum TopologyEdit {
    /// A store was added under this id.
    AddStore(StoreId),
    /// A store was removed, whatever it still held going to `residual_to`.
    RemoveStore {
        store: StoreId,
        residual_to: Account,
    },
    /// A link was added under this id.
    AddLink(LinkId),
    RemoveLink(LinkId),
    SetLinkOpen {
        link: LinkId,
        open: bool,
    },
    /// The loss gate (§7.7).
    SetMinor {
        store: StoreId,
        minor: bool,
    },
    Transfer {
        from: Account,
        to: Account,
        volume: f64,
    },
    /// A basin re-flooded from these seeds at its current level, keeping its
    /// volume.
    Reregion {
        basin: StoreId,
        seeds: Vec<SpanRef>,
    },
}
