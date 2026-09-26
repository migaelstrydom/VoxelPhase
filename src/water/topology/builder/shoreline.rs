//! Shorelines: where a channel meets standing water (§8.2). The settle asks
//! whether one has moved since the channel was laid, and if so lays the
//! network again, pouring each reach's water back where it stands.
//!
//! ```text
//!   each reach ──▶ a body's water risen over one of its cells?
//!   each channel's end in a basin ──▶ its shoreline left dry?
//! ```
//!
//! A channel is laid down to the first span water stands over. Drowning is
//! judged on a cell's highest floor, `DROWN_MARGIN` up, and exposure on its
//! lowest, so a level hovering at a shoreline does not lay the network
//! again and again.

use crate::water::geometry::{Column, SpanGraph, SpanRef};
use crate::water::ids::StoreId;
use crate::water::network::{Network, Store};

use super::super::router::{channel_heads, downstream_of};
use super::{Topology, TopologyBuilder};

/// A cell is drowned once water stands this far over the highest point of
/// its floor, m.
const DROWN_MARGIN: f32 = 0.05;

impl TopologyBuilder {
    /// Whether water has moved along a channel's bed since it was laid:
    /// risen over a reach's cells, or fallen from the shoreline a channel
    /// ends at.
    pub(super) fn shorelines_moved(&self, t: &Topology) -> bool {
        t.network.store_ids().into_iter().any(|id| {
            let Some(reach) = t.network.store(id).and_then(Store::as_reach) else {
                return false;
            };
            let heads = channel_heads(t.network, id);
            first_drowned(t.geometry.graph(), t.network, &reach.cells, &heads).is_some()
                || shore_exposed(t, id)
        })
    }
}

/// Whether the basin a reach runs into has fallen from its shoreline: from
/// the span its outlet names, where the water stood when it was laid, below
/// that span's lowest floor. Over a fall too: the fall lands short of the
/// water.
fn shore_exposed(t: &Topology, id: StoreId) -> bool {
    let exposed = || {
        let outlet = t.network.store(id)?.as_reach()?.outlet.as_ref()?;
        let down = downstream_of(t.network, id)?;
        let level = t.network.store(down)?.as_basin()?.level();
        let graph = t.geometry.graph();
        let shore = graph.span_at(Column::containing(outlet.at.x, outlet.at.z), outlet.at.y)?;
        Some(level < graph.span(shore).floor_min)
    };
    exposed().unwrap_or(false)
}

/// The first of `cells` over which a body other than one of `heads` stands
/// more than `DROWN_MARGIN` above its highest floor, and that body. A lake
/// standing over its own outlet covers the first cells of the channel
/// leaving it, and does not drown the channel it feeds.
fn first_drowned(
    graph: &SpanGraph,
    network: &Network,
    cells: &[SpanRef],
    heads: &[StoreId],
) -> Option<(usize, StoreId)> {
    cells.iter().enumerate().find_map(|(i, cell)| {
        let body = graph.owner(*cell).body.filter(|b| !heads.contains(b))?;
        let level = network.store(body)?.surface()?;
        (level > graph.span(*cell).floor_max + DROWN_MARGIN).then_some((i, body))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::geometry::{Span, SpanChunk, SpanChunkCoord, COLUMNS_PER_CHUNK};
    use crate::water::network::Ocean;

    /// A channel's cells down column row k = 0, on a bed falling 0.1 m a
    /// column from 5 m, and a network holding one body at `level` that owns
    /// the cells from i = 6 to i = 9.
    fn channel_past_a_body(level: f32) -> (SpanGraph, Network, Vec<SpanRef>, StoreId) {
        let coord = SpanChunkCoord { x: 0, z: 0 };
        let mut graph = SpanGraph::new(coord, coord);
        let columns: Vec<Vec<Span>> = (0..COLUMNS_PER_CHUNK)
            .map(|local| {
                let f = 5.0 - 0.1 * coord.column(local).i as f32;
                vec![Span {
                    floor_c: f,
                    floor_min: f,
                    floor_max: f,
                    ceiling: f32::INFINITY,
                }]
            })
            .collect();
        graph.replace_chunk(coord, SpanChunk::from_columns(&columns, 1));
        let mut network = Network::default();
        let body = network.add_store(Store::Ocean(Ocean {
            level,
            swell: 0.0,
            region_version: 0,
        }));
        let cells: Vec<SpanRef> = (0..14)
            .map(|i| graph.span_at(Column::new(i, 0), 5.5).unwrap())
            .collect();
        for cell in &cells[6..10] {
            graph.owner_mut(*cell).body = Some(body);
        }
        (graph, network, cells, body)
    }

    #[test]
    fn water_rising_over_the_middle_of_a_channel_drowns_it_there() {
        // Floors under the body run 4.4 down to 4.1. At 4.36 it stands more
        // than 5 cm over the floor at i = 7 (4.3), so the channel is cut
        // there, in the middle, though the body is not what it runs into.
        let (graph, network, cells, body) = channel_past_a_body(4.36);
        assert_eq!(
            first_drowned(&graph, &network, &cells, &[]),
            Some((7, body))
        );
        // At 4.34 it is within the margin at i = 7, and over it at i = 8.
        let (graph, network, cells, _) = channel_past_a_body(4.34);
        assert_eq!(
            first_drowned(&graph, &network, &cells, &[]),
            Some((8, body))
        );
        // Below every floor it owns, it drowns nothing.
        let (graph, network, cells, _) = channel_past_a_body(4.1);
        assert_eq!(first_drowned(&graph, &network, &cells, &[]), None);
    }

    #[test]
    fn the_lake_a_channel_leaves_does_not_drown_it() {
        let (graph, network, cells, body) = channel_past_a_body(6.0);
        assert!(first_drowned(&graph, &network, &cells, &[]).is_some());
        assert_eq!(first_drowned(&graph, &network, &cells, &[body]), None);
    }
}
