//! Interfaces: the heights where two stores meet across a link (§7.9).
//!
//! ```text
//!   link + the flow's sign ──▶ the side water leaves, and the side it enters
//!                          ──▶ upper (the surface it leaves at)
//!                              lip   (the floor it leaves over)
//!                              lower (the surface it enters, never below
//!                                     the ground its arc lands on, nor below
//!                                     still water standing over that ground)
//! ```
//!
//! A pure function of the network, read where it is needed: by the fall
//! renderer, and by a reach's surface where it meets a port. Nothing here is
//! topology; a height that moves re-lays nothing.
//!
//! A reach's end is drawn at the height it leaves at: the water it enters,
//! or, where that stands lower, the brink over its lip. The reach below, or
//! the one fed without a sheet between, starts at that same height, so two
//! surfaces meet wherever no sheet joins them.

use nalgebra::Point3;

use crate::water::ids::StoreId;

use super::link::FallPath;
use super::net::{LinkEntry, Network};
use super::reach::ReachEnds;
use super::store::{Port, Store};

/// An inflow entering a reach within this of its top meets its upstream
/// end, m; one further down joins it mid-reach.
const AT_THE_TOP: f32 = 1.0;

/// A step shorter than this is not drawn, m.
pub const STEP_EPSILON: f32 = 0.01;

/// The heights where a link's two stores meet, in the direction it flows.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Interface {
    /// The surface the water leaves at.
    pub upper: f32,
    /// The floor it leaves over.
    pub lip: f32,
    /// The surface it enters, never below where its arc meets the ground.
    pub lower: f32,
    /// Whether it runs back over a reversible link, from `down` to `up`.
    pub back: bool,
}

impl Interface {
    /// The step water drops that shows: from where it leaves to the higher
    /// of the water it enters and the lip. Zero once the two surfaces meet.
    pub fn step(&self) -> f32 {
        (self.upper - self.lower.max(self.lip)).max(0.0)
    }

    /// Whether the water below stands under the lip, so the jet falls free
    /// and draws in air.
    pub fn free(&self) -> bool {
        self.lower < self.lip
    }
}

/// The heights across `link` while it carries `q` m³/s (negative back over
/// a reversible link). `still` is the level of still water standing over a
/// point, if any: where the water leaving a lake lands under that same lake,
/// as over its own outlet, there is no step to see. `None` for a link out of
/// a sink.
pub fn interface(
    network: &Network,
    link: &LinkEntry,
    q: f64,
    still: &dyn Fn(Point3<f32>) -> Option<f32>,
) -> Option<Interface> {
    let back = q < 0.0 && link.back.is_some();
    let (leaving, leaving_port, entering, entering_port) = if back {
        (link.down, link.down_port, link.up, link.up_port)
    } else {
        (link.up, link.up_port, link.down, link.down_port)
    };
    let (lip, fall) = link.side(q);
    let landing = fall.and_then(FallPath::landing);
    let entered = match network.store(entering)? {
        Store::Reach(reach) => {
            let at = landing.unwrap_or(lip.at);
            let distance = match entering_port {
                Port::Upstream => reach.distance_at(at.x, at.z),
                Port::Downstream => reach.length,
            };
            reach.bed_at(distance) + reach.running().depth
        }
        Store::Sink => f32::NEG_INFINITY,
        store => store.level_at(store.volume(), entering_port),
    };
    let over_landing = landing.and_then(|p| still(p)).unwrap_or(f32::NEG_INFINITY);
    let lower = entered
        .max(landing.map_or(f32::NEG_INFINITY, |p| p.y))
        .max(over_landing);
    let upper = match network.store(leaving)? {
        Store::Reservoir => lip.height(),
        Store::Sink => return None,
        Store::Reach(reach) => lower.max(lip.height() + reach.brink_depth()),
        store => store.level_at(store.volume(), leaving_port),
    };
    Some(Interface {
        upper,
        lip: lip.height(),
        lower,
        back,
    })
}

/// The heights across every link, and the flow it carries, by link slot:
/// worked out once a tick, for the renderer and for [`reach_ends`].
pub fn interfaces(
    network: &Network,
    flows: impl Fn(&LinkEntry) -> f64,
    still: &dyn Fn(Point3<f32>) -> Option<f32>,
) -> Vec<Option<(Interface, f64)>> {
    let mut all = vec![None; network.link_slots()];
    for (id, link) in network.links() {
        let q = flows(link);
        all[id.0 as usize] = interface(network, link, q, still).map(|h| (h, q));
    }
    all
}

/// How far every reach's ends are eased to meet the stores at its ports, by
/// store slot (§7.9), from every link's `heights`. Its downstream end goes
/// to the height its water leaves at. Its upstream end goes to the height
/// the water feeding it leaves at, unless a sheet falls between the two.
pub fn reach_ends(network: &Network, heights: &[Option<(Interface, f64)>]) -> Vec<ReachEnds> {
    let mut ends = vec![ReachEnds::default(); network.store_slots()];
    for (id, link) in network.links() {
        let Some((heights, q)) = heights.get(id.0 as usize).copied().flatten() else {
            continue;
        };
        let (leaving, entering) = if heights.back {
            (link.down, link.up)
        } else {
            (link.up, link.down)
        };
        if let Some(reach) = as_reach(network, leaving) {
            ends[leaving.0 as usize].downstream =
                heights.upper - reach.normal_surface(reach.length);
        }
        let (lip, fall) = link.side(q);
        let sheet = fall.is_some() && heights.step() > STEP_EPSILON;
        if let Some(reach) = as_reach(network, entering).filter(|_| !sheet) {
            let at = fall.and_then(FallPath::landing).unwrap_or(lip.at);
            if reach.distance_at(at.x, at.z) <= AT_THE_TOP {
                ends[entering.0 as usize].upstream = heights.upper - reach.normal_surface(0.0);
            }
        }
    }
    ends
}

fn as_reach(network: &Network, id: StoreId) -> Option<&super::reach::Reach> {
    network.store(id).and_then(Store::as_reach)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::water::network::links::FixedRate;
    use crate::water::network::{Lip, RatingCurve, Reach};
    use nalgebra::{Vector2, Vector3};

    #[test]
    fn a_fall_landing_partway_down_a_reach_meets_it_where_it_lands() {
        let mut network = Network::default();
        let spring = network.add_store(Store::Reservoir);
        // A dry reach 20 m long, its bed falling from 5 m to 4 m.
        let points: Vec<Point3<f32>> = (0..=40)
            .map(|i| Point3::new(i as f32 * 0.5, 5.0 - i as f32 * 0.025, 0.0))
            .collect();
        let reach = network.add_store(Store::Reach(Reach::new(
            Vec::new(),
            &points,
            RatingCurve::default(),
            Vec::new(),
        )));
        let link = LinkEntry {
            up: spring,
            down: reach,
            law: Box::new(FixedRate { discharge: 1.0 }),
            open: true,
            up_port: Port::Downstream,
            down_port: Port::Upstream,
            lip: Lip {
                at: Point3::new(12.0, 8.0, 2.0),
                direction: Vector2::zeros(),
            },
            fall: Some(FallPath {
                points: vec![Point3::new(12.0, 8.0, 2.0), Point3::new(12.0, 4.7, 0.0)],
                times: vec![0.0, 0.8],
                velocity: Vector3::zeros(),
            }),
            back: None,
        };
        let heights = interface(&network, &link, 1.0, &|_| None).unwrap();
        // 12 m down, the bed stands at 4.7, not the 5 m at the reach's top.
        assert!((heights.lower - 4.7).abs() < 1e-3, "{heights:?}");
        assert_eq!(heights.upper, 8.0);
        // Still water standing over the landing raises it.
        let pooled = interface(&network, &link, 1.0, &|_| Some(6.0)).unwrap();
        assert_eq!(pooled.lower, 6.0);
    }

    fn heights(upper: f32, lip: f32, lower: f32) -> Interface {
        Interface {
            upper,
            lip,
            lower,
            back: false,
        }
    }

    #[test]
    fn a_sheet_shortens_to_nothing_as_the_water_below_rises_over_the_lip() {
        let free = heights(5.3, 5.0, 3.0);
        assert!(free.free());
        assert!((free.step() - 0.3).abs() < 1e-6);
        // Just under the lip, then over it: the step is continuous.
        let under = heights(5.3, 5.0, 4.99);
        let over = heights(5.3, 5.0, 5.01);
        assert!(under.free() && !over.free());
        assert!((under.step() - over.step()).abs() < 0.02);
        assert_eq!(heights(5.3, 5.0, 5.3).step(), 0.0);
        assert_eq!(heights(5.3, 5.0, 5.5).step(), 0.0);
    }
}
