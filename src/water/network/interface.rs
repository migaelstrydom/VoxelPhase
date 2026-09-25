//! Interfaces: the heights where two stores meet across a link (§7.9).
//!
//! ```text
//!   link + the flow's sign ──▶ the side water leaves, and the side it enters
//!                          ──▶ upper (the surface it leaves at)
//!                              lip   (the floor it leaves over)
//!                              lower (the surface it enters, never below
//!                                     the ground its arc lands on)
//! ```
//!
//! A pure function of the network, read where it is needed: by the fall
//! renderer, and by a reach's surface where it meets a port. Nothing here is
//! topology; a height that moves re-lays nothing.

use super::link::FallPath;
use super::net::{LinkEntry, Network};
use super::store::{Port, Store};

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
/// a reversible link). `None` for a link out of a sink.
pub fn interface(network: &Network, link: &LinkEntry, q: f64) -> Option<Interface> {
    let back = q < 0.0 && link.back.is_some();
    let (leaving, leaving_port, entering, entering_port) = if back {
        (link.down, link.down_port, link.up, link.up_port)
    } else {
        (link.up, link.up_port, link.down, link.down_port)
    };
    let (lip, fall) = link.side(q);
    let upper = match network.store(leaving)? {
        Store::Reservoir => lip.height(),
        Store::Sink => return None,
        store => store.level_at(store.volume(), leaving_port),
    };
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
    let ground = landing.map_or(f32::NEG_INFINITY, |p| p.y);
    Some(Interface {
        upper,
        lip: lip.height(),
        lower: entered.max(ground),
        back,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

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
