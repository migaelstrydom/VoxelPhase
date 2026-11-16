//! Pumps: water lifted from one place in the field and thrown out at
//! another.
//!
//! ```text
//!   intake ──draw──▶ (as much as its rate, no more than stands over it)
//!                      │
//!   outlet ◀──pour─────┘  the same water, as a jet, landing like a spring's
//! ```
//!
//! A pump makes no water and loses none: what it cannot draw it does not
//! pour, so one whose intake stands dry is still. Whoever owns a pump names
//! it, says where it stands ([`PumpPlacement`]) and moves it as its body
//! moves.

use std::collections::BTreeMap;

use nalgebra::{Point3, Vector3};

use crate::water::geometry::{Column, SpanGraph, SpanIndex};

use super::cells::{FieldCells, CELL_AREA};
use super::sources::landing_cells;

/// Names a pump. Its owner chooses the name, which need only be unlike
/// every other pump's in the field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PumpId {
    /// Whose pump it is.
    pub owner: u64,
    /// Which of its owner's outlets it feeds.
    pub spout: u32,
}

/// Least distance a pump's intake or outlet must move, m, or change in its
/// water's velocity, m/s, for the field to stand it anew. A pump on a body
/// at rest still reads a pose that wobbles in its last bits; restanding it
/// would retrace its cells and redraw its jet every frame. A millimetre
/// per second shifts a jet's landing by about a millimetre a second of
/// flight.
const PUMP_MOVE_TOLERANCE: f32 = 1e-3;

/// Least change in a jet's discharge, as a fraction of it, that is drawn.
const DRAWN_DISCHARGE_CHANGE: f32 = 0.01;

/// Where a pump stands and how hard it works.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PumpPlacement {
    /// Where it draws water: it takes only what stands above this point.
    pub intake: Point3<f32>,
    /// Where its water comes out.
    pub outlet: Point3<f32>,
    /// The velocity its water leaves the outlet at, m/s.
    pub velocity: Vector3<f32>,
    /// The most it moves, m³/s.
    pub rate: f32,
}

impl PumpPlacement {
    /// Whether a pump standing at `self` has been moved, or set working
    /// differently, by being asked to stand at `other`.
    fn moved_to(&self, other: &PumpPlacement) -> bool {
        (self.intake - other.intake).norm() > PUMP_MOVE_TOLERANCE
            || (self.outlet - other.outlet).norm() > PUMP_MOVE_TOLERANCE
            || (self.velocity - other.velocity).norm() > PUMP_MOVE_TOLERANCE
            || self.rate != other.rate
    }
}

/// A stream of water thrown into the air from a point: a spring's, or a
/// running pump's.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Jet {
    pub origin: Point3<f32>,
    pub velocity: Vector3<f32>,
    /// m³/s it carries now.
    pub discharge: f32,
    /// The cell its arc meets.
    pub landing: SpanIndex,
}

impl Jet {
    /// Whether `other` flies the same arc into the same cell, whatever
    /// each carries.
    pub fn same_arc(&self, other: &Jet) -> bool {
        self.origin == other.origin
            && self.velocity == other.velocity
            && self.landing == other.landing
    }

    /// Whether `other` would be drawn as this jet is: the same arc,
    /// carrying the same water to within what can be seen.
    pub fn drawn_alike(&self, other: &Jet) -> bool {
        self.same_arc(other)
            && (self.discharge - other.discharge).abs()
                <= DRAWN_DISCHARGE_CHANGE * self.discharge.abs().max(other.discharge.abs())
    }
}

/// One pump, as the ground and water stand.
#[derive(Debug, Clone)]
struct Pump {
    placement: PumpPlacement,
    /// The cells it draws from: the one its intake stands in and those
    /// joined to it.
    intake: Vec<SpanIndex>,
    /// The cells its water lands in; empty when it falls out of the world.
    landing: Vec<SpanIndex>,
    /// What it moved over the last step, m³/s.
    discharge: f32,
}

impl Pump {
    fn new(placement: PumpPlacement, graph: &SpanGraph, cells: &FieldCells) -> Self {
        let mut pump = Self {
            placement,
            intake: Vec::new(),
            landing: Vec::new(),
            discharge: 0.0,
        };
        pump.retrace(graph, cells);
        pump
    }

    /// Find the cells it draws from and pours into again.
    fn retrace(&mut self, graph: &SpanGraph, cells: &FieldCells) {
        let at = self.placement.intake;
        self.intake.clear();
        let column = Column::containing(at.x, at.z);
        if graph.contains_column(column) {
            if let Some(span) = graph.span_at(column, at.y) {
                let around = graph.neighbours(span).into_iter().map(|n| n.span);
                self.intake = std::iter::once(span)
                    .chain(around)
                    .filter_map(|s| cells.index(graph, s.column, s.ordinal))
                    .collect();
            }
        }
        self.landing = landing_cells(graph, cells, self.placement.outlet, self.placement.velocity);
    }

    /// Move a step's water from intake to landing. Returns the volume that
    /// left the field because the outlet throws it out of the world, m³.
    fn run(&mut self, cells: &mut FieldCells, dt: f32) -> f64 {
        let mouth = self.placement.intake.y as f64;
        // What stands over the mouth in each cell it draws from, m.
        let over: Vec<f64> = self
            .intake
            .iter()
            .map(|&at| (cells.surface(at) - mouth).min(cells.depth(at)).max(0.0))
            .collect();
        let standing: f64 = over.iter().sum::<f64>() * CELL_AREA;
        let wanted = self.placement.rate.max(0.0) as f64 * dt as f64;
        let drawn = wanted.min(standing);
        self.discharge = if dt > 0.0 {
            (drawn / dt as f64) as f32
        } else {
            0.0
        };
        if drawn <= 0.0 {
            return 0.0;
        }
        let share = drawn / standing;
        for (&at, over) in self.intake.iter().zip(&over) {
            *cells.depth_mut(at) -= over * share;
            cells.wake(at.slot as usize);
        }
        if self.landing.is_empty() {
            return drawn;
        }
        let rise = drawn / self.landing.len() as f64 / CELL_AREA;
        for &at in &self.landing {
            *cells.depth_mut(at) += rise;
            cells.wake(at.slot as usize);
        }
        0.0
    }
}

/// Every pump in a field.
#[derive(Debug, Default)]
pub struct Pumps {
    pumps: BTreeMap<PumpId, Pump>,
}

impl Pumps {
    /// Stand pump `id` at `placement`: a new pump if the field holds none
    /// by that name, the same one moved or working differently if it does.
    pub fn set(
        &mut self,
        id: PumpId,
        placement: PumpPlacement,
        graph: &SpanGraph,
        cells: &FieldCells,
    ) {
        match self.pumps.get_mut(&id) {
            Some(pump) if !pump.placement.moved_to(&placement) => {}
            Some(pump) => {
                pump.placement = placement;
                pump.retrace(graph, cells);
            }
            None => {
                self.pumps.insert(id, Pump::new(placement, graph, cells));
            }
        }
    }

    /// Take out every pump `keep` does not want.
    pub fn retain(&mut self, mut keep: impl FnMut(PumpId) -> bool) {
        self.pumps.retain(|&id, _| keep(id));
    }

    /// Find every pump's cells again, after the ground changed.
    pub fn retrace(&mut self, graph: &SpanGraph, cells: &FieldCells) {
        for pump in self.pumps.values_mut() {
            pump.retrace(graph, cells);
        }
    }

    /// Run every pump for a step. Returns the volume thrown out of the
    /// world, m³.
    pub fn run(&mut self, cells: &mut FieldCells, dt: f32) -> f64 {
        self.pumps
            .values_mut()
            .map(|pump| pump.run(cells, dt))
            .sum()
    }

    /// What pump `id` moved over the last step, m³/s.
    pub fn discharge(&self, id: PumpId) -> Option<f32> {
        self.pumps.get(&id).map(|pump| pump.discharge)
    }

    /// The jet of every pump that is moving water into the field.
    pub fn jets(&self) -> impl Iterator<Item = Jet> + '_ {
        self.pumps.values().filter_map(|pump| {
            let landing = *pump.landing.first()?;
            (pump.discharge > 0.0).then_some(Jet {
                origin: pump.placement.outlet,
                velocity: pump.placement.velocity,
                discharge: pump.discharge,
                landing,
            })
        })
    }
}

#[cfg(all(test, feature = "water_scenarios"))]
mod tests {
    use super::*;
    use crate::water::field::WaterField;
    use crate::water_viewer::find;

    const DT: f32 = 1.0 / 60.0;

    /// The pond of the `island_pool` scenario: its surface at -1.5 m, its
    /// bed some metres under that at `IN_THE_POND`.
    fn pond() -> WaterField {
        let scenario = find("island_pool").unwrap();
        let (level, terrain) = scenario.terrain().unwrap();
        WaterField::build(level.water.as_ref().unwrap(), &terrain)
    }

    const IN_THE_POND: Point3<f32> = Point3::new(12.0, -2.5, 0.0);
    const PUMP: PumpId = PumpId { owner: 7, spout: 0 };

    /// A pump drawing at `IN_THE_POND` and throwing its water up and
    /// along the pond.
    fn fountain(rate: f32) -> PumpPlacement {
        PumpPlacement {
            intake: IN_THE_POND,
            outlet: IN_THE_POND + Vector3::new(0.0, 3.0, 0.0),
            velocity: Vector3::new(1.0, 4.0, 0.0),
            rate,
        }
    }

    #[test]
    fn a_pump_moves_its_rate_and_makes_no_water() {
        let mut field = pond();
        let before = field.volume();
        field.set_pump(PUMP, fountain(0.3));
        for _ in 0..600 {
            field.step(DT);
        }
        assert!((field.pump_discharge(PUMP).unwrap() - 0.3).abs() < 1.0e-4);
        assert_eq!(field.jets().count(), 1);
        let ledger = field.ledger();
        assert_eq!(ledger.sourced, 0.0);
        let drift = (field.volume() + ledger.lost + ledger.sunk - before).abs();
        assert!(drift < 1.0e-6, "the field's water changed by {drift} m³");
    }

    #[test]
    fn a_pump_with_its_intake_out_of_the_water_is_still() {
        let mut field = pond();
        let mut placement = fountain(0.3);
        placement.intake.y = -1.0;
        field.set_pump(PUMP, placement);
        for _ in 0..60 {
            field.step(DT);
        }
        assert_eq!(field.pump_discharge(PUMP), Some(0.0));
        assert_eq!(field.jets().count(), 0);
    }

    #[test]
    fn a_pump_moved_out_of_the_water_stops_and_starts_again_when_put_back() {
        let mut field = pond();
        field.set_pump(PUMP, fountain(0.3));
        field.step(DT);
        assert!(field.pump_discharge(PUMP).unwrap() > 0.0);

        let mut ashore = fountain(0.3);
        ashore.intake = Point3::new(25.0, 1.0, 0.0);
        field.set_pump(PUMP, ashore);
        field.step(DT);
        assert_eq!(field.pump_discharge(PUMP), Some(0.0));

        field.set_pump(PUMP, fountain(0.3));
        field.step(DT);
        assert!(field.pump_discharge(PUMP).unwrap() > 0.0);
    }

    #[test]
    fn a_pump_thrown_out_of_the_world_drains_into_the_ledger() {
        let mut field = pond();
        let before = field.volume();
        let mut placement = fountain(0.3);
        placement.velocity = Vector3::new(200.0, 4.0, 0.0);
        field.set_pump(PUMP, placement);
        for _ in 0..60 {
            field.step(DT);
        }
        let ledger = field.ledger();
        assert!((ledger.sunk - 0.3).abs() < 1.0e-3, "sunk {}", ledger.sunk);
        assert!((field.volume() + ledger.lost + ledger.sunk - before).abs() < 1.0e-6);
    }

    #[test]
    fn a_pump_whose_pose_wobbles_keeps_flying_the_same_jet() {
        let mut field = pond();
        field.set_pump(PUMP, fountain(0.3));
        field.step(DT);
        let jet = field.jets().next().unwrap();
        for frame in 0..60 {
            let mut wobbled = fountain(0.3);
            let wobble = if frame % 2 == 0 { 1.0e-6 } else { -1.0e-6 };
            wobbled.velocity.x += wobble;
            wobbled.outlet.z += wobble;
            field.set_pump(PUMP, wobbled);
            field.step(DT);
            assert!(field.jets().next().unwrap().same_arc(&jet));
        }
    }

    #[test]
    fn a_pump_creeping_by_less_than_the_tolerance_a_frame_is_still_moved() {
        let mut field = pond();
        field.set_pump(PUMP, fountain(0.3));
        field.step(DT);
        let jet = field.jets().next().unwrap();
        let mut crept = fountain(0.3);
        for _ in 0..10 {
            crept.velocity.x += 0.5 * PUMP_MOVE_TOLERANCE;
            field.set_pump(PUMP, crept);
            field.step(DT);
        }
        let moved = field.jets().next().unwrap();
        assert!(!moved.same_arc(&jet));
        assert!((moved.velocity.x - crept.velocity.x).abs() <= PUMP_MOVE_TOLERANCE);
    }

    #[test]
    fn a_removed_pump_moves_nothing() {
        let mut field = pond();
        field.set_pump(PUMP, fountain(0.3));
        field.retain_pumps(|_| false);
        field.step(DT);
        assert_eq!(field.pump_discharge(PUMP), None);
        assert_eq!(field.jets().count(), 0);
    }
}
