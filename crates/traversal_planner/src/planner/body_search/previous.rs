use super::search::{BodySearch, STATION_TOLERANCE};

/// The last plan's body path, as the next search reads it.
#[derive(Clone, Debug, PartialEq)]
pub struct PreviousPath {
    /// The flights it settled on.
    pub flights: Vec<FlightSpan>,
    pub rides: Vec<RideSpan>,
    /// How far along the path it settled, in metres: flying from further on
    /// costs no churn, as it chose nothing there knowing what lay past it.
    pub settled_until: f32,
}

/// A stretch the body rode, along the path, and its speed at the end.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RideSpan {
    pub from: f32,
    pub to: f32,
    pub end_speed: f32,
}

/// Where a flight takes off and lands, along the path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightSpan {
    pub takeoff: f32,
    pub landing: f32,
}

impl FlightSpan {
    /// Whether a flight from `takeoff` to `landing` spans this one.
    pub(super) fn is_spanned_by(&self, takeoff: f32, landing: f32) -> bool {
        takeoff <= self.takeoff + STATION_TOLERANCE && landing >= self.landing - STATION_TOLERANCE
    }
}

/// How near a flight's takeoff and landing must be to the last plan's for
/// it to be the same flight, in metres.
const CHURN_MATCH: f32 = 0.1;

impl BodySearch<'_> {
    /// What flying from `takeoff` to `landing` costs in churn: nothing if
    /// the last plan flew there too, or settled nothing there.
    pub(super) fn flight_churn(&self, takeoff: f32, landing: f32) -> f32 {
        let Some(previous) = self
            .previous
            .filter(|previous| takeoff < previous.settled_until)
        else {
            return 0.0;
        };
        let matches = previous.flights.iter().any(|span| {
            (span.takeoff - takeoff).abs() <= CHURN_MATCH
                && (span.landing - landing).abs() <= CHURN_MATCH
        });
        if matches {
            0.0
        } else {
            self.settings.costs.churn
        }
    }

    /// What riding from `from` to `to` costs in churn: riding on past where
    /// the last plan took off.
    pub(super) fn ride_churn(&self, from: f32, to: f32) -> f32 {
        let passes_takeoff = self.previous.is_some_and(|previous| {
            previous.flights.iter().any(|span| {
                let left_behind = span.takeoff + CHURN_MATCH;
                from < left_behind && to >= left_behind
            })
        });
        if passes_takeoff {
            self.settings.costs.churn
        } else {
            0.0
        }
    }
}
