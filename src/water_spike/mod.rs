//! The hydrology design's stage 0.5 spikes: measurements with kill criteria,
//! run before anything of the old water is deleted. Backs
//! `src/bin/water_spike.rs`.

mod routing;
mod spans;

pub use routing::{
    hierarchy, hierarchy_report, route_level, route_report, route_river, route_staircase,
    HierarchyFindings, RouteFindings,
};
pub use spans::{check_level, report, SpanFindings};
