mod adjacency_filter;
mod contact_source;
mod coplanar_stabilizer;
mod dynamic_contacts;
mod normal_cluster;
mod static_contacts;

pub use adjacency_filter::filter_internal_edge_contacts;
pub use contact_source::{ContactFeature, ContactSource, SourcedContact};
pub use dynamic_contacts::generate_dynamic_contacts;
pub use normal_cluster::{NormalClusterConfig, NormalClusterer};
pub use static_contacts::generate_static_contacts;
