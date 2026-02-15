use nalgebra::Vector3;

use crate::physics::pipeline::contact_reducer::ContactReducer;
use crate::physics::pipeline::solver::ContactConstraint;

pub struct NormalCluster {
    /// Sum of contact normals weighted by depth, used to compute a blended normal.
    normal_sum: Vector3<f32>,
    /// Sum of weights contributing to normal_sum.
    weight_sum: f32,
    /// Representative contact retained from this cluster.
    best_contact: ContactConstraint,
    /// Raw depth of best_contact for selection comparison.
    best_raw_depth: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct NormalClusterConfig {
    /// Maximum number of contacts to keep after clustering.
    pub max_points: usize,
    /// Cosine threshold for grouping normals into the same cluster.
    pub normal_cluster_dot: f32,
    /// Maximum distance between contact points to cluster together.
    pub point_cluster_distance: f32,
}

impl Default for NormalClusterConfig {
    fn default() -> Self {
        Self {
            max_points: 4,
            normal_cluster_dot: 0.98,
            point_cluster_distance: 0.05,
        }
    }
}

pub struct NormalClusterer {
    /// Maximum number of contacts to keep after clustering.
    max_points: usize,
    /// Cosine threshold for grouping normals into the same cluster.
    normal_cluster_dot: f32,
    /// Maximum distance between contact points to cluster together.
    point_cluster_distance: f32,
    /// Margin used to weight normals by effective depth.
    contact_margin: f32,
    /// Shared reducer for final contact trimming.
    reducer: ContactReducer,
}

impl NormalClusterer {
    pub fn from_config(config: NormalClusterConfig, contact_margin: f32) -> Self {
        Self::new(
            config.max_points,
            config.normal_cluster_dot,
            config.point_cluster_distance,
            contact_margin,
        )
    }

    pub fn new(
        max_points: usize,
        normal_cluster_dot: f32,
        point_cluster_distance: f32,
        contact_margin: f32,
    ) -> Self {
        Self {
            max_points,
            normal_cluster_dot,
            point_cluster_distance,
            contact_margin,
            reducer: ContactReducer::new(max_points),
        }
    }

    pub fn cluster(&self, contacts: Vec<ContactConstraint>) -> Vec<ContactConstraint> {
        if contacts.len() <= 1 {
            return contacts;
        }

        let mut clusters: Vec<NormalCluster> = Vec::new();
        let max_dist_sq = self.point_cluster_distance * self.point_cluster_distance;
        for contact in contacts {
            let mut best_idx = None;
            let mut best_dot = self.normal_cluster_dot;

            for (idx, cluster) in clusters.iter().enumerate() {
                let base_normal = if cluster.weight_sum > 1e-6 {
                    (cluster.normal_sum / cluster.weight_sum).normalize()
                } else {
                    cluster.best_contact.normal
                };
                let dot = contact.normal.dot(&base_normal);
                let delta = contact.point - cluster.best_contact.point;
                let dist_sq = delta.magnitude_squared();
                if dot >= best_dot && dist_sq <= max_dist_sq {
                    best_dot = dot;
                    best_idx = Some(idx);
                }
            }

            let weight = (contact.raw_depth + self.contact_margin).max(0.0);
            if let Some(idx) = best_idx {
                let cluster = &mut clusters[idx];
                cluster.normal_sum += contact.normal * weight;
                cluster.weight_sum += weight;
                if contact.raw_depth > cluster.best_raw_depth {
                    cluster.best_raw_depth = contact.raw_depth;
                    cluster.best_contact = contact;
                }
            } else {
                clusters.push(NormalCluster {
                    normal_sum: contact.normal * weight,
                    weight_sum: weight,
                    best_raw_depth: contact.raw_depth,
                    best_contact: contact,
                });
            }
        }

        let merged: Vec<ContactConstraint> = clusters
            .into_iter()
            .map(|cluster| {
                let mut contact = cluster.best_contact;
                if cluster.weight_sum > 1e-6 {
                    contact.normal = (cluster.normal_sum / cluster.weight_sum).normalize();
                }
                contact
            })
            .collect();

        if merged.len() > self.max_points {
            return self.reducer.reduce(merged);
        }

        merged
    }
}
