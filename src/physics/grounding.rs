use rustc_hash::FxHashMap;

use nalgebra::Vector3;

use crate::physics::contact_event::{ContactEvent, ContactSource};
use crate::physics::RigidBodyHandle;

#[derive(Clone, Copy, Debug)]
pub struct GroundingConfig {
    /// Minimum normal.y required to treat a contact as supporting.
    pub min_support_normal_y: f32,
    /// Bias added to contact depth to weight near-surface contacts.
    pub depth_weight_bias: f32,
}

impl Default for GroundingConfig {
    fn default() -> Self {
        Self {
            min_support_normal_y: 0.5,
            depth_weight_bias: 0.001,
        }
    }
}

pub struct GroundingDetector {
    /// Configuration values driving support detection.
    config: GroundingConfig,
}

impl GroundingDetector {
    pub fn new(config: GroundingConfig) -> Self {
        Self { config }
    }

    pub fn grounded_bodies(&self, contacts: &[ContactEvent]) -> FxHashMap<RigidBodyHandle, bool> {
        let mut accum: FxHashMap<RigidBodyHandle, SupportAccumulator> = FxHashMap::default();
        for contact in contacts {
            if contact.source != ContactSource::Narrowphase {
                continue;
            }
            let normal = contact.raw_normal;
            if normal.magnitude_squared() <= 1e-8 {
                continue;
            }
            let entry = accum
                .entry(contact.body_b)
                .or_insert_with(SupportAccumulator::default);
            entry.add(normal, contact.depth, self.config.depth_weight_bias);
        }

        accum
            .into_iter()
            .map(|(handle, acc)| {
                let support_normal = acc.support_normal();
                let grounded = support_normal.y >= self.config.min_support_normal_y
                    || acc.max_normal_y >= self.config.min_support_normal_y;
                (handle, grounded)
            })
            .collect()
    }
}

#[derive(Default)]
struct SupportAccumulator {
    /// Weighted sum of support normals for a body.
    normal_sum: Vector3<f32>,
    /// Sum of weights contributing to normal_sum.
    weight_sum: f32,
    /// Largest normal.y observed for the body this frame.
    max_normal_y: f32,
}

impl SupportAccumulator {
    fn add(&mut self, normal: Vector3<f32>, depth: f32, depth_weight_bias: f32) {
        let weight = (depth + depth_weight_bias).max(0.0);
        self.normal_sum += normal * weight;
        self.weight_sum += weight;
        self.max_normal_y = self.max_normal_y.max(normal.y);
    }

    fn support_normal(&self) -> Vector3<f32> {
        if self.weight_sum > 1e-6 {
            (self.normal_sum / self.weight_sum).normalize()
        } else {
            Vector3::y()
        }
    }
}
