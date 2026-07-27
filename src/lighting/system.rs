use specs::{Entities, Join, ReadStorage, System, Write};

use crate::components::{CameraComponent, Position};
use crate::lighting::collector::{ActiveLights, LightCandidate, LightCollector, LightId};
use crate::lighting::point_light::PointLight;

/// Feeds every `Position` + `PointLight` entity to a [`LightCollector`] and
/// publishes the result as the [`ActiveLights`] resource.
///
/// A thin ECS adapter: all selection logic lives in the collector, which is
/// testable without a `World`. Must run after the camera has been updated for
/// the frame and before rendering reads `ActiveLights`.
pub struct LightCollectionSystem {
    collector: LightCollector,
}

impl LightCollectionSystem {
    pub fn new(collector: LightCollector) -> Self {
        Self { collector }
    }
}

impl Default for LightCollectionSystem {
    fn default() -> Self {
        Self::new(LightCollector::default())
    }
}

impl<'a> System<'a> for LightCollectionSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, PointLight>,
        ReadStorage<'a, CameraComponent>,
        Write<'a, ActiveLights>,
    );

    fn run(&mut self, (entities, positions, lights, cameras, mut active): Self::SystemData) {
        let camera_position = match cameras.join().next() {
            Some(camera) => camera.0.position.coords,
            // No camera means nothing is being rendered this frame; leave the
            // previous set alone rather than churning it.
            None => return,
        };

        let candidates =
            (&entities, &positions, &lights)
                .join()
                .map(|(entity, position, light)| {
                    LightCandidate::new(LightId(entity.id()), position.0, *light)
                });

        self.collector
            .collect(camera_position, candidates, &mut active);
    }
}
