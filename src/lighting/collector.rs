use std::cmp::Ordering;

use nalgebra::Vector3;

use crate::lighting::point_light::PointLight;
use crate::rendering::colour::Colour;

/// Maximum number of point lights uploaded per frame.
///
/// The renderer uploads a fixed-size array and every lit fragment loops over
/// all of it, so this is both a GPU cost ceiling and an array bound. It must
/// stay equal to `MAX_ACTIVE_LIGHTS` in shader/lighting.glsl — a mismatch
/// silently corrupts the tail of the light array.
pub const MAX_ACTIVE_LIGHTS: usize = 16;

/// Stable per-frame identity of a light source.
///
/// Sourced from the ECS entity id. Only used to break scoring ties
/// deterministically, so any id that is stable across frames will do.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LightId(pub u32);

/// A light offered to the collector, decoupled from any ECS type.
#[derive(Clone, Copy, Debug)]
pub struct LightCandidate {
    /// Stable identity, used for deterministic tie-breaking.
    pub id: LightId,

    /// World position of the owning entity. The light's own offset is applied
    /// by the collector.
    pub entity_position: Vector3<f32>,

    /// The light itself.
    pub light: PointLight,
}

impl LightCandidate {
    pub fn new(id: LightId, entity_position: Vector3<f32>, light: PointLight) -> Self {
        Self {
            id,
            entity_position,
            light,
        }
    }
}

/// A light that made it into the frame's active set, with its position resolved
/// to world space.
///
/// The fields are grouped so they pack onto two `vec4`s when the GPU layout
/// lands: (position, range) and (colour, intensity). `id` is CPU-side only.
#[derive(Clone, Copy, Debug)]
pub struct ActiveLight {
    /// Which candidate this came from. Not uploaded; kept for debug overlays
    /// and for reasoning about frame-to-frame slot stability.
    pub id: LightId,

    /// Light position in world space (entity position + light offset).
    pub position: Vector3<f32>,

    /// Distance at which contribution reaches zero.
    pub range: f32,

    /// Linear colour. Alpha is ignored.
    pub colour: Colour,

    /// Luminance the light emits, independent of `colour`'s hue. See
    /// `PointLight::intensity`.
    pub intensity: f32,
}

/// The point lights the renderer should light the whole frame with.
///
/// Rebuilt from scratch every frame by [`LightCollector`]. Ordered by relevance
/// (most relevant first), deterministically for a given candidate set.
#[derive(Clone, Debug, Default)]
pub struct ActiveLights {
    lights: Vec<ActiveLight>,
}

impl ActiveLights {
    /// Build a light set directly, bypassing the collector.
    ///
    /// For callers that already know exactly which lights they want — the
    /// visual bench authors its lights by hand rather than gathering them from
    /// a world. Truncated to [`MAX_ACTIVE_LIGHTS`], since anything beyond that
    /// would be silently dropped by the shader anyway.
    pub fn from_lights(mut lights: Vec<ActiveLight>) -> Self {
        lights.truncate(MAX_ACTIVE_LIGHTS);
        Self { lights }
    }

    /// The active lights, most relevant first. Never longer than
    /// [`MAX_ACTIVE_LIGHTS`].
    pub fn lights(&self) -> &[ActiveLight] {
        &self.lights
    }

    pub fn len(&self) -> usize {
        self.lights.len()
    }

    pub fn is_empty(&self) -> bool {
        self.lights.is_empty()
    }
}

/// Selects the per-frame point light set from all candidate lights.
///
/// ```text
///   candidates ──▶ cull (out of relevance range)
///              ──▶ score (intensity / (1 + d²))
///              ──▶ sort (score desc, id asc)
///              ──▶ truncate to max_lights ──▶ ActiveLights
/// ```
///
/// Holds no state between frames: the same candidate set always produces the
/// same output, in the same order, whatever order the candidates arrive in.
/// That matters because a light changing slots between frames is visible as
/// flicker once the shader indexes the array.
#[derive(Clone, Copy, Debug)]
pub struct LightCollector {
    /// How many lights survive selection. Clamped to [`MAX_ACTIVE_LIGHTS`].
    max_lights: usize,

    /// How far past its own `range` a light may sit from the camera and still
    /// be considered relevant.
    ///
    /// Culling on camera distance alone would drop a lamp that is lighting the
    /// wall in front of the player, so the meaningful quantity is
    /// `distance - range`: the gap between the camera and the light's sphere of
    /// influence. Any lit geometry in view lies within that gap, so the margin
    /// is a view-distance budget rather than a property of the light. The
    /// default is a fraction of the camera's far plane (100 units), keeping
    /// lights that reach visible geometry while discarding distant ones.
    cull_margin: f32,
}

impl LightCollector {
    /// Default relevance budget beyond a light's range, in world units.
    pub const DEFAULT_CULL_MARGIN: f32 = 60.0;

    /// A collector keeping at most `max_lights` lights (clamped to
    /// [`MAX_ACTIVE_LIGHTS`]) with the default relevance margin.
    pub fn new(max_lights: usize) -> Self {
        Self {
            max_lights: max_lights.min(MAX_ACTIVE_LIGHTS),
            cull_margin: Self::DEFAULT_CULL_MARGIN,
        }
    }

    /// Override how far beyond a light's range it stays relevant.
    pub fn with_cull_margin(mut self, cull_margin: f32) -> Self {
        self.cull_margin = cull_margin.max(0.0);
        self
    }

    /// Rebuild `out` from `candidates` as seen from `camera_position`.
    ///
    /// `out` is cleared first, so a frame with no candidates yields an empty
    /// set rather than stale lights.
    pub fn collect<I>(&self, camera_position: Vector3<f32>, candidates: I, out: &mut ActiveLights)
    where
        I: IntoIterator<Item = LightCandidate>,
    {
        let mut scored: Vec<(f32, ActiveLight)> = candidates
            .into_iter()
            .filter_map(|candidate| self.score(camera_position, candidate))
            .collect();

        scored.sort_by(|a, b| Self::compare(a, b));
        scored.truncate(self.max_lights);

        out.lights.clear();
        out.lights
            .extend(scored.into_iter().map(|(_, light)| light));
    }

    /// Score a candidate, or reject it as irrelevant to this camera.
    fn score(
        &self,
        camera_position: Vector3<f32>,
        candidate: LightCandidate,
    ) -> Option<(f32, ActiveLight)> {
        let light = candidate.light;
        // Negated comparisons so NaN parameters are rejected too.
        if !(light.intensity > 0.0) || !(light.range > 0.0) {
            return None;
        }

        let position = light.world_position(candidate.entity_position);
        let distance = (position - camera_position).norm();
        if !distance.is_finite() || distance - light.range > self.cull_margin {
            return None;
        }

        let score = light.intensity / (1.0 + distance * distance);
        if !score.is_finite() {
            return None;
        }

        Some((
            score,
            ActiveLight {
                id: candidate.id,
                position,
                range: light.range,
                colour: light.colour,
                intensity: light.intensity,
            },
        ))
    }

    /// Most relevant first; equal scores break on ascending id so the ordering
    /// does not depend on the order candidates arrived in.
    fn compare(a: &(f32, ActiveLight), b: &(f32, ActiveLight)) -> Ordering {
        b.0.partial_cmp(&a.0)
            .unwrap_or(Ordering::Equal)
            .then_with(|| a.1.id.cmp(&b.1.id))
    }
}

impl Default for LightCollector {
    fn default() -> Self {
        Self::new(MAX_ACTIVE_LIGHTS)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn candidate(id: u32, position: Vector3<f32>, intensity: f32, range: f32) -> LightCandidate {
        LightCandidate::new(
            LightId(id),
            position,
            PointLight::new(Colour::WHITE, intensity, range),
        )
    }

    fn ids(active: &ActiveLights) -> Vec<u32> {
        active.lights().iter().map(|l| l.id.0).collect()
    }

    #[test]
    fn empty_input_yields_empty_set() {
        let mut active = ActiveLights::default();
        LightCollector::default().collect(Vector3::zeros(), Vec::new(), &mut active);
        assert!(active.is_empty());
    }

    #[test]
    fn previous_lights_are_discarded() {
        let collector = LightCollector::default();
        let mut active = ActiveLights::default();

        collector.collect(
            Vector3::zeros(),
            vec![candidate(1, Vector3::new(1.0, 0.0, 0.0), 1.0, 10.0)],
            &mut active,
        );
        assert_eq!(active.len(), 1);

        collector.collect(Vector3::zeros(), Vec::new(), &mut active);
        assert!(active.is_empty());
    }

    #[test]
    fn caps_at_max_active_lights() {
        // 40 candidates all in range, at increasing distance.
        let candidates: Vec<_> = (0..40)
            .map(|i| candidate(i, Vector3::new(i as f32 * 0.1, 0.0, 0.0), 1.0, 100.0))
            .collect();

        let mut active = ActiveLights::default();
        LightCollector::default().collect(Vector3::zeros(), candidates, &mut active);

        assert_eq!(active.len(), MAX_ACTIVE_LIGHTS);
        // Nearest lights win, in order.
        assert_eq!(
            ids(&active),
            (0..MAX_ACTIVE_LIGHTS as u32).collect::<Vec<_>>()
        );
    }

    #[test]
    fn respects_a_lower_cap() {
        let candidates: Vec<_> = (0..10)
            .map(|i| candidate(i, Vector3::new(i as f32, 0.0, 0.0), 1.0, 100.0))
            .collect();

        let mut active = ActiveLights::default();
        LightCollector::new(4).collect(Vector3::zeros(), candidates, &mut active);
        assert_eq!(ids(&active), vec![0, 1, 2, 3]);
    }

    #[test]
    fn culls_lights_beyond_range_plus_margin() {
        let collector = LightCollector::default().with_cull_margin(0.0);
        let mut active = ActiveLights::default();

        collector.collect(
            Vector3::zeros(),
            vec![
                // Sphere of influence reaches the camera.
                candidate(1, Vector3::new(9.0, 0.0, 0.0), 1.0, 10.0),
                // Sphere of influence stops short of it.
                candidate(2, Vector3::new(11.0, 0.0, 0.0), 1.0, 10.0),
            ],
            &mut active,
        );

        assert_eq!(ids(&active), vec![1]);
    }

    #[test]
    fn cull_margin_keeps_lights_whose_influence_is_ahead_of_the_camera() {
        let collector = LightCollector::default().with_cull_margin(20.0);
        let mut active = ActiveLights::default();

        collector.collect(
            Vector3::zeros(),
            vec![candidate(1, Vector3::new(25.0, 0.0, 0.0), 1.0, 10.0)],
            &mut active,
        );

        assert_eq!(ids(&active), vec![1]);
    }

    #[test]
    fn zero_intensity_and_zero_range_lights_are_rejected() {
        let mut active = ActiveLights::default();
        LightCollector::default().collect(
            Vector3::zeros(),
            vec![
                candidate(1, Vector3::new(1.0, 0.0, 0.0), 0.0, 10.0),
                candidate(2, Vector3::new(1.0, 0.0, 0.0), 1.0, 0.0),
            ],
            &mut active,
        );
        assert!(active.is_empty());
    }

    #[test]
    fn nearer_light_outranks_farther_light_of_equal_intensity() {
        let mut active = ActiveLights::default();
        LightCollector::default().collect(
            Vector3::zeros(),
            vec![
                candidate(7, Vector3::new(20.0, 0.0, 0.0), 1.0, 50.0),
                candidate(3, Vector3::new(2.0, 0.0, 0.0), 1.0, 50.0),
            ],
            &mut active,
        );
        assert_eq!(ids(&active), vec![3, 7]);
    }

    #[test]
    fn brighter_light_outranks_dimmer_light_at_equal_distance() {
        let mut active = ActiveLights::default();
        LightCollector::default().collect(
            Vector3::zeros(),
            vec![
                candidate(1, Vector3::new(5.0, 0.0, 0.0), 1.0, 50.0),
                candidate(2, Vector3::new(0.0, 5.0, 0.0), 9.0, 50.0),
            ],
            &mut active,
        );
        assert_eq!(ids(&active), vec![2, 1]);
    }

    #[test]
    fn ties_break_on_ascending_id_regardless_of_input_order() {
        // Four identical lights equidistant from the camera: only the id can
        // order them.
        let positions = [
            Vector3::new(5.0, 0.0, 0.0),
            Vector3::new(-5.0, 0.0, 0.0),
            Vector3::new(0.0, 5.0, 0.0),
            Vector3::new(0.0, 0.0, 5.0),
        ];
        let forward: Vec<_> = positions
            .iter()
            .enumerate()
            .map(|(i, p)| candidate(i as u32 + 1, *p, 1.0, 50.0))
            .collect();
        let mut reversed = forward.clone();
        reversed.reverse();

        let collector = LightCollector::default();
        let mut a = ActiveLights::default();
        let mut b = ActiveLights::default();
        collector.collect(Vector3::zeros(), forward, &mut a);
        collector.collect(Vector3::zeros(), reversed, &mut b);

        assert_eq!(ids(&a), vec![1, 2, 3, 4]);
        assert_eq!(ids(&a), ids(&b));
    }

    #[test]
    fn cap_selection_is_independent_of_input_order() {
        let mut forward: Vec<_> = (0..30)
            .map(|i| candidate(i, Vector3::new(i as f32, 0.0, 0.0), 1.0, 100.0))
            .collect();
        // Equal-score pairs mirrored about the camera, to exercise tie-breaking
        // right at the cap boundary.
        forward.extend(
            (0..30).map(|i| candidate(i + 100, Vector3::new(-(i as f32), 0.0, 0.0), 1.0, 100.0)),
        );
        let mut reversed = forward.clone();
        reversed.reverse();

        let collector = LightCollector::default();
        let mut a = ActiveLights::default();
        let mut b = ActiveLights::default();
        collector.collect(Vector3::zeros(), forward, &mut a);
        collector.collect(Vector3::zeros(), reversed, &mut b);

        assert_eq!(a.len(), MAX_ACTIVE_LIGHTS);
        assert_eq!(ids(&a), ids(&b));
    }

    #[test]
    fn world_position_includes_the_light_offset() {
        let light =
            PointLight::new(Colour::WHITE, 1.0, 20.0).with_offset(Vector3::new(0.0, 3.0, 0.0));
        let mut active = ActiveLights::default();
        LightCollector::default().collect(
            Vector3::zeros(),
            vec![LightCandidate::new(
                LightId(1),
                Vector3::new(4.0, 0.0, 0.0),
                light,
            )],
            &mut active,
        );

        assert_eq!(active.lights()[0].position, Vector3::new(4.0, 3.0, 0.0));
    }

    #[test]
    fn offset_participates_in_culling_and_scoring() {
        // Both entities sit at the same place; the offset one is pushed out of
        // relevance range.
        let collector = LightCollector::default().with_cull_margin(0.0);
        let mut active = ActiveLights::default();
        collector.collect(
            Vector3::zeros(),
            vec![
                LightCandidate::new(
                    LightId(1),
                    Vector3::new(9.0, 0.0, 0.0),
                    PointLight::new(Colour::WHITE, 1.0, 10.0),
                ),
                LightCandidate::new(
                    LightId(2),
                    Vector3::new(9.0, 0.0, 0.0),
                    PointLight::new(Colour::WHITE, 1.0, 10.0)
                        .with_offset(Vector3::new(0.0, 20.0, 0.0)),
                ),
            ],
            &mut active,
        );

        assert_eq!(ids(&active), vec![1]);
    }
}
