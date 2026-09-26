use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

/// The water at a body: what it is standing in, floating in, or falling into.
///
/// The water's counterpart to [`Grounding`](super::Grounding), and like it a
/// measurement: written once a frame by `ImmersionSystem`, read by the motion
/// FSM to decide whether the character is wading or swimming and by the
/// animator to know where the surface is.
#[derive(Component, Debug, Clone, Copy, Default, PartialEq)]
#[storage(DenseVecStorage)]
pub struct Immersion {
    /// The water in the body's column. `None` where there is none.
    pub water: Option<WaterAtBody>,
}

/// The water in one body's column.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaterAtBody {
    /// The water's still level: the hydrology's answer, without swell or
    /// ripples. What decisions are made against, so a passing wave does not
    /// flick a wader into a swimmer and back.
    pub level: f32,
    /// The surface as drawn, swell and ripples included. What a pose is fitted
    /// to, so arms and head meet the water the player sees.
    pub surface: f32,
    /// The floor the water rests on.
    pub floor: f32,
    /// The water's own velocity: a river's flow, a lake's drift to its outlet.
    pub current: Vector3<f32>,
}

impl WaterAtBody {
    /// Still depth of the water, level to floor.
    pub fn depth(&self) -> f32 {
        (self.level - self.floor).max(0.0)
    }

    /// How far `y` is below the still level; negative above it.
    pub fn below_level(&self, y: f32) -> f32 {
        self.level - y
    }
}

impl Immersion {
    pub fn dry() -> Self {
        Self { water: None }
    }

    pub fn in_water(water: WaterAtBody) -> Self {
        Self { water: Some(water) }
    }

    /// Still depth of the water here, zero when dry.
    pub fn depth(&self) -> f32 {
        self.water.map_or(0.0, |w| w.depth())
    }

    /// The water's velocity here, zero when dry.
    pub fn current(&self) -> Vector3<f32> {
        self.water.map_or_else(Vector3::zeros, |w| w.current)
    }
}
