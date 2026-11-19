//! A brazier: a goal the player lights by arriving at it, and ruins by
//! knocking it over.
//!
//! ```text
//!   Goal.reached ──▶ BrazierSystem ──▶ OnFire::everlasting + FireSeat (the
//!                         │            flame in the bowl), embers glow
//!   Orientation ──tipped──┘──▶ Goal.ruined: put out, never to be lit
//!                         │
//!                         └──▶ LightShaft aim: pale while it waits, gold
//!                              once lit, gone once spilled
//! ```
//!
//! A brazier is a destination, and says so from across the level without a
//! word: a shaft of pale light stands up from its bowl into the sky, and its
//! banked embers breathe a dull red. Lit, the shaft turns to gold. The
//! brazier is an ordinary heavy body: a blast, a boulder or a flood can
//! tip it. Tipped past [`Brazier::tip_limit`] its coals are spilled, and
//! that cannot be undone.

use std::f32::consts::TAU;

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, Entities, Join, Read, ReadStorage, System, WriteStorage};

use crate::components::{MaterialModulation, Orientation};
use crate::fire::components::{FireSeat, OnFire};
use crate::light_shaft::{LightShaft, ShaftLook};
use crate::rendering::material::SurfaceModulation;
use crate::time::Time;

use super::goal::Goal;

/// A brazier, on the entity that carries its [`Goal`].
#[derive(Component, Clone, Copy, Debug)]
#[storage(DenseVecStorage)]
pub struct Brazier {
    /// Where its flame burns, in the body's frame.
    pub flame: FireSeat,

    /// How far it may lean from upright, radians, before its coals spill.
    pub tip_limit: f32,

    /// Whether it is burning.
    pub lit: bool,

    /// Seconds its banked embers have been breathing.
    pub breathing: f32,
}

impl Brazier {
    /// An unlit brazier whose flame burns at `flame`.
    pub fn new(flame: FireSeat) -> Self {
        Self {
            flame,
            tip_limit: TIP_LIMIT,
            lit: false,
            breathing: 0.0,
        }
    }

    /// Whether, turned as `orientation`, it has tipped past its limit.
    pub fn tipped(&self, orientation: &Orientation) -> bool {
        (orientation.0 * Vector3::y()).angle(&Vector3::y()) > self.tip_limit
    }

    /// How its embers glow: in full once lit; banked, breathing slowly,
    /// while it waits; not at all once spilled.
    pub fn embers(&self, goal: &Goal) -> SurfaceModulation {
        let glow = if self.lit {
            1.0
        } else if goal.ruined {
            0.0
        } else {
            let breath = (self.breathing * BREATHS_PER_SECOND * TAU).sin();
            BANKED * (1.0 + BANKED_BREATH * breath)
        };
        SurfaceModulation::emissive(glow)
    }

    /// How its shaft of light looks: pale while it waits, gold once lit,
    /// gone once spilled.
    pub fn shaft(&self, goal: &Goal) -> ShaftLook {
        if self.lit {
            SHAFT_LIT
        } else if goal.ruined {
            ShaftLook::NONE
        } else {
            SHAFT_WAITING
        }
    }
}

/// Banked embers: their glow as a share of a lit brazier's, how far it
/// swells and fades either side of that, and how slowly.
const BANKED: f32 = 0.14;
const BANKED_BREATH: f32 = 0.35;
const BREATHS_PER_SECOND: f32 = 0.3;

/// The shaft over a brazier that waits: a pale, cool light from the sky,
/// such as Bosch's blessed rise through.
const SHAFT_WAITING: ShaftLook = ShaftLook {
    colour: Vector3::new(0.82, 0.9, 1.0),
    radiance: 0.55,
    presence: 1.0,
};
/// The shaft over a lit brazier: warm gold, and brighter.
const SHAFT_LIT: ShaftLook = ShaftLook {
    colour: Vector3::new(1.0, 0.7, 0.32),
    radiance: 1.1,
    presence: 1.0,
};

/// Well past the angle a tripod topples from, so a brazier rocked by a
/// blast that settles back is still a brazier, and short of lying on its
/// side, so one fallen against a wall is not.
const TIP_LIMIT: f32 = 0.7;

/// Lights braziers whose goal has been reached, and ruins ones knocked over.
pub struct BrazierSystem;

impl<'a> System<'a> for BrazierSystem {
    type SystemData = (
        Entities<'a>,
        Read<'a, Time>,
        ReadStorage<'a, Orientation>,
        WriteStorage<'a, Brazier>,
        WriteStorage<'a, Goal>,
        WriteStorage<'a, OnFire>,
        WriteStorage<'a, FireSeat>,
        WriteStorage<'a, MaterialModulation>,
        WriteStorage<'a, LightShaft>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            time,
            orientations,
            mut braziers,
            mut goals,
            mut fires,
            mut seats,
            mut glows,
            mut shafts,
        ) = data;
        let dt = time.delta_seconds();

        for (entity, orientation, brazier, goal) in
            (&entities, &orientations, &mut braziers, &mut goals).join()
        {
            if brazier.tipped(orientation) {
                goal.ruined = true;
            }
            brazier.breathing += dt;

            let burning = goal.reached && !goal.ruined;
            if burning != brazier.lit {
                brazier.lit = burning;
                if burning {
                    fires.insert(entity, OnFire::everlasting()).ok();
                    seats.insert(entity, brazier.flame).ok();
                } else {
                    fires.remove(entity);
                    seats.remove(entity);
                }
            }
            glows
                .insert(entity, MaterialModulation(brazier.embers(goal)))
                .ok();
            if let Some(shaft) = shafts.get_mut(entity) {
                shaft.aim = brazier.shaft(goal);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::UnitQuaternion;
    use specs::{Builder, RunNow, World, WorldExt};

    use super::*;

    fn world_with_brazier(lean: f32) -> (World, specs::Entity) {
        let mut world = World::new();
        world.register::<Orientation>();
        world.register::<Brazier>();
        world.register::<Goal>();
        world.register::<OnFire>();
        world.register::<FireSeat>();
        world.register::<MaterialModulation>();
        world.register::<LightShaft>();
        world.insert(Time::fixed(1.0 / 60.0));
        let seat = FireSeat {
            base: Vector3::new(0.0, 0.4, 0.0),
            size: Vector3::new(0.5, 0.8, 0.5),
        };
        let entity = world
            .create_entity()
            .with(Orientation(UnitQuaternion::from_axis_angle(
                &Vector3::x_axis(),
                lean,
            )))
            .with(Brazier::new(seat))
            .with(Goal::new(2.0))
            .build();
        (world, entity)
    }

    fn reach(world: &World, entity: specs::Entity) {
        world
            .write_storage::<Goal>()
            .get_mut(entity)
            .unwrap()
            .reached = true;
    }

    #[test]
    fn reaching_it_lights_it_in_its_bowl() {
        let (world, entity) = world_with_brazier(0.1);
        BrazierSystem.run_now(&world);
        assert!(!world.read_storage::<OnFire>().contains(entity));

        reach(&world, entity);
        BrazierSystem.run_now(&world);

        assert!(world.read_storage::<Brazier>().get(entity).unwrap().lit);
        assert!(world.read_storage::<OnFire>().contains(entity));
        assert!(world.read_storage::<FireSeat>().contains(entity));
    }

    #[test]
    fn knocked_over_it_is_ruined_and_cannot_be_lit() {
        let (world, entity) = world_with_brazier(1.2);
        BrazierSystem.run_now(&world);
        assert!(world.read_storage::<Goal>().get(entity).unwrap().ruined);

        reach(&world, entity);
        BrazierSystem.run_now(&world);
        assert!(!world.read_storage::<OnFire>().contains(entity));
    }

    #[test]
    fn knocked_over_once_lit_it_goes_out_for_good() {
        let (world, entity) = world_with_brazier(0.0);
        reach(&world, entity);
        BrazierSystem.run_now(&world);
        assert!(world.read_storage::<OnFire>().contains(entity));

        world
            .write_storage::<Orientation>()
            .get_mut(entity)
            .unwrap()
            .0 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.4);
        BrazierSystem.run_now(&world);
        world
            .write_storage::<Orientation>()
            .get_mut(entity)
            .unwrap()
            .0 = UnitQuaternion::identity();
        BrazierSystem.run_now(&world);

        assert!(!world.read_storage::<OnFire>().contains(entity));
        assert!(!world.read_storage::<Brazier>().get(entity).unwrap().lit);
    }

    #[test]
    fn its_shaft_waits_pale_burns_gold_and_goes_with_its_coals() {
        let shaft_of =
            |world: &World, entity| world.read_storage::<LightShaft>().get(entity).unwrap().aim;
        let (mut world, entity) = world_with_brazier(0.0);
        world
            .write_storage::<LightShaft>()
            .insert(
                entity,
                LightShaft::new(Vector3::zeros(), 30.0, (0.3, 1.0), ShaftLook::NONE),
            )
            .unwrap();
        BrazierSystem.run_now(&world);
        assert_eq!(shaft_of(&world, entity), SHAFT_WAITING);

        reach(&world, entity);
        BrazierSystem.run_now(&world);
        assert_eq!(shaft_of(&world, entity), SHAFT_LIT);

        world
            .write_storage::<Orientation>()
            .get_mut(entity)
            .unwrap()
            .0 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.4);
        BrazierSystem.run_now(&world);
        assert_eq!(shaft_of(&world, entity), ShaftLook::NONE);
        world.maintain();
    }
}
