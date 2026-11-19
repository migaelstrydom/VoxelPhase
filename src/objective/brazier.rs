//! A brazier: a goal the player lights by arriving at it, and ruins by
//! knocking it over.
//!
//! ```text
//!   Goal.reached ──▶ BrazierSystem ──▶ OnFire::everlasting + FireSeat (the
//!                         │            flame in the bowl), embers glow
//!   Orientation ──tipped──┘──▶ Goal.ruined: put out, never to be lit
//! ```
//!
//! The brazier is an ordinary heavy body: a blast, a boulder or a flood can
//! tip it. Tipped past [`Brazier::tip_limit`] its coals are spilled, and
//! that cannot be undone.

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage, Entities, Join, ReadStorage, System, WriteStorage};

use crate::components::{MaterialModulation, Orientation};
use crate::fire::components::{FireSeat, OnFire};
use crate::rendering::material::SurfaceModulation;

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
}

impl Brazier {
    /// An unlit brazier whose flame burns at `flame`.
    pub fn new(flame: FireSeat) -> Self {
        Self {
            flame,
            tip_limit: TIP_LIMIT,
            lit: false,
        }
    }

    /// Whether, turned as `orientation`, it has tipped past its limit.
    pub fn tipped(&self, orientation: &Orientation) -> bool {
        (orientation.0 * Vector3::y()).angle(&Vector3::y()) > self.tip_limit
    }

    /// How its embers glow, lit or not.
    pub fn embers(&self) -> SurfaceModulation {
        SurfaceModulation::emissive(if self.lit { 1.0 } else { 0.0 })
    }
}

/// Well past the angle a tripod topples from, so a brazier rocked by a
/// blast that settles back is still a brazier, and short of lying on its
/// side, so one fallen against a wall is not.
const TIP_LIMIT: f32 = 0.7;

/// Lights braziers whose goal has been reached, and ruins ones knocked over.
pub struct BrazierSystem;

impl<'a> System<'a> for BrazierSystem {
    type SystemData = (
        Entities<'a>,
        ReadStorage<'a, Orientation>,
        WriteStorage<'a, Brazier>,
        WriteStorage<'a, Goal>,
        WriteStorage<'a, OnFire>,
        WriteStorage<'a, FireSeat>,
        WriteStorage<'a, MaterialModulation>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, orientations, mut braziers, mut goals, mut fires, mut seats, mut glows) =
            data;

        for (entity, orientation, brazier, goal) in
            (&entities, &orientations, &mut braziers, &mut goals).join()
        {
            if brazier.tipped(orientation) {
                goal.ruined = true;
            }

            let burning = goal.reached && !goal.ruined;
            if burning == brazier.lit {
                continue;
            }
            brazier.lit = burning;
            if burning {
                fires.insert(entity, OnFire::everlasting()).ok();
                seats.insert(entity, brazier.flame).ok();
            } else {
                fires.remove(entity);
                seats.remove(entity);
            }
            glows
                .insert(entity, MaterialModulation(brazier.embers()))
                .ok();
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
}
