//! Whether a set of passive bodies ever gains energy it was not given.
//!
//! Contacts, friction and joints can only take energy out of bodies that
//! nothing drives, so a group of them may gain energy only by the work done
//! on it from outside: by a body the audit does not judge — a character, a
//! driven platform — pushing it through a contact. Whatever more it gains,
//! the solver made up.
//!
//! ```text
//!   each frame:  energy of each object (kinetic, spin, height)
//!                work done on it by bodies outside the audit ◄── ContactWorkLedger
//!                objects in contact join one group
//!   at the end:  per group, (energy − work received) per kg,
//!                its largest rise above its lowest so far
//! ```
//!
//! Energy passes between objects that touch, so it is judged over each group
//! of objects that touched, not object by object: one object's knock may be
//! what sets another moving. It is measured against the least the group has
//! had since the audit began, not against where it started, so energy lost to
//! friction and then regained is caught.

use rustc_hash::FxHashMap;

use super::handle::RigidBodyHandle;
use super::world::PhysicsWorld;

/// Watches objects of passive bodies, frame by frame, for energy from
/// nowhere.
pub struct EnergyAudit {
    /// The objects judged, in the order they were given.
    objects: Vec<Audited>,
    /// Which object each audited body belongs to.
    object_of: FxHashMap<RigidBodyHandle, usize>,
    /// Each object's parent in its group's tree; a root is its own.
    parent: Vec<usize>,
}

/// One object's record.
struct Audited {
    /// Its bodies.
    handles: Vec<RigidBodyHandle>,
    /// Their total mass, in kg.
    mass: f32,
    /// Its energy after every observed frame, in joules; the first entry is
    /// where the audit began.
    energy: Vec<f32>,
    /// Work bodies outside the audit had done on it by each of those frames,
    /// in joules, running total.
    received: Vec<f32>,
    /// Whether something outside the contacts fed it, so its group is not
    /// judged on energy at all.
    exempt: bool,
    /// Whether any of its bodies became non-finite.
    non_finite: bool,
}

impl EnergyAudit {
    /// Begin auditing `objects`, each a set of bodies judged as one, from
    /// their state now. Switches on the world's contact-work ledger, which the
    /// audit reads.
    pub fn begin(world: &mut PhysicsWorld, objects: Vec<Vec<RigidBodyHandle>>) -> Self {
        world.record_contact_work(true);
        let object_of = objects
            .iter()
            .enumerate()
            .flat_map(|(object, handles)| handles.iter().map(move |&h| (h, object)))
            .collect();
        let parent = (0..objects.len()).collect();
        let objects = objects
            .into_iter()
            .map(|handles| {
                let (energy, mass) = energy_and_mass(world, &handles);
                Audited {
                    handles,
                    mass,
                    energy: vec![energy],
                    received: vec![0.0],
                    exempt: false,
                    non_finite: false,
                }
            })
            .collect();
        Self {
            objects,
            object_of,
            parent,
        }
    }

    /// Record the frame the world just stepped.
    pub fn observe(&mut self, world: &PhysicsWorld) {
        let mut received = vec![0.0f32; self.objects.len()];
        for (on, by, work) in world.contact_work().iter() {
            let Some(&object) = self.object_of.get(&on) else {
                continue;
            };
            if by.is_some_and(|by| !self.object_of.contains_key(&by)) {
                received[object] += work;
            }
        }
        for (object, work) in self.objects.iter_mut().zip(received) {
            let (energy, _) = energy_and_mass(world, &object.handles);
            object.non_finite |= !energy.is_finite();
            object.energy.push(energy);
            let total = object.received.last().copied().unwrap_or(0.0) + work;
            object.received.push(total);
        }
        for contact in world.contact_events() {
            let Some(body_a) = contact.body_a else {
                continue;
            };
            if let (Some(&a), Some(&b)) = (
                self.object_of.get(&body_a),
                self.object_of.get(&contact.body_b),
            ) {
                self.join(a, b);
            }
        }
    }

    /// Leave `object`'s group unjudged on energy: something other than a
    /// contact fed it — water lifting it, say.
    pub fn exempt(&mut self, object: usize) {
        self.objects[object].exempt = true;
    }

    /// Whether any body of `object` became NaN or infinite.
    pub fn is_non_finite(&self, object: usize) -> bool {
        self.objects[object].non_finite
    }

    /// For each object, the most its group's energy per kilogram, less the
    /// work done on the group from outside, rose above the least it had had
    /// since the audit began. `None` where the group is not judged: an object
    /// in it was exempted or became non-finite.
    pub fn gains(&mut self) -> Vec<Option<f32>> {
        let count = self.objects.len();
        let roots: Vec<usize> = (0..count).map(|o| self.root(o)).collect();
        let mut judged = vec![true; count];
        for (object, &root) in self.objects.iter().zip(&roots) {
            judged[root] &= !object.exempt && !object.non_finite;
        }

        let frames = self.objects.first().map_or(0, |o| o.energy.len());
        let mut unexplained = vec![vec![0.0f32; frames]; count];
        let mut mass = vec![0.0f32; count];
        for (object, &root) in self.objects.iter().zip(&roots) {
            mass[root] += object.mass;
            for (frame, total) in unexplained[root].iter_mut().enumerate() {
                *total += object.energy[frame] - object.received[frame];
            }
        }
        let gain_of_root: Vec<f32> = unexplained
            .iter()
            .zip(&mass)
            .map(|(series, &mass)| largest_rise(series) / mass.max(f32::MIN_POSITIVE))
            .collect();

        roots
            .iter()
            .map(|&root| judged[root].then_some(gain_of_root[root]))
            .collect()
    }

    fn root(&mut self, mut object: usize) -> usize {
        while self.parent[object] != object {
            self.parent[object] = self.parent[self.parent[object]];
            object = self.parent[object];
        }
        object
    }

    /// Put two objects in the same group.
    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.root(a), self.root(b));
        if a != b {
            self.parent[a] = b;
        }
    }
}

/// The largest amount a series ever rose above its lowest value before.
fn largest_rise(series: &[f32]) -> f32 {
    let mut lowest = f32::INFINITY;
    series
        .iter()
        .map(|&value| {
            lowest = lowest.min(value);
            value - lowest
        })
        .fold(0.0, f32::max)
}

/// The bodies' kinetic energy, spin included, and height in the world's
/// gravity, in joules, and their total mass. Non-finite if any body is.
fn energy_and_mass(world: &PhysicsWorld, handles: &[RigidBodyHandle]) -> (f32, f32) {
    let gravity = world.config().gravity;
    handles
        .iter()
        .filter_map(|&h| world.body(h))
        .map(|body| {
            let mass = body.mass();
            let spin = body.angular_velocity();
            let rotational = body
                .world_inv_inertia()
                .try_inverse()
                .map_or(0.0, |inertia| 0.5 * spin.dot(&(inertia * spin)));
            let kinetic = 0.5 * mass * body.linear_velocity().norm_squared();
            let height = -mass * gravity.dot(&body.position().coords);
            (kinetic + rotational + height, mass)
        })
        .fold((0.0, 0.0), |(e, m), (be, bm)| (e + be, m + bm))
}

/// The bodies' energy per kilogram of them all: kinetic, spin and height.
pub fn energy_per_kg(world: &PhysicsWorld, handles: &[RigidBodyHandle]) -> f32 {
    let (energy, mass) = energy_and_mass(world, handles);
    if mass > 0.0 {
        energy / mass
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::physics::{ColliderDesc, PhysicsConfig, RigidBodyDesc};

    /// An audit of objects of a kilogram each, with these energy histories and
    /// no work received, built without a world.
    fn audit(histories: &[&[f32]]) -> EnergyAudit {
        EnergyAudit {
            objects: histories
                .iter()
                .map(|energy| Audited {
                    handles: Vec::new(),
                    mass: 1.0,
                    energy: energy.to_vec(),
                    received: vec![0.0; energy.len()],
                    exempt: false,
                    non_finite: false,
                })
                .collect(),
            object_of: FxHashMap::default(),
            parent: (0..histories.len()).collect(),
        }
    }

    /// One object's knock sets another moving: the first loses what the
    /// second gains. Judged apart, the second gains; judged as the group
    /// their touching made them, neither does.
    #[test]
    fn energy_passed_between_objects_that_touched_is_no_gain() {
        let histories: [&[f32]; 2] = [&[2.0, 1.0, 0.5, 0.5], &[0.5, 0.5, 1.0, 0.6]];

        let apart = audit(&histories).gains();
        assert_eq!(apart[1], Some(0.5), "judged alone, the struck object gains");

        let mut touched = audit(&histories);
        touched.join(0, 1);
        let together = touched.gains();
        assert!(
            together.iter().all(|g| g.is_some_and(|g| g < 1e-6)),
            "{together:?}"
        );
    }

    /// Energy lost and then regained is a gain, though the object never
    /// gets back above where it started.
    #[test]
    fn energy_regained_after_it_was_lost_is_a_gain() {
        let gains = audit(&[&[2.0, 0.5, 1.5]]).gains();
        assert_eq!(gains, vec![Some(1.0)]);
    }

    /// Work a body outside the audit did on an object explains what the
    /// object gained by it.
    #[test]
    fn energy_given_from_outside_is_no_gain() {
        let mut given = audit(&[&[0.5, 2.0, 2.5]]);
        given.objects[0].received = vec![0.0, 1.5, 1.5];
        assert_eq!(given.gains(), vec![Some(0.5)]);
    }

    /// A group with an exempted object in it is not judged at all.
    #[test]
    fn an_exempted_group_is_not_judged() {
        let mut audit = audit(&[&[0.5, 2.0], &[0.5, 0.5]]);
        audit.join(0, 1);
        audit.exempt(1);
        assert_eq!(audit.gains(), vec![None, None]);
    }

    /// Energy per kilogram counts height in the world's gravity: a body a
    /// metre higher has 9.81 J/kg more.
    #[test]
    fn energy_per_kilogram_counts_height() {
        let mut world = PhysicsWorld::new(PhysicsConfig::default());
        let body = world.create_body(RigidBodyDesc::dynamic().position(Point3::origin()));
        world.attach_collider(body, ColliderDesc::sphere(0.5));
        let low = energy_per_kg(&world, &[body]);
        world
            .body_mut(body)
            .unwrap()
            .set_position(Point3::new(0.0, 1.0, 0.0));
        let high = energy_per_kg(&world, &[body]);
        assert!((high - low - 9.81).abs() < 0.01, "{low} then {high}");
    }
}
