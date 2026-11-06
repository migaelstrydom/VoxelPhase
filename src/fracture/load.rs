//! What each bond of a compound is being asked to hold, this frame, judged
//! by what it holds on.
//!
//! ```text
//!   blasts ────────────▶ loading per area at the torn-off ──┐
//!                         side's centre of mass               ├─▶ BondLoads::breaks
//!   contact spikes ────▶ largest on the torn-off side, ───────┘
//!   (per collider)        felt at the anchor over the bond's reach
//! ```
//!
//! A bond's **torn-off side** ([`TornSides`]) is what comes away if it alone
//! gives way: the side not held by the world through a tie, else the
//! lighter. A bond does not fail because the air where it holds is loaded
//! but because what hangs off it is pushed away from what holds it, so two
//! branches off one fork break apart from each other, each by the blast on
//! its own crown. A bond in a cycle, the world's ties included, frees
//! nothing on its own; it is a seam sharing a load that has other paths,
//! and is judged where it holds: the blast at its anchor, spikes on either
//! part.
//!
//! Two loads arrive by different routes and are *not* interchangeable numbers,
//! which is why they are kept apart all the way to the threshold test:
//!
//! - A **blast** is an analytic loading with its own distance falloff, read
//!   where the engine strikes the side it pushes: its centre of mass.
//! - A **contact spike** is momentum the solver actually pushed through a
//!   collider. For a heavy compound it is far larger — several tonnes of
//!   bridge settling puts more through one beam than any grenade — because
//!   it is the whole body's momentum funnelling through a few contacts.
//!
//! A single threshold cannot serve both: the plank bridge is authored to come
//! apart under an 8 N·s/m² blast, and its own weight moves 1700 N·s through a
//! beam when it settles. So blast is tested against the bond's `threshold`
//! and contact against its `toughness`, or the compound's default toughness
//! where the bond states none. A bond with neither never breaks on contact.
//!
//! ## Contact on the torn-off side
//!
//! A spike on the kept side moves the kept side, the held or heavier one;
//! what hangs off it follows at the share its own mass is of the whole. A
//! load that passes *through* the torn-off side — a table top struck while
//! its leg stands on the floor — shows on it as the spike where it is braced.
//! So only the torn-off side's spikes count.
//!
//! ## Locality
//!
//! A contact spike is felt at the bond as it falls off with distance from
//! where it landed, over the bond's `reach`. A part may be long and carry
//! many bonds — the bridge's beams run its whole length and every plank
//! hangs off them — and an impact at one end does not load a bond at the
//! other. Without this, one knock on a beam breaks all sixteen of its bonds
//! at once and the bridge does not shed planks, it ceases to exist.

use nalgebra::{Isometry3, Point3};
use rustc_hash::FxHashMap;

use super::contact_load::ContactSpike;
use super::parts::{Bond, PartGraph, PartId};
use super::torn_side::TornSides;
use crate::physics::{ColliderHandle, PhysicsImpulse};

/// The loads on one compound's bonds this frame.
pub struct BondLoads<'a> {
    /// This frame's contact spike on each collider.
    spikes: FxHashMap<ColliderHandle, ContactSpike>,
    /// The spikes that landed, each with the part it landed on.
    landed: Vec<(PartId, ContactSpike)>,
    /// This frame's blasts.
    blasts: &'a [PhysicsImpulse],
    /// The body's pose, which places bond anchors in the world.
    pose: Isometry3<f32>,
    /// What each bond tears off, centres in the world.
    sides: TornSides,
    /// Contact spike at a bond, N·s, above which a bond that states no
    /// toughness of its own lets go. `None`: such a bond never breaks on
    /// contact.
    default_toughness: Option<f32>,
}

impl<'a> BondLoads<'a> {
    /// The loads on `graph`'s bonds, on a body at `pose` whose bonds tear
    /// off `sides`.
    pub fn new(
        graph: &PartGraph,
        spikes: impl IntoIterator<Item = (ColliderHandle, ContactSpike)>,
        blasts: &'a [PhysicsImpulse],
        pose: Isometry3<f32>,
        sides: TornSides,
        default_toughness: Option<f32>,
    ) -> Self {
        let spikes: FxHashMap<ColliderHandle, ContactSpike> = spikes.into_iter().collect();
        let landed = graph
            .parts()
            .iter()
            .flat_map(|part| part.colliders.iter().map(move |c| (part.id, c)))
            .filter_map(|(part, collider)| Some((part, *spikes.get(collider)?)))
            .filter(|(_, spike)| spike.magnitude > 0.0)
            .collect();
        Self {
            spikes,
            landed,
            blasts,
            pose,
            sides,
            default_toughness,
        }
    }

    /// Whether `bond` is being asked to hold more than it can.
    pub fn breaks(&self, graph: &PartGraph, bond: &Bond) -> bool {
        if self.blast_load(bond) > bond.threshold {
            return true;
        }
        bond.toughness
            .or(self.default_toughness)
            .is_some_and(|toughness| self.contact_load(graph, bond) > toughness)
    }

    /// The blasts' loading per area, N·s/m², at the centre of mass of what
    /// `bond` tears off, or at its anchor if it tears nothing off.
    pub fn blast_load(&self, bond: &Bond) -> f32 {
        let at = self
            .sides
            .of_bond(bond)
            .map_or_else(|| self.pose * bond.anchor, |side| side.centre);
        self.blasts
            .iter()
            .filter_map(|blast| blast.loading_at(at))
            .map(|loading| loading.per_area.magnitude())
            .sum()
    }

    /// The contact spike this bond feels, N·s: the largest any collider of
    /// what it tears off took, or of either part if it tears nothing off,
    /// falling off with distance from where it landed to the anchor over the
    /// bond's reach.
    ///
    /// The quantity a bond's toughness is measured against, so the tests
    /// that choose that number read it directly.
    pub fn contact_load(&self, graph: &PartGraph, bond: &Bond) -> f32 {
        let anchor = self.pose * bond.anchor;
        let felt = |spike: &ContactSpike| felt_at(spike, anchor, bond.reach);
        match self.sides.of_bond(bond) {
            Some(side) => self
                .landed
                .iter()
                .filter(|(part, _)| self.sides.holds(side, *part))
                .map(|(_, spike)| felt(spike))
                .fold(0.0, f32::max),
            None => [bond.a, bond.b]
                .iter()
                .filter_map(|id| graph.part(*id))
                .flat_map(|part| part.colliders.iter())
                .filter_map(|collider| self.spikes.get(collider))
                .map(felt)
                .fold(0.0, f32::max),
        }
    }
}

/// Whether anything this frame could break one of `graph`'s bonds: a blast,
/// or a spike larger than the least toughness among them. A spike is felt at
/// a bond at most at full strength, so nothing smaller breaks anything, and
/// a frame without either need not judge the bonds at all.
pub fn may_break(
    graph: &PartGraph,
    spikes: &[ContactSpike],
    blasted: bool,
    default_toughness: Option<f32>,
) -> bool {
    if blasted {
        return true;
    }
    let least = graph
        .bonds()
        .iter()
        .filter_map(|bond| bond.toughness.or(default_toughness))
        .reduce(f32::min);
    least.is_some_and(|least| spikes.iter().any(|spike| spike.magnitude > least))
}

/// A spike as felt at `anchor`, falling off with distance from where it
/// landed over `reach`.
fn felt_at(spike: &ContactSpike, anchor: Point3<f32>, reach: f32) -> f32 {
    if spike.magnitude <= 0.0 || reach <= 0.0 {
        return spike.magnitude;
    }
    let ratio = (anchor - spike.point).magnitude() / reach;
    spike.magnitude / (1.0 + ratio * ratio)
}

#[cfg(test)]
mod tests {
    use generational_arena::Arena;
    use nalgebra::Vector3;

    use super::*;
    use crate::physics::ConstraintHandle;

    fn spike(magnitude: f32, z: f32) -> ContactSpike {
        ContactSpike {
            magnitude,
            point: Point3::new(0.0, 0.0, z),
        }
    }

    /// The loads on `graph`, each part weighing a kilogram at the origin.
    fn at_anchors<'a>(
        graph: &PartGraph,
        spikes: impl IntoIterator<Item = (ColliderHandle, ContactSpike)>,
        blasts: &'a [PhysicsImpulse],
        pose: Isometry3<f32>,
        default_toughness: Option<f32>,
    ) -> BondLoads<'a> {
        let sides = TornSides::of(graph, |_| Some((1.0, Point3::origin())));
        BondLoads::new(graph, spikes, blasts, pose, sides, default_toughness)
    }

    /// The bridge's shape: two long beams, two planks across them at their
    /// ends, each bond where a plank crosses a beam. The bonds on the first
    /// beam are returned: every bond is in a cycle, judged where it holds.
    fn beam_and_planks() -> (PartGraph, Bond, Bond) {
        let mut graph = PartGraph::default();
        let beam = graph.add_part([ColliderHandle::for_test(0)]);
        let near = graph.add_part([ColliderHandle::for_test(1)]);
        let far = graph.add_part([ColliderHandle::for_test(2)]);
        let other_beam = graph.add_part([ColliderHandle::for_test(3)]);
        let bond = |beam: PartId, plank: PartId, z: f32| Bond {
            a: beam,
            b: plank,
            anchor: Point3::new(0.0, 0.0, z),
            reach: 2.0,
            threshold: 8.0,
            toughness: None,
        };
        let (near_bond, far_bond) = (bond(beam, near, -3.5), bond(beam, far, 3.5));
        graph.bond(near_bond);
        graph.bond(far_bond);
        graph.bond(Bond {
            anchor: Point3::new(1.0, 0.0, -3.5),
            ..bond(other_beam, near, -3.5)
        });
        graph.bond(Bond {
            anchor: Point3::new(1.0, 0.0, 3.5),
            ..bond(other_beam, far, 3.5)
        });
        (graph, near_bond, far_bond)
    }

    /// A tree's shape: a trunk welded to the ground and two branches off
    /// one fork, sharing its anchor, their crowns out to either side.
    struct Fork {
        graph: PartGraph,
        left: Bond,
        right: Bond,
        sides: TornSides,
    }

    impl Fork {
        const LEFT_CROWN: Point3<f32> = Point3::new(-1.0, 3.0, 0.0);

        fn new() -> Self {
            let mut graph = PartGraph::default();
            let trunk = graph.add_part([ColliderHandle::for_test(0)]);
            let left = graph.add_part([ColliderHandle::for_test(1)]);
            let right = graph.add_part([ColliderHandle::for_test(2)]);
            graph.tie(ConstraintHandle(Arena::new().insert(())), trunk);
            let bond = |branch: PartId| Bond {
                a: trunk,
                b: branch,
                anchor: Point3::new(0.0, 2.0, 0.0),
                reach: 1.5,
                threshold: 20.0,
                toughness: Some(100.0),
            };
            let (to_left, to_right) = (bond(left), bond(right));
            graph.bond(to_left);
            graph.bond(to_right);
            let sides = TornSides::of(&graph, |part| {
                Some(match part {
                    p if p == trunk => (100.0, Point3::new(0.0, 1.0, 0.0)),
                    p if p == left => (10.0, Self::LEFT_CROWN),
                    _ => (10.0, Point3::new(1.0, 3.0, 0.0)),
                })
            });
            Self {
                graph,
                left: to_left,
                right: to_right,
                sides,
            }
        }

        fn loads<'a>(
            &self,
            spikes: impl IntoIterator<Item = (ColliderHandle, ContactSpike)>,
            blasts: &'a [PhysicsImpulse],
        ) -> BondLoads<'a> {
            BondLoads::new(
                &self.graph,
                spikes,
                blasts,
                Isometry3::identity(),
                self.sides.clone(),
                None,
            )
        }
    }

    /// Two branches off one fork: a blast beside one crown breaks that
    /// branch alone, though both are held at the same place, which the blast
    /// barely reaches.
    #[test]
    fn a_blast_beside_one_branch_breaks_only_that_branch() {
        let fork = Fork::new();
        let blasts = [PhysicsImpulse::radial(
            Fork::LEFT_CROWN + Vector3::new(-1.5, 0.0, 0.0),
            3.0,
            100.0,
            0.0,
        )];
        let loads = fork.loads([], &blasts);
        assert!(loads.blast_load(&fork.left) > fork.left.threshold);
        assert!(loads.breaks(&fork.graph, &fork.left));
        assert_eq!(loads.blast_load(&fork.right), 0.0);
        assert!(!loads.breaks(&fork.graph, &fork.right));
    }

    /// A blow on the trunk by the fork moves the trunk, not the branches; one
    /// on a crown loads its own fork alone.
    #[test]
    fn a_branch_feels_the_blows_on_itself() {
        let fork = Fork::new();
        let at = |p: Point3<f32>| ContactSpike {
            magnitude: 1_000.0,
            point: p,
        };
        let on_trunk = fork.loads(
            [(ColliderHandle::for_test(0), at(Point3::new(0.0, 2.0, 0.0)))],
            &[],
        );
        assert_eq!(on_trunk.contact_load(&fork.graph, &fork.left), 0.0);
        assert!(!on_trunk.breaks(&fork.graph, &fork.right));

        let on_crown = fork.loads([(ColliderHandle::for_test(1), at(Fork::LEFT_CROWN))], &[]);
        assert!(on_crown.breaks(&fork.graph, &fork.left));
        assert!(!on_crown.breaks(&fork.graph, &fork.right));
    }

    /// Nothing breaks a bond in a frame with no blast and no spike past the
    /// least toughness, so such a frame is not judged.
    #[test]
    fn a_quiet_frame_cannot_break_anything() {
        let fork = Fork::new();
        let spike = |magnitude| ContactSpike {
            magnitude,
            point: Point3::origin(),
        };
        assert!(!may_break(&fork.graph, &[spike(99.0)], false, None));
        assert!(may_break(&fork.graph, &[spike(101.0)], false, None));
        assert!(may_break(&fork.graph, &[], true, None));
        let (graph, _, _) = beam_and_planks();
        assert!(!may_break(&graph, &[spike(1.0e9)], false, None));
        assert!(may_break(&graph, &[spike(11.0)], false, Some(10.0)));
    }

    /// A hit at one end of a long part does not reach its distant bonds.
    #[test]
    fn a_hit_on_a_long_part_does_not_reach_its_distant_bonds() {
        let (graph, near, far) = beam_and_planks();
        let loads = at_anchors(
            &graph,
            [(ColliderHandle::for_test(0), spike(10_000.0, -3.5))],
            &[],
            Isometry3::identity(),
            Some(1_000.0),
        );
        assert!(loads.breaks(&graph, &near), "the struck end should let go");
        assert!(!loads.breaks(&graph, &far), "the far end should hold");
    }

    /// A blast is read where the bond is: one whose reach ends short of a
    /// bond does not break it, however hard it strikes the parts' far ends.
    #[test]
    fn a_blast_is_judged_at_the_anchor() {
        let (graph, near, far) = beam_and_planks();
        let blasts = [PhysicsImpulse::radial(
            Point3::new(0.0, 0.0, -4.0),
            2.0,
            100.0,
            0.0,
        )];
        let loads = at_anchors(&graph, [], &blasts, Isometry3::identity(), Some(1_000.0));
        assert!(loads.blast_load(&near) > near.threshold);
        assert!(loads.breaks(&graph, &near));
        assert_eq!(loads.blast_load(&far), 0.0);
        assert!(!loads.breaks(&graph, &far));
    }

    /// Anchors are in the body's frame, so they are judged where the body
    /// has taken them.
    #[test]
    fn anchors_are_placed_by_the_body() {
        let (graph, near, _) = beam_and_planks();
        let blasts = [PhysicsImpulse::radial(
            Point3::new(10.0, 0.0, -3.5),
            1.0,
            100.0,
            0.0,
        )];
        let moved = Isometry3::translation(10.0, 0.0, 0.0);
        let loads = at_anchors(&graph, [], &blasts, moved, None);
        assert!(loads.breaks(&graph, &near));
    }

    /// A bond's own toughness overrides the compound's default, either way.
    #[test]
    fn a_bond_s_toughness_overrides_the_compound_s() {
        let (graph, near, _) = beam_and_planks();
        let loads = at_anchors(
            &graph,
            [(ColliderHandle::for_test(1), spike(50.0, -3.5))],
            &[],
            Isometry3::identity(),
            Some(100.0),
        );
        assert!(!loads.breaks(&graph, &near));
        let brittle = Bond {
            toughness: Some(20.0),
            ..near
        };
        assert!(loads.breaks(&graph, &brittle));
        let tough = Bond {
            toughness: Some(500.0),
            ..near
        };
        let defaulted_low = at_anchors(
            &graph,
            [(ColliderHandle::for_test(1), spike(50.0, -3.5))],
            &[],
            Isometry3::identity(),
            Some(10.0),
        );
        assert!(!defaulted_low.breaks(&graph, &tough));
    }

    /// A bond with no toughness, on a compound with no default, never breaks
    /// on contact, however hard it is hit; a blast still breaks it.
    #[test]
    fn a_bond_without_toughness_never_breaks_on_contact() {
        let (graph, near, _) = beam_and_planks();
        let loads = at_anchors(
            &graph,
            [(ColliderHandle::for_test(1), spike(1.0e9, -3.5))],
            &[],
            Isometry3::identity(),
            None,
        );
        assert!(!loads.breaks(&graph, &near));
    }
}
