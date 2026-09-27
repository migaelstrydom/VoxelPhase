use nalgebra::Vector3;
use rustc_hash::FxHashSet;

use crate::rendering::reflection::owner::ProbeOwner;

/// How much nearer to the camera a newcomer must be than an object holding a
/// probe to take its place, as a factor on the holder's distance.
///
/// Without it two objects at nearly the same distance trade a probe back and
/// forth as the camera moves, and each trade costs a full six-face capture.
const HOLDER_ADVANTAGE: f32 = 0.75;

/// Frames a probe is kept after its object stops asking for one — half a
/// second at 60 Hz — unless a newcomer needs the slot.
///
/// An object stops asking when it leaves the camera's view. Turning away
/// and back is common, and without this every return would cost the object a
/// frame of sky and a full six-face capture. A lingering probe is not
/// refreshed, so keeping it costs nothing but the slot.
pub const LINGER_FRAMES: u32 = 30;

/// One probe's place in the atlas: its cube is layers `6 * slot` to
/// `6 * slot + 5`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ProbeSlot(pub u32);

/// A live probe.
#[derive(Clone, Copy, Debug)]
pub struct Probe {
    /// The object it serves.
    pub owner: ProbeOwner,
    /// Where its faces are captured from: the object's origin, as of the last
    /// time the object was drawn.
    pub centre: Vector3<f32>,
    /// Whether all six faces hold a capture of this owner's surroundings.
    /// False from admission until its first capture is recorded, and until
    /// then the probe's faces hold whatever the slot's last owner left.
    pub captured: bool,
    /// Frames since its object last asked for it. Zero while the object is
    /// asking; a probe whose object has stopped lingers until this passes
    /// [`LINGER_FRAMES`], and is not refreshed meanwhile.
    pub idle: u32,
}

/// An object asking for a probe this frame.
#[derive(Clone, Copy, Debug)]
struct Request {
    owner: ProbeOwner,
    centre: Vector3<f32>,
    /// From the camera. Nearer objects are served first.
    distance: f32,
}

/// Which objects hold the atlas's probes.
///
/// ```text
///   frame n:    draw ──request──▶ held? ── yes ──▶ slot, sampled this frame
///                                        └─ no ──▶ remembered, sky this frame
///   frame n+1:  begin_frame: rank frame n's requests by distance;
///               holders outside the top `capacity` let go,
///               holders that did not ask linger a while, then let go,
///               the nearest newcomers take the free slots, and then the
///               slots of the longest-lingering probes
/// ```
///
/// Slots change hands only at the start of a frame, never while one is
/// being drawn. That is what lets the renderer tell, before any draw of a
/// frame is issued, whether any probe is live, and so whether to collect the
/// frame's draws for capture at all; an object admitted mid-frame would
/// otherwise be captured without whatever had already been drawn.
pub struct ProbePool {
    slots: Vec<Option<Probe>>,
    /// This frame's requests, ranked at the next [`Self::begin_frame`].
    requests: Vec<Request>,
}

impl ProbePool {
    pub fn new(capacity: u32) -> Self {
        Self {
            slots: vec![None; capacity as usize],
            requests: Vec::new(),
        }
    }

    /// Hand the slots out for a new frame, from what was asked for in the
    /// last one: the `capacity` nearest objects keep or get a probe, objects
    /// that did not ask keep theirs for a while as long as no one else needs
    /// it, and everything else lets go.
    ///
    /// At most `max_admissions` newcomers are admitted, because each costs
    /// six faces on its first frame; the rest wait for a later one.
    pub fn begin_frame(&mut self, max_admissions: usize) {
        let mut requests = std::mem::take(&mut self.requests);
        let holding: FxHashSet<ProbeOwner> = self.live().map(|(_, probe)| probe.owner).collect();
        let ranked_distance = |request: &Request| {
            if holding.contains(&request.owner) {
                request.distance * HOLDER_ADVANTAGE
            } else {
                request.distance
            }
        };
        requests.sort_by(|a, b| ranked_distance(a).total_cmp(&ranked_distance(b)));
        // An object drawn twice in a frame asked twice; it holds one probe.
        let mut seen = FxHashSet::default();
        requests.retain(|request| seen.insert(request.owner));
        let asked = seen;
        requests.truncate(self.slots.len());

        let kept: FxHashSet<ProbeOwner> = requests.iter().map(|request| request.owner).collect();
        for slot in &mut self.slots {
            let Some(probe) = slot else {
                continue;
            };
            if kept.contains(&probe.owner) {
                probe.idle = 0;
            } else if asked.contains(&probe.owner) {
                // Asked, and outranked: someone nearer needs the slot now.
                *slot = None;
            } else {
                probe.idle += 1;
                if probe.idle > LINGER_FRAMES {
                    *slot = None;
                }
            }
        }

        let newcomers = requests
            .iter()
            .filter(|request| !holding.contains(&request.owner))
            .take(max_admissions);
        for request in newcomers {
            let Some(index) = self.slot_for_newcomer() else {
                break;
            };
            self.slots[index] = Some(Probe {
                owner: request.owner,
                centre: request.centre,
                captured: false,
                idle: 0,
            });
        }
    }

    /// Where a newcomer goes: a free slot, or else the one whose probe has
    /// lingered longest. `None` when every probe is wanted.
    fn slot_for_newcomer(&self) -> Option<usize> {
        if let Some(free) = self.slots.iter().position(Option::is_none) {
            return Some(free);
        }
        self.slots
            .iter()
            .enumerate()
            .filter_map(|(index, slot)| slot.map(|probe| (index, probe.idle)))
            .filter(|&(_, idle)| idle > 0)
            .max_by_key(|&(_, idle)| idle)
            .map(|(index, _)| index)
    }

    /// Ask for a probe for `owner`, centred on `centre`, `distance` from the
    /// camera. Returns its slot when it already holds one; otherwise the
    /// request is considered at the next [`Self::begin_frame`].
    ///
    /// A holder's probe moves with it: its centre is updated here, and every
    /// face captured from now on is captured from there.
    pub fn request(
        &mut self,
        owner: ProbeOwner,
        centre: Vector3<f32>,
        distance: f32,
    ) -> Option<ProbeSlot> {
        self.requests.push(Request {
            owner,
            centre,
            distance,
        });
        let index = self
            .slots
            .iter()
            .position(|slot| slot.is_some_and(|probe| probe.owner == owner))?;
        if let Some(probe) = &mut self.slots[index] {
            probe.centre = centre;
        }
        Some(ProbeSlot(index as u32))
    }

    /// Every live probe, in slot order.
    pub fn live(&self) -> impl Iterator<Item = (ProbeSlot, &Probe)> {
        self.slots
            .iter()
            .enumerate()
            .filter_map(|(index, slot)| slot.as_ref().map(|probe| (ProbeSlot(index as u32), probe)))
    }

    /// The live probe in `slot`.
    pub fn get(&self, slot: ProbeSlot) -> Option<&Probe> {
        self.slots.get(slot.0 as usize)?.as_ref()
    }

    /// Whether an object asked for a probe this frame that it does not hold,
    /// while a slot stands free for it: the next frame will admit it.
    pub fn has_waiting(&self) -> bool {
        let free = self
            .slots
            .iter()
            .any(|slot| slot.is_none_or(|probe| probe.idle > 0));
        free && self.requests.iter().any(|request| {
            !self
                .slots
                .iter()
                .any(|slot| slot.is_some_and(|probe| probe.owner == request.owner))
        })
    }

    /// Whether no probe is live.
    pub fn is_empty(&self) -> bool {
        self.slots.iter().all(Option::is_none)
    }

    /// Record that all six of `slot`'s faces have been captured.
    pub fn mark_captured(&mut self, slot: ProbeSlot) {
        if let Some(Some(probe)) = self.slots.get_mut(slot.0 as usize) {
            probe.captured = true;
        }
    }

    /// Let go of every probe, as when probes are switched off.
    pub fn clear(&mut self) {
        self.slots.iter_mut().for_each(|slot| *slot = None);
        self.requests.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ask(pool: &mut ProbePool, owner: u64, distance: f32) -> Option<ProbeSlot> {
        pool.request(ProbeOwner(owner), Vector3::zeros(), distance)
    }

    fn owners(pool: &ProbePool) -> Vec<u64> {
        let mut owners: Vec<u64> = pool.live().map(|(_, probe)| probe.owner.0).collect();
        owners.sort();
        owners
    }

    #[test]
    fn a_request_is_served_from_the_next_frame() {
        let mut pool = ProbePool::new(4);
        pool.begin_frame(4);
        assert_eq!(ask(&mut pool, 1, 5.0), None);
        pool.begin_frame(4);
        assert_eq!(ask(&mut pool, 1, 5.0), Some(ProbeSlot(0)));
        assert!(!pool.get(ProbeSlot(0)).unwrap().captured);
    }

    #[test]
    fn the_nearest_objects_hold_the_probes() {
        let mut pool = ProbePool::new(2);
        for (owner, distance) in [(1, 30.0), (2, 5.0), (3, 10.0)] {
            ask(&mut pool, owner, distance);
        }
        pool.begin_frame(8);
        assert_eq!(owners(&pool), vec![2, 3]);
    }

    #[test]
    fn an_object_that_stops_asking_lingers_then_lets_go() {
        let mut pool = ProbePool::new(2);
        ask(&mut pool, 1, 5.0);
        pool.begin_frame(8);
        for _ in 0..LINGER_FRAMES {
            pool.begin_frame(8);
            assert_eq!(owners(&pool), vec![1]);
        }
        pool.begin_frame(8);
        assert!(pool.is_empty());
    }

    #[test]
    fn a_returning_object_gets_its_lingering_probe_back() {
        let mut pool = ProbePool::new(2);
        ask(&mut pool, 1, 5.0);
        pool.begin_frame(8);
        let slot = ask(&mut pool, 1, 5.0);
        pool.begin_frame(8);
        pool.begin_frame(8);
        assert_eq!(ask(&mut pool, 1, 5.0), slot);
        pool.begin_frame(8);
        assert_eq!(pool.get(slot.unwrap()).unwrap().idle, 0);
    }

    #[test]
    fn a_newcomer_takes_the_longest_lingering_slot() {
        let mut pool = ProbePool::new(2);
        ask(&mut pool, 1, 5.0);
        pool.begin_frame(8);
        ask(&mut pool, 2, 5.0);
        pool.begin_frame(8);
        // 1 has lingered two frames, 2 one; both idle now.
        pool.begin_frame(8);
        ask(&mut pool, 3, 5.0);
        ask(&mut pool, 4, 5.0);
        pool.begin_frame(8);
        assert_eq!(owners(&pool), vec![3, 4]);
    }

    #[test]
    fn a_nearer_newcomer_outranks_a_holder_still_asking() {
        let mut pool = ProbePool::new(2);
        ask(&mut pool, 3, 5.0);
        ask(&mut pool, 4, 6.0);
        pool.begin_frame(8);
        ask(&mut pool, 3, 5.0);
        ask(&mut pool, 4, 6.0);
        ask(&mut pool, 5, 1.0);
        pool.begin_frame(8);
        assert_eq!(owners(&pool), vec![3, 5]);
    }

    #[test]
    fn a_holder_keeps_its_probe_against_a_slightly_nearer_newcomer() {
        let mut pool = ProbePool::new(1);
        ask(&mut pool, 1, 10.0);
        pool.begin_frame(8);

        ask(&mut pool, 1, 10.0);
        ask(&mut pool, 2, 9.0);
        pool.begin_frame(8);
        assert_eq!(owners(&pool), vec![1]);

        ask(&mut pool, 1, 10.0);
        ask(&mut pool, 2, 5.0);
        pool.begin_frame(8);
        assert_eq!(owners(&pool), vec![2]);
    }

    #[test]
    fn admissions_per_frame_are_capped() {
        let mut pool = ProbePool::new(8);
        let ask_all = |pool: &mut ProbePool| {
            for owner in 0..5 {
                ask(pool, owner, owner as f32);
            }
        };
        ask_all(&mut pool);
        pool.begin_frame(2);
        assert_eq!(owners(&pool), vec![0, 1]);
        ask_all(&mut pool);
        pool.begin_frame(2);
        assert_eq!(owners(&pool), vec![0, 1, 2, 3]);
    }

    #[test]
    fn a_request_waits_only_while_a_slot_is_free() {
        let mut pool = ProbePool::new(1);
        ask(&mut pool, 1, 1.0);
        assert!(pool.has_waiting());
        pool.begin_frame(4);
        ask(&mut pool, 1, 1.0);
        assert!(!pool.has_waiting());
        ask(&mut pool, 2, 5.0);
        assert!(!pool.has_waiting());
    }

    #[test]
    fn a_holder_keeps_its_slot() {
        let mut pool = ProbePool::new(4);
        ask(&mut pool, 7, 1.0);
        pool.begin_frame(4);
        let slot = ask(&mut pool, 7, 1.0);
        ask(&mut pool, 3, 0.5);
        pool.begin_frame(4);
        assert_eq!(ask(&mut pool, 7, 1.0), slot);
    }
}
