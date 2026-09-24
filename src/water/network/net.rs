//! The network's tables: every store and link, indexed by id.
//!
//! Stores and links live in `Vec`s and are never moved, so iteration follows
//! id order and is deterministic. A removed entry leaves an empty slot; ids
//! are never reused, so a body's id names it for its whole life and no
//! longer.

use crate::water::ids::{LinkId, StoreId};

use super::link::Link;
use super::store::{Port, Store, StoreView};

/// A link and the stores it joins.
#[derive(Debug)]
pub struct LinkEntry {
    pub up: StoreId,
    pub down: StoreId,
    pub law: Box<dyn Link>,
    /// A closed link carries exactly zero (§7.3).
    pub open: bool,
    pub up_port: Port,
    pub down_port: Port,
}

/// Every store and link.
#[derive(Debug, Default)]
pub struct Network {
    stores: Vec<Option<Store>>,
    links: Vec<Option<LinkEntry>>,
}

impl Network {
    pub fn add_store(&mut self, store: Store) -> StoreId {
        self.stores.push(Some(store));
        StoreId(self.stores.len() as u32 - 1)
    }

    /// Remove a store and every link touching it, returning it.
    pub fn remove_store(&mut self, id: StoreId) -> Option<Store> {
        for slot in self.links.iter_mut() {
            if slot.as_ref().is_some_and(|l| l.up == id || l.down == id) {
                *slot = None;
            }
        }
        self.stores.get_mut(id.0 as usize)?.take()
    }

    pub fn store(&self, id: StoreId) -> Option<&Store> {
        self.stores.get(id.0 as usize)?.as_ref()
    }

    pub fn store_mut(&mut self, id: StoreId) -> Option<&mut Store> {
        self.stores.get_mut(id.0 as usize)?.as_mut()
    }

    /// Every live store, in id order.
    pub fn stores(&self) -> impl Iterator<Item = (StoreId, &Store)> {
        self.stores
            .iter()
            .enumerate()
            .filter_map(|(i, s)| s.as_ref().map(|s| (StoreId(i as u32), s)))
    }

    /// Ids of every live store.
    pub fn store_ids(&self) -> Vec<StoreId> {
        self.stores().map(|(id, _)| id).collect()
    }

    /// One past the highest store id ever issued.
    pub fn store_slots(&self) -> usize {
        self.stores.len()
    }

    pub fn add_link(&mut self, link: LinkEntry) -> LinkId {
        self.links.push(Some(link));
        LinkId(self.links.len() as u32 - 1)
    }

    pub fn remove_link(&mut self, id: LinkId) -> Option<LinkEntry> {
        self.links.get_mut(id.0 as usize)?.take()
    }

    pub fn link(&self, id: LinkId) -> Option<&LinkEntry> {
        self.links.get(id.0 as usize)?.as_ref()
    }

    pub fn link_mut(&mut self, id: LinkId) -> Option<&mut LinkEntry> {
        self.links.get_mut(id.0 as usize)?.as_mut()
    }

    /// Every live link, in id order.
    pub fn links(&self) -> impl Iterator<Item = (LinkId, &LinkEntry)> {
        self.links
            .iter()
            .enumerate()
            .filter_map(|(i, l)| l.as_ref().map(|l| (LinkId(i as u32), l)))
    }

    /// Water held in every finite store, m³.
    pub fn held_volume(&self) -> f64 {
        self.stores()
            .filter(|(_, s)| s.is_finite())
            .map(|(_, s)| s.volume())
            .sum()
    }

    /// A store seen from a port, at `volume`.
    pub fn view(&self, id: StoreId, volume: f64, port: Port) -> Option<StoreView<'_>> {
        Some(StoreView {
            store: self.store(id)?,
            volume,
            port,
        })
    }
}
