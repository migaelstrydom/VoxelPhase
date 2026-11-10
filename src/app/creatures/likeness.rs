//! Creatures' sculpted looks, made once and shared.
//!
//! ```text
//!   kind ──first asked for──▶ sculpt()           tens of thousands of
//!                              │                  triangles, a moment's work
//!   (kind, level's ids) ──▶ recoloured ──▶ Arc    one upload, drawn for every
//!                                                 creature of the kind
//! ```
//!
//! Sculpting is too slow to do per creature, and a model drawn in a level
//! names that level's materials, so there are two things to keep: the
//! sculpture against the palette's slots, for as long as the game runs, and
//! its copy in the current level's materials.

use std::sync::{Arc, Mutex};

use rustc_hash::FxHashMap;

use super::palette::in_level;
use crate::animation::stilts::Dress;
use crate::model::Model;
use crate::rendering::material::MaterialId;

/// One kind's look: as sculpted, and as last drawn in a level's materials.
struct Kept<T> {
    sculpted: Arc<T>,
    in_level: Option<(Vec<MaterialId>, Arc<T>)>,
}

/// The looks kept so far, by kind.
struct Looks<T>(Mutex<Option<FxHashMap<&'static str, Kept<T>>>>);

impl<T> Looks<T> {
    const fn new() -> Self {
        Self(Mutex::new(None))
    }

    /// `kind`'s look in the level whose bestiary materials are `ids`.
    fn get(
        &self,
        kind: &'static str,
        ids: &[MaterialId],
        sculpt: impl FnOnce() -> T,
        recolour: impl FnOnce(&T, &[MaterialId]) -> T,
    ) -> Arc<T> {
        let mut looks = self
            .0
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let kept = looks
            .get_or_insert_with(FxHashMap::default)
            .entry(kind)
            .or_insert_with(|| Kept {
                sculpted: Arc::new(sculpt()),
                in_level: None,
            });
        match &kept.in_level {
            Some((held, look)) if held == ids => look.clone(),
            _ => {
                let look = Arc::new(recolour(&kept.sculpted, ids));
                kept.in_level = Some((ids.to_vec(), look.clone()));
                look
            }
        }
    }

    /// `kind`'s look against the palette's slots.
    fn sculpted(&self, kind: &'static str, sculpt: impl FnOnce() -> T) -> Arc<T> {
        let mut looks = self
            .0
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner());
        let kept = looks
            .get_or_insert_with(FxHashMap::default)
            .entry(kind)
            .or_insert_with(|| Kept {
                sculpted: Arc::new(sculpt()),
                in_level: None,
            });
        kept.sculpted.clone()
    }
}

static DRESSES: Looks<Dress> = Looks::new();
static MODELS: Looks<Model> = Looks::new();

/// The dress of the walkers of `kind`, drawn in the level's bestiary
/// materials `ids`; `sculpt` makes it against the palette's slots, the
/// first time.
pub fn dress(kind: &'static str, ids: &[MaterialId], sculpt: fn() -> Dress) -> Arc<Dress> {
    DRESSES.get(kind, ids, sculpt, |dress, ids| {
        dress.recoloured(in_level(ids))
    })
}

/// The dress of the walkers of `kind` against the palette's slots: for
/// tools that draw it with the palette's own materials.
pub fn dress_sculpted(kind: &'static str, sculpt: fn() -> Dress) -> Arc<Dress> {
    DRESSES.sculpted(kind, sculpt)
}

/// The model of the creatures of `kind`, as [`dress`] is for walkers.
pub fn model(kind: &'static str, ids: &[MaterialId], sculpt: fn() -> Model) -> Arc<Model> {
    MODELS.get(kind, ids, sculpt, |model, ids| {
        let mut model = model.clone();
        let to_level = in_level(ids);
        for primitive in model.parts.iter_mut().flat_map(|part| &mut part.primitives) {
            primitive.material = to_level(primitive.material);
        }
        model
    })
}

/// The model of the creatures of `kind` against the palette's slots.
pub fn model_sculpted(kind: &'static str, sculpt: fn() -> Model) -> Arc<Model> {
    MODELS.sculpted(kind, sculpt)
}
