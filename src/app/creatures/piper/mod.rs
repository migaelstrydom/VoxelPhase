//! Piper — a bagpipe that plays itself: a bag of pink skin like a stomach,
//! squatting on two small folded legs, with a bald head sewn into its mouth
//! and blowing.
//!
//! The ranged threat. It never moves. Whoever comes into its sight has a
//! bomb lobbed at them every few seconds, in a high slow arc, so a route it
//! overlooks is crossed in dashes; and the bombs are real ones, so the route
//! does not stay as it was. Its bag sags when it has thrown and swells tight
//! as the next throw comes, which is the warning. It bursts at a blow.
//! `visual_bench piper` shows it.
//!
//! ```text
//!   part      what it is                                      who moves it
//!   ────────  ──────────────────────────────────────────────  ─────────────
//!   BAG       the bag: veined, sagging in rolls, stitched     LobberSystem:
//!             along its seam, tied off behind                 swells, sags
//!   FIXTURES  drones, chanter and blowpipe of turned wood     nobody
//!             and brass; the head; the legs; a hand
//!   CHEEKS    the head's cheeks, blown out round the pipe     LobberSystem:
//!                                                             puff
//!   EYES      two bulging eyes, each about its own middle     LobberSystem:
//!                                                             follow you
//! ```
//!
//! ```text
//!   def      the spawnable: its body, its lobbing, the parts of its model
//!            and who poses them
//!   bag      the bag's form, its seam and thread, and its painted hide
//!   pipes    pipes of turned wood and brass: the drones, chanter, blowpipe
//!   head     the head, its neck, cheeks and eyes
//!   limbs    the folded legs, and the hand and arm on the chanter
//!   frame    the measures the parts are placed by
//!   shaping  placing, projecting and painting, shared by the above
//! ```

mod bag;
mod def;
mod frame;
mod head;
mod limbs;
mod pipes;
mod shaping;

pub use def::PiperDef;
