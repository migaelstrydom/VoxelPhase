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
//!   part       what it is                                   who moves it
//!   ─────────  ───────────────────────────────────────────  ──────────────
//!   BAG        the bag: veined, sagging in rolls, stitched  LobberSystem:
//!              along its seam, tied off behind              swells, sags
//!   HEAD       the head, its neck and the blowpipe in its   rides the bag
//!              lips                                         at its collar
//!   DRONES     three drones of turned wood and brass        each rides the
//!                                                           bag at its stock
//!   CHANTER    the chanter and the hand on it               rides the bag
//!                                                           at its stock
//!   FOREARM,   the arm, its elbow on a thigh                turns on the
//!   UPPER_ARM                                               elbow to keep
//!                                                           hold
//!   LEGS       the folded legs                              nobody
//!   EYES       two bulging eyes, each about its own middle  follow you, in
//!                                                           the head
//!   cheeks     the face blown out round the pipe: not a     puff, in the
//!              part but a mesh shaped each frame            head
//! ```
//!
//! What is fastened into the bag rides it as it swells and breathes, so a
//! pipe stays in its stock and the head in its collar; the blowpipe, held
//! in the lips, slides in and out of its stock instead.
//!
//! ```text
//!   def      the spawnable: its body, its lobbing, the parts of its model
//!            and who poses them
//!   bag      the bag's form, its seam and thread, and its painted hide
//!   pipes    pipes of turned wood and brass: the drones, chanter, blowpipe
//!   head     the head, its neck, eyes, and the cheeks blown out of it
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
