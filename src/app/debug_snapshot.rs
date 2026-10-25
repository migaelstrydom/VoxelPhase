//! What F3 keeps: a screenshot of the game and the frame's debug log, side by
//! side in `debug_snapshots/`.
//!
//! ```text
//!   F3, frame N    ──▶ debug log printed, kept; renderer asked for a picture
//!   frame N+1 draws ──▶ Screenshot reads it back before present
//!   end of frame N+1 ──▶ snapshot-<ms>.png + snapshot-<ms>.txt written
//! ```

use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

use specs::World;

use crate::rendering::renderer::Renderer;

/// Where snapshots are written, relative to the working directory.
const SNAPSHOT_DIR: &str = "debug_snapshots";

/// A snapshot asked for whose picture has not been drawn yet.
#[derive(Default)]
pub struct PendingSnapshot {
    /// The debug log of the frame F3 was pressed on.
    log: Option<String>,
}

/// Asks the renderer for the next frame's picture and keeps `log` to write
/// beside it. Does nothing in a world without a renderer.
pub fn request_snapshot(world: &mut World, log: String) {
    let Some(mut renderer) = world.try_fetch_mut::<Renderer>() else {
        return;
    };
    renderer.screenshot.request();
    drop(renderer);
    world
        .entry::<PendingSnapshot>()
        .or_insert_with(PendingSnapshot::default)
        .log = Some(log);
}

/// Writes the snapshot once the renderer has its picture.
pub fn write_finished_snapshot(world: &mut World) {
    let Some(picture) = world
        .try_fetch_mut::<Renderer>()
        .and_then(|mut renderer| renderer.screenshot.take())
    else {
        return;
    };
    let log = world
        .try_fetch_mut::<PendingSnapshot>()
        .and_then(|mut pending| pending.log.take())
        .unwrap_or_default();
    let stem = snapshot_stem(Path::new(SNAPSHOT_DIR));
    let written = std::fs::create_dir_all(SNAPSHOT_DIR)
        .map_err(|error| error.to_string())
        .and_then(|()| {
            picture
                .save(stem.with_extension("png"))
                .map_err(|error| error.to_string())
        })
        .and_then(|()| {
            std::fs::write(stem.with_extension("txt"), log).map_err(|error| error.to_string())
        });
    match written {
        Ok(()) => log::info!("Snapshot written to {}.png/.txt", stem.display()),
        Err(error) => log::error!("Snapshot not written to {}: {error}", stem.display()),
    }
}

/// `dir/snapshot-<milliseconds since the epoch>`, without an extension.
fn snapshot_stem(dir: &Path) -> PathBuf {
    let millis = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |since| since.as_millis());
    dir.join(format!("snapshot-{millis}"))
}
