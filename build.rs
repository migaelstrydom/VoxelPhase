//! Compiles every GLSL shader under `shader/` to SPIR-V in `OUT_DIR`, where
//! `embedded_spirv!` includes it from.
//!
//! `shader/fire/advect.comp` becomes `$OUT_DIR/shader/fire/advect.comp.spv`.
//! Headers (`.glsl`) are only ever `#include`d, so they are not compiled on
//! their own; any change under `shader/` recompiles everything.
//!
//! `glslc` is found, in order, at `$GLSLC`, at `$VULKAN_SDK/bin/glslc`, and on
//! `PATH`. The Vulkan SDK ships it.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};

/// Root of the GLSL sources, relative to the manifest.
const SHADER_DIR: &str = "shader";

/// Extensions of the shader stages compiled to SPIR-V.
const STAGE_EXTENSIONS: [&str; 3] = ["vert", "frag", "comp"];

fn main() {
    println!("cargo:rerun-if-changed={SHADER_DIR}");
    println!("cargo:rerun-if-env-changed=GLSLC");
    println!("cargo:rerun-if-env-changed=VULKAN_SDK");

    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("cargo sets OUT_DIR"));
    let glslc = find_glslc();

    let mut sources = Vec::new();
    collect_stage_sources(Path::new(SHADER_DIR), &mut sources);
    sources.sort();

    let jobs: Vec<(PathBuf, Child)> = sources
        .into_iter()
        .map(|source| {
            let child = spawn_compile(&glslc, &source, &out_dir);
            (source, child)
        })
        .collect();

    let failures: Vec<String> = jobs
        .into_iter()
        .filter_map(|(source, child)| {
            let output = child.wait_with_output().expect("glslc runs to completion");
            (!output.status.success()).then(|| {
                format!(
                    "{}:\n{}",
                    source.display(),
                    String::from_utf8_lossy(&output.stderr)
                )
            })
        })
        .collect();

    if !failures.is_empty() {
        panic!("shader compilation failed:\n\n{}", failures.join("\n"));
    }
}

/// Every shader stage source under `dir`, recursively.
fn collect_stage_sources(dir: &Path, sources: &mut Vec<PathBuf>) {
    let entries = fs::read_dir(dir).unwrap_or_else(|e| panic!("reading {}: {e}", dir.display()));
    for entry in entries {
        let path = entry.expect("directory entry is readable").path();
        if path.is_dir() {
            collect_stage_sources(&path, sources);
        } else if path
            .extension()
            .and_then(|ext| ext.to_str())
            .is_some_and(|ext| STAGE_EXTENSIONS.contains(&ext))
        {
            sources.push(path);
        }
    }
}

/// Starts `glslc` on one source, writing `$OUT_DIR/<source>.spv`.
fn spawn_compile(glslc: &Path, source: &Path, out_dir: &Path) -> Child {
    let mut output = out_dir.join(source).into_os_string();
    output.push(".spv");
    let output = PathBuf::from(output);
    fs::create_dir_all(output.parent().expect("output has a parent"))
        .expect("create the SPIR-V output directory");

    Command::new(glslc)
        .arg("-I")
        .arg(SHADER_DIR)
        .arg(source)
        .arg("-o")
        .arg(&output)
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap_or_else(|e| panic!("starting {}: {e}", glslc.display()))
}

/// Locates `glslc`, or fails the build saying how to provide it.
fn find_glslc() -> PathBuf {
    if let Some(path) = env::var_os("GLSLC") {
        return PathBuf::from(path);
    }
    if let Some(sdk) = env::var_os("VULKAN_SDK") {
        let path = Path::new(&sdk).join("bin").join("glslc");
        if path.is_file() {
            return path;
        }
    }
    let on_path = Command::new("glslc")
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .is_ok_and(|status| status.success());
    if on_path {
        return PathBuf::from("glslc");
    }
    panic!(
        "glslc not found. Install the Vulkan SDK (https://vulkan.lunarg.com/sdk/home) and \
         either source its setup-env.sh, which sets VULKAN_SDK, put glslc on PATH, or set \
         GLSLC to its full path."
    );
}
