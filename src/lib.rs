// The Web Worker pool uses nightly wasm atomic-wait intrinsics; the attribute
// is scoped to the `threads` feature so stable builds are unaffected.
#![cfg_attr(feature = "threads", feature(stdarch_wasm_atomic_wait))]

pub mod camera;
pub mod energy;
pub mod physics;
pub mod quadtree;
pub mod simulation;
pub mod url;

#[cfg(all(target_arch = "wasm32", feature = "threads"))]
pub mod wasm_pool;