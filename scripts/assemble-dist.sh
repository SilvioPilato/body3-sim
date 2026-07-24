#!/usr/bin/env bash
# Build the wasm bundle and assemble dist/ — the single source of truth for the
# deployable artifact, used by both CI (.github/workflows/deploy.yml) and the
# local test helper (scripts/serve-web.sh).
#
# The Barnes-Hut force solver runs across a hand-rolled Web Worker pool
# (src/wasm_pool.rs), so the wasm is built with a shared, atomics-enabled linear
# memory. That needs a nightly toolchain (the prebuilt std has no atomics):
#   - nightly + rust-src, with -Z build-std (rebuilds std with atomics)
#   - target-feature=+atomics,+bulk-memory,+mutable-globals
#   - an imported+exported SHARED memory: index.html creates the
#     SharedArrayBuffer-backed WebAssembly.Memory and injects it as env.memory
#     via a miniquad plugin, and every worker instantiates the same module
#     against it
#   - the `threads` cargo feature, which enables the pool
# The extra symbol exports are the per-worker stack/TLS setup hooks and the pool
# entry points the workers and dispatcher call from JS.
#
# Still macroquad's own JS loader (mq_js_bundle.js), not wasm-bindgen.
# SharedArrayBuffer requires the COOP/COEP headers in the committed _headers.
set -euo pipefail
cd "$(dirname "$0")/.."

export RUSTFLAGS="\
-C target-feature=+atomics,+bulk-memory,+mutable-globals \
-C link-arg=--allow-undefined \
-C link-arg=--shared-memory \
-C link-arg=--import-memory \
-C link-arg=--export-memory \
-C link-arg=--initial-memory=67108864 \
-C link-arg=--max-memory=1073741824 \
-C link-arg=--export=__wasm_init_tls \
-C link-arg=--export=__tls_size \
-C link-arg=--export=__tls_align \
-C link-arg=--export=__tls_base \
-C link-arg=--export=__stack_pointer \
-C link-arg=--export=pool_alloc \
-C link-arg=--export=pool_run \
-C link-arg=--export=pool_worker_loop \
-C link-arg=--export=pool_worker_count \
-C link-arg=--export=pool_selfcheck"

echo "cargo +nightly build --release --target wasm32-unknown-unknown --features threads -Z build-std=std,panic_abort"
cargo +nightly build --release --target wasm32-unknown-unknown --features threads -Z build-std=std,panic_abort

WASM="target/wasm32-unknown-unknown/release/body3-sim.wasm"
[ -f "$WASM" ] || { echo "error: wasm artifact missing: $WASM" >&2; exit 1; }

echo "assembling dist/"
# Overwrite in place rather than rm -rf'ing the dir: the artifact is always the
# same four files (CI runs on a fresh checkout, so no stale files accumulate),
# and it avoids failing when a local file watcher holds the directory open.
mkdir -p dist
cp -f index.html mq_js_bundle.js _headers "$WASM" dist/

echo "dist/ ready:"
ls -1 dist/