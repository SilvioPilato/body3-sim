#!/usr/bin/env bash
# Build the wasm bundle and assemble dist/ — the single source of truth for the
# deployable artifact, used by both CI (.github/workflows/deploy.yml) and the
# local test helper (scripts/serve-web.sh). macroquad uses miniquad's own JS
# loader (mq_js_bundle.js + load(".wasm")), not wasm-bindgen, so the wasm is
# built directly with cargo — there is no trunk/wasm-bindgen step.
set -euo pipefail
cd "$(dirname "$0")/.."

echo "cargo build --release --target wasm32-unknown-unknown"
cargo build --release --target wasm32-unknown-unknown

WASM="target/wasm32-unknown-unknown/release/body3-sim.wasm"
[ -f "$WASM" ] || { echo "error: wasm artifact missing: $WASM" >&2; exit 1; }

echo "assembling dist/"
rm -rf dist
mkdir -p dist
cp index.html mq_js_bundle.js _headers "$WASM" dist/

echo "dist/ ready:"
ls -1 dist/
