#!/usr/bin/env bash
# Local web smoke-test: build + assemble dist/, then serve it with the COOP/COEP
# headers SharedArrayBuffer requires. Open the printed URL in a browser.
# Usage: bash scripts/serve-web.sh [port]   (default 8080)
set -euo pipefail
cd "$(dirname "$0")/.."
bash scripts/assemble-dist.sh
PY="$(command -v python3 || command -v python)"
exec "$PY" scripts/coop-server.py dist "${1:-8080}"
