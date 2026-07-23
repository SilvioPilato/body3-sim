# Cloudflare Pages + GitHub Actions Deploy — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the local PowerShell `gh-pages` deploy with push-to-deploy to Cloudflare Pages via GitHub Actions (production on `main` + per-PR preview URLs).

**Architecture:** A single bash script (`scripts/assemble-dist.sh`) is the one source of truth that builds the macroquad wasm and assembles the four-file `dist/`. Both the CI workflow and a local test helper call it, so the deployed artifact and the locally-served artifact are byte-identical. CI uploads `dist/` straight to Cloudflare via `wrangler-action` — no `gh-pages` branch, no committed build artifacts.

**Tech Stack:** Rust + `wasm32-unknown-unknown` (macroquad/miniquad, **no** wasm-bindgen), bash, Python 3 (local static server), GitHub Actions, `cloudflare/wrangler-action@v3`, Cloudflare Pages.

**Spec:** [docs/superpowers/specs/2026-07-24-cloudflare-pages-gha-deploy-design.md](../specs/2026-07-24-cloudflare-pages-gha-deploy-design.md)

---

## File Structure

- **Create** `scripts/assemble-dist.sh` — build wasm + assemble `dist/` (shared by CI and local helper).
- **Create** `scripts/coop-server.py` — local static server sending COOP/COEP headers + `application/wasm`.
- **Create** `scripts/serve-web.sh` — local smoke-test: assemble then serve.
- **Create** `.github/workflows/deploy.yml` — CI build + Cloudflare Pages deploy.
- **Delete** `scripts/publish-web.ps1` — obsolete.
- **Rewrite** `docs/web-deploy.md` — GHA flow, one-time setup, local testing.
- **Rename** local branch `master` → `main` (one-time, manual task).

**Note on exec bits:** scripts are invoked as `bash scripts/<name>.sh` (not `./…`) in CI and the helper, so a missing git execute bit on Windows never matters.

---

## Task 0: Commit prerequisite wasm fixes

These edits were already made this session and are required for the browser build to run at all (without them the wasm panics every frame). Commit them before building deploy infra on top.

**Files:**
- Modify (already edited): `src/main.rs` (`std::time::Instant::now()` → `get_time()`)
- Modify (already edited): `Cargo.toml` (`[profile.release] opt-level = "s"`)

- [ ] **Step 1: Confirm the edits are present**

Run: `git diff --stat src/main.rs Cargo.toml`
Expected: both files show as modified.

- [ ] **Step 2: Verify the wasm build succeeds**

Run: `cargo build --release --target wasm32-unknown-unknown`
Expected: `Finished ... release` with exit 0.

- [ ] **Step 3: Commit**

```bash
git add src/main.rs Cargo.toml
git commit -m "fix: use macroquad get_time() instead of Instant on wasm; size-opt release profile"
```

---

## Task 1: Shared assemble script

**Files:**
- Create: `scripts/assemble-dist.sh`

- [ ] **Step 1: Write the script**

```bash
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
```

- [ ] **Step 2: Run it**

Run: `bash scripts/assemble-dist.sh`
Expected: build finishes, then prints `dist/ ready:` and lists `_headers`, `body3-sim.wasm`, `index.html`, `mq_js_bundle.js`.

- [ ] **Step 3: Verify all four files landed**

Run: `ls dist/_headers dist/body3-sim.wasm dist/index.html dist/mq_js_bundle.js`
Expected: all four paths exist (no "No such file").

- [ ] **Step 4: Commit**

```bash
git add scripts/assemble-dist.sh
git commit -m "build: add shared assemble-dist.sh for web artifact"
```

---

## Task 2: Local test helper

**Files:**
- Create: `scripts/coop-server.py`
- Create: `scripts/serve-web.sh`

- [ ] **Step 1: Write the COOP/COEP server**

```python
#!/usr/bin/env python3
"""Static server for local wasm testing. Sends the COOP/COEP headers that
SharedArrayBuffer (and thus macroquad's threaded wasm) requires — a plain
`python -m http.server` does not, so the app cannot run under it. Mirrors the
production `_headers` file and serves .wasm as application/wasm.

Usage: python scripts/coop-server.py [dir] [port]   (defaults: dist 8080)
"""
import http.server, socketserver, sys, os

directory = sys.argv[1] if len(sys.argv) > 1 else "dist"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080
os.chdir(directory)

class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      ".wasm": "application/wasm", ".js": "text/javascript"}
    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        super().end_headers()

with socketserver.TCPServer(("127.0.0.1", port), Handler) as httpd:
    print(f"serving {directory} on http://127.0.0.1:{port}  (ctrl-c to stop)", flush=True)
    httpd.serve_forever()
```

- [ ] **Step 2: Write the helper wrapper**

```bash
#!/usr/bin/env bash
# Local web smoke-test: build + assemble dist/, then serve it with the COOP/COEP
# headers SharedArrayBuffer requires. Open the printed URL in a browser.
# Usage: bash scripts/serve-web.sh [port]   (default 8080)
set -euo pipefail
cd "$(dirname "$0")/.."
bash scripts/assemble-dist.sh
PY="$(command -v python3 || command -v python)"
exec "$PY" scripts/coop-server.py dist "${1:-8080}"
```

- [ ] **Step 3: Start the helper in the background and probe headers**

Run:
```bash
bash scripts/serve-web.sh 8088 & SRV=$!; sleep 6
curl -s -I http://127.0.0.1:8088/ | grep -i "cross-origin"
curl -s -o /dev/null -w "wasm: %{http_code} %{content_type}\n" http://127.0.0.1:8088/body3-sim.wasm
kill $SRV
```
Expected: both `Cross-Origin-Opener-Policy: same-origin` and `Cross-Origin-Embedder-Policy: require-corp` lines print, and `wasm: 200 application/wasm`.

- [ ] **Step 4: Commit**

```bash
git add scripts/coop-server.py scripts/serve-web.sh
git commit -m "build: add local COOP/COEP web test helper"
```

---

## Task 3: GitHub Actions deploy workflow

**Files:**
- Create: `.github/workflows/deploy.yml`

- [ ] **Step 1: Write the workflow**

```yaml
name: deploy

on:
  push:
    branches: [main]
  pull_request:

jobs:
  deploy:
    runs-on: ubuntu-latest
    permissions:
      contents: read
      deployments: write
    steps:
      - uses: actions/checkout@v4

      - name: Install Rust + wasm target
        uses: dtolnay/rust-toolchain@stable
        with:
          targets: wasm32-unknown-unknown

      - uses: Swatinem/rust-cache@v2

      - name: Build + assemble dist/
        run: bash scripts/assemble-dist.sh

      - name: Deploy to Cloudflare Pages
        uses: cloudflare/wrangler-action@v3
        with:
          apiToken: ${{ secrets.CLOUDFLARE_API_TOKEN }}
          accountId: ${{ secrets.CLOUDFLARE_ACCOUNT_ID }}
          command: >-
            pages deploy dist
            --project-name=body3-sim
            --branch=${{ github.head_ref || github.ref_name }}
```

- [ ] **Step 2: Validate the YAML parses**

Run: `python -c "import yaml,sys; yaml.safe_load(open('.github/workflows/deploy.yml')); print('yaml ok')"`
Expected: `yaml ok` (no traceback).

- [ ] **Step 3: (If `actionlint` is installed) lint the workflow**

Run: `command -v actionlint >/dev/null && actionlint .github/workflows/deploy.yml || echo "actionlint not installed — skip"`
Expected: no errors, or the skip message.

- [ ] **Step 4: Commit**

```bash
git add .github/workflows/deploy.yml
git commit -m "ci: deploy to Cloudflare Pages via GitHub Actions"
```

---

## Task 4: Remove PowerShell script + rewrite docs

**Files:**
- Delete: `scripts/publish-web.ps1`
- Rewrite: `docs/web-deploy.md`

- [ ] **Step 1: Delete the obsolete script**

```bash
git rm scripts/publish-web.ps1
```

- [ ] **Step 2: Rewrite `docs/web-deploy.md`**

Replace the whole file with:

````markdown
# Web Deploy

body3-sim runs in the browser via `wasm32-unknown-unknown`. macroquad ships its
own JS bootstrap (`mq_js_bundle.js` + `load(".wasm")`), **not** wasm-bindgen, so
the wasm is built directly with `cargo build` (no trunk/wasm-bindgen step) and
served alongside the hand-written [`index.html`](../index.html).

Deploys run in CI: pushing to `main` builds the wasm and publishes to Cloudflare
Pages; every pull request gets its own preview URL.

## One-time setup

1. **Create the GitHub repo** and push `main`.

2. **Create the Cloudflare Pages project** (Direct Upload type):
   ```sh
   npx wrangler pages project create body3-sim --production-branch=main
   ```
   (or via the Cloudflare dashboard — Pages → Create → Direct Upload).

3. **Create a Cloudflare API token** — dashboard → My Profile → API Tokens →
   template *Cloudflare Pages: Edit*. Note your Account ID (dashboard sidebar).

4. **Add GitHub repo secrets** (Settings → Secrets and variables → Actions):
   - `CLOUDFLARE_API_TOKEN`
   - `CLOUDFLARE_ACCOUNT_ID`

## Per-deploy workflow

Push to `main`. The [`deploy` workflow](../.github/workflows/deploy.yml) builds
the wasm and deploys production automatically. Open a pull request to get a
preview URL for the branch (posted in the Actions run).

What CI does:
1. installs the Rust `wasm32-unknown-unknown` target (cached)
2. `bash scripts/assemble-dist.sh` → builds wasm, assembles `dist/`
3. `wrangler pages deploy dist` → uploads to Cloudflare Pages
4. Cloudflare serves the deploy in ~1 min.

## Local testing

Build and serve the exact production artifact locally, with the COOP/COEP
headers SharedArrayBuffer needs:

```sh
bash scripts/serve-web.sh          # http://127.0.0.1:8080
bash scripts/serve-web.sh 9000     # custom port
```

A plain `python -m http.server` will **not** work — it omits the COOP/COEP
headers and the app fails to start.

## Smoke test (manual, every deploy)

1. Open the deployed URL in a fresh tab.
2. Pick a non-default scenario in the sidebar (e.g. Slingshot).
3. Click **Apply** — the address bar should update to `?scenario=slingshot`.
4. Copy the URL, open a new tab, paste — the sim should load in the Slingshot
   scenario without any sidebar interaction.
5. Change a slider, click **Copy link** — the URL should encode the non-default
   value (`?scenario=centralswarm&swarm_size=2000`).

## Troubleshooting

- **`SharedArrayBuffer is not defined`** — check the deployed headers
  (`curl -I https://your-site/`). Both `Cross-Origin-Opener-Policy: same-origin`
  and `Cross-Origin-Embedder-Policy: require-corp` must be present. Cloudflare
  Pages reads the `_headers` file committed at repo root; `scripts/assemble-dist.sh`
  copies it into `dist/`.
- **`cargo test` fails because a test imports body3_sim::url** — make sure
  `src/lib.rs` has `pub mod url;` and `src/url.rs` exists. The tests are pure
  Rust, no wasm toolchain needed.
- **wasm binary too large (display warning)** — `Cargo.toml`'s
  `[profile.release] opt-level = "s"` targets size; `brotli` on Pages further
  compresses transit. Anything under 4 MB is fine.
- **Black screen / `unreachable` in console** — a Rust panic on wasm. Common
  cause: calling an API unsupported on `wasm32-unknown-unknown` (e.g.
  `std::time::Instant::now()`). Use macroquad's `get_time()` instead.
````

- [ ] **Step 3: Verify no stale references remain**

Run: `grep -rn -i "trunk\|publish-web\|gh-pages" docs/web-deploy.md scripts/ .github/ ; echo "exit=$?"`
Expected: no matches (`grep` exits 1, prints `exit=1`).

- [ ] **Step 4: Commit**

```bash
git add -A docs/web-deploy.md scripts/publish-web.ps1
git commit -m "docs: rewrite web-deploy for GHA/Cloudflare; drop publish-web.ps1"
```

---

## Task 5: Branch rename + remote (manual, human-driven)

These steps require your GitHub and Cloudflare accounts — run them yourself; they are not automatable from this session.

- [ ] **Step 1: Rename the local branch**

Precondition (verify safe): `git worktree list` shows only the main worktree, `git remote -v` is empty.
```bash
git branch -m master main
```

- [ ] **Step 2: Create the GitHub repo and push**

```bash
git remote add origin git@github.com:<you>/body3-sim.git
git push -u origin main
```

- [ ] **Step 3: Complete Cloudflare + secrets setup**

Follow `docs/web-deploy.md` → One-time setup (steps 2–4): create the Pages
project, the API token, and add the two GitHub secrets.

- [ ] **Step 4: Verify the pipeline end-to-end**

1. Open a throwaway PR (edit a comment) → confirm the Actions run succeeds and a
   preview URL is posted; open it and run the smoke test.
2. Merge to `main` → confirm the production deploy runs and the `*.pages.dev`
   URL updates.

---

## Notes / risks

- **Fork PRs get no secrets:** GitHub withholds `secrets.*` from pull requests
  opened off forks, so a fork PR's deploy step fails (no CF token). For a solo
  repo (PRs from same-repo branches) this never triggers. If external
  contributors appear later, gate the deploy step behind
  `if: github.event.pull_request.head.repo.full_name == github.repository`.
- **Project must exist before first deploy:** `wrangler pages deploy` against a
  non-existent project fails non-interactively in CI. Task 5 step 3 creates it
  first.
- **`get_time()` fix (Task 0) is load-bearing:** without it the deployed wasm
  panics every frame. It is committed first for that reason.
