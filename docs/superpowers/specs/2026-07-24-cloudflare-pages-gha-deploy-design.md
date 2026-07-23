# body3-sim — Cloudflare Pages deploy via GitHub Actions

**Date:** 2026-07-24
**Status:** Design approved (pending spec review)

## Problem

body3-sim is a macroquad wasm app (miniquad's own JS loader, **not** wasm-bindgen).
The deployable artifact is a `dist/` folder of four files:
`index.html`, `mq_js_bundle.js`, `_headers`, `body3-sim.wasm`.

The current deploy path — `scripts/publish-web.ps1` — builds the wasm locally,
assembles `dist/`, then commits the built artifact to a `gh-pages` orphan branch
that Cloudflare Pages serves. Problems:

- PowerShell-only; not cross-platform.
- Commits build output to a branch (artifact churn in git history).
- Uses `git worktree` juggling against a `gh-pages` branch.
- References `git push origin gh-pages` but the repo has **no git remote** — so
  the script cannot actually run as written.

## Goal

Push-to-deploy to Cloudflare Pages via GitHub Actions, covering **production**
(push to `main`) and **PR previews** (per-branch preview URLs). Remove the
PowerShell script and the `gh-pages` branch model. Keep a cross-platform local
test helper. The `dist/` artifact contract stays identical — only the delivery
mechanism changes.

## Architecture

Pipeline:

```
GitHub push/PR → GHA workflow → cargo build wasm → assemble dist/ → wrangler pages deploy → Cloudflare Pages
```

### Shared assemble step — `scripts/assemble-dist.sh`

A single bash script is the one source of truth for producing `dist/`, used by
**both** the CI workflow and the local helper (no drift between them). It:

1. `cargo build --release --target wasm32-unknown-unknown`
2. Asserts `target/wasm32-unknown-unknown/release/body3-sim.wasm` exists (exit
   non-zero with a clear message if not).
3. Cleans and recreates `dist/`, copies the four files into it:
   `index.html`, `mq_js_bundle.js`, `_headers`, `body3-sim.wasm`.

Bash runs in CI (ubuntu) and on Windows via Git Bash, matching the existing
project convention (the Bash tool is already used here).

### CI workflow — `.github/workflows/deploy.yml`

- **Triggers:** `push` to `main` (production) and `pull_request` (preview).
- **Runner:** `ubuntu-latest`. The wasm32 build needs **no system libraries** —
  macroquad's native backend deps (X11/ALSA) are native-only and never compiled
  for the wasm target. No `apt install` step.
- **Steps:**
  1. `actions/checkout`
  2. Install stable Rust toolchain + `wasm32-unknown-unknown` target.
  3. `Swatinem/rust-cache` — avoids recompiling the ~51-crate dependency tree on
     every push (cold ~1–2 min → warm seconds).
  4. `scripts/assemble-dist.sh` → produces `dist/`.
  5. `cloudflare/wrangler-action` running
     `pages deploy dist --project-name=body3-sim --branch=<branch>`,
     where `<branch>` = `${{ github.head_ref || github.ref_name }}` (the source
     branch name in both push and PR contexts).
- **Branch → environment mapping:** Cloudflare Pages treats a deploy whose branch
  equals the project's **production branch** (`main`) as production; any other
  branch name yields a **preview** deployment with its own unique URL. Passing
  the source branch name gives production on `main` and a preview per PR branch
  automatically.
- **Secrets** (GitHub repo settings): `CLOUDFLARE_API_TOKEN` (scope: Cloudflare
  Pages → Edit) and `CLOUDFLARE_ACCOUNT_ID`, passed to `wrangler-action`.

### Local test helper — `scripts/serve-web.sh` (kept)

Cross-platform local smoke-test path:

1. Runs `scripts/assemble-dist.sh` (same `dist/` as production).
2. Launches a small Python COOP/COEP static server (`scripts/coop-server.py`)
   bound to `dist/`.

Rationale: SharedArrayBuffer requires `Cross-Origin-Opener-Policy: same-origin`
+ `Cross-Origin-Embedder-Policy: require-corp`. A plain `python -m http.server`
does not send these, so it can't run the app locally. The helper reproduces the
exact production headers (mirroring `_headers`) and serves `body3-sim.wasm` with
`Content-Type: application/wasm`. Python is already available on the dev machine
(3.12) and is cross-platform.

### One-time setup (manual, documented — not code)

1. Create the GitHub repo and push.
2. Cloudflare: create the `body3-sim` Pages project (Direct Upload type),
   production branch = `main`. Via dashboard or
   `wrangler pages project create body3-sim --production-branch=main`.
3. Create a Cloudflare API token (My Profile → API Tokens → template *Cloudflare
   Pages: Edit*); note the Account ID.
4. Add GitHub repo secrets `CLOUDFLARE_API_TOKEN` and `CLOUDFLARE_ACCOUNT_ID`.

After that, the workflow owns every deploy.

### Branch rename

`git branch -m master main` before the first GitHub push, so the local branch
matches GitHub's default and the workflow's `main` trigger. Safe: the repo is
local-only (no remote, no other worktrees), so the rename has no upstream or
worktree side effects.

## Removed

- `scripts/publish-web.ps1` — deleted; its build/assemble/push job is now split
  between `assemble-dist.sh` and the workflow.
- The `gh-pages` orphan-branch model — abandoned entirely. Wrangler uploads
  `dist/` directly to Cloudflare; no orphan branch, no committed artifacts, no
  `git worktree`. Nothing to delete (no such branch or remote exists yet) — the
  design simply stops referencing it.

## Kept unchanged

`index.html`, `mq_js_bundle.js`, `_headers`, and the `Cargo.toml`
`[profile.release] opt-level = "s"` — the artifact contract is untouched.

## Docs

Rewrite `docs/web-deploy.md` for the GHA flow:

- Intro: macroquad (no wasm-bindgen), the four-file `dist/` artifact.
- One-time setup: GitHub secrets + Cloudflare Pages project.
- Per-deploy: "push to `main`, done"; PR-preview note.
- Local testing: `scripts/serve-web.sh`.
- Keep the existing smoke-test and troubleshooting sections (COOP/COEP headers,
  wasm size) — still applicable.

## Error handling

- Build failure → job fails, no deploy.
- Missing wasm artifact → `assemble-dist.sh` asserts and exits non-zero.
- Missing/invalid secrets → `wrangler-action` fails with a clear error.
- `_headers` absent from `dist/` would break SharedArrayBuffer → `assemble-dist.sh`
  always copies it; the smoke test catches any regression.

## Testing / verification

- **CI:** open a PR → confirm the preview URL deploys and loads (canvas renders,
  no console errors, `?scenario=slingshot` selects the preset). Merge to `main` →
  confirm the production URL updates.
- **Local:** `scripts/serve-web.sh`, then the browser smoke test in
  `docs/web-deploy.md`.

## Out of scope (YAGNI)

- Custom domain (Cloudflare provides `*.pages.dev`).
- CDN/cache tuning beyond Cloudflare defaults.
- Staging / multi-environment.
- Rollback automation (Cloudflare keeps deploy history; roll back in dashboard).
