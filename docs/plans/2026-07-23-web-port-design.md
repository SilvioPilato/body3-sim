# Web Port Design

**Goal:** Ship body3-sim as an interactive playground in the browser — same UX as
the desktop build (sidebar scenario picker, sliders, Apply button), with
shareable URLs that encode scenario + parameters.

**Hosting:** Cloudflare Pages, static site, free tier. `_headers` file enables
COOP/COEP so `SharedArrayBuffer` is available — leaves `wasm-bindgen-rayon` as a
zero-cost future option, not a requirement today.

**Build:** `trunk` builds locally, artifacts land in `dist/`, a script
(`scripts/publish-web.ps1`) pushes `dist/` to a `gh-pages` orphan branch, which
Cloudflare Pages deploys. `master` stays pure source — no wasm binary in its
history.

**Tech:** macroquad 0.4 + egui-macroquad are wasm-capable today; the library
already uses only `Vec2` math with `#[cfg(target_arch = "wasm32")]`-gated
threading (`src/energy.rs:38-87`). No new runtime dependencies for v1.

---

## §1 — Build & Deploy Pipeline

`trunk` wraps the cargo wasm build and emits the bundle (HTML, JS glue, wasm)
into `dist/`. It's the standard for macroquad + egui web exports and handles
miniquad's wasm bootstrap without hand-rolling JS.

**Manual build flow:**
```
trunk build --release
git checkout gh-pages
git rm -rf .
cp -r ../dist/* .
git add -A && git commit -m "deploy: <sha>"
git push origin gh-pages
```
Wrapped as `scripts/publish-web.ps1` — one command.

**Cloudflare Pages config** (committed to `master` for reproducibility):
- Framework: None
- Build command: *(empty — artifact already in `gh-pages`)*
- Output directory: `/` (root of `gh-pages`)
- Branch: `gh-pages`
- Headers: `_headers` file → `COOP: same-origin` / `COEP: require-corp` (so
  SharedArrayBuffer / `wasm-bindgen-rayon` are available when added — zero-cost
  option today)

`gh-pages` is force-pushed on each build (no history growth). `master` stays
pure source.

---

## §2 — Library & Build Target Changes

`Cargo.toml` gains a WASM target with feature-gated deps so `master` still
builds native-only by default:

```toml
[features]
default = []
# Enables wasm-bindgen-rayon-style parallelism in physics::walk_forces when
# COOP/COEP makes SharedArrayBuffer available. No-op on native builds.
web-parallel = ["wasm-bindgen-rayon"]

[dependencies]
macroquad = "0.4.15"
egui-macroquad = "0.17.3"
# wasm-only crates pulled in conditionally so `cargo test` (native) stays clean.
[target.'cfg(target_arch = "wasm32")'.dependencies]
wasm-bindgen-rayon = { version = "0.1", optional = true }

[profile.release.package."*"]
opt-level = "s"
```

`src/lib.rs` unchanged — already platform-agnostic (`Vec2` only).

`src/physics.rs` — `walk_forces` stays single-threaded today. When
`web-parallel` is on, the same signature swaps to `par_iter_mut` behind
`#[cfg(feature = "web-parallel")]`. No change in this round; the feature is a
flag for **later**. We keep the option open without wiring it now — the headers
already enable SAB.

`src/energy.rs` — the existing wasm stub (`energy.rs:38-87`) stays. Notes updated
to mention COOP/COEP now enabled so a future `TotalEnergyWorker` web backend can
use rayon.

`Trunk.toml` (new, at repo root): points at `src/main.rs`, sets build target
directory to `dist`, tweaks `wasm-opt` flags. One line of config.

One-time tooling install (documented in `docs/web-deploy.md`):
```
rustup target add wasm32-unknown-unknown
cargo install trunk
```

---

## §3 — Code Changes for Web Compatibility

The codebase is mostly ready. Three concrete changes:

**1. `src/main.rs` — `std::process::exit` (line 400) is not available on wasm.
Wrap it:**
```rust
#[cfg(not(target_arch = "wasm32"))]
if bench_samples.len() >= BENCH_FRAME_COUNT {
    report_benchmark(&mut bench_samples, bench_swarm_size);
    std::process::exit(0);
}
#[cfg(target_arch = "wasm32")]
{ /* noop — benchmark mode not meaningful in browser */ }
```

**2. `src/main.rs` — window_conf**: macroquad on wasm ignores
`window_width/height/resizable` (browser controls the canvas). Current
`window_conf()` (line 75) keeps working as a no-op — no change needed.

**3. `src/energy.rs` — existing stub is correct.** No code change. The "no live
energy display on web" behavior is already what ships.

**4. Index HTML** (`index.html` at repo root for trunk): canvas fills the
viewport, loads the wasm, no special JS. Trunk generates the final
`dist/index.html` from this:

```html
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>body3-sim</title>
  <style>html,body{margin:0;padding:0;background:#000;overflow:hidden}canvas{width:100vw;height:100vh;display:block}</style>
</head>
<body><link data-trunk rel="rust" data-bin="body3-sim"/></body>
</html>
```

**5. `_headers` file** (committed at repo root, trunk copies it to `dist/`):
```
/*
  Cross-Origin-Opener-Policy: same-origin
  Cross-Origin-Embedder-Policy: require-corp
```

What is **not** touched: `simulation.rs`, `quadtree.rs`, `camera.rs`, all tests.
They already use only `Vec2` math and `#[cfg(target_arch = "wasm32")]`-gated
threading. Zero changes needed there.

---

## §4 — Shareable URLs

URL encodes scenario + parameters so a link reproduces exact state. Simplest
viable design:

**Format** — query string, one key per field, only non-defaults included:
```
#/?scenario=centralswarm&swarm_size=2000&physics_dt=0.005
#/?scenario=slingshot
#/?scenario=randomnbody&count=12&mass_max=3000&seed=7
```

**Where**: `src/url.rs` (new, ~80 lines). Pure functions
`encode(config) -> String` and `decode(query) -> Option<SimulationConfig>`. No
macroquad/wasm deps — just string parsing. Testable from `cargo test` — no
browser needed.

**How**:
- `Simulation::new` startup: read `window.location.search` on wasm (via
  `web-sys` or a macroquad helper), fall back to `Default::default()` on native,
  gated `#[cfg(target_arch = "wasm32")]`.
- On `sim.reset(pending)` (the "Apply" button): update URL via
  `history.replaceState` so the address bar always reflects current state.
  Native: noop.
- UI: add a "Copy link" button next to "Apply" that copies the current URL to
  clipboard. Uses `navigator.clipboard.writeText` on wasm, prints to stdout on
  native.

**Alternative — base64-encoded blob in URL hash** (e.g.
`#/?config=eyJzY2VuYXJpbyI6...`):
- Pros: shorter URLs for dense param sets, one round-trip of `serde_json` +
  `base64`.
- Cons: opaque (user can't edit it), needs `serde` plus `base64` as new deps,
  no SEO value.

Recommend **plain query string** for v1 — readable, debuggable, no new deps.
Migration to base64 later is a one-function swap.

---

## §5 — Testing Strategy

One principle: tests must run on `cargo test` (native) without ever invoking a
browser or wasm. Everything that matters is testable headless.

**`tests/url_encode.rs`** (new ~120 lines) — covers the pure encoder/decoder:
- Every scenario, every parameter, default vs. overridden values
- Round-trip: encode → decode → assert equality
- Unknown scenario name → returns None
- Malformed numbers (e.g. `physics_dt=abc`) → returns None
- Empty query → returns default config
- Trailing junk / extra keys ignored gracefully

Existing suites unaffected: the 51 tests we have today keep running on native.
`cargo test` stays the gate.

Wasm-only smoke check: one manual `scripts/publish-web.ps1` step after every
deploy — open the URL, click "Apply" with a non-default scenario, verify the
page URL got the query appended. Documented in `docs/web-deploy.md` as a
checklist item, no automation.

What we skip (YAGNI):
- `wasm-bindgen-test` / `cargo test --target wasm32-unknown-unknown` — wiring
  `wasm-pack test` is real tooling cost (geckodriver, chromedriver, Node
  toolchain) for a playground. Skip.
- Browserstack / Playwright — same. Revisit if this becomes a real web app.

CI: GitHub Actions on `master` push runs `cargo test` (native) only. No web
build pipeline in CI — `scripts/publish-web.ps1` is the deploy trigger. Adds
zero CI minutes.

---

## §6 — File Inventory

| File | Action | Lines est. |
|---|---|---|
| `Cargo.toml` | Modify — `web-parallel` feature, wasm target deps, release profile | +12 |
| `Trunk.toml` | Create — trunk build config | ~15 |
| `index.html` | Create — trunk entry point | ~12 |
| `_headers` | Create — COOP/COEP | ~3 |
| `src/url.rs` | Create — URL encode/decode | ~80 |
| `src/lib.rs` | Modify — `pub mod url;` | +1 |
| `src/main.rs` | Modify — wasm-gated `process::exit`, URL read on startup, URL update on reset, "Copy link" button | ~30 |
| `src/energy.rs` | Modify — comment refresh (COOP/COEP note) | ~5 |
| `tests/url_encode.rs` | Create — URL round-trip suite | ~120 |
| `scripts/publish-web.ps1` | Create — local build + `gh-pages` push | ~25 |
| `docs/web-deploy.md` | Create — one-time setup + deploy checklist + smoke test | ~60 |
| `.gitignore` | Modify — ignore `dist/` build output | +1 |

**Total**: ~360 new lines, 7 files modified. No changes to physics,
simulation, quadtree, camera, or existing tests.

---

## Out of Scope, Deliberately

- **Parallel physics on web.** `walk_forces` and `total_energy` stay
  single-threaded. Headers enable SAB so `wasm-bindgen-rayon` is a future
  feature flag, not a current rewrite.
- **Touch / mobile UI.** egui's slider work on touch but the layout is sized for
  desktop. Responsive layout is v2.
- **Server-side anything.** Static site only.
- **Analytics, presets gallery, embedding.** Playground first; extras later.