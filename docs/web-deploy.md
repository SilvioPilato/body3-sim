# Web Deploy

body3-sim runs in the browser via `wasm32-unknown-unknown`. macroquad ships its
own JS bootstrap (`mq_js_bundle.js` + `load(".wasm")`), **not** wasm-bindgen, so
there is no `trunk` / wasm-bindgen step — the wasm is built directly with
`cargo build` and served alongside the hand-written [`index.html`](../index.html).
Builds are produced locally and committed to an orphan `gh-pages` branch that
Cloudflare Pages serves.

## One-time setup

1. **Install the wasm target:**
   ```sh
   rustup target add wasm32-unknown-unknown
   ```

2. **Create the `gh-pages` orphan branch** (skip if it already exists):
   ```sh
   git checkout --orphan gh-pages
   git rm -rf .
   echo "placeholder" > index.html
   git add index.html
   git commit -m "init gh-pages"
   git push -u origin gh-pages
   git checkout master
   ```

3. **Connect Cloudflare Pages** to the repo:
   - Project name: `body3-sim`
   - Production branch: `gh-pages`
   - Build command: *(empty — the artifact is already in the branch)*
   - Output directory: `/`
   - Environment variables: none
   - Click "Save and Deploy" once the first `gh-pages` push is in.

## Per-deploy workflow

One command from `master` (or any branch that has the source):

```sh
./scripts/publish-web.ps1
```

What it does:

1. `cargo build --target wasm32-unknown-unknown --release` → emits the wasm
2. assembles `dist/` (`index.html` + `mq_js_bundle.js` + `_headers` + `.wasm`)
3. `git worktree add _gh-pages gh-pages` (or reuses an existing one)
4. copies `dist/` contents into the worktree
5. `git add -A && git commit -m "deploy: <sha>"`
6. `git push origin gh-pages`
7. `git worktree remove _gh-pages`
8. Cloudflare Pages detects the push and rebuilds in 1-2 min.

## Smoke test (manual, every deploy)

1. Open the deployed URL in a fresh tab.
2. Pick a non-default scenario in the sidebar (e.g. Slingshot).
3. Click **Apply** — the browser address bar should update to
   `?scenario=slingshot`.
4. Copy the URL, open a new tab, paste — the sim should load in the Slingshot
   scenario without any sidebar interaction.
5. Change a slider, click **Copy link** — the URL should encode the
   non-default value (`?scenario=centralswarm&swarm_size=2000`).

## Troubleshooting

- **`SharedArrayBuffer is not defined`** — check the deployed headers
  (`curl -I https://your-site/`). Both `Cross-Origin-Opener-Policy: same-origin`
  and `Cross-Origin-Embedder-Policy: require-corp` must be present. Cloudflare
  Pages reads the `_headers` file committed at repo root.
- **`cargo test` fails because a test imports body3_sim::url** — make sure
  `src/lib.rs` has `pub mod url;` and `src/url.rs` exists. The tests are pure
  Rust, no wasm toolchain needed.
- **wasm binary too large (display warning)** — `Cargo.toml`'s
  `[profile.release] opt-level = "s"` targets size; `brotli` on Pages further
  compresses transit. Anything under 4 MB is fine.