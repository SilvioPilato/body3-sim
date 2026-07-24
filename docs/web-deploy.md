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
1. installs the Rust **nightly** toolchain with `wasm32-unknown-unknown` and
   `rust-src` (the force-solver Web Worker pool needs a `-Z build-std` build with
   atomics/shared-memory — see `scripts/assemble-dist.sh` and `src/wasm_pool.rs`)
2. `bash scripts/assemble-dist.sh` → builds the threaded wasm, assembles `dist/`
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
