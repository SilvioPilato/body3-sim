# Build the wasm bundle locally and push dist/ to the gh-pages orphan branch.
# Requires: rustup target add wasm32-unknown-unknown.
# Cloudflare Pages deploys from gh-pages automatically on push.
#
# Note: macroquad uses its own JS bootstrap (mq_js_bundle.js + load(.wasm)),
# not wasm-bindgen — so we bypass trunk's wasm-bindgen linker and build the
# wasm directly with `cargo build --target wasm32-unknown-unknown`.

$ErrorActionPreference = "Stop"

Write-Host "cargo build --target wasm32-unknown-unknown --release" -ForegroundColor Cyan
cargo build --target wasm32-unknown-unknown --release
if ($LASTEXITCODE -ne 0) { throw "cargo build failed" }

$wasm = "target\wasm32-unknown-unknown\release\body3-sim.wasm"
if (-not (Test-Path $wasm)) { throw "wasm artifact missing: $wasm" }

Write-Host "assembling dist/" -ForegroundColor Cyan
if (Test-Path "dist") { Remove-Item "dist" -Recurse -Force }
New-Item -ItemType Directory -Path "dist" -Force | Out-Null
Copy-Item "index.html" "dist\"
Copy-Item "mq_js_bundle.js" "dist\"
Copy-Item "_headers" "dist\"
Copy-Item $wasm "dist\body3-sim.wasm"

$worktree = "_gh-pages"
if (Test-Path $worktree) {
    Write-Host "removing existing worktree" -ForegroundColor DarkGray
    git worktree remove $worktree --force
}

Write-Host "adding gh-pages worktree" -ForegroundColor Cyan
git worktree add $worktree gh-pages

Write-Host "copying dist -> gh-pages" -ForegroundColor Cyan
Remove-Item "$worktree\*" -Recurse -Force -ErrorAction SilentlyContinue
Copy-Item "dist\*" $worktree -Recurse -Force

$sha = git rev-parse --short HEAD
Push-Location $worktree
git add -A
git commit -m "deploy: $sha" --allow-empty
git push origin gh-pages
Pop-Location

git worktree remove $worktree --force
Write-Host "deployed $sha to gh-pages - Cloudflare Pages will rebuild in 1-2 min" -ForegroundColor Green