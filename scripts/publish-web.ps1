# Build the wasm bundle locally and push dist/ to the gh-pages orphan branch.
# Requires: rustup target add wasm32-unknown-unknown; cargo install trunk.
# Cloudflare Pages deploys from gh-pages automatically on push.

$ErrorActionPreference = "Stop"

Write-Host "trunk build --release" -ForegroundColor Cyan
trunk build --release
if ($LASTEXITCODE -ne 0) { throw "trunk build failed" }

if (-not (Test-Path "dist\index.html")) { throw "dist/index.html missing — trunk did not emit the bundle" }

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
Write-Host "deployed $sha to gh-pages — Cloudflare Pages will rebuild in 1-2 min" -ForegroundColor Green