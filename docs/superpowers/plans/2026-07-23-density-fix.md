# Constant-Density Spawn (central_swarm) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. **Ask the user before every git commit.**

**Goal:** Make `central_swarm` spawn density constant across `swarm_size` by scaling the annulus radii with `sqrt(swarm_size)`, restoring Barnes-Hut's O(n log n) force-walk behavior at large n, with a zoom-to-fit camera so the whole swarm stays visible.

**Architecture:** `central_swarm`'s annulus bounds (60..280 at the reference n=1000) scale by `sqrt(n/1000)`, so annulus area grows proportionally to n and density stays ~4.2e-3 bodies/unit² at every size. `Simulation::world_half_size` (quadtree root extent) grows to contain the scaled annulus — required because `Quadtree::insert` has no bounds check and silently misfiles out-of-root bodies. Rendering in `main.rs` gains a `Camera2D` zoom-to-fit into an 800x800 viewport, with the draw-circle radius compensated so bodies stay ~5px on screen. At the default n=1000 the scale factor is exactly 1.0: spawn positions, world extent, and rendering are pixel-identical to today.

**Tech Stack:** Rust (edition 2024), macroquad 0.4.15, egui-macroquad 0.17.3, criterion 0.5.

**Read first (current state, do not trust summaries over code):**
- `src/simulation.rs` — `central_swarm` (~line 86), `Simulation::new` (~line 244)
- `src/physics.rs` — `walk_forces`, `total_energy`, `total_energy_approx`
- `src/quadtree.rs` — `insert` has **no bounds check** (line 80)
- `src/main.rs` — draw loop (~line 267), `--benchmark` mode (~line 232), `ENERGY_LOG_INTERVAL_FRAMES` comment (lines 6-15)
- `benches/physics_benchmarks.rs` — all groups hardcode `half_size = SCREEN_SIZE / 2.0`
- `examples/profile_workload.rs`, `tests/energy_approx_accuracy.rs`

**Evidence this plan is based on (session findings, re-verify if state moved):**
- `walk_forces` grew ~169x from n=1000→64000 vs ~103x predicted by O(n log n); `quadtree_build` matched prediction (~105x). Anomaly isolated to the force walk, consistent with density-driven degradation of the opening-angle criterion.
- `total_energy_approx` error vs exact: 0.5% @ n=500 → 200% @ n=44000 (density-driven).
- Unexplained: at n=44000, exact `total_energy` log diverged from ~-3e11 to ~+3.9e16 within a few frames (plausibly close encounters at extreme density). Verification-only here, not a separate fix.

**Design decisions (made with user, do not revisit silently):**
1. **Radius scaling:** both `min_radius` and `max_radius` scale by `sqrt(n/1000)`. Scaling only max_radius would over-densify the inner annulus (close encounters near the central body). Reference n=1000 = current default → zero change at default.
2. **UX: camera zoom-to-fit** (chosen over clamp-to-screen and fixed-camera crop). Clamp was rejected: it re-breaks density beyond the clamp point — exactly where benchmarks run (44000/64000). Crop was rejected: most bodies invisible at high n is misleading.
3. **Quadtree root extent** must scale with the swarm (`world_half_size`), with a 10% margin. Not optional: `insert` does no bounds checking.
4. **Scope:** `RandomSwarm` and `RandomNBody` are out of scope for density scaling (their radius/spread is user-controlled via UI sliders). Exception: the new `world_extent` helper handles `RandomSwarm`'s `radius_range.1` too, because `Simulation::new` uses it for all scenarios and today a `radius max` up to 600 silently overflows the 400 half-size root. `RandomNBody`'s `position_spread` keeps current behavior (pre-existing, out of scope).
5. **Not in this plan:** reviving `total_energy_approx`/`total_energy_sampled` as the energy display; fixing the 44000 energy divergence (verification only). Both are follow-ups contingent on post-fix measurements.

---

### Task 1: Constant-density spawn + world extent (TDD)

**Files:**
- Modify: `src/simulation.rs` (`central_swarm` ~line 86, `Simulation::new` ~line 244)
- Test: `tests/spawn_density.rs` (create)

- [ ] **Step 1: Write the failing test**

Create `tests/spawn_density.rs`:

```rust
use body3_sim::simulation::{central_swarm_radii, Scenario, Simulation, SimulationConfig};

fn make_sim(n: usize) -> Simulation {
    Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: n },
        screen_size: 800.0,
        physics_dt: 0.005,
        time_scale: 1.0,
    })
}

#[test]
fn spawn_radii_scale_with_sqrt_of_swarm_size() {
    for n in [1000usize, 8000, 64000] {
        let scale = (n as f32 / 1000.0).sqrt();
        let (expected_min, expected_max) = central_swarm_radii(n);
        assert!((expected_min - 60.0 * scale).abs() < 1e-4);
        assert!((expected_max - 280.0 * scale).abs() < 1e-4);

        let sim = make_sim(n);
        let center = 400.0_f32;
        let mut max_seen = 0.0_f32;
        // objects()[0] is the central body; swarm bodies start at index 1.
        for obj in &sim.objects()[1..] {
            let r = ((obj.position.x - center).powi(2) + (obj.position.y - center).powi(2)).sqrt();
            assert!(
                (expected_min - 1e-2..=expected_max + 1e-2).contains(&r),
                "n={n} body radius {r} outside [{expected_min}, {expected_max}]"
            );
            max_seen = max_seen.max(r);
        }
        // golden-angle fill actually reaches the outer edge (not clamped inside)
        assert!(max_seen > expected_max * 0.99, "n={n} max_seen={max_seen}");
    }
}

#[test]
fn default_spawn_is_pixel_compatible_with_before() {
    // n=1000 => scale == 1.0: radii [60, 280], world half-size 400 — the
    // exact values the code had before this change.
    let sim = make_sim(1000);
    assert_eq!(sim.world_half_size(), 400.0);
    let (min_r, max_r) = central_swarm_radii(1000);
    assert_eq!((min_r, max_r), (60.0, 280.0));
}

#[test]
fn world_extent_contains_scaled_swarm() {
    for n in [1000usize, 8000, 64000] {
        let sim = make_sim(n);
        let (_, max_r) = central_swarm_radii(n);
        assert!(
            sim.world_half_size() >= max_r,
            "n={n} half_size={} < max_radius={max_r}",
            sim.world_half_size()
        );
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test --test spawn_density`
Expected: FAIL to compile — `central_swarm_radii` and `world_half_size()` don't exist yet.

- [ ] **Step 3: Implement**

In `src/simulation.rs`, add near the top (after `MAX_FRAME_TIME`):

```rust
// Reference swarm_size at which the annulus is [MIN_RADIUS, MAX_RADIUS].
// Both bounds scale with sqrt(n / REF_N) so the annulus area grows
// proportionally to n and spawn density (bodies/area) stays constant.
// Constant density keeps the Barnes-Hut opening-angle geometry scale-invariant:
// without it, packing more bodies into a fixed area makes the force walk
// degrade past O(n log n) (measured: ~169x vs ~103x predicted, n=1000->64000).
const CENTRAL_SWARM_REF_N: f32 = 1000.0;
const CENTRAL_SWARM_MIN_RADIUS: f32 = 60.0;
const CENTRAL_SWARM_MAX_RADIUS: f32 = 280.0;
// Quadtree::insert has no bounds check — bodies outside the root quadrant are
// silently misfiled into corner quadrants, unbalancing the tree. The root
// half-size must contain the whole swarm, with margin for orbital drift.
const WORLD_EXTENT_MARGIN: f32 = 1.1;

// Annulus radius bounds for a CentralSwarm of `n` bodies.
pub fn central_swarm_radii(n: usize) -> (f32, f32) {
    let scale = (n as f32 / CENTRAL_SWARM_REF_N).sqrt();
    (CENTRAL_SWARM_MIN_RADIUS * scale, CENTRAL_SWARM_MAX_RADIUS * scale)
}
```

In `central_swarm`, replace:

```rust
    let min_radius = 60.0_f32;
    let max_radius = 280.0_f32;
```

with:

```rust
    let (min_radius, max_radius) = central_swarm_radii(n);
```

In `impl Simulation`, add the extent helper and getter, and use it in `new`:

```rust
    // Half-size of the square physics domain (quadtree root) for a scenario:
    // at least screen_size/2, grown to contain scenario extents that exceed it.
    // CentralSwarm radii scale with sqrt(n); RandomSwarm's radius max (UI slider,
    // up to 600) can also exceed the default 400. Single source of this rule so
    // benches/examples compute the same half-size production runs with.
    pub fn world_extent(scenario: &Scenario, screen_size: f32) -> f32 {
        let base = screen_size / 2.0;
        match scenario {
            Scenario::CentralSwarm { swarm_size } => {
                base.max(central_swarm_radii(*swarm_size).1 * WORLD_EXTENT_MARGIN)
            }
            Scenario::RandomSwarm(params) => base.max(params.radius_range.1 * WORLD_EXTENT_MARGIN),
            _ => base,
        }
    }

    pub fn world_half_size(&self) -> f32 {
        self.world_half_size
    }
```

In `Simulation::new`, replace:

```rust
        let world_half_size = config.screen_size / 2.0;
```

with:

```rust
        let world_half_size = Self::world_extent(&config.scenario, config.screen_size);
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test --test spawn_density`
Expected: PASS (3 tests).

- [ ] **Step 5: Run the full test suite (regression check)**

Run: `cargo test`
Expected: PASS, including `tests/energy_approx_accuracy.rs` and `tests/verlet_cache_regression.rs`. Note: at n=500 the scale is ~0.707 (radii shrink), but density is unchanged, so the approx-energy error pinned by that test should be essentially the same as before. If it fails, stop and investigate — do not loosen the test to make it pass without user sign-off.

- [ ] **Step 6: Manual smoke test**

Run: `cargo run --release` (default CentralSwarm, n=1000) for a few seconds.
Expected: swarm looks and orbits exactly as before (scale=1.0 → identical spawn), energy log prints, no panic. Then in the UI set swarm_size to 50000, click "Applica": bodies spawn in a much larger annulus and most are off-screen (camera comes in Task 2) — that is expected at this point, not a bug.

- [ ] **Step 7: Commit (ask user first)**

```bash
git add src/simulation.rs tests/spawn_density.rs
git commit -m "Scale central_swarm annulus with sqrt(n) for constant density

Fixed 60..280 radius bounds made spawn density grow linearly with
swarm_size, degrading the Barnes-Hut force walk past O(n log n)
(measured ~169x vs ~103x predicted, n=1000->64000; quadtree_build
tracked prediction at ~105x). Radii now scale with sqrt(n/1000) so
area grows with n. world_half_size grows to contain the scaled swarm
(quadtree insert has no bounds check). n=1000 is pixel-identical to
before (scale == 1.0)."
```

---

### Task 2: Zoom-to-fit camera in `main.rs`

**Files:**
- Modify: `src/main.rs` (draw loop, lines ~267-271)

No unit test (rendering); verification is visual + `--benchmark`.

- [ ] **Step 1: Implement the camera**

In `main.rs`, replace this block (currently lines ~267-271):

```rust
        for (obj, color) in sim.objects().iter().zip(colors.iter()) {
            draw_circle(obj.position.x, obj.position.y, 5.0, *color);
        }
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 20.0, 20.0, WHITE);
        draw_text(&format!("Energy: {:.4}", total_energy), 10.0, 40.0, 20.0, WHITE);
```

with:

```rust
        // Zoom-to-fit: the physics domain grows with sqrt(swarm_size)
        // (constant spawn density), so map the whole world square onto the
        // fixed 800x800 sim area left of the egui sidebar. At the default
        // n=1000, world == screen and this camera is the identity mapping.
        let screen_size = sim.config().screen_size;
        let world_size = sim.world_half_size() * 2.0;
        set_camera(&Camera2D {
            target: vec2(screen_size / 2.0, screen_size / 2.0),
            zoom: vec2(2.0 / world_size, -2.0 / world_size),
            viewport: Some((0, 0, screen_size as u32, screen_size as u32)),
            ..Default::default()
        });
        // Compensate dot size: 5 screen px regardless of zoom level.
        let dot_radius = 5.0 * (world_size / screen_size);
        for (obj, color) in sim.objects().iter().zip(colors.iter()) {
            draw_circle(obj.position.x, obj.position.y, dot_radius, *color);
        }
        set_default_camera();
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 20.0, 20.0, WHITE);
        draw_text(&format!("Energy: {:.4}", total_energy), 10.0, 40.0, 20.0, WHITE);
```

Notes:
- `viewport: Some((0, 0, 800, 800))` keeps the sim in the left square of the 1080x800 window; without it the camera stretches the world across the sidebar area too.
- `zoom` y is negative to keep macroquad's default y-down screen orientation.
- `set_default_camera()` before `draw_text`/`egui_macroquad::draw()` keeps HUD text and the sidebar in raw screen pixels, unchanged.

- [ ] **Step 2: Build**

Run: `cargo build --release`
Expected: 0 errors.

- [ ] **Step 3: Visual verification**

Run: `cargo run --release`
Expected at default n=1000: **pixel-identical** to before (camera is identity, dot radius 5.0 world units = 5px).
Then set swarm_size = 50000, "Applica": the whole annulus is visible, scaled down; bodies remain ~5px dots; the swarm orbits coherently; sidebar/HUD unchanged.

- [ ] **Step 4: Commit (ask user first)**

```bash
git add src/main.rs
git commit -m "Add zoom-to-fit camera for density-scaled worlds

Maps the (now sqrt(n)-scaled) physics domain onto the fixed 800x800
sim viewport left of the sidebar, with dot radius compensated to stay
~5px. Identity mapping at the default n=1000."
```

---

### Task 3: Fix benchmark half-size + re-establish scaling baselines

**Files:**
- Modify: `benches/physics_benchmarks.rs`

The benches currently hardcode `half_size = SCREEN_SIZE / 2.0` (400). After Task 1, production runs at 44000/64000 use a much larger root; measuring with 400 would benchmark a tree shape production never builds (bodies outside the root, misfiled) — invalid numbers.

- [ ] **Step 1: Use the production extent in every bench**

Refactor the helper to return the sim, and derive `half_size` from it. Replace `build_objects`:

```rust
fn build_sim(swarm_size: usize) -> Simulation {
    Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size },
        screen_size: SCREEN_SIZE,
        physics_dt: PHYSICS_DT,
        time_scale: 1.0,
    })
}
```

In each of the 6 bench functions, replace the per-size setup pattern:

```rust
        let objects = build_objects(n);            // (or Rc::new(build_objects(n)))
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = SCREEN_SIZE / 2.0;
```

with:

```rust
        let sim = build_sim(n);
        let objects = sim.objects().to_vec();      // wrap in Rc::new(...) where the old code did
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = sim.world_half_size();
```

Keep everything else (criterion group structure, SWARM_SIZES, warm-up pattern in `bench_verlet_step_cached`) unchanged.

- [ ] **Step 2: Build the benches**

Run: `cargo bench --no-run`
Expected: 0 errors.

- [ ] **Step 3: Run the full suite and record numbers**

Run: `cargo bench 2>&1 | Tee-Object -FilePath "$env:TEMP\bench_after_density_fix.txt"` (long — 6 groups x 7 sizes; expect several minutes, more at 64000)

Then extract per-group times at n=1000 and n=64000 and compute the growth ratio. O(n log n) predicts **~103x** for 64x bodies.

**Success criterion (the point of this whole plan):** `walk_forces` ratio within **103x ± 15% (88x-119x)**. Previously ~169x.
Also record `quadtree_build`, `compute_accelerations`, `verlet_step`, `verlet_step_cached` ratios. `clone_objects` should stay ~64x (pure O(n) — sanity anchor).

If `walk_forces` still exceeds the band: stop, report the new curve, and investigate (e.g. check tree depth distribution, `MAX_DEPTH=20` saturation at 64000, or cache effects) — do not declare success on "looks better."

- [ ] **Step 4: Commit (ask user first)**

```bash
git add benches/physics_benchmarks.rs
git commit -m "Benchmark with production world extent; new scaling baselines

Benches hardcoded half_size=400, which after the sqrt(n) spawn scaling
would measure a tree shape production never builds (out-of-root bodies
misfiled by insert). Now derived from Simulation::world_half_size().
Post-fix walk_forces n=1000->64000: <FILL MEASURED>x vs 103x O(n log n)
prediction (was ~169x before the density fix)."
```

(Fill the measured ratio into the commit message from Step 3's data.)

---

### Task 4: End-to-end verification at 44000 + energy divergence re-check

**Files:**
- Modify: `examples/profile_workload.rs` (stale comment only)

- [ ] **Step 1: Headless physics timing**

Run: `cargo build --release --example profile_workload; ./target/release/examples/profile_workload.exe`
Expected: completes; record ms/step. Expect a substantial drop vs the pre-fix figure (~37ms/step reference in the old plan's notes). This is the same deterministic workload as before, so numbers are directly comparable.

Update the now-stale comment on `PROFILE_SWARM_SIZE` (line 3, "the empirically '20-30 FPS' cliff point") to reflect that 44000 was the pre-density-fix cliff; keep the constant itself (continuity of the measurement point).

- [ ] **Step 2: Full render-path benchmark**

Run: `cargo run --release -- --benchmark`
Expected: prints min/mean/p50/p95/p99/max frame time at swarm_size=44000. Record and compare against the pre-fix baseline. Expect p50 to improve markedly (force walk restored toward O(n log n)); exact magnitude is data, not a promise.

- [ ] **Step 3: Energy divergence re-check (observation only)**

The pre-fix anomaly: exact `total_energy` diverged from ~-3e11 to ~+3.9e16 within the first logged frames at n=44000 (plausibly close encounters at extreme density).

Run: `cargo run --release`, set swarm_size=44000, "Applica", watch the `total_energy=...` log for ~1 minute (at 44000 the log interval is ~199 frames).
Expected: energy stays bounded (large negative, slowly drifting) — no sign flip, no runaway to 1e16. If it still diverges: that is a **separate numerical-stability bug** (close encounters / f32 precision), out of scope here — report it, don't fix it in this plan.

- [ ] **Step 4: Full test suite**

Run: `cargo test`
Expected: all PASS.

- [ ] **Step 5: Commit (ask user first)**

```bash
git add examples/profile_workload.rs
git commit -m "Update stale cliff-point comment after density fix"
```

---

### Task 5: Re-measure `total_energy_approx` viability (characterization)

**Files:**
- Modify: `tests/energy_approx_accuracy.rs`
- Modify: `src/main.rs` (comment lines 6-15, if measurements warrant)

The approx-energy error was density-driven (0.5% @ 500 → 200% @ 44000). With density now constant, the error curve should be flat. If so, `total_energy_approx` becomes a viable O(n log n) replacement for the exact O(n^2) energy display — but **adopting it is a follow-up decision, not this plan**. This task only measures and documents.

- [ ] **Step 1: Add a characterization test (measure first, then pin)**

Add to `tests/energy_approx_accuracy.rs`:

```rust
// Post density-fix characterization: spawn density is now constant across
// swarm_size, so the approx error should stay flat instead of exploding with
// n (pre-fix: 0.5% @ 500 -> 200% @ 44000). Threshold set from measured values
// plus margin; exact energy is O(n^2), so keep sizes moderate in CI.
#[test]
fn approx_error_stays_flat_at_constant_density() {
    for swarm_size in [500, 2000, 8000] {
        // ... same body as approx_energy_matches_exact_at_normal_density ...
    }
}
```

Run it first with a temporary loose bound (or just print the errors via `cargo test -- --nocapture`) at [500, 2000, 8000] to get real numbers. **Then** set the assertion bound to the worst observed error + 50% margin, and record the measured errors in the test comment. Do not guess the bound before measuring.

Also run one manual (non-committed, e.g. temporary) check at 32000-44000 to document the high-end error in the commit message — this is the number that decides whether reviving approx for the energy display is worth a follow-up plan.

- [ ] **Step 2: Update stale comments**

- `src/main.rs` lines 6-15 (`ENERGY_LOG_INTERVAL_FRAMES` comment): the "error grows sharply with density ... 200% at n=44000" framing predates the fix. Update with the measured post-fix error curve.
- `tests/energy_approx_accuracy.rs` header comment: same update.

- [ ] **Step 3: Run tests**

Run: `cargo test`
Expected: all PASS.

- [ ] **Step 4: Commit (ask user first)**

```bash
git add tests/energy_approx_accuracy.rs src/main.rs
git commit -m "Characterize approx-energy error at constant density

Post-fix measured relative error: <FILL> @ 500, <FILL> @ 2000, <FILL> @ 8000
(pre-fix: 0.5% / 4.3% / 30.8%, 200% @ 44000). Flat curve would make
total_energy_approx viable as the O(n log n) energy display — follow-up
decision, not adopted here."
```

---

## Non-goals (explicit)

- No density scaling for `RandomSwarm`/`RandomNBody` (user-controlled radius/spread). Known trap remains if a user combines extreme slider values; `world_extent` at least keeps the quadtree root correct for `RandomSwarm`.
- No replacement of the exact `total_energy` display with an approximation (follow-up, contingent on Task 5).
- No fix for close-encounter/f32 numerical instability if it persists post-fix (Task 4 Step 3 only detects it).
- No changes to `TETHA_THRESHOLD`, `SOFTENING`, `BUCKET_CAP`, `MAX_DEPTH` — the fix targets density, not BH tuning. If Task 3 shows `MAX_DEPTH=20` saturating at 64000, that's a separate finding to report.
- No edits to historical plan docs under `docs/superpowers/plans/2026-07-22-*` — new baselines live in this plan's commit messages and Task 3/5 results.

## Success checklist (final)

- [ ] `cargo test` green (old + new tests)
- [ ] `walk_forces` 1000→64000 growth within 88x-119x (criterion)
- [ ] `--benchmark` p50 at 44000 improved vs pre-fix baseline (recorded)
- [ ] Default n=1000 rendering pixel-identical to pre-change
- [ ] Energy log at 44000 bounded (or divergence reported as separate bug)
- [ ] Post-fix approx-energy error curve measured and documented
