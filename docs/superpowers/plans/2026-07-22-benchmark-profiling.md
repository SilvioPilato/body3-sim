# Deterministic Benchmarking and Profiling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `criterion` benchmarks that isolate the CPU cost of each physics phase (quadtree build, force-walk, full step, vector clone) at fixed swarm sizes, plus a standalone example binary suitable for `samply` profiling — both bypassing rendering/wall-clock entirely, to explain why FPS drops far steeper than O(n log n) past ~20-40k bodies.

**Architecture:** Widen `src/physics.rs`'s API minimally (make `compute_accelerations` `pub`, extract a new `pub fn walk_forces`) so a `benches/` crate can call each phase directly. Add `benches/physics_benchmarks.rs` (criterion, `harness = false`) and `examples/profile_workload.rs` (no window/GPU, fixed deterministic workload). Neither touches `main.rs` or runtime app behavior.

**Tech Stack:** Rust (edition 2024), `criterion = "0.5"` (dev-dependency only), external `samply` CLI (installed separately, not a Cargo dependency).

**Spec:** `docs/superpowers/specs/2026-07-22-benchmark-profiling-design.md`

---

### Task 1: Widen `physics.rs` to expose build/walk phases separately

**Files:**
- Modify: `src/physics.rs`

- [ ] **Step 1: Replace `compute_accelerations` with a pub wrapper + extracted `walk_forces`**

Current code (lines 85-112):

```rust
    fn compute_accelerations(objects: &[PhysicsObject], center: Vec2, half_size: f32) -> Vec<Vec2>{
        let tree = Quadtree::build(objects, center, half_size);
        let mut res = Vec::new();
        for i in 0..objects.len() {
            let mut acc = Vec2::ZERO;
            tree.root.walk(&mut |node| {
                if let Some(indices) = node.indices {
                    // foglia: forza diretta, i è catturato dalla closure
                    for &j in indices {
                        if j != i { 
                            acc+= Physics::compute_acceleration(objects[i].position, objects[j].position, objects[j].mass);
                        }
                    }
                    WalkDecision::Skip
                } else {
                    let d = Vec2::distance(objects[i].position, node.center_of_mass);
                    if d == 0.0 || (node.half_size * 2.0) / d > TETHA_THRESHOLD  {
                        WalkDecision::Descend
                    } else {
                        acc += Physics::compute_acceleration(objects[i].position, node.center_of_mass, node.total_mass);
                        WalkDecision::Skip
                    }
                }
            }, &tree.objects);
            res.push(acc);
        }
        res
    }
```

Replace with:

```rust
    pub fn compute_accelerations(objects: &[PhysicsObject], center: Vec2, half_size: f32) -> Vec<Vec2> {
        let tree = Quadtree::build(objects, center, half_size);
        Self::walk_forces(objects, &tree)
    }

    pub fn walk_forces(objects: &[PhysicsObject], tree: &Quadtree<'_>) -> Vec<Vec2> {
        let mut res = Vec::new();
        for i in 0..objects.len() {
            let mut acc = Vec2::ZERO;
            tree.root.walk(&mut |node| {
                if let Some(indices) = node.indices {
                    // foglia: forza diretta, i è catturato dalla closure
                    for &j in indices {
                        if j != i {
                            acc += Physics::compute_acceleration(objects[i].position, objects[j].position, objects[j].mass);
                        }
                    }
                    WalkDecision::Skip
                } else {
                    let d = Vec2::distance(objects[i].position, node.center_of_mass);
                    if d == 0.0 || (node.half_size * 2.0) / d > TETHA_THRESHOLD {
                        WalkDecision::Descend
                    } else {
                        acc += Physics::compute_acceleration(objects[i].position, node.center_of_mass, node.total_mass);
                        WalkDecision::Skip
                    }
                }
            }, &tree.objects);
            res.push(acc);
        }
        res
    }
```

This is behavior-preserving: `compute_accelerations` does exactly what it did before (build then walk), just split into two `pub` pieces so a benchmark can measure each separately. `EulerSimple`/`Verlet` (the only existing callers) are unaffected — same call, same result.

- [ ] **Step 2: Verify no behavior change**

Run: `cargo build`
Expected: `0 errors`. Same pre-existing baseline warning (`unused import: NodeView`) — no new warnings (both new functions are `pub`, so no dead-code lint; `Quadtree` is already imported at the top of the file).

Run: `cargo run` for a few seconds (default `CentralSwarm`, swarm_size 1000)
Expected: window opens, swarm renders and orbits exactly as before, `total_energy=...` prints periodically, no panic. This confirms the extraction didn't change simulation behavior — bodies should look and move identically to before this change (same physics, just reorganized code).

- [ ] **Step 3: Commit**

```bash
git add src/physics.rs
git commit -m "Split compute_accelerations into build + walk_forces

Both now pub. Lets a benchmark measure quadtree build cost and
force-walk cost separately instead of only their combined total —
needed to diagnose why FPS drops steeper than O(n log n) predicts
at large swarm sizes. Behavior-preserving: compute_accelerations
still does exactly build-then-walk, existing callers unaffected."
```

---

### Task 2: Add criterion benchmarks

**Files:**
- Modify: `Cargo.toml`
- Create: `benches/physics_benchmarks.rs`

- [ ] **Step 1: Add the dev-dependency and bench target**

In `Cargo.toml`, add:

```toml
[dev-dependencies]
criterion = { version = "0.5", features = ["html_reports"] }

[[bench]]
name = "physics_benchmarks"
harness = false
```

(This goes after the existing `[dependencies]` section — `egui-macroquad` under `[dependencies]` stays where it is, untouched.)

- [ ] **Step 2: Write `benches/physics_benchmarks.rs`**

```rust
use std::rc::Rc;

use body3_sim::physics::{Physics, PhysicsObject, PhysicsSystem, Verlet};
use body3_sim::quadtree::Quadtree;
use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};
use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use macroquad::math::vec2;

const SWARM_SIZES: [usize; 7] = [1000, 2000, 4000, 8000, 16000, 32000, 64000];
const SCREEN_SIZE: f32 = 800.0;
const PHYSICS_DT: f32 = 0.005;

fn build_objects(swarm_size: usize) -> Vec<PhysicsObject> {
    let sim = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size },
        screen_size: SCREEN_SIZE,
        physics_dt: PHYSICS_DT,
        time_scale: 1.0,
    });
    sim.objects().to_vec()
}

fn bench_quadtree_build(c: &mut Criterion) {
    let mut group = c.benchmark_group("quadtree_build");
    for &n in &SWARM_SIZES {
        let objects = build_objects(n);
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = SCREEN_SIZE / 2.0;
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Quadtree::build(&objects, center, half_size));
        });
    }
    group.finish();
}

fn bench_walk_forces(c: &mut Criterion) {
    let mut group = c.benchmark_group("walk_forces");
    for &n in &SWARM_SIZES {
        let objects = build_objects(n);
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = SCREEN_SIZE / 2.0;
        let tree = Quadtree::build(&objects, center, half_size);
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Physics::walk_forces(&objects, &tree));
        });
    }
    group.finish();
}

fn bench_compute_accelerations(c: &mut Criterion) {
    let mut group = c.benchmark_group("compute_accelerations");
    for &n in &SWARM_SIZES {
        let objects = build_objects(n);
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = SCREEN_SIZE / 2.0;
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Physics::compute_accelerations(&objects, center, half_size));
        });
    }
    group.finish();
}

fn bench_verlet_step(c: &mut Criterion) {
    let mut group = c.benchmark_group("verlet_step");
    for &n in &SWARM_SIZES {
        let objects = Rc::new(build_objects(n));
        let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
        let half_size = SCREEN_SIZE / 2.0;
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Verlet::execute(objects.clone(), PHYSICS_DT, center, half_size));
        });
    }
    group.finish();
}

fn bench_clone_objects(c: &mut Criterion) {
    let mut group = c.benchmark_group("clone_objects");
    for &n in &SWARM_SIZES {
        let objects = build_objects(n);
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| objects.to_vec());
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_quadtree_build,
    bench_walk_forces,
    bench_compute_accelerations,
    bench_verlet_step,
    bench_clone_objects
);
criterion_main!(benches);
```

`SWARM_SIZES` stops at 64000 rather than going further, since the manual scaling test already showed 60000 bodies at 9 FPS (in-app) — well past the "unusable" threshold; the benchmarks exist to explain the shape of the curve already observed, not to explore beyond it.

- [ ] **Step 3: Run the benchmarks**

Run: `cargo bench` (this will take a while — criterion runs multiple warm-up + measurement iterations per benchmark, and there are 5 phases x 7 sizes = 35 benchmarks total; expect several minutes)

**If it doesn't compile as-is:** criterion's exact API (e.g. `BenchmarkId::from_parameter`, `group.bench_with_input`'s closure signature) was written from memory without the ability to fetch current docs. If you hit a compile error, diagnose from the actual error and fix the minimal API detail needed — criterion's grouped/parameterized benchmark pattern has been stable for a long time, so this is a low-risk area, but don't restructure the benchmark set (5 phases x 7 sizes) or the `build_objects` helper if you can avoid it; only adjust the specific API call that doesn't match.

Expected: `cargo bench` completes with 0 errors, prints timing summaries for all 35 benchmarks, and writes an HTML report under `target/criterion/`.

- [ ] **Step 4: Commit**

```bash
git add Cargo.toml Cargo.lock benches/physics_benchmarks.rs
git commit -m "Add criterion benchmarks for physics phases

Measures quadtree build, force-walk, combined compute_accelerations,
a full Verlet step, and the PhysicsObject vector clone, each across
swarm_size in [1000..64000], bypassing rendering/wall-clock entirely
via the existing deterministic CentralSwarm scenario."
```

---

### Task 3: Add the profiling example binary

**Files:**
- Create: `examples/profile_workload.rs`

- [ ] **Step 1: Write `examples/profile_workload.rs`**

```rust
use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

const PROFILE_SWARM_SIZE: usize = 44_000; // the empirically "20-30 FPS" cliff point
const PROFILE_ITERATIONS: u32 = 300;

fn main() {
    let mut sim = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: PROFILE_SWARM_SIZE },
        screen_size: 800.0,
        physics_dt: 0.005,
        time_scale: 1.0,
    });

    let start = std::time::Instant::now();
    for _ in 0..PROFILE_ITERATIONS {
        let dt = sim.config().physics_dt;
        sim.update(dt);
    }
    let elapsed = start.elapsed();

    println!(
        "{PROFILE_ITERATIONS} steps at swarm_size={PROFILE_SWARM_SIZE}: {:.3}s total, {:.3}ms/step",
        elapsed.as_secs_f64(),
        elapsed.as_secs_f64() * 1000.0 / PROFILE_ITERATIONS as f64
    );
}
```

No `#[macroquad::main]`, no window, no GPU — a plain `fn main()`. `time_scale: 1.0` plus passing exactly `physics_dt` as the update's "frame time" makes the accumulator threshold hit exactly once per call (`1.0 * physics_dt == physics_dt`), so every call runs exactly one `Verlet::execute` substep — deterministic, no dependence on real elapsed time. 300 iterations is chosen to comfortably exceed a few seconds of runtime even at the slow end (~30-40ms/step at 44k bodies per the manual test), giving a profiler enough samples.

- [ ] **Step 2: Build and run it directly (sanity check, no profiler yet)**

Run: `cargo build --release --example profile_workload`
Expected: `0 errors`.

Run: `./target/release/examples/profile_workload.exe`
Expected: it runs for several seconds (no window ever appears — this is expected, there is none), then prints one line like `300 steps at swarm_size=44000: X.XXXs total, Y.YYYms/step`, then exits. The ms/step figure should be in a broadly similar ballpark to the manual test's observation at swarm_size 44000 (~37ms/step, i.e. 1000/27fps) — not identical, since this measures physics alone with no rendering overlaid, so it may well be faster; a wildly different order of magnitude (10x+ off) would suggest something is wrong rather than "rendering overhead was removed."

- [ ] **Step 3: Commit**

```bash
git add examples/profile_workload.rs
git commit -m "Add standalone example for samply profiling

No window/GPU/egui — fixed deterministic workload (300 steps at
swarm_size=44000, the empirically observed 20-30 FPS cliff point)
so a sampling profiler can be pointed at pure physics cost without
rendering noise mixed in."
```

---

### Task 4: Run a real profiling pass and report findings

**Files:** None (investigation only — no code changes expected from this task).

This task has no automated pass/fail; it's the actual point of the whole plan — using the infrastructure built in Tasks 1-3 to get real data on where time goes.

- [ ] **Step 1: Install samply (one-time, external tool)**

Run: `cargo install samply`
Expected: installs successfully (may take a few minutes to compile). If it fails to install on this machine (e.g. missing build tools), report that specifically rather than trying to work around it silently — profiling can still proceed later once resolved, and the benchmark data from Task 2 stands on its own regardless.

- [ ] **Step 2: Record a profile**

Run: `samply record target/release/examples/profile_workload.exe`
Expected: the binary runs to completion (as in Task 3 Step 2, same printed line), then `samply` opens the Firefox Profiler UI in a browser with the recorded call tree.

- [ ] **Step 3: Read the results**

Look at the call tree / flamegraph rooted at `main` -> `Simulation::update` -> `Verlet::execute`. Note which of `Quadtree::build` vs `Physics::walk_forces` (or their combined `compute_accelerations` caller) vs the `objects.clone()` inside `Verlet::execute` dominates the self-time, especially compared to what Task 2's criterion numbers show at `swarm_size=44000`. This directly answers the question that motivated this whole plan: is the steeper-than-O(n log n) drop coming from tree-build allocation churn, from the walk itself, from the vector clone, or from something not yet isolated (e.g. quadtree depth hitting `MAX_DEPTH` and degrading into large linear leaf scans)?

- [ ] **Step 4: Report**

Summarize what the criterion numbers (Task 2) and the profiler's call tree (this task) show — which phase's cost grows fastest relative to `swarm_size`, and whether that matches or contradicts the "not simply O(n log n)" observation from the manual test. No code changes are expected as part of this task; a follow-up plan would be the place to actually act on whatever the data shows.
