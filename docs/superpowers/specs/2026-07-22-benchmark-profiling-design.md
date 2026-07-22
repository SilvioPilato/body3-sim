# Deterministic benchmarking and profiling infrastructure

Date: 2026-07-22

## Context

A manual scaling test (launching the GUI app at increasing `swarm_size` and
reading the on-screen FPS counter from screenshots) found that performance
holds roughly flat from 1000 to ~2500 bodies (182 -> 168 FPS) then drops far
more steeply than the O(n log n) Barnes-Hut force calculation alone would
predict once past ~20000-40000 bodies (98 FPS at 20000, 27 FPS at 44000, 9
FPS at 60000, release build). That test was explicitly acknowledged as
noisy: FPS is driven by `get_frame_time()` (real wall-clock), mixed with
GPU/vsync/egui-panel rendering cost, and subject to whatever else was
running on the machine at the time. It cannot isolate *which* part of the
physics step is responsible for the steeper-than-expected drop.

This spec adds infrastructure to answer that with real data: `criterion`
benchmarks that measure pure CPU cost of specific physics phases at fixed,
reproducible inputs (no window, no GPU, no wall-clock jitter), plus a
minimal standalone binary suitable for attaching a sampling profiler
(`samply`) to see exactly where time goes inside a single call.

## Goals

- Add `criterion` as the only new dependency, as a `[dev-dependencies]` entry
  (never compiled into the shipped app or `cargo build --release` of the
  main binary — only affects `cargo bench`).
- Add `benches/physics_benchmarks.rs` measuring five phases — quadtree
  build, force-walk, the existing combined build+walk call, a full Verlet
  step, and the `Vec<PhysicsObject>` clone — each across a fixed geometric
  progression of `swarm_size` (1000, 2000, 4000, 8000, 16000, 32000, 64000),
  using the existing `CentralSwarm` scenario (fully deterministic, no RNG)
  as the data source via the already-public `Simulation`/`SimulationConfig`
  API.
- Widen `src/physics.rs`'s API surface minimally so the benchmark crate
  (which, like any Rust integration test/bench, only sees the library's
  `pub` items) can call the phases directly:
  - `Physics::compute_accelerations` becomes `pub` (currently private).
  - A new `pub fn Physics::walk_forces(objects: &[PhysicsObject], tree:
    &Quadtree) -> Vec<Vec2>` is extracted from `compute_accelerations`'s
    existing loop body, so build cost and walk cost can be measured
    separately. `compute_accelerations` becomes a thin wrapper (`build`
    then `walk_forces`) with identical behavior — existing callers
    (`Verlet`, `EulerSimple`) are unaffected.
  - `Quadtree::build` is already `pub`; no change needed there.
- Add `examples/profile_workload.rs`: no macroquad window, no GPU, no
  egui — just a fixed, large, deterministic `CentralSwarm` swarm size,
  run through a fixed number of `Simulation::update()` calls with
  `time_scale: 1.0` and a constant `physics_dt`-sized input (so exactly one
  substep runs per call, avoiding any accumulator drift or wall-clock
  variance), then a short printed summary (elapsed time, iteration count,
  average ms/step) before exiting. This is the thing `samply record` gets
  pointed at.
- Document the `samply` workflow (install + invocation) so profiling is
  reproducible by anyone who picks this project up later, without adding
  `samply` itself as a project dependency (it's an external CLI tool,
  installed once via `cargo install samply`, not a `Cargo.toml` entry).

## Non-goals

- No CI integration (no automated perf-regression gate) — this is a manual
  investigation/diagnosis tool for now, not a gate on every commit.
- No change to the shipped app's runtime behavior, `main.rs`, or the GUI —
  this is purely additive dev-tooling (`benches/`, `examples/`, two `pub`
  changes in `physics.rs` that don't alter behavior).
- No attempt to actually fix the steep scaling drop in this spec — the
  point of this infrastructure is to produce the data needed to diagnose
  it correctly, not to guess-and-check a fix blind.
- No CLI argument parsing for `profile_workload`'s swarm size — it's a
  fixed constant in the file, edited directly when a different size needs
  profiling, consistent with how this project already does one-off
  scaling experiments (matches the manual process just used, minus the
  wall-clock noise).
- No automated test suite beyond what's described here — this whole spec
  *is* the project's answer to "how do we test performance repeatably";
  it doesn't touch or expand the (still nonexistent) correctness test
  suite.

## Architecture

`Cargo.toml` gains:

```toml
[dev-dependencies]
criterion = { version = "0.5", features = ["html_reports"] }

[[bench]]
name = "physics_benchmarks"
harness = false
```

`src/physics.rs` changes (behavior-preserving):

```rust
pub fn compute_accelerations(objects: &[PhysicsObject], center: Vec2, half_size: f32) -> Vec<Vec2> {
    let tree = Quadtree::build(objects, center, half_size);
    Self::walk_forces(objects, &tree)
}

pub fn walk_forces(objects: &[PhysicsObject], tree: &Quadtree) -> Vec<Vec2> {
    // exact body already inside today's compute_accelerations, unchanged
}
```

`benches/physics_benchmarks.rs` (new file, criterion harness) benchmarks,
for each `swarm_size` in `[1000, 2000, 4000, 8000, 16000, 32000, 64000]`:

1. **`quadtree_build`** — `Quadtree::build(objects, center, half_size)`.
2. **`walk_forces`** — tree built once outside the measured region, then
   `Physics::walk_forces(objects, &tree)` measured alone.
3. **`compute_accelerations`** — `Physics::compute_accelerations(objects,
   center, half_size)` (today's combined build+walk cost, as a sanity
   check that should track close to phase 1 + phase 2's sum).
4. **`verlet_step`** — `Verlet::execute(Rc::new(objects.to_vec()), dt,
   center, half_size)` (a full physics substep: two force evaluations plus
   integration — the real per-substep cost the app pays).
5. **`clone_objects`** — `objects.to_vec()` alone, isolating the O(n)
   `PhysicsObject` vector clone flagged earlier as a minor, unconfirmed
   cost contributor.

Input data for every benchmark iteration comes from `Simulation::new(
SimulationConfig { scenario: Scenario::CentralSwarm { swarm_size: n },
screen_size: 800.0, physics_dt: 0.005, time_scale: 1.0 }).objects()` — the
existing public API, no new exports needed for scenario generation.
`center`/`half_size` for the direct `Quadtree`/`Physics` calls are derived
the same way `Simulation::new` derives them internally
(`vec2(screen_size / 2.0, screen_size / 2.0)` / `screen_size / 2.0`) — no
new accessor needed on `Simulation` since `screen_size` is already reachable
through the public `SimulationConfig`.

`examples/profile_workload.rs` (new file):

```rust
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
        sim.update(sim.config().physics_dt);
    }
    let elapsed = start.elapsed();

    println!(
        "{PROFILE_ITERATIONS} steps at swarm_size={PROFILE_SWARM_SIZE}: {:.3}s total, {:.3}ms/step",
        elapsed.as_secs_f64(),
        elapsed.as_secs_f64() * 1000.0 / PROFILE_ITERATIONS as f64
    );
}
```

`time_scale: 1.0` plus passing exactly `physics_dt` as the "frame time"
each call makes `Simulation::update`'s accumulator hit its threshold
exactly once per call (`1.0 * physics_dt == physics_dt`), so every call
runs exactly one `Verlet::execute` substep — no drift, no dependence on
real elapsed time, fully reproducible run to run. 300 iterations is chosen
to comfortably exceed a few seconds of runtime even at the slow end of the
observed range (~30-40ms/step at 44k bodies, per the manual test), giving
`samply` enough wall-clock duration to collect a useful number of samples.

### Profiling workflow (documented, not code)

```
cargo install samply          # one-time, external tool
cargo build --release --example profile_workload
samply record target/release/examples/profile_workload
```

`samply record` launches the binary, samples it while it runs, and opens
the result in the Firefox Profiler UI (local, no data leaves the machine)
once the process exits.

## Data flow

1. `cargo bench` builds and runs `benches/physics_benchmarks.rs` against
   the library's public API, producing criterion's standard HTML report
   (`target/criterion/`) with per-phase, per-swarm_size timing and
   statistical confidence intervals.
2. `cargo build --release --example profile_workload` produces a
   standalone binary with no window/GPU/egui involved.
3. `samply record` on that binary produces an interactive flamegraph/call
   tree, opened in-browser, showing where wall-clock time is actually
   spent inside the 300 profiled steps.
4. Both are independent of the GUI app — neither touches `main.rs` or
   changes runtime behavior of the shipped simulation.

## Error handling

None introduced beyond what already exists. No new fallible paths — the
two `physics.rs` changes are visibility/extraction only, behavior-identical
to today. The example binary has no error handling needs (no I/O beyond a
single `println!`, no user input, panics identically to today's app on any
internal invariant violation, which is out of scope here).

## Testing

This spec's own deliverable *is* the testing/measurement infrastructure —
there is no separate "test the tests" step beyond:

- `cargo bench` runs to completion with 0 errors, producing a report for
  all 5 phases x 7 swarm sizes.
- `cargo build --release --example profile_workload` compiles and, when
  run directly (without `samply`), prints a plausible summary line (e.g.
  ms/step in the same ballpark as the manual test's ~1/27fps ≈ 37ms at
  swarm_size 44000, allowing for the fact this measures physics alone with
  no rendering overlaid).
- One real `samply record` pass against the example, confirming the
  profiler UI opens and shows a call tree rooted in `Simulation::update`.
