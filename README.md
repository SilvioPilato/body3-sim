# body3-sim

An interactive N-body gravity simulator: a Barnes-Hut quadtree force solver
with a velocity-Verlet integrator, rendered in real time via [macroquad](https://github.com/not-fl3/macroquad).
Runs natively or in the browser (WebGL/WASM).

## Running

```sh
cargo run --release
```

Native builds respect `--benchmark [N]`, which runs a fixed swarm size (default
44,000 bodies) for a fixed number of frames and prints render-time percentiles
instead of opening interactively.

## What it simulates

Pick a scenario from the sidebar:

- **Central Swarm** / **Random Swarm** — a massive core with orbiting bodies,
  circularized against the force field they actually feel
- **Galaxy Collision** — two swarms launched at each other on a grazing pass
- **Solar System**, **Circumbinary**, **Trojan (L4/L5)** — orbital-mechanics
  presets (prograde circular orbits, a binary star with planets, Lagrange-point
  clusters)
- **Figure Eight** — the Chenciner-Montgomery three-body choreography
- **Dual Circle**, **Triangle Circle**, **Burrau 1913**, **Slingshot** —
  classic few-body configurations
- **Random N-Body** — unconstrained random initial conditions

Non-default configurations round-trip through the URL (**Copy link** /
**Apply**), so a specific setup can be shared or bookmarked.

## Physics

- **Force solver**: Barnes-Hut quadtree (`src/quadtree.rs`, `src/physics.rs`),
  O(n log n) instead of the exact O(n²) all-pairs sum. The opening-angle
  threshold (`theta_threshold`) trades accuracy for speed.
- **Integrator**: velocity Verlet, substepped at a fixed `physics_dt`
  independent of frame rate.
- **Softening**: a Plummer softening length caps the close-encounter force so
  the fixed-timestep integrator stays energy-stable. Both `physics_dt` and
  `softening` are derived per scenario from a stability criterion
  (`simulation::integration_params`) rather than hand-tuned.
- **Energy display**: the exact total energy is O(n²), so at large swarm
  sizes it's computed on a background thread (`src/energy.rs`) and updates
  asynchronously rather than blocking the render loop. Unavailable on wasm32
  (no threads without SharedArrayBuffer).

## Project layout

```
src/simulation.rs   scenario definitions, initial conditions, integration loop
src/physics.rs       force/energy computation, Verlet integrator
src/quadtree.rs      Barnes-Hut tree
src/camera.rs        auto-fit camera (follows center of mass, zooms to contain the system)
src/energy.rs        background thread for exact energy computation
src/url.rs           SimulationConfig <-> URL query string
src/main.rs          window, UI (egui), render loop
tests/               integration and regression tests
examples/            diagnostic tools used to derive/validate the physics constants above
benches/             Criterion benchmarks for the force solver and integrator
```

## Testing

```sh
cargo test              # unit + integration tests
cargo bench              # Criterion benchmarks (force solver, integrator)
cargo run --example energy_bench    # headless energy-conservation profiling
```

## Web build

See [docs/web-deploy.md](docs/web-deploy.md) for the WASM build, local
COOP/COEP test server, and Cloudflare Pages deploy setup.
