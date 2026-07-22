# Simulation module design

Date: 2026-07-22

## Context

`src/main.rs` currently mixes four concerns: macroquad window setup, initial
scenario generation (`central_swarm`, plus three unused scenario functions:
`dual_circle`, `triangle_circle`, `burrau_1913`), the fixed-timestep physics
loop (accumulator + `Verlet::execute`), and rendering/HUD text.

The goal of this change is to extract everything that is not rendering into
a standalone `simulation` module inside the `body3-sim` lib crate, so that a
future GUI can construct/reconfigure a `Simulation` without touching
`main.rs`'s render loop. As part of this extraction we also want the
scenario system to support pseudo-random presets (not just the four fixed,
deterministic ones), since the eventual GUI should let a user roll a
randomized starting configuration in addition to picking a fixed preset.

This spec covers only the module extraction and the scenario/config API
shape. It does **not** cover building the GUI itself — that is future work
once this module exists.

## Goals

- Move all non-rendering simulation logic out of `main.rs` into
  `src/simulation.rs` (new module in the `body3-sim` lib crate).
- Fix the same "hardcoded center" bug in `dual_circle`, `triangle_circle`,
  and `burrau_1913` that was already fixed for `central_swarm` (they
  currently hardcode `cx = 300.0, cy = 300.0` instead of taking the actual
  world center).
- Design `SimulationConfig` / `Scenario` so that:
  - The four existing presets remain available and selectable.
  - Two new pseudo-random scenario kinds can be added: a randomized
    variation of the central-swarm preset, and an arbitrary-N-body random
    generator.
  - Random scenarios are reproducible given the same seed.
- Support runtime reconfiguration via `Simulation::reset(config)` (rebuild
  the scenario without recreating the OS window), since a future GUI will
  want an "Apply" button that swaps configuration without restarting the
  process.
- Keep `main.rs` reduced to: build a `SimulationConfig`, construct a
  `Simulation`, and each frame call `update`/read `objects`/`total_energy`
  for rendering.

## Non-goals

- No GUI/egui integration in this change — this is prep work only.
- No serialization (save/load config to disk) — out of scope until a GUI
  needs to persist presets.
- No change to the physics algorithms themselves (`Verlet`, `Physics`,
  `Quadtree` are untouched).
- No automated test suite is being introduced. The project currently has
  zero `#[test]` functions; this change follows the existing convention and
  is verified by `cargo build` + a manual smoke run, consistent with how
  prior changes in this session were verified.
- Window size remains fixed at process startup (macroquad limitation noted
  previously); `SimulationConfig::screen_size` is read once to build
  `window_conf()` and is not able to resize the live window if changed via
  `reset`.

## Architecture

New module `src/simulation.rs`, registered in `src/lib.rs` as
`pub mod simulation;`. It depends only on `crate::physics`, `std::rc::Rc`,
`macroquad::math::{Vec2, vec2}`, and `macroquad::rand::{gen_range, srand}`
(not `macroquad::prelude`), so it stays free of windowing/rendering/input
concerns and could in principle be reused without macroquad's app loop.

`main.rs` becomes:

```rust
use macroquad::prelude::*;
use body3_sim::simulation::{Simulation, SimulationConfig};

fn window_conf() -> Conf {
    let screen_size = SimulationConfig::default().screen_size;
    Conf {
        window_title: "Simulation".to_owned(),
        window_width: screen_size as i32,
        window_height: screen_size as i32,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut sim = Simulation::new(SimulationConfig::default());
    loop {
        clear_background(BLACK);
        sim.update(get_frame_time());
        let total_energy = sim.total_energy();
        println!("total_energy={:.4}", total_energy);
        for obj in sim.objects() {
            draw_circle(obj.position.x, obj.position.y, 5.0, RED);
        }
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 20.0, 20.0, WHITE);
        draw_text(&format!("Energy: {:.4}", total_energy), 10.0, 40.0, 20.0, WHITE);
        next_frame().await
    }
}
```

## Public API (`src/simulation.rs`)

```rust
pub enum Scenario {
    CentralSwarm { swarm_size: usize },
    DualCircle,
    TriangleCircle,
    Burrau1913,
    RandomSwarm(RandomSwarmParams),
    RandomNBody(RandomNBodyParams),
}

pub struct RandomSwarmParams {
    pub seed: u64,
    pub swarm_size: usize,
    pub radius_range: (f32, f32),
    pub central_mass_range: (f32, f32),
    pub light_mass_range: (f32, f32),
}

pub struct RandomNBodyParams {
    pub seed: u64,
    pub count: usize,
    pub mass_range: (f32, f32),
    pub position_spread: f32,
    pub velocity_range: (f32, f32),
}

pub struct SimulationConfig {
    pub scenario: Scenario,
    pub screen_size: f32,
    pub physics_dt: f32,
    pub time_scale: f32,
}

pub struct Simulation { /* config, center, world_half_size, objects, accumulator */ }

impl Simulation {
    pub fn new(config: SimulationConfig) -> Self;
    pub fn reset(&mut self, config: SimulationConfig);
    pub fn update(&mut self, frame_time: f32);
    pub fn objects(&self) -> &[PhysicsObject];
    pub fn total_energy(&self) -> f32;
    pub fn config(&self) -> &SimulationConfig;
}
```

`SimulationConfig::default()` matches today's runtime values:
`Scenario::CentralSwarm { swarm_size: 1000 }`, `screen_size: 800.0`,
`physics_dt: 0.005`, `time_scale: 0.3`.

`swarm_size` moves out of the top-level config (it previously applied
globally even though only one scenario used it) and into the two variants
that actually need a body count (`CentralSwarm`, `RandomSwarm`). The other
three fixed presets keep their hardcoded body counts (2, 3, 3
respectively) internally, since those are structurally fixed presets, not
tunable parameters.

`RandomSwarmParams` and `RandomNBodyParams` each implement `Default` with
placeholder-but-reasonable ranges (e.g. `radius_range` matching today's
`60.0..280.0`), so a future GUI has sensible starting values when the user
switches the scenario dropdown to a random kind. Exact default numbers are
an implementation detail, not a design constraint.

### Scenario generation

`build_scenario(scenario: &Scenario, center: Vec2) -> Vec<PhysicsObject>`
matches on the enum and dispatches to one private generator function per
variant:

- `central_swarm(swarm_size, center)` — existing golden-angle logic,
  unchanged, already takes `center`.
- `dual_circle(center)`, `triangle_circle(center)`, `burrau_1913(center)` —
  ported as-is but changed to take `center: Vec2` instead of hardcoding
  `cx = 300.0, cy = 300.0`, fixing the same off-center bug already fixed on
  `central_swarm`.
- `random_swarm(params: &RandomSwarmParams, center: Vec2)` — same shape as
  `central_swarm` (one central mass + `swarm_size` light bodies in orbit),
  keeping the golden-angle *angular* placement (for even coverage) but
  randomizing the *radius* per body within `radius_range`, and drawing
  central mass / light mass from their respective ranges. Velocity is not a
  separate random draw: it reuses `central_swarm`'s derived tangential
  circular-orbit speed (`sqrt(GRAVITY * central_mass / radius)`) computed
  from the already-randomized radius and central mass — this is why
  `RandomSwarmParams` has no `velocity_range` field, unlike
  `RandomNBodyParams`.
- `random_n_body(params: &RandomNBodyParams, center: Vec2)` — `count`
  bodies with random mass (from `mass_range`), random position within
  `position_spread` of `center`, and random velocity (magnitude from
  `velocity_range`, random direction). No structural assumption of a
  central body.

### Determinism

Random generators seed macroquad's built-in RNG once via
`macroquad::rand::srand(params.seed)` and then draw values with
`macroquad::rand::gen_range(...)`. This requires no new dependency (the
`rand` crate is not added). Because the app is single-threaded and the call
order within a given generator function is fixed, the same seed always
produces the same scenario. Reseeding is local to scenario construction —
nothing else in the codebase currently consumes randomness, so there is no
cross-talk with other systems.

## Data flow

1. `main()` builds a `SimulationConfig` (today: always `::default()`;
   later: from GUI state) and constructs `Simulation::new(config)`.
2. Each frame, `main()` calls `sim.update(get_frame_time())`. Internally
   this accumulates `frame_time * config.time_scale` and drains it in
   `config.physics_dt` steps via `Verlet::execute`, exactly as today's
   inline loop does.
3. `main()` reads `sim.objects()` to draw circles and `sim.total_energy()`
   for the HUD text and the `println!` log line.
4. (Future, not implemented here) a GUI panel would mutate a
   `SimulationConfig` and call `sim.reset(config)` on "Apply", which
   rebuilds `center`/`world_half_size`/`objects`/`accumulator` from
   scratch via `Simulation::new` semantics. If the new config's
   `screen_size` differs from the one the OS window was created with,
   `reset` does **not** resize the window (see Non-goals) — `center` and
   `world_half_size` would then reflect a world that doesn't match the
   visible canvas. Not a concern for this change (`main.rs` never calls
   `reset`), but worth resolving (e.g. clamp/ignore `screen_size` changes
   in `reset`, or make the window resizable) before a GUI wires up "Apply".

## Error handling

None introduced. No I/O, no parsing, no fallible construction — every path
here is infallible, same as the code being moved.

## Testing

No automated test suite exists in the project today (verified: zero
`#[test]` occurrences). This change is verified the same way prior changes
in this session were verified:

- `cargo build` with zero errors.
- A manual smoke run (`cargo run`) to confirm the window opens, the default
  `CentralSwarm` scenario renders and orbits as before, FPS/Energy HUD text
  still appears, and there is no panic.
