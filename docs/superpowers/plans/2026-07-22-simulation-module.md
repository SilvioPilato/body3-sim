# Simulation Module Extraction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract all non-rendering simulation logic out of `src/main.rs` into a new `src/simulation.rs` module, and give it a `SimulationConfig`/`Scenario` shape that supports both the four existing fixed presets and two new pseudo-random scenario kinds — preparing the codebase for a future configuration GUI.

**Architecture:** `src/simulation.rs` owns `SimulationConfig`, the `Scenario` enum (with per-variant params for the two random scenarios), and a `Simulation` struct (`new`/`reset`/`update`/`objects`/`total_energy`/`config`). It depends only on `crate::physics` and `macroquad::math`/`macroquad::rand` — no windowing/rendering. `src/main.rs` shrinks to window setup + the render loop, calling into `Simulation`.

**Tech Stack:** Rust (edition 2024), macroquad 0.4.15 (`macroquad::math` for `Vec2`/`vec2`, `macroquad::rand` for seeded RNG — no new crate dependency).

**Spec:** `docs/superpowers/specs/2026-07-22-simulation-module-design.md`

---

## Reference: current code being moved

`src/main.rs` today (before this plan) contains `central_swarm(n, center)` (already takes `center`, correct), plus three **unused** functions that hardcode `cx = 300.0, cy = 300.0`: `dual_circle()`, `triangle_circle()`, `burrau_1913()`. It also contains the consts `PHYSICS_DT`, `SWARM_SIZE`, `SCREEN_SIZE`, `TIME_SCALE`, `window_conf()`, and the `main()` loop with the accumulator/`Verlet::execute` logic. All of this moves into `simulation.rs` except `window_conf()` and the render loop, which stay in `main.rs`.

---

### Task 1: Create the `simulation` module

**Files:**
- Create: `src/simulation.rs`
- Modify: `src/lib.rs`

- [ ] **Step 1: Register the module**

In `src/lib.rs`, add the module declaration:

```rust
pub mod physics;
pub mod quadtree;
pub mod simulation;
```

- [ ] **Step 2: Write `src/simulation.rs` — imports and config types**

```rust
use std::f32::consts::TAU;
use std::rc::Rc;

use macroquad::math::{Vec2, vec2};
use macroquad::rand::{gen_range, srand};

use crate::physics::{GRAVITY, Physics, PhysicsObject, PhysicsSystem, Verlet};

#[derive(Clone, Copy, Debug)]
pub enum Scenario {
    CentralSwarm { swarm_size: usize },
    DualCircle,
    TriangleCircle,
    Burrau1913,
    RandomSwarm(RandomSwarmParams),
    RandomNBody(RandomNBodyParams),
}

#[derive(Clone, Copy, Debug)]
pub struct RandomSwarmParams {
    pub seed: u64,
    pub swarm_size: usize,
    pub radius_range: (f32, f32),
    pub central_mass_range: (f32, f32),
    pub light_mass_range: (f32, f32),
}

impl Default for RandomSwarmParams {
    fn default() -> Self {
        Self {
            seed: 42,
            swarm_size: 300,
            radius_range: (60.0, 280.0),
            central_mass_range: (5_000.0, 30_000.0),
            light_mass_range: (0.5, 2.0),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RandomNBodyParams {
    pub seed: u64,
    pub count: usize,
    pub mass_range: (f32, f32),
    pub position_spread: f32,
    pub velocity_range: (f32, f32),
}

impl Default for RandomNBodyParams {
    fn default() -> Self {
        Self {
            seed: 42,
            count: 6,
            mass_range: (50.0, 2_000.0),
            position_spread: 300.0,
            velocity_range: (0.0, 40.0),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SimulationConfig {
    pub scenario: Scenario,
    pub screen_size: f32,
    pub physics_dt: f32,
    pub time_scale: f32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            scenario: Scenario::CentralSwarm { swarm_size: 1000 },
            screen_size: 800.0,
            physics_dt: 0.005,
            time_scale: 0.3,
        }
    }
}
```

These are the same default values `main.rs` uses today (`SWARM_SIZE = 1000`, `SCREEN_SIZE = 800.0`, `PHYSICS_DT = 0.005`, `TIME_SCALE = 0.3`), so behavior doesn't change once main.rs is switched over in Task 2.

- [ ] **Step 3: Add the scenario generator functions (ported presets)**

Append to `src/simulation.rs`. These three are moved from `main.rs` verbatim except they now take `center: Vec2` instead of hardcoding `cx = 300.0, cy = 300.0` — fixing the same off-center bug already fixed on `central_swarm`:

```rust
fn central_swarm(n: usize, center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let central_mass = 20_000.0_f32;
    let light_mass = 1.0_f32;
    let min_radius = 60.0_f32;
    let max_radius = 280.0_f32;

    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject {
        position: Vec2 { x: cx, y: cy },
        velocity: Vec2::ZERO,
        mass: central_mass,
    });

    // golden-angle spread: even radial/angular coverage, no rand dependency.
    let golden_angle = TAU * 0.618_034_f32;
    for i in 0..n {
        let radius = min_radius + (max_radius - min_radius) * (i as f32 / n.max(1) as f32);
        let angle = golden_angle * i as f32;
        let dir = Vec2 { x: angle.cos(), y: angle.sin() };
        let position = Vec2 { x: cx, y: cy } + dir * radius;
        let speed = (GRAVITY * central_mass / radius).sqrt();
        let tangent = Vec2 { x: -dir.y, y: dir.x } * speed;
        objects.push(PhysicsObject { position, velocity: tangent, mass: light_mass });
    }
    objects
}

fn dual_circle(center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let m1 = 50.0_f32;
    let m2 = 20.0_f32;
    let d = 200.0_f32; // distance between bodies
    let r1 = d * m2 / (m1 + m2);
    let r2 = d * m1 / (m1 + m2);
    let v_factor = (GRAVITY / (d * (m1 + m2))).sqrt();
    let v1 = m2 * v_factor;
    let v2 = m1 * v_factor;
    let obj_a = PhysicsObject { position: Vec2 { x: cx - r1, y: cy }, mass: m1, velocity: Vec2 { x: 0.0, y: -v1 } };
    let obj_b = PhysicsObject { position: Vec2 { x: cx + r2, y: cy }, mass: m2, velocity: Vec2 { x: 0.0, y: v2 } };
    vec![obj_a, obj_b]
}

fn triangle_circle(center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let m = 20.0_f32;
    let side = 200.0_f32;
    let r = side / 3.0_f32.sqrt();

    let v = (GRAVITY * m / side).sqrt();

    let p0 = Vec2 { x: cx, y: cy - r };
    let p1 = Vec2 { x: cx - side / 2.0, y: cy + r / 2.0 };
    let p2 = Vec2 { x: cx + side / 2.0, y: cy + r / 2.0 };

    let v0 = Vec2 { x: -v, y: 0.0 };
    let v1 = Vec2 { x: v / 2.0, y: v * 3.0_f32.sqrt() / 2.0 };
    let v2 = Vec2 { x: v / 2.0, y: -v * 3.0_f32.sqrt() / 2.0 };

    let obj_a = PhysicsObject { position: p0, mass: m, velocity: v0 };
    let obj_b = PhysicsObject { position: p1, mass: m, velocity: v1 };
    let obj_c = PhysicsObject { position: p2, mass: m, velocity: v2 };
    vec![obj_a, obj_b, obj_c]
}

fn burrau_1913(center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let scale = 50.0_f32;

    let obj_a = PhysicsObject {
        position: Vec2 { x: cx + 1.0 * scale, y: cy - 3.0 * scale },
        mass: 3.0,
        velocity: Vec2::ZERO,
    };
    let obj_b = PhysicsObject {
        position: Vec2 { x: cx - 2.0 * scale, y: cy + 1.0 * scale },
        mass: 4.0,
        velocity: Vec2::ZERO,
    };
    let obj_c = PhysicsObject {
        position: Vec2 { x: cx + 1.0 * scale, y: cy + 1.0 * scale },
        mass: 5.0,
        velocity: Vec2::ZERO,
    };
    vec![obj_a, obj_b, obj_c]
}
```

- [ ] **Step 4: Add the two random scenario generators**

Append to `src/simulation.rs`:

```rust
fn random_swarm(params: &RandomSwarmParams, center: Vec2) -> Vec<PhysicsObject> {
    srand(params.seed);
    let central_mass = gen_range(params.central_mass_range.0, params.central_mass_range.1);

    let mut objects = Vec::with_capacity(params.swarm_size + 1);
    objects.push(PhysicsObject {
        position: center,
        velocity: Vec2::ZERO,
        mass: central_mass,
    });

    // keep golden-angle angular spacing for even coverage; randomize radius per body.
    let golden_angle = TAU * 0.618_034_f32;
    for i in 0..params.swarm_size {
        let radius = gen_range(params.radius_range.0, params.radius_range.1);
        let light_mass = gen_range(params.light_mass_range.0, params.light_mass_range.1);
        let angle = golden_angle * i as f32;
        let dir = Vec2 { x: angle.cos(), y: angle.sin() };
        let position = center + dir * radius;
        // derived circular-orbit speed from the randomized radius/central_mass, not a separate random draw.
        let speed = (GRAVITY * central_mass / radius).sqrt();
        let tangent = Vec2 { x: -dir.y, y: dir.x } * speed;
        objects.push(PhysicsObject { position, velocity: tangent, mass: light_mass });
    }
    objects
}

fn random_n_body(params: &RandomNBodyParams, center: Vec2) -> Vec<PhysicsObject> {
    srand(params.seed);
    let mut objects = Vec::with_capacity(params.count);
    for _ in 0..params.count {
        let mass = gen_range(params.mass_range.0, params.mass_range.1);
        let offset = Vec2 {
            x: gen_range(-params.position_spread, params.position_spread),
            y: gen_range(-params.position_spread, params.position_spread),
        };
        let speed = gen_range(params.velocity_range.0, params.velocity_range.1);
        let angle = gen_range(0.0, TAU);
        let velocity = Vec2 { x: angle.cos(), y: angle.sin() } * speed;
        objects.push(PhysicsObject { position: center + offset, velocity, mass });
    }
    objects
}

fn build_scenario(scenario: &Scenario, center: Vec2) -> Vec<PhysicsObject> {
    match scenario {
        Scenario::CentralSwarm { swarm_size } => central_swarm(*swarm_size, center),
        Scenario::DualCircle => dual_circle(center),
        Scenario::TriangleCircle => triangle_circle(center),
        Scenario::Burrau1913 => burrau_1913(center),
        Scenario::RandomSwarm(params) => random_swarm(params, center),
        Scenario::RandomNBody(params) => random_n_body(params, center),
    }
}
```

- [ ] **Step 5: Add the `Simulation` struct**

Append to `src/simulation.rs`:

```rust
pub struct Simulation {
    config: SimulationConfig,
    center: Vec2,
    world_half_size: f32,
    objects: Rc<Vec<PhysicsObject>>,
    accumulator: f32,
}

impl Simulation {
    pub fn new(config: SimulationConfig) -> Self {
        let center = vec2(config.screen_size / 2.0, config.screen_size / 2.0);
        let world_half_size = config.screen_size / 2.0;
        let objects = Rc::new(build_scenario(&config.scenario, center));
        Self { config, center, world_half_size, objects, accumulator: 0.0 }
    }

    pub fn reset(&mut self, config: SimulationConfig) {
        *self = Self::new(config);
    }

    pub fn update(&mut self, frame_time: f32) {
        self.accumulator += frame_time * self.config.time_scale;
        while self.accumulator >= self.config.physics_dt {
            self.objects = Verlet::execute(self.objects.clone(), self.config.physics_dt, self.center, self.world_half_size);
            self.accumulator -= self.config.physics_dt;
        }
    }

    pub fn objects(&self) -> &[PhysicsObject] {
        &self.objects
    }

    pub fn total_energy(&self) -> f32 {
        Physics::total_energy(&self.objects)
    }

    pub fn config(&self) -> &SimulationConfig {
        &self.config
    }
}
```

- [ ] **Step 6: Verify it compiles**

Run: `cargo build`
Expected: `0 errors`. Warnings are OK only if they already existed before this task (check against the baseline below) — no *new* warnings should appear, since every item added here is either `pub` (no dead-code lint) or reachable from `Simulation`/`build_scenario`.

Baseline (existing warnings, unrelated to this task, OK to still see): `dual_circle`, `triangle_circle`, `burrau_1913` "never used" in `main.rs` (they still live there until Task 2 removes them).

- [ ] **Step 7: Commit**

```bash
git add src/simulation.rs src/lib.rs
git commit -m "Add simulation module with config-driven scenario selection

Extracts scenario generation and the physics step loop out of
main.rs into a standalone, render-agnostic module, and adds two
seeded pseudo-random scenario kinds (RandomSwarm, RandomNBody)
alongside the four existing fixed presets."
```

---

### Task 2: Switch `main.rs` to use the `simulation` module

**Files:**
- Modify: `src/main.rs` (full rewrite of the file)

- [ ] **Step 1: Replace the contents of `src/main.rs`**

```rust
use body3_sim::simulation::{Simulation, SimulationConfig};
use macroquad::prelude::*;

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

This removes `central_swarm`, `dual_circle`, `triangle_circle`, `burrau_1913`, and the `PHYSICS_DT`/`SWARM_SIZE`/`SCREEN_SIZE`/`TIME_SCALE` consts from `main.rs` entirely — they now live in `simulation.rs`.

- [ ] **Step 2: Verify it compiles with no warnings**

Run: `cargo build`
Expected: `0 errors, 0 warnings`. This is the point where the `dual_circle`/`triangle_circle`/`burrau_1913` "never used" warnings (present since before this plan) disappear, because they're now called from `build_scenario` in `simulation.rs`.

- [ ] **Step 3: Manual smoke run**

Run: `cargo run` (let it run ~3-5 seconds, then stop it)
Expected: an 800x800 window opens, titled "Simulation"; a central mass with an orbiting swarm renders and moves (same look as before this refactor); "FPS: ..." and "Energy: ..." text appears top-left; no panic/crash in the terminal.

- [ ] **Step 4: Commit**

```bash
git add src/main.rs
git commit -m "Reduce main.rs to window setup and the render loop

All scenario/physics logic now lives in body3_sim::simulation;
main.rs only builds a SimulationConfig, owns the Simulation, and
draws it each frame."
```

---

### Task 3: Manually verify all six scenario kinds

**Files:**
- Temporarily modify: `src/main.rs` (not committed — reverted at the end of this task)

The four ported presets were exercised visually before (via the app) except `dual_circle`/`triangle_circle`/`burrau_1913`, which were dead code and have never actually been run in this app. The two random generators are brand new code. Since this project has no automated test suite (by design — see the spec's Non-goals), verify all six by eye before calling this done.

- [ ] **Step 1: Try each non-default scenario**

For each variant below, edit the `Simulation::new(...)` line in `src/main.rs` to use it instead of `SimulationConfig::default()`, then `cargo run`, watch for a couple seconds, and close the window (Esc or close button) before moving to the next:

```rust
// 1. DualCircle
let mut sim = Simulation::new(SimulationConfig { scenario: Scenario::DualCircle, ..SimulationConfig::default() });

// 2. TriangleCircle
let mut sim = Simulation::new(SimulationConfig { scenario: Scenario::TriangleCircle, ..SimulationConfig::default() });

// 3. Burrau1913
let mut sim = Simulation::new(SimulationConfig { scenario: Scenario::Burrau1913, ..SimulationConfig::default() });

// 4. RandomSwarm
let mut sim = Simulation::new(SimulationConfig { scenario: Scenario::RandomSwarm(Default::default()), ..SimulationConfig::default() });

// 5. RandomNBody
let mut sim = Simulation::new(SimulationConfig { scenario: Scenario::RandomNBody(Default::default()), ..SimulationConfig::default() });
```

(This requires adding `Scenario` to the `use body3_sim::simulation::{...}` import line temporarily — remember to remove it in Step 3.)

Expected for each:
- No panic/crash in the terminal.
- Bodies appear inside the 800x800 window (not all clustered off-screen or all stacked at one point).
- `DualCircle`/`TriangleCircle`: bodies orbit smoothly, roughly centered in the window (this is the first real run of these two — they were unreachable dead code before).
- `Burrau1913`: three bodies start motionless and begin accelerating toward each other (no initial velocity, by design).
- `RandomSwarm`: looks like a variant of the default swarm — one central body, satellites in golden-angle-spaced orbits at varied radii.
- `RandomNBody`: `count` bodies scattered around the center with visibly different masses (via draw radius if you want a quick visual proxy — not required, `count`/positions/velocities being non-degenerate is the main thing to confirm).

- [ ] **Step 2: Re-run the default scenario**

Revert `src/main.rs` to `Simulation::new(SimulationConfig::default())`, `cargo run`, confirm it's back to the `CentralSwarm` look from Task 2's smoke run.

- [ ] **Step 3: Confirm no leftover diff**

Run: `git status`
Expected: `src/main.rs` matches the version committed in Task 2 (no diff) — this task is verification-only, nothing new to commit. If `git diff src/main.rs` shows anything, revert it (`git checkout -- src/main.rs`) since this task's edits were scratch/throwaway.
