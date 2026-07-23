# Swarm Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the simulation from injecting spurious energy and flying apart, by deriving the integration parameters from the symplectic stability criterion, spawning swarms on orbits consistent with the force field they actually feel, and keeping every body inside the quadtree root.

**Architecture:** Three independent changes, in dependency order. (1) `softening` and `physics_dt` stop being global constants and become **derived** per scenario from `softening >= (dt^2 * G * m_dominant)^(1/3)`, with `dt` chosen by body count (cost) since the criterion leaves exactly one free variable. (2) Swarm orbiters get their circular speed from the **measured** acceleration rather than from the core mass alone — this is what makes a large softening self-consistent, so it must land after (1). (3) The quadtree root is refitted to the bodies' bounding box each substep, so `Quadrant::insert` can never misfile an escaper into a corner quadrant.

**Tech Stack:** Rust 2024, macroquad 0.4 (`Vec2` math only in the library), criterion for benches. No new dependencies.

---

## Background: why this order

Measured on `CentralSwarm`, 10 simulated seconds, quadtree root refitted so misfiling cannot confound (`examples/stability_sweep.rs`):

| change | n=1000 energy drift | n=8000 energy drift |
|---|---|---|
| baseline (`softening=1.0`) | +54.35% | +63457% |
| corrected softening (44.2) | **+0.64%** | +7.51% |
| corrected softening + fix C | +0.07% | **-0.54%** |

`theta` is exonerated — *smaller* theta makes drift worse (1.8 → +54%, 0.2 → +83%), so the injection is not a Barnes-Hut accuracy problem. `dt` alone is non-monotone (chaos, not truncation error). Softening is the controlling knob.

Removing net spawn momentum was measured and **deliberately excluded**: `|v_com|` is 0.029–0.082, i.e. 0.014–0.16% of `world_half_size` over a full run, and the already-shipped `CameraFit` centers on the center of mass anyway. It is noise at this scale.

### The defect being fixed

[src/physics.rs:11-20](../../../src/physics.rs#L11-L20) derives the softening floor from `sqrt(softening^3 / (GRAVITY*m))` evaluated at **m = 1.0** (the light mass). Its stated numbers confirm this: `softening=0.001 -> ~1e-7` is `sqrt(1e-9/1e5)`, and `softening=1.0 -> ~3e-3` is `sqrt(1/1e5)`.

But every orbiter's binding encounter is with the **core mass, 20000**:

```
sqrt(1^3 / (1e5 * 2e4)) = 2.2e-5   vs   dt = 0.005     ->  220x below the criterion
```

The existing validation (`|growth| ~0.9x over T=0.3s`) is 60 steps; the divergence needs ~2000 to appear.

---

## File Structure

| File | Responsibility | Action |
|---|---|---|
| `src/physics.rs` | `min_softening(dt, mass)` — the criterion, one place | Modify |
| `src/simulation.rs` | Per-scenario `dominant_mass`, `body_count`, `integration_params`; swarm circularization | Modify |
| `src/quadtree.rs` | `fitting_root(objects)` — a root that provably contains every body | Modify |
| `src/main.rs` | Recompute derived params on scenario change; show softening | Modify |
| `tests/stability_regression.rs` | Energy-drift regression, the acceptance test for the whole plan | Create |
| `tests/integration_params.rs` | The criterion and per-scenario derivation | Create |
| `tests/quadtree_bounds.rs` | Out-of-root bodies produce correct forces | Create |
| `tests/verlet_cache_regression.rs` | Pinned trajectory — must be re-pinned after Task 4 | Modify |

`examples/escape_diagnostic.rs` and `examples/stability_sweep.rs` already exist and stay as-is; they are the measurement tools that produced the numbers above.

---

### Task 1: Energy-drift regression test (fails on purpose)

Establishes the acceptance criterion before any fix. This test must **fail** at the end of this task and stay in the tree.

**Files:**
- Create: `tests/stability_regression.rs`

- [ ] **Step 1: Write the failing test**

```rust
// Acceptance test for the swarm-stability work. A fixed-dt Verlet integrator
// is symplectic: over a bounded run the total energy must stay put. It does
// not today (see docs/superpowers/plans/2026-07-23-swarm-stability.md), and
// that drift is what makes swarms fly off-screen.
//
// Kept deliberately small so it runs in a debug `cargo test`; the full sweeps
// live in examples/stability_sweep.rs.
use body3_sim::physics::Physics;
use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

const STEPS: usize = 600;
const MAX_DRIFT_PERCENT: f32 = 5.0;

fn energy_drift_percent(n: usize, steps: usize) -> f32 {
    let config = SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: n },
        time_scale: 1.0,
        ..Default::default()
    };
    let mut sim = Simulation::new(config);
    let e0 = Physics::total_energy(sim.objects(), sim.config().softening);
    for _ in 0..steps {
        sim.update(sim.config().physics_dt);
    }
    let e = Physics::total_energy(sim.objects(), sim.config().softening);
    100.0 * (e - e0) / e0.abs()
}

#[test]
fn central_swarm_conserves_energy() {
    let drift = energy_drift_percent(300, STEPS);
    assert!(
        drift.abs() < MAX_DRIFT_PERCENT,
        "energy drift {drift:+.2}% exceeds +-{MAX_DRIFT_PERCENT}%"
    );
}

#[test]
fn central_swarm_stays_near_its_spawn_extent() {
    // Physical relaxation does expand a 2D cluster, so this is a loose bound:
    // it catches "blew up by orders of magnitude", not gentle spreading.
    let config = SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: 300 },
        time_scale: 1.0,
        ..Default::default()
    };
    let mut sim = Simulation::new(config);
    let center = macroquad::math::vec2(config.screen_size / 2.0, config.screen_size / 2.0);
    let spawn_max = body3_sim::simulation::central_swarm_radii(300).1;
    for _ in 0..STEPS {
        sim.update(sim.config().physics_dt);
    }
    let mut radii: Vec<f32> = sim
        .objects()
        .iter()
        .map(|o| (o.position - center).length())
        .collect();
    radii.sort_by(f32::total_cmp);
    let p98 = radii[((radii.len() - 1) as f32 * 0.98) as usize];
    assert!(
        p98 < spawn_max * 4.0,
        "p98 radius {p98:.0} vs spawn max {spawn_max:.0} (limit {:.0})",
        spawn_max * 4.0
    );
}
```

- [ ] **Step 2: Run it and confirm it fails**

Run: `cargo test --test stability_regression`
Expected: both tests FAIL, with a drift well above 5%.

- [ ] **Step 3: Check the runtime is acceptable in debug**

Run: `cargo test --test stability_regression -- --nocapture`
If the run exceeds ~15 seconds, lower `STEPS` to 400 or `swarm_size` to 200 and re-run. Record the final numbers in the test's comment. Do **not** switch the test to release-only — it must run in the default `cargo test`.

- [ ] **Step 4: Commit**

```bash
git add tests/stability_regression.rs
git commit -m "test: add failing energy-drift regression for swarm stability"
```

---

### Task 2: The stability criterion as a function

**Files:**
- Modify: `src/physics.rs:11-26`
- Create: `tests/integration_params.rs`

- [ ] **Step 1: Write the failing test**

```rust
// tests/integration_params.rs
use body3_sim::physics::{min_softening, GRAVITY};

#[test]
fn min_softening_inverts_the_encounter_timescale_criterion() {
    // The criterion is sqrt(softening^3 / (G*m)) >= dt. min_softening returns
    // the softening at which that holds with equality, so feeding its output
    // back through the timescale must reproduce dt.
    for &(dt, mass) in &[(0.005f32, 20_000.0f32), (1e-4, 40_000.0), (0.002, 5.0)] {
        let soft = min_softening(dt, mass);
        let timescale = (soft.powi(3) / (GRAVITY * mass)).sqrt();
        assert!(
            (timescale - dt).abs() / dt < 1e-3,
            "dt={dt} mass={mass}: timescale={timescale} != dt"
        );
    }
}

#[test]
fn min_softening_matches_the_measured_swarm_value() {
    // The value the sweep in examples/stability_sweep.rs found to work for
    // CentralSwarm at the production dt: predicted 36.8, first clean sweep
    // point 32, verified good at 44.
    let soft = min_softening(0.005, 20_000.0);
    assert!((soft - 36.84).abs() < 0.1, "got {soft}");
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --test integration_params`
Expected: compile error, `cannot find function min_softening`.

- [ ] **Step 3: Implement `min_softening` and correct the stale comment**

In `src/physics.rs`, replace the `DEFAULT_SOFTENING` comment block (lines 9-20) and the constant with:

```rust
// Plummer softening replaces the bare 1/r^2 singularity with
// 1/(r^2 + softening^2), capping the peak close-encounter force at
// ~GRAVITY*m/softening^2 and the potential-well depth at ~GRAVITY*m/softening.
// That bounds the smallest resolvable encounter timescale to
// ~sqrt(softening^3 / (GRAVITY*m)); fixed-dt Verlet stays symplectic (energy
// conserved) only while that timescale >= dt.
//
// The mass in that expression is the mass of the body being encountered, and
// the binding encounter for every swarm orbiter is with the CORE (20000), not
// with another light body (1.0). Evaluating the criterion at the light mass —
// as this file previously did — understates the required softening by ~220x
// at the production dt, and the resulting energy injection is what made
// swarms fly apart (+54% energy over 10 simulated seconds at n=1000; see
// examples/stability_sweep.rs). Callers derive their softening from
// `min_softening` via `simulation::integration_params` instead of hardcoding.
pub fn min_softening(dt: f32, mass: f32) -> f32 {
    (dt * dt * GRAVITY * mass).cbrt()
}

// Fallback for callers without a scenario handy (benches, examples that pin a
// value deliberately). NOT the production value — production derives it.
pub const DEFAULT_SOFTENING: f32 = 1.0;
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cargo test --test integration_params`
Expected: PASS, 2 tests.

- [ ] **Step 5: Confirm nothing else broke**

Run: `cargo test`
Expected: all green — `DEFAULT_SOFTENING` still exists with the same value, so no existing test changes behavior yet.

- [ ] **Step 6: Commit**

```bash
git add src/physics.rs tests/integration_params.rs
git commit -m "feat: add min_softening stability criterion, fix its derivation"
```

---

### Task 3: Derive `physics_dt` and `softening` per scenario

The criterion ties `dt` and `softening` together, leaving one free variable. Choose `dt` by **cost** — few-body scenarios can afford a tiny timestep, swarms cannot — then derive `softening` from it.

**Files:**
- Modify: `src/simulation.rs`
- Modify: `tests/integration_params.rs`

- [ ] **Step 1: Write the failing tests**

Append to `tests/integration_params.rs`:

```rust
use body3_sim::simulation::{
    body_count, dominant_mass, integration_params, Scenario, Simulation, SimulationConfig,
};

fn all_scenarios() -> Vec<Scenario> {
    use body3_sim::simulation::{RandomNBodyParams, RandomSwarmParams};
    vec![
        Scenario::CentralSwarm { swarm_size: 1000 },
        Scenario::DualCircle,
        Scenario::TriangleCircle,
        Scenario::Burrau1913,
        Scenario::SolarSystem,
        Scenario::FigureEight,
        Scenario::Circumbinary,
        Scenario::Trojan,
        Scenario::Slingshot,
        Scenario::GalaxyCollision { swarm_size: 2000 },
        Scenario::RandomSwarm(RandomSwarmParams::default()),
        Scenario::RandomNBody(RandomNBodyParams::default()),
    ]
}

#[test]
fn body_count_matches_what_the_scenario_actually_builds() {
    // Keeps the hand-written counts honest: integration_params picks the
    // timestep from body_count, so a wrong count silently mis-tunes physics.
    for scenario in all_scenarios() {
        let sim = Simulation::new(SimulationConfig { scenario, ..Default::default() });
        assert_eq!(
            body_count(&scenario),
            sim.objects().len(),
            "body_count disagrees with build_scenario for {scenario:?}"
        );
    }
}

#[test]
fn dominant_mass_is_the_heaviest_body_present() {
    for scenario in all_scenarios() {
        let sim = Simulation::new(SimulationConfig { scenario, ..Default::default() });
        let heaviest = sim.objects().iter().map(|o| o.mass).fold(0.0f32, f32::max);
        let claimed = dominant_mass(&scenario);
        assert!(
            claimed >= heaviest * 0.999,
            "{scenario:?}: dominant_mass {claimed} < heaviest body {heaviest}"
        );
    }
}

#[test]
fn every_scenario_satisfies_the_criterion() {
    for scenario in all_scenarios() {
        let (dt, softening) = integration_params(&scenario);
        let required = body3_sim::physics::min_softening(dt, dominant_mass(&scenario));
        assert!(
            softening >= required,
            "{scenario:?}: softening {softening} below required {required} at dt {dt}"
        );
    }
}

#[test]
fn defaults_come_from_the_scenario() {
    let config = SimulationConfig::default();
    let (dt, softening) = integration_params(&config.scenario);
    assert_eq!(config.physics_dt, dt);
    assert_eq!(config.softening, softening);
    // The measured-good swarm value, not the old 1.0.
    assert!(config.softening > 30.0, "got {}", config.softening);
}

#[test]
fn few_body_presets_keep_softening_small_relative_to_their_geometry() {
    // A softening comparable to the orbital radii would erase exactly the
    // close-encounter physics these presets exist to show.
    for (scenario, smallest_length) in [
        (Scenario::SolarSystem, 90.0f32),
        (Scenario::Slingshot, 60.0),
        (Scenario::Trojan, 250.0),
    ] {
        let (_, softening) = integration_params(&scenario);
        assert!(
            softening < smallest_length * 0.1,
            "{scenario:?}: softening {softening} is >10% of {smallest_length}"
        );
    }
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --test integration_params`
Expected: compile error — `body_count`, `dominant_mass`, `integration_params` do not exist.

- [ ] **Step 3: Extract the swarm mass literals into constants**

In `src/simulation.rs`, `central_swarm_at` currently hardcodes its masses. Replace the two `let` bindings at the top of the function with references to new module-level constants, declared next to `CENTRAL_SWARM_MIN_RADIUS`:

```rust
// Core and orbiter masses for CentralSwarm / GalaxyCollision. Named because
// integration_params needs the core mass to derive the softening.
pub const CENTRAL_SWARM_CORE_MASS: f32 = 20_000.0;
pub const CENTRAL_SWARM_LIGHT_MASS: f32 = 1.0;
```

and inside `central_swarm_at`:

```rust
    let central_mass = CENTRAL_SWARM_CORE_MASS;
    let light_mass = CENTRAL_SWARM_LIGHT_MASS;
```

- [ ] **Step 4: Add the derivation**

Add to `src/simulation.rs`, after `central_swarm_radii`:

```rust
// The symplectic criterion (physics::min_softening) ties dt and softening
// together, leaving exactly one free choice. Pick dt by COST: the force
// evaluation is trivial at a handful of bodies, so few-body presets can afford
// a tiny timestep and therefore a small, geometry-preserving softening; swarms
// cannot, so they take a large dt and pay for it with a large softening. That
// large softening is only self-consistent because swarm orbits are built from
// the measured force field (see `circularize`), not from an analytic Kepler
// speed that ignores it.
const FEW_BODY_MAX: usize = 100;
const FEW_BODY_DT: f32 = 1.0e-4;
const SWARM_DT: f32 = 0.005;
// Margin above the criterion's equality point. The sweep in
// examples/stability_sweep.rs is clean from ~32 upward at dt=0.005 where the
// criterion predicts 36.8, so a modest margin is enough.
const SOFTENING_SAFETY: f32 = 1.2;

/// Number of bodies `build_scenario` will produce. Drives the timestep choice.
pub fn body_count(scenario: &Scenario) -> usize {
    match scenario {
        Scenario::CentralSwarm { swarm_size } => swarm_size + 1,
        Scenario::DualCircle => 2,
        Scenario::TriangleCircle => 3,
        Scenario::Burrau1913 => 3,
        Scenario::SolarSystem => SOLAR_PLANETS.len() + 1,
        Scenario::FigureEight => 3,
        Scenario::Circumbinary => CIRCUMBINARY_PLANETS.len() + 2,
        Scenario::Trojan => 2 * TROJAN_COUNT_PER_POINT + 2,
        Scenario::Slingshot => SLINGSHOT_IMPACT_PARAMS.len() + 1,
        // galaxy_collision splits `swarm_size` in two and adds a core to each.
        Scenario::GalaxyCollision { swarm_size } => swarm_size + 2,
        Scenario::RandomSwarm(p) => p.swarm_size + 1,
        Scenario::RandomNBody(p) => p.count,
    }
}

/// Mass of the heaviest body in the scenario — the one that sets the shortest
/// encounter timescale, and therefore the softening floor.
pub fn dominant_mass(scenario: &Scenario) -> f32 {
    match scenario {
        Scenario::CentralSwarm { .. } | Scenario::GalaxyCollision { .. } => CENTRAL_SWARM_CORE_MASS,
        Scenario::DualCircle => 50.0,
        Scenario::TriangleCircle => 20.0,
        Scenario::Burrau1913 => 5.0,
        Scenario::SolarSystem => SOLAR_SUN_MASS,
        Scenario::FigureEight => FIG8_MASS,
        Scenario::Circumbinary => CIRCUMBINARY_STAR_A_MASS.max(CIRCUMBINARY_STAR_B_MASS),
        Scenario::Trojan => TROJAN_SUN_MASS,
        Scenario::Slingshot => SLINGSHOT_PLANET_MASS,
        Scenario::RandomSwarm(p) => p.central_mass_range.1,
        Scenario::RandomNBody(p) => p.mass_range.1,
    }
}

/// `(physics_dt, softening)` for a scenario, satisfying the stability
/// criterion by construction.
pub fn integration_params(scenario: &Scenario) -> (f32, f32) {
    let dt = if body_count(scenario) <= FEW_BODY_MAX { FEW_BODY_DT } else { SWARM_DT };
    (dt, crate::physics::min_softening(dt, dominant_mass(scenario)) * SOFTENING_SAFETY)
}
```

- [ ] **Step 5: Make the defaults derived**

Replace `SimulationConfig::default`'s body in `src/simulation.rs`:

```rust
impl Default for SimulationConfig {
    fn default() -> Self {
        Self::for_scenario(Scenario::CentralSwarm { swarm_size: 1000 })
    }
}

impl SimulationConfig {
    /// Config for a scenario with dt/softening derived from the stability
    /// criterion. Use this rather than mutating `scenario` on an existing
    /// config, which would leave the previous scenario's physics parameters in
    /// place.
    pub fn for_scenario(scenario: Scenario) -> Self {
        let (physics_dt, softening) = integration_params(&scenario);
        Self {
            scenario,
            screen_size: 1000.0,
            physics_dt,
            time_scale: 0.3,
            theta_threshold: crate::physics::DEFAULT_TETHA_THRESHOLD,
            softening,
        }
    }
}
```

- [ ] **Step 6: Run the new tests**

Run: `cargo test --test integration_params`
Expected: PASS, 7 tests.

- [ ] **Step 7: Run the regression test — it should now largely pass**

Run: `cargo test --test stability_regression`
Expected: `central_swarm_conserves_energy` PASSES. If it does not, stop and check `dominant_mass` for `CentralSwarm` — the sweep says drift should land near +0.6% at this softening.

- [ ] **Step 8: Fix the fallout in the rest of the suite**

Run: `cargo test`

Expected failures and the correct response for each:
- `tests/spawn_density.rs::default_spawn_is_pixel_compatible_with_before` asserts `world_half_size() == 400.0` using `screen_size: 800.0` from its own config — unaffected, must still pass.
- Any test constructing `SimulationConfig { .. ..Default::default() }` and overriding `scenario` now inherits the **default scenario's** dt/softening. Update those to `SimulationConfig::for_scenario(scenario)` plus field overrides. Check `tests/theta_config.rs`, `tests/energy_approx_accuracy.rs`, `tests/galaxy_collision.rs`, `tests/circumbinary_trojan_slingshot.rs`, `tests/solar_figure8.rs`.
- `tests/verlet_cache_regression.rs` pins `softening: 1.0` explicitly in its own config — it must keep doing so and must still pass unchanged. If it fails here, something other than the defaults changed; investigate before re-pinning.

- [ ] **Step 9: Commit**

```bash
git add src/simulation.rs tests/integration_params.rs tests/
git commit -m "feat: derive physics_dt and softening per scenario from stability criterion"
```

---

### Task 4: Wire the derived parameters into the UI

Without this, changing scenario in the sidebar keeps the previous scenario's physics parameters and silently breaks the criterion.

**Files:**
- Modify: `src/main.rs:190-274` (`draw_panel`)

**Design constraint:** the derived pair must be recomputed **only when the scenario or one of its parameters actually changes**. Recomputing every frame would clobber the `physics_dt` slider one frame after the user drags it, silently making the override useless.

- [ ] **Step 1: Track whether the scenario changed this frame**

At the top of the `show` closure in `draw_panel`, before the `ComboBox`:

```rust
            // Set by anything that feeds integration_params. Only then is the
            // derived (dt, softening) pair recomputed — doing it every frame
            // would overwrite the physics_dt override slider below.
            let mut scenario_changed = false;
```

In the `ComboBox`, preserve the user's view/pacing settings rather than resetting the whole config:

```rust
                        if ui.selectable_label(kind == current_kind, kind.label()).clicked() && kind != current_kind {
                            pending.scenario = kind.default_scenario();
                            scenario_changed = true;
                        }
```

- [ ] **Step 2: Flag the scenario-parameter sliders**

Every slider inside `match &mut pending.scenario { .. }` that feeds `body_count` or `dominant_mass` must set the flag. Those are: both `swarm_size` sliders, `RandomSwarm`'s `central mass max`, `RandomNBody`'s `count` and `mass max`. Wrap each with its `Response`:

```rust
                Scenario::CentralSwarm { swarm_size } => {
                    scenario_changed |= ui
                        .add(egui::Slider::new(swarm_size, CENTRAL_SWARM_SIZE_RANGE).text("swarm_size"))
                        .changed();
                }
```

Apply the same `scenario_changed |= ...changed();` shape to the other four. The remaining sliders (radii, light masses, seeds) do not affect the criterion and can stay as they are.

- [ ] **Step 3: Re-derive only on change, and show the result**

After the `match` block:

```rust
            if scenario_changed {
                let (dt, softening) = body3_sim::simulation::integration_params(&pending.scenario);
                pending.physics_dt = dt;
                pending.softening = softening;
            }

            ui.separator();
            ui.label(format!("softening: {:.2}  (derived)", pending.softening));
```

- [ ] **Step 4: Label the dt slider as the override it now is**

```rust
            if ui.add(egui::Slider::new(&mut pending.physics_dt, PHYSICS_DT_RANGE).text("physics_dt (override)")).changed() {
                sim.set_physics_dt(pending.physics_dt);
            }
```

Note that a manual `physics_dt` can break the criterion on purpose — that is the point of an override, and `examples/stability_sweep.rs` is the tool for exploring it. It is reset the next time the scenario changes.

- [ ] **Step 4: Verify it builds and behaves**

Run: `cargo build --release`
Run: `cargo run --release`
Check by hand: switch scenario to Slingshot and back to Central Swarm; the softening label should read ~3.3 for Slingshot and ~44 for Central Swarm.

- [ ] **Step 5: Commit**

```bash
git add src/main.rs
git commit -m "feat: re-derive dt and softening when the scenario or its parameters change"
```

---

### Task 5: Fix C — swarm orbits from the measured force field

**Files:**
- Modify: `src/simulation.rs` (`central_swarm_at`, `random_swarm`, `build_scenario`, `Simulation::new`)
- Modify: `tests/stability_regression.rs`
- Modify: `tests/verlet_cache_regression.rs` (re-pin)

- [ ] **Step 1: Write the failing test**

Append to `tests/stability_regression.rs`:

```rust
use body3_sim::physics::Physics;

#[test]
fn swarm_orbiters_are_circular_in_the_actual_force_field() {
    // v^2 / r must equal the radial acceleration the body really feels.
    // Deriving the speed from the core mass alone ignores the swarm's own
    // mass, which at large n exceeds the core.
    let config = SimulationConfig::for_scenario(Scenario::CentralSwarm { swarm_size: 2000 });
    let sim = Simulation::new(config);
    let center = macroquad::math::vec2(config.screen_size / 2.0, config.screen_size / 2.0);
    let acc = Physics::compute_accelerations(
        sim.objects(),
        center,
        sim.world_half_size(),
        config.theta_threshold,
        config.softening,
    );

    let mut worst = 0.0f32;
    for (obj, a) in sim.objects().iter().zip(acc.iter()).skip(1) {
        let d = obj.position - center;
        let r = d.length();
        let a_radial = -a.dot(d / r);
        if a_radial <= 0.0 {
            continue;
        }
        let expected = (a_radial * r).sqrt();
        worst = worst.max((obj.velocity.length() - expected).abs() / expected);
    }
    assert!(worst < 0.02, "worst relative speed error {:.1}%", worst * 100.0);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --test stability_regression swarm_orbiters`
Expected: FAIL with a worst error of roughly 5-10% at n=2000 (it grows with n).

- [ ] **Step 3: Implement circularization**

Add to `src/simulation.rs`:

```rust
// Sets each orbiter's speed to the circular speed for the acceleration it
// ACTUALLY feels, rather than the analytic sqrt(G*M_core/r), which ignores
// both the swarm's own mass and the softening. Two reasons this is the
// measured form rather than an enclosed-mass estimate:
//
//   - the shell theorem does not hold in 2D with a 1/r^2 force (arc length
//     grows as r, force falls as 1/r^2, so the near arc wins and exterior
//     matter pulls outward instead of cancelling), so an enclosed-mass sum
//     would systematically overestimate the required speed
//   - it needs no radius-ordered indices, so it works unchanged for
//     random_swarm, whose radii are drawn at random
//
// `objects[0]` is the core and is skipped. Velocities are ignored by the force
// evaluation, so this may be called before or after they are set.
fn circularize(objects: &mut [PhysicsObject], center: Vec2, theta: f32, softening: f32) {
    let (root_center, half_size) = crate::quadtree::fitting_root(objects);
    let acc = Physics::compute_accelerations(objects, root_center, half_size, theta, softening);
    for (obj, a) in objects.iter_mut().zip(acc.iter()).skip(1) {
        let d = obj.position - center;
        let r = d.length();
        if r <= 0.0 {
            continue;
        }
        let dir = d / r;
        let a_radial = -a.dot(dir); // positive => net pull toward the center
        if a_radial <= 0.0 {
            continue; // net outward: no circular orbit exists here
        }
        obj.velocity = vec2(-dir.y, dir.x) * (a_radial * r).sqrt();
    }
}
```

> **Depends on `quadtree::fitting_root` from Task 6.** If executing tasks in order, either implement Task 6 Step 3 first, or temporarily pass `(center, Simulation::world_extent(scenario, screen_size))` here and switch after Task 6. Prefer implementing Task 6 first; the plan orders C before D only because C's stability payoff is larger.

- [ ] **Step 4: Call it from the swarm builders**

`central_swarm_at` builds positions and analytic velocities, then applies `bulk`. Restructure so the analytic speed is gone and the bulk is applied after circularization:

```rust
fn central_swarm_at(n: usize, center: Vec2, bulk: Vec2, theta: f32, softening: f32) -> Vec<PhysicsObject> {
    let (min_radius, max_radius) = central_swarm_radii(n);

    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject {
        position: center,
        velocity: Vec2::ZERO,
        mass: CENTRAL_SWARM_CORE_MASS,
    });

    let golden_angle = TAU * 0.618_034_f32;
    for i in 0..n {
        let radius = min_radius + (max_radius - min_radius) * (i as f32 / n.max(1) as f32);
        let angle = golden_angle * i as f32;
        let dir = Vec2 { x: angle.cos(), y: angle.sin() };
        objects.push(PhysicsObject {
            position: center + dir * radius,
            velocity: Vec2::ZERO,
            mass: CENTRAL_SWARM_LIGHT_MASS,
        });
    }

    // Circularize about this swarm's own center BEFORE the bulk boost, so a
    // GalaxyCollision core is circularized against its own swarm only and the
    // rigid translation leaves the internal orbits untouched.
    circularize(&mut objects, center, theta, softening);
    for obj in objects.iter_mut() {
        obj.velocity += bulk;
    }
    objects
}
```

Apply the same treatment to `random_swarm`: drop the `let speed = (GRAVITY * central_mass / radius).sqrt();` line and its tangent, spawn with `Vec2::ZERO`, then call `circularize(&mut objects, center, theta, softening)` before returning.

- [ ] **Step 5: Thread theta and softening to the builders**

`build_scenario` needs them. Change its signature and the `Simulation::new` call site:

```rust
fn build_scenario(scenario: &Scenario, center: Vec2, theta: f32, softening: f32) -> Vec<PhysicsObject> {
```

Only the three swarm arms use the new parameters; prefix the others' unused bindings as needed. In `Simulation::new`:

```rust
        let objects = Rc::new(build_scenario(
            &config.scenario,
            center,
            config.theta_threshold,
            config.softening,
        ));
```

`galaxy_collision` gains the same two parameters and forwards them to both `central_swarm_at` calls.

- [ ] **Step 6: Run the new test**

Run: `cargo test --test stability_regression`
Expected: all three tests PASS.

- [ ] **Step 7: Re-pin the trajectory regression**

`tests/verlet_cache_regression.rs` pins exact positions and velocities for a CentralSwarm, and its header comment already says to re-pin when the spawn changes. Spawn velocities have now changed by design.

Print the new expectations (the test already has a printing path at line 48), verify the values are finite and of a sane magnitude, paste them into `EXPECTED`, and update the comment at lines 18-22 to say the pin now also depends on `circularize`.

Run: `cargo test --test verlet_cache_regression`
Expected: PASS.

- [ ] **Step 8: Confirm the few-body presets are untouched**

`circularize` is only called from the swarm builders, so the analytic Kepler assertions in `tests/solar_figure8.rs:29-32` and `tests/circumbinary_trojan_slingshot.rs:52` must still hold.

Run: `cargo test`
Expected: all green.

- [ ] **Step 9: Commit**

```bash
git add src/simulation.rs tests/
git commit -m "feat: build swarm orbits from the measured force field"
```

---

### Task 6: Fix D — a quadtree root that always contains every body

`Quadrant::insert` has no bounds check, so a body outside the root is filed into whichever corner quadrant `find_quadrant` picks. Its node's `center_of_mass` and `half_size` then misdescribe it, and the opening-angle test makes wrong descend/skip decisions **for every body**, not just the escaper.

**Files:**
- Modify: `src/quadtree.rs`
- Modify: `src/simulation.rs` (`Simulation::update`)
- Modify: `src/physics.rs:66-71` (the cache-reuse comment)
- Create: `tests/quadtree_bounds.rs`

- [ ] **Step 1: Write the failing test**

```rust
// tests/quadtree_bounds.rs
use body3_sim::physics::{Physics, PhysicsObject, DEFAULT_SOFTENING, DEFAULT_TETHA_THRESHOLD};
use body3_sim::quadtree::fitting_root;
use macroquad::math::{vec2, Vec2};

fn body(x: f32, y: f32, mass: f32) -> PhysicsObject {
    PhysicsObject { position: vec2(x, y), velocity: Vec2::ZERO, mass }
}

#[test]
fn fitting_root_contains_every_body() {
    let objects = vec![
        body(-5_000.0, 12.0, 1.0),
        body(3.0, 40_000.0, 1.0),
        body(100.0, 100.0, 20_000.0),
    ];
    let (center, half) = fitting_root(&objects);
    for o in &objects {
        assert!(
            (o.position.x - center.x).abs() <= half && (o.position.y - center.y).abs() <= half,
            "body at {:?} outside root center={center:?} half={half}",
            o.position
        );
    }
}

#[test]
fn fitting_root_survives_degenerate_input() {
    // All bodies coincident: extent is zero, and a zero half-size would make
    // every subdivision degenerate.
    let objects = vec![body(7.0, 7.0, 1.0), body(7.0, 7.0, 1.0)];
    let (_, half) = fitting_root(&objects);
    assert!(half > 0.0 && half.is_finite(), "half={half}");

    let (_, half) = fitting_root(&[]);
    assert!(half > 0.0 && half.is_finite(), "empty: half={half}");
}

#[test]
fn a_body_outside_the_static_root_gets_the_right_force() {
    // The regression: with a root that does not contain it, an escaper is
    // misfiled and the resulting accelerations are wrong for everyone.
    // A fitted root must reproduce the exact all-pairs answer closely.
    let mut objects = vec![body(0.0, 0.0, 20_000.0)];
    for i in 0..8 {
        let a = i as f32;
        objects.push(body(50.0 + a * 10.0, 20.0 - a * 5.0, 1.0));
    }
    objects.push(body(9_000.0, -9_000.0, 1.0)); // the escaper

    let (center, half) = fitting_root(&objects);
    let fitted = Physics::compute_accelerations(&objects, center, half, DEFAULT_TETHA_THRESHOLD, DEFAULT_SOFTENING);

    // theta = 0 forces the walk to descend to leaves everywhere: exact forces.
    let exact = Physics::compute_accelerations(&objects, center, half, 0.0, DEFAULT_SOFTENING);

    for (i, (f, e)) in fitted.iter().zip(exact.iter()).enumerate() {
        let err = (*f - *e).length() / e.length().max(1e-6);
        assert!(err < 0.05, "body {i}: relative force error {:.1}%", err * 100.0);
    }
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --test quadtree_bounds`
Expected: compile error, `fitting_root` not found.

- [ ] **Step 3: Implement `fitting_root`**

Add to `src/quadtree.rs`:

```rust
// Root center and half-size that provably contain every body. `insert` has no
// bounds check by design (it is the hot path), so the root must be correct by
// construction: a body outside it is filed into a corner quadrant, and the
// node summaries it lands in then misdescribe its position for every
// opening-angle test in the walk.
//
// Non-finite positions are skipped rather than propagated — an infinite root
// would collapse the tree to a single leaf and quietly turn the walk into
// O(n^2). The floor keeps the half-size positive for coincident or empty
// input, where the extent is zero.
pub fn fitting_root(objects: &[PhysicsObject]) -> (Vec2, f32) {
    const MARGIN: f32 = 1.01;
    const MIN_HALF_SIZE: f32 = 1.0;

    let mut lo = vec2(f32::INFINITY, f32::INFINITY);
    let mut hi = vec2(f32::NEG_INFINITY, f32::NEG_INFINITY);
    for o in objects {
        if o.position.is_finite() {
            lo = lo.min(o.position);
            hi = hi.max(o.position);
        }
    }
    if !lo.is_finite() || !hi.is_finite() {
        return (Vec2::ZERO, MIN_HALF_SIZE);
    }
    let center = (lo + hi) * 0.5;
    let half = ((hi - lo).max_element() * 0.5 * MARGIN).max(MIN_HALF_SIZE);
    (center, half)
}
```

- [ ] **Step 4: Add a debug-only bounds check to catch regressions**

In `Quadtree::build`, inside the insertion loop:

```rust
            debug_assert!(
                (obj.position.x - center.x).abs() <= half_size
                    && (obj.position.y - center.y).abs() <= half_size,
                "body at {:?} outside root center={center:?} half_size={half_size}; \
                 build the root with fitting_root",
                obj.position
            );
```

- [ ] **Step 5: Run the tests**

Run: `cargo test --test quadtree_bounds`
Expected: PASS, 3 tests.

- [ ] **Step 6: Use the fitted root in the simulation loop**

In `Simulation::update`, replace the static `self.center` / `self.world_half_size` arguments:

```rust
        while self.accumulator >= self.config.physics_dt {
            // Refit per substep: bodies move, and a body outside the root is
            // silently misfiled (see quadtree::fitting_root).
            let (root_center, root_half) = crate::quadtree::fitting_root(&self.objects);
            let (objects, acc_new) = Verlet::execute_cached(
                self.objects.clone(),
                self.config.physics_dt,
                root_center,
                root_half,
                self.config.theta_threshold,
                self.config.softening,
                self.cached_acceleration.as_deref(),
            );
```

`world_half_size` stays: `CameraFit` uses it as the zoom-in floor, and `Simulation::world_extent` remains the documented spawn extent.

- [ ] **Step 7: Correct the cache-reuse comment**

`src/physics.rs:66-71` claims the cached acceleration reuse is "exact, not an approximation". That held when the root was fixed. With a refitted root, the previous substep's `acc_new` was evaluated on a slightly different tree. Positions are still identical, so it remains a valid Barnes-Hut evaluation at the correct positions, but "exact" is now too strong. Amend it:

```rust
    // Same integration as `execute`, but accepts the previous step's acc_new
    // as this step's acc_old instead of recomputing it. Force only depends on
    // position, and acc_new(t) is evaluated at the exact position acc_old(t+1)
    // would be evaluated at (nothing moves between one step's end and the next
    // step's start) — so the positions are exact. The tree, however, is refit
    // each substep (Simulation::update), so the reused value came from a
    // slightly different root than the one this step would have built: it
    // stays a valid Barnes-Hut evaluation at the right positions, not a
    // bit-identical one. Pass `None` on the first call.
```

- [ ] **Step 8: Full verification**

Run: `cargo test`
Expected: all green. `tests/verlet_cache_regression.rs` pins a trajectory and **will** shift slightly from the root change — re-pin it once more if it fails, and note both causes in its comment.

Run: `cargo run --release -- --benchmark 44000`
Expected: completes, prints a benchmark line. Compare `p50` against the pre-task number and record it in the commit message; `fitting_root` adds one O(n) pass per substep against an O(n log n) tree build, so any regression beyond ~1ms is a bug.

- [ ] **Step 9: Commit**

```bash
git add src/quadtree.rs src/simulation.rs src/physics.rs tests/
git commit -m "fix: refit the quadtree root each substep so no body is misfiled"
```

---

### Task 7: Verify the whole thing at scale and record the result

**Files:**
- Modify: `examples/stability_sweep.rs` (only if its hardcoded assumptions no longer hold)

- [ ] **Step 1: Re-run the ablation and the sweep**

```bash
cargo run --release --example escape_diagnostic
cargo run --release --example stability_sweep
```

Both build swarms by hand and pass explicit parameters, so they still measure the pre-fix baseline — that is intentional, they are the control. Confirm the numbers still match this plan's Background table; if they moved, the shared constants drifted and the diagnostics need updating.

- [ ] **Step 2: Measure the shipped configuration at scale**

Run: `cargo run --release --example escape_diagnostic` and compare against a run through the real `Simulation` path at n=8000 and n=44000. Record energy drift and p98 radius.

Acceptance: at n=8000, energy drift within a few percent over 10 simulated seconds (measured -0.54% for softening+C). At n=44000, judge on p98 radius only — `total_energy` sums ~1e9 pairs in f32 there and is not trustworthy, which is already documented for `total_energy_approx`.

- [ ] **Step 3: Confirm the original symptom is gone**

Run: `cargo run --release`, select Central Swarm, raise `swarm_size` to 44000, Apply, and watch for ~30 seconds. The camera should stay near the spawn framing instead of zooming out continuously.

- [ ] **Step 4: Commit any diagnostic updates**

```bash
git add examples/
git commit -m "chore: refresh stability diagnostics after the fix"
```

---

## Out of scope, deliberately

- **Removing net spawn momentum (the original "fix B").** Measured drift is 0.014–0.16% of `world_half_size` over a full run, and `CameraFit` already tracks the center of mass. In the ablation it made n=1000 *worse* by reshuffling the chaotic trajectory. Not worth the churn.
- **`theta` tuning.** Measured as counterproductive: smaller theta increases energy drift.
- **Fixing `total_energy` precision at n=44000.** Real, already documented, and independent of this work.
