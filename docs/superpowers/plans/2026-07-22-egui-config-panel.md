# egui Configuration Panel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a live egui sidebar to the running simulation so a user can pick any of the 6 `Scenario` variants, tune their parameters, and rebuild the simulation without recompiling — plus live (no-rebuild) control over `time_scale`/`physics_dt`.

**Architecture:** One new dependency (`egui-macroquad`). Two small additive methods on `Simulation` (`set_time_scale`, `set_physics_dt`) in `src/simulation.rs`. All UI wiring (scenario-kind tracking, widget layout, the Applica/Reroll buttons) lives in `src/main.rs` only — `simulation.rs` stays render/UI-agnostic, per the existing architectural boundary from the prior plan.

**Tech Stack:** Rust (edition 2024), macroquad 0.4.15, `egui-macroquad = "0.17.3"` (pulls in `egui 0.31.1` and `macroquad 0.4.14` compatibly — verified against our pinned `0.4.15`).

**Spec:** `docs/superpowers/specs/2026-07-22-egui-config-panel-design.md`
**Builds on:** `docs/superpowers/plans/2026-07-22-simulation-module.md` (already implemented — `src/simulation.rs` exists with `Scenario`/`SimulationConfig`/`Simulation`)

---

### Task 1: Add the dependency and the two `Simulation` setter methods

**Files:**
- Modify: `Cargo.toml`
- Modify: `src/simulation.rs`

- [ ] **Step 1: Add the dependency**

In `Cargo.toml`, under `[dependencies]`:

```toml
[dependencies]
macroquad = "0.4.15"
egui-macroquad = "0.17.3"
```

- [ ] **Step 2: Add the two setter methods to `Simulation`**

In `src/simulation.rs`, inside `impl Simulation { ... }`, add these two methods after `pub fn config(&self) -> &SimulationConfig { &self.config }` (currently the last method in the impl block, ending at line 264):

```rust
    pub fn set_time_scale(&mut self, time_scale: f32) {
        self.config.time_scale = time_scale;
    }

    pub fn set_physics_dt(&mut self, physics_dt: f32) {
        self.config.physics_dt = physics_dt.max(0.0001);
    }
```

These mutate the config of the *running* simulation directly — no rebuild, bodies keep their current positions/velocities. `set_physics_dt`'s clamp is a backstop (the GUI's slider range, added in Task 2, is the actual guard that keeps this value sane before it ever gets here) — see the spec's Error Handling section for why both layers exist.

- [ ] **Step 3: Verify it compiles**

Run: `cargo build`
Expected: `0 errors`. This will also be the first build that pulls down `egui-macroquad` and its transitive deps (`egui`, `egui-miniquad`, etc.) — expect a longer-than-usual build the first time. Warnings: same pre-existing baseline as before (`unused import: NodeView` in `physics.rs`) — no new warnings, since `set_time_scale`/`set_physics_dt` are `pub` (no dead-code lint) and the new dependency isn't used anywhere yet in this task.

- [ ] **Step 4: Commit**

```bash
git add Cargo.toml Cargo.lock src/simulation.rs
git commit -m "Add egui-macroquad dependency and live config setters

Simulation::set_time_scale/set_physics_dt let the GUI (next task)
adjust simulation speed without rebuilding the scenario, unlike
reset() which rebuilds everything from scratch."
```

---

### Task 2: Wire the sidebar into `main.rs`

**Files:**
- Modify: `src/main.rs` (full rewrite of the file)

- [ ] **Step 1: Replace the contents of `src/main.rs`**

```rust
use body3_sim::simulation::{RandomNBodyParams, RandomSwarmParams, Scenario, Simulation, SimulationConfig};
use egui_macroquad::egui;
use macroquad::prelude::*;

const SIDEBAR_WIDTH: f32 = 280.0;

fn window_conf() -> Conf {
    let screen_size = SimulationConfig::default().screen_size;
    Conf {
        window_title: "Simulation".to_owned(),
        window_width: (screen_size + SIDEBAR_WIDTH) as i32,
        window_height: screen_size as i32,
        window_resizable: false,
        ..Default::default()
    }
}

#[derive(Clone, Copy, PartialEq)]
enum ScenarioKind {
    CentralSwarm,
    DualCircle,
    TriangleCircle,
    Burrau1913,
    RandomSwarm,
    RandomNBody,
}

impl ScenarioKind {
    const ALL: [ScenarioKind; 6] = [
        ScenarioKind::CentralSwarm,
        ScenarioKind::DualCircle,
        ScenarioKind::TriangleCircle,
        ScenarioKind::Burrau1913,
        ScenarioKind::RandomSwarm,
        ScenarioKind::RandomNBody,
    ];

    fn label(&self) -> &'static str {
        match self {
            ScenarioKind::CentralSwarm => "Central Swarm",
            ScenarioKind::DualCircle => "Dual Circle",
            ScenarioKind::TriangleCircle => "Triangle Circle",
            ScenarioKind::Burrau1913 => "Burrau 1913",
            ScenarioKind::RandomSwarm => "Random Swarm",
            ScenarioKind::RandomNBody => "Random N-Body",
        }
    }

    fn from_scenario(scenario: &Scenario) -> Self {
        match scenario {
            Scenario::CentralSwarm { .. } => ScenarioKind::CentralSwarm,
            Scenario::DualCircle => ScenarioKind::DualCircle,
            Scenario::TriangleCircle => ScenarioKind::TriangleCircle,
            Scenario::Burrau1913 => ScenarioKind::Burrau1913,
            Scenario::RandomSwarm(_) => ScenarioKind::RandomSwarm,
            Scenario::RandomNBody(_) => ScenarioKind::RandomNBody,
        }
    }

    fn default_scenario(&self) -> Scenario {
        match self {
            ScenarioKind::CentralSwarm => Scenario::CentralSwarm { swarm_size: 1000 },
            ScenarioKind::DualCircle => Scenario::DualCircle,
            ScenarioKind::TriangleCircle => Scenario::TriangleCircle,
            ScenarioKind::Burrau1913 => Scenario::Burrau1913,
            ScenarioKind::RandomSwarm => Scenario::RandomSwarm(RandomSwarmParams::default()),
            ScenarioKind::RandomNBody => Scenario::RandomNBody(RandomNBodyParams::default()),
        }
    }
}

fn draw_panel(ctx: &egui::Context, pending: &mut SimulationConfig, sim: &mut Simulation) {
    egui::SidePanel::right("config_panel")
        .resizable(false)
        .exact_width(SIDEBAR_WIDTH)
        .show(ctx, |ui| {
            ui.heading("Simulation");
            ui.separator();

            let current_kind = ScenarioKind::from_scenario(&pending.scenario);
            egui::ComboBox::from_label("Scenario")
                .selected_text(current_kind.label())
                .show_ui(ui, |ui| {
                    for kind in ScenarioKind::ALL {
                        if ui.selectable_label(kind == current_kind, kind.label()).clicked() && kind != current_kind {
                            pending.scenario = kind.default_scenario();
                        }
                    }
                });

            ui.separator();

            match &mut pending.scenario {
                Scenario::CentralSwarm { swarm_size } => {
                    ui.add(egui::Slider::new(swarm_size, 1..=3000).text("swarm_size"));
                }
                Scenario::DualCircle | Scenario::TriangleCircle | Scenario::Burrau1913 => {
                    ui.label("Fixed preset, no parameters.");
                }
                Scenario::RandomSwarm(params) => {
                    ui.add(egui::Slider::new(&mut params.swarm_size, 1..=3000).text("swarm_size"));
                    ui.add(egui::Slider::new(&mut params.radius_range.0, 1.0..=params.radius_range.1).text("radius min"));
                    ui.add(egui::Slider::new(&mut params.radius_range.1, params.radius_range.0..=600.0).text("radius max"));
                    ui.add(egui::Slider::new(&mut params.central_mass_range.0, 100.0..=params.central_mass_range.1).text("central mass min"));
                    ui.add(egui::Slider::new(&mut params.central_mass_range.1, params.central_mass_range.0..=100_000.0).text("central mass max"));
                    ui.add(egui::Slider::new(&mut params.light_mass_range.0, 0.1..=params.light_mass_range.1).text("light mass min"));
                    ui.add(egui::Slider::new(&mut params.light_mass_range.1, params.light_mass_range.0..=10.0).text("light mass max"));
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut params.seed).prefix("seed: "));
                        if ui.button("Reroll").clicked() {
                            params.seed = macroquad::rand::rand() as u64;
                        }
                    });
                }
                Scenario::RandomNBody(params) => {
                    ui.add(egui::Slider::new(&mut params.count, 1..=100).text("count"));
                    ui.add(egui::Slider::new(&mut params.mass_range.0, 1.0..=params.mass_range.1).text("mass min"));
                    ui.add(egui::Slider::new(&mut params.mass_range.1, params.mass_range.0..=5000.0).text("mass max"));
                    ui.add(egui::Slider::new(&mut params.position_spread, 10.0..=400.0).text("position spread"));
                    ui.add(egui::Slider::new(&mut params.velocity_range.0, 0.0..=params.velocity_range.1).text("velocity min"));
                    ui.add(egui::Slider::new(&mut params.velocity_range.1, params.velocity_range.0..=200.0).text("velocity max"));
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut params.seed).prefix("seed: "));
                        if ui.button("Reroll").clicked() {
                            params.seed = macroquad::rand::rand() as u64;
                        }
                    });
                }
            }

            ui.separator();

            if ui.add(egui::Slider::new(&mut pending.time_scale, 0.0..=5.0).text("time_scale")).changed() {
                sim.set_time_scale(pending.time_scale);
            }
            if ui.add(egui::Slider::new(&mut pending.physics_dt, 0.0005..=0.02).text("physics_dt")).changed() {
                sim.set_physics_dt(pending.physics_dt);
            }

            ui.separator();

            if ui.button("Applica").clicked() {
                sim.reset(*pending);
            }
        });
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut sim = Simulation::new(SimulationConfig::default());
    let mut pending = *sim.config();

    loop {
        clear_background(BLACK);
        sim.update(get_frame_time());

        egui_macroquad::ui(|ctx| {
            draw_panel(ctx, &mut pending, &mut sim);
        });

        let total_energy = sim.total_energy();
        println!("total_energy={:.4}", total_energy);
        for obj in sim.objects() {
            draw_circle(obj.position.x, obj.position.y, 5.0, RED);
        }
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 20.0, 20.0, WHITE);
        draw_text(&format!("Energy: {:.4}", total_energy), 10.0, 40.0, 20.0, WHITE);

        egui_macroquad::draw();

        next_frame().await
    }
}
```

Notes for whoever implements this (this is exact code, not pseudocode, but here's the reasoning in case something doesn't compile as-is):

- `ScenarioKind` is a UI-only helper (unit-only enum, not related to `Scenario`'s data-carrying variants) that exists purely so the `ComboBox` has something `PartialEq` to compare against — `Scenario` itself is not modified, no new derives needed on it in `simulation.rs`.
- `params.radius_range.0..=params.radius_range.1`-style ranges (min slider's upper bound is the current max, max slider's lower bound is the current min) are two *disjoint* tuple fields being borrowed at once (one `&mut`, one read) — this is standard Rust field-splitting and compiles fine, no `RefCell`/cloning needed.
- `macroquad::rand::rand()` (not `gen_range`) is quad-rand's "give me a fresh u32 from the global RNG" function — used only for the Reroll button, fully qualified so no new `use` is needed and there's no ambiguity with anything in `macroquad::prelude::*`.
- Order in the frame loop matters: `egui_macroquad::ui(...)` (builds/measures the panel, processes input) happens *before* the canvas `draw_circle`/`draw_text` calls, but `egui_macroquad::draw()` (actually paints pixels) happens *after* them — so the panel visually sits on top of the canvas, per the spec.

- [ ] **Step 2: Verify it compiles**

Run: `cargo build`
Expected: `0 errors`. Same pre-existing baseline warning as before (`NodeView` unused import in `physics.rs`) — no new warnings.

- [ ] **Step 3: Commit**

```bash
git add src/main.rs
git commit -m "Add egui sidebar for live scenario/parameter configuration

Window widens to fit a right-docked panel: scenario picker with
per-variant parameter widgets, live time_scale/physics_dt sliders,
Reroll for random-scenario seeds, and an Applica button that
rebuilds the simulation from the edited config."
```

(Manual verification of the panel's actual behavior — clicking through scenarios, dragging sliders, confirming Applica/Reroll work — happens in Task 3, not here. Don't skip committing just because you haven't run it yet; Task 3 is the dedicated verification step.)

---

### Task 3: Manually verify the panel end-to-end

**Files:** None (verification only — no code changes, nothing to commit from this task).

This project has no automated test suite (established convention). This task is the substitute: actually run the app, actually click things, actually look at what happens. Use whatever screenshot/mouse-automation tooling is available in your environment (on Windows, PowerShell + `System.Drawing`/`System.Windows.Forms` for screenshots, plus a `user32.dll` `SetCursorPos`/`mouse_event`/`SetForegroundWindow` P/Invoke helper for clicks, works well — build one in the scratch/temp directory, not committed to the repo). Exact pixel coordinates can't be specified in advance here since they depend on the actual rendered layout — take a screenshot first, locate the widget, then click it.

- [ ] **Step 1: Launch and confirm the base layout**

Run the built binary, screenshot after ~1-2s. Expected: window is 1080x800 (800 sim canvas + 280 sidebar), simulation canvas on the left showing the default `CentralSwarm` (matches Task 3 of the prior plan's default-scenario screenshot), sidebar on the right showing: heading "Simulation", a "Scenario" combo box reading "Central Swarm", a `swarm_size` slider, then `time_scale`/`physics_dt` sliders, and an "Applica" button. No panic in the terminal.

- [ ] **Step 2: Open the scenario dropdown, confirm all 6 options**

Click the "Scenario" combo box, screenshot. Expected: a dropdown list showing all 6 labels (Central Swarm, Dual Circle, Triangle Circle, Burrau 1913, Random Swarm, Random N-Body) — confirms `ScenarioKind::ALL` renders completely.

- [ ] **Step 3: Switch to Random N-Body, confirm conditional fields swap**

Click "Random N-Body" in the dropdown, screenshot. Expected: the swarm_size slider is gone, replaced by `count`/`mass_range` min+max/`position_spread`/`velocity_range` min+max sliders, a `seed` drag-value, and a "Reroll" button. The simulation canvas should NOT have changed yet (switching the dropdown only edits `pending`, it doesn't call `reset` — confirm the canvas still shows the old `CentralSwarm` swarm).

- [ ] **Step 4: Click Applica, confirm the canvas rebuilds**

Click "Applica", screenshot. Expected: the canvas now shows a small number of scattered bodies (matching the `RandomNBody` look already confirmed visually in the prior plan's Task 3), not the `CentralSwarm` swarm — confirms `sim.reset(pending)` actually rebuilds using the edited scenario.

- [ ] **Step 5: Click Reroll, confirm the seed changes without needing Applica**

Note the `seed` value shown, click "Reroll", screenshot. Expected: the displayed seed number changes immediately (Reroll only edits `pending.scenario`'s seed — the canvas itself shouldn't change until Applica is clicked again; if you also want to see the new seed's layout, click Applica once more and confirm the body positions differ from Step 4's screenshot).

- [ ] **Step 6: Drag `time_scale`, confirm it's live (no restart)**

Switch back to `Central Swarm` in the dropdown and click Applica (to get back to a moving swarm), then drag the `time_scale` slider to a different value (e.g. from 0.3 toward 1.5) and screenshot immediately after. Expected: the slider's displayed number changes, and the simulation does NOT restart (bodies are wherever they already were, not back at their initial golden-angle-spaced spawn positions) — this confirms `set_time_scale` took the live-update path rather than `reset`.

- [ ] **Step 7: Report**

Summarize pass/fail for each step above. If everything passes, the plan is done — no commit needed for this task (nothing changed). If anything fails, fix the specific issue in `main.rs` (Task 2's file) and re-run the affected step(s) — do not silently patch around a failure without re-verifying it.
