# egui configuration panel design

Date: 2026-07-22

## Context

`src/simulation.rs` (from the prior `simulation-module` plan) already exposes a
render-agnostic `Simulation`/`SimulationConfig`/`Scenario` API designed for
exactly this purpose: a GUI that lets a user pick a scenario, tune its
parameters, and rebuild the simulation on demand via `Simulation::reset()`.
`main.rs` currently hardcodes `SimulationConfig::default()` at startup with
no way to change it without recompiling.

This spec adds a live configuration sidebar using `egui` (via the
`egui-macroquad` bridge crate), so the user can pick any of the 6 `Scenario`
variants, edit their parameters, and apply changes without restarting the
process.

## Goals

- Add `egui-macroquad = "0.17.3"` as the only new dependency. Verified: it
  depends on `macroquad = "0.4.14"` (compatible with our pinned `0.4.15`,
  same major.minor line, semver-compatible) and `egui = "0.31.1"`.
- Widen the window from `SCREEN_SIZE` (800) to `SCREEN_SIZE + SIDEBAR_WIDTH`
  (800 + 280 = 1080), keeping height at `SCREEN_SIZE` (800). The simulation
  canvas keeps drawing in world coordinates `0..800` on both axes — no
  coordinate remapping needed, since macroquad's default screen-space origin
  is the window's top-left and the added window width is exactly where the
  egui side panel docks (confirmed against the approved layout mockup: sim
  canvas left, panel right, same height).
- A right-docked egui side panel, always visible (not toggleable — the
  window is already sized to include it, so hiding it would just leave dead
  space).
- The panel lets the user:
  - Pick any of the 6 `Scenario` variants from a dropdown.
  - Edit the parameters relevant to whichever variant is selected (nothing
    for `DualCircle`/`TriangleCircle`/`Burrau1913`, which are fixed presets;
    `swarm_size` for `CentralSwarm`; the full param set for
    `RandomSwarm`/`RandomNBody`, including an editable `seed` field and a
    "Reroll" button that randomizes `seed` in the pending config only).
  - Adjust `time_scale` and `physics_dt` with immediate effect (no restart,
    bodies keep their current positions/velocities).
  - Click "Applica" to rebuild the simulation from the currently-edited
    scenario config (full `Simulation::reset()` — bodies respawn).
- No serialization/persistence of configs across runs (out of scope, same
  as the prior plan's non-goals).

## Non-goals

- No window resizing at runtime — `screen_size`/window dimensions are fixed
  at process start, same known limitation documented in the prior spec.
- No multi-scenario "compare side by side" or saved presets file — single
  live `Simulation`, single pending config, no disk I/O.
- No redesign of `Simulation`'s existing `new`/`reset`/`objects`/
  `total_energy`/`config` methods — only two new methods are added
  (`set_time_scale`, `set_physics_dt`), everything else from the prior plan
  is reused as-is.
- No automated tests — same established project convention as the prior
  plan (zero `#[test]` in the codebase). Verified manually (build + a
  driven smoke run exercising the panel's controls).

## Architecture

`Cargo.toml` gains one dependency: `egui-macroquad = "0.17.3"`.

`main.rs`'s `window_conf()` changes its `window_width` from `SCREEN_SIZE as
i32` to `(SCREEN_SIZE + SIDEBAR_WIDTH) as i32` (a new const,
`SIDEBAR_WIDTH: f32 = 280.0`), height unchanged.

`main()` gains a `pending: SimulationConfig` local, initialized from
`sim.config().clone()` (config types are already `Copy`). Each frame:

1. `sim.update(get_frame_time())` — unchanged from today.
2. `egui_macroquad::ui(|ctx| { draw_panel(ctx, &mut pending, &mut sim) })`
   — builds the side panel, described below.
3. Existing canvas drawing (`draw_circle` loop, FPS/Energy `draw_text`) —
   unchanged, still writes into the left 800x800 region.
4. `egui_macroquad::draw()` — renders the panel on top.
5. `next_frame().await` — unchanged.

`draw_panel` (new, in `main.rs` — this is UI wiring, not simulation logic,
so it does not belong in `simulation.rs`) is responsible for:

- A `ComboBox` over the 6 scenario kinds. Changing it resets `pending
  .scenario` to that variant's `Default` (so switching away from
  `RandomSwarm` and back doesn't preserve stale, possibly-inconsistent
  half-edited values from a different variant).
- A `match` on `pending.scenario` rendering the variant-specific widgets
  (sliders/`DragValue` for numeric fields, an editable `u64` field + a
  "Reroll" button for `seed` on the two random variants).
- Two sliders for `pending.time_scale` and `pending.physics_dt`, each an
  `egui::Slider` bounded by a fixed range so `pending` itself can never
  hold an invalid value: `time_scale` in `0.0..=5.0` (0 pauses the
  simulation, 5 is 5x speed), `physics_dt` in `0.0005..=0.02`. On
  `response.changed()`, immediately call `sim.set_time_scale(pending
  .time_scale)` / `sim.set_physics_dt(pending.physics_dt)` — these mutate
  the *running* simulation without touching `pending`'s other fields or
  rebuilding anything. Because the slider range already excludes
  `physics_dt <= 0.0`, `pending.physics_dt` is never invalid by the time
  either `set_physics_dt` or `reset(pending)` sees it — the slider range is
  the primary guard; `set_physics_dt`'s own clamp (see below) is a
  defense-in-depth backstop for the theoretical case of a config value that
  didn't come from this slider (there is no such path today, but the clamp
  costs nothing and keeps the method safe to call from anywhere).
- An "Applica" button. On click, calls `sim.reset(pending)` — full rebuild
  using whatever scenario/params are currently in `pending` (including
  whatever `time_scale`/`physics_dt` values are already there from the live
  sliders, so Applica does not silently revert a speed change the user
  already made live).

## Public API additions (`src/simulation.rs`)

```rust
impl Simulation {
    pub fn set_time_scale(&mut self, time_scale: f32) {
        self.config.time_scale = time_scale;
    }

    pub fn set_physics_dt(&mut self, physics_dt: f32) {
        self.config.physics_dt = physics_dt.max(0.0001);
    }
}
```

The `physics_dt` clamp is new: `Simulation::update`'s accumulator loop
(`while self.accumulator >= self.config.physics_dt { ...; self.accumulator
-= self.config.physics_dt; }`) would hang forever on `physics_dt <= 0.0`.
This was flagged as a forward-looking Minor note in the prior plan's final
review ("worth a guard once this field becomes GUI-editable") — this is
that point. `0.0001` is an arbitrary small positive floor; the exact value
isn't load-bearing, just needs to keep the loop from stalling. No change to
`new`/`reset` — they already fully rebuild from a config the caller
controls, so no separate guard is needed there (a config with
`physics_dt <= 0.0` passed to `new`/`reset` directly is still possible in
principle, but that path isn't reachable from the GUI described here, since
the GUI only ever calls `set_physics_dt` for live changes and `reset` with
whatever `pending.physics_dt` already passed through the same clamp).

No other changes to `simulation.rs`'s existing types or methods.

## Data flow

1. Startup: `Simulation::new(SimulationConfig::default())`, `pending =
   *sim.config()`.
2. Every frame: `sim.update(...)` advances physics using `sim`'s *own*
   internal config (not `pending`) — this is what makes live `time_scale`/
   `physics_dt` changes take effect immediately without a rebuild.
3. User interacts with the panel, mutating `pending` (and, for the two live
   sliders only, also calling a `set_*` method on `sim` in the same frame).
4. User clicks Applica: `sim.reset(pending)` replaces `sim`'s entire
   internal state (config + center + world_half_size + objects +
   accumulator) with a fresh build from `pending`. After this, `sim.config()
   == pending` again (they were already in sync for time_scale/physics_dt;
   reset makes the scenario fields match too).

## Error handling

- `physics_dt` is kept valid by two layers: the panel's slider range
  (`0.0005..=0.02`, see above) is the primary guard — it's the only way
  this GUI ever writes to `pending.physics_dt`, so that field can't hold
  `<= 0.0` by construction. `set_physics_dt`'s own `.max(0.0001)` clamp is
  a secondary backstop on the `Simulation` method itself, not load-bearing
  for this GUI's data flow today, but keeps the method safe to call from
  any future caller that isn't this panel. `reset`/`new` don't need their
  own separate guard: the only value they ever receive for `physics_dt` in
  this feature is `pending.physics_dt`, which the slider range already
  keeps valid.
- Numeric range fields (`radius_range`, `mass_range`, etc.) use `egui::
  DragValue` with `clamp_range(...)` so the min of a pair can't be dragged
  above its max — prevents degenerate/inverted ranges reaching `gen_range`.
- No other fallible paths introduced.

## Testing

No automated test suite (established convention, see prior spec's
Non-goals). Verified manually:

- `cargo build`, 0 errors.
- A driven smoke run: launch, confirm the window is 1080x800 with the sim
  canvas on the left and the panel on the right, exercise the scenario
  dropdown through all 6 variants (confirming the conditional fields swap
  correctly and Applica rebuilds visibly), drag `time_scale` and confirm
  the simulation visibly speeds up/slows down without the bodies jumping
  back to their initial layout, click Reroll + Applica on a random variant
  and confirm the layout changes.
