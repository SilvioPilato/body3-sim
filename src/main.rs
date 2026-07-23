use body3_sim::simulation::{RandomNBodyParams, RandomSwarmParams, Scenario, Simulation, SimulationConfig};
use egui_macroquad::egui;
use macroquad::prelude::*;

const SIDEBAR_WIDTH: f32 = 280.0;
// total_energy() is exact O(n^2) (~1.1s at n=44000), so it runs on a
// background worker (see `energy_worker` below) instead of the render
// thread. Physics::total_energy_approx (Barnes-Hut) would be fast enough,
// but its error grows with n regardless of theta (measured via
// examples/energy_theta_sweep: ~0.5% @ n=500, ~185% @ n=44000), so it's not
// wired into the UI. Baseline interval tuned for the default swarm_size
// (1000); energy_log_interval scales it up at larger n.
const ENERGY_LOG_INTERVAL_FRAMES: u64 = 30;

fn energy_log_interval(swarm_size: usize) -> u64 {
    let scale = ((swarm_size as f64 / 1000.0).sqrt()).max(1.0);
    ((ENERGY_LOG_INTERVAL_FRAMES as f64) * scale) as u64
}

// UI slider bounds. Values only (no behavior change) — pulled out of
// draw_panel so the tunable ranges live in one place instead of scattered
// inline literals.
const CENTRAL_SWARM_SIZE_RANGE: std::ops::RangeInclusive<usize> = 1..=50_000;

const RANDOM_SWARM_SIZE_RANGE: std::ops::RangeInclusive<usize> = 1..=3_000;
const RANDOM_SWARM_RADIUS_MAX: f32 = 600.0;
const RANDOM_SWARM_CENTRAL_MASS_MIN: f32 = 100.0;
const RANDOM_SWARM_CENTRAL_MASS_MAX: f32 = 100_000.0;
const RANDOM_SWARM_LIGHT_MASS_MIN: f32 = 0.1;
const RANDOM_SWARM_LIGHT_MASS_MAX: f32 = 10.0;

const RANDOM_NBODY_COUNT_RANGE: std::ops::RangeInclusive<usize> = 1..=100;
const RANDOM_NBODY_MASS_MIN: f32 = 1.0;
const RANDOM_NBODY_MASS_MAX: f32 = 5_000.0;
const RANDOM_NBODY_POSITION_SPREAD_RANGE: std::ops::RangeInclusive<f32> = 10.0..=400.0;
const RANDOM_NBODY_VELOCITY_MAX: f32 = 200.0;

const TIME_SCALE_RANGE: std::ops::RangeInclusive<f32> = 0.0..=5.0;
// Lowered from 0.0005 to 0.0001 so the few-body presets' derived dt
// (FEW_BODY_DT = 1e-4, see simulation::integration_params) lands inside the
// slider and does not get silently clamped to 5e-4 on the first render.
const PHYSICS_DT_RANGE: std::ops::RangeInclusive<f32> = 0.0001..=0.02;

// `--benchmark`: forces this swarm_size (matches examples/profile_workload.rs's
// documented "20-30 FPS" cliff point, so the two numbers are comparable) and
// measures BENCH_FRAME_COUNT real frames (physics + draw_circle + egui, the
// full production render path) after discarding BENCH_WARMUP_FRAMES to let
// shader compilation / GPU pipeline warm-up drop out of the stats.
const BENCH_SWARM_SIZE: usize = 44_000;
const BENCH_FRAME_COUNT: usize = 300;
const BENCH_WARMUP_FRAMES: usize = 30;

fn report_benchmark(samples: &mut [f64], swarm_size: usize) {
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = samples.len();
    let percentile = |p: f64| samples[(((n - 1) as f64) * p).round() as usize];
    let mean: f64 = samples.iter().sum::<f64>() / n as f64;
    println!(
        "render benchmark: swarm_size={swarm_size} frames={n}\n  min={:.3}ms mean={:.3}ms p50={:.3}ms p95={:.3}ms p99={:.3}ms max={:.3}ms",
        samples[0],
        mean,
        percentile(0.50),
        percentile(0.95),
        percentile(0.99),
        samples[n - 1]
    );
}

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
    SolarSystem,
    FigureEight,
    Circumbinary,
    Trojan,
    Slingshot,
    GalaxyCollision,
    RandomSwarm,
    RandomNBody,
}

impl ScenarioKind {
    const ALL: [ScenarioKind; 12] = [
        ScenarioKind::CentralSwarm,
        ScenarioKind::DualCircle,
        ScenarioKind::TriangleCircle,
        ScenarioKind::Burrau1913,
        ScenarioKind::SolarSystem,
        ScenarioKind::FigureEight,
        ScenarioKind::Circumbinary,
        ScenarioKind::Trojan,
        ScenarioKind::Slingshot,
        ScenarioKind::GalaxyCollision,
        ScenarioKind::RandomSwarm,
        ScenarioKind::RandomNBody,
    ];

    fn label(&self) -> &'static str {
        match self {
            ScenarioKind::CentralSwarm => "Central Swarm",
            ScenarioKind::DualCircle => "Dual Circle",
            ScenarioKind::TriangleCircle => "Triangle Circle",
            ScenarioKind::Burrau1913 => "Burrau 1913",
            ScenarioKind::SolarSystem => "Solar System",
            ScenarioKind::FigureEight => "Figure Eight",
            ScenarioKind::Circumbinary => "Circumbinary",
            ScenarioKind::Trojan => "Trojan (L4/L5)",
            ScenarioKind::Slingshot => "Slingshot",
            ScenarioKind::GalaxyCollision => "Galaxy Collision",
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
            Scenario::SolarSystem => ScenarioKind::SolarSystem,
            Scenario::FigureEight => ScenarioKind::FigureEight,
            Scenario::Circumbinary => ScenarioKind::Circumbinary,
            Scenario::Trojan => ScenarioKind::Trojan,
            Scenario::Slingshot => ScenarioKind::Slingshot,
            Scenario::GalaxyCollision { .. } => ScenarioKind::GalaxyCollision,
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
            ScenarioKind::SolarSystem => Scenario::SolarSystem,
            ScenarioKind::FigureEight => Scenario::FigureEight,
            ScenarioKind::Circumbinary => Scenario::Circumbinary,
            ScenarioKind::Trojan => Scenario::Trojan,
            ScenarioKind::Slingshot => Scenario::Slingshot,
            ScenarioKind::GalaxyCollision => Scenario::GalaxyCollision { swarm_size: 2000 },
            ScenarioKind::RandomSwarm => Scenario::RandomSwarm(RandomSwarmParams::default()),
            ScenarioKind::RandomNBody => Scenario::RandomNBody(RandomNBodyParams::default()),
        }
    }
}

fn random_body_color() -> Color {
    // fixed high saturation/value: random hue only, so a color can never land near-black.
    let hue = macroquad::rand::gen_range(0.0, 360.0);
    hsv_to_color(hue, 0.75, 0.95)
}

fn hsv_to_color(h: f32, s: f32, v: f32) -> Color {
    let c = v * s;
    let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;
    let (r, g, b) = match (h / 60.0) as u32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    Color::new(r + m, g + m, b + m, 1.0)
}

// Returns true if the scenario was reset this frame, so the caller can snap
// the camera to the new configuration instead of sliding to it.
fn draw_panel(ctx: &egui::Context, pending: &mut SimulationConfig, sim: &mut Simulation) -> bool {
    let mut applied = false;
    egui::SidePanel::right("config_panel")
        .resizable(false)
        .exact_width(SIDEBAR_WIDTH)
        .show(ctx, |ui| {
            ui.heading("Simulation");
            ui.separator();

            // Set by anything that feeds integration_params. Only then is the
            // derived (dt, softening) pair recomputed — doing it every frame
            // would overwrite the physics_dt override slider below.
            let mut scenario_changed = false;
            let current_kind = ScenarioKind::from_scenario(&pending.scenario);
            egui::ComboBox::from_label("Scenario")
                .selected_text(current_kind.label())
                .show_ui(ui, |ui| {
                    for kind in ScenarioKind::ALL {
                        if ui.selectable_label(kind == current_kind, kind.label()).clicked() && kind != current_kind {
                            pending.scenario = kind.default_scenario();
                            scenario_changed = true;
                        }
                    }
                });

            ui.separator();

            match &mut pending.scenario {
                Scenario::CentralSwarm { swarm_size } => {
                    scenario_changed |= ui
                        .add(egui::Slider::new(swarm_size, CENTRAL_SWARM_SIZE_RANGE).text("swarm_size"))
                        .changed();
                }
                Scenario::GalaxyCollision { swarm_size } => {
                    scenario_changed |= ui
                        .add(egui::Slider::new(swarm_size, CENTRAL_SWARM_SIZE_RANGE).text("swarm_size (total)"))
                        .changed();
                }
                Scenario::DualCircle
                | Scenario::TriangleCircle
                | Scenario::Burrau1913
                | Scenario::SolarSystem
                | Scenario::FigureEight
                | Scenario::Circumbinary
                | Scenario::Trojan
                | Scenario::Slingshot => {
                    ui.label("Fixed preset, no parameters.");
                }
                Scenario::RandomSwarm(params) => {
                    ui.add(egui::Slider::new(&mut params.swarm_size, RANDOM_SWARM_SIZE_RANGE).text("swarm_size"));
                    ui.add(egui::Slider::new(&mut params.radius_range.0, 1.0..=params.radius_range.1).text("radius min"));
                    ui.add(egui::Slider::new(&mut params.radius_range.1, params.radius_range.0..=RANDOM_SWARM_RADIUS_MAX).text("radius max"));
                    ui.add(egui::Slider::new(&mut params.central_mass_range.0, RANDOM_SWARM_CENTRAL_MASS_MIN..=params.central_mass_range.1).text("central mass min"));
                    scenario_changed |= ui
                        .add(egui::Slider::new(&mut params.central_mass_range.1, params.central_mass_range.0..=RANDOM_SWARM_CENTRAL_MASS_MAX).text("central mass max"))
                        .changed();
                    ui.add(egui::Slider::new(&mut params.light_mass_range.0, RANDOM_SWARM_LIGHT_MASS_MIN..=params.light_mass_range.1).text("light mass min"));
                    ui.add(egui::Slider::new(&mut params.light_mass_range.1, params.light_mass_range.0..=RANDOM_SWARM_LIGHT_MASS_MAX).text("light mass max"));
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut params.seed).prefix("seed: "));
                        if ui.button("Reroll").clicked() {
                            params.seed = macroquad::rand::rand() as u64;
                        }
                    });
                }
                Scenario::RandomNBody(params) => {
                    scenario_changed |= ui
                        .add(egui::Slider::new(&mut params.count, RANDOM_NBODY_COUNT_RANGE).text("count"))
                        .changed();
                    ui.add(egui::Slider::new(&mut params.mass_range.0, RANDOM_NBODY_MASS_MIN..=params.mass_range.1).text("mass min"));
                    scenario_changed |= ui
                        .add(egui::Slider::new(&mut params.mass_range.1, params.mass_range.0..=RANDOM_NBODY_MASS_MAX).text("mass max"))
                        .changed();
                    ui.add(egui::Slider::new(&mut params.position_spread, RANDOM_NBODY_POSITION_SPREAD_RANGE).text("position spread"));
                    ui.add(egui::Slider::new(&mut params.velocity_range.0, 0.0..=params.velocity_range.1).text("velocity min"));
                    ui.add(egui::Slider::new(&mut params.velocity_range.1, params.velocity_range.0..=RANDOM_NBODY_VELOCITY_MAX).text("velocity max"));
                    ui.horizontal(|ui| {
                        ui.add(egui::DragValue::new(&mut params.seed).prefix("seed: "));
                        if ui.button("Reroll").clicked() {
                            params.seed = macroquad::rand::rand() as u64;
                        }
                    });
                }
            }

            if scenario_changed {
                let (dt, softening) = body3_sim::simulation::integration_params(&pending.scenario);
                pending.physics_dt = dt;
                pending.softening = softening;
            }

            ui.separator();
            ui.label(format!("softening: {:.2}  (derived)", pending.softening));

            ui.separator();

            if ui.add(egui::Slider::new(&mut pending.time_scale, TIME_SCALE_RANGE).text("time_scale")).changed() {
                sim.set_time_scale(pending.time_scale);
            }
            if ui.add(egui::Slider::new(&mut pending.physics_dt, PHYSICS_DT_RANGE).text("physics_dt (override)")).changed() {
                sim.set_physics_dt(pending.physics_dt);
            }

            ui.separator();

            if ui.button("Apply").clicked() {
                sim.reset(*pending);
                #[cfg(target_arch = "wasm32")]
                body3_sim::url::write_url_query(&body3_sim::url::encode(pending));
                applied = true;
            }
            if ui.button("Copy link").clicked() {
                let url = body3_sim::url::encode(pending);
                #[cfg(target_arch = "wasm32")]
                body3_sim::url::copy_link(&url);
                #[cfg(not(target_arch = "wasm32"))]
                println!("config url: {}", url);
            }
        });
    applied
}

#[macroquad::main(window_conf)]
async fn main() {
    let args: Vec<String> = std::env::args().collect();
    let benchmark_mode = args.iter().any(|a| a == "--benchmark" || a.starts_with("--benchmark="));
    // --benchmark [N] or --benchmark=N; otherwise defaults to BENCH_SWARM_SIZE.
    let bench_swarm_size: usize = args
        .iter()
        .position(|a| a == "--benchmark")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse::<usize>().ok())
        .or_else(|| {
            args.iter().find_map(|a| {
                a.strip_prefix("--benchmark=").and_then(|s| s.parse::<usize>().ok())
            })
        })
        .unwrap_or(BENCH_SWARM_SIZE);

    let mut config = SimulationConfig::default();
    if benchmark_mode {
        config.scenario = Scenario::CentralSwarm { swarm_size: bench_swarm_size };
    } else {
        #[cfg(target_arch = "wasm32")]
        {
            let query = body3_sim::url::read_url_query();
            if let Some(decoded) = body3_sim::url::decode(&query) {
                config = decoded;
            }
        }
    }

    let mut sim = Simulation::new(config);
    let mut pending = *sim.config();
    let mut colors: Vec<Color> = (0..sim.objects().len()).map(|_| random_body_color()).collect();
    let mut total_energy = sim.total_energy();
    let mut frame_count: u64 = 0;
    let mut bench_samples: Vec<f64> = Vec::with_capacity(BENCH_FRAME_COUNT);
    let mut bench_frames_seen: usize = 0;
    let mut energy_worker = body3_sim::energy::EnergyWorker::new();
    let mut camera_fit = body3_sim::camera::CameraFit::new(
        vec2(config.screen_size / 2.0, config.screen_size / 2.0),
        sim.world_half_size(),
    );
    camera_fit.snap(sim.objects());

    loop {
        // macroquad's get_time() (f64 seconds), not std::time::Instant — the
        // latter's now() is unsupported on wasm32-unknown-unknown and panics
        // every frame in the browser.
        let frame_start = get_time();

        clear_background(BLACK);
        sim.update(get_frame_time());

        let mut applied = false;
        egui_macroquad::ui(|ctx| {
            applied = draw_panel(ctx, &mut pending, &mut sim);
        });
        if applied {
            // world_half_size (the zoom-in floor) is scenario-dependent, so the
            // fit is rebuilt rather than just re-snapped.
            camera_fit = body3_sim::camera::CameraFit::new(
                vec2(sim.config().screen_size / 2.0, sim.config().screen_size / 2.0),
                sim.world_half_size(),
            );
            camera_fit.snap(sim.objects());
        }

        if colors.len() != sim.objects().len() {
            colors = (0..sim.objects().len()).map(|_| random_body_color()).collect();
        }

        // exact energy is O(n^2) (~1.1s at n=44000) — computed on a background
        // thread from a snapshot so the render loop never stalls; display/print
        // update when the result arrives. On wasm32 the worker is a no-op stub,
        // so the energy display just stays at its initial value there.
        if frame_count % energy_log_interval(sim.objects().len()) == 0 {
            energy_worker.request(sim.objects(), sim.config().softening);
        }
        if let Some(energy) = energy_worker.try_recv() {
            total_energy = energy;
            println!("total_energy={:.4}", total_energy);
        }
        frame_count += 1;

        // Zoom-to-fit, re-evaluated every frame: Simulation::world_half_size
        // only covers the spawn extent, but the system expands past it at
        // runtime (ejected bodies, core-collapse rebound). CameraFit tracks
        // the center of mass and 98th-percentile radius instead, floored at
        // world_half_size so no preset ever zooms in past its original
        // framing.
        let screen_size = sim.config().screen_size;
        camera_fit.update(sim.objects(), get_frame_time());
        let world_size = camera_fit.half_size() * 2.0;
        set_camera(&Camera2D {
            target: camera_fit.center(),
            zoom: vec2(2.0 / world_size, -2.0 / world_size),
            viewport: Some((0, 0, screen_size as i32, screen_size as i32)),
            ..Default::default()
        });
        // Compensate dot size: ~6 screen px regardless of zoom level.
        // draw_rectangle (6 verts) instead of draw_circle (~30 verts/body):
        // at n=44000 circle tessellation dominates draw cost (~23ms), and a
        // ~6px square reads as a point at that size.
        let dot_side = 6.0 * (world_size / screen_size);
        for (obj, color) in sim.objects().iter().zip(colors.iter()) {
            draw_rectangle(
                obj.position.x - dot_side * 0.5,
                obj.position.y - dot_side * 0.5,
                dot_side,
                dot_side,
                *color,
            );
        }
        set_default_camera();
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 20.0, 20.0, WHITE);
        draw_text(&format!("Energy: {:.4}", total_energy), 10.0, 40.0, 20.0, WHITE);

        egui_macroquad::draw();

        next_frame().await;

        if benchmark_mode {
            bench_frames_seen += 1;
            if bench_frames_seen > BENCH_WARMUP_FRAMES {
                bench_samples.push((get_time() - frame_start) * 1000.0);
            }
            if bench_samples.len() >= BENCH_FRAME_COUNT {
                report_benchmark(&mut bench_samples, bench_swarm_size);
                #[cfg(not(target_arch = "wasm32"))]
                std::process::exit(0);
                #[cfg(target_arch = "wasm32")]
                {
                    // benchmark mode is not meaningful in browser; stop
                    // collecting and continue rendering so the tab stays alive.
                    bench_samples.clear();
                    bench_frames_seen = 0;
                }
            }
        }
    }
}
