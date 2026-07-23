use body3_sim::simulation::{RandomNBodyParams, RandomSwarmParams, Scenario, Simulation, SimulationConfig};
use egui_macroquad::egui;
use macroquad::prelude::*;

const SIDEBAR_WIDTH: f32 = 280.0;
// total_energy() is exact O(n^2) (all-pairs potential) — recomputing it on
// the render thread at large swarm sizes (~1.1s at n=44000, measured) stalls
// rendering, so it is offloaded to a background worker (see `energy_worker`
// usage in main below); the exact value updates asynchronously when the
// result arrives. Physics::total_energy_approx (Barnes-Hut tree walk) exists
// as a faster alternative but stays unusable at the high-n regime where speed
// matters: its error grows with PAIR AGGREGATION COUNT, not density —
// measured via examples/energy_theta_sweep at ~0.5% @ n=500, ~30% @ n=8000,
// ~185% @ n=44000 (theta=1.8, post density-fix). Sweeping theta only moves it
// ~10-20%; the n-growth is intrinsic. So it is not wired into the UI.
// Baseline interval tuned for this codebase's default swarm_size (1000);
// energy_log_interval scales it up at larger n. With the worker in place the
// interval only throttles how often a NEW snapshot is requested — a request
// is dropped if the previous computation is still in flight, which at
// n>=~16000 means effectively one computation at a time.
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
const PHYSICS_DT_RANGE: std::ops::RangeInclusive<f32> = 0.0005..=0.02;

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
                    ui.add(egui::Slider::new(swarm_size, CENTRAL_SWARM_SIZE_RANGE).text("swarm_size"));
                }
                Scenario::DualCircle | Scenario::TriangleCircle | Scenario::Burrau1913 => {
                    ui.label("Fixed preset, no parameters.");
                }
                Scenario::RandomSwarm(params) => {
                    ui.add(egui::Slider::new(&mut params.swarm_size, RANDOM_SWARM_SIZE_RANGE).text("swarm_size"));
                    ui.add(egui::Slider::new(&mut params.radius_range.0, 1.0..=params.radius_range.1).text("radius min"));
                    ui.add(egui::Slider::new(&mut params.radius_range.1, params.radius_range.0..=RANDOM_SWARM_RADIUS_MAX).text("radius max"));
                    ui.add(egui::Slider::new(&mut params.central_mass_range.0, RANDOM_SWARM_CENTRAL_MASS_MIN..=params.central_mass_range.1).text("central mass min"));
                    ui.add(egui::Slider::new(&mut params.central_mass_range.1, params.central_mass_range.0..=RANDOM_SWARM_CENTRAL_MASS_MAX).text("central mass max"));
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
                    ui.add(egui::Slider::new(&mut params.count, RANDOM_NBODY_COUNT_RANGE).text("count"));
                    ui.add(egui::Slider::new(&mut params.mass_range.0, RANDOM_NBODY_MASS_MIN..=params.mass_range.1).text("mass min"));
                    ui.add(egui::Slider::new(&mut params.mass_range.1, params.mass_range.0..=RANDOM_NBODY_MASS_MAX).text("mass max"));
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

            ui.separator();

            if ui.add(egui::Slider::new(&mut pending.time_scale, TIME_SCALE_RANGE).text("time_scale")).changed() {
                sim.set_time_scale(pending.time_scale);
            }
            if ui.add(egui::Slider::new(&mut pending.physics_dt, PHYSICS_DT_RANGE).text("physics_dt")).changed() {
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
    }

    let mut sim = Simulation::new(config);
    let mut pending = *sim.config();
    let mut colors: Vec<Color> = (0..sim.objects().len()).map(|_| random_body_color()).collect();
    let mut total_energy = sim.total_energy();
    let mut frame_count: u64 = 0;
    let mut bench_samples: Vec<f64> = Vec::with_capacity(BENCH_FRAME_COUNT);
    let mut bench_frames_seen: usize = 0;
    let mut energy_worker = body3_sim::energy::EnergyWorker::new();

    loop {
        let frame_start = std::time::Instant::now();

        clear_background(BLACK);
        sim.update(get_frame_time());

        egui_macroquad::ui(|ctx| {
            draw_panel(ctx, &mut pending, &mut sim);
        });

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

        // Zoom-to-fit: the physics domain grows with sqrt(swarm_size)
        // (constant spawn density), so map the whole world square onto the
        // fixed 800x800 sim area left of the egui sidebar. At the default
        // n=1000, world == screen and this camera is the identity mapping.
        let screen_size = sim.config().screen_size;
        let world_size = sim.world_half_size() * 2.0;
        set_camera(&Camera2D {
            target: vec2(screen_size / 2.0, screen_size / 2.0),
            zoom: vec2(2.0 / world_size, -2.0 / world_size),
            viewport: Some((0, 0, screen_size as i32, screen_size as i32)),
            ..Default::default()
        });
        // Compensate dot size: ~6 screen px regardless of zoom level. Use
        // draw_rectangle (6 verts) instead of draw_circle (~30 verts/body):
        // at n=44000 circle tessellation dominates the draw cost (~23ms) on CPU
        // vertex gen + upload; a ~6px square is indistinguishable from a point at
        // that size and visibly lighter than 10px. WASM-identical API (WebGL batch path unchanged).
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
                bench_samples.push(frame_start.elapsed().as_secs_f64() * 1000.0);
            }
            if bench_samples.len() >= BENCH_FRAME_COUNT {
                report_benchmark(&mut bench_samples, bench_swarm_size);
                std::process::exit(0);
            }
        }
    }
}
