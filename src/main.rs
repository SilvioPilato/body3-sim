use body3_sim::simulation::{RandomNBodyParams, RandomSwarmParams, Scenario, Simulation, SimulationConfig};
use egui_macroquad::egui;
use macroquad::prelude::*;

const SIDEBAR_WIDTH: f32 = 280.0;
// total_energy() is O(n^2) (all-pairs potential) — recomputing it every frame
// dominates cost at large swarm sizes. Displayed value only needs to be
// recognizable to a human, not per-frame-accurate, so it's throttled.
const ENERGY_LOG_INTERVAL_FRAMES: u64 = 30;

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
    let mut colors: Vec<Color> = (0..sim.objects().len()).map(|_| random_body_color()).collect();
    let mut total_energy = sim.total_energy();
    let mut frame_count: u64 = 0;

    loop {
        clear_background(BLACK);
        sim.update(get_frame_time());

        egui_macroquad::ui(|ctx| {
            draw_panel(ctx, &mut pending, &mut sim);
        });

        if colors.len() != sim.objects().len() {
            colors = (0..sim.objects().len()).map(|_| random_body_color()).collect();
        }

        if frame_count % ENERGY_LOG_INTERVAL_FRAMES == 0 {
            total_energy = sim.total_energy();
            println!("total_energy={:.4}", total_energy);
        }
        frame_count += 1;

        for (obj, color) in sim.objects().iter().zip(colors.iter()) {
            draw_circle(obj.position.x, obj.position.y, 5.0, *color);
        }
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 20.0, 20.0, WHITE);
        draw_text(&format!("Energy: {:.4}", total_energy), 10.0, 40.0, 20.0, WHITE);

        egui_macroquad::draw();

        next_frame().await
    }
}
