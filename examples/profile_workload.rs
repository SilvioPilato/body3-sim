use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

const PROFILE_SWARM_SIZE: usize = 44_000; // pre-density-fix "20-30 FPS" cliff point; post density-fix + theta=1.8 it's ~10.9 ms/step headless (was ~37 ms)
const PROFILE_ITERATIONS: u32 = 300;

fn main() {
    // Optional CLI override: `cargo run --release --example profile_workload -- <swarm_size> [iterations]`
    let args: Vec<String> = std::env::args().collect();
    let swarm_size = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(PROFILE_SWARM_SIZE);
    let iterations = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(PROFILE_ITERATIONS);

    let mut sim = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size },
        screen_size: 800.0,
        physics_dt: 0.005,
        time_scale: 1.0,
        theta_threshold: body3_sim::physics::DEFAULT_TETHA_THRESHOLD,
        softening: body3_sim::physics::DEFAULT_SOFTENING,
    });

    let start = std::time::Instant::now();
    for _ in 0..iterations {
        // time_scale=1.0 and feeding physics_dt back as the frame time makes
        // the accumulator hit its threshold exactly once per call (dt == dt),
        // so every iteration runs exactly one Verlet substep, deterministically.
        let dt = sim.config().physics_dt;
        sim.update(dt);
    }
    let elapsed = start.elapsed();

    println!(
        "{iterations} steps at swarm_size={swarm_size}: {:.3}s total, {:.3}ms/step",
        elapsed.as_secs_f64(),
        elapsed.as_secs_f64() * 1000.0 / iterations as f64
    );
}
