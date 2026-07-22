use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

const PROFILE_SWARM_SIZE: usize = 44_000; // the empirically "20-30 FPS" cliff point
const PROFILE_ITERATIONS: u32 = 300;

fn main() {
    let mut sim = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: PROFILE_SWARM_SIZE },
        screen_size: 800.0,
        physics_dt: 0.005,
        time_scale: 1.0,
    });

    let start = std::time::Instant::now();
    for _ in 0..PROFILE_ITERATIONS {
        // time_scale=1.0 and feeding physics_dt back as the frame time makes
        // the accumulator hit its threshold exactly once per call (dt == dt),
        // so every iteration runs exactly one Verlet substep, deterministically.
        let dt = sim.config().physics_dt;
        sim.update(dt);
    }
    let elapsed = start.elapsed();

    println!(
        "{PROFILE_ITERATIONS} steps at swarm_size={PROFILE_SWARM_SIZE}: {:.3}s total, {:.3}ms/step",
        elapsed.as_secs_f64(),
        elapsed.as_secs_f64() * 1000.0 / PROFILE_ITERATIONS as f64
    );
}
