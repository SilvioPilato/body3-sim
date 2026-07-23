// softening_sweep: measures energy-conservation vs Plummer softening length.
//
// For each softening value it runs the same fixed-time simulation (default
// T=0.3s at n=44000, dt=0.005) and reports the exact-energy |growth| =
// |E_end| / |E_start|. growth ~1.0 means energy is conserved (stable); large
// growth means close-encounter chaos is injecting spurious energy (the
// fixed-dt Verlet blowup documented in the density-fix report). This is the
// runtime-parameterized version of the earlier recompile-per-value sweep:
// softening is now SimulationConfig.softening, so one build sweeps them all.
//
//   cargo run --release --example softening_sweep -- [n] [steps] [dt]
//
// Defaults: n=44000, steps=60 (=> T=0.3s at dt=0.005), dt=0.005. Writes
// softening_sweep_n{n}.csv alongside a printed table.

use std::time::Instant;

use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

const DEFAULT_N: usize = 44_000;
const DEFAULT_STEPS: u32 = 60;
const DEFAULT_DT: f32 = 0.005;

// Spans the knee: below ~0.3 the encounter timescale sqrt(eps^3/(G*m)) drops
// under dt and energy diverges; at/above ~1.0 it is resolved and growth ~1.
const SOFTENINGS: [f32; 10] = [0.001, 0.01, 0.1, 0.3, 0.5, 1.0, 2.0, 3.0, 5.0, 10.0];

struct Row {
    softening: f32,
    e0: f32,
    e_end: f32,
    growth: f32,
    wall_ms: f64,
}

fn parse_arg<T: std::str::FromStr>(idx: usize, args: &[String], default: T) -> T {
    args.get(idx).and_then(|s| s.parse().ok()).unwrap_or(default)
}

fn run_one(n: usize, steps: u32, dt: f32, softening: f32) -> Row {
    let mut sim = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: n },
        physics_dt: dt,
        time_scale: 1.0,
        softening,
        ..SimulationConfig::default()
    });

    let e0 = sim.total_energy();
    let t0 = Instant::now();
    for _ in 0..steps {
        sim.update(dt);
    }
    let wall_ms = t0.elapsed().as_secs_f64() * 1000.0;
    let e_end = sim.total_energy();
    let growth = e_end.abs() / e0.abs().max(1e-12);

    Row { softening, e0, e_end, growth, wall_ms }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let n: usize = parse_arg(0, &args, DEFAULT_N);
    let steps: u32 = parse_arg(1, &args, DEFAULT_STEPS);
    let dt: f32 = parse_arg(2, &args, DEFAULT_DT);
    let sim_time = steps as f32 * dt;

    println!(
        "softening_sweep: n={n} steps={steps} dt={dt} (T={sim_time:.4}s), |growth| = |E_end|/|E_start| (1.0 = conserved)"
    );
    println!();
    println!("{:>10} {:>16} {:>16} {:>12} {:>10}", "softening", "E_start", "E_end", "|growth|", "wall_ms");

    let mut rows: Vec<Row> = Vec::new();
    for &s in &SOFTENINGS {
        let r = run_one(n, steps, dt, s);
        println!(
            "{:>10.3} {:>16.4e} {:>16.4e} {:>12.2} {:>10.1}",
            r.softening, r.e0, r.e_end, r.growth, r.wall_ms
        );
        rows.push(r);
    }

    let csv_path = format!("softening_sweep_n{n}.csv");
    let mut csv = String::from("softening,steps,dt,e_start,e_end,abs_growth,wall_ms\n");
    for r in &rows {
        csv.push_str(&format!(
            "{},{},{},{},{},{},{}\n",
            r.softening, steps, dt, r.e0, r.e_end, r.growth, r.wall_ms
        ));
    }
    std::fs::write(&csv_path, csv).expect("write csv");
    println!();
    println!("csv written: {csv_path}");
}
