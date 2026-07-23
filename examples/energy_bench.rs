// energy_bench: headless energy profiler.
//
// Runs N physics steps and samples both the exact O(n^2) total_energy and the
// Barnes-Hut O(n log n) total_energy_approx at regular intervals, then prints
// an ASCII sparkline graph + a per-sample table of the relative error
// |approx - exact| / |exact|, and writes a CSV for external plotting.
//
// No window/render (plain fn main) — energy is a physics quantity, independent
// of the render path, so measuring it headless keeps the run fast and clean
// and never distorts a frame-time benchmark. Usage:
//
//   cargo run --release --example energy_bench -- [n] [steps] [sample_every] [dt]
//
// Defaults: n=44000, steps=300, sample_every=30, dt=0.005. time_scale=1.0 so
// each update(dt) runs exactly one Verlet substep (deterministic, matches
// examples/profile_workload.rs methodology). To compare energy across dt
// values at equal simulated time, pick steps = round(T / dt) for a fixed T.

use std::time::Instant;

use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

const DEFAULT_N: usize = 44_000;
const DEFAULT_STEPS: u32 = 300;
const DEFAULT_SAMPLE_EVERY: u32 = 30;
const DEFAULT_DT: f32 = 0.005;

struct Sample {
    step: u32,
    exact: f32,
    approx: f32,
}

fn sample(sim: &Simulation, step: u32) -> Sample {
    Sample { step, exact: sim.total_energy(), approx: sim.total_energy_approx() }
}

fn sparkline(vals: &[f64]) -> String {
    if vals.is_empty() {
        return String::new();
    }
    let min = vals.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    const BLOCKS: [char; 8] = ['\u{2581}', '\u{2582}', '\u{2583}', '\u{2584}', '\u{2585}', '\u{2586}', '\u{2587}', '\u{2588}'];
    let range = max - min;
    if range == 0.0 {
        return vals.iter().map(|_| BLOCKS[3]).collect();
    }
    vals.iter()
        .map(|v| {
            let t = (v - min) / range;
            let idx = ((t * 7.0).round() as i64).clamp(0, 7) as usize;
            BLOCKS[idx]
        })
        .collect()
}

fn parse_arg(idx: usize, args: &[String], default: &str) -> String {
    args.get(idx).cloned().unwrap_or_else(|| default.to_string())
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let n: usize = parse_arg(0, &args, &DEFAULT_N.to_string()).parse().unwrap_or(DEFAULT_N);
    let steps: u32 = parse_arg(1, &args, &DEFAULT_STEPS.to_string()).parse().unwrap_or(DEFAULT_STEPS);
    let sample_every: u32 = parse_arg(2, &args, &DEFAULT_SAMPLE_EVERY.to_string())
        .parse()
        .unwrap_or(DEFAULT_SAMPLE_EVERY)
        .max(1);
    let dt: f32 = parse_arg(3, &args, &DEFAULT_DT.to_string()).parse().unwrap_or(DEFAULT_DT);

    let mut sim = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: n },
        physics_dt: dt,
        time_scale: 1.0,
        ..SimulationConfig::default()
    });

    let mut samples: Vec<Sample> = Vec::new();
    samples.push(sample(&sim, 0));
    let t0 = Instant::now();
    for step in 1..=steps {
        sim.update(sim.config().physics_dt);
        if step % sample_every == 0 {
            samples.push(sample(&sim, step));
        }
    }
    let wall_ms = t0.elapsed().as_secs_f64() * 1000.0;
    let ms_per_step = wall_ms / steps as f64;
    let sim_time = steps as f32 * dt;

    let exact_vals: Vec<f64> = samples.iter().map(|s| s.exact as f64).collect();
    let approx_vals: Vec<f64> = samples.iter().map(|s| s.approx as f64).collect();
    let rel_errs: Vec<f64> = samples
        .iter()
        .map(|s| {
            let e = s.exact.abs().max(1e-12) as f64;
            ((s.approx as f64 - s.exact as f64) / e).abs() * 100.0
        })
        .collect();

    println!(
        "energy_bench: n={n} steps={steps} sample_every={sample_every} dt={dt} ({} samples, theta={}, sim_time={sim_time:.4}s wall={wall_ms:.1}ms {ms_per_step:.2}ms/step)",
        samples.len(),
        sim.config().theta_threshold
    );
    println!();
    println!("exact  energy: {}", sparkline(&exact_vals));
    println!("approx energy: {}", sparkline(&approx_vals));
    println!("rel err %:     {}", sparkline(&rel_errs));
    println!();
    println!("{:>6} {:>16} {:>16} {:>10}", "step", "exact", "approx", "rel_err_%");
    for (s, r) in samples.iter().zip(rel_errs.iter()) {
        println!("{:>6} {:>16.4e} {:>16.4e} {:>10.3}", s.step, s.exact, s.approx, r);
    }
    println!();

    let e0 = samples.first().map(|s| s.exact).unwrap_or(0.0);
    let e_end = samples.last().map(|s| s.exact).unwrap_or(0.0);
    let growth = e_end.abs() / e0.abs().max(1e-12);
    let mean_err: f64 = rel_errs.iter().sum::<f64>() / rel_errs.len().max(1) as f64;
    let max_err: f64 = rel_errs.iter().cloned().fold(0.0_f64, f64::max);
    println!("exact energy: {:.4e} -> {:.4e} (|growth| {:.2}x)", e0, e_end, growth);
    println!("rel err: mean={:.3}%  max={:.3}%", mean_err, max_err);

    let csv_path = format!("energy_bench_n{n}_steps{steps}.csv");
    let mut csv = String::from("step,exact,approx,rel_err_percent\n");
    for (s, r) in samples.iter().zip(rel_errs.iter()) {
        csv.push_str(&format!("{},{},{},{}\n", s.step, s.exact, s.approx, r));
    }
    std::fs::write(&csv_path, csv).expect("write csv");
    println!("csv written: {csv_path}");
}