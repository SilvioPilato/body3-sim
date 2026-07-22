use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

// Characterization test: pins Simulation::update's trajectory for a small
// deterministic scenario before the acc_new -> acc_old caching optimization
// lands in Verlet. Caching must not change a single bit of this output —
// acc_new(t) and acc_old(t+1) are mathematically the same quantity.
#[test]
fn verlet_trajectory_matches_pinned_baseline() {
    let mut sim = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: 5 },
        screen_size: 800.0,
        physics_dt: 0.005,
        time_scale: 1.0,
    });

    for _ in 0..10 {
        sim.update(0.005);
    }

    let objects: Vec<_> = sim.objects().to_vec();
    for (i, obj) in objects.iter().enumerate() {
        eprintln!(
            "{i}: pos=({:.6}, {:.6}) vel=({:.6}, {:.6})",
            obj.position.x, obj.position.y, obj.velocity.x, obj.velocity.y
        );
    }

    const EXPECTED: [(f32, f32, f32, f32); 6] = [
        (400.001343, 400.010559, -0.260776, 0.107282),
        (378.540680, 340.168854, 5241.006836, -1526.177734),
        (500.903778, 368.397797, 1329.645020, 4103.954590),
        (264.248718, 459.880005, -1491.686646, -3349.742188),
        (591.555664, 385.502472, 247.240021, 3216.213379),
        (186.649338, 299.033691, 1243.816284, -2631.465088),
    ];

    for (i, (obj, exp)) in objects.iter().zip(EXPECTED.iter()).enumerate() {
        assert!((obj.position.x - exp.0).abs() < 1e-4, "obj {i} position.x: {} vs {}", obj.position.x, exp.0);
        assert!((obj.position.y - exp.1).abs() < 1e-4, "obj {i} position.y: {} vs {}", obj.position.y, exp.1);
        assert!((obj.velocity.x - exp.2).abs() < 1e-4, "obj {i} velocity.x: {} vs {}", obj.velocity.x, exp.2);
        assert!((obj.velocity.y - exp.3).abs() < 1e-4, "obj {i} velocity.y: {} vs {}", obj.velocity.y, exp.3);
    }
}
