use body3_sim::simulation::{RandomSwarmParams, Scenario, Simulation, SimulationConfig};

// Characterization test: pins Simulation::update's trajectory for a small
// deterministic scenario before the acc_new -> acc_old caching optimization
// lands in Verlet. Caching must not change a single bit of this output —
// acc_new(t) and acc_old(t+1) are mathematically the same quantity.
//
// Uses RandomSwarm (not CentralSwarm) deliberately: RandomSwarm's radius is a
// fixed user parameter and does not scale with central_swarm's sqrt(n) law,
// so this cache-equivalence guard stays independent of the density fix (and
// any future CentralSwarm geometry tweak). Bounded regime (radii 60-280
// around a ~20000-mass center) keeps pinned values in a sane magnitude range.
#[test]
fn verlet_trajectory_matches_pinned_baseline() {
    let mut sim = Simulation::new(SimulationConfig {
        scenario: Scenario::RandomSwarm(RandomSwarmParams {
            seed: 42,
            swarm_size: 6,
            radius_range: (60.0, 280.0),
            central_mass_range: (20000.0, 20001.0),
            light_mass_range: (1.0, 2.0),
        }),
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

    const EXPECTED: [(f32, f32, f32, f32); 7] = [
        (400.002594, 399.999146, 0.152050, 0.004110),
        (455.964569, 541.665100, -3362.504150, 1341.465698),
        (493.724060, 325.748749, 2552.687744, 3173.327393),
        (263.981537, 464.094452, -1563.722900, -3289.985596),
        (630.781921, 321.162842, 926.553955, 2709.567627),
        (133.621109, 316.314056, 801.782593, -2553.403564),
        (365.986206, 545.089294, -3564.842285, -823.293457),
    ];

    for (i, (obj, exp)) in objects.iter().zip(EXPECTED.iter()).enumerate() {
        assert!((obj.position.x - exp.0).abs() < 1e-4, "obj {i} position.x: {} vs {}", obj.position.x, exp.0);
        assert!((obj.position.y - exp.1).abs() < 1e-4, "obj {i} position.y: {} vs {}", obj.position.y, exp.1);
        assert!((obj.velocity.x - exp.2).abs() < 1e-4, "obj {i} velocity.x: {} vs {}", obj.velocity.x, exp.2);
        assert!((obj.velocity.y - exp.3).abs() < 1e-4, "obj {i} velocity.y: {} vs {}", obj.velocity.y, exp.3);
    }
}
