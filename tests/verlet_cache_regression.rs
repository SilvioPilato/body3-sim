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
//
// theta=1.5 default applied (SimulationConfig.theta_threshold = 1.5). With
// only 7 bodies the quadtree has almost no internal nodes, so the opening-angle
// test rarely fires and theta is a no-op at this scale.
//
// EXPECTED re-pinned at SOFTENING=1.0 (was 0.001): the softened Plummer force
// perturbs the trajectory. At these separations (radii 60-280) the shift is
// tiny (~0.007 in position over 10 steps) but exceeds the 1e-4 tolerance, so
// the baseline was regenerated. This guards the acc-caching identity, not the
// SOFTENING value — re-pin if SOFTENING changes again.
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
        theta_threshold: 1.5,
        softening: 1.0,
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
        (400.002594, 399.999146, 0.151999, 0.004148),
        (455.971680, 541.668884, -3362.242676, 1341.728516),
        (493.723694, 325.725250, 2553.209229, 3172.285400),
        (263.977997, 464.102325, -1563.984863, -3289.687744),
        (630.782715, 321.162140, 926.591858, 2709.541748),
        (133.620483, 316.314117, 801.756470, -2553.406006),
        (365.990295, 545.096924, -3564.767822, -822.893921),
    ];

    for (i, (obj, exp)) in objects.iter().zip(EXPECTED.iter()).enumerate() {
        assert!((obj.position.x - exp.0).abs() < 1e-4, "obj {i} position.x: {} vs {}", obj.position.x, exp.0);
        assert!((obj.position.y - exp.1).abs() < 1e-4, "obj {i} position.y: {} vs {}", obj.position.y, exp.1);
        assert!((obj.velocity.x - exp.2).abs() < 1e-4, "obj {i} velocity.x: {} vs {}", obj.velocity.x, exp.2);
        assert!((obj.velocity.y - exp.3).abs() < 1e-4, "obj {i} velocity.y: {} vs {}", obj.velocity.y, exp.3);
    }
}
