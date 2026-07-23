use body3_sim::simulation::{central_swarm_radii, Scenario, Simulation, SimulationConfig};

fn make_sim(n: usize) -> Simulation {
    Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: n },
        screen_size: 800.0,
        physics_dt: 0.005,
        time_scale: 1.0,
        theta_threshold: 1.5,
        softening: body3_sim::physics::DEFAULT_SOFTENING,
    })
}

#[test]
fn spawn_radii_scale_with_sqrt_of_swarm_size() {
    for n in [1000usize, 8000, 64000] {
        let scale = (n as f32 / 1000.0).sqrt();
        let (expected_min, expected_max) = central_swarm_radii(n);
        assert!((expected_min - 60.0 * scale).abs() < 1e-4);
        assert!((expected_max - 280.0 * scale).abs() < 1e-4);

        let sim = make_sim(n);
        let center = 400.0_f32;
        let mut max_seen = 0.0_f32;
        // objects()[0] is the central body; swarm bodies start at index 1.
        for obj in &sim.objects()[1..] {
            let r = ((obj.position.x - center).powi(2) + (obj.position.y - center).powi(2)).sqrt();
            assert!(
                (expected_min - 1e-2..=expected_max + 1e-2).contains(&r),
                "n={n} body radius {r} outside [{expected_min}, {expected_max}]"
            );
            max_seen = max_seen.max(r);
        }
        // golden-angle fill actually reaches the outer edge (not clamped inside)
        assert!(max_seen > expected_max * 0.99, "n={n} max_seen={max_seen}");
    }
}

#[test]
fn default_spawn_is_pixel_compatible_with_before() {
    // n=1000 => scale == 1.0: radii [60, 280], world half-size 400 — the
    // exact values the code had before this change.
    let sim = make_sim(1000);
    assert_eq!(sim.world_half_size(), 400.0);
    let (min_r, max_r) = central_swarm_radii(1000);
    assert_eq!((min_r, max_r), (60.0, 280.0));
}

#[test]
fn world_extent_contains_scaled_swarm() {
    for n in [1000usize, 8000, 64000] {
        let sim = make_sim(n);
        let (_, max_r) = central_swarm_radii(n);
        assert!(
            sim.world_half_size() >= max_r,
            "n={n} half_size={} < max_radius={max_r}",
            sim.world_half_size()
        );
    }
}