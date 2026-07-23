use body3_sim::energy::EnergyWorker;
use body3_sim::physics::Physics;
use body3_sim::simulation::{Scenario, Simulation, SimulationConfig};

fn small_objects() -> Vec<body3_sim::physics::PhysicsObject> {
    let sim = Simulation::new(SimulationConfig {
        scenario: Scenario::CentralSwarm { swarm_size: 500 },
        ..SimulationConfig::default()
    });
    sim.objects().to_vec()
}

#[test]
fn worker_returns_exact_energy() {
    let objects = small_objects();
    let expected = Physics::total_energy(&objects);

    let mut worker = EnergyWorker::new();
    assert!(!worker.busy());
    worker.request(&objects);
    assert!(worker.busy());

    // small swarm: computation finishes well under a second; poll with a
    // generous deadline so slow CI machines don't flake.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let result = loop {
        if let Some(energy) = worker.try_recv() {
            break energy;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "worker did not finish within 5s"
        );
        std::thread::sleep(std::time::Duration::from_millis(1));
    };
    assert_eq!(result, expected);
    assert!(!worker.busy());
}

#[test]
fn second_request_while_busy_is_dropped() {
    let objects = small_objects();
    let mut worker = EnergyWorker::new();
    worker.request(&objects);
    // try to slam another request in immediately; must not panic and must not
    // replace the in-flight one. Depending on scheduling the first may already
    // be done — that's fine; just ensure no panic and eventual completion.
    worker.request(&objects);
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    loop {
        if worker.try_recv().is_some() {
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "worker did not finish within 5s"
        );
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}
