use body3_sim::camera::{CameraFit, FIT_MARGIN, FIT_PERCENTILE};
use body3_sim::physics::PhysicsObject;
use macroquad::math::{vec2, Vec2};

// A ring of `n` equal-mass bodies of radius `r` around `center`. Equal masses
// and even angular spacing put the center of mass exactly at `center`, so the
// expected fit is analytic.
fn ring(n: usize, center: Vec2, r: f32) -> Vec<PhysicsObject> {
    (0..n)
        .map(|i| {
            let angle = std::f32::consts::TAU * i as f32 / n as f32;
            PhysicsObject {
                position: center + vec2(angle.cos(), angle.sin()) * r,
                velocity: Vec2::ZERO,
                mass: 1.0,
            }
        })
        .collect()
}

#[test]
fn floor_prevents_zooming_in_past_the_scenario_framing() {
    // Cluster far smaller than the floor: every existing preset must keep its
    // original framing rather than being zoomed in on.
    let objects = ring(500, vec2(400.0, 400.0), 50.0);
    let mut fit = CameraFit::new(vec2(400.0, 400.0), 400.0);
    fit.snap(&objects);

    assert_eq!(fit.half_size(), 400.0);
}

#[test]
fn expands_past_the_floor_when_the_system_grows() {
    let objects = ring(500, vec2(400.0, 400.0), 800.0);
    let mut fit = CameraFit::new(vec2(400.0, 400.0), 400.0);
    fit.snap(&objects);

    let expected = 800.0 * FIT_MARGIN;
    assert!(
        (fit.half_size() - expected).abs() < 1.0,
        "half_size={} expected~{expected}",
        fit.half_size()
    );
}

#[test]
fn a_few_escapers_do_not_drag_the_view_out() {
    // 1% of the bodies run off to a huge distance. Below the FIT_PERCENTILE
    // cut, so the view stays sized by the bulk of the cluster.
    let mut objects = ring(1000, vec2(0.0, 0.0), 100.0);
    objects.extend(ring(10, vec2(0.0, 0.0), 500_000.0));

    let mut fit = CameraFit::new(Vec2::ZERO, 0.0);
    fit.snap(&objects);

    let expected = 100.0 * FIT_MARGIN;
    assert!(
        (fit.half_size() - expected).abs() < 1.0,
        "10/1010 outliers moved the fit: half_size={}",
        fit.half_size()
    );
}

#[test]
fn a_real_bulk_expansion_does_move_the_view() {
    // The complement of the previous test: once more than (1 - FIT_PERCENTILE)
    // of the system is out there, it is no longer an outlier tail and the view
    // must follow it. Guards against the percentile silently clipping a
    // genuine expansion.
    let outliers = (1000.0 * (1.0 - FIT_PERCENTILE) * 3.0) as usize;
    let mut objects = ring(1000, Vec2::ZERO, 100.0);
    objects.extend(ring(outliers, Vec2::ZERO, 5_000.0));

    let mut fit = CameraFit::new(Vec2::ZERO, 0.0);
    fit.snap(&objects);

    assert!(fit.half_size() > 1_000.0, "half_size={}", fit.half_size());
}

#[test]
fn view_follows_the_center_of_mass() {
    // Cluster nowhere near the sim center: the view must recenter on it, so a
    // system with net drift stays framed.
    let com = vec2(-3_000.0, 1_500.0);
    let objects = ring(500, com, 100.0);

    let mut fit = CameraFit::new(Vec2::ZERO, 400.0);
    fit.snap(&objects);

    assert!((fit.center() - com).length() < 1.0, "center={}", fit.center());
}

#[test]
fn center_of_mass_is_mass_weighted() {
    // A heavy body must dominate the framing, not be averaged away by many
    // light ones — the swarm cases are exactly this shape (20000 core, n light).
    let objects = vec![
        PhysicsObject { position: vec2(0.0, 0.0), velocity: Vec2::ZERO, mass: 999.0 },
        PhysicsObject { position: vec2(1000.0, 0.0), velocity: Vec2::ZERO, mass: 1.0 },
    ];
    let mut fit = CameraFit::new(Vec2::ZERO, 0.0);
    fit.snap(&objects);

    assert!((fit.center().x - 1.0).abs() < 1e-2, "center={}", fit.center());
}

#[test]
fn update_is_gradual_and_snap_is_immediate() {
    let objects = ring(500, vec2(400.0, 400.0), 2_000.0);

    let mut smoothed = CameraFit::new(vec2(400.0, 400.0), 400.0);
    smoothed.update(&objects, 1.0 / 60.0);

    let mut snapped = CameraFit::new(vec2(400.0, 400.0), 400.0);
    snapped.snap(&objects);

    assert!(smoothed.half_size() > 400.0, "one frame moved nothing");
    assert!(
        smoothed.half_size() < snapped.half_size(),
        "one frame jumped the whole way: {} vs {}",
        smoothed.half_size(),
        snapped.half_size()
    );
}

#[test]
fn smoothing_is_frame_rate_independent() {
    let objects = ring(500, vec2(400.0, 400.0), 2_000.0);

    // Same wall-clock elapsed time, different frame rates: the view must end
    // up in (nearly) the same place, otherwise the camera speed would depend
    // on FPS — which swings hard with swarm_size in this app.
    let mut slow = CameraFit::new(vec2(400.0, 400.0), 400.0);
    slow.update(&objects, 0.1);

    let mut fast = CameraFit::new(vec2(400.0, 400.0), 400.0);
    for _ in 0..10 {
        fast.update(&objects, 0.01);
    }

    let rel = (slow.half_size() - fast.half_size()).abs() / slow.half_size();
    assert!(rel < 0.01, "slow={} fast={}", slow.half_size(), fast.half_size());
}

#[test]
fn expansion_outruns_contraction() {
    // Asymmetric rates: the view must reach a grown system faster than it
    // settles back onto a shrunk one, so bodies never cross the edge while the
    // camera is still catching up.
    let grown = ring(500, vec2(400.0, 400.0), 2_000.0);
    let compact = ring(500, vec2(400.0, 400.0), 100.0);

    // Both moves span the same interval, floor <-> grown-fit, in opposite
    // directions, so the fraction covered in one frame is directly comparable.
    let floor = 400.0_f32;
    let mut reference = CameraFit::new(vec2(400.0, 400.0), floor);
    reference.snap(&grown);
    let grown_half = reference.half_size();
    let span = grown_half - floor;
    assert!(span > 0.0);

    let mut expanding = CameraFit::new(vec2(400.0, 400.0), floor);
    expanding.update(&grown, 1.0 / 60.0);
    let expand_frac = (expanding.half_size() - floor) / span;

    let mut contracting = CameraFit::new(vec2(400.0, 400.0), floor);
    contracting.snap(&grown);
    contracting.update(&compact, 1.0 / 60.0);
    let contract_frac = (grown_half - contracting.half_size()) / span;

    assert!(expand_frac > 0.0 && contract_frac > 0.0);
    assert!(
        expand_frac > contract_frac * 2.0,
        "expand_frac={expand_frac} contract_frac={contract_frac}"
    );
}

#[test]
fn non_finite_positions_hold_the_previous_view() {
    // If the integration diverges, holding the last good view beats feeding
    // NaN into the projection matrix (which blanks the whole render).
    let mut fit = CameraFit::new(vec2(400.0, 400.0), 400.0);
    fit.snap(&ring(500, vec2(400.0, 400.0), 800.0));
    let good_center = fit.center();
    let good_half = fit.half_size();

    let broken = vec![PhysicsObject {
        position: vec2(f32::NAN, f32::NAN),
        velocity: Vec2::ZERO,
        mass: 1.0,
    }];
    fit.update(&broken, 1.0 / 60.0);
    fit.snap(&broken);

    assert_eq!(fit.center(), good_center);
    assert_eq!(fit.half_size(), good_half);
}

#[test]
fn empty_and_massless_inputs_do_not_panic() {
    let mut fit = CameraFit::new(vec2(400.0, 400.0), 400.0);
    fit.update(&[], 1.0 / 60.0);
    fit.snap(&[]);

    let massless = vec![PhysicsObject {
        position: vec2(10.0, 10.0),
        velocity: Vec2::ZERO,
        mass: 0.0,
    }];
    fit.update(&massless, 1.0 / 60.0);

    assert_eq!(fit.half_size(), 400.0);
}

#[test]
fn single_body_is_handled() {
    let objects = vec![PhysicsObject {
        position: vec2(5_000.0, 5_000.0),
        velocity: Vec2::ZERO,
        mass: 1.0,
    }];
    let mut fit = CameraFit::new(vec2(400.0, 400.0), 400.0);
    fit.snap(&objects);

    // Distance from its own center of mass is 0, so only the floor applies.
    assert_eq!(fit.half_size(), 400.0);
    assert!((fit.center() - vec2(5_000.0, 5_000.0)).length() < 1e-2);
}
