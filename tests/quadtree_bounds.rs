// tests/quadtree_bounds.rs
use body3_sim::physics::{Physics, PhysicsObject, DEFAULT_SOFTENING, DEFAULT_TETHA_THRESHOLD};
use body3_sim::quadtree::fitting_root;
use macroquad::math::{vec2, Vec2};

fn body(x: f32, y: f32, mass: f32) -> PhysicsObject {
    PhysicsObject { position: vec2(x, y), velocity: Vec2::ZERO, mass }
}

#[test]
fn fitting_root_contains_every_body() {
    let objects = vec![
        body(-5_000.0, 12.0, 1.0),
        body(3.0, 40_000.0, 1.0),
        body(100.0, 100.0, 20_000.0),
    ];
    let (center, half) = fitting_root(&objects);
    for o in &objects {
        assert!(
            (o.position.x - center.x).abs() <= half && (o.position.y - center.y).abs() <= half,
            "body at {:?} outside root center={center:?} half={half}",
            o.position
        );
    }
}

#[test]
fn fitting_root_survives_degenerate_input() {
    // All bodies coincident: extent is zero, and a zero half-size would make
    // every subdivision degenerate.
    let objects = vec![body(7.0, 7.0, 1.0), body(7.0, 7.0, 1.0)];
    let (_, half) = fitting_root(&objects);
    assert!(half > 0.0 && half.is_finite(), "half={half}");

    let (_, half) = fitting_root(&[]);
    assert!(half > 0.0 && half.is_finite(), "empty: half={half}");
}

#[test]
fn a_body_outside_the_static_root_gets_the_right_force() {
    // The regression: with a root that does not contain it, an escaper is
    // misfiled and the resulting accelerations are wrong for everyone.
    // A fitted root must reproduce the exact all-pairs answer closely.
    let mut objects = vec![body(0.0, 0.0, 20_000.0)];
    for i in 0..8 {
        let a = i as f32;
        objects.push(body(50.0 + a * 10.0, 20.0 - a * 5.0, 1.0));
    }
    objects.push(body(9_000.0, -9_000.0, 1.0)); // the escaper

    let (center, half) = fitting_root(&objects);
    let fitted = Physics::compute_accelerations(&objects, center, half, DEFAULT_TETHA_THRESHOLD, DEFAULT_SOFTENING);

    // theta = 0 forces the walk to descend to leaves everywhere: exact forces.
    let exact = Physics::compute_accelerations(&objects, center, half, 0.0, DEFAULT_SOFTENING);

    for (i, (f, e)) in fitted.iter().zip(exact.iter()).enumerate() {
        let err = (*f - *e).length() / e.length().max(1e-6);
        assert!(err < 0.05, "body {i}: relative force error {:.1}%", err * 100.0);
    }
}