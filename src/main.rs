use std::f32::consts::TAU;
use std::rc::Rc;

use body3_sim::physics::{GRAVITY, Physics, PhysicsObject, PhysicsSystem, Verlet};
use macroquad::prelude::*;

const PHYSICS_DT: f32 = 0.005;
const SWARM_SIZE: usize = 1000;
const SCREEN_SIZE: f32 = 800.0;
const TIME_SCALE: f32 = 0.3;

fn window_conf() -> Conf {
    Conf {
        window_title: "Simulation".to_owned(),
        window_width: SCREEN_SIZE as i32,
        window_height: SCREEN_SIZE as i32,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let center = vec2(SCREEN_SIZE / 2.0, SCREEN_SIZE / 2.0);
    let world_half_size = SCREEN_SIZE / 2.0;
    let mut objects = Rc::new(central_swarm(SWARM_SIZE, center));
    let mut physics_accumulator = 0.0;
    loop {
        clear_background(BLACK);
        physics_accumulator += get_frame_time() * TIME_SCALE;
        while physics_accumulator >= PHYSICS_DT {
            objects = Verlet::execute(objects.clone(), PHYSICS_DT, center, world_half_size);
            physics_accumulator -= PHYSICS_DT;
        }
        let total_energy = Physics::total_energy(&objects);
        println!("total_energy={:.4}", total_energy);
        for obj in objects.iter() {
            draw_circle(obj.position.x, obj.position.y, 5.0, RED);
        }
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 20.0, 20.0, WHITE);
        draw_text(&format!("Energy: {:.4}", total_energy), 10.0, 40.0, 20.0, WHITE);
        next_frame().await
    }
}

fn central_swarm(n: usize, center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let central_mass = 20_000.0_f32;
    let light_mass = 1.0_f32;
    let min_radius = 60.0_f32;
    let max_radius = 280.0_f32;

    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject {
        position: Vec2 { x: cx, y: cy },
        velocity: Vec2::ZERO,
        mass: central_mass,
    });

    // golden-angle spread: even radial/angular coverage, no rand dependency.
    let golden_angle = TAU * 0.618_034_f32;
    for i in 0..n {
        let radius = min_radius + (max_radius - min_radius) * (i as f32 / n.max(1) as f32);
        let angle = golden_angle * i as f32;
        let dir = Vec2 { x: angle.cos(), y: angle.sin() };
        let position = Vec2 { x: cx, y: cy } + dir * radius;
        let speed = (GRAVITY * central_mass / radius).sqrt();
        let tangent = Vec2 { x: -dir.y, y: dir.x } * speed;
        objects.push(PhysicsObject { position, velocity: tangent, mass: light_mass });
    }
    objects
}

fn dual_circle() -> Vec<PhysicsObject> {
    let cx = 300.0_f32;
    let cy = 300.0_f32;
    let m1 = 50.0_f32;
    let m2 = 20.0_f32;
    let d = 200.0_f32; // distance between bodies
    // distances from center of mass
    let r1 = d * m2 / (m1 + m2);
    let r2 = d * m1 / (m1 + m2);
    // circular orbital speeds: v_i = m_j * sqrt(G / (d * (m1+m2)))
    let v_factor = (GRAVITY / (d * (m1 + m2))).sqrt();
    let v1 = m2 * v_factor;
    let v2 = m1 * v_factor;
    // place bodies on horizontal axis, velocities perpendicular (vertical)
    let obj_a = PhysicsObject { position: Vec2 { x: cx - r1, y: cy }, mass: m1, velocity: Vec2 { x: 0.0, y: -v1 } };
    let obj_b = PhysicsObject { position: Vec2 { x: cx + r2, y: cy }, mass: m2, velocity: Vec2 { x: 0.0, y: v2 } };
    vec![obj_a, obj_b]
}

fn triangle_circle() -> Vec<PhysicsObject> {
    // Lagrange equilateral triangle: 3 equal masses in circular orbit
    let cx = 300.0_f32;
    let cy = 300.0_f32;
    let m = 20.0_f32;
    let side = 200.0_f32;
    let r = side / 3.0_f32.sqrt(); // circumradius

    // orbital speed for equilateral triangle: v = sqrt(G * m / side)
    let v = (GRAVITY * m / side).sqrt();

    // positions: equilateral triangle centered at (cx, cy)
    let p0 = Vec2 { x: cx, y: cy - r };                    // top
    let p1 = Vec2 { x: cx - side / 2.0, y: cy + r / 2.0 }; // bottom-left
    let p2 = Vec2 { x: cx + side / 2.0, y: cy + r / 2.0 }; // bottom-right

    // velocities: tangent to circle (clockwise on screen = CCW in math)
    let v0 = Vec2 { x: -v, y: 0.0 };
    let v1 = Vec2 { x: v / 2.0, y: v * 3.0_f32.sqrt() / 2.0 };
    let v2 = Vec2 { x: v / 2.0, y: -v * 3.0_f32.sqrt() / 2.0 };

    let obj_a = PhysicsObject { position: p0, mass: m, velocity: v0 };
    let obj_b = PhysicsObject { position: p1, mass: m, velocity: v1 };
    let obj_c = PhysicsObject { position: p2, mass: m, velocity: v2 };
    vec![obj_a, obj_b, obj_c]
}

fn burrau_1913() -> Vec<PhysicsObject> {
    // Burrau's problem (1913): masses 3, 4, 5 at vertices of a 3-4-5 right
    // triangle, all starting from rest. Center of mass is at the origin.
    //   m1=3 at ( 1, 3)   m2=4 at (-2,-1)   m3=5 at ( 1,-1)
    //   d(m2,m3)=3  d(m1,m3)=4  d(m1,m2)=5  (right angle at m3)
    let cx = 300.0_f32;
    let cy = 300.0_f32;
    let scale = 50.0_f32; // pixels per unit

    // screen coords (y flipped)
    let obj_a = PhysicsObject {
        position: Vec2 { x: cx + 1.0 * scale, y: cy - 3.0 * scale },
        mass: 3.0,
        velocity: Vec2::ZERO,
    };
    let obj_b = PhysicsObject {
        position: Vec2 { x: cx - 2.0 * scale, y: cy + 1.0 * scale },
        mass: 4.0,
        velocity: Vec2::ZERO,
    };
    let obj_c = PhysicsObject {
        position: Vec2 { x: cx + 1.0 * scale, y: cy + 1.0 * scale },
        mass: 5.0,
        velocity: Vec2::ZERO,
    };
    vec![obj_a, obj_b, obj_c]
}