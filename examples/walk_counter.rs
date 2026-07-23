// walk_counter: discriminate F3 / F4 / F5 for the super-log walk_forces scaling.
//
// F3 = tree shape: MAX_DEPTH=20 saturation -> large leaves -> per-visit
//      direct-calc cost grows (compute_acceleration calls per body grows).
// F4 = opening angle theta too small -> too many descents -> too many visits.
// F5 = per-visit cost growth (cache miss etc.) with visit count O(n log n).
//
// We call the REAL Quadtree walk (tree.root.walk, pub) with a closure that
// replicates Physics::walk_forces' opening-angle decision exactly but counted.
// This measures:
//   - total internal-node visits (theta-decision points)
//   - total leaf-node hits (where direct pair calc would run)
//   - total compute_acceleration calls = sum of indices.len() over leaf hits
//     (the O(per-leaf) inner work)
//   - max leaf size
//
// If compute_calls ratio @64000/1000 ~ 102x but time ratio ~170x -> F5 (per-visit
//   cost growing, e.g. cache miss on bigger tree).
// If total visits ratio ~170x -> too many descends -> F4 (theta) or F3 (tree
//   deeper than O(log n) due to imbalance/saturation).
// If max_leaf_size >> BUCKET_CAP=4 at n=64000 -> F3 saturation confirmed.
// Theta sweep: rerun the count with theta in {0.5, 1.0, 2.0}; if visits ratio
// normalizes at larger theta -> F4 confirmed as the lever.

use std::f32::consts::TAU;

use body3_sim::physics::{GRAVITY, PhysicsObject};
use body3_sim::quadtree::{NodeView, Quadtree, WalkDecision};
use body3_sim::simulation::central_swarm_radii;

use macroquad::math::{Vec2, vec2};

const SCREEN: f32 = 800.0;
const CENTRAL_MASS: f32 = 20_000.0;
const LIGHT_MASS: f32 = 1.0;
const GOLDEN_ANGLE: f32 = TAU * 0.618_034;
const SIZES: [usize; 3] = [1_000, 8_000, 64_000];

#[derive(Default)]
struct Counters {
    internal_visits: u64,
    leaf_hits: u64,
    compute_calls: u64,
    max_leaf_size: usize,
}

fn count_walk_forces(objects: &[PhysicsObject], tree: &Quadtree, theta: f32) -> Counters {
    let mut total = Counters::default();
    for i in 0..objects.len() {
        let mut local = Counters::default();
        let pos_i = objects[i].position;
        tree.walk(
            &mut |node: NodeView| {
                if let Some(indices) = node.indices {
                    local.leaf_hits += 1;
                    let n = indices.len();
                    local.compute_calls += n as u64;
                    if n > local.max_leaf_size {
                        local.max_leaf_size = n;
                    }
                    WalkDecision::Skip
                } else {
                    local.internal_visits += 1;
                    let d = Vec2::distance(pos_i, node.center_of_mass);
                    if d == 0.0 || (node.half_size * 2.0) / d > theta {
                        WalkDecision::Descend
                    } else {
                        WalkDecision::Skip
                    }
                }
            },
        );
        total.internal_visits += local.internal_visits;
        total.leaf_hits += local.leaf_hits;
        total.compute_calls += local.compute_calls;
        if local.max_leaf_size > total.max_leaf_size {
            total.max_leaf_size = local.max_leaf_size;
        }
    }
    total
}

fn build_a(n: usize, min_r: f32, max_r: f32, center: Vec2) -> Vec<PhysicsObject> {
    let cx = center.x;
    let cy = center.y;
    let mut objects = Vec::with_capacity(n + 1);
    objects.push(PhysicsObject { position: vec2(cx, cy), velocity: Vec2::ZERO, mass: CENTRAL_MASS });
    for i in 0..n {
        let radius = min_r + (max_r - min_r) * (i as f32 / n.max(1) as f32);
        let angle = GOLDEN_ANGLE * i as f32;
        let dir = vec2(angle.cos(), angle.sin());
        let position = vec2(cx, cy) + dir * radius;
        let speed = (GRAVITY * CENTRAL_MASS / radius).sqrt();
        let tangent = vec2(-dir.y, dir.x) * speed;
        objects.push(PhysicsObject { position, velocity: tangent, mass: LIGHT_MASS });
    }
    objects
}

fn main() {
    let (max_min, max_max) = central_swarm_radii(SIZES[2]);
    let half_size = (SCREEN / 2.0).max(max_max * 1.1);
    let center = vec2(SCREEN / 2.0, SCREEN / 2.0);

    println!("walk_counter: scenario A (current), fixed annulus min={max_min} max={max_max} half={half_size}");
    println!();
    for &theta in &[0.5_f32, 1.0, 2.0] {
        println!("--- theta = {theta} ---");
        println!(
            "{:>6} {:>14} {:>10} {:>14} {:>12} {:>14}",
            "n", "internal_vis", "leaf_hits", "compute_calls", "max_leaf", "visits/body"
        );
        let mut prev_calls: Option<u64> = None;
        let mut prev_visits: Option<u64> = None;
        for &n in &SIZES {
            let objects = build_a(n, max_min, max_max, center);
            let tree = Quadtree::build(&objects, center, half_size);
            let c = count_walk_forces(&objects, &tree, theta);
            let visits = c.internal_visits + c.leaf_hits;
            let visits_per_body = visits as f64 / n as f64;
            println!(
                "{:>6} {:>14} {:>10} {:>14} {:>12} {:>14.2}",
                n, c.internal_visits, c.leaf_hits, c.compute_calls, c.max_leaf_size, visits_per_body
            );
            if let Some(p) = prev_calls {
                let r = c.compute_calls as f64 / p as f64;
                println!("         -> compute_calls ratio vs prev step: {:.1}x", r);
            }
            if let Some(p) = prev_visits {
                let r = visits as f64 / p as f64;
                println!("         -> total visits  ratio vs prev step: {:.1}x", r);
            }
            prev_calls = Some(c.compute_calls);
            prev_visits = Some(visits);
        }
        // overall ratio 64000/1000
        let small = build_a(SIZES[0], max_min, max_max, center);
        let large = build_a(SIZES[2], max_min, max_max, center);
        let t_small = Quadtree::build(&small, center, half_size);
        let t_large = Quadtree::build(&large, center, half_size);
        let cs = count_walk_forces(&small, &t_small, theta);
        let cl = count_walk_forces(&large, &t_large, theta);
        let v_small = cs.internal_visits + cs.leaf_hits;
        let v_large = cl.internal_visits + cl.leaf_hits;
        println!(
            "overall n=1000 -> n=64000: compute_calls {:.1}x  visits {:.1}x  (O(n log n) pred ~102.5x)",
            cl.compute_calls as f64 / cs.compute_calls as f64,
            v_large as f64 / v_small as f64
        );
        println!();
    }
}