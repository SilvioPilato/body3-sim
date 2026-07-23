use macroquad::math::{Vec2, vec2};

use crate::physics::PhysicsObject;

const BUCKET_CAP: usize = 4;
const MAX_DEPTH: u32 = 20;
// Sentinel for "no child" / "no overflow" in the flat arena's u32 index slots.
const EMPTY: u32 = u32::MAX;

#[derive(PartialEq)]
pub enum WalkDecision { Descend, Skip }

pub struct NodeView<'a> { pub total_mass: f32, pub center_of_mass: Vec2, pub half_size: f32, pub indices: Option<&'a [usize]> }

// Flat arena node. The whole tree is one `Vec<Node>` instead of a graph of
// heap-boxed quadrants each owning a `Vec` of indices — killing the per-node
// `Box` and per-leaf `Vec` allocation churn that dominated build cost (and that
// wasm's dlmalloc punishes hardest).
//
// Every node is a square region. A leaf stores its bodies inline in `bodies`
// (up to BUCKET_CAP); `internal` marks a region that has been subdivided, whose
// bodies have moved down into `children` (indices into the same arena, or
// EMPTY). `overflow` is the escape hatch for the MAX_DEPTH-saturated case where
// more than BUCKET_CAP bodies pile into one leaf that can no longer split: the
// full index list then lives in `Quadtree::overflow` and `bodies` is ignored.
struct Node {
    center: Vec2,
    half_size: f32,
    total_mass: f32,
    center_of_mass: Vec2,
    children: [u32; 4],
    bodies: [usize; BUCKET_CAP],
    body_count: u32,
    internal: bool,
    overflow: u32,
}

impl Node {
    fn leaf(center: Vec2, half_size: f32) -> Self {
        Node {
            center,
            half_size,
            total_mass: 0.0,
            center_of_mass: center,
            children: [EMPTY; 4],
            bodies: [0; BUCKET_CAP],
            body_count: 0,
            internal: false,
            overflow: EMPTY,
        }
    }
}

pub struct Quadtree {
    nodes: Vec<Node>,
    // Body lists for leaves that saturated at MAX_DEPTH and overflowed their
    // inline bucket. Almost always empty; only allocates under extreme
    // clustering (many bodies closer than half_size / 2^MAX_DEPTH).
    overflow: Vec<Vec<usize>>,
}

fn find_quadrant(center: Vec2, pos: Vec2) -> usize {
    let right = pos.x >= center.x;
    let down = pos.y >= center.y;
    match (right, down) {
        (false, true) => 0,
        (true, true) => 1,
        (false, false) => 2,
        (true, false) => 3,
    }
}

fn sub_center(center: Vec2, half_size: f32, q: usize) -> Vec2 {
    let offset = half_size / 2.0;
    match q {
        0 => vec2(center.x - offset, center.y + offset),
        1 => vec2(center.x + offset, center.y + offset),
        2 => vec2(center.x - offset, center.y - offset),
        3 => vec2(center.x + offset, center.y - offset),
        _ => unreachable!(),
    }
}

impl Quadtree {
    pub fn build(objects: &[PhysicsObject], center: Vec2, half_size: f32) -> Self {
        // Node count is bounded by ~2n (n leaves + internal nodes above them);
        // one up-front reservation keeps the whole build to a single allocation
        // in the common case.
        let mut nodes: Vec<Node> = Vec::with_capacity(2 * objects.len() + 16);
        // Root is a pre-subdivided region: bodies live in its leaf children,
        // never in the root directly (matches the previous Quadrant-rooted tree,
        // so the walk visits the exact same set of nodes).
        let mut root = Node::leaf(center, half_size);
        root.internal = true;
        nodes.push(root);
        let mut overflow: Vec<Vec<usize>> = Vec::new();

        for (i, obj) in objects.iter().enumerate() {
            debug_assert!(
                (obj.position.x - center.x).abs() <= half_size
                    && (obj.position.y - center.y).abs() <= half_size,
                "body at {:?} outside root center={center:?} half_size={half_size}; \
                 build the root with fitting_root",
                obj.position
            );
            Self::insert(&mut nodes, &mut overflow, objects, 0, i, 0);
        }
        Quadtree { nodes, overflow }
    }

    // Insert body `index` into the subtree rooted at `node_idx`. Threads the
    // arena by index (not by reference) so the recursive descent can push new
    // nodes without holding an aliasing borrow. `objects` is needed to read the
    // saved bodies' positions/masses when a full leaf subdivides.
    fn insert(
        nodes: &mut Vec<Node>,
        overflow: &mut Vec<Vec<usize>>,
        objects: &[PhysicsObject],
        node_idx: usize,
        index: usize,
        depth: u32,
    ) {
        let pos = objects[index].position;
        let m = objects[index].mass;

        // Aggregate this region's mass / center-of-mass on the way down.
        {
            let node = &mut nodes[node_idx];
            let new_total = node.total_mass + m;
            node.center_of_mass = (node.center_of_mass * node.total_mass + pos * m) / new_total;
            node.total_mass = new_total;
        }

        let (center, half_size) = { let n = &nodes[node_idx]; (n.center, n.half_size) };
        let q = find_quadrant(center, pos);
        let child = nodes[node_idx].children[q];

        if child == EMPTY {
            let mut leaf = Node::leaf(sub_center(center, half_size, q), half_size / 2.0);
            leaf.bodies[0] = index;
            leaf.body_count = 1;
            leaf.total_mass = m;
            leaf.center_of_mass = pos;
            let new_idx = nodes.len() as u32;
            nodes.push(leaf);
            nodes[node_idx].children[q] = new_idx;
            return;
        }

        let child = child as usize;
        if nodes[child].internal {
            Self::insert(nodes, overflow, objects, child, index, depth + 1);
            return;
        }

        // Child is a leaf. Room to spare, or already spilled to the overflow
        // pool, or saturated at MAX_DEPTH (can't subdivide): append. Otherwise
        // subdivide and re-file.
        let bc = nodes[child].body_count as usize;
        if nodes[child].overflow != EMPTY {
            let ov = nodes[child].overflow as usize;
            overflow[ov].push(index);
            Self::add_leaf_mass(&mut nodes[child], pos, m);
        } else if bc < BUCKET_CAP {
            let leaf = &mut nodes[child];
            leaf.bodies[bc] = index;
            leaf.body_count += 1;
            Self::add_leaf_mass(leaf, pos, m);
        } else if depth >= MAX_DEPTH {
            // Saturated leaf: move its inline bodies into a fresh overflow list
            // and continue appending there. `bodies` is ignored from now on.
            let mut list = Vec::with_capacity(BUCKET_CAP * 2);
            list.extend_from_slice(&nodes[child].bodies);
            list.push(index);
            let ov = overflow.len() as u32;
            overflow.push(list);
            nodes[child].overflow = ov;
            Self::add_leaf_mass(&mut nodes[child], pos, m);
        } else {
            let saved = nodes[child].bodies;
            {
                let leaf = &mut nodes[child];
                leaf.internal = true;
                leaf.body_count = 0;
                leaf.total_mass = 0.0;
                leaf.center_of_mass = leaf.center;
                leaf.children = [EMPTY; 4];
            }
            for &bi in saved.iter() {
                Self::insert(nodes, overflow, objects, child, bi, depth + 1);
            }
            Self::insert(nodes, overflow, objects, child, index, depth + 1);
        }
    }

    fn add_leaf_mass(leaf: &mut Node, pos: Vec2, m: f32) {
        let new_total = leaf.total_mass + m;
        leaf.center_of_mass = (leaf.center_of_mass * leaf.total_mass + pos * m) / new_total;
        leaf.total_mass = new_total;
    }

    // Visit the tree with the same node ordering and leaf/internal distinction
    // the previous Quadrant::walk exposed: each of a region's children is either
    // a leaf (yielded with its index slice, half_size 0.0) or an internal region
    // (yielded with its half_size; descended into only if the visitor says so).
    pub fn walk<F>(&self, visitor: &mut F)
    where F: FnMut(NodeView) -> WalkDecision
    {
        if !self.nodes.is_empty() {
            self.walk_node(0, visitor);
        }
    }

    fn walk_node<F>(&self, idx: usize, visitor: &mut F)
    where F: FnMut(NodeView) -> WalkDecision
    {
        let node = &self.nodes[idx];
        for &c in &node.children {
            if c == EMPTY {
                continue;
            }
            let child = &self.nodes[c as usize];
            if child.internal {
                let decision = visitor(NodeView {
                    total_mass: child.total_mass,
                    center_of_mass: child.center_of_mass,
                    half_size: child.half_size,
                    indices: None,
                });
                if decision == WalkDecision::Descend {
                    self.walk_node(c as usize, visitor);
                }
            } else {
                let indices = if child.overflow != EMPTY {
                    &self.overflow[child.overflow as usize][..]
                } else {
                    &child.bodies[..child.body_count as usize]
                };
                visitor(NodeView {
                    total_mass: child.total_mass,
                    center_of_mass: child.center_of_mass,
                    half_size: 0.0,
                    indices: Some(indices),
                });
            }
        }
    }
}

// Root center and half-size that provably contain every body. `insert` has no
// bounds check (it's the hot path), so a body outside the root gets filed
// into the wrong corner and misdescribes every opening-angle test that node
// participates in.
//
// Non-finite positions are skipped rather than propagated (an infinite root
// would collapse the tree to one leaf, turning the walk into O(n^2)); the
// floor keeps half-size positive for coincident or empty input.
pub fn fitting_root(objects: &[PhysicsObject]) -> (Vec2, f32) {
    const MARGIN: f32 = 1.01;
    const MIN_HALF_SIZE: f32 = 1.0;

    let mut lo = vec2(f32::INFINITY, f32::INFINITY);
    let mut hi = vec2(f32::NEG_INFINITY, f32::NEG_INFINITY);
    for o in objects {
        if o.position.is_finite() {
            lo = lo.min(o.position);
            hi = hi.max(o.position);
        }
    }
    if !lo.is_finite() || !hi.is_finite() {
        return (Vec2::ZERO, MIN_HALF_SIZE);
    }
    let center = (lo + hi) * 0.5;
    let half = ((hi - lo).max_element() * 0.5 * MARGIN).max(MIN_HALF_SIZE);
    (center, half)
}
