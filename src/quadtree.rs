use macroquad::math::{Vec2, vec2};

use crate::physics::PhysicsObject;

const BUCKET_CAP: usize = 4;
const MAX_DEPTH: u32 = 20;

#[derive(PartialEq)]
pub enum WalkDecision { Descend, Skip }

pub struct NodeView<'a> { pub total_mass: f32, pub center_of_mass: Vec2, pub half_size: f32, pub indices: Option<&'a [usize]> }

pub struct Quadtree<'a> {
    pub root: Quadrant,
    pub objects: Vec<&'a PhysicsObject>,
}

enum QuadNode {
    Leaf{
        indices: Vec<usize>,
        total_mass: f32,
        center_of_mass: Vec2,
    },
    Internal(Quadrant),
}

pub struct Quadrant {
    center: Vec2,
    half_size: f32,
    total_mass: f32,
    center_of_mass: Vec2,
    children: [Option<Box<QuadNode>>; 4],
}

impl Quadrant {
    pub fn new(center: Vec2, half_size: f32) -> Self {
        Quadrant {
            center,
            half_size,
            total_mass: 0.0,
            center_of_mass: center,
            children: [None, None, None, None],
        }
    }

    fn sub_center(&self, q_index: usize) -> Vec2 {
        let offset = self.half_size / 2.0;
        match q_index {
            0 => vec2(self.center.x - offset, self.center.y + offset),
            1 => vec2(self.center.x + offset, self.center.y + offset),
            2 => vec2(self.center.x - offset, self.center.y - offset),
            3 => vec2(self.center.x + offset, self.center.y - offset),
            _ => unreachable!(),
        }
    }

    pub fn walk<F>(&self, visitor: &mut F, objects: &[&PhysicsObject])
    where F: FnMut(NodeView) -> WalkDecision
    {
        for child in &self.children {
            match child {
                Some(node) => {
                    match node.as_ref() {
                        QuadNode::Leaf { indices, total_mass, center_of_mass } => {
                            visitor(NodeView { total_mass: *total_mass, center_of_mass: *center_of_mass, half_size: 0.0, indices: Some(indices) });
                        },
                        QuadNode::Internal(quadrant) => {
                            let decision = visitor(NodeView { total_mass: quadrant.total_mass, center_of_mass: quadrant.center_of_mass, half_size: quadrant.half_size, indices: None });
                            if decision == WalkDecision::Descend {
                                quadrant.walk(visitor, objects);
                            }
                        },
                    }
                },
                None => {},
            }
        }
    }

    pub fn insert(&mut self, index: usize, objects: &[&PhysicsObject], depth: u32) {
        let q_index = self.find_quadrant(&objects[index].position);
        let m = objects[index].mass;
        let new_total = self.total_mass + m;
        self.center_of_mass = (self.center_of_mass * self.total_mass + objects[index].position * m) / new_total;
        self.total_mass += m;

        match self.children[q_index].take() {
            Some(mut boxed) => match &mut *boxed {
                QuadNode::Leaf { indices, total_mass, center_of_mass } => {
                    if indices.len() < BUCKET_CAP || depth >= MAX_DEPTH {
                        let new_leaf_total = *total_mass + m;
                        *center_of_mass = (*center_of_mass * *total_mass + objects[index].position * m) / new_leaf_total;
                        *total_mass = new_leaf_total;
                        indices.push(index);
                        self.children[q_index] = Some(boxed);
                    } else {
                        let existing = std::mem::take(indices);
                        let mut sub = Quadrant::new(self.sub_center(q_index), self.half_size / 2.0);
                        for i in existing {
                            sub.insert(i, objects, depth + 1);
                        }
                        sub.insert(index, objects, depth + 1);
                        self.children[q_index] = Some(Box::new(QuadNode::Internal(sub)));
                    }
                }
                QuadNode::Internal(sub) => {
                    sub.insert(index, objects, depth + 1);
                    self.children[q_index] = Some(boxed);
                }
            },
            None => {
                self.children[q_index] = Some(Box::new(QuadNode::Leaf {
                    indices: vec![index],
                    total_mass: m,
                    center_of_mass: objects[index].position,
                }));
            }
        }
    }

    pub fn find_quadrant(&self, pos: &Vec2) -> usize {
        let right = pos.x >= self.center.x;
        let down = pos.y >= self.center.y;

        match (right, down) {
            (false, true) => 0,
            (true, true) => 1,
            (false, false) => 2,
            (true, false) => 3,
        }
    }
}

impl<'a> Quadtree<'a> {
    pub fn new(center: Vec2, half_size: f32) -> Self {
        Quadtree {
            root: Quadrant::new(center, half_size),
            objects: Vec::new(),
        }
    }

    pub fn build(objects: &'a [PhysicsObject], center: Vec2, half_size: f32) -> Self {
        let mut tree = Self::new(center, half_size);
        for (i, obj) in objects.iter().enumerate() {
            debug_assert!(
                (obj.position.x - center.x).abs() <= half_size
                    && (obj.position.y - center.y).abs() <= half_size,
                "body at {:?} outside root center={center:?} half_size={half_size}; \
                 build the root with fitting_root",
                obj.position
            );
            tree.objects.push(obj);
            tree.root.insert(i, &tree.objects, 0);
        }
        tree
    }

    pub fn insert(&mut self, obj: &'a PhysicsObject) {
        let index = self.objects.len();
        self.objects.push(obj);
        self.root.insert(index, &self.objects, 0);
    }
}

// Root center and half-size that provably contain every body. `insert` has no
// bounds check by design (it is the hot path), so the root must be correct by
// construction: a body outside it is filed into a corner quadrant, and the
// node summaries it lands in then misdescribe its position for every
// opening-angle test in the walk.
//
// Non-finite positions are skipped rather than propagated — an infinite root
// would collapse the tree to a single leaf and quietly turn the walk into
// O(n^2). The floor keeps the half-size positive for coincident or empty
// input, where the extent is zero.
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