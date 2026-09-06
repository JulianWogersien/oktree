use std::time::Duration;

use cgmath::{ Point3, Vector3 };
use macroquad::prelude::*;
use oktree::prelude::*;
use ::rand::RngExt;

const RANGE: u32 = 256;
const SIZE: u32 = 16;
const COUNTER: usize = 1024;
const SPAWN_VOLUME_FREQUENCY: f64 = 0.05;
const SPAWN_FREQUENCY: Duration = Duration::from_millis(10);

fn window_conf() -> Conf {
    Conf {
        window_title: "Octree Demo".to_owned(),
        window_width: 1280,
        window_height: 720,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut tree: Octree<u32, DummyVolume<u32>> = Octree::from_aabb(
        Aabb::new_unchecked(TUVec3::splat(RANGE / 2), RANGE / 2)
    );

    let mut spawn_timer = SpawnTimer::new(SPAWN_FREQUENCY);
    let mut mode = Mode::Insert;
    let mut counter: usize = 0;

    let mut rnd = ::rand::rng();

    let eye: Point3<f32> = Point3::new(-(RANGE as f32), 0.0, (RANGE / 2) as f32);
    let target: Point3<f32> = Point3::new(
        (RANGE / 2) as f32,
        (RANGE / 2) as f32,
        (RANGE / 2) as f32
    );
    let up: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

    loop {
        clear_background(BLACK);

        let delta = Duration::from_secs_f32(get_frame_time());

        if spawn_timer.tick(delta) {
            step_simulation(&mut tree, &mut mode, &mut counter, &mut rnd);
        }

        set_camera(
            &(Camera3D {
                position: point3_to_vec3(eye),
                target: point3_to_vec3(target),
                up: vec3(up.x, up.y, up.z),
                ..Default::default()
            })
        );

        draw_nodes(&tree);
        draw_elements(&tree);

        set_default_camera();

        next_frame().await;
    }
}

fn step_simulation(
    tree: &mut Octree<u32, DummyVolume<u32>>,
    mode: &mut Mode,
    counter: &mut usize,
    rnd: &mut impl ::rand::Rng
) {
    match *mode {
        Mode::Insert => {
            let position = TUVec3 {
                x: rnd.random_range(0..RANGE),
                y: rnd.random_range(0..RANGE),
                z: rnd.random_range(0..RANGE),
            };
            if rnd.random_bool(SPAWN_VOLUME_FREQUENCY) {
                let c = DummyVolume::new(position, rnd.random_range(0..SIZE));
                tree.insert(c).ok();
            } else {
                let c = DummyCell::new(position);
                tree.insert(DummyVolume {
                    aabb: c.position().unit_aabb(),
                }).ok();
            }
            *counter += 1;
            if *counter >= COUNTER {
                *mode = Mode::Remove;
            }
        }
        Mode::Remove => {
            let next = tree.iter_elements().next();
            match next {
                Some(e) => {
                    let e = e.0;
                    tree.remove(e).ok();
                }
                None => {
                    *counter = 0;
                    *mode = Mode::Insert;
                }
            }
        }
    }
}

fn draw_nodes(tree: &Octree<u32, DummyVolume<u32>>) {
    for node in tree.iter_nodes() {
        let center: Point3<f32> = node.aabb.center().into();
        let size = node.aabb.size() as f32;
        let pos = point3_to_vec3(center);
        let extent = vec3(size, size, size);

        match node.ntype {
            NodeType::Empty => draw_cube_wires(pos, extent, Color::new(0.7, 0.7, 0.7, 1.0)),
            NodeType::Leaf(_) => draw_cube_wires(pos, extent, Color::new(0.9, 0.45, 0.0, 1.0)),
            NodeType::Branch(_) => (),
        };
    }
}

fn draw_elements(tree: &Octree<u32, DummyVolume<u32>>) {
    for element in tree.iter() {
        let center: Point3<f32> = element.volume().center().into();
        let pos = point3_to_vec3(center);
        let radius = element.volume().size() as f32;

        draw_sphere_wires(pos, radius, None, RED);
    }
}

fn point3_to_vec3(p: Point3<f32>) -> Vec3 {
    vec3(p.x, p.y, p.z)
}

enum Mode {
    Insert,
    Remove,
}

struct SpawnTimer {
    accumulated: Duration,
    interval: Duration,
}

impl SpawnTimer {
    fn new(interval: Duration) -> Self {
        Self {
            accumulated: Duration::ZERO,
            interval,
        }
    }

    fn tick(&mut self, delta: Duration) -> bool {
        self.accumulated += delta;
        if self.accumulated >= self.interval {
            self.accumulated -= self.interval;
            true
        } else {
            false
        }
    }
}

struct DummyCell<U: Unsigned> {
    position: TUVec3<U>,
}

impl<U: Unsigned> Position for DummyCell<U> {
    type U = U;
    fn position(&self) -> TUVec3<U> {
        self.position
    }
}

impl<U: Unsigned> DummyCell<U> {
    fn new(position: TUVec3<U>) -> Self {
        DummyCell { position }
    }
}

struct DummyVolume<U: Unsigned> {
    aabb: Aabb<U>,
}

impl<U: Unsigned> Volume for DummyVolume<U> {
    type U = U;
    fn volume(&self) -> Aabb<Self::U> {
        self.aabb
    }
}

impl<U: Unsigned> DummyVolume<U> {
    fn new(position: TUVec3<U>, size: U) -> Self {
        DummyVolume {
            aabb: Aabb::new_unchecked(position, size),
        }
    }
}
