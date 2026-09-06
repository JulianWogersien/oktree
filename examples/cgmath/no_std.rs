#![no_std]

use cgmath::{ Point3, Vector3, prelude::*, raycast::RayCast3d };
use oktree::prelude::*;

extern crate alloc;
extern crate wee_alloc;

#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

fn main() -> Result<(), TreeError> {
    let aabb = Aabb::new_unchecked(TUVec3::splat(8u8), 8);
    let mut tree = Octree::from_aabb(aabb);

    let e1_id = tree.insert(TUVec3u8::new(3, 3, 3))?;
    assert_eq!(e1_id, ElementId(0));

    let hit = tree.ray_cast(&RayCast3d::new(Point3::from_value(1.0), Vector3::unit_x(), 10.0));
    assert_eq!(hit, HitResult {
        element: None,
        distance: 0.0,
    });
    Ok(())
}
