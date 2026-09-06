use crate::{
    bounding::{ Aabb, TUVec3, Unsigned },
    node::NodeType,
    tree::Octree,
    ElementId,
    NodeId,
    Volume,
};
use alloc::vec::Vec;
use cgmath::{
    Point3,
    Vector3,
    bounding::{ AABB3d, BoundingSphere, IntersectsVolume },
    raycast::RayCast3d,
};
use heapless::Vec as HVec;
use num::cast;

impl<U, T> Octree<U, T> where U: Unsigned, T: Volume<U = U> {
    /// Intersects an [`Octree`] with the [`RayCast3d`].
    ///
    /// Returns a [`HitResult`] with [`ElementId`] and the doistance to
    /// the intersection if any.
    ///
    /// ```rust
    /// use oktree::prelude::*;
    /// use cgmath::prelude::*;
    /// use cgmath::raycast::RayCast3d;
    /// use cgmath::{Point3, Vector3};
    ///
    /// let mut tree = Octree::from_aabb(Aabb::new(TUVec3::splat(16), 16).unwrap());
    ///
    /// let c1 = TUVec3u8::new(1u8, 1, 1);
    /// let c1_id = tree.insert(c1).unwrap();
    ///
    /// let ray = RayCast3d::new(Point3::new(5.0, 1.5, 1.5), -Vector3::unit_x(), 10.0);
    ///
    /// assert_eq!(
    ///     tree.ray_cast(&ray),
    ///     HitResult {
    ///         element: Some(c1_id),
    ///         distance: 3.0
    ///     }
    /// )
    /// ```
    pub fn ray_cast(&self, ray: &RayCast3d<f32>) -> HitResult {
        let mut hit = HitResult::default();
        self.recursive_ray_cast(self.root, ray, &mut hit);
        hit
    }

    fn recursive_ray_cast(&self, node: NodeId, ray: &RayCast3d<f32>, hit: &mut HitResult) {
        // We use a heapless stack to loop through the nodes until we complete the cast however
        // if the stack becomes full then then we fallbackon recursive calls.
        let mut stack = HVec::<_, 32>::new();
        stack.push(node).unwrap();
        while let Some(node) = stack.pop() {
            let n = &self.nodes[node];
            let aabb: AABB3d<f32> = n.aabb.into();
            if ray.intersects(&aabb) {
                match n.ntype {
                    NodeType::Empty => (),

                    NodeType::Leaf(element) => {
                        let aabb = self.elements[element].volume().into();
                        if let Some(dist) = ray.aabb_intersection_at(&aabb) {
                            match hit.element {
                                Some(_) => {
                                    if hit.distance > dist {
                                        hit.element = Some(element);
                                        hit.distance = dist;
                                    }
                                }
                                None => {
                                    hit.element = Some(element);
                                    hit.distance = dist;
                                }
                            }
                        }
                    }

                    NodeType::Branch(branch) => {
                        let mut iter = branch.children.iter();
                        while let Some(child) = iter.next() {
                            // If we can't push to the stack (to be processed on the next loop
                            // iteration) then we fallback to recursive calls.
                            if stack.push(*child).is_err() {
                                self.recursive_ray_cast(*child, ray, hit);
                                for child in iter.by_ref() {
                                    self.recursive_ray_cast(*child, ray, hit);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// Intersect [`Octree`] with [`Aabb3d`] or [`BoundingSphere`].
    ///
    /// Returns the [`vector`](Vec) of [`elements`](ElementId),
    /// intersected by volume.
    ///
    /// ```rust
    /// use oktree::prelude::*;
    /// use cgmath::prelude::*;
    /// use cgmath::{bounding::{BoundingSphere, AABB3d}, Vector3, Point3};
    ///
    /// let mut tree = Octree::from_aabb(Aabb::new(TUVec3::splat(16), 16).unwrap());
    ///
    /// let c1 = TUVec3u8::new(1u8, 1, 1);
    /// let c1_id = tree.insert(c1).unwrap();
    ///
    /// // Bounding box intersection
    /// let aabb = AABB3d::new(Point3::origin(), Vector3::from_value(5.0));
    /// assert_eq!(tree.intersect(&aabb), vec![c1_id]);
    ///
    /// // Bounding sphere intersection
    /// let sphere = BoundingSphere::new(Point3::origin(), 6.0);
    /// assert_eq!(tree.intersect(&sphere), vec![c1_id]);
    /// ```
    pub fn intersect<Volume: IntersectsVolume<AABB3d<f32>>>(
        &self,
        volume: &Volume
    ) -> Vec<ElementId> {
        let mut elements = Vec::with_capacity(10);
        self.rintersect(self.root, volume, &mut elements);
        elements.sort();
        elements.dedup();
        elements
    }

    fn rintersect<Volume: IntersectsVolume<AABB3d<f32>>>(
        &self,
        node: NodeId,
        volume: &Volume,
        elements: &mut Vec<ElementId>
    ) {
        // We use a heapless stack to loop through the nodes until we complete the cast however
        // if the stack becomes full then then we fallbackon recursive calls.
        let mut stack = HVec::<_, 32>::new();
        stack.push(node).unwrap();
        while let Some(node) = stack.pop() {
            let n = self.nodes[node];
            match n.ntype {
                NodeType::Empty => (),

                NodeType::Leaf(e) => {
                    let aabb = self.elements[e].volume().into();
                    if volume.intersects(&aabb) {
                        elements.push(e);
                    }
                }

                NodeType::Branch(branch) => {
                    let aabb: AABB3d<f32> = n.aabb.into();

                    if volume.intersects(&aabb) {
                        let mut iter = branch.children.iter();
                        while let Some(child) = iter.next() {
                            // If we can't push to the stack (to be processed on the next loop
                            // iteration) then we fallback to recursive calls.
                            if stack.push(*child).is_err() {
                                self.rintersect(*child, volume, elements);
                                for child in iter.by_ref() {
                                    self.rintersect(*child, volume, elements);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Ray intersection result.
///
/// Contains `Some(`[`ElementId`]`)` in case of intersection,
/// [None] otherwise.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct HitResult {
    pub element: Option<ElementId>,
    pub distance: f32,
}

impl<U: Unsigned> From<Aabb<U>> for AABB3d<f32> {
    fn from(value: Aabb<U>) -> Self {
        AABB3d {
            min: value.min.into(),
            max: value.max.into(),
        }
    }
}

impl<U: Unsigned> From<TUVec3<U>> for Point3<f32> {
    fn from(value: TUVec3<U>) -> Self {
        Point3::new(cast(value.x).unwrap(), cast(value.y).unwrap(), cast(value.z).unwrap())
    }
}

impl<U: Unsigned> From<TUVec3<U>> for Vector3<f32> {
    fn from(value: TUVec3<U>) -> Self {
        Vector3::new(cast(value.x).unwrap(), cast(value.y).unwrap(), cast(value.z).unwrap())
    }
}

impl<U, T> IntersectsVolume<AABB3d<f32>> for Octree<U, T> where U: Unsigned, T: Volume<U = U> {
    /// Check if a [`Aabb3d`] volume intersects with the [`Octree`] root node.
    fn intersects(&self, volume: &AABB3d<f32>) -> bool {
        let aabb: AABB3d<f32> = self.nodes[self.root].aabb.into();
        volume.intersects(&aabb)
    }
}

impl<U, T> IntersectsVolume<BoundingSphere<f32>>
    for Octree<U, T>
    where U: Unsigned, T: Volume<U = U>
{
    /// Check if a [`BoundingSphere`] volume intersects with the [`Octree`] root node.
    fn intersects(&self, volume: &BoundingSphere<f32>) -> bool {
        let aabb: AABB3d<f32> = self.nodes[self.root].aabb.into();
        volume.intersects(&aabb)
    }
}

trait IntersectVolume<Volume, T> where Volume: IntersectsVolume<AABB3d<f32>> {
    fn intersect(&self, volume: &Volume) -> Vec<ElementId>;
}

impl<U, T> IntersectVolume<AABB3d<f32>, T> for Octree<U, T> where U: Unsigned, T: Volume<U = U> {
    fn intersect(&self, volume: &AABB3d<f32>) -> Vec<ElementId> {
        self.intersect(volume)
    }
}

impl<U, T> IntersectVolume<BoundingSphere<f32>, T>
    for Octree<U, T>
    where U: Unsigned, T: Volume<U = U>
{
    fn intersect(&self, volume: &BoundingSphere<f32>) -> Vec<ElementId> {
        self.intersect(volume)
    }
}

#[cfg(test)]
mod tests {
    use crate::Position;
    use cgmath::prelude::*;

    use super::*;

    struct DummyCell<U: Unsigned> {
        position: TUVec3<U>,
        node: NodeId,
    }

    impl<U: Unsigned> Position for DummyCell<U> {
        type U = U;
        fn position(&self) -> TUVec3<U> {
            self.position
        }
    }

    impl<U: Unsigned> DummyCell<U> {
        fn new(position: TUVec3<U>) -> Self {
            DummyCell {
                position,
                node: Default::default(),
            }
        }
    }

    struct DummyVolume<U: Unsigned> {
        aabb: Aabb<U>,
        node: NodeId,
    }

    impl<U: Unsigned> Volume for DummyVolume<U> {
        type U = U;
        fn volume(&self) -> Aabb<U> {
            self.aabb
        }
    }

    impl<U: Unsigned> DummyVolume<U> {
        fn new(aabb: Aabb<U>) -> Self {
            Self {
                aabb,
                node: Default::default(),
            }
        }
    }

    #[test]
    fn test_ray_intersection() {
        let aabb = Aabb::new(TUVec3::new(4u16, 4, 4), 4);
        assert!(aabb.is_ok());
        let mut tree = Octree::from_aabb(aabb.unwrap());

        let c1 = DummyCell::new(TUVec3::new(3, 1, 1));
        assert_eq!(tree.insert(c1), Ok(ElementId(0)));

        let c2 = DummyCell::new(TUVec3::new(1, 5, 1));
        assert_eq!(tree.insert(c2), Ok(ElementId(1)));

        // hit 2nd
        let ray = RayCast3d::new(Point3::new(1.5, 1.5, 1.5), Vector3::unit_y(), 10.0);
        assert_eq!(tree.ray_cast(&ray), HitResult {
            element: Some((1).into()),
            distance: 3.5,
        });

        // miss
        let ray = RayCast3d::new(Point3::new(0.0, 0.0, 0.0), Vector3::unit_y(), 10.0);
        assert_eq!(tree.ray_cast(&ray), HitResult {
            element: None,
            distance: 0.0,
        });

        // hit 1st
        let ray = RayCast3d::new(Point3::new(0.0, 1.05, 1.05), Vector3::unit_x(), 10.0);
        assert_eq!(tree.ray_cast(&ray), HitResult {
            element: Some((0).into()),
            distance: 3.0,
        });

        // miss
        let ray = RayCast3d::new(Point3::new(40.0, 40.0, 40.0), Vector3::unit_x(), 10.0);
        assert_eq!(tree.ray_cast(&ray), HitResult {
            element: None,
            distance: 0.0,
        });

        // miss
        let ray = RayCast3d::new(Point3::new(7.0, 5.9, 1.01), -Vector3::unit_x(), 10.0);
        assert_eq!(tree.ray_cast(&ray), HitResult {
            element: Some((1).into()),
            distance: 5.0,
        });

        // miss
        let ray = RayCast3d::new(Point3::new(1.01, 1.01, 1.01), -Vector3::unit_x(), 10.0);
        assert_eq!(tree.ray_cast(&ray), HitResult {
            element: None,
            distance: 0.0,
        });

        // hit 1st
        let ray = RayCast3d::new(Point3::new(3.05, 10.0, 1.05), -Vector3::unit_y(), 10.0);
        assert_eq!(tree.ray_cast(&ray), HitResult {
            element: Some((0).into()),
            distance: 8.0,
        });
    }

    #[test]
    fn intersects_volume() {
        let aabb = Aabb::new_unchecked(TUVec3::splat(16u16), 16);
        let mut tree = Octree::from_aabb(aabb);

        let c1 = DummyCell::new(TUVec3::new(3, 1, 1));
        assert_eq!(tree.insert(c1), Ok(ElementId(0)));

        let box1 = AABB3d::new(Point3::from_value(8.0), Vector3::from_value(8.0));
        assert!(tree.intersects(&box1));

        let box2 = AABB3d::new(Point3::from_value(16.0), Vector3::from_value(16.0));
        assert!(tree.intersects(&box2));

        let box3 = AABB3d::new(Point3::from_value(16.0), Vector3::new(1.0, 1.0, 50.0));
        assert!(tree.intersects(&box3));

        let box5 = AABB3d::new(Point3::from_value(50.0), Vector3::new(1.0, 1.0, 1.0));
        assert!(!tree.intersects(&box5));

        let sphere1 = BoundingSphere::new(Point3::from_value(16.0), 16.0);
        assert!(tree.intersects(&sphere1));

        let sphere2 = BoundingSphere::new(Point3::from_value(40.0), 8.0);
        assert!(!tree.intersects(&sphere2));

        let sphere3 = BoundingSphere::new(Point3::new(40.0, 16.0, 16.0), 8.0);
        assert!(tree.intersects(&sphere3));

        let sphere4 = BoundingSphere::new(Point3::new(40.01, 16.0, 16.0), 8.0);
        assert!(!tree.intersects(&sphere4));

        let sphere5 = BoundingSphere::new(Point3::new(40.0, 16.0, 16.0), 8.01);
        assert!(tree.intersects(&sphere5));

        let sphere6 = BoundingSphere::new(Point3::new(39.99, 16.0, 16.0), 8.0);
        assert!(tree.intersects(&sphere6));
    }

    #[test]
    fn intersect_point_volume() {
        let aabb = Aabb::new_unchecked(TUVec3::splat(16u16), 16);
        let mut tree = Octree::from_aabb(aabb);

        let c1 = DummyCell::new(TUVec3::new(3, 1, 1));
        assert_eq!(tree.insert(c1), Ok(ElementId(0)));

        let c2 = DummyCell::new(TUVec3::new(1, 5, 1));
        assert_eq!(tree.insert(c2), Ok(ElementId(1)));

        let c3 = DummyCell::new(TUVec3::new(1, 1, 7));
        assert_eq!(tree.insert(c3), Ok(ElementId(2)));

        let box1 = AABB3d::new(Point3::new(0.0, 0.0, 0.0), Vector3::from_value(10.0));
        let mut test = tree.intersect(&box1);
        test.sort();
        assert_eq!(test, vec![ElementId(0), ElementId(1), ElementId(2)]);

        let box2 = AABB3d::new(Point3::new(0.0, 0.0, 0.0), Vector3::from_value(5.0));
        let mut test = tree.intersect(&box2);
        test.sort();
        assert_eq!(test, vec![ElementId(0), ElementId(1)]);

        let box3 = AABB3d::new(Point3::new(10.0, 0.0, 10.0), Vector3::from_value(5.0));
        let mut test = tree.intersect(&box3);
        test.sort();
        assert_eq!(test, vec![]);

        let sphere1 = BoundingSphere::new(Point3::new(0.0, 0.0, 0.0), 10.0);
        let mut test = tree.intersect(&sphere1);
        test.sort();
        assert_eq!(test, vec![ElementId(0), ElementId(1), ElementId(2)]);

        let sphere2 = BoundingSphere::new(Point3::new(0.0, 0.0, 0.0), 6.0);
        let mut test = tree.intersect(&sphere2);
        test.sort();
        assert_eq!(test, vec![ElementId(0), ElementId(1)]);

        let sphere3 = BoundingSphere::new(Point3::new(10.0, 0.0, 10.0), 5.0);
        let mut test = tree.intersect(&sphere3);
        test.sort();
        assert_eq!(test, vec![]);
    }

    #[test]
    fn intersect_volume_volume() {
        let aabb = Aabb::new(TUVec3::splat(8), 8u8).unwrap();
        let mut tree = Octree::from_aabb_with_capacity(aabb, 10);

        let v1_volume = Aabb::new(TUVec3::new(9, 5, 4), 4).unwrap();
        let v1 = DummyVolume::new(v1_volume);
        let v1_id = tree.insert(v1).unwrap();

        let v2_volume = Aabb::new(TUVec3::new(14, 14, 4), 4).unwrap();
        let v2 = DummyVolume::new(v2_volume);
        let v2_id = tree.insert(v2).unwrap();

        let v3_volume = Aabb::new(TUVec3::new(7, 5, 4), 4).unwrap();
        let v3 = DummyVolume::new(v3_volume);
        assert!(tree.insert(v3).is_err());
        //
        // Searching by point
        assert_eq!(tree.find(&TUVec3::new(9, 5, 4)), Some(v1_id));
        assert_eq!(tree.find(&TUVec3::new(16, 12, 2)), Some(v2_id));
        assert_eq!(tree.find(&TUVec3::new(1, 2, 8)), None);
        assert_eq!(tree.find(&&TUVec3::splat(100)), None);

        let ray = RayCast3d::new(Point3::new(9.0, 15.0, 4.0), -Vector3::unit_y(), 100.0);

        // Hit!
        assert_eq!(tree.ray_cast(&ray), HitResult {
            element: Some(ElementId(0)),
            distance: 6.0,
        });

        assert_eq!(tree.remove(ElementId(0)), Ok(()));

        // Miss!
        assert_eq!(tree.ray_cast(&ray), HitResult {
            element: None,
            distance: 0.0,
        });

        let v1_volume = Aabb::new(TUVec3::new(9, 5, 4), 4).unwrap();
        let v1 = DummyVolume::new(v1_volume);
        let v1_id = tree.insert(v1).unwrap();

        // Aabb intersection
        let aabb = AABB3d::new(Point3::from_value(8.0), Vector3::from_value(8.0));
        assert_eq!(tree.intersect(&aabb), vec![v1_id, v2_id]);

        // Sphere intersection
        let sphere = BoundingSphere::new(Point3::new(15.0, 15.0, 0.0), 5.0);
        assert_eq!(tree.intersect(&sphere), vec![v2_id]);
    }
}
