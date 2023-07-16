use crate::bounding_volume::Aabb;
use crate::math::{Isometry};
use crate::query::sat;
use crate::shape::{Cuboid, Triangle};
use ad_trait::AD;

/// Tests if a triangle intersects an Aabb.
pub fn intersection_test_aabb_triangle<T: AD>(aabb1: &Aabb<T>, triangle2: &Triangle<T>) -> bool {
    let cuboid1 = Cuboid::new(aabb1.half_extents());
    let pos12 = Isometry::from_parts((-aabb1.center().coords).into(), na::one());
    intersection_test_cuboid_triangle(&pos12, &cuboid1, triangle2)
}

/// Tests if a triangle intersects a cuboid.
#[inline]
pub fn intersection_test_triangle_cuboid<T: AD>(
    pos12: &Isometry<T>,
    triangle1: &Triangle<T>,
    cuboid2: &Cuboid<T>,
) -> bool {
    intersection_test_cuboid_triangle(&pos12.inverse(), cuboid2, triangle1)
}

/// Tests if a triangle intersects an cuboid.
#[inline]
pub fn intersection_test_cuboid_triangle<T: AD>(
    pos12: &Isometry<T>,
    cube1: &Cuboid<T>,
    triangle2: &Triangle<T>,
) -> bool {
    let sep1 =
        sat::cuboid_support_map_find_local_separating_normal_oneway(cube1, triangle2, &pos12).0;
    if sep1 > T::zero() {
        return false;
    }

    let pos21 = pos12.inverse();
    let sep2 = sat::triangle_cuboid_find_local_separating_normal_oneway(triangle2, cube1, &pos21).0;
    if sep2 > T::zero() {
        return false;
    }

    #[cfg(feature = "dim2")]
    return true; // This case does not exist in 2D.
    #[cfg(feature = "dim3")]
    {
        let sep3 =
            sat::cuboid_triangle_find_local_separating_edge_twoway(cube1, triangle2, &pos12).0;
        sep3 <= T::zero()
    }
}
