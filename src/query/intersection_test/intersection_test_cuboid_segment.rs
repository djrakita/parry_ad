use crate::bounding_volume::Aabb;
use crate::math::{Isometry};
use crate::query::sat;
use crate::shape::{Cuboid, Segment};
use ad_trait::AD;

/// Test if a segment intersects an Aabb.
pub fn intersection_test_aabb_segment<T: AD>(aabb1: &Aabb<T>, segment2: &Segment<T>) -> bool {
    let cuboid1 = Cuboid::new(aabb1.half_extents());
    let pos12 = Isometry::from_parts((-aabb1.center().coords).into(), na::one());
    intersection_test_cuboid_segment(&pos12, &cuboid1, segment2)
}

/// Test if a segment intersects a cuboid.
#[inline]
pub fn intersection_test_segment_cuboid<T: AD>(
    pos12: &Isometry<T>,
    segment1: &Segment<T>,
    cuboid2: &Cuboid<T>,
) -> bool {
    intersection_test_cuboid_segment(&pos12.inverse(), cuboid2, segment1)
}

/// Test if a segment intersects a cuboid.
#[inline]
pub fn intersection_test_cuboid_segment<T: AD>(
    pos12: &Isometry<T>,
    cube1: &Cuboid<T>,
    segment2: &Segment<T>,
) -> bool {
    let sep1 =
        sat::cuboid_support_map_find_local_separating_normal_oneway(cube1, segment2, &pos12).0;
    if sep1 > T::zero() {
        return false;
    }

    // This case does not exist in 3D.
    #[cfg(feature = "dim2")]
    {
        let sep2 = sat::segment_cuboid_find_local_separating_normal_oneway(
            segment2,
            cube1,
            &pos12.inverse(),
        )
        .0;
        if sep2 > T::zero() {
            return false;
        }
    }

    #[cfg(feature = "dim2")]
    return true; // This case does not exist in 2D.
    #[cfg(feature = "dim3")]
    {
        let sep3 = sat::cuboid_segment_find_local_separating_edge_twoway(cube1, segment2, &pos12).0;
        sep3 <= T::zero()
    }
}
