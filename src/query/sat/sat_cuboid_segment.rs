use crate::math::{Isometry, Vector};
use crate::query::sat;
use crate::shape::{Cuboid, Segment};
use ad_trait::AD;

/// Finds the best separating edge between a cuboid and a segment.
///
/// All combinations of edges from the cuboid and the segment are taken into
/// account.
#[cfg(feature = "dim3")]
pub fn cuboid_segment_find_local_separating_edge_twoway<T: AD>(
    cube1: &Cuboid<T>,
    segment2: &Segment<T>,
    pos12: &Isometry<T>,
) -> (T, Vector<T>) {
    let x2 = pos12 * (segment2.b - segment2.a);

    let axes = [
        // Vector::{x, y ,z}().cross(y2)
        Vector::new(T::zero(), -x2.z, x2.y),
        Vector::new(x2.z, T::zero(), -x2.x),
        Vector::new(-x2.y, x2.x, T::zero()),
    ];

    sat::cuboid_support_map_find_local_separating_edge_twoway(cube1, segment2, &axes, pos12)
}

/// Finds the best separating normal between a cuboid and a segment.
///
/// Only the normals of `segment1` are tested.
#[cfg(feature = "dim2")]
pub fn segment_cuboid_find_local_separating_normal_oneway<T: AD>(
    segment1: &Segment<T>,
    shape2: &Cuboid<T>,
    pos12: &Isometry<T>,
) -> (T, Vector<T>) {
    sat::point_cuboid_find_local_separating_normal_oneway(
        segment1.a,
        segment1.normal(),
        shape2,
        pos12,
    )
}
