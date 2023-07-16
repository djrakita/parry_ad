use crate::math::{Isometry};
use crate::query::ClosestPoints;
use crate::shape::Cuboid;
use ad_trait::AD;

/// Distance between two cuboids.
#[inline]
pub fn distance_cuboid_cuboid<T: AD>(pos12: &Isometry<T>, cuboid1: &Cuboid<T>, cuboid2: &Cuboid<T>) -> T {
    match crate::query::details::closest_points_cuboid_cuboid(pos12, cuboid1, cuboid2, T::constant(f64::MAX)) {
        ClosestPoints::WithinMargin(p1, p2) => na::distance(&p1, &(pos12 * p2)),
        _ => T::zero(),
    }
}
