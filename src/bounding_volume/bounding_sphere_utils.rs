use crate::math::{Point};
use crate::utils;
use na::{self, ComplexField};
use ad_trait::AD;

/// Computes the bounding sphere of a set of point, given its center.
// FIXME: return a bounding sphere?
#[inline]
pub fn point_cloud_bounding_sphere_with_center<T: AD>(
    pts: &[Point<T>],
    center: Point<T>,
) -> (Point<T>, T) {
    let mut sqradius = T::zero();

    for pt in pts.iter() {
        let distance_squared = na::distance_squared(pt, &center);

        if distance_squared > sqradius {
            sqradius = distance_squared
        }
    }

    (center, ComplexField::sqrt(sqradius))
}

/// Computes a bounding sphere of the specified set of point.
// FIXME: return a bounding sphere?
#[inline]
pub fn point_cloud_bounding_sphere<T: AD>(pts: &[Point<T>]) -> (Point<T>, T) {
    point_cloud_bounding_sphere_with_center(pts, utils::center(pts))
}
