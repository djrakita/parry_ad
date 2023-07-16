use crate::math::{Isometry, Point};
use crate::shape::{Ball, Shape};
use ad_trait::AD;

/// Distance between a ball and a convex polyhedron.
///
/// This function panics if the input shape does not implement
/// both the ConvexPolyhedron and PointQuery traits.
#[inline]
pub fn distance_ball_convex_polyhedron<T: AD>(
    pos12: &Isometry<T>,
    ball1: &Ball<T>,
    shape2: &(impl Shape<T> + ?Sized),
) -> T {
    distance_convex_polyhedron_ball(&pos12.inverse(), shape2, ball1)
}

/// Distance between a convex polyhedron and a ball.
///
/// This function panics if the input shape does not implement
/// both the ConvexPolyhedron and PointQuery traits.
#[inline]
pub fn distance_convex_polyhedron_ball<T: AD>(
    pos12: &Isometry<T>,
    shape1: &(impl Shape<T> + ?Sized),
    ball2: &Ball<T>,
) -> T {
    let center2_1 = Point::from(pos12.translation.vector);
    let proj = shape1.project_local_point(&center2_1, true);
    (na::distance(&proj.point, &center2_1) - ball2.radius).max(T::zero())
}
