use crate::math::{Isometry};
use crate::query::ClosestPoints;
use crate::shape::{Ball, Shape};
use ad_trait::AD;

/// ClosestPoints between a ball and a convex polyhedron.
///
/// This function panics if the input shape does not implement
/// both the ConvexPolyhedron and PointQuery traits.
#[inline]
pub fn closest_points_ball_convex_polyhedron<T: AD>(
    pos12: &Isometry<T>,
    ball1: &Ball<T>,
    shape2: &(impl Shape<T> + ?Sized),
    prediction: T,
) -> ClosestPoints<T> {
    match crate::query::details::contact_ball_convex_polyhedron(pos12, ball1, shape2, prediction) {
        Some(contact) => {
            if contact.dist <= T::zero() {
                ClosestPoints::Intersecting
            } else {
                ClosestPoints::WithinMargin(contact.point1, contact.point2)
            }
        }
        None => ClosestPoints::Disjoint,
    }
}

/// ClosestPoints between a convex polyhedron and a ball.
///
/// This function panics if the input shape does not implement
/// both the ConvexPolyhedron and PointQuery traits.
#[inline]
pub fn closest_points_convex_polyhedron_ball<T: AD>(
    pos12: &Isometry<T>,
    shape1: &(impl Shape<T> + ?Sized),
    ball2: &Ball<T>,
    prediction: T,
) -> ClosestPoints<T> {
    match crate::query::details::contact_convex_polyhedron_ball(pos12, shape1, ball2, prediction) {
        Some(contact) => {
            if contact.dist <= T::zero() {
                ClosestPoints::Intersecting
            } else {
                ClosestPoints::WithinMargin(contact.point1, contact.point2)
            }
        }
        None => ClosestPoints::Disjoint,
    }
}
