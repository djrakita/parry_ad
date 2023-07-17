use crate::math::{Isometry, Point};
use crate::query::PointQuery;
use crate::shape::Ball;
use ad_trait::AD;

/// Intersection test between a ball and a shape implementing the `PointQuery` trait.
pub fn intersection_test_ball_point_query<P: ?Sized + PointQuery<T>, T: AD>(
    pos12: &Isometry<T>,
    ball1: &Ball<T>,
    point_query2: &P,
) -> bool {
    intersection_test_point_query_ball(&pos12.inverse(), point_query2, ball1)
}

/// Intersection test between a shape implementing the `PointQuery` trait and a ball.
pub fn intersection_test_point_query_ball<P: ?Sized + PointQuery<T>, T: AD>(
    pos12: &Isometry<T>,
    point_query1: &P,
    ball2: &Ball<T>,
) -> bool {
    let local_p2_1 = Point::from(pos12.translation.vector);
    let proj = point_query1.project_local_point(&local_p2_1, true);
    proj.is_inside || (local_p2_1 - proj.point).norm_squared() <= ball2.radius * ball2.radius
}
