use crate::math::{Isometry, Point};
use crate::query::ClosestPoints;
use crate::shape::Ball;
use ad_trait::AD;

/// Closest points between balls.
///
/// Each returned point is expressed on the local-space of the corresponding shape.
#[inline]
pub fn closest_points_ball_ball<T: AD>(
    pos12: &Isometry<T>,
    b1: &Ball<T>,
    b2: &Ball<T>,
    margin: T,
) -> ClosestPoints<T> {
    assert!(
        margin >= T::zero(),
        "The proximity margin must be positive or null."
    );

    let r1 = b1.radius;
    let r2 = b2.radius;
    let delta_pos = pos12.translation.vector;
    let distance = delta_pos.norm();
    let sum_radius = r1 + r2;

    if distance - margin <= sum_radius {
        if distance <= sum_radius {
            ClosestPoints::Intersecting
        } else {
            let normal = delta_pos.normalize();
            let p1 = Point::from(normal * r1);
            let p2 = Point::from(pos12.inverse_transform_vector(&normal) * -r2);
            ClosestPoints::WithinMargin(p1, p2)
        }
    } else {
        ClosestPoints::Disjoint
    }
}
