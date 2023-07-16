use crate::math::{Point};
use crate::shape::Ball;
use ad_trait::AD;

/// Intersection test between balls.
#[inline]
pub fn intersection_test_ball_ball<T: AD>(center12: &Point<T>, b1: &Ball<T>, b2: &Ball<T>) -> bool {
    let r1 = b1.radius;
    let r2 = b2.radius;
    let distance_squared = center12.coords.norm_squared();
    let sum_radius = r1 + r2;
    distance_squared <= sum_radius * sum_radius
}
