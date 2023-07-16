use crate::math::{Point};
use crate::shape::Ball;
use na::{self, ComplexField};
use ad_trait::AD;

/// Distance between balls.
#[inline]
pub fn distance_ball_ball<T: AD>(b1: &Ball<T>, center2: &Point<T>, b2: &Ball<T>) -> T {
    let r1 = b1.radius;
    let r2 = b2.radius;
    let distance_squared = center2.coords.norm_squared();
    let sum_radius = r1 + r2;

    if distance_squared <= sum_radius * sum_radius {
        T::zero()
    } else {
        ComplexField::sqrt(distance_squared) - sum_radius
    }
}
