use crate::math::{Isometry, Point, Vector};
use crate::query::Contact;
use crate::shape::Ball;
use na::{self, ComplexField, Unit};
use ad_trait::AD;

/// Contact between balls.
#[inline]
pub fn contact_ball_ball<T: AD>(
    pos12: &Isometry<T>,
    b1: &Ball<T>,
    b2: &Ball<T>,
    prediction: T,
) -> Option<Contact<T>> {
    let r1 = b1.radius;
    let r2 = b2.radius;
    let center2_1 = pos12.translation.vector;
    let distance_squared = center2_1.norm_squared();
    let sum_radius = r1 + r2;
    let sum_radius_with_error = sum_radius + prediction;

    if distance_squared < sum_radius_with_error * sum_radius_with_error {
        let normal1 = if !distance_squared.is_zero() {
            Unit::new_normalize(center2_1)
        } else {
            Vector::x_axis()
        };
        let normal2 = -pos12.inverse_transform_unit_vector(&normal1);
        let point1 = Point::from(*normal1 * r1);
        let point2 = Point::from(*normal2 * r2);

        Some(Contact::new(
            point1,
            point2,
            normal1,
            normal2,
            ComplexField::sqrt(distance_squared) - sum_radius,
        ))
    } else {
        None
    }
}
