use crate::shape::Cuboid;
use crate::transformation::utils;
use na::{self, Point2};
use ad_trait::AD;

impl<T: AD> Cuboid<T> {
    /// Discretize the boundary of this cuboid as a polygonal line.
    pub fn to_polyline(&self) -> Vec<Point2<T>> {
        utils::scaled(unit_rectangle(), self.half_extents * T::constant(2.0))
    }
}

/// The contour of a unit cuboid lying on the x-y plane.
fn unit_rectangle<T: AD>() -> Vec<Point2<T>> {
    vec![
        Point2::new(-T::constant(0.5), -T::constant(0.5)),
        Point2::new(T::constant(0.5), -T::constant(0.5)),
        Point2::new(T::constant(0.5), T::constant(0.5)),
        Point2::new(-T::constant(0.5), T::constant(0.5)),
    ]
}
