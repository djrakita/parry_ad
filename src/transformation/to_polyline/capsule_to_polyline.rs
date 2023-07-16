use crate::shape::Capsule;
use crate::transformation::utils;
use na::{self, Point2, RealField, Vector2};
use ad_trait::AD;

impl<T: AD> Capsule<T> {
    /// Discretize the boundary of this capsule as a polygonal line.
    pub fn to_polyline(&self, nsubdiv: u32) -> Vec<Point2<T>> {
        let pi = T::constant(f64::pi());
        let dtheta = pi / T::constant(nsubdiv as f64);

        let mut points: Vec<Point2<T>> = Vec::with_capacity(nsubdiv as usize);

        utils::push_xy_arc(self.radius, nsubdiv, dtheta, &mut points);

        let npoints = points.len();

        for i in 0..npoints {
            let new_point = points[i] + Vector2::new(na::zero(), self.half_height());

            points.push(-new_point);
            points[i] = new_point;
        }

        utils::transformed(points, self.canonical_transform())
    }
}
