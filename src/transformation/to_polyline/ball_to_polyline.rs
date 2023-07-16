use crate::shape::Ball;
use crate::transformation::utils;
use na::{self, Point2, RealField};
use ad_trait::AD;

impl<T: AD> Ball<T> {
    /// Discretize the boundary of this ball as a polygonal line.
    pub fn to_polyline(&self, nsubdivs: u32) -> Vec<Point2<T>> {
        let diameter = self.radius * T::constant(2.0);
        let two_pi = T::two_pi();
        let dtheta = two_pi / (nsubdivs as T);

        let mut pts = Vec::with_capacity(nsubdivs as usize);
        utils::push_xy_arc(diameter / T::constant(2.0), nsubdivs, dtheta, &mut pts);

        pts
    }
}
